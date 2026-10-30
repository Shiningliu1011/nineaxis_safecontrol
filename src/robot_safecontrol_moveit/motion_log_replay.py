import argparse
import json
from bisect import bisect_left, bisect_right
from collections import Counter
from dataclasses import asdict
from hashlib import sha256
from math import isclose, isfinite
from pathlib import Path
from statistics import linear_regression

import numpy as np
from construct import ConstructError

from .drempower_can import (
    CMD_POSITION_ANGLE_MODE1, CMD_PROPERTY_READ, decode_feedback, encode_trapezoid_position,
)
from .drempower_joint_control import JointFeedback
from .drempower_serial import SERIAL_CAN_FRAME
from .single_joint_execution import (
    ArrivalWindow,
    ExecutionLimits,
    MotionRejected,
    ObservedFeedback,
    SingleJointMonitor,
    VelocityWindow,
)


def replay_motion_log(source: bytes, frame_source: bytes) -> dict:
    events = [json.loads(line) for line in source.decode("utf-8").splitlines()]
    frames = json.loads(frame_source)
    start = events[0]
    limits = ExecutionLimits(**start["limits"])
    calibration = start["calibration"]
    feedback_frames, invalid_frames = [], []
    for index, frame in enumerate(frames):
        if frame["direction"] != "rx":
            continue
        try:
            parsed = SERIAL_CAN_FRAME.parse(bytes.fromhex(frame["raw_hex"]))
        except ConstructError as error:
            invalid_frames.append({"index": index, "raw_hex": frame["raw_hex"], "reason": str(error)})
            continue
        if parsed.frame_id & 0x1F in (1, 3, 5, 7):
            feedback_frames.append((frame["monotonic_s"], decode_feedback(parsed.frame_id, parsed.data)))
    stamps = [frame[0] for frame in feedback_frames]
    if stamps != sorted(stamps):
        raise ValueError("原始反馈记录的时间顺序无效")
    monitor, initial, rejection = None, None, None
    matched = 0
    window = VelocityWindow(limits.velocity_window_s, limits.stationary_window_s)
    arrival = None
    hold_failures: Counter[str] = Counter()
    hold_samples = 0
    for index, event in enumerate(events):
        if event["event"] == "command_requested":
            if initial is None or monitor is not None:
                raise ValueError("运动日志的初始反馈或命令顺序无效")
            target = calibration["sign"] * (event["arguments"]["angle"] - calibration["zero_offset_deg"])
            active_limits = limits.for_motion(target - initial.state.angle_deg)
            monitor = SingleJointMonitor(start["node_id"], active_limits)
            monitor.start_absolute(initial, target, event["monotonic_s"])
            continue
        if "sample" not in event:
            continue
        value = dict(event["sample"])
        sample = ObservedFeedback(**{**value, "state": JointFeedback(**value["state"])})
        raw_index = bisect_right(stamps, sample.received_s) - 1
        if raw_index < 0:
            raise ValueError(f"事件 {index} 缺少原始接收帧")
        stamp, raw = feedback_frames[raw_index]
        decoded = JointFeedback(calibration["sign"] * (raw.pos_deg - calibration["zero_offset_deg"]),
                                calibration["sign"] * raw.vel_rpm)
        if (raw.node_id != sample.node_id or not sample.request_s <= stamp <= sample.received_s
                or not isclose(decoded.angle_deg, sample.state.angle_deg, abs_tol=1e-9, rel_tol=0)
                or not isclose(decoded.speed_rpm, sample.state.speed_rpm, abs_tol=1e-9, rel_tol=0)
                or sample.traj_done != raw.traj_done or sample.axis_error != raw.axis_error):
            raise ValueError(f"事件 {index} 与原始收发帧不符")
        # 从原始反馈重算速度窗口，核对日志中的派生值。
        recomputed = window.add(ObservedFeedback(sample.node_id, decoded, sample.request_s,
                                                sample.received_s, raw.traj_done, raw.axis_error))
        for name in ("mean_speed_rpm", "position_speed_rpm", "window_duration_s", "stationary_mean_speed_rpm",
                     "stationary_position_span_deg", "stationary_window_duration_s"):
            if name not in value:
                continue
            actual, recorded = getattr(recomputed, name), getattr(sample, name)
            if (actual is None) != (recorded is None) or (
                actual is not None and not isclose(actual, recorded, abs_tol=1e-8, rel_tol=1e-8)
            ):
                raise ValueError(f"事件 {index} 的 {name} 与原始反馈重算值不符")
        sample = recomputed
        matched += 1
        if event["event"] == "feedback":
            if monitor is None:
                initial = sample
            elif rejection is None:
                try:
                    monitor.observe(sample, event["monotonic_s"])
                except MotionRejected as error:
                    rejection = {"event_index": index, "reason": str(error), "sample": asdict(sample)}
        elif event["event"] == "hold_requested":
            arrival = ArrivalWindow(sample.state.angle_deg, limits)
        elif event["event"] == "hold_feedback":
            if arrival is None:
                raise ValueError("停车观察缺少保持请求")
            hold_samples += 1
            arrival.observe(sample, event["monotonic_s"])
            hold_failures.update(name for name, passed in arrival.last_conditions.items() if not passed)
    return {
        "validation_scope": "offline_original_device_log_replay",
        "feedback_source": "recorded_device_feedback", "hardware_qualified": False,
        "device_sample_time_known": False, "physical_stop_independently_verified": False,
        "source_sha256": sha256(source).hexdigest(), "source_bytes": len(source),
        "frames_sha256": sha256(frame_source).hexdigest(), "joint": start["joint"],
        "events": len(events), "frames": len(frames), "matched_samples": matched, "invalid_rx_frames": invalid_frames,
        "recorded_limits": start["limits"], "recorded_terminal": events[-1],
        "current_replay_rejection": rejection,
        "current_replay_phase": monitor.phase if monitor is not None else None,
        "hold": {"samples": hold_samples, "condition_failures": dict(hold_failures),
                 "max_settled_s": arrival.max_settled_s if arrival else None,
                 "last_conditions": arrival.last_conditions if arrival else None},
    }


def replay_encoder_probe(source: bytes, frame_source: bytes, capture_source: bytes) -> dict:
    from .joint_table_readonly import replay_encoder_samples, replay_readonly_report

    capture = json.loads(capture_source)
    if (capture["scope"] != "attended_bounded_J1_counter_characterisation"
            or capture["frames_sha256"] != sha256(frame_source).hexdigest()
            or capture["log_sha256"] != sha256(source).hexdigest()):
        raise ValueError("计数实验的范围或原始文件身份不符")
    report = replay_motion_log(source, frame_source)
    if (capture["status"] != "completed" or report["recorded_terminal"]["event"] != "completed"
            or report["current_replay_rejection"] is not None or report["current_replay_phase"] != "completed"):
        raise ValueError("计数实验尚未通过完整运动与终态复算")
    events = [json.loads(line) for line in source.decode("utf-8").splitlines()]
    if report["joint"] != "J1" or events[0]["coordinate_basis"] != "forward_output_shaft_deg":
        raise ValueError("计数实验只支持已有输出轴单位的 J1 记录")
    frames = json.loads(frame_source)
    read_frames, commands, acknowledgements = [], [], []
    pending_read, pending_motion = None, None
    previous_s = -1.
    for frame in frames:
        stamp = frame["monotonic_s"]
        if type(stamp) not in (float, int) or not isfinite(stamp) or not 0 <= stamp or stamp < previous_s:
            raise ValueError("计数实验的原始帧主机时间无效")
        previous_s = stamp
        parsed = SERIAL_CAN_FRAME.parse(bytes.fromhex(frame["raw_hex"]))
        node, command = parsed.frame_id >> 5, parsed.frame_id & 0x1F
        if frame["direction"] == "tx":
            if pending_read is not None:
                raise ValueError("计数实验包含未完成的读取")
            pending_motion = None
            if command == CMD_PROPERTY_READ:
                pending_read = node
                read_frames.append(frame)
            elif command == CMD_POSITION_ANGLE_MODE1 and node == 1:
                commands.append((stamp, parsed.data))
                pending_motion = node
            else:
                raise ValueError("计数实验包含范围之外的运动或参数写入")
        elif frame["direction"] == "rx":
            if pending_read == node:
                read_frames.append(frame)
                pending_read = None
            elif pending_motion == node:
                acknowledgements.append(decode_feedback(parsed.frame_id, parsed.data))
                pending_motion = None
            else:
                raise ValueError("计数实验的回复缺少对应请求")
        else:
            raise ValueError("计数实验原始帧方向无效")
    requested = [event for event in events if event["event"] == "command_requested"]
    returned = [event for event in events if event["event"] == "command_call_returned"]
    if len(commands) != 1 or len(requested) != 1 or len(returned) != 1 or pending_read is not None:
        raise ValueError("计数实验需要完整的单次 J1 位置命令")
    arguments = requested[0]["arguments"]
    expected = encode_trapezoid_position(arguments["id_num"], arguments["angle"],
                                         speed=arguments["speed"], accel=arguments["param"])
    if (arguments["id_num"] != 1 or arguments["mode"] != 1 or commands[0][1] != expected
            or not requested[0]["monotonic_s"] <= commands[0][0] <= returned[0]["monotonic_s"]):
        raise ValueError("计数实验的位置命令与原始帧不符")
    # 仅提取原始读取事务供现有解析器核对；完整运动帧身份保存在外层报告。
    reads = replay_readonly_report(json.dumps({"frames": read_frames}).encode())
    reply_enabled = reads["initial_nodes"]["1"]["auto_reply"]
    if reply_enabled not in (0, 1) or len(acknowledgements) != reply_enabled:
        raise ValueError("计数实验的自动回复数量与设备配置不符")
    if any(value.axis_error for value in acknowledgements):
        raise ValueError("计数实验的命令回复报告轴错误")
    samples = replay_encoder_samples(capture, reads)
    raw_rows = reads["property_observations"]
    requests = [row["request_s"] for row in raw_rows]
    for row in capture["property_observations"]:
        index = bisect_left(requests, row["request_s"])
        if index >= len(raw_rows):
            raise ValueError("计数实验属性读取缺少原始请求")
        raw = raw_rows[index]
        if (any(row[name] != raw[name] for name in ("node_id", "name", "address", "kind", "value"))
                or not row["request_s"] <= raw["request_s"] <= raw["received_s"] <= row["received_s"]):
            raise ValueError("计数实验属性读取与原始帧不符")
    if len(samples) != report["matched_samples"] or not samples:
        raise ValueError("计数实验的连续采集与反馈覆盖不符")
    feedback_counts = Counter(row["node_id"] for row in reads["feedback_observations"])
    other_counts = Counter(event["observation"]["node_id"] for event in events
                           if event["event"] == "other_joint_feedback")
    if (set(feedback_counts) != set(range(1, 10))
            or any(feedback_counts[node] != len(samples) for node in range(1, 10))
            or any(other_counts[node] != len(samples) for node in range(2, 10))):
        raise ValueError("计数实验缺少九轴连续反馈覆盖")
    values = [row["values"] for row in samples]
    rotor = [row["rotor_pos_turns"] for row in values]
    turn = [row["encoder_turn_abs"] for row in values]
    output = [row["output_pos_deg"] for row in values]
    cpr = values[0]["cpr"]
    absolute = np.array([row["encoder_pos_abs"] for row in values])
    if cpr <= 0 or np.any(absolute < 0) or np.any(absolute >= cpr):
        raise ValueError("本次计数实验的单圈计数没有处于 0–cpr 范围")
    if np.any(np.abs(np.diff(rotor)) >= .5):
        raise ValueError("计数采样间隔不能唯一展开单圈计数")
    unwrapped = np.unwrap(absolute, period=cpr)
    turn_fit = linear_regression(rotor, turn)
    absolute_fit = linear_regression(rotor, unwrapped)
    rotor_fit = linear_regression(output, rotor)
    return {**report, "validation_scope": "offline_original_J1_encoder_probe_replay",
            "capture_sha256": sha256(capture_source).hexdigest(),
            "motion_commands_sent": 1, "parameter_writes_sent": 0, "other_joint_motion_commands_sent": 0,
            "capture_status": capture["status"], "encoder_samples": len(samples),
            "feedback_counts_by_node": dict(feedback_counts),
            "configuration_unchanged": all(row["configuration_matches"] is True for row in samples),
            "initial_encoder": values[0], "final_encoder": values[-1],
            "counter_relationship": {
                "method": "linear_regression_of_sequential_host_reads",
                "turn_abs_per_reported_rotor_turn": turn_fit.slope,
                "turn_abs_intercept": turn_fit.intercept,
                "unwrapped_pos_abs_per_reported_rotor_turn": absolute_fit.slope,
                "pos_abs_unwrap_period_counts": cpr,
                "reported_rotor_turns_per_output_degree": rotor_fit.slope,
                "independent_mechanical_position_measured": False,
            }}


def main():
    parser = argparse.ArgumentParser(description="完整真实关节日志与原始收发帧的离线核查")
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--frames", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    captures = parser.add_mutually_exclusive_group()
    captures.add_argument("--encoder-capture", type=Path, help="同时核对 J1 连续计数、配置和实际运动命令")
    captures.add_argument("--tracking-capture", type=Path, help="核对内含原始帧的九轴连续参考，包含中止与未入日志反馈")
    parser.add_argument("--root", type=Path, help="连续参考回放所用设备坐标配置的仓库目录")
    args = parser.parse_args()
    if args.tracking_capture and (args.frames is not None or args.root is None):
        parser.error("--tracking-capture 需要 --root，并从采集文件读取原始帧")
    if not args.tracking_capture and args.frames is None:
        parser.error("单关节及计数回放需要 --frames")
    if args.output.exists():
        raise FileExistsError(args.output)
    if args.tracking_capture:
        from .tracking_log_replay import replay_tracking_capture

        report = replay_tracking_capture(args.log.read_bytes(), args.tracking_capture.read_bytes(), args.root)
    else:
        report = (replay_encoder_probe(args.log.read_bytes(), args.frames.read_bytes(), args.encoder_capture.read_bytes())
                  if args.encoder_capture else replay_motion_log(args.log.read_bytes(), args.frames.read_bytes()))
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(f"离线核查 {report['matched_samples']} 条原始反馈：{args.output}")


if __name__ == "__main__":
    main()
