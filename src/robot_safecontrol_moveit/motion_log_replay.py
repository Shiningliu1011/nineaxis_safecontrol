import argparse
import json
from bisect import bisect_right
from collections import Counter
from dataclasses import asdict
from hashlib import sha256
from math import isclose
from pathlib import Path

from construct import ConstructError

from .drempower_can import decode_feedback
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
    window = VelocityWindow(limits.velocity_window_s)
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
        for name in ("mean_speed_rpm", "position_speed_rpm", "window_duration_s"):
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
        "hold": {"samples": hold_samples, "condition_failures": dict(hold_failures),
                 "max_settled_s": arrival.max_settled_s if arrival else None,
                 "last_conditions": arrival.last_conditions if arrival else None},
    }


def main():
    parser = argparse.ArgumentParser(description="完整真实关节日志与原始收发帧的离线核查")
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--frames", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    report = replay_motion_log(args.log.read_bytes(), args.frames.read_bytes())
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(f"离线核查 {report['matched_samples']} 条原始反馈：{args.output}")


if __name__ == "__main__":
    main()
