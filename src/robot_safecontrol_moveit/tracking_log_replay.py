from bisect import bisect_left
from collections import Counter
from dataclasses import asdict
from hashlib import sha256
import json
from math import isfinite
from pathlib import Path

from construct import Float32l, Int16sl, Struct, Terminated
import numpy as np

from .drempower_can import (
    CMD_POSITION_ANGLE_MODE1, CMD_PRESET_ANGLE, CMD_PROPERTY_READ, CMD_SYSTEM,
    SYSTEM_ORDER_START_ANGLE_TRACKING, can_node_id, decode_feedback, encode_system, encode_trapezoid_position,
)
from .drempower_joint_control import JointFeedback
from .drempower_serial import SERIAL_CAN_FRAME
from .joint_table_execution import JointTableControl
from .joint_table_readonly import replay_readonly_report
from .robot_spec import DEFAULT_JOINT_NAMES
from .single_joint_execution import ObservedFeedback, VelocityWindow


POSITION_DATA = Struct("angle" / Float32l, "second" / Int16sl, "third" / Int16sl, Terminated)


def _raw_tracking(capture: dict, control: JointTableControl) -> dict:
    reads, feedback, positions, broadcasts, motion = [], [], [], [], []
    pending_read, pending_motion = None, None
    active, presets = {}, []
    acknowledgements: Counter[int] = Counter()
    joints_by_node = {control._node(joint): joint for joint in DEFAULT_JOINT_NAMES}
    latest_feedback_s: dict[int, float] = {}
    previous_s = -1.
    counts: Counter[str] = Counter()
    for index, frame in enumerate(capture["frames"]):
        stamp = frame["monotonic_s"]
        if type(stamp) not in (float, int) or not isfinite(stamp) or stamp < 0 or stamp < previous_s:
            raise ValueError("连续参考原始帧主机时间无效")
        previous_s = stamp
        parsed = SERIAL_CAN_FRAME.parse(bytes.fromhex(frame["raw_hex"]))
        node, command = can_node_id(parsed.frame_id)
        if parsed.frame_id > 0x7ff or node > 9:
            raise ValueError("连续参考原始帧节点无效")
        if frame["direction"] == "tx":
            if pending_read is not None:
                raise ValueError("连续参考存在未完成的属性读取")
            pending_motion = None
            counts[f"0x{command:02x}"] += 1
            if command == CMD_PROPERTY_READ and 1 <= node <= 9:
                pending_read = node
                reads.append(frame)
            elif command in (CMD_PRESET_ANGLE, CMD_POSITION_ANGLE_MODE1) and 1 <= node <= 9:
                data = POSITION_DATA.parse(parsed.data)
                if not isfinite(data.angle) or data.second < 0 or data.third <= 0:
                    raise ValueError("连续参考的目标或模式参数无效")
                item = {"node_id": node, "data": parsed.data, "decoded": data,
                        "monotonic_s": stamp, "raw_frame_index": index}
                motion.append(item)
                pending_motion = (node, index)
                if command == CMD_PRESET_ANGLE:
                    if data.second != 0 or data.third > 10000:
                        raise ValueError("连续参考预设需要零第二字段及有效滤波带宽")
                    presets.append(item)
                else:
                    positions.append(item)
                    active[node] = (data.angle, stamp, "position_request")
            elif command == CMD_SYSTEM and node == 0 and parsed.data == encode_system(0, SYSTEM_ORDER_START_ANGLE_TRACKING):
                if [row["node_id"] for row in presets] != list(range(1, 10)):
                    raise ValueError("连续广播前缺少完整顺序九轴预设")
                if set(latest_feedback_s) != set(joints_by_node):
                    raise ValueError("连续广播前没有完整九轴原始反馈")
                broadcasts.append({"monotonic_s": stamp, "presets": presets,
                                   "maximum_feedback_age_s": max(stamp - value for value in latest_feedback_s.values())})
                active = {row["node_id"]: (row["decoded"].angle, stamp, "host_broadcast") for row in presets}
                presets = []
            else:
                raise ValueError("连续参考原始记录含范围外运动或参数写入")
        elif frame["direction"] == "rx":
            if pending_read == node:
                reply_source = "property_read"
                reads.append(frame)
                pending_read = None
            elif pending_motion is not None and pending_motion[0] == node:
                if command not in (1, 3, 5, 7):
                    raise ValueError("连续参考命令回复不是原始快速反馈")
                acknowledgements[pending_motion[1]] += 1
                pending_motion = None
                reply_source = "motion_acknowledgement"
            else:
                raise ValueError("连续参考回复缺少对应原始请求")
            if command in (1, 3, 5, 7):
                raw = decode_feedback(parsed.frame_id, parsed.data)
                joint = joints_by_node[node]
                latest_feedback_s[node] = stamp
                state = control.decode_state(joint, raw.pos_deg, raw.vel_rpm)
                target, target_s, target_kind = active.get(node, (None, None, None))
                reference = (None if target is None else control.project_position(
                    joint, control.decode_state(joint, target, 0.).angle_deg))
                feedback.append({"node_id": node, "joint": joint, "received_s": stamp, "state": asdict(state),
                                 "position": control.project_position(joint, state.angle_deg),
                                 "device_position_deg": raw.pos_deg, "device_speed_rpm": raw.vel_rpm,
                                 "device_torque_nm": raw.torque_nm, "reply_source": reply_source,
                                 "axis_error": raw.axis_error, "traj_done": raw.traj_done,
                                 "raw_frame_index": index, "raw_hex": frame["raw_hex"],
                                 "reference": reference, "reference_time_s": target_s, "reference_source": target_kind})
        else:
            raise ValueError("连续参考原始帧方向无效")
    if pending_read is not None:
        raise ValueError("连续参考原始读取不完整")
    if presets and (capture["status"] != "aborted"
                    or [row["node_id"] for row in presets] != list(range(1, len(presets) + 1))):
        raise ValueError("连续参考原始预设批次不完整")
    readonly = replay_readonly_report(json.dumps({"frames": reads}).encode())
    for row in motion:
        expected = readonly["initial_nodes"].get(str(row["node_id"]), {}).get("auto_reply")
        if expected not in (0, 1) or acknowledgements[row["raw_frame_index"]] != expected:
            raise ValueError("连续参考自动回复数量与原始设置不符")
    return {"feedback": feedback, "positions": positions, "broadcasts": broadcasts,
            "unbroadcast_presets": [{"node_id": row["node_id"], "monotonic_s": row["monotonic_s"],
                                     "raw_frame_index": row["raw_frame_index"], "target_device_deg": row["decoded"].angle}
                                    for row in presets],
            "tx_commands": dict(counts), "readonly": readonly}


def _response_dynamics(rows: list[dict]) -> dict:
    result = {}
    for joint in DEFAULT_JOINT_NAMES:
        moving = [row for row in rows if row["joint"] == joint and row["reference_source"] == "host_broadcast"]
        unit = "mm" if joint == "J1" else "deg"
        largest = None
        for before, after in zip(moving, moving[1:]):
            delta = after["position"] - before["position"]
            if largest is not None and abs(delta) <= abs(largest["position_change"]):
                continue
            interval = after["received_s"] - before["received_s"]
            largest = {
                "before": {key: value for key, value in before.items() if key != "state"},
                "after": {key: value for key, value in after.items() if key != "state"},
                "position_change": delta,
                "reference_change": after["reference"] - before["reference"],
                "host_receive_interval_s": interval,
                "position_rate_per_host_second": delta / interval if interval > 0 else None,
            }
        peak = max(moving, key=lambda row: abs(row["position"] - row["reference"])) if moving else None
        result[joint] = {
            "unit": unit, "samples_during_host_broadcast": len(moving),
            "largest_adjacent_position_change": largest,
            "maximum_error_sample": ({**{key: value for key, value in peak.items() if key != "state"},
                                      "tracking_error": peak["position"] - peak["reference"]} if peak else None),
            "maximum_absolute_device_torque_nm": max(abs(row["device_torque_nm"]) for row in moving) if moving else None,
        }
    return {"time_basis": "host_receive_intervals", "device_sample_time_known": False,
            "independent_motion_measurement": False, "joints": result}


def _match_feedback(events: list[dict], rows: list[dict], control: JointTableControl) -> set[int]:
    by_node = {node: [row for row in rows if row["node_id"] == node] for node in range(1, 10)}
    times = {node: [row["received_s"] for row in values] for node, values in by_node.items()}
    windows = {joint: VelocityWindow(1., 5.) for joint in DEFAULT_JOINT_NAMES}
    matched: set[int] = set()
    for event in events:
        if event["event"] != "feedback":
            continue
        if set(event["samples"]) != set(DEFAULT_JOINT_NAMES):
            raise ValueError("连续参考日志反馈缺少完整九轴")
        for joint in DEFAULT_JOINT_NAMES:
            node = control._node(joint)
            value = event["samples"][joint]
            index = bisect_left(times[node], value["request_s"])
            if index >= len(by_node[node]):
                raise ValueError("连续参考日志反馈没有原始回复")
            raw = by_node[node][index]
            if (not value["request_s"] <= raw["received_s"] <= value["received_s"]
                    or raw["raw_frame_index"] in matched):
                raise ValueError("连续参考日志反馈的主机时间或原始帧使用无效")
            sample = windows[joint].add(ObservedFeedback(
                node, JointFeedback(**raw["state"]), value["request_s"], value["received_s"],
                raw["traj_done"], raw["axis_error"]))
            if asdict(sample) != value:
                raise ValueError("连续参考日志反馈、速度或静止窗口与原始帧不符")
            matched.add(raw["raw_frame_index"])
    if not matched:
        raise ValueError("连续参考日志没有可核对的九轴反馈")
    return matched


def _match_commands(events: list[dict], trace: dict, control: JointTableControl):
    batches, reference = [], None
    position_requests = []
    for event in events:
        kind = event["event"]
        if kind == "tracking_reference":
            reference = event
        elif kind in ("tracking_broadcast", "broadcast_write_returned", "restore_bandwidths"):
            if kind == "tracking_broadcast":
                if reference is None:
                    raise ValueError("连续参考广播缺少完整 SI 目标")
                expected = control.encoder_targets(reference["q_si"])
                if any(abs(expected[node] - event["targets_device_deg"][str(node)]) > 1e-9 for node in range(1, 10)):
                    raise ValueError("连续参考 SI 目标与设备目标不符")
            batches.append(event)
        elif kind in ("low_speed_seed_requested", "terminal_hold_requested", "abort_hold_requested", "hold_requested"):
            position_requests.append(event)
    if len(batches) != len(trace["broadcasts"]) or len(position_requests) != len(trace["positions"]):
        raise ValueError("连续参考或保持的原始命令数量与日志不符")
    for event, raw in zip(batches, trace["broadcasts"]):
        if not event["broadcast_requested_s"] <= raw["monotonic_s"] <= event["broadcast_write_finished_s"]:
            raise ValueError("连续广播的主机发送时间与原始帧不符")
        if set(event["targets_device_deg"]) != set(map(str, range(1, 10))):
            raise ValueError("连续广播日志目标缺少完整九轴")
        for preset in raw["presets"]:
            node, data = str(preset["node_id"]), preset["decoded"]
            if (data.angle != Float32l.parse(Float32l.build(event["targets_device_deg"][node]))
                    or data.third != round(event["bandwidths"][node] * 100)):
                raise ValueError("连续参考的原始目标或滤波带宽与日志不符")
    for event, raw in zip(position_requests, trace["positions"]):
        args = event["arguments"]
        if (args["mode"] != 1 or args["id_num"] != raw["node_id"]
                or raw["data"] != encode_trapezoid_position(
                    args["id_num"], args["angle"], speed=args["speed"], accel=args["param"])
                or event["monotonic_s"] > raw["monotonic_s"]):
            raise ValueError("九轴保持命令与原始位置参数不符")


def replay_tracking_capture(source: bytes, capture_source: bytes, root: Path) -> dict:
    capture = json.loads(capture_source)
    events = [json.loads(line) for line in source.splitlines()]
    if not events or capture["log_sha256"] != sha256(source).hexdigest():
        raise ValueError("连续参考原始日志身份不符")
    if capture["status"] not in ("completed", "aborted") or events[-1]["event"] != capture["status"]:
        raise ValueError("连续参考采集状态与日志终态不符")
    scope = capture["scope"]
    if scope != "attended_nineaxis_small_relative_continuous_roundtrip" and not (
            scope == "attended_small_timed_preset_broadcast_characterisation" and capture.get("protocol_kind") == "tracking"):
        raise ValueError("连续参考原始采集范围无效")
    basis = capture.get("review", capture)
    config_hashes = {name: sha256((root / "config" / name).read_bytes()).hexdigest()
                     for name in ("drempower.yaml", "hardware_joint_zero.yaml", "j1_transmission.yaml")}
    if "configuration_sha256" in basis and basis["configuration_sha256"] != config_hashes:
        raise ValueError("连续参考的设备坐标配置身份不符")
    lower, upper = np.asarray(basis["lower_si"]), np.asarray(basis["upper_si"])
    if (lower.shape != (9,) or upper.shape != (9,) or not np.all(np.isfinite([lower, upper]))
            or np.any(lower >= upper)):
        raise ValueError("连续参考完整检查范围无效")
    control = JointTableControl(root)
    trace = _raw_tracking(capture, control)
    matched = _match_feedback(events, trace["feedback"], control)
    _match_commands(events, trace, control)
    violations, last = [], {}
    errors: dict[str, list[float]] = {joint: [] for joint in DEFAULT_JOINT_NAMES}
    for raw in trace["feedback"]:
        node, position = raw["node_id"], raw["position"]
        joint_index = DEFAULT_JOINT_NAMES.index(raw["joint"])
        unit = "mm" if raw["joint"] == "J1" else "deg"
        si = position / 1000 if raw["joint"] == "J1" else np.deg2rad(position)
        row = {name: value for name, value in raw.items() if name != "state"}
        row.update(unit=unit, included_in_feedback_journal=raw["raw_frame_index"] in matched)
        row["tracking_error"] = None if raw["reference"] is None else position - raw["reference"]
        last[raw["joint"]] = row
        if raw["reference_source"] == "host_broadcast":
            errors[raw["joint"]].append(abs(row["tracking_error"]))
        if si < lower[joint_index] or si > upper[joint_index]:
            violations.append(row)
    ages = [row["maximum_feedback_age_s"] for row in trace["broadcasts"]]
    intervals = np.diff([row["monotonic_s"] for row in trace["broadcasts"]])
    if capture["status"] == "completed" and violations:
        raise ValueError("连续参考完成终态与原始范围违例不符")
    roundtrip = scope == "attended_nineaxis_small_relative_continuous_roundtrip" and capture["status"] == "completed"
    if roundtrip and (not any(row["event"] == "reference_reversal" for row in events)
                      or capture.get("restored_operating_properties") != capture["original_operating_properties"]):
        raise ValueError("九轴往返完成终态缺少换向或模式恢复证据")
    return {
        "validation_scope": "offline_original_tracking_replay", "status": "replayed_completed",
        "capture_status": capture["status"], "recorded_terminal": events[-1], "source_sha256": sha256(source).hexdigest(),
        "capture_sha256": sha256(capture_source).hexdigest(), "configuration_sha256": config_hashes,
        "configuration_declared_in_capture": "configuration_sha256" in basis,
        "feedback_source": "recorded_device_feedback", "matched_samples": len(matched),
        "raw_feedback_samples": len(trace["feedback"]), "broadcast_batches": len(trace["broadcasts"]),
        "unbroadcast_presets": trace["unbroadcast_presets"],
        "maximum_broadcast_feedback_age_s": max(ages) if ages else None,
        "broadcast_interval_s": ({"minimum": float(np.min(intervals)), "maximum": float(np.max(intervals)),
                                  "median": float(np.median(intervals))} if len(intervals) else None),
        "position_commands": len(trace["positions"]), "tx_commands": trace["tx_commands"],
        "first_interval_violation": violations[0] if violations else None, "interval_violations": violations,
        "last_original_feedback": last,
        "response_dynamics": _response_dynamics(trace["feedback"]),
        "maximum_absolute_host_broadcast_error": {joint: max(values) if values else None for joint, values in errors.items()},
        "reference_time_basis": "most_recent_original_host_broadcast_write_for_each_received_feedback",
        "continuous_roundtrip_completed": roundtrip, "hardware_qualified": False,
        "independent_physical_precision_verified": False, "device_sample_time_known": False,
        "physical_stop_independently_verified": False, "parameter_writes_sent": 0,
    }
