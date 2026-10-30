import argparse
import json
import time
from bisect import bisect_left
from dataclasses import asdict
from datetime import datetime
from hashlib import sha256
from math import isfinite
from pathlib import Path
from zipfile import ZipFile
from zoneinfo import ZoneInfo

from construct import Const, ConstructError, Int16ul, Struct

from .drempower_can import (
    CMD_PROPERTY_READ, PROPERTY_TYPES, decode_feedback, decode_property_reply,
)
from .drempower_serial import DrEmpowerSerial, SERIAL_CAN_FRAME
from .joint_table_execution import JointTableControl
from .robot_spec import DEFAULT_JOINT_NAMES
from .single_joint_execution import VendorFeedbackReader

# 地址与类型来自厂家串口 SDK v2.0 的 interface_enums.py。
COMMON_PROPERTIES = {
    "node_id": (31001, "u32"), "serial_number_l": (3, "u32"),
    "serial_number_h": (4, "u32"), "hw_version": (5, "u32"), "fw_version": (6, "u32"),
    "axis_state": (30002, "u32"), "axis_error": (30001, "u32"),
    "can_error": (20001, "u32"), "motor_error": (33001, "u32"),
    "encoder_error": (34001, "u32"), "controller_error": (32001, "u32"),
    "vbus_voltage": (1, "f32"), "auto_reply": (31214, "u32"),
    "output_pos_deg": (38007, "f32"), "output_vel_rpm": (38008, "f32"),
    "can_timeout_ms": (21003, "u32"), "traj_mode": (35104, "u32"),
}
J1_PROPERTIES = {
    "circular_min_deg": (38010, "f32"), "circular_max_deg": (38011, "f32"),
    "circular_limit_enabled": (31202, "u32"), "multi_circle_enabled": (31207, "u32"),
    "zero_offset_counts": (31205, "f32"), "stored_turns_counts": (31206, "s32"),
    "encoder_turn_abs": (34010, "s32"), "battery_voltage": (34008, "f32"),
}
POSITION_PROPERTIES = {
    "node_id": (31001, "u32"), "axis_state": (30002, "u32"),
    "axis_error": (30001, "u32"), "output_pos_deg": (38007, "f32"),
    "output_vel_rpm": (38008, "f32"), "output_vel_limit_rpm": (38012, "f32"),
    "trap_velocity_rpm": (38014, "f32"), "trap_acceleration_rpm_s": (38015, "f32"),
    "trap_deceleration_rpm_s": (38016, "f32"), "traj_mode": (35104, "u32"),
}
OPERATING_PROPERTIES = {
    **{name: field for name, field in COMMON_PROPERTIES.items() if name != "serial_number_h"},
    **POSITION_PROPERTIES,
    "can_baud_rate": (21001, "u32"), "control_mode": (32105, "u32"),
    "input_mode": (32106, "u32"), "input_filter_bandwidth": (32118, "f32"),
    "trajectory_done": (32008, "u32"),
}
SERVO_PROPERTIES = {
    **OPERATING_PROPERTIES,
    "output_input_pos_deg": (38001, "f32"), "output_input_vel_rpm": (38002, "f32"),
    "output_input_torque_nm": (38003, "f32"), "output_pos_setpoint_deg": (38004, "f32"),
    "output_vel_setpoint_rpm": (38005, "f32"), "output_torque_setpoint_nm": (38006, "f32"),
    "output_torque_estimate_nm": (38009, "f32"), "output_torque_limit_nm": (38013, "f32"),
    "pos_gain": (32108, "f32"), "vel_gain": (32109, "f32"),
    "vel_integrator_gain": (32110, "f32"),
    "current_control_bandwidth": (33114, "f32"),
}
MULTI_CIRCLE_PROPERTIES = {
    "node_id": (31001, "u32"), "hw_version": (5, "u32"), "fw_version": (6, "u32"),
    "encoder_mode": (34101, "u32"), "battery_voltage": (34008, "f32"),
    "encoder_error": (34001, "u32"), "axis_error": (30001, "u32"),
    "axis_state": (30002, "u32"), "multi_circle_enabled": (31207, "u32"),
    "encoder_turn_abs": (34010, "s32"), "zero_offset_counts": (31205, "f32"),
    "circular_min_deg": (38010, "f32"), "circular_max_deg": (38011, "f32"),
    "circular_limit_enabled": (31202, "u32"),
}

ENCODER_PROPERTIES = {
    **{name: field for name, field in COMMON_PROPERTIES.items() if name != "serial_number_h"},
    "gear_ratio": (31208, "f32"), "cpr": (34103, "s32"), "pole_pairs": (33102, "s32"),
    **POSITION_PROPERTIES,
    **{name: field for name, field in J1_PROPERTIES.items() if name != "stored_turns_counts"},
    "rotor_pos_turns": (34004, "f32"), "rotor_vel_turns_s": (34006, "f32"),
    "encoder_pos_abs": (34009, "s32"),
}

ENCODER_CONFIGURATION_FIELDS = (
    "node_id", "serial_number_l", "hw_version", "fw_version", "gear_ratio", "cpr", "pole_pairs",
    "zero_offset_counts", "multi_circle_enabled", "circular_min_deg", "circular_max_deg",
    "circular_limit_enabled",
)


PROPERTY_READ_REQUEST = Struct("address" / Int16ul, "type_code" / Int16ul, "reserved" / Const(bytes(4)))


def replay_readonly_report(source: bytes) -> dict:
    recorded = json.loads(source)
    fields = {**COMMON_PROPERTIES, **J1_PROPERTIES, **POSITION_PROPERTIES, **MULTI_CIRCLE_PROPERTIES,
              **ENCODER_PROPERTIES, **OPERATING_PROPERTIES, **SERVO_PROPERTIES, "encoder_ready": (34002, "u32")}
    names = {(address, kind): name for name, (address, kind) in fields.items()}
    kinds = {code: name for name, (code, _) in PROPERTY_TYPES.items()}
    report = {
        "validation_scope": "offline_original_readonly_replay",
        "source_sha256": sha256(source).hexdigest(), "source_bytes": len(source),
        "source_status": recorded.get("status"), "status": "replayed_completed",
        "device_sample_time_known": False, "hardware_qualified": False,
        "motion_commands_sent": 0, "parameter_writes_sent": 0,
        "initial_nodes": {}, "nodes": {}, "property_observations": [],
        "feedback_observations": [], "incomplete_reads": [],
    }
    frames = recorded["frames"]
    if not frames:
        raise ValueError("原始读取报告缺少收发帧")
    pending = None
    previous_s = -1.
    for index, frame in enumerate(frames):
        stamp = frame["monotonic_s"]
        if type(stamp) not in (float, int) or not isfinite(stamp) or stamp < 0 or stamp < previous_s:
            raise ValueError(f"原始帧 {index} 的主机时间无效")
        previous_s = stamp
        raw = bytes.fromhex(frame["raw_hex"])
        try:
            parsed = SERIAL_CAN_FRAME.parse(raw)
        except ConstructError as error:
            # 原始采集在最后一次读取失败时终止；保留已有数据与缺失字段。
            if (recorded.get("status") != "readonly_failed" or frame["direction"] != "rx"
                    or pending is None or index != len(frames) - 1):
                raise
            node, address, kind, request_s = pending
            report["status"] = "replayed_incomplete"
            report["incomplete_reads"].append({
                "node_id": node, "name": names.get((address, kind)), "address": address, "kind": kind,
                "request_s": request_s, "received_s": stamp, "response_bytes": len(raw), "reason": str(error),
            })
            pending = None
            break
        node = parsed.frame_id >> 5
        if not 1 <= node <= 9 or parsed.frame_id > 0x7FF:
            raise ValueError(f"原始帧 {index} 的节点或 CAN ID 无效")
        if frame["direction"] == "tx":
            if parsed.frame_id & 0x1F != CMD_PROPERTY_READ:
                raise ValueError(f"原始帧 {index} 包含运动或参数写入命令")
            if pending is not None:
                raise ValueError(f"原始帧 {index} 存在未完成的读取")
            request = PROPERTY_READ_REQUEST.parse(parsed.data)
            if request.type_code not in kinds:
                raise ValueError(f"原始帧 {index} 的属性类型无效")
            if request.address == 0 and parsed.data != bytes(8):
                raise ValueError(f"原始帧 {index} 的快速读取请求必须为八个零字节")
            pending = (node, request.address, kinds[request.type_code], stamp)
        elif frame["direction"] == "rx":
            if pending is None or pending[0] != node:
                raise ValueError(f"原始帧 {index} 没有对应节点的读取请求")
            _, address, kind, request_s = pending
            if address == 0:
                feedback = asdict(decode_feedback(parsed.frame_id, parsed.data))
                report["feedback_observations"].append({**feedback, "request_s": request_s, "received_s": stamp})
            else:
                value = decode_property_reply(parsed.frame_id, parsed.data, node_id=node,
                                              address=address, value_kind=kind)
                name = names.get((address, kind))
                report["property_observations"].append({
                    "node_id": node, "name": name, "address": address, "kind": kind,
                    "value": value, "request_s": request_s, "received_s": stamp,
                })
                if name is not None:
                    report["initial_nodes"].setdefault(str(node), {}).setdefault(name, value)
                    report["nodes"].setdefault(str(node), {})[name] = value
            pending = None
        else:
            raise ValueError(f"原始帧 {index} 的 direction 无效")
    if pending is not None:
        raise ValueError("原始报告存在未完成的读取")
    declared = recorded.get("properties", {})
    if "node_id" in declared:
        groups = [(declared["node_id"], declared)]
    else:
        groups = [(group["node_id"], group) for group in declared.values()]
    if "values" in recorded:
        groups.append((recorded["node_id"], recorded["values"]))
    for node_id, group in groups:
        node_values = report["initial_nodes"].get(str(node_id), {})
        for name, value in group.items():
            if name in fields and (name not in node_values or node_values[name] != value):
                raise ValueError(f"报告中的 {name} 与原始收发帧不符")
    if "property_observations" in recorded:
        ledger = recorded["property_observations"]
        raw_rows = report["property_observations"]
        if len(ledger) != len(raw_rows):
            raise ValueError("属性读取记录数量与原始帧不符")
        for row, raw_row in zip(ledger, raw_rows):
            if (any(row[name] != raw_row[name] for name in ("node_id", "name", "address", "kind", "value"))
                    or not _read_interval_contains(row["request_s"], row["received_s"], raw_row)):
                raise ValueError("属性读取记录与原始帧不符")
    report["encoder_samples"] = replay_encoder_samples(recorded, report)
    return report


def _read_interval_contains(start, finish, row: dict) -> bool:
    return (type(start) in (int, float) and type(finish) in (int, float)
            and isfinite(start) and isfinite(finish)
            and 0 <= start <= row["request_s"] <= row["received_s"] <= finish)


def replay_encoder_samples(recorded: dict, report: dict) -> list[dict]:
    samples = recorded.get("encoder_samples", [])
    if not samples:
        return []
    fields = recorded["encoder_sample_fields"]
    if not fields or len(set(fields)) != len(fields) or any(name not in ENCODER_PROPERTIES for name in fields):
        raise ValueError("连续采集字段无效")
    names = tuple(dict.fromkeys((*ENCODER_CONFIGURATION_FIELDS, *fields)))
    rows = report["property_observations"]
    request_times = [row["request_s"] for row in rows]
    previous_finish = 0.
    verified = []
    for sample in samples:
        start, finish = sample["started_s"], sample["finished_s"]
        if (type(start) not in (int, float) or type(finish) not in (int, float)
                or not isfinite(start) or not isfinite(finish) or not 0 <= previous_finish <= start <= finish):
            raise ValueError("连续采集主机时间无效")
        first, last = bisect_left(request_times, start), bisect_left(request_times, finish)
        observations = rows[first:last]
        if (any(row["node_id"] != sample["node_id"] or not _read_interval_contains(start, finish, row)
                for row in observations)
                or [row["name"] for row in observations] != list(names[:len(observations)])):
            raise ValueError("连续采集读取顺序或时间与原始帧不符")
        values = {row["name"]: row["value"] for row in observations}
        missing = [name for name in names if name not in values]
        initial = report["initial_nodes"][str(sample["node_id"])]
        changed = any(values[name] != initial[name] for name in ENCODER_CONFIGURATION_FIELDS if name in values)
        matches = False if changed else None if missing else True
        if (values != sample["values"] or missing != sample["missing_properties"]
                or sample["configuration_matches"] is not matches
                or (recorded.get("status") == "readonly_completed" and matches is not True)):
            raise ValueError("连续采集数值或配置检查与原始帧不符")
        verified.append({**sample, "observations": observations})
        previous_finish = finish
    return verified


def replay_readonly_archive(path: Path, member: str) -> dict:
    with ZipFile(path) as archive:
        if archive.namelist().count(member) != 1 or archive.namelist().count("provenance.json") != 1:
            raise ValueError("原始归档必须具有唯一的数据文件和 provenance.json")
        source = archive.read(member)
        provenance = json.loads(archive.read("provenance.json"))
        files = provenance["files"] if isinstance(provenance, dict) else provenance
        entries = [item for item in files if item["member"] == member]
        if (len(entries) != 1 or entries[0]["bytes"] != len(source)
                or entries[0]["sha256"] != sha256(source).hexdigest()):
            raise ValueError(f"原始归档校验失败：{member}")
    return {**replay_readonly_report(source), "archive_member": member,
            "archive_sha256": sha256(path.read_bytes()).hexdigest(), "archive_member_verified": True}


def read_recorded_property(vendor, report: dict, joint: str, node: int, name: str, field: tuple):
    address, kind = field
    report["reading_joint"], report["reading_property"] = joint, name
    request_s = time.monotonic()
    value = vendor.read_property(node, address, kind)
    report["property_observations"].append({
        "joint": joint, "node_id": node, "name": name, "address": address,
        "kind": kind, "value": value, "request_s": request_s,
        "received_s": time.monotonic(), "device_sample_time_known": False,
    })
    return value


def sample_encoder_properties(vendor, report: dict, joint: str, node: int, fields: tuple):
    names = tuple(dict.fromkeys((*ENCODER_CONFIGURATION_FIELDS, *fields)))
    values: dict = {}
    sample = {"joint": joint, "node_id": node, "started_s": time.monotonic(), "values": values,
              "finished_s": None, "configuration_matches": None, "device_sample_time_known": False}
    report["encoder_samples"].append(sample)
    try:
        for name in names:
            values[name] = read_recorded_property(vendor, report, joint, node, name,
                                                   ENCODER_PROPERTIES[name])
            if name in ENCODER_CONFIGURATION_FIELDS and values[name] != report["properties"][joint][name]:
                sample["configuration_matches"] = False
                raise ValueError(f"{joint} 连续计数采集期间设备配置变化：{name}")
        sample["configuration_matches"] = True
    finally:
        sample["finished_s"] = time.monotonic()
        sample["missing_properties"] = [name for name in names if name not in values]


def main():
    parser = argparse.ArgumentParser(description="九轴与J1断电恢复参数的非运动读取")
    parser.add_argument("--root", type=Path, required=True)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port")
    source.add_argument("--replay", type=Path, help="离线核对原始只读 JSON 或 ZIP")
    parser.add_argument("--archive-member", help="ZIP 中具有 provenance 校验的只读 JSON")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--scope", choices=("identity", "position", "j1-recovery", "multi-circle", "encoder", "operating", "servo"),
                        default="identity")
    parser.add_argument("--joints", nargs="+", choices=DEFAULT_JOINT_NAMES,
                        help="encoder 或 servo 读取的关节，默认全部九轴")
    parser.add_argument("--fields", nargs="+", choices=tuple(ENCODER_PROPERTIES),
                        help="encoder 明确读取的字段；设备身份始终读取")
    parser.add_argument("--sample-fields", nargs="+", choices=tuple(ENCODER_PROPERTIES),
                        help="encoder 连续采集的属性；每批同时核对设备计数配置")
    args = parser.parse_args()
    if not isfinite(args.duration) or not 1 <= args.duration <= 30:
        parser.error("读取时间必须在 1–30 秒内")
    if args.archive_member and args.replay is None:
        parser.error("--archive-member 需要 --replay")
    if args.joints and (args.scope not in ("encoder", "servo") or args.replay is not None):
        parser.error("--joints 仅适用于 --scope encoder 或 servo 的设备读取")
    if (args.fields or args.sample_fields) and (args.scope != "encoder" or args.replay is not None):
        parser.error("--fields 和 --sample-fields 仅适用于 --scope encoder 的设备读取")
    if args.replay is not None:
        if args.replay.suffix == ".zip" and not args.archive_member:
            parser.error("ZIP 回放需要 --archive-member")
        report = (replay_readonly_archive(args.replay, args.archive_member) if args.archive_member
                  else replay_readonly_report(args.replay.read_bytes()))
        with args.output.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
        print(f"原始只读记录离线核对完成：{args.output}")
        return
    control = JointTableControl(args.root)
    joints = ("J1",) if args.scope in ("j1-recovery", "multi-circle") else tuple(args.joints or DEFAULT_JOINT_NAMES)
    if (len(set(joints)) != len(joints) or (args.fields and len(set(args.fields)) != len(args.fields))
            or (args.sample_fields and len(set(args.sample_fields)) != len(args.sample_fields))):
        parser.error("关节和字段不能重复")
    requested_fields = {}
    for joint in joints:
        if args.scope == "encoder":
            fields = {name: ENCODER_PROPERTIES[name]
                      for name in ("node_id", "serial_number_l", "hw_version", "fw_version")}
            fields.update({name: ENCODER_PROPERTIES[name] for name in (args.fields or ENCODER_PROPERTIES)})
            if args.sample_fields:
                fields.update({name: ENCODER_PROPERTIES[name]
                               for name in (*ENCODER_CONFIGURATION_FIELDS, *args.sample_fields)})
        elif args.scope == "position":
            fields = POSITION_PROPERTIES
        elif args.scope == "operating":
            fields = OPERATING_PROPERTIES
        elif args.scope == "servo":
            fields = SERVO_PROPERTIES
        elif args.scope == "multi-circle":
            fields = MULTI_CIRCLE_PROPERTIES
        elif args.scope == "j1-recovery":
            fields = {"node_id": (31001, "u32"), "hw_version": (5, "u32"),
                      "fw_version": (6, "u32"), **J1_PROPERTIES}
        else:
            fields = {**COMMON_PROPERTIES, **(J1_PROPERTIES if joint == "J1" else {})}
        requested_fields[joint] = fields
    report = {
        "started_at": datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(), "port": args.port,
        "status": "reading", "motion_commands_sent": 0, "hardware_qualified": False,
        "scope": args.scope, "parameter_writes_sent": 0,
        "property_observations": [], "requested_properties": requested_fields,
        "encoder_samples": [], "encoder_sample_fields": args.sample_fields or [],
        "read_time_basis": "host_monotonic_request_and_receive",
        "property_profile": ("manufacturer_G_static_v1" if args.scope == "encoder"
                             else "manufacturer_python_interface_enums_v2"),
        "device_sample_time_known": False, "zero_recovery_verified": False,
        "physical_stop_independently_verified": False, "properties": {}, "samples": [],
        "configuration_sha256": {name: sha256((args.root / "config" / name).read_bytes()).hexdigest()
                                  for name in ("drempower.yaml", "hardware_joint_zero.yaml", "j1_transmission.yaml")},
    }
    with args.output.open("x", encoding="utf-8") as stream:
        try:
            # 读取连接没有任何节点的运动或参数写入权限。
            with DrEmpowerSerial(args.port) as vendor:
                try:
                    reader = VendorFeedbackReader(vendor, control, 0.2)
                    if args.scope in ("position", "j1-recovery", "multi-circle", "encoder", "operating", "servo"):
                        for joint in joints:
                            report["reading_joint"] = joint
                            report["reading_property"] = "fast_feedback"
                            report["samples"].append(dict(joint=joint, **asdict(reader.read(joint))))
                    for joint in joints:
                        report["reading_joint"] = joint
                        node = control._node(joint)
                        values = report["properties"][joint] = {}
                        for name, field in requested_fields[joint].items():
                            values[name] = read_recorded_property(vendor, report, joint, node, name, field)
                        if values["node_id"] != node:
                            raise ValueError(f"{joint} 节点回读与配置不符")
                    until = time.monotonic() + args.duration
                    while time.monotonic() < until:
                        for joint in joints:
                            sample = reader.read(joint)
                            report["samples"].append(dict(joint=joint, **asdict(sample)))
                            if args.sample_fields:
                                sample_encoder_properties(vendor, report, joint, control._node(joint),
                                                          tuple(args.sample_fields))
                        time.sleep(0.02)
                    report["status"] = "readonly_completed"
                finally:
                    report["frames"] = vendor.frames
        except BaseException as error:
            report.update(status="readonly_failed", reason=str(error))
            raise
        finally:
            report["missing_properties"] = {
                joint: [name for name in fields if name not in report["properties"].get(joint, {})]
                for joint, fields in requested_fields.items()
            }
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
    print(f"非运动读取完成：{args.output}")


if __name__ == "__main__":
    main()
