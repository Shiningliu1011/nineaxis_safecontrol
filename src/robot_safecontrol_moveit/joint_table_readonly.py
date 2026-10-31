import argparse
import json
import time
from dataclasses import asdict
from datetime import datetime
from hashlib import sha256
from math import isfinite
from pathlib import Path
from zoneinfo import ZoneInfo

from .drempower_serial import DrEmpowerSerial
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
MULTI_CIRCLE_PROPERTIES = {
    "node_id": (31001, "u32"), "hw_version": (5, "u32"), "fw_version": (6, "u32"),
    "encoder_mode": (34101, "u32"), "battery_voltage": (34008, "f32"),
    "encoder_error": (34001, "u32"), "axis_error": (30001, "u32"),
    "axis_state": (30002, "u32"), "multi_circle_enabled": (31207, "u32"),
    "encoder_turn_abs": (34010, "s32"), "zero_offset_counts": (31205, "f32"),
    "circular_min_deg": (38010, "f32"), "circular_max_deg": (38011, "f32"),
    "circular_limit_enabled": (31202, "u32"),
}


def main():
    parser = argparse.ArgumentParser(description="九轴与J1断电恢复参数的非运动读取")
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--port", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--scope", choices=("identity", "position", "j1-recovery", "multi-circle"), default="identity")
    args = parser.parse_args()
    if not isfinite(args.duration) or not 1 <= args.duration <= 30:
        parser.error("读取时间必须在 1–30 秒内")
    control = JointTableControl(args.root)
    report = {
        "started_at": datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(), "port": args.port,
        "status": "reading", "motion_commands_sent": 0, "hardware_qualified": False,
        "scope": args.scope,
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
                    joints = ("J1",) if args.scope in ("j1-recovery", "multi-circle") else DEFAULT_JOINT_NAMES
                    if args.scope in ("position", "j1-recovery", "multi-circle"):
                        for joint in joints:
                            report["reading_joint"] = joint
                            report["reading_property"] = "fast_feedback"
                            report["samples"].append(dict(joint=joint, **asdict(reader.read(joint))))
                    for joint in joints:
                        report["reading_joint"] = joint
                        node = control._node(joint)
                        if args.scope == "position":
                            fields = POSITION_PROPERTIES
                        elif args.scope == "multi-circle":
                            fields = MULTI_CIRCLE_PROPERTIES
                        elif args.scope == "j1-recovery":
                            fields = {"node_id": (31001, "u32"), "hw_version": (5, "u32"),
                                      "fw_version": (6, "u32"), **J1_PROPERTIES}
                        else:
                            fields = {**COMMON_PROPERTIES, **(J1_PROPERTIES if joint == "J1" else {})}
                        values = report["properties"][joint] = {}
                        for name, (address, kind) in fields.items():
                            report["reading_property"] = name
                            values[name] = vendor.read_property(node, address, kind)
                        if values["node_id"] != node:
                            raise ValueError(f"{joint} 节点回读与配置不符")
                    until = time.monotonic() + args.duration
                    while time.monotonic() < until:
                        for joint in joints:
                            sample = reader.read(joint)
                            report["samples"].append(dict(joint=joint, **asdict(sample)))
                        time.sleep(0.02)
                    report["status"] = "readonly_completed"
                finally:
                    report["frames"] = vendor.frames
        except BaseException as error:
            report.update(status="readonly_failed", reason=str(error))
            raise
        finally:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
    print(f"非运动读取完成：{args.output}")


if __name__ == "__main__":
    main()
