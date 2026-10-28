import argparse
from dataclasses import asdict, replace
from datetime import datetime
import json
from math import isfinite
from pathlib import Path
import statistics
import time
from zoneinfo import ZoneInfo

import yaml

from .drempower_can import decode_property_reply, encode_property_read
from .drempower_joint_control import DrEmpowerJointControl
from .drempower_serial import DrEmpowerSerial
from .motor_stop_latch import MotorStopLatch
from .single_joint_execution import ExecutionLimits, VendorFeedbackReader, run_single_joint
from .unit_conversion import JointNodeMap


def load_control(root: Path):
    with (root / "config/drempower.yaml").open() as stream:
        params = yaml.safe_load(stream)["/**"]["ros__parameters"]
    return DrEmpowerJointControl(root / "config/hardware_joint_zero.yaml", JointNodeMap(params["node_ids"]))


def read_property(vendor, node, address, kind):
    vendor.send_command(node, 0x1E, encode_property_read(node, address, kind))
    frame = vendor.receive_data(return_id=True)
    return decode_property_reply((frame[1] << 8) | frame[2], bytes(frame[3:]),
                                 node_id=node, address=address, value_kind=kind)


def diagnose(vendor, control, joints, duration_s):
    reader = VendorFeedbackReader(vendor, control, 0.2)
    properties = {}
    for joint in joints:
        node = control._node(joint)
        properties[joint] = {
            "node_id": read_property(vendor, node, 31001, "u32"),
            "axis_state": read_property(vendor, node, 30002, "u32"),
            "axis_error": read_property(vendor, node, 30001, "u32"),
            "can_error": read_property(vendor, node, 20001, "u32"),
            "motor_error": read_property(vendor, node, 33001, "u32"),
            "encoder_error": read_property(vendor, node, 34001, "u32"),
            "controller_error": read_property(vendor, node, 32001, "u32"),
            "vbus_voltage": read_property(vendor, node, 1, "f32"),
            "auto_reply": read_property(vendor, node, 31214, "u32"),
            "output_pos": read_property(vendor, node, 38007, "f32"),
            "output_vel": read_property(vendor, node, 38008, "f32"),
        }
        if properties[joint]["node_id"] != node:
            raise ValueError("节点属性与关节映射不符")
    samples = {joint: [] for joint in joints}
    until = time.monotonic() + duration_s
    while time.monotonic() < until:
        for joint in joints:
            item = reader.read(joint)
            row = asdict(item)
            row["property_vel"] = read_property(vendor, item.node_id, 38008, "f32")
            row["axis_state"] = read_property(vendor, item.node_id, 30002, "u32")
            samples[joint].append(row)
        time.sleep(0.02)
    summary = {}
    for joint, rows in samples.items():
        angles = [row["state"]["angle_deg"] for row in rows]
        velocities = [row["state"]["speed_rpm"] for row in rows]
        elapsed = rows[-1]["received_s"] - rows[0]["received_s"]
        interval_velocities = [(right["state"]["angle_deg"] - left["state"]["angle_deg"])
                               / (right["received_s"] - left["received_s"]) / 6
                               for left, right in zip(rows, rows[1:])]
        summary[joint] = dict(samples=len(rows), median_angle_deg=statistics.median(angles),
                              position_range_deg=max(angles) - min(angles),
                              max_raw_rpm=max(map(abs, velocities)),
                              max_property_vel=max(abs(row["property_vel"]) for row in rows),
                              max_position_difference_rpm=max(map(abs, interval_velocities)),
                              net_position_rpm=(angles[-1] - angles[0]) / elapsed / 6,
                              faults=sum(row["axis_error"] for row in rows),
                              properties=properties[joint])
    return dict(summary=summary, samples=samples)


def main():
    parser = argparse.ArgumentParser(description="DrEmpower 单关节连接与低速检查")
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--port", required=True)
    parser.add_argument("--output", type=Path, required=True)
    commands = parser.add_subparsers(dest="action", required=True)
    diagnostic = commands.add_parser("diagnose")
    diagnostic.add_argument("--joints", nargs="+", choices=[f"J{i}" for i in range(2, 10)],
                            default=[f"J{i}" for i in range(2, 10)])
    diagnostic.add_argument("--duration", type=float, default=5)
    motion = commands.add_parser("move")
    motion.add_argument("--joint", choices=[f"J{i}" for i in range(2, 10)], required=True)
    target_group = motion.add_mutually_exclusive_group(required=True)
    target_group.add_argument("--delta-deg", type=float)
    target_group.add_argument("--target-deg", type=float)
    motion.add_argument("--speed-rpm", type=float, default=0.05)
    motion.add_argument("--accel-rpm-s", type=float, default=0.05)
    motion.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if args.action == "diagnose" and not 1 <= args.duration <= 60:
        parser.error("采集时间必须在 1–60 秒内")
    control = load_control(args.root)
    if args.action == "move":
        limits = ExecutionLimits.from_urdf(args.root / "models/ninezzhou/urdf/ninezzhou.urdf", args.joint)
        limits = replace(limits, speed_rpm=args.speed_rpm, accel_rpm_s=args.accel_rpm_s)
        # 按最长允许行程及梯形加减速时间计算观察时限。
        travel_s = limits.max_step_deg / (6 * limits.speed_rpm) + limits.speed_rpm / limits.accel_rpm_s
        limits = replace(limits, motion_timeout_s=max(limits.motion_timeout_s, travel_s + 2))
        if args.delta_deg is not None and not 0 < abs(args.delta_deg) <= limits.max_step_deg:
            parser.error("单次增量必须非零且不超过 5°")
        if args.target_deg is not None and (not isfinite(args.target_deg)
                                             or not limits.lower_deg <= args.target_deg <= limits.upper_deg):
            parser.error("绝对目标必须位于 URDF 关节范围内")
        if not args.execute:
            print(json.dumps(dict(joint=args.joint, delta_deg=args.delta_deg, target_deg=args.target_deg,
                                  node=control._node(args.joint), limits=asdict(limits),
                                  status="plan_only"), ensure_ascii=False, indent=2))
            return
        latch = MotorStopLatch(args.root / ".scratch/motor_control/stop.json")
        latch.require_clear()
        frames_path = args.output.with_suffix(".frames.json")
        if args.output.exists() or frames_path.exists():
            raise FileExistsError("执行日志或收发记录已存在")
        try:
            with DrEmpowerSerial(args.port, command_node=control._node(args.joint)) as vendor:
                try:
                    vendor.prepare_motion()
                    run_single_joint(vendor, control, args.joint, args.delta_deg, limits, args.output,
                                     target_deg=args.target_deg)
                finally:
                    with frames_path.open("x") as frames:
                        json.dump(vendor.frames, frames, ensure_ascii=False, indent=2)
        except BaseException as error:
            latch.trip(reason=str(error), log_path=args.output)
            raise
        description = (f"模型相对角度 {args.delta_deg:+g}°" if args.delta_deg is not None
                       else f"模型绝对目标 {args.target_deg:+g}°")
        print(f"{args.joint} 已完成{description}并保持目标位置", flush=True)
        return
    with args.output.open("x", encoding="utf-8") as log:
        report = dict(started_at=datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(),
                      action=args.action, port=args.port, motion_commands_sent=0)
        with DrEmpowerSerial(args.port) as vendor:
            try:
                report.update(diagnose(vendor, control, args.joints, args.duration))
                print(json.dumps(report["summary"], ensure_ascii=False, indent=2), flush=True)
            finally:
                report["frames"] = vendor.frames
                log.write(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
