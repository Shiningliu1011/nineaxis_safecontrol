#!/usr/bin/env python3
"""零位标定程序（机械零位标记法）。

流程：
  1. 连接所有电机（使能）
  2. 手动摆到机械零位（限位开关/卡槽/标记处）
  3. 对每个关节执行 set_zero（固化电机零位）
  4. 发 ±5° 命令验证正方向与符号
  5. 把 sign 和 zero_offset_deg 写入 config/hardware_joint_zero.yaml

用法：
  python3 scripts/calibrate_zero.py --interface can0 --node-ids 1,2,3,4,5,6,7,8,9

前置：CANable 已连接、vcan0 或 can0 可用、电机已上电。
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import yaml

# 把 src 加入路径
REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))


def main():
    parser = argparse.ArgumentParser(description="零位标定程序")
    parser.add_argument("--interface", default="can0", help="SocketCAN 接口名")
    parser.add_argument("--node-ids", default="1,2,3,4,5,6,7,8,9",
                        help="逗号分隔的节点 ID")
    parser.add_argument("--output", default=str(REPO_ROOT / "config" / "hardware_joint_zero.yaml"),
                        help="输出 YAML 路径")
    parser.add_argument("--test-amount", type=float, default=5.0,
                        help="方向验证角度（度）")
    parser.add_argument("--dry-run", action="store_true",
                        help="不发送 CAN 帧，只打印流程")
    args = parser.parse_args()

    node_ids = [int(x) for x in args.node_ids.split(",")]
    joint_names = [f"J{i}" for i in range(1, len(node_ids) + 1)]

    print(f"零位标定程序")
    print(f"  接口: {args.interface}")
    print(f"  节点: {node_ids}")
    print(f"  输出: {args.output}")
    print()

    if args.dry_run:
        print("[dry-run] 不发送 CAN 帧。")
        print()
        for name, nid in zip(joint_names, node_ids):
            print(f"  {name} (node {nid}): set_zero → ±{args.test_amount}° 验证")
        print()
        print("标定完成后，编辑 config/hardware_joint_zero.yaml：")
        print("  sign: +1 或 -1（正方向是否与 URDF 一致）")
        print("  zero_offset_deg: 电机零位与机械零位的角度差（set_zero 后通常为 0）")
        return

    # 实际发送（需要 python-can + 真实 CAN 总线）
    try:
        import can
    except ImportError:
        print("错误：python-can 未安装。使用 --dry-run 查看流程。")
        sys.exit(1)

    from robot_safecontrol_moveit.drempower_can import (
        can_id, encode_system, encode_property_write, encode_position,
        SYSTEM_ORDER_CLEAR_ERROR, SYSTEM_ORDER_SET_ZERO,
        CMD_SYSTEM, CMD_PROPERTY_WRITE, CMD_POSITION_ANGLE_MODE0,
        AXIS_STATE_CLOSED_LOOP, PROP_AXIS_REQUESTED_STATE,
    )

    bus = can.interface.Bus(channel=args.interface, bustype="socketcan")

    def send_frame(frame_id, data):
        msg = can.Message(arbitration_id=frame_id, data=data, is_extended_id=False)
        bus.send(msg)
        time.sleep(0.01)

    # 步骤 1：使能所有电机
    print("步骤 1：使能所有电机...")
    for nid in node_ids:
        send_frame(can_id(nid, CMD_SYSTEM), encode_system(nid, SYSTEM_ORDER_CLEAR_ERROR))
        send_frame(can_id(nid, CMD_PROPERTY_WRITE),
                   encode_property_write(nid, PROP_AXIS_REQUESTED_STATE,
                                         AXIS_STATE_CLOSED_LOOP, "u16"))
    time.sleep(0.5)
    print("  所有电机已使能。")

    # 步骤 2：等用户摆到机械零位
    input("\n步骤 2：请手动将机械臂摆到机械零位（各轴限位/卡槽/标记处），然后按 Enter...")

    # 步骤 3：set_zero
    print("\n步骤 3：对每个关节执行 set_zero...")
    for nid in node_ids:
        send_frame(can_id(nid, CMD_SYSTEM), encode_system(nid, SYSTEM_ORDER_SET_ZERO))
        print(f"  node {nid}: set_zero 已发送")
    time.sleep(0.5)
    print("  所有关节零位已固化。")

    # 步骤 4：方向验证
    print(f"\n步骤 4：方向验证（±{args.test_amount}°）...")
    results = {}
    for name, nid in zip(joint_names, node_ids):
        print(f"\n  {name} (node {nid}):")
        input(f"    按 Enter 发送 +{args.test_amount}° 命令...")
        send_frame(can_id(nid, CMD_POSITION_ANGLE_MODE0),
                   encode_position(nid, args.test_amount, speed=1.0, filter_accel=1.0))
        response = input(f"    关节是否向 URDF 正方向移动？(y/n): ").strip().lower()
        sign = 1 if response == "y" else -1
        results[name] = {"sign": sign, "zero_offset_deg": 0.0}
        # 回零
        send_frame(can_id(nid, CMD_POSITION_ANGLE_MODE0),
                   encode_position(nid, 0.0, speed=1.0, filter_accel=1.0))
        time.sleep(0.5)

    # 步骤 5：写入 YAML
    print(f"\n步骤 5：写入 {args.output}...")
    output_path = Path(args.output)
    output_data = {}
    for name in joint_names:
        output_data[name] = {
            "sign": results[name]["sign"],
            "zero_offset_deg": results[name]["zero_offset_deg"],
        }
    with open(output_path, "w", encoding="utf-8") as f:
        yaml.dump(output_data, f, default_flow_style=False, allow_unicode=True)
    print("  标定完成！请检查输出文件并验证。")

    # 失能
    print("\n失能所有电机...")
    for nid in node_ids:
        send_frame(can_id(nid, CMD_PROPERTY_WRITE),
                   encode_property_write(nid, PROP_AXIS_REQUESTED_STATE, 1, "u16"))
    bus.shutdown()
    print("完成。")


if __name__ == "__main__":
    main()
