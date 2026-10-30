import argparse
from contextlib import redirect_stdout
import json
import os
import select
import sys
import time
from dataclasses import asdict, replace
from decimal import ROUND_FLOOR, Decimal
from hashlib import sha256
from math import isfinite
from pathlib import Path
from typing import TextIO
from uuid import uuid4

from .j1_execution import J1ScrewControl
from .joint_table import (
    JointTable,
    JointTableRow,
    joint_execution_limits,
)
from .joint_table_execution import JointTableExecutor
from .oscbf_trajectory import bootstrap_portable
from .robot_spec import DEFAULT_JOINT_NAMES


def positioning_limits(root: Path, maximum_speed: dict, acceleration: dict) -> dict:
    if set(maximum_speed) != set(DEFAULT_JOINT_NAMES) or set(acceleration) != set(DEFAULT_JOINT_NAMES):
        raise ValueError("速度和加速度必须完整包含 J1–J9")
    limits = joint_execution_limits(root)
    linear = J1ScrewControl.from_root(root)
    for joint in DEFAULT_JOINT_NAMES:
        values = (maximum_speed[joint], acceleration[joint])
        if any(type(value) not in (int, float) or not isfinite(value) or value <= 0 for value in values):
            raise ValueError(f"{joint} 速度和加速度必须为有限正数")
        factor = Decimal(str(linear.degrees_per_mm if joint == "J1" else 1))
        speed, accel = (float((Decimal(str(value)) * factor / 6).quantize(Decimal("0.01"), rounding=ROUND_FLOOR))
                        for value in values)
        # 单组定位采用项目单位：J1 为 1 mm，其余关节为 0.1°。
        tolerance = linear.degrees_per_mm if joint == "J1" else .1
        limits[joint] = replace(limits[joint], speed_rpm=speed, accel_rpm_s=accel, tolerance_deg=tolerance)
    return limits


def single_target_table(target_joints: dict, model, *, pose_id: str = "manual_001") -> JointTable:
    from work.robot_units import joints_mm_deg_to_si

    if set(target_joints) != set(DEFAULT_JOINT_NAMES):
        raise ValueError("TARGET_JOINTS 必须完整包含 J1–J9")
    values = [target_joints[joint] for joint in DEFAULT_JOINT_NAMES]
    if any(type(value) not in (int, float) or not isfinite(value) for value in values):
        raise ValueError("TARGET_JOINTS 必须包含九个有限数值")
    source = json.dumps(dict(zip(("J1_mm", *(f"{joint}_deg" for joint in DEFAULT_JOINT_NAMES[1:])), values)),
                        allow_nan=False, sort_keys=True).encode("utf-8")
    table = JointTable((JointTableRow(pose_id, tuple(joints_mm_deg_to_si(values))),),
                       sha256(source).hexdigest(), model.model_hash)
    table.preview(model, measurement_kind="positioning")
    return table


class PositioningOperator:
    def __init__(self, stream: TextIO, *, output: TextIO, start_immediately: bool):
        self.fd = stream.fileno()
        self.output = output
        self.start_immediately = start_immediately
        self.first_started = False
        self.buffer = b""
        self.eof = False
        self.prompt_identity: tuple[str, int] | None = None

    def _read_commands(self) -> list[str]:
        if self.eof:
            return ["q"]
        if not select.select([self.fd], [], [], 0)[0]:
            return []
        data = os.read(self.fd, 4096)
        if not data:
            self.eof = True
            return ["q"]
        lines = (self.buffer + data).split(b"\n")
        self.buffer = lines.pop()
        if len(self.buffer) > 4096:
            raise ValueError("终端输入过长")
        return [line.decode("utf-8").strip().lower() for line in lines]

    def _begin_prompt(self, executor: JointTableExecutor, commands: list[str]) -> list[str]:
        # 清除提示出现前的 Enter 和 r，保留退出请求和无效输入。
        for _ in range(16):
            if self.eof or not select.select([self.fd], [], [], 0)[0]:
                break
            commands.extend(self._read_commands())
        else:
            raise ValueError("终端输入过多")
        self.buffer = b""
        commands = [value for value in commands if value not in ("", "r")]
        self.prompt_identity = (executor.phase, executor.index)
        if executor.phase == "holding_zero":
            print("九轴回零完成，保持使能并持续监测；输入 q 并回车退出。", file=self.output, flush=True)
            return commands
        if executor.phase == "holding_position":
            pose_id = executor.table.rows[executor.index].pose_id
            feedback = executor.feedback_report()
            errors = ", ".join(f"{joint}={value['error_mm_deg']:+.6g} {'mm' if joint == 'J1' else '°'}"
                               for joint, value in feedback.items())
            print(f"{pose_id} 九轴已到位，持续监测并保持目标。误差：{errors}", file=self.output, flush=True)
            if executor.index + 1 == len(executor.table.rows):
                print("最后一组保持中；实验结束后输入 r 并回车回零；输入 q 并回车退出。",
                      file=self.output, flush=True)
                return commands
        next_index = executor.index + (executor.phase == "holding_position")
        row = executor.preview["rows"][next_index]
        print(f"下一组 {row['pose_id']}：{row['target_mm_deg']}（J1：mm；J2–J9：°）。"
              "按 Enter 执行；实验结束后输入 r 并回车回零；输入 q 并回车退出。",
              file=self.output, flush=True)
        return commands

    def step(self, executor: JointTableExecutor, *, now_s: float) -> list[dict]:
        if not executor.manual_positioning:
            raise ValueError("终端操作仅用于手动定位")
        commands = self._read_commands()
        automatic_start = executor.phase == "ready" and self.start_immediately and not self.first_started
        waiting = executor.phase in ("ready", "holding_position", "holding_zero")
        if waiting and not automatic_start and self.prompt_identity != (executor.phase, executor.index):
            commands = self._begin_prompt(executor, commands)
        if any(value not in ("", "q", "r") for value in commands):
            raise ValueError("终端只接受 Enter、r 或 q")
        current_s = time.monotonic() if executor.feedback_source == "hardware_feedback" else now_s
        if "q" in commands:
            if waiting:
                executor.finish_positioning(now_s=current_s)
                return []
            raise KeyboardInterrupt("operator_stop")
        if automatic_start:
            self.first_started = True
            return executor.begin(now_s=current_s)
        if "r" in commands and executor.phase in ("ready", "holding_position"):
            commands_to_send = executor.begin_return_to_zero(now_s=current_s)
            print("已确认实验结束，正在准备九轴回零。", file=self.output, flush=True)
            return commands_to_send
        if executor.phase == "holding_zero":
            return []
        if "" not in commands or not waiting:
            return []
        if executor.phase == "holding_position":
            if executor.index + 1 == len(executor.table.rows):
                return []
            executor.advance_positioning(now_s=current_s)
        self.first_started = True
        return executor.begin(now_s=current_s)


def run_positioning_cli(*, root: Path, target_joints: dict, maximum_speed: dict, acceleration: dict,
                        csv_path: Path | None = None, port: str | None = None, argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="九轴绝对位置与 CSV 手动逐组定位")
    parser.add_argument("--root", type=Path, default=root)
    targets = parser.add_mutually_exclusive_group()
    targets.add_argument("--csv", type=Path, default=csv_path)
    targets.add_argument("--joints", type=float, nargs=9, metavar="VALUE",
                         help="按 J1–J9 输入九个绝对目标；J1 使用 mm，其余使用 °")
    parser.add_argument("--port", default=port)
    parser.add_argument("--preview", action="store_true", help="检查目标与参数并显示报告")
    parser.add_argument("--start-readonly", type=Path, help="从完整原始只读记录生成共同进度预览")
    parser.add_argument("--scene-config", type=Path, help="共同进度预览使用的静态实验场景配置")
    parser.add_argument("--experiment-policy", type=Path, help="本机有人值守定位的现场记录")
    parser.add_argument("--output", type=Path, help="预览 JSON 或运行 JSONL 的新文件路径")
    args = parser.parse_args(argv)
    if (args.start_readonly is not None or args.scene_config is not None) and not args.preview:
        parser.error("--start-readonly 和 --scene-config 需要 --preview")
    if args.scene_config is not None and args.start_readonly is None:
        parser.error("--scene-config 需要 --start-readonly")
    if args.joints is not None:
        args.csv = None
        target_joints = dict(zip(DEFAULT_JOINT_NAMES, args.joints))
    root = args.root.resolve()
    if not args.preview and not args.port:
        parser.error("请设置 SERIAL_PORT 或提供 --port")
    bootstrap_portable(root / "portable_oscbf")
    from work.nineaxis_model import NineaxisModel

    model = NineaxisModel.load(root=root)
    source_path = (args.csv if args.csv.is_absolute() else root / args.csv) if args.csv is not None else None
    table = JointTable.load(source_path, model) if source_path is not None else single_target_table(target_joints, model)
    limits = positioning_limits(root, maximum_speed, acceleration)
    preview = table.preview(model, measurement_kind="positioning", execution_limits=limits)
    control = J1ScrewControl.from_root(root)
    preview["parameters"] = {
        joint: {"requested_speed": maximum_speed[joint], "requested_acceleration": acceleration[joint],
                "effective_speed": limit.speed_rpm * 6 / (control.degrees_per_mm if joint == "J1" else 1),
                "effective_acceleration": limit.accel_rpm_s * 6 / (control.degrees_per_mm if joint == "J1" else 1),
                "position_tolerance": limit.tolerance_deg / (control.degrees_per_mm if joint == "J1" else 1),
                "position_unit": "mm" if joint == "J1" else "°",
                "speed_unit": "mm/s" if joint == "J1" else "°/s",
                "acceleration_unit": "mm/s²" if joint == "J1" else "°/s²", "limits": asdict(limit)}
        for joint, limit in limits.items()
    }
    preview["input_kind"] = "csv" if args.csv is not None else "joints" if args.joints is not None else "TARGET_JOINTS"
    preview["first_pose_requires_enter"] = args.csv is not None
    preview["subsequent_poses_require_enter"] = True
    preview["continue_monitoring_after_last_pose"] = True
    zero_table = single_target_table(dict.fromkeys(DEFAULT_JOINT_NAMES, 0.), model, pose_id="return_to_zero")
    preview["return_to_zero"] = zero_table.preview(model, measurement_kind="positioning", execution_limits=limits)[
        "rows"][0]
    preview["return_to_zero_requires_confirmation"] = True
    preview["return_to_zero_command"] = "r"
    preview["continue_monitoring_after_return_to_zero"] = True
    if args.preview:
        if args.start_readonly is not None:
            from .positioning_plan import synchronized_preview

            start_path = args.start_readonly if args.start_readonly.is_absolute() else root / args.start_readonly
            scene_path = (None if args.scene_config is None else args.scene_config if args.scene_config.is_absolute()
                          else root / args.scene_config)
            with redirect_stdout(sys.stderr):
                preview["synchronized_plan"] = synchronized_preview(
                    root, model, table, limits, start_path, scene_config=scene_path)
            preview["collision_checked"] = preview["synchronized_plan"]["collision_checked"]
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x", encoding="utf-8") as stream:
                json.dump(preview, stream, ensure_ascii=False, indent=2, allow_nan=False)
                stream.write("\n")
        print(json.dumps(preview, ensure_ascii=False, indent=2, allow_nan=False))
        if args.scene_config is not None and not preview["synchronized_plan"]["reference_route_accepted"]:
            return 2
        return 0
    from .attended_positioning import AttendedPositioning
    from .joint_table_runtime import run_joint_table

    policy_path = args.experiment_policy or root / ".scratch/motor_control/attended-positioning.json"
    session = AttendedPositioning.load(root, policy_path, table, model, port=args.port, limits=limits)
    if not Path(args.port).exists():
        raise FileNotFoundError(args.port)

    output = args.output if args.output is not None else root / ".scratch/joint-positioning" / f"{uuid4().hex}.jsonl"
    output.parent.mkdir(parents=True, exist_ok=True)
    executor = JointTableExecutor(table, model, measurement_kind="positioning", task_id="manual-positioning",
                                  feedback_source="hardware_feedback", limits=session.limits, manual_positioning=True)
    operator = PositioningOperator(sys.stdin, output=sys.stdout, start_immediately=args.csv is None)
    print(f"运行记录：{output}", flush=True)
    if args.csv is None:
        print(f"本次目标：{preview['rows'][0]['target_mm_deg']}（J1：mm；J2–J9：°）", flush=True)
    print("有人值守低速实验：请持续保持运动范围内无人。正在核验设备零位配置、实际位置与九轴静止。", flush=True)
    run_joint_table(executor, port=args.port, log_path=output, measurement_dir=output.parent,
                    positioning_operator=operator, attended=session)
    return 0
