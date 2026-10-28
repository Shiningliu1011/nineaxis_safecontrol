from __future__ import annotations

import csv
from dataclasses import dataclass
from hashlib import sha256
import io
import json
from math import isfinite
from pathlib import Path
from typing import NoReturn

import numpy as np

from .hardware_contract import HardwareState
from .drempower_can import CMD_POSITION_ANGLE_MODE1, can_id, encode_trapezoid_position
from .drempower_joint_control import DrEmpowerJointControl
from .j1_execution import J1ScrewControl
from .motor_motion_limits import MAX_SINGLE_JOINT_SPEED_RPM, validate_motion_parameters
from .robot_spec import DEFAULT_JOINT_NAMES
from .single_joint_execution import ExecutionLimits, MotionRejected


TABLE_COLUMNS = ("pose_id", "J1_mm", *(f"{name}_deg" for name in DEFAULT_JOINT_NAMES[1:]))
MEASUREMENT_KINDS = ("camera", "laser_tracker")


def _device_commands(q_mm_deg, linear: J1ScrewControl, rotary: DrEmpowerJointControl) -> list[dict]:
    commands = []
    linear_limits = linear.limits(speed_mm_s=0.25, accel_mm_s2=0.05)
    rotary_limits = ExecutionLimits(-90, 90, speed_rpm=0.41)
    for index, joint in enumerate(DEFAULT_JOINT_NAMES):
        control, limits = (linear, linear_limits) if index == 0 else (rotary, rotary_limits)
        target = q_mm_deg[index] * linear.degrees_per_mm if index == 0 else q_mm_deg[index]
        validate_motion_parameters(limits.speed_rpm, limits.accel_rpm_s, axis_type=limits.axis_type)
        arguments = control.position_arguments(joint, target, speed_rpm=limits.speed_rpm,
                                               accel_rpm_s=limits.accel_rpm_s)
        payload = encode_trapezoid_position(arguments["id_num"], arguments["angle"],
                                            speed=arguments["speed"], accel=arguments["param"])
        commands.append(dict(joint=joint, arguments=arguments,
                             frame_id=can_id(arguments["id_num"], CMD_POSITION_ANGLE_MODE1),
                             payload_hex=payload.hex()))
    return commands


@dataclass(frozen=True)
class JointTableRow:
    pose_id: str
    q_si: tuple[float, ...]


@dataclass(frozen=True)
class JointTable:
    rows: tuple[JointTableRow, ...]
    source_sha256: str
    model_hash: str

    @classmethod
    def load(cls, path: Path, model) -> JointTable:
        from work.robot_units import joints_mm_deg_to_si

        if tuple(model.joint_names) != DEFAULT_JOINT_NAMES:
            raise ValueError("姿态表需要 J1–J9 的共同模型")
        rows = []
        identities = set()
        source = path.read_bytes()
        with io.StringIO(source.decode("utf-8-sig"), newline="") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames != list(TABLE_COLUMNS):
                raise ValueError(f"CSV 列顺序必须为 {','.join(TABLE_COLUMNS)}")
            for number, row in enumerate(reader, 2):
                if set(row) != set(TABLE_COLUMNS) or any(value is None for value in row.values()):
                    raise ValueError(f"CSV 第 {number} 行列数无效")
                pose_id = row["pose_id"].strip()
                if not pose_id or pose_id in identities:
                    raise ValueError(f"CSV 第 {number} 行 pose_id 为空或重复")
                q = joints_mm_deg_to_si([float(row[name]) for name in TABLE_COLUMNS[1:]])
                if (not np.all(np.isfinite(q)) or np.any(q < model.position_lower)
                        or np.any(q > model.position_upper)):
                    raise ValueError(f"CSV 第 {number} 行目标超出共同模型位置范围")
                rows.append(JointTableRow(pose_id, tuple(q)))
                identities.add(pose_id)
        if not rows:
            raise ValueError("姿态表至少需要一组目标")
        return cls(tuple(rows), sha256(source).hexdigest(), model.model_hash)

    def preview(self, model, *, measurement_kind: str) -> dict:
        from work.robot_units import joints_si_to_mm_deg

        if measurement_kind not in MEASUREMENT_KINDS:
            raise ValueError("measurement_kind 必须为 camera 或 laser_tracker")
        if self.model_hash != model.model_hash:
            raise ValueError("姿态表 model_hash 与当前模型不符")
        if not self.rows or len({row.pose_id for row in self.rows}) != len(self.rows):
            raise ValueError("姿态表需要非空且唯一的 pose_id")
        for row in self.rows:
            q = np.asarray(row.q_si)
            if (not row.pose_id.strip() or q.shape != (9,) or not np.all(np.isfinite(q))
                    or np.any(q < model.position_lower) or np.any(q > model.position_upper)):
                raise ValueError(f"姿态表目标无效：{row.pose_id}")
        linear = J1ScrewControl.from_root(model.root)
        calibration_path = model.root / "config/hardware_joint_zero.yaml"
        rotary = DrEmpowerJointControl(calibration_path, linear.converter.node_map)
        return dict(
            status="targets_validated", execution="not_started", hardware_executable=False,
            collision_checked=False, measurement_kind=measurement_kind,
            position_mode="absolute_trapezoid", parameter_scope="existing_low_speed_checks",
            zero_calibration_sha256=sha256(calibration_path.read_bytes()).hexdigest(),
            transmission_sha256=sha256((model.root / "config/j1_transmission.yaml").read_bytes()).hexdigest(),
            node_mapping_sha256=sha256((model.root / "config/drempower.yaml").read_bytes()).hexdigest(),
            source_sha256=self.source_sha256, model_hash=self.model_hash,
            joint_names=list(DEFAULT_JOINT_NAMES), columns=list(TABLE_COLUMNS),
            rows=[dict(pose_id=row.pose_id, target_mm_deg=joints_si_to_mm_deg(row.q_si).tolist(),
                       target_si=list(row.q_si), nominal_tcp_matrix_m=model.fk(row.q_si).tolist(),
                       device_commands=_device_commands(joints_si_to_mm_deg(row.q_si), linear, rotary))
                  for row in self.rows],
        )


class JointTableProgress:
    def __init__(self, table: JointTable, model, root: Path, *, measurement_kind: str,
                 feedback_source: str):
        if feedback_source != "simulation_feedback":
            raise RuntimeError("joint_table_live_disabled: 多轴真实反馈与停车资格尚未完成验证")
        table.preview(model, measurement_kind=measurement_kind)
        linear = J1ScrewControl.from_root(root)
        limits = linear.limits(speed_mm_s=0.25, accel_mm_s2=0.05)
        rotary = ExecutionLimits(-90, 90)
        self.position_tolerance = np.array([
            limits.tolerance_deg / linear.degrees_per_mm / 1000,
            *([np.deg2rad(rotary.tolerance_deg)] * 8),
        ])
        self.stopped_speed = np.array([
            limits.stopped_rpm * 6 / linear.degrees_per_mm / 1000,
            *([np.deg2rad(rotary.stopped_rpm * 6)] * 8),
        ])
        self.maximum_speed = np.array([
            limits.max_feedback_rpm * 6 / linear.degrees_per_mm / 1000,
            *([np.deg2rad(MAX_SINGLE_JOINT_SPEED_RPM * 6)] * 8),
        ])
        self.table, self.model = table, model
        self.measurement_kind = measurement_kind
        self.index = 0
        self.phase = "ready"
        self.last_stamp_s: float | None = None
        self.settled_since_s: float | None = None
        self.last_position: np.ndarray | None = None
        self.records: list[dict] = []

    def begin(self) -> JointTableRow:
        if self.phase != "ready":
            raise RuntimeError(f"当前阶段 {self.phase} 不允许开始下一组")
        self.phase = "moving"
        self.settled_since_s = None
        return self.table.rows[self.index]

    def _reject(self, reason: str) -> NoReturn:
        self.phase = "aborted"
        raise MotionRejected(reason)

    def observe(self, state: HardwareState, *, now_s: float, model_hash: str) -> bool:
        if self.phase not in ("ready", "moving", "awaiting_measurement"):
            raise RuntimeError(f"当前阶段 {self.phase} 不接受到位反馈")
        if (model_hash != self.table.model_hash or state.mode != "sim"
                or not state.feedback_ok or not state.watchdog_ok or state.estop_active or state.fault_code):
            self._reject("joint_table_feedback_untrusted")
        if (state.q.shape != (9,) or state.qdot.shape != (9,)
                or not np.all(np.isfinite(state.q)) or not np.all(np.isfinite(state.qdot))):
            self._reject("joint_table_feedback_invalid")
        if (not isfinite(now_s) or not isfinite(state.stamp_s)
                or not 0 <= state.stamp_s <= now_s or now_s - state.stamp_s > 0.2
                or (self.last_stamp_s is not None
                    and not 0 < state.stamp_s - self.last_stamp_s <= 0.2)):
            self._reject("joint_table_feedback_time_invalid")
        if np.any(state.q < self.model.position_lower) or np.any(state.q > self.model.position_upper):
            self._reject("joint_table_joint_limit")
        if np.any(np.abs(state.qdot) > self.maximum_speed):
            self._reject("joint_table_velocity_limit")
        if self.last_stamp_s is not None and self.last_position is not None:
            allowed = self.maximum_speed * (state.stamp_s - self.last_stamp_s) + self.position_tolerance
            if np.any(np.abs(state.q - self.last_position) > allowed):
                self._reject("joint_table_feedback_position_jump")
        self.last_stamp_s = state.stamp_s
        self.last_position = state.q.copy()
        if self.phase == "ready":
            if self.index > 0:
                previous = np.asarray(self.table.rows[self.index - 1].q_si)
                if (np.any(np.abs(state.q - previous) > self.position_tolerance)
                        or np.any(np.abs(state.qdot) > self.stopped_speed)):
                    self._reject("joint_table_pose_changed_between_rows")
            return False
        target = np.asarray(self.table.rows[self.index].q_si)
        settled = (np.all(np.abs(state.q - target) <= self.position_tolerance)
                   and np.all(np.abs(state.qdot) <= self.stopped_speed))
        if not settled:
            if self.phase == "awaiting_measurement":
                self._reject("joint_table_pose_changed_during_measurement")
            self.settled_since_s = None
            return False
        if self.settled_since_s is None:
            self.settled_since_s = state.stamp_s
        if state.stamp_s - self.settled_since_s >= 0.5:
            self.phase = "awaiting_measurement"
            return True
        return False

    def record_measurement(self, files: list[Path], *, now_s: float) -> dict:
        if self.phase != "awaiting_measurement":
            raise RuntimeError("全部关节持续到位后才能记录本组采集文件")
        if (not isfinite(now_s) or self.last_stamp_s is None
                or not 0 <= now_s - self.last_stamp_s <= 0.2):
            self._reject("joint_table_feedback_timeout")
        if self.last_position is None:
            self._reject("joint_table_feedback_missing")
        if not files or any(not path.is_file() or path.stat().st_size == 0 for path in files):
            raise ValueError("本次标定需要已经保存且非空的采集文件")
        record = dict(
            pose_id=self.table.rows[self.index].pose_id, measurement_kind=self.measurement_kind,
            model_hash=self.table.model_hash, table_sha256=self.table.source_sha256,
            feedback_source="simulation_feedback", feedback_stamp_s=self.last_stamp_s,
            feedback_q_si=self.last_position.tolist(),
            files=[dict(path=str(path.resolve()), sha256=sha256(path.read_bytes()).hexdigest()) for path in files],
        )
        self.records.append(record)
        self.index += 1
        self.phase = "completed" if self.index == len(self.table.rows) else "ready"
        return record

    def write_report(self, path: Path) -> None:
        with path.open("x", encoding="utf-8") as stream:
            json.dump(dict(
                measurement_kind=self.measurement_kind, phase=self.phase,
                model_hash=self.table.model_hash, table_sha256=self.table.source_sha256,
                feedback_source="simulation_feedback", hardware_executable=False,
                joint_names=list(DEFAULT_JOINT_NAMES), records=self.records,
            ), stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
