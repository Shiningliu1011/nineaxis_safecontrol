import json
from dataclasses import asdict
from math import degrees, isfinite
from pathlib import Path
from time import monotonic
from typing import NoReturn
from uuid import uuid4

from .drempower_joint_control import DrEmpowerJointControl
from .j1_execution import J1ScrewControl
from .joint_table import JointTable, joint_execution_limits
from .measurement_files import PreparedMeasurementFile, prepare_measurement_files
from .robot_spec import DEFAULT_JOINT_NAMES
from .single_joint_execution import (
    MotionRejected,
    ObservedFeedback,
    SingleJointMonitor,
    VelocityWindow,
    stationary_conditions,
)


class JointTableControl:
    def __init__(self, root: Path):
        self.linear = J1ScrewControl.from_root(root)
        self.rotary = DrEmpowerJointControl(root / "config/hardware_joint_zero.yaml",
                                           self.linear.converter.node_map)

    def control(self, joint):
        return self.linear if joint == "J1" else self.rotary

    def _node(self, joint):
        return self.control(joint)._node(joint)

    def decode_state(self, joint, angle_deg, speed_rpm):
        return self.control(joint).decode_state(joint, angle_deg, speed_rpm)

    def target_deg(self, joint: str, value_si: float) -> float:
        return float(value_si * 1000 * self.linear.degrees_per_mm if joint == "J1"
                     else degrees(value_si))

    def position_arguments(self, joint, target_deg, limits):
        if joint == "J1":
            return self.linear.position_arguments(joint, target_deg, speed_rpm=limits.speed_rpm,
                                                  accel_rpm_s=limits.accel_rpm_s, bounds=limits.bounds)
        return self.control(joint).position_arguments(
            joint, target_deg, speed_rpm=limits.speed_rpm, accel_rpm_s=limits.accel_rpm_s)

    def project_position(self, joint: str, angle_deg: float) -> float:
        return float(angle_deg / self.linear.degrees_per_mm if joint == "J1" else angle_deg)


class JointTableExecutor:
    def __init__(self, table: JointTable, model, *, measurement_kind: str, task_id: str,
                 feedback_source: str, measurement_timeout_s: float = 300.0,
                 limits: dict | None = None):
        self.preview = table.preview(model, measurement_kind=measurement_kind)
        if measurement_kind == "positioning" and len(table.rows) != 1:
            raise ValueError("positioning 仅支持一组绝对目标")
        if not task_id.strip():
            raise ValueError("标定任务必须具有独立 task_id")
        if feedback_source not in ("hardware_feedback", "recorded_device_feedback", "simulation_feedback"):
            raise ValueError("必须明确反馈来源")
        if not isfinite(measurement_timeout_s) or measurement_timeout_s <= 0:
            raise ValueError("测量等待期限必须为有限正数")
        self.table, self.model = table, model
        self.measurement_kind, self.task_id = measurement_kind, task_id
        self.feedback_source = feedback_source
        self.measurement_timeout_s = measurement_timeout_s
        self.measurement_started_s: float | None = None
        self.session_id = str(uuid4())
        self.control = JointTableControl(model.root)
        self.limits = joint_execution_limits(model.root) if limits is None else dict(limits)
        if set(self.limits) != set(DEFAULT_JOINT_NAMES):
            raise ValueError("执行限制必须完整包含九轴")
        # 每轴保持各自的速度与时间历史，开始下一组时继续使用。
        self.windows = {joint: VelocityWindow(limit.velocity_window_s, limit.stationary_window_s)
                        for joint, limit in self.limits.items()}
        self.monitors = {joint: SingleJointMonitor(self.control._node(joint), limit)
                         for joint, limit in self.limits.items()}
        self.phase = "initializing"
        self.index = 0
        self.reason = ""
        self.latest: dict[str, ObservedFeedback] = {}
        self.settled_since_s: float | None = None
        self.started_s: float | None = None
        self.row_started_s: float | None = None
        self.last_now_s: float | None = None
        self.reference: dict[str, float] = {}
        self.commands: list[dict] = []
        self.records: list[dict] = []
        self.attempts: list[dict] = []
        self.used_measurements: set[str] = set()
        self.termination: dict | None = None
        self.rejection_observation: dict | None = None

    def current_attempt(self) -> dict | None:
        if (self.index < len(self.table.rows) and self.attempts
                and self.attempts[-1]["pose_id"] == self.table.rows[self.index].pose_id
                and self.attempts[-1]["status"] != "completed"):
            return self.attempts[-1]
        return None

    def reject(self, reason: str) -> NoReturn:
        if not self.reason:
            self.reason = reason
        self.phase = "aborted"
        attempt = self.current_attempt()
        if attempt is not None:
            attempt.update(status="aborted", rejection_reason=self.reason,
                           stop_result="external_stop_required")
        raise MotionRejected(self.reason)

    def check_time(self, now_s: float):
        if self.phase == "aborted":
            raise MotionRejected(self.reason)
        if not isfinite(now_s) or now_s < 0 or (self.last_now_s is not None and now_s < self.last_now_s):
            self.reject("joint_table_clock_invalid")
        self.last_now_s = now_s
        if self.started_s is None:
            self.started_s = now_s
        for joint, sample in self.latest.items():
            if not 0 <= now_s - sample.request_s <= self.limits[joint].feedback_timeout_s:
                self.reject(f"{joint}:feedback_timeout")
        initial_timeout = max(5., max(limit.stationary_window_s for limit in self.limits.values()) + 2.)
        if self.phase == "initializing" and now_s - self.started_s > initial_timeout:
            self.reject("joint_table_initial_stationary_timeout")
        if (self.phase == "awaiting_measurement" and self.measurement_started_s is not None
                and now_s - self.measurement_started_s > self.measurement_timeout_s):
            self.reject("joint_table_measurement_timeout")
        if self.phase == "moving" and self.row_started_s is not None:
            for joint, monitor in self.monitors.items():
                elapsed = now_s - self.row_started_s
                if monitor.phase == "moving" and elapsed > monitor.limits.motion_timeout_s:
                    self.reject(f"{joint}:motion_timeout")
                if (monitor.phase == "settling" and monitor.position_arrived_s is not None
                        and now_s - monitor.position_arrived_s > monitor.limits.hold_timeout_s):
                    self.reject(f"{joint}:settling_timeout")

    def observe(self, samples: dict[str, ObservedFeedback], *, now_s: float) -> bool:
        if self.phase in ("aborted", "completed", "positioning_completed"):
            raise MotionRejected(self.reason or "joint_table_already_completed")
        # 一轮必须包含九个独立请求的反馈，保留逐轴时间。
        if set(samples) != set(DEFAULT_JOINT_NAMES):
            self.reject("joint_table_incomplete_feedback")
        if (not isfinite(now_s) or now_s < 0
                or (self.last_now_s is not None and now_s < self.last_now_s)):
            self.reject("joint_table_clock_invalid")
        self.last_now_s = now_s
        if self.started_s is None:
            self.started_s = now_s
        current = {}
        try:
            for joint in DEFAULT_JOINT_NAMES:
                monitor = self.monitors[joint]
                sample = self.windows[joint].add(samples[joint])
                if self.phase == "initializing" and sample.window_duration_s < self.limits[joint].velocity_window_s:
                    monitor.validate_hold_feedback(sample, now_s)
                elif self.phase == "moving" and monitor.phase in ("moving", "settling"):
                    monitor.observe(sample, now_s)
                else:
                    monitor.validate_feedback(sample, now_s)
                    if self.phase == "moving" and monitor.phase == "completed" and (
                        abs(sample.state.angle_deg - self.reference[joint]) > self.limits[joint].tolerance_deg
                        or not all(stationary_conditions(sample, self.limits[joint]).values()) or not sample.traj_done
                    ):
                        self.reject(f"{joint}:target_left_settling_tolerance")
                current[joint] = sample
        except MotionRejected as error:
            rejected = samples[joint]
            values = {"angle_deg": rejected.state.angle_deg, "speed_rpm": rejected.state.speed_rpm,
                      "request_s": rejected.request_s, "received_s": rejected.received_s}
            self.rejection_observation = {
                "joint": joint, "reason": str(error), "trusted": False,
                "node_id": rejected.node_id, "target_deg": self.reference.get(joint),
                "values": {name: value if isfinite(value) else None for name, value in values.items()},
                "invalid_fields": [name for name, value in values.items() if not isfinite(value)],
                "traj_done": rejected.traj_done, "axis_error": rejected.axis_error,
            }
            self.latest.update(current)
            self.reject(f"{joint}:{error}")
        self.latest = current
        self.check_time(now_s)
        conditions = {}
        for joint, sample in current.items():
            target = self.reference.get(joint, sample.state.angle_deg)
            limit = self.limits[joint]
            conditions[joint] = {
                "position": bool(abs(sample.state.angle_deg - target) <= limit.tolerance_deg),
                **stationary_conditions(sample, limit),
                "trajectory_done": bool(sample.traj_done),
                "velocity_window": bool(sample.window_duration_s >= limit.velocity_window_s),
            }
        stationary = all(all(value.values()) for value in conditions.values())
        attempt = self.current_attempt()
        if attempt is not None:
            attempt["last_conditions"] = conditions
            attempt["feedback"] = self.feedback_report()
        if not stationary:
            if self.phase in ("ready", "awaiting_measurement"):
                self.reject("joint_table_pose_changed_while_stopped")
            self.settled_since_s = None
            if self.phase == "initializing":
                self.reference = {joint: sample.state.angle_deg for joint, sample in current.items()}
            return False
        if self.settled_since_s is None:
            self.settled_since_s = max(sample.received_s for sample in current.values())
        elapsed = min(sample.request_s for sample in current.values()) - self.settled_since_s
        if elapsed < max(limit.settle_s for limit in self.limits.values()):
            return False
        if self.phase == "initializing":
            self.phase = "ready"
        elif self.phase == "moving":
            if self.row_started_s is None:
                self.reject("joint_table_row_clock_invalid")
            self.phase = "awaiting_measurement"
            self.measurement_started_s = now_s
            self.attempts[-1].update(status=self.phase, arrival_elapsed_s=now_s - self.row_started_s,
                                     feedback_stopped=True, stop_result="all_joint_feedback_stationary",
                                     stop_confirmed_s=now_s)
        return True

    def begin(self, *, now_s: float) -> list[dict]:
        if self.phase != "ready":
            self.reject("joint_table_not_ready")
        row = self.table.rows[self.index]
        reference = {joint: self.control.target_deg(joint, target_si)
                     for joint, target_si in zip(DEFAULT_JOINT_NAMES, row.q_si)}
        self.attempts.append({
            "pose_id": row.pose_id, "target_si": list(row.q_si),
            "target_mm_deg": self.preview["rows"][self.index]["target_mm_deg"],
            "started_s": None, "status": "preparing", "feedback_stopped": False,
            "stop_result": "not_confirmed", "rejection_reason": "", "commands": [],
            "feedback": self.feedback_report(reference=reference),
        })
        self.check_time(now_s)
        self.attempts[-1]["started_s"] = now_s
        if set(self.latest) != set(DEFAULT_JOINT_NAMES):
            self.reject("joint_table_incomplete_feedback")
        commands, monitors = [], {}
        try:
            for joint, target in reference.items():
                sample = self.latest[joint]
                delta = target - sample.state.angle_deg
                limit = self.limits[joint]
                if delta != 0:
                    limit = limit.for_motion(delta)
                monitor = SingleJointMonitor(self.control._node(joint), limit)
                monitor.start_absolute(sample, target, now_s)
                monitors[joint] = monitor
                commands.append({"joint": joint, "arguments": self.control.position_arguments(joint, target, limit)})
        except (MotionRejected, ValueError) as error:
            self.reject(f"{joint}:{error}")
        self.monitors, self.reference, self.commands = monitors, reference, commands
        self.phase, self.row_started_s = "moving", now_s
        self.settled_since_s = None
        self.attempts[-1].update(status="moving", commands=commands, feedback=self.feedback_report())
        return commands

    def feedback_report(self, *, reference: dict[str, float] | None = None):
        targets = self.reference if reference is None else reference
        return {joint: dict(
            **asdict(sample), position_mm_deg=self.control.project_position(joint, sample.state.angle_deg),
            error_mm_deg=self.control.project_position(joint, sample.state.angle_deg - targets[joint])
            if joint in targets else None,
        ) for joint, sample in self.latest.items()}

    def record_measurement(self, files: list[Path], *, now_s: float, session_id: str,
                           task_id: str, pose_id: str, measurement_kind: str) -> dict:
        if self.feedback_source == "hardware_feedback":
            self.reject("joint_table_measurement_requires_async_preparation")
        self.validate_measurement_identity(now_s=now_s, session_id=session_id, task_id=task_id,
                                           pose_id=pose_id, measurement_kind=measurement_kind)
        try:
            prepared = prepare_measurement_files(files)
        except (ValueError, OSError) as error:
            self.reject(f"joint_table_measurement_files_invalid:{error}")
        return self.record_prepared_measurement(
            prepared, now_s=now_s, session_id=session_id, task_id=task_id,
            pose_id=pose_id, measurement_kind=measurement_kind)

    def validate_measurement_identity(self, *, now_s, session_id, task_id, pose_id, measurement_kind):
        self.check_time(now_s)
        if (self.measurement_kind == "positioning" or self.phase != "awaiting_measurement" or session_id != self.session_id
                or task_id != self.task_id or measurement_kind != self.measurement_kind
                or pose_id != self.table.rows[self.index].pose_id):
            self.reject("joint_table_measurement_identity_invalid")

    def record_prepared_measurement(self, files: tuple[PreparedMeasurementFile, ...], *,
                                    now_s: float, session_id: str, task_id: str, pose_id: str,
                                    measurement_kind: str) -> dict:
        self.validate_measurement_identity(now_s=now_s, session_id=session_id, task_id=task_id,
                                           pose_id=pose_id, measurement_kind=measurement_kind)
        if not files:
            self.reject("joint_table_measurement_files_missing")
        try:
            for file in files:
                file.require_unchanged()
        except (ValueError, OSError) as error:
            self.reject(f"joint_table_measurement_files_changed:{error}")
        # 真机接受记录前再次检查当前时效，不能用文件处理前的时间宣告完成。
        recorded_s = monotonic() if self.feedback_source == "hardware_feedback" else now_s
        self.check_time(recorded_s)
        entries = [{"path": str(file.path), "sha256": file.sha256, "bytes": file.size} for file in files]
        identities = [file.sha256 for file in files]
        if (len(set(identities)) != len(entries) or self.used_measurements.intersection(identities)
                or self.table.source_sha256 in identities):
            self.reject("joint_table_measurement_reused")
        self.attempts[-1].update(status="completed", completed_s=recorded_s,
                                 measurement_files=entries, feedback=self.feedback_report())
        record = {
            "session_id": self.session_id, "task_id": self.task_id, "measurement_kind": self.measurement_kind,
            "pose_id": pose_id, "recorded_s": recorded_s, "files": entries,
            "feedback": self.feedback_report(), "execution": dict(self.attempts[-1]),
            "feedback_source": self.feedback_source,
        }
        self.records.append(record)
        self.used_measurements.update(identities)
        self.index += 1
        self.phase = "completed" if self.index == len(self.table.rows) else "ready"
        return record

    def report(self):
        return {
            "session_id": self.session_id, "task_id": self.task_id, "measurement_kind": self.measurement_kind,
            "phase": self.phase, "rejection_reason": self.reason, "model_hash": self.table.model_hash,
            "table_sha256": self.table.source_sha256, "feedback_source": self.feedback_source,
            "measurement_timeout_s": self.measurement_timeout_s,
            "device_sample_time_known": False, "physical_stop_independently_verified": False,
            "acceleration_feedback_validated": False,
            "hardware_qualified": False, "joint_names": list(DEFAULT_JOINT_NAMES),
            "limits": {joint: asdict(limit) for joint, limit in self.limits.items()},
            "zero_calibration_sha256": self.preview["zero_calibration_sha256"],
            "transmission_sha256": self.preview["transmission_sha256"],
            "node_mapping_sha256": self.preview["node_mapping_sha256"],
            "attempts": self.attempts, "records": self.records, "termination": self.termination,
            "rejection_observation": self.rejection_observation,
        }

    def write_report(self, path: Path):
        with path.open("x", encoding="utf-8") as stream:
            json.dump(self.report(), stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
