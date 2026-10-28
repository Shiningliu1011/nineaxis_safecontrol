from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
import json
import sys
import time

import jax
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .oscbf_trajectory import bootstrap_portable
from .robot_spec import DEFAULT_JOINT_NAMES
from .ros_conventions import JOINT_STATE_TOPIC, command_stream_qos, state_stream_qos
from .static_experiment_capacity import RecordingBudget, RecordingLimits
from .static_cycle_timing import CycleTimings


STATE_TOPIC = "/static_experiment/state"
COMMAND_TOPIC = "/static_experiment/command"
ADMISSION_TOPIC = "/static_experiment/admission"


def _resources(config):
    bootstrap_portable(config.resource_root / "portable_oscbf")
    from work.static_collision import StaticGeometry
    from work.static_motion import MotionPlanner

    jax.config.update("jax_enable_x64", True)
    limits = config.execution_limits()
    plant = config.plant_parameters()
    planner = MotionPlanner(
        limits["lower"], limits["upper"], limits["max_velocity"], limits["max_acceleration"],
        max_jerk=limits["max_jerk"], minimum_duration_s=plant["minimum_duration_s"],
    )
    return config.load_path()[0], StaticGeometry(config.scene()), planner


class StaticExperimentPlant(Node):
    def __init__(self, config, recorder):
        super().__init__("static_experiment_plant")
        from work.static_execution import ExecutionSchedule
        from work.static_motion import ActuatorState, certify_motion

        self.config, self.recorder = config, recorder
        self.recording_budget = RecordingBudget(RecordingLimits(
            config.values["timeout_s"], config.values["settle_s"], config.values["control"]["period_s"],
        ), capacity_check=recorder.check_capacity)
        self._recording_finished = False
        self.path, self.geometry, self.planner = _resources(config)
        self.schedule = ExecutionSchedule(
            ActuatorState(config.initial_position, np.zeros(9), np.zeros(9)),
            self.planner, config.values["control"]["period_s"],
        )
        initial_certificate = certify_motion(self.schedule.active.motion, self.geometry, self.planner)
        if not initial_certificate.accepted:
            raise ValueError(f"初始状态无法满足固定场景限制：{initial_certificate}")
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="static_admission")
        self._pending_check = None
        self._state_pub = self.create_publisher(JointState, JOINT_STATE_TOPIC, state_stream_qos())
        self._feedback_pub = self.create_publisher(String, STATE_TOPIC, state_stream_qos())
        self._admission_pub = self.create_publisher(String, ADMISSION_TOPIC, command_stream_qos())
        self.create_subscription(String, COMMAND_TOPIC, self._on_command, command_stream_qos())
        self.create_timer(.002, self._tick)

    def _admission(self, command_id, reason, received_s, certificate=None):
        result = {
            "command_id": command_id, "accepted": reason == "accepted", "reason": reason,
            "step": self.schedule.step, "received_monotonic_s": received_s,
            "completed_monotonic_s": time.monotonic(),
            "certificate": None if certificate is None else asdict(certificate),
        }
        self.recorder.append(
            "admission", monotonic_s=time.monotonic(),
            simulation_s=self.schedule.step * .002, values=result,
        )
        self._admission_pub.publish(String(data=json.dumps(result, allow_nan=False)))

    def _on_command(self, message):
        from work.static_execution import ScheduledMotion
        from work.static_motion import certify_motion

        self.recording_budget.begin("received_command")
        received_s = time.monotonic()
        request = json.loads(message.data)
        if request["config_sha256"] != self.config.identity:
            raise ValueError("候选运动的场景身份不一致")
        command = ScheduledMotion.from_record(request["command"], self.planner)
        self.recorder.append(
            "received_command", monotonic_s=received_s,
            simulation_s=self.schedule.step * .002, values=request,
        )
        if self._pending_check is not None or self.schedule.pending is not None:
            self._admission(command.command_id, "admission_busy", received_s)
            return
        if command.reference_start_m + command.feed_m_s * command.motion.period_s > self.path.total_length_m + 1e-12:
            self._admission(command.command_id, "path_interval", received_s)
            return
        future = self._executor.submit(certify_motion, command.motion, self.geometry, self.planner)
        self._pending_check = command, future, received_s

    def _tick(self):
        self.recording_budget.check()
        if self._pending_check is not None:
            command, future, received_s = self._pending_check
            if future.done():
                certificate = future.result()
                reason = self.schedule.queue(command, certificate)
                self._admission(command.command_id, reason, received_s, certificate)
                self._pending_check = None
        self.recording_budget.begin("feedback")
        snapshot = self.schedule.advance()
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(DEFAULT_JOINT_NAMES)
        message.position = snapshot["position"]
        message.velocity = snapshot["velocity"]
        now = time.monotonic()
        recording_s = self.recorder.append(
            "feedback", monotonic_s=now, simulation_s=snapshot["simulation_s"],
            ros_time_ns=message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec,
            values={
                "q": snapshot["position"], "velocity": snapshot["velocity"],
                "acceleration": snapshot["acceleration"], "step": snapshot["step"],
                "reference_progress_m": snapshot["reference_progress_m"],
                "phase": snapshot["phase"], "active_command_id": snapshot["active_command_id"],
            },
        )
        if recording_s > .02:
            self.recorder.append("recording_delay", monotonic_s=time.monotonic(), values={
                "step": snapshot["step"], "duration_s": recording_s,
            })
        self._state_pub.publish(message)
        self._feedback_pub.publish(String(data=json.dumps({
            **snapshot, "published_monotonic_s": now, "config_sha256": self.config.identity,
        }, allow_nan=False)))

    @property
    def stopped(self):
        state = self.schedule.snapshot()
        return (self.schedule.pending is None and self._pending_check is None
                and np.max(np.abs(state["velocity"])) <= 1e-10
                and np.max(np.abs(state["acceleration"])) <= 1e-10)

    def finish_recording(self, reason=None):
        if self._recording_finished:
            return
        self._recording_finished = True
        self._executor.shutdown(wait=True)
        if self._pending_check is not None:
            command, future, received_s = self._pending_check
            self._pending_check = None
            try:
                certificate = future.result()
            except BaseException:
                self._admission(command.command_id, "admission_failure", received_s)
                raise
            self._admission(
                command.command_id, reason or "terminated_before_admission",
                received_s, certificate,
            )

    def destroy_node(self):
        self.finish_recording()
        return super().destroy_node()


class StaticExperimentController(Node):
    def __init__(self, config, recorder):
        super().__init__("static_experiment_controller")
        from work.static_control import StaticControl
        from work.static_motion import ActuatorState

        self.config, self.recorder = config, recorder
        self.recording_budget = RecordingBudget(RecordingLimits(
            config.values["timeout_s"], config.values["settle_s"], config.values["control"]["period_s"],
        ), capacity_check=self._check_recording_capacity)
        self._recording_finished = False
        self.path, geometry, self.planner = _resources(config)
        self.settings = config.values["control"]
        self.control = StaticControl(self.path, geometry, self.planner, self.settings)
        self.control.warmup(ActuatorState(config.initial_position, np.zeros(9), np.zeros(9)))
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="static_control")
        self._calculation = None
        self.latest = None
        self._waiting_id = None
        self._last_calculation_start_step = None
        self._sequence = 0
        self._cycle_started = {}
        self.cycle_timings = CycleTimings(
            maximum_records=self.recording_budget.limits["maximum_candidate_records"],
        )
        self._event_count = 0
        self._endpoint_since = None
        self._execution_state = "tracking"
        self._termination = None
        self._last_telemetry = time.monotonic()
        self._publisher = self.create_publisher(String, COMMAND_TOPIC, command_stream_qos())
        self.create_subscription(String, STATE_TOPIC, self._on_state, state_stream_qos())
        self.create_subscription(String, ADMISSION_TOPIC, self._on_admission, command_stream_qos())
        self.create_timer(.002, self._tick)

    def _check_recording_capacity(self):
        self.recorder.check_capacity(pending_cycles=len(self._cycle_started))

    def _on_state(self, message):
        value = json.loads(message.data)
        if value["config_sha256"] != self.config.identity:
            raise ValueError("plant 反馈的场景身份不一致")
        if self.latest is not None and value["step"] <= self.latest["step"]:
            raise ValueError("plant 反馈积分步没有递增")
        self.recording_budget.start(value["published_monotonic_s"])
        self.latest = value

    def _on_admission(self, message):
        value = json.loads(message.data)
        command_id = value["command_id"]
        started = self._cycle_started.pop(command_id)
        self.recorder.append(
            "admission", monotonic_s=time.monotonic(), values=value,
        )
        sequence = self._record_cycle(started, command_id, value["accepted"], value["reason"])
        if not value["accepted"]:
            self._waiting_id = None
            self.control.recovery.admission_rejected(value["step"] * .002, value["reason"])
            self._record_transitions()
        self.cycle_timings.complete(sequence)

    def _record_cycle(self, started, command_id, accepted, reason):
        completed = time.monotonic()
        self.recorder.append("cycle", monotonic_s=completed, values={
            "command_id": command_id, "duration_s": completed - started,
            "accepted": accepted, "reason": reason, "period_s": self.settings["period_s"],
        })
        sequence = self.recorder.last_sequence
        self.cycle_timings.record(sequence, started)
        return sequence

    def _record_transitions(self):
        for event in self.control.recovery.events[self._event_count:]:
            self.recorder.append("transition", monotonic_s=time.monotonic(), values=event)
        self._event_count = len(self.control.recovery.events)

    def record_fault(self, reason):
        simulation_s = self.latest["simulation_s"] if self.latest is not None else 0.
        self.control.recovery.fault(simulation_s, reason)
        self._record_transitions()

    @property
    def execution_state(self):
        if self._execution_state == "holding" and self._cycle_started:
            return "finishing"
        return self._execution_state

    def progress_snapshot(self):
        return {"termination": self._termination, "steps": self._sequence}

    def _tick(self):
        callback_started = time.monotonic()
        from work.static_execution import ScheduledMotion

        self.recording_budget.check()
        if any(time.monotonic() - started > 1. for started in self._cycle_started.values()):
            raise RuntimeError("plant 运动准入回复超过 1 s")
        if self.latest is None or self._execution_state != "tracking":
            return
        snapshot = self.latest
        feedback_age = time.monotonic() - snapshot["published_monotonic_s"]
        if feedback_age > .2:
            raise RuntimeError(f"实验反馈超过 200 ms：step={snapshot['step']}，age_s={feedback_age}")
        if self._calculation is not None:
            future, context = self._calculation
            if future.done():
                self._calculation = None
                self._finish_calculation(future, **context)
            return
        if self._waiting_id is not None:
            if snapshot["active_command_id"] != self._waiting_id:
                return
            self._waiting_id = None
        if snapshot["pending_command_id"] is not None:
            return
        active = ScheduledMotion.from_record(snapshot["active_motion"], self.planner)
        ticks = round(self.settings["period_s"] / .002)
        start_step = active.next_control_step(
            snapshot["step"], previous_start_step=self._last_calculation_start_step,
        )
        if start_step is None:
            return
        initial = active.state_at_step(start_step)
        progress = active.reference_at_step(start_step)
        executed_feed = active.feed_m_s if start_step == active.start_step + ticks else 0.
        self.recording_budget.begin("calculation")
        started = callback_started
        future = self._executor.submit(
            self.control.propose, initial, progress,
            time_s=start_step * .002, executed_feed=executed_feed,
        )
        self._last_calculation_start_step = start_step
        self._calculation = future, {
            "snapshot": snapshot, "initial": initial, "start_step": start_step,
            "progress": progress, "started": started,
        }

    def _finish_calculation(
            self, future, *, snapshot, initial, start_step, progress, started, stop_reason=None):
        from work.static_execution import ScheduledMotion
        from work.static_velocity_solver import SolverFailure

        error = future.exception()
        if error is not None:
            if isinstance(error, SolverFailure):
                self.recorder.append(f"{error.solver}_failure", monotonic_s=time.monotonic(), values={
                    "problem": {key: value.tolist() for key, value in error.problem.items()},
                    "reason": str(error),
                })
            self._record_cycle(started, None, False, "calculation_failure")
        candidate = future.result()
        terms = self.control.last_terms
        record = {
            "initial": initial.to_record(), "start_step": start_step,
            "source_feedback_step": snapshot["step"], "reference_progress_m": progress,
            "reference_position_m": np.asarray(terms.position_m).tolist(),
            "reference_axis": np.asarray(terms.rotation)[:, 0].tolist(),
            "position_error_m": float(np.linalg.norm(terms.error[:3])),
            "tool_axis_angle_rad": float(terms.angle_rad),
            "geometry": [row.to_record() for row in self.control.last_measurements],
            "trials": self.control.last_trials,
        }
        self.recorder.append("candidate", monotonic_s=time.monotonic(), values=record)
        self._record_transitions()
        if stop_reason is not None:
            self.recorder.append("rejection", monotonic_s=time.monotonic(), values={
                "reason": stop_reason, "start_step": start_step,
            })
            self._record_cycle(started, None, False, stop_reason)
            return
        if self.control.recovery.phase == "FAULT":
            sequence = self._record_cycle(started, None, False, self.control.recovery.reason)
            self._execution_state, self._termination = "holding", self.control.recovery.reason
            self.cycle_timings.complete(sequence)
            return
        if candidate is None:
            self.recorder.append("rejection", monotonic_s=time.monotonic(), values={
                "reason": "no_certified_candidate", "start_step": start_step,
            })
            sequence = self._record_cycle(started, None, False, "no_certified_candidate")
            self.cycle_timings.complete(sequence)
            return
        self._sequence += 1
        command = ScheduledMotion(
            self._sequence, start_step, progress, candidate.solution.feed_m_s, candidate.phase, candidate.motion,
        )
        self.recorder.append("command", monotonic_s=time.monotonic(), values={
            "command_id": command.command_id, "scheduled": command.to_record(),
            "reference_progress_m": progress, "reference_position_m": record["reference_position_m"],
            "reference_axis": record["reference_axis"], "at_endpoint": candidate.phase == "ENDPOINT_HOLD",
            "feedrate_m_s": candidate.solution.feed_m_s, "qp_ok": True,
            "q": candidate.motion.at_time(candidate.motion.period_s).position.tolist(),
        })
        self._cycle_started[command.command_id] = started
        self._waiting_id = command.command_id
        self._publisher.publish(String(data=json.dumps({
            "config_sha256": self.config.identity, "command": command.to_record(),
        }, allow_nan=False)))
        endpoint_valid = False
        if snapshot["reference_progress_m"] >= self.path.total_length_m:
            actual_position, actual_rotation = self.control.geometry.kinematics.ee_pose(np.array(snapshot["position"]))
            desired_axis = self.path.rotations[-1, :, 0]
            actual_axis = actual_rotation[:, 0]
            actual_angle = np.arctan2(np.linalg.norm(np.cross(actual_axis, desired_axis)), actual_axis @ desired_axis)
            endpoint_valid = (np.linalg.norm(actual_position - self.path.positions_m[-1]) <= .0005
                              and actual_angle <= np.deg2rad(.5))
        if endpoint_valid:
            if self._endpoint_since is None:
                self._endpoint_since = snapshot["simulation_s"]
            if snapshot["simulation_s"] - self._endpoint_since >= self.config.values["endpoint_hold_s"]:
                self._execution_state, self._termination = "holding", "completed"
        else:
            self._endpoint_since = None
        if time.monotonic() - self._last_telemetry >= 2:
            self.get_logger().info(
                f"progress={progress / self.path.total_length_m:.1%} "
                f"phase={candidate.phase} feed={candidate.solution.feed_m_s:.6f}m/s "
                f"position_error={record['position_error_m'] * 1000:.4f}mm "
                f"axis_error={np.rad2deg(record['tool_axis_angle_rad']):.5f}deg"
            )
            self._last_telemetry = time.monotonic()

    def finish_recording(self, reason=None):
        if self._recording_finished:
            return
        self._recording_finished = True
        self._executor.shutdown(wait=True)
        try:
            if self._calculation is not None:
                future, context = self._calculation
                self._calculation = None
                self._finish_calculation(
                    future, **context, stop_reason=reason or "terminated_during_calculation",
                )
        finally:
            for command_id, started in self._cycle_started.items():
                self._record_cycle(started, command_id, False, reason or "terminated_before_admission")
            self._cycle_started.clear()
            fault = reason or ("fault" if sys.exc_info()[1] is not None else None)
            if fault is not None:
                self.record_fault(fault)

    def destroy_node(self):
        self.finish_recording()
        return super().destroy_node()
