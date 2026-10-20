from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from work.static_motion import ActuatorState, CandidateMotion, MotionCertificate, MotionPlanner


@dataclass(frozen=True)
class ScheduledMotion:
    command_id: int
    start_step: int
    reference_start_m: float
    feed_m_s: float
    phase: str
    motion: CandidateMotion

    def __post_init__(self):
        if (not isinstance(self.command_id, int) or self.command_id < 0
                or not isinstance(self.start_step, int) or self.start_step < 0
                or not np.isfinite(self.reference_start_m) or self.reference_start_m < 0
                or not np.isfinite(self.feed_m_s) or self.feed_m_s < 0
                or (self.phase != "RUNNING" and self.feed_m_s != 0.)
                or self.phase not in ("RUNNING", "REFERENCE_PAUSED", "RECOVERING", "ENDPOINT_HOLD")):
            raise ValueError("运动调度参数无效")
        ticks = self.motion.period_s / .002
        if not np.isclose(ticks, round(ticks), atol=1e-10):
            raise ValueError("控制周期必须为 2 ms 的整数倍")

    def state_at_step(self, step: int):
        return self.motion.at_time(max(0, step - self.start_step) * .002)

    def reference_at_step(self, step: int):
        effective_s = min(max(0, step - self.start_step) * .002, self.motion.period_s)
        return self.reference_start_m + self.feed_m_s * effective_s

    def to_record(self):
        return {
            "command_id": self.command_id, "start_step": self.start_step,
            "reference_start_m": self.reference_start_m, "feed_m_s": self.feed_m_s,
            "phase": self.phase, "motion": self.motion.to_record(),
        }

    @classmethod
    def from_record(cls, values, planner: MotionPlanner):
        return cls(
            command_id=values["command_id"], start_step=values["start_step"],
            reference_start_m=values["reference_start_m"], feed_m_s=values["feed_m_s"],
            phase=values["phase"], motion=planner.from_record(values["motion"]),
        )


class ExecutionSchedule:
    def __init__(self, initial: ActuatorState, planner: MotionPlanner, period_s: float):
        self.step = 0
        self.active = ScheduledMotion(
            0, 0, 0., 0., "REFERENCE_PAUSED", planner.propose(initial, np.zeros(9), period_s=period_s),
        )
        self.pending: ScheduledMotion | None = None
        self.last_request_id = 0

    @staticmethod
    def _same_state(first, second):
        return all(np.allclose(getattr(first, name), getattr(second, name), atol=1e-10, rtol=0)
                   for name in ("position", "velocity", "acceleration"))

    def queue(self, command: ScheduledMotion, certificate: MotionCertificate) -> str:
        if command.command_id <= self.last_request_id:
            return "repeated_command"
        self.last_request_id = command.command_id
        if not certificate.accepted:
            return certificate.reason
        if command.start_step <= self.step:
            return "late_command"
        if self.pending is not None:
            return "pending_command"
        if command.motion.period_s != self.active.motion.period_s:
            return "period_mismatch"
        if not self._same_state(command.motion.initial, self.active.state_at_step(command.start_step)):
            return "state_mismatch"
        if abs(command.reference_start_m - self.active.reference_at_step(command.start_step)) > 1e-12:
            return "reference_mismatch"
        self.pending = command
        return "accepted"

    def advance(self) -> dict:
        self.step += 1
        if self.pending is not None and self.step == self.pending.start_step:
            if not self._same_state(self.pending.motion.initial, self.active.state_at_step(self.step)):
                raise RuntimeError("获准运动激活时状态不一致")
            self.active, self.pending = self.pending, None
        return self.snapshot()

    def snapshot(self) -> dict:
        state = self.active.state_at_step(self.step)
        elapsed = (self.step - self.active.start_step) * .002
        phase = (self.active.phase if elapsed <= self.active.motion.period_s
                 or self.active.phase == "ENDPOINT_HOLD" else "REFERENCE_PAUSED")
        return {
            "step": self.step, "simulation_s": self.step * .002, **state.to_record(),
            "reference_progress_m": self.active.reference_at_step(self.step),
            "active_command_id": self.active.command_id, "phase": phase,
            "active_motion": self.active.to_record(),
            "pending_command_id": None if self.pending is None else self.pending.command_id,
        }
