from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RecoveryRules:
    mode: str
    pause_position_m: float = .001
    resume_position_m: float = .0003
    pause_angle_rad: float = np.deg2rad(.2)
    resume_angle_rad: float = np.deg2rad(.04)
    confirmation_s: float = .2
    timeout_s: float = 30.
    feed_acceleration_m_s2: float = .03

    def __post_init__(self):
        values = [value for name, value in vars(self).items() if name != "mode"]
        if self.mode not in ("basic", "improved") or not np.all(np.isfinite(values)) or min(values) <= 0:
            raise ValueError("恢复规则参数无效")
        if (self.resume_position_m >= self.pause_position_m
                or self.resume_angle_rad >= self.pause_angle_rad
                or self.confirmation_s < .2 or self.timeout_s > 30):
            raise ValueError("恢复阈值与持续时间不满足实验要求")


class RecoveryState:
    def __init__(self, rules: RecoveryRules):
        self.rules = rules
        self.phase = "REFERENCE_PAUSED"
        self.reason = "initial_alignment"
        self.pause_started_s: float | None = None
        self.valid_since_s: float | None = None
        self.first_positive_s: float | None = None
        self.last_time_s: float | None = None
        self.events: list[dict] = []

    def fault(self, time_s: float, reason: str):
        self._transition("FAULT", time_s, reason)

    def admission_rejected(self, time_s: float, reason: str):
        if self.phase in ("FAULT", "ENDPOINT_HOLD"):
            return
        if self.pause_started_s is None:
            self.pause_started_s = time_s
        self.valid_since_s, self.first_positive_s = None, None
        self._transition("REFERENCE_PAUSED", time_s, reason)

    def _transition(self, phase: str, time_s: float, reason: str):
        if self.phase != phase:
            event = {"from": self.phase, "to": phase, "time_s": time_s, "reason": reason}
            if phase == "RUNNING" and self.first_positive_s is not None:
                event["recovery_delay_s"] = time_s - self.first_positive_s
            self.events.append(event)
        self.phase, self.reason = phase, reason

    def update(
            self, *, time_s: float, position_error_m: float, angle_rad: float,
            positive_candidate: bool, endpoint: bool, nominal_feed: float,
            executed_feed: float, period_s: float) -> float:
        values = [time_s, position_error_m, angle_rad, nominal_feed, executed_feed, period_s]
        if not np.all(np.isfinite(values)) or min(values) < 0 or period_s == 0:
            raise ValueError("恢复状态输入无效")
        if self.last_time_s is not None and time_s < self.last_time_s:
            raise ValueError("恢复状态时间不能递减")
        self.last_time_s = time_s
        if self.phase == "FAULT":
            return 0.
        if endpoint:
            self._transition("ENDPOINT_HOLD", time_s, "endpoint")
            return 0.
        excessive = position_error_m > self.rules.pause_position_m or angle_rad > self.rules.pause_angle_rad
        if self.phase == "RUNNING" and (excessive or not positive_candidate):
            self.pause_started_s, self.valid_since_s, self.first_positive_s = time_s, None, None
            self._transition("REFERENCE_PAUSED", time_s, "tracking_error" if excessive else "candidate_rejected")
        if self.phase in ("REFERENCE_PAUSED", "RECOVERING"):
            if self.pause_started_s is None:
                self.pause_started_s = time_s
            if time_s - self.pause_started_s > self.rules.timeout_s:
                self.fault(time_s, "pause_timeout")
                return 0.
            if positive_candidate and self.first_positive_s is None:
                self.first_positive_s = time_s
            ready = positive_candidate and not excessive
            if self.rules.mode == "improved":
                ready = (ready and position_error_m <= self.rules.resume_position_m
                         and angle_rad <= self.rules.resume_angle_rad)
            if ready:
                if self.valid_since_s is None:
                    self.valid_since_s = time_s
                confirmed = self.rules.mode == "basic" or time_s - self.valid_since_s >= self.rules.confirmation_s
                if confirmed:
                    self._transition("RUNNING", time_s, "forward_candidate_confirmed")
                    self.pause_started_s = None
                else:
                    self._transition("RECOVERING", time_s, "confirming")
            else:
                self.valid_since_s = None
                self._transition("REFERENCE_PAUSED", time_s, "awaiting_forward_candidate")
        if self.phase != "RUNNING":
            return 0.
        return min(nominal_feed, executed_feed + self.rules.feed_acceleration_m_s2 * period_s)
