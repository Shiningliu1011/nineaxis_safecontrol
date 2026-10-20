from __future__ import annotations

from dataclasses import dataclass
from functools import cached_property
from collections import defaultdict

import numpy as np
from ruckig import ControlInterface, InputParameter, Result, Ruckig, Trajectory

from work.static_motion_bounds import MotionBounds
from work.static_swept_geometry import transform_derivatives_batch
from work.obb_collision_model import OBB_LINK_NAMES


def _joint_vector(value, name):
    result = np.asarray(value, dtype=float).copy()
    if result.shape != (9,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} 必须为九维有限数组")
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class ActuatorState:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray

    def __post_init__(self):
        for name in ("position", "velocity", "acceleration"):
            object.__setattr__(self, name, _joint_vector(getattr(self, name), name))

    def to_record(self):
        return {name: getattr(self, name).tolist() for name in ("position", "velocity", "acceleration")}

    @classmethod
    def from_record(cls, record):
        return cls(**{name: record[name] for name in ("position", "velocity", "acceleration")})


@dataclass(frozen=True)
class CandidateMotion:
    initial: ActuatorState
    target_velocity: np.ndarray
    period_s: float
    tracking: Trajectory
    stopping: Trajectory

    @property
    def duration_s(self):
        return self.period_s + self.stopping.duration

    @cached_property
    def bounds(self):
        return MotionBounds(self)

    def at_time(self, time_s: float) -> ActuatorState:
        if not np.isfinite(time_s) or time_s < 0:
            raise ValueError("运动时间必须为有限非负值")
        if time_s <= self.period_s:
            values = self.tracking.at_time(time_s)
        else:
            values = self.stopping.at_time(min(time_s - self.period_s, self.stopping.duration))
        return ActuatorState(*values)

    def values_at_times(self, times) -> np.ndarray:
        times = np.asarray(times, dtype=float)
        if times.ndim != 1 or not np.all(np.isfinite(times)) or np.any(times < 0):
            raise ValueError("运动时间批次必须为有限非负数组")
        values = np.asarray([
            self.tracking.at_time(float(value)) if value <= self.period_s else
            self.stopping.at_time(float(min(value - self.period_s, self.stopping.duration)))
            for value in times
        ])
        if values.shape != (len(times), 3, 9) or not np.all(np.isfinite(values)):
            raise ValueError("Ruckig 返回无效的批量状态")
        return values

    def to_record(self):
        return {
            "initial": self.initial.to_record(), "target_velocity": self.target_velocity.tolist(),
            "period_s": self.period_s,
        }


@dataclass(frozen=True)
class MotionCertificate:
    accepted: bool
    reason: str
    pair: tuple[str, str] | None
    checked_intervals: int
    minimum_proven_margin_m: float | None
    covered_pairs: int = 0


class MotionPlanner:
    def __init__(
            self, lower, upper, max_velocity, max_acceleration, *,
            jerk_time: float, minimum_duration_s: float):
        self.lower = _joint_vector(lower, "lower")
        self.upper = _joint_vector(upper, "upper")
        self.max_velocity = _joint_vector(max_velocity, "max_velocity")
        self.max_acceleration = _joint_vector(max_acceleration, "max_acceleration")
        if (np.any(self.lower >= self.upper) or np.any(self.max_velocity <= 0)
                or np.any(self.max_acceleration <= 0)
                or not np.isfinite(jerk_time) or jerk_time <= 0
                or not np.isfinite(minimum_duration_s) or minimum_duration_s <= 0):
            raise ValueError("Ruckig 执行器限制无效")
        self.max_jerk = self.max_acceleration / jerk_time
        self.minimum_duration_s = minimum_duration_s
        self._ruckig = Ruckig(9, .002)

    def _calculate(self, state: ActuatorState, target_velocity) -> Trajectory:
        data = InputParameter(9)
        data.control_interface = ControlInterface.Velocity
        data.current_position = state.position.tolist()
        data.current_velocity = state.velocity.tolist()
        data.current_acceleration = state.acceleration.tolist()
        data.target_velocity = _joint_vector(target_velocity, "target_velocity").tolist()
        data.target_acceleration = [0.] * 9
        data.max_velocity = self.max_velocity.tolist()
        data.max_acceleration = self.max_acceleration.tolist()
        data.max_jerk = self.max_jerk.tolist()
        data.minimum_duration = self.minimum_duration_s
        self._ruckig.validate_input(data, True, True)
        trajectory = Trajectory(9)
        result = self._ruckig.calculate(data, trajectory)
        if result not in (Result.Working, Result.Finished):
            raise RuntimeError(f"Ruckig 无法生成候选运动：{result}")
        return trajectory

    def propose(self, initial: ActuatorState, target_velocity, *, period_s: float) -> CandidateMotion:
        if not np.isfinite(period_s) or period_s <= 0:
            raise ValueError("候选周期必须为有限正数")
        target = _joint_vector(target_velocity, "target_velocity")
        if np.any(np.abs(target) > self.max_velocity):
            raise ValueError("目标速度超出执行器限制")
        tracking = self._calculate(initial, target)
        next_state = ActuatorState(*tracking.at_time(period_s))
        stopping = self._calculate(next_state, np.zeros(9))
        return CandidateMotion(initial, target, period_s, tracking, stopping)

    def from_record(self, record) -> CandidateMotion:
        return self.propose(
            ActuatorState.from_record(record["initial"]), record["target_velocity"],
            period_s=record["period_s"],
        )


def certify_motion(
        motion: CandidateMotion, geometry, planner: MotionPlanner,
        *, minimum_interval_s: float = 1e-5, maximum_intervals: int = 4096) -> MotionCertificate:
    if (not np.isfinite(minimum_interval_s) or minimum_interval_s <= 0
            or not isinstance(maximum_intervals, int) or maximum_intervals <= 0):
        raise ValueError("区间检查参数无效")
    intervals = [(0., motion.period_s, None), (motion.period_s, motion.duration_s, None)]
    checked, minimum_margin = 0, float("inf")
    bounds, coefficients = None, None
    third_bounds = {}
    while intervals:
        groups = defaultdict(list)
        for start, end, indices in intervals:
            groups[None if indices is None else tuple(indices)].append((start, end))
        intervals = []
        for key, batch in groups.items():
            indices = None if key is None else np.asarray(key)
            if checked + len(batch) > maximum_intervals:
                return MotionCertificate(False, "interval_budget", None, checked + len(batch), None)
            endpoints = np.asarray(batch)
            midpoints = endpoints.mean(axis=1)
            states = motion.values_at_times(midpoints)
            if np.any(states[:, 0] < planner.lower) or np.any(states[:, 0] > planner.upper):
                return MotionCertificate(False, "joint_position", None, checked + len(batch), None)
            derivatives = transform_derivatives_batch(states)
            queries = geometry.query_transforms(
                np.stack([derivatives[name][0] for name in OBB_LINK_NAMES], axis=1),
                pair_indices=indices,
            )
            checked += len(batch)
            margins = np.array([
                [row.distance.distance_m - row.required_m for row in rows] for rows in queries
            ])
            invalid = next((row for rows in queries for row in rows if not row.distance.valid), None)
            violated = np.argwhere(margins < 0)
            if invalid is not None or len(violated):
                row = invalid if invalid is not None else queries[violated[0, 0]][violated[0, 1]]
                return MotionCertificate(False, "collision_clearance", row.pair, checked, None)
            if bounds is None:
                bounds = motion.bounds
                if np.any(bounds.lower < planner.lower) or np.any(bounds.upper > planner.upper):
                    return MotionCertificate(False, "joint_position", None, checked, None)
                if (np.any(bounds.max_velocity > planner.max_velocity + 1e-12)
                        or np.any(bounds.max_acceleration > planner.max_acceleration + 1e-12)
                        or np.any(bounds.max_jerk > planner.max_jerk + 1e-12)):
                    return MotionCertificate(False, "actuator_limits", None, checked, None)
                coefficients = np.array([geometry.displacement_coefficients(row.pair) for row in queries[0]])
            if indices is None:
                indices = np.arange(len(queries[0]))
            lower, upper = bounds.position_ranges(endpoints[:, 0], endpoints[:, 1])
            displacement = np.maximum(states[:, 0] - lower, upper - states[:, 0])
            displacement = np.where(displacement > 0, np.nextafter(displacement, np.inf), 0.)
            proven = margins - displacement @ coefficients[indices].T
            uncertain = np.flatnonzero(np.any(proven < 0, axis=0))
            if len(uncertain):
                projected = geometry.swept.lower_bounds_batch(
                    [[rows[index] for index in uncertain] for rows in queries], bounds,
                    (endpoints[:, 1] - endpoints[:, 0]) / 2,
                    derivatives, third_bounds=third_bounds,
                )
                proven[:, uncertain] = np.maximum(proven[:, uncertain], projected)
            accepted = proven >= 0
            if np.any(accepted):
                minimum_margin = min(minimum_margin, float(np.min(proven[accepted])))
            for offset, (start, end) in enumerate(batch):
                if np.all(accepted[offset]):
                    continue
                if end - start <= minimum_interval_s:
                    pair = queries[offset][int(np.argmin(proven[offset]))].pair
                    return MotionCertificate(False, "unproven_interval", pair, checked, None)
                remaining = indices[~accepted[offset]]
                midpoint = midpoints[offset]
                intervals.extend([(start, midpoint, remaining), (midpoint, end, remaining)])
    return MotionCertificate(True, "certified", None, checked, minimum_margin, len(coefficients))
