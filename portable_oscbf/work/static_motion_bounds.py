from __future__ import annotations

import numpy as np
from scipy.interpolate import PPoly


def _polynomial(trajectory, profile, joint: int, horizon: float) -> PPoly:
    knots, coefficients = [0.], []
    parts = ([profile.brake] if profile.brake.duration > 0 else []) + [profile]
    for part in parts:
        for index, duration in enumerate(part.t):
            if not np.isfinite(duration) or duration < 0:
                raise ValueError("Ruckig 分段时间无效")
            if duration == 0:
                continue
            coefficients.append([
                part.j[index] / 6, part.a[index] / 2, part.v[index], part.p[index],
            ])
            knots.append(min(knots[-1] + duration, horizon))
            if knots[-1] == horizon:
                break
        if knots[-1] == horizon:
            break
    if knots[-1] < horizon:
        position, velocity, acceleration = trajectory.at_time(knots[-1])
        coefficients.append([0., acceleration[joint] / 2, velocity[joint], position[joint]])
        knots.append(horizon)
    values = np.asarray(coefficients, dtype=float).T
    if not np.all(np.isfinite(values)):
        raise ValueError("Ruckig 分段系数无效")
    return PPoly(values, knots, extrapolate=False)


class MotionBounds:
    def __init__(self, motion):
        self._motion = motion
        events = [[] for _ in range(9)]
        positions = [[] for _ in range(9)]
        self.max_velocity = np.zeros(9)
        self.max_acceleration = np.zeros(9)
        self.max_jerk = np.zeros(9)
        for trajectory, horizon, offset in (
                (motion.tracking, motion.period_s, 0.),
                (motion.stopping, motion.stopping.duration, motion.period_s)):
            if horizon <= 0:
                raise ValueError("连续运动范围要求有限正轨迹时长")
            profiles = trajectory.profiles
            if len(profiles) != 1 or len(profiles[0]) != 9:
                raise ValueError("连续运动范围要求单段九轴 Ruckig 轨迹")
            joint_times = []
            for joint in range(9):
                polynomial = _polynomial(trajectory, profiles[0][joint], joint, horizon)
                critical = [polynomial.x]
                for order in (1, 2):
                    roots = polynomial.derivative(order).roots(
                        discontinuity=False, extrapolate=False,
                    )
                    # PPoly 用 NaN 标记整段恒零；区间端点已经包含在检查范围内。
                    roots = roots[~np.isnan(roots)]
                    if not np.all(np.isfinite(roots)):
                        raise ValueError("Ruckig 导数根无效")
                    critical.append(roots[(roots >= 0) & (roots <= horizon)])
                joint_times.append(np.unique(np.concatenate(critical)))
                self.max_jerk[joint] = max(
                    self.max_jerk[joint], float(np.max(np.abs(polynomial.c[0] * 6))),
                )
            times, inverse = np.unique(np.concatenate(joint_times), return_inverse=True)
            values = np.asarray([trajectory.at_time(float(value)) for value in times])
            cursor = 0
            for joint, critical in enumerate(joint_times):
                indices = inverse[cursor:cursor + len(critical)]
                cursor += len(critical)
                events[joint].extend(critical + offset)
                positions[joint].extend(values[indices, 0, joint])
                self.max_velocity[joint] = max(
                    self.max_velocity[joint], float(np.max(np.abs(values[indices, 1, joint]))),
                )
                self.max_acceleration[joint] = max(
                    self.max_acceleration[joint], float(np.max(np.abs(values[indices, 2, joint]))),
                )
        self._events = tuple(np.asarray(values) for values in events)
        self._positions = tuple(np.asarray(values) for values in positions)
        self.lower = np.array([np.min(values) for values in self._positions])
        self.upper = np.array([np.max(values) for values in self._positions])
        for values in (
                *self._events, *self._positions, self.lower, self.upper,
                self.max_velocity, self.max_acceleration, self.max_jerk):
            values.setflags(write=False)

    def position_range(self, start: float, end: float) -> tuple[np.ndarray, np.ndarray]:
        lower, upper = self.position_ranges([start], [end])
        return lower[0], upper[0]

    def position_ranges(self, starts, ends) -> tuple[np.ndarray, np.ndarray]:
        starts, ends = np.asarray(starts), np.asarray(ends)
        if (starts.ndim != 1 or starts.shape != ends.shape or not len(starts)
                or not np.all(np.isfinite(starts)) or not np.all(np.isfinite(ends))
                or np.any(starts < 0) or np.any(ends < starts)
                or np.any(ends > self._motion.duration_s)):
            raise ValueError("运动范围查询时间无效")
        values = self._motion.values_at_times(np.r_[starts, ends])[:, 0]
        first, last = values[:len(starts)], values[len(starts):]
        lower, upper = np.minimum(first, last), np.maximum(first, last)
        for joint, (events, positions) in enumerate(zip(self._events, self._positions)):
            selected = (events >= starts[:, None]) & (events <= ends[:, None])
            lower[:, joint] = np.minimum(lower[:, joint], np.where(selected, positions, np.inf).min(axis=1))
            upper[:, joint] = np.maximum(upper[:, joint], np.where(selected, positions, -np.inf).max(axis=1))
        return lower, upper
