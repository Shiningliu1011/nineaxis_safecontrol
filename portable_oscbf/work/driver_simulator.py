from __future__ import annotations

import numpy as np
from ruckig import InputParameter, OutputParameter, Result, Ruckig


class SCurveDriverSimulator:
    """使用连续的速度、加速度和 jerk 限制跟随位置目标。"""

    def __init__(
        self,
        dq_max: np.ndarray,
        ddq_max: np.ndarray,
        *,
        q_min: np.ndarray,
        q_max: np.ndarray,
        jerk_time: float = 0.08,
        minimum_duration_s: float = 0.08,
        dt: float = 0.01,
    ) -> None:
        self._v_max = np.asarray(dq_max, dtype=float).copy()
        self._a_max = np.asarray(ddq_max, dtype=float).copy()
        self._q_min = np.asarray(q_min, dtype=float).copy()
        self._q_max = np.asarray(q_max, dtype=float).copy()
        if self._v_max.ndim != 1 or self._v_max.size == 0:
            raise ValueError("dq_max must be a nonempty vector")
        for name, value in (
            ("dq_max", self._v_max), ("ddq_max", self._a_max),
            ("q_min", self._q_min), ("q_max", self._q_max),
        ):
            if value.shape != self._v_max.shape or not np.all(np.isfinite(value)):
                raise ValueError(f"{name} must be a finite vector matching dq_max")
        if np.any(self._v_max <= 0) or np.any(self._a_max <= 0):
            raise ValueError("velocity and acceleration limits must be positive")
        if np.any(self._q_min >= self._q_max):
            raise ValueError("q_min must be less than q_max")
        if not np.isfinite(jerk_time) or jerk_time <= 0:
            raise ValueError("jerk_time must be finite and positive")
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive")
        if not np.isfinite(minimum_duration_s) or minimum_duration_s <= 0:
            raise ValueError("minimum_duration_s must be finite and positive")
        self._j_max = self._a_max / jerk_time
        self._otg = Ruckig(self._v_max.size, dt)
        self._input = InputParameter(self._v_max.size)
        self._output = OutputParameter(self._v_max.size)
        self._input.max_velocity = self._v_max.tolist()
        self._input.max_acceleration = self._a_max.tolist()
        self._input.max_jerk = self._j_max.tolist()
        # 连续位置指令共享足够的运动时间，减少每次更新引起的加减速。
        self._input.minimum_duration = minimum_duration_s
        self._initialized = False

    def _positions(self, value: np.ndarray) -> np.ndarray:
        positions = np.asarray(value, dtype=float)
        if positions.shape != self._q_min.shape or not np.all(np.isfinite(positions)):
            raise ValueError("positions must be a finite vector matching joint limits")
        if np.any(positions < self._q_min) or np.any(positions > self._q_max):
            raise ValueError("positions exceed joint limits")
        return positions

    def reset(self, q0: np.ndarray) -> None:
        positions = self._positions(q0)
        self._input.current_position = positions.tolist()
        self._input.current_velocity = np.zeros_like(positions).tolist()
        self._input.current_acceleration = np.zeros_like(positions).tolist()
        self._input.target_position = positions.tolist()
        self._input.target_velocity = np.zeros_like(positions).tolist()
        self._input.target_acceleration = np.zeros_like(positions).tolist()
        self._otg.reset()
        self._initialized = True

    def step(self, target_position: np.ndarray) -> np.ndarray:
        if not self._initialized:
            raise RuntimeError("reset(q0) is required before stepping the driver")
        self._input.target_position = self._positions(target_position).tolist()
        result = self._otg.update(self._input, self._output)
        if result not in (Result.Working, Result.Finished):
            raise RuntimeError(f"actuator trajectory generation failed: {result}")
        positions = np.asarray(self._output.new_position)
        if (not np.all(np.isfinite(positions))
                or np.any(positions < self._q_min - 1e-10)
                or np.any(positions > self._q_max + 1e-10)):
            raise RuntimeError("actuator trajectory exceeds joint position limits")
        # 目标更新时沿用当前速度和加速度，跟踪及保持阶段采用相同限制。
        self._output.pass_to_input(self._input)
        return positions.copy()

    @property
    def velocity(self) -> np.ndarray:
        return np.asarray(self._input.current_velocity).copy()

    @property
    def acceleration(self) -> np.ndarray:
        return np.asarray(self._input.current_acceleration).copy()
