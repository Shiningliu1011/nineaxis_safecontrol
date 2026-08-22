"""真机安全网关（位置契约）——纯逻辑，无 ROS / 无 I/O。

语义沿袭参考项目 newaxis/hardware_contract.py（影子模式硬件合同），
改写为位置命令契约：

    真实编码器 / 仿真反馈 → HardwareState
    → OSCBF 控制器 / 过渡回放 → PositionCommand
    → CommandSafetyGate（只检验、不重塑）
    → 安全命令（或零速保持命令）

规则：反馈/命令超时、急停、驱动错误码、关节数不匹配、非有限控制量、
速度/相邻命令变化限幅违例 → 输出"零速保持"命令（保持在当前位置），
锁存首个停车原因直到外部监控健康确认（``acknowledge_stop``）。

注意：软件零速 ≠ 物理停止。物理急停（断电/制动）是独立链路，本模块
只负责软件侧的安全合同。``WatchdogConfig.from_actuator_limit_profile``
在硬件模式（``require_hardware_executable=True``）下对未标定关节
（当前即 J1 丝杠传动）fail-closed。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol

import numpy as np


@dataclass(frozen=True)
class HardwareState:
    """关节反馈：位置、速度、时间戳与健康标志。"""

    q: np.ndarray
    qdot: np.ndarray
    stamp_s: float
    feedback_ok: bool
    estop_active: bool = False
    fault_code: str = ""
    mode: str = "shadow"
    watchdog_ok: bool = True

    def __post_init__(self) -> None:
        object.__setattr__(self, "q", np.asarray(self.q, dtype=float).reshape(-1))
        object.__setattr__(self, "qdot", np.asarray(self.qdot, dtype=float).reshape(-1))
        if self.q.shape != self.qdot.shape:
            raise ValueError("q 与 qdot 形状必须一致")


@dataclass(frozen=True)
class PositionCommand:
    """关节位置命令（100 Hz 目标位置）。"""

    q: np.ndarray
    stamp_s: float
    valid_until_s: float
    source: str
    stop_reason: str = ""
    mode: str = "shadow"
    limit_status: str = "within_limits"

    def __post_init__(self) -> None:
        object.__setattr__(self, "q", np.asarray(self.q, dtype=float).reshape(-1))

    @property
    def is_stop(self) -> bool:
        # 正常跟踪命令可以恰为零；只有显式停车原因才是安全停车事件。
        return bool(self.stop_reason)


@dataclass(frozen=True)
class WatchdogConfig:
    """看门狗配置：反馈/命令超时 + 速度与相邻命令变化限幅。"""

    feedback_timeout_s: float
    command_timeout_s: float
    qdot_limit: np.ndarray
    dq_per_command_limit: np.ndarray

    def __post_init__(self) -> None:
        timeout = float(self.feedback_timeout_s)
        command_timeout = float(self.command_timeout_s)
        if timeout <= 0.0 or command_timeout <= 0.0:
            raise ValueError("watchdog 超时必须为正")
        qdot_limit = np.asarray(self.qdot_limit, dtype=float).reshape(-1)
        dq_limit = np.asarray(self.dq_per_command_limit, dtype=float).reshape(-1)
        if qdot_limit.shape != dq_limit.shape or np.any(qdot_limit <= 0.0) \
                or np.any(dq_limit <= 0.0):
            raise ValueError("限幅必须为非零正数且形状对齐")
        object.__setattr__(self, "qdot_limit", qdot_limit)
        object.__setattr__(self, "dq_per_command_limit", dq_limit)

    @classmethod
    def from_actuator_limit_profile(
        cls, profile: "ActuatorLimitProfileLike", *, feedback_timeout_s: float,
        command_timeout_s: float, dq_per_command_limit,
        require_hardware_executable: bool = False,
    ) -> "WatchdogConfig":
        """从单一执行器限幅真源构建看门狗。

        影子模式（``require_hardware_executable=False``）允许记录仿真限幅；
        硬件模式 fail-closed——未标定关节（J1 丝杠传动）直接拒绝构造，
        到不了任何真实驱动路径。
        """
        if require_hardware_executable:
            profile.require_hardware_executable()
        return cls(
            feedback_timeout_s=feedback_timeout_s,
            command_timeout_s=command_timeout_s,
            qdot_limit=np.asarray(profile.velocity_limits, dtype=float),
            dq_per_command_limit=dq_per_command_limit,
        )


class ActuatorLimitProfileLike(Protocol):
    velocity_limits: np.ndarray

    def require_hardware_executable(self) -> None:
        ...


class CommandSafetyGate:
    """只检验、不重塑：限幅违例是停车事件，不是裁剪后的继续运行。

    安全网关不会静默改形 CBF 输出。限幅由控制器在 QP 内强制执行；
    意外违例在这里成为显式受控停车事件（影子/vcan 验证用）。
    """

    def __init__(self, config: WatchdogConfig) -> None:
        self.config = config
        self._last_safe_position: Optional[np.ndarray] = None
        self._latched_stop_reason = ""

    @property
    def latched_stop_reason(self) -> str:
        """首个停车原因；持续到健康确认（仅外部监控调用 acknowledge）。"""
        return self._latched_stop_reason

    def acknowledge_stop(self, state: HardwareState, *, now_s: float) -> bool:
        """仅在外部监控确认健康后清除锁存（本方法不进入控制循环自动执行）。"""
        if not self._latched_stop_reason:
            return False
        if self._state_stop_reason(state, now_s):
            return False
        self._latched_stop_reason = ""
        self._last_safe_position = None
        return True

    def evaluate(
        self, state: HardwareState, requested: PositionCommand, *, now_s: float
    ) -> PositionCommand:
        """返回不变的安全命令，或显式零速保持命令。"""
        now = float(now_s)
        if self._latched_stop_reason:
            return self._stop(state, now, self._latched_stop_reason)
        state_stop_reason = self._state_stop_reason(state, now)
        if state_stop_reason:
            return self._stop(state, now, state_stop_reason)
        if now > requested.valid_until_s or now - requested.stamp_s > self.config.command_timeout_s:
            return self._stop(state, now, "command_timeout")
        if requested.q.shape != self.config.qdot_limit.shape:
            return self._stop(state, now, "command_joint_count_mismatch")
        if not np.all(np.isfinite(requested.q)):
            return self._stop(state, now, "command_not_finite")
        if requested.stop_reason:
            return self._stop(
                state, now, requested.stop_reason, source=requested.source,
                limit_status="requested_stop",
            )
        # 相邻命令变化限幅（首条命令建立基准）。
        if self._last_safe_position is not None and np.any(
            np.abs(requested.q - self._last_safe_position)
            > self.config.dq_per_command_limit
        ):
            return self._stop(state, now, "command_rate_limit")
        safe = requested.q.copy()
        self._last_safe_position = safe
        return PositionCommand(
            q=safe, stamp_s=now,
            valid_until_s=now + self.config.command_timeout_s,
            source=requested.source, mode=requested.mode,
        )

    def _state_stop_reason(self, state: HardwareState, now_s: float) -> str:
        if state.q.shape != self.config.qdot_limit.shape:
            return "joint_count_mismatch"
        if state.estop_active:
            return "estop_active"
        if state.fault_code:
            return "driver_fault"
        if not state.watchdog_ok:
            return "watchdog_not_ok"
        if not state.feedback_ok:
            return "feedback_not_ok"
        if not np.all(np.isfinite(state.q)) or not np.all(np.isfinite(state.qdot)):
            return "state_not_finite"
        # 陈旧反馈优先于速度违例：超时是更根本的原因，先锁存它。
        if float(now_s) - state.stamp_s > self.config.feedback_timeout_s:
            return "feedback_timeout"
        if np.any(np.abs(state.qdot) > self.config.qdot_limit):
            return "state_velocity_limit"
        return ""

    def _stop(
        self, state: HardwareState, now_s: float, reason: str,
        *, source: str = "safety_gate", limit_status: str | None = None,
    ) -> PositionCommand:
        if not self._latched_stop_reason:
            self._latched_stop_reason = str(reason)
        # 零速保持：保持在当前实际位置（不回零、不继续追目标）。
        # 位置未知（非有限）时兜底顺序：最后安全目标 → 齐零（回零位姿）；
        # 桥接层必须对该输出 fail-closed（不发送），不能把未知位置当真。
        hold = np.asarray(state.q, dtype=float).copy()
        if not np.all(np.isfinite(hold)):
            hold = self._last_safe_position if self._last_safe_position is not None \
                else np.zeros_like(self.config.qdot_limit)
        self._last_safe_position = None
        return PositionCommand(
            q=hold, stamp_s=now_s, valid_until_s=now_s, source=source,
            stop_reason=self._latched_stop_reason, mode="stopped",
            limit_status=(self._latched_stop_reason if limit_status is None
                          else limit_status),
        )


class ShadowCommandRecorder:
    """记录"本应发送"的安全命令；API 无任何 send 方法。"""

    def __init__(self) -> None:
        self.records: list[tuple[HardwareState, PositionCommand]] = []

    def record(self, state: HardwareState, command: PositionCommand) -> None:
        self.records.append((state, command))

    @property
    def stop_count(self) -> int:
        return sum(1 for _, command in self.records if command.is_stop)
