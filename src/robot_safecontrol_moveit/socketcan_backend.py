"""CAN management and legacy polling seam. Not a qualified live control loop.

Real transport uses the optional python-can adapter. No automatic enable or
network configuration occurs; sim/shadow bridge never constructs this bus.
Motion commands require an explicit permit from the qualified safety owner;
without it, the transport remains read-only even when a real CAN socket opens.
"""

from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass, field
from typing import Callable, Protocol

import numpy as np

from .python_can_backend import ReceivedFrame
from .drempower_can import (
    MAX_NODE_ID,
    PROP_AXIS_REQUESTED_STATE,
    PROP_AXIS_CURRENT_STATE,
    PROP_AXIS_ERROR,
    encode_property_read,
    decode_property_reply,
    CMD_POSITION_ANGLE_MODE0,
    CMD_PROPERTY_WRITE,
    CMD_SYSTEM,
    SYSTEM_ORDER_CLEAR_ERROR,
    SYSTEM_ORDER_ESTOP,
    AXIS_STATE_CLOSED_LOOP,
    AXIS_STATE_IDLE,
    can_id,
    decode_feedback,
    encode_position,
    encode_property_write,
    encode_system,
)


# --- 抽象接口 ----------------------------------------------------------------

class CANBusBackend(Protocol):
    """CAN 总线收发接口（duck-type，注入 FakeCANBackend 或真实后端）。"""

    def send(self, frame_id: int, data: bytes) -> bool:
        """发送一帧，返回 True=成功。"""
        ...

    def recv(
        self, node_id: int, timeout_s: float = 0.01,
    ) -> ReceivedFrame | None:
        """接收 node_id 的响应帧，并保留底层接收时间。"""
        ...

    def close(self) -> None:
        """释放总线资源。"""
        ...


# --- 配置与状态 ---------------------------------------------------------------

@dataclass
class CANBusConfig:
    """总线配置。"""

    interface: str = "vcan0"
    bitrate: int = 1000000
    node_ids: tuple[int, ...] = tuple(range(1, 10))
    feedback_timeout_s: float = 0.2
    poll_interval_s: float = 0.001

    def __post_init__(self) -> None:
        if not math.isfinite(self.poll_interval_s) or self.poll_interval_s <= 0.0:
            raise ValueError("poll_interval_s 必须为正")
        if not math.isfinite(self.feedback_timeout_s) or self.feedback_timeout_s <= 0:
            raise ValueError("feedback timeout must be finite and positive")
        if (not self.node_ids or len(set(self.node_ids)) != len(self.node_ids)
                or any(type(n) is not int or not 1 <= n <= MAX_NODE_ID for n in self.node_ids)):
            raise ValueError("unique non-broadcast node IDs required")


@dataclass(frozen=True)
class CANNodeState:
    """单节点最新反馈状态。"""

    node_id: int
    pos_deg: float
    vel_rpm: float
    torque_nm: float
    traj_done: bool
    axis_error: bool
    stamp_s: float
    online: bool


@dataclass
class CANBusMetrics:
    """轮询/发送统计。"""

    sent: int = 0
    lost: int = 0
    stale_count: int = 0
    error_count: int = 0
    reconnect_count: int = 0
    _latencies: list[float] = field(default_factory=list, repr=False)

    def record_send(self, success: bool) -> None:
        self.sent += 1
        if not success:
            self.lost += 1

    def record_latency(self, latency_s: float) -> None:
        self._latencies.append(float(latency_s))

    def record_stale(self) -> None:
        self.stale_count += 1

    def record_error(self) -> None:
        self.error_count += 1

    def record_reconnect(self) -> None:
        self.reconnect_count += 1

    @property
    def loss_rate(self) -> float:
        return float(self.lost) / float(self.sent) if self.sent > 0 else 0.0

    @property
    def latency_p50(self) -> float:
        return float(np.percentile(self._latencies, 50)) if self._latencies else 0.0

    @property
    def latency_p95(self) -> float:
        return float(np.percentile(self._latencies, 95)) if self._latencies else 0.0

    @property
    def latency_p99(self) -> float:
        return float(np.percentile(self._latencies, 99)) if self._latencies else 0.0


# --- 自由函数 ----------------------------------------------------------------

# Quick feedback keeps the 0x1E read command's bits 3..4 and sets bit 0.
# Bits 1 and 2 carry trajectory-done and axis-error flags, respectively.
_QUICK_FEEDBACK_MASK = 0x19
_PROPERTY_TYPE_CODES = frozenset(range(5))
_MAX_CLOCK_SKEW_S = 0.05


def _offline_state(node_id: int, *, axis_error: bool = False) -> CANNodeState:
    return CANNodeState(
        node_id=node_id, pos_deg=math.nan, vel_rpm=math.nan, torque_nm=math.nan,
        traj_done=False, axis_error=axis_error, stamp_s=float("-inf"), online=False,
    )


def _looks_like_property_reply(data: bytes) -> bool:
    """Recognize the typed address/type envelope used by property replies.

    Quick feedback uses address zero. A nonzero address with a valid type code
    is therefore never a valid quick feedback payload and must not be decoded
    as a motor state when it is left in the receive queue.
    """
    if len(data) != 8:
        return False
    address, type_code = struct.unpack_from("<HH", data)
    return address != 0 and type_code in _PROPERTY_TYPE_CODES


def _coerce_received_frame(result) -> ReceivedFrame | None:
    """Normalize the legacy tuple seam without manufacturing a timestamp."""
    if result is None:
        return None
    if isinstance(result, ReceivedFrame):
        return result
    if isinstance(result, tuple) and len(result) == 2:
        # Legacy backends did not expose transport time. Treat those frames as
        # unqualified for feedback rather than stamping them with ``now``.
        return ReceivedFrame(int(result[0]), bytes(result[1]), float("nan"))
    raise ValueError("backend recv must return ReceivedFrame or (frame_id, data)")


def poll_node_state(
    node_id: int, backend: CANBusBackend, now_s: float, *, timeout_s: float = 0.01,
    feedback_timeout_s: float = 0.2,
) -> CANNodeState:
    """单节点轮询：发一帧读状态，离线返回 offline 状态。"""
    if (not math.isfinite(now_s) or not math.isfinite(feedback_timeout_s)
            or feedback_timeout_s <= 0.0):
        raise ValueError("finite poll time and positive feedback timeout required")
    request_started_s = max(float(now_s), time.time())
    # 发一个无害的属性读帧（不改变电机状态）
    sent = backend.send(can_id(node_id, 0x1E), b"\x00" * 8)
    result = backend.recv(node_id, timeout_s=timeout_s) if sent else None
    try:
        frame = _coerce_received_frame(result)
    except (TypeError, ValueError):
        return _offline_state(node_id, axis_error=True)
    if frame is None:
        return _offline_state(node_id)
    try:
        observed_s = time.time()
        if (frame.frame_id >> 5 != node_id
                or (frame.frame_id & _QUICK_FEEDBACK_MASK) != _QUICK_FEEDBACK_MASK
                or not math.isfinite(frame.timestamp_s)
                or frame.timestamp_s < request_started_s
                or frame.timestamp_s > observed_s + _MAX_CLOCK_SKEW_S
                or observed_s - frame.timestamp_s > feedback_timeout_s
                or _looks_like_property_reply(frame.data)):
            raise ValueError("stale, foreign, property or non-quick feedback frame")
        fb = decode_feedback(frame.frame_id, frame.data)
    except (ValueError, struct.error):
        return _offline_state(node_id, axis_error=True)
    return CANNodeState(
        node_id=node_id,
        pos_deg=fb.pos_deg,
        vel_rpm=fb.vel_rpm,
        torque_nm=fb.torque_nm,
        traj_done=fb.traj_done,
        axis_error=fb.axis_error,
        stamp_s=frame.timestamp_s,
        online=True,
    )


def discover_nodes(
    backend: CANBusBackend, *, timeout_s: float = 0.01,
    max_nodes: int = MAX_NODE_ID,
) -> list[CANNodeState]:
    """轮询节点 1..max_nodes 并返回状态列表。"""
    now = time.time()
    states = []
    for node_id in range(1, min(max_nodes, MAX_NODE_ID) + 1):
        states.append(poll_node_state(node_id, backend, now, timeout_s=timeout_s))
        if states[-1].online is False and node_id > 9:
            # 超出 9 关节后连续离线则提前终止（节省轮询时间）
            offline_streak = 0
            for state in reversed(states):
                if state.online:
                    break
                offline_streak += 1
            if offline_streak >= 5:
                break
    return states


def check_node_status(node_id: int, backend: CANBusBackend) -> CANNodeStatus:
    """单节点诊断：使能状态 + 错误码。"""
    values = []
    for address in (PROP_AXIS_CURRENT_STATE, PROP_AXIS_ERROR):
        if not backend.send(can_id(node_id, 0x1E), encode_property_read(node_id, address)):
            return CANNodeStatus(node_id, False, 0, 0)
        result = backend.recv(node_id, timeout_s=0.01)
        if result is None:
            return CANNodeStatus(node_id, False, 0, 0)
        try:
            values.append(decode_property_reply(*result, node_id=node_id, address=address))
        except ValueError:
            return CANNodeStatus(node_id, False, 0, 0)
    return CANNodeStatus(node_id, True, *values)


# --- 主类 --------------------------------------------------------------------

@dataclass(frozen=True)
class CANNodeStatus:
    node_id: int
    online: bool
    state: int  # 1=IDLE, 8=CLOSED_LOOP
    error_code: int


@dataclass
class SystemOrder:
    """系统命令请求（用于 send_estop 等广播命令）。"""

    order_num: int
    broadcast: bool = True


class SocketCANBus:
    """CAN 总线管理器：轮询、发送、统计、重连。

    The transport is not itself a live-control qualification gate. Callers
    must inject a safety-owned ``motion_permit`` before position, enable or
    disable commands can be sent; the default is fail-closed.
    """

    def __init__(
        self, config: CANBusConfig, *, backend: CANBusBackend | None = None,
        motion_permit: Callable[[], bool] | None = None,
    ) -> None:
        self.config = config
        self.metrics = CANBusMetrics()
        self._node_states: dict[int, CANNodeState] = {}
        self._backend: CANBusBackend | None = backend
        self._connected = backend is not None
        self._motion_permit = motion_permit

    @property
    def is_connected(self) -> bool:
        return self._connected and self._backend is not None

    def reconnect(self, *, backend: CANBusBackend | None = None) -> None:
        """Explicit transport replacement; never sends enable or motion."""
        if self._backend is not None and self._backend is not backend:
            self.close()
        if backend is not None:
            self._backend = backend
        elif self._backend is None:
            # 真实后端：重新打开 socket
            self._backend = self._create_backend()
        self._connected = True
        self.metrics.record_reconnect()

    def _create_backend(self) -> CANBusBackend:
        """Open the configured Linux interface without configuring it."""
        from .python_can_backend import PythonCANBackend
        return PythonCANBackend(self.config.interface)

    def close(self) -> None:
        if self._backend is not None:
            self._backend.close()
        self._backend = None
        self._connected = False
        self._node_states.clear()

    def send_position(self, node_id: int, target_deg: float, *,
                      speed: float = 1.0, filter_accel: float = 1.0) -> bool:
        """发送位置命令（0x19）。"""
        if not self.is_connected or not self._motion_is_permitted():
            return False
        data = encode_position(node_id, target_deg, speed=speed,
                               filter_accel=filter_accel)
        success = self._backend.send(can_id(node_id, CMD_POSITION_ANGLE_MODE0), data)
        self.metrics.record_send(success)
        return success

    def send_enable(self, node_id: int) -> bool:
        """使能节点（clear_error → requested_state=8）。"""
        if not self.is_connected or not self._motion_is_permitted():
            return False
        # clear_error
        data_clear = encode_system(node_id, SYSTEM_ORDER_CLEAR_ERROR)
        cleared = self._backend.send(can_id(node_id, CMD_SYSTEM), data_clear)
        self.metrics.record_send(cleared)
        if not cleared:
            return False
        # write requested_state = CLOSED_LOOP
        data_enable = encode_property_write(
            node_id, PROP_AXIS_REQUESTED_STATE, AXIS_STATE_CLOSED_LOOP, "u32")
        success = self._backend.send(
            can_id(node_id, CMD_PROPERTY_WRITE), data_enable)
        self.metrics.record_send(success)
        return success

    def send_disable(self, node_id: int) -> bool:
        """失能节点（requested_state=IDLE）。"""
        if not self.is_connected or not self._motion_is_permitted():
            return False
        data = encode_property_write(
            node_id, PROP_AXIS_REQUESTED_STATE, AXIS_STATE_IDLE, "u32")
        success = self._backend.send(
            can_id(node_id, CMD_PROPERTY_WRITE), data)
        self.metrics.record_send(success)
        return success

    def send_estop(self) -> bool:
        """广播紧急停止（node_id=0）。"""
        if not self.is_connected:
            return False
        data = encode_system(0, SYSTEM_ORDER_ESTOP)
        success = self._backend.send(can_id(0, CMD_SYSTEM), data)
        self.metrics.record_send(success)
        return success

    def _motion_is_permitted(self) -> bool:
        """Fail closed until a qualified safety owner supplies a permit."""
        if self._motion_permit is None:
            return False
        try:
            return bool(self._motion_permit())
        except Exception:
            return False

    def poll_all(self) -> list[CANNodeState]:
        """轮询所有配置节点并更新内部状态；返回状态列表。"""
        if not self.is_connected:
            raise RuntimeError("CAN bus disconnected")
        now = time.time()
        states = []
        for node_id in self.config.node_ids:
            prev = self._node_states.get(node_id)
            requested_at = time.monotonic()
            state = poll_node_state(
                node_id, self._backend, now,
                feedback_timeout_s=self.config.feedback_timeout_s,
            )
            if state.online:
                self.metrics.record_latency(time.monotonic() - requested_at)
            states.append(state)
            self._node_states[node_id] = state
            if state.axis_error:
                self.metrics.record_error()
            # 超时检测：之前在线、现在离线
            if prev is not None and prev.online and not state.online:
                self.metrics.record_stale()
        return states

    def get_node_state(self, node_id: int) -> CANNodeState | None:
        return self._node_states.get(node_id)

    @property
    def node_states(self) -> dict[int, CANNodeState]:
        return dict(self._node_states)
