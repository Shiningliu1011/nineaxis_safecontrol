"""CAN 总线后端（duck-type 抽象 + 轮询层）——无 ROS / 无 I/O。

本模块不实现真实 SocketCAN（等 CANable 到货后接入），但定义了
``CANBusBackend`` 抽象（send/recv/close），测试用 FakeCANBackend
注入，生产用 SocketCANBackend（待实现）替换。

职责：
- 轮询 9 节点状态，超时检测，重连
- 统计发送/丢失/延迟（CANBusMetrics）
- send_position / send_enable / send_estop 高层接口
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Protocol, Sequence

import numpy as np

from .drempower_can import (
    MAX_NODE_ID,
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
    ) -> tuple[int, bytes] | None:
        """接收 node_id 的响应帧，超时返回 None。"""
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
        if self.poll_interval_s <= 0.0:
            raise ValueError("poll_interval_s 必须为正")


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

def poll_node_state(
    node_id: int, backend: CANBusBackend, now_s: float,
) -> CANNodeState:
    """单节点轮询：发一帧读状态，离线返回 offline 状态。"""
    # 发一个无害的属性读帧（不改变电机状态）
    backend.send(can_id(node_id, 0x1E), b"\x00" * 8)
    result = backend.recv(node_id, timeout_s=0.01)
    if result is None:
        return CANNodeState(
            node_id=node_id, pos_deg=0.0, vel_rpm=0.0, torque_nm=0.0,
            traj_done=False, axis_error=False, stamp_s=now_s, online=False,
        )
    resp_id, data = result
    try:
        fb = decode_feedback(resp_id, data)
    except (ValueError, struct.error):
        return CANNodeState(
            node_id=node_id, pos_deg=0.0, vel_rpm=0.0, torque_nm=0.0,
            traj_done=False, axis_error=True, stamp_s=now_s, online=False,
        )
    return CANNodeState(
        node_id=node_id,
        pos_deg=fb.pos_deg,
        vel_rpm=fb.vel_rpm,
        torque_nm=fb.torque_nm,
        traj_done=fb.traj_done,
        axis_error=fb.axis_error,
        stamp_s=now_s,
        online=True,
    )


import struct  # noqa: E402  — 延迟导入避免循环（decode_feedback 用 struct）


def discover_nodes(
    backend: CANBusBackend, *, timeout_s: float = 0.01,
    max_nodes: int = MAX_NODE_ID,
) -> list[CANNodeState]:
    """轮询节点 1..max_nodes 并返回状态列表。"""
    now = time.time()
    states = []
    for node_id in range(1, min(max_nodes, MAX_NODE_ID) + 1):
        states.append(poll_node_state(node_id, backend, now))
        if states[-1].online is False and node_id > 9:
            # 超出 9 关节后连续离线则提前终止（节省轮询时间）
            offline_streak = sum(1 for s in reversed(states) if not s.online)
            if offline_streak >= 5:
                break
    return states


def check_node_status(node_id: int, backend: CANBusBackend) -> CANNodeStatus:
    """单节点诊断：使能状态 + 错误码。"""
    # 读 requested_state（地址 30002）
    backend.send(can_id(node_id, 0x1E), struct.pack("<H", 30002) + b"\x00" * 6)
    result = backend.recv(node_id, timeout_s=0.01)
    if result is None:
        return CANNodeStatus(node_id=node_id, online=False, state=0, error_code=0)
    _, data = result
    if len(data) >= 4:
        state = struct.unpack("<H", data[2:4])[0]
    else:
        state = 0
    return CANNodeStatus(node_id=node_id, online=True, state=state, error_code=0)


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
    """CAN 总线管理器：轮询、发送、统计、重连。"""

    def __init__(
        self, config: CANBusConfig, *, backend: CANBusBackend | None = None,
    ) -> None:
        self.config = config
        self.metrics = CANBusMetrics()
        self._node_states: dict[int, CANNodeState] = {}
        self._backend: CANBusBackend | None = backend
        self._connected = backend is not None

    @property
    def is_connected(self) -> bool:
        return self._connected and self._backend is not None

    def reconnect(self, *, backend: CANBusBackend | None = None) -> None:
        """重连（当前为 no-op，真实后端实现时替换）。"""
        if backend is not None:
            self._backend = backend
        elif self._backend is None:
            # 真实后端：重新打开 socket
            self._backend = self._create_backend()
        self._connected = True
        self.metrics.record_reconnect()

    def _create_backend(self) -> CANBusBackend:
        """工厂方法：真实后端实现时替换（目前返回 None，调用方必须注入）。"""
        raise NotImplementedError(
            "真实 SocketCAN 后端未实现；请注入 backend 参数")

    def close(self) -> None:
        if self._backend is not None:
            self._backend.close()
        self._backend = None
        self._connected = False

    def send_position(self, node_id: int, target_deg: float, *,
                      speed: float = 1.0, filter_accel: float = 1.0) -> bool:
        """发送位置命令（0x19）。"""
        if not self.is_connected:
            return False
        data = encode_position(node_id, target_deg, speed=speed,
                               filter_accel=filter_accel)
        success = self._backend.send(can_id(node_id, CMD_POSITION_ANGLE_MODE0), data)
        self.metrics.record_send(success)
        return success

    def send_enable(self, node_id: int) -> bool:
        """使能节点（clear_error → requested_state=8）。"""
        if not self.is_connected:
            return False
        # clear_error
        data_clear = encode_system(node_id, SYSTEM_ORDER_CLEAR_ERROR)
        self._backend.send(can_id(node_id, CMD_SYSTEM), data_clear)
        # write requested_state = CLOSED_LOOP
        data_enable = encode_property_write(
            node_id, 30002, AXIS_STATE_CLOSED_LOOP, "u16")
        success = self._backend.send(
            can_id(node_id, CMD_PROPERTY_WRITE), data_enable)
        self.metrics.record_send(success)
        return success

    def send_disable(self, node_id: int) -> bool:
        """失能节点（requested_state=IDLE）。"""
        if not self.is_connected:
            return False
        data = encode_property_write(
            node_id, 30002, AXIS_STATE_IDLE, "u16")
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

    def poll_all(self) -> list[CANNodeState]:
        """轮询所有配置节点并更新内部状态；返回状态列表。"""
        now = time.time()
        states = []
        for node_id in self.config.node_ids:
            prev = self._node_states.get(node_id)
            state = poll_node_state(node_id, self._backend, now)
            states.append(state)
            self._node_states[node_id] = state
            if state.axis_error:
                self.metrics.record_error()
            # 超时检测：之前在线、现在离线
            if prev is not None and prev.online and not state.online:
                self.metrics.record_stale()
            if state.online and prev is not None and prev.online:
                dt = state.stamp_s - prev.stamp_s
                if dt > 0:
                    self.metrics.record_latency(dt)
        return states

    def get_node_state(self, node_id: int) -> CANNodeState | None:
        return self._node_states.get(node_id)

    @property
    def node_states(self) -> dict[int, CANNodeState]:
        return dict(self._node_states)
