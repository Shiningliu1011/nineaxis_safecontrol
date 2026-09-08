"""SocketCAN 后端测试（无 root/vcan 限制）。

验证 9 节点轮询、丢帧统计、超时/离线检测、重连行为；全部通过
FakeCANBackend 注入，不依赖硬件。
"""

import struct
import time

import numpy as np
import pytest

from robot_safecontrol_moveit.drempower_can import (
    can_id,
    decode_feedback,
    encode_position,
    encode_system,
)
from robot_safecontrol_moveit.socketcan_backend import (
    CANBusBackend,
    CANBusConfig,
    CANBusMetrics,
    CANNodeState,
    CANNodeStatus,
    SocketCANBus,
    check_node_status,
    discover_nodes,
    poll_node_state,
)


class FakeCANBackend(CANBusBackend):
    """测试替身：可配置丢帧、延迟、离线节点；无 I/O。"""

    def __init__(
        self,
        *,
        latency_s: float = 0.0,
        loss_rate: float = 0.0,
        offline_nodes: set[int] | None = None,
        error_nodes: set[int] | None = None,
        known_nodes: set[int] | None = None,
        seed: int = 42,
    ) -> None:
        self.latency_s = latency_s
        self.loss_rate = loss_rate
        self.offline_nodes = offline_nodes or set()
        self.error_nodes = error_nodes or set()
        # 默认只响应 1-9 关节（仿真与真机一致）
        self.known_nodes = known_nodes or set(range(1, 10))
        self.rng = np.random.default_rng(seed)
        self.sent_frames: list[tuple[int, bytes]] = []
        self.received_frames: list[tuple[int, bytes, float]] = []
        self.sent_count = 0
        self.lost_count = 0

    def send(self, frame_id: int, data: bytes) -> bool:
        """发送：按丢帧率决定是否丢失；返回 True=成功。"""
        self.sent_count += 1
        if self.rng.random() < self.loss_rate:
            self.lost_count += 1
            return False
        self.sent_frames.append((frame_id, data))
        if self.latency_s > 0.0:
            time.sleep(self.latency_s)
        return True

    def recv(
        self, node_id: int, timeout_s: float = 0.01,
    ) -> tuple[int, bytes] | None:
        """接收：构造反馈帧，离线/错误节点返回 None 或带错误标志。"""
        if node_id in self.offline_nodes or node_id not in self.known_nodes:
            return None
        has_error = node_id in self.error_nodes
        resp_id = can_id(node_id, 0x19) | (0x04 if has_error else 0x00)
        # 优先回放该节点的最后一个位置命令（0x19）
        pos_deg = 0.0
        for fid, data in reversed(self.sent_frames):
            sent_node = (fid >> 5) & 0x3F
            sent_cmd = fid & 0x1F
            if sent_node == node_id and sent_cmd == 0x19:
                pos_deg = struct.unpack("<f", data[:4])[0]
                break
        data = struct.pack("<fhh", pos_deg, 0, 0)
        return resp_id, data

    def close(self) -> None:
        pass


def _config() -> CANBusConfig:
    return CANBusConfig(
        interface="vcan0",
        bitrate=1000000,
        node_ids=tuple(range(1, 10)),
        feedback_timeout_s=0.2,
        poll_interval_s=0.001,
    )


def _node_state(node_id: int, *, pos_deg: float = 0.0) -> CANNodeState:
    return CANNodeState(
        node_id=node_id,
        pos_deg=pos_deg,
        vel_rpm=0.0,
        torque_nm=0.0,
        traj_done=True,
        axis_error=False,
        stamp_s=time.time(),
        online=True,
    )


class TestCANBusConfig:
    def test_defaults(self) -> None:
        cfg = _config()
        assert cfg.interface == "vcan0"
        assert len(cfg.node_ids) == 9
        assert 1 in cfg.node_ids and 9 in cfg.node_ids

    def test_zero_poll_rejected(self) -> None:
        with pytest.raises(ValueError):
            CANBusConfig(poll_interval_s=-0.1)


class TestFakeBackend:
    def test_send_records_frame(self) -> None:
        fb = FakeCANBackend()
        data = encode_position(1, 45.0, speed=1.0, filter_accel=1.0)
        assert fb.send(can_id(1, 0x19), data) is True
        assert fb.sent_count == 1
        assert len(fb.sent_frames) == 1

    def test_loss_drops_frame(self) -> None:
        fb = FakeCANBackend(loss_rate=1.0)  # 全丢
        assert fb.send(can_id(1, 0x19), b"\x00" * 8) is False
        assert fb.lost_count == 1
        assert len(fb.sent_frames) == 0

    def test_offline_node_returns_none(self) -> None:
        fb = FakeCANBackend(offline_nodes={3})
        assert fb.recv(3) is None
        assert fb.recv(1) is not None

    def test_error_node_flagged(self) -> None:
        fb = FakeCANBackend(error_nodes={5})
        result = fb.recv(5)
        assert result is not None
        resp_id, _ = result
        assert resp_id & 0x04  # axis_error bit


class TestPollNodeState:
    def test_online_feedback_decoded(self) -> None:
        fb = FakeCANBackend()
        fb.send(can_id(1, 0x19), encode_position(1, 90.0, speed=2.0, filter_accel=5.0))
        state = poll_node_state(1, fb, time.time())
        assert state.online is True
        assert state.pos_deg == pytest.approx(90.0, abs=1e-3)
        assert state.node_id == 1

    def test_offline_feedback(self) -> None:
        fb = FakeCANBackend(offline_nodes={3})
        state = poll_node_state(3, fb, time.time())
        assert state.online is False
        assert state.node_id == 3


class TestDiscoverNodes:
    def test_discovers_all_online(self) -> None:
        fb = FakeCANBackend()
        states = discover_nodes(fb, timeout_s=0.01, max_nodes=9)
        online = [s for s in states if s.online]
        assert len(online) == 9
        assert all(s.online for s in online)

    def test_detects_offline_nodes(self) -> None:
        fb = FakeCANBackend(offline_nodes={2, 7})
        states = discover_nodes(fb, timeout_s=0.01, max_nodes=9)
        online = [s for s in states if s.online]
        offline = [s for s in states if not s.online]
        assert len(online) == 7
        assert len(offline) == 2
        assert sorted(s.node_id for s in offline) == [2, 7]


class TestCANBusMetrics:
    def test_count_updates(self) -> None:
        m = CANBusMetrics()
        m.record_send(True)
        m.record_send(False)
        m.record_send(True)
        assert m.sent == 3
        assert m.lost == 1
        assert m.loss_rate == pytest.approx(1.0 / 3.0)

    def test_latency_percentiles(self) -> None:
        m = CANBusMetrics()
        for lat in [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]:
            m.record_latency(lat)
        assert m.latency_p50 == pytest.approx(5.5, abs=0.1)
        assert m.latency_p95 == pytest.approx(9.55, abs=0.2)
        assert m.latency_p99 == pytest.approx(9.91, abs=0.2)


class TestCANBus:
    def test_connect_and_close(self) -> None:
        backend = FakeCANBackend()
        bus = SocketCANBus(_config(), backend=backend)
        assert bus.is_connected
        bus.close()
        assert not bus.is_connected

    def test_poll_all_nodes(self) -> None:
        backend = FakeCANBackend()
        bus = SocketCANBus(_config(), backend=backend)
        bus.send_enable(1)
        # send_enable 发 clear_error + write_requested_state 两帧
        assert len(backend.sent_frames) == 2
        states = bus.poll_all()
        assert len(states) == 9
        assert all(s.online for s in states)
        bus.close()

    def test_stale_detection(self) -> None:
        backend = FakeCANBackend()
        cfg = _config()
        cfg.feedback_timeout_s = 0.01
        bus = SocketCANBus(cfg, backend=backend)
        bus.poll_all()  # 全部在线
        # 让节点 3 离线
        backend.offline_nodes.add(3)
        bus.poll_all()
        assert bus.metrics.stale_count > 0
        bus.close()

    def test_reconnect_on_error(self) -> None:
        backend = FakeCANBackend()
        bus = SocketCANBus(_config(), backend=backend)
        bus._backend = None  # 模拟连接丢失
        assert not bus.is_connected
        new_backend = FakeCANBackend()
        bus.reconnect(backend=new_backend)
        assert bus.is_connected
        bus.close()

    def test_estop_sends_broadcast(self) -> None:
        backend = FakeCANBackend()
        bus = SocketCANBus(_config(), backend=backend)
        bus.send_estop()
        assert len(backend.sent_frames) == 1
        fid, data = backend.sent_frames[0]
        assert fid == can_id(0, 0x08)  # broadcast
        bus.close()


def test_metrics_no_global_leak() -> None:
    m1 = CANBusMetrics()
    m2 = CANBusMetrics()
    m1.record_send(True)
    assert m2.sent == 0  # 无状态泄漏
