"""真机执行端节点（hardware_bridge）话题级测试。

验证：订阅 /oscbf_command → 安全网关 → 单位换算 → 帧编码 → 后端发送；
反馈帧 → 解码 → 单位换算 → 发布 /mujoco_joint_states。
shadow 模式记录但不发送；live 模式真实发送。
"""

import time

import numpy as np
import pytest
import rclpy
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState

from robot_safecontrol_moveit.drempower_can import encode_position
from robot_safecontrol_moveit.hardware_contract import HardwareState, PositionCommand
from robot_safecontrol_moveit.socketcan_backend import CANBusBackend, CANBusConfig, CANNodeState
from robot_safecontrol_moveit.ros_conventions import JOINT_STATE_TOPIC, OSCBF_COMMAND_TOPIC, state_stream_qos
from robot_safecontrol_moveit.unit_conversion import TransmissionSpec


class FakeCANBackendForBridge(CANBusBackend):
    """为 hardware_bridge 测试定制的替身：记录发送帧，可注入故障。"""

    def __init__(self, *, offline_nodes: set[int] | None = None,
                 error_nodes: set[int] | None = None) -> None:
        self.offline_nodes = offline_nodes or set()
        self.error_nodes = error_nodes or set()
        self.sent_frames: list[tuple[int, bytes]] = []
        self.send_count = 0
        self._node_positions: dict[int, float] = {}

    def send(self, frame_id: int, data: bytes) -> bool:
        self.send_count += 1
        self.sent_frames.append((frame_id, data))
        # 记录位置命令供反馈回放
        cmd = frame_id & 0x1F
        node = (frame_id >> 5) & 0x3F
        if cmd == 0x19 and len(data) >= 4:
            import struct
            pos = struct.unpack("<f", data[:4])[0]
            self._node_positions[node] = pos
        return True

    def recv(self, node_id: int, timeout_s: float = 0.01):
        import struct
        if node_id in self.offline_nodes:
            return None
        has_error = node_id in self.error_nodes
        resp_id = ((node_id << 5) | 0x19) | (0x04 if has_error else 0x00)
        pos_deg = self._node_positions.get(node_id, 0.0)
        data = struct.pack("<fhh", pos_deg, 0, 0)
        return resp_id, data

    def close(self) -> None:
        pass


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def bridge_node(ros_context):
    from robot_safecontrol_moveit.hardware_bridge import HardwareBridge
    backend = FakeCANBackendForBridge()
    j1_trans = TransmissionSpec(lead_mm_per_rev=10.0)
    node = HardwareBridge(
        backend=backend, hardware_mode="live",
        j1_transmission=j1_trans,
    )
    yield node, backend
    node.destroy_node()


@pytest.fixture
def shadow_bridge(ros_context):
    from robot_safecontrol_moveit.hardware_bridge import HardwareBridge
    backend = FakeCANBackendForBridge()
    j1_trans = TransmissionSpec(lead_mm_per_rev=10.0)
    node = HardwareBridge(
        backend=backend, hardware_mode="shadow",
        j1_transmission=j1_trans,
    )
    yield node, backend
    node.destroy_node()


def _make_joint_state(positions: list[float]) -> JointState:
    msg = JointState()
    msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    msg.name = [f"J{i}" for i in range(1, 10)]
    msg.position = positions
    return msg


class TestNodeConstruction:
    def test_node_name(self, bridge_node) -> None:
        node, _ = bridge_node
        assert node.get_name() == "hardware_bridge"

    def test_subscribes_command_topic(self, bridge_node) -> None:
        node, _ = bridge_node
        topic_names = [name for name, _ in node.get_topic_names_and_types()]
        assert OSCBF_COMMAND_TOPIC in topic_names

    def test_publishes_state_topic(self, bridge_node) -> None:
        node, _ = bridge_node
        topic_names = [name for name, _ in node.get_topic_names_and_types()]
        assert JOINT_STATE_TOPIC in topic_names


class TestShadowMode:
    def test_shadow_does_not_send_can(self, shadow_bridge) -> None:
        node, backend = shadow_bridge
        q = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
        node._on_command(_make_joint_state(q))
        # shadow 模式不发送 CAN 帧
        assert backend.send_count == 0

    def test_shadow_records_command(self, shadow_bridge) -> None:
        node, backend = shadow_bridge
        q = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
        node._on_command(_make_joint_state(q))
        assert node._last_command is not None
        assert np.allclose(node._last_command.q, q)


class TestLiveMode:
    def test_command_sends_can_frames(self, bridge_node) -> None:
        node, backend = bridge_node
        q = [0.29, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        node._on_command(_make_joint_state(q))
        assert backend.send_count > 0
        # 应该发了 9 轴的位置命令
        assert backend.send_count == 9

    def test_state_published_after_feedback(self, bridge_node) -> None:
        node, backend = bridge_node
        q = [0.29, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        node._on_command(_make_joint_state(q))
        # 模拟反馈：设置 last_feedback_q 后手动发布
        node._last_feedback_q = np.array(q)
        node._publish_feedback_as_state()
        # 发布不抛错即通过
        assert node._last_feedback_q is not None
        assert len(node._last_feedback_q) == 9


class TestSafetyGateIntegration:
    def test_j1_uncalibrated_blocks_command(self, ros_context) -> None:
        from robot_safecontrol_moveit.hardware_bridge import HardwareBridge
        backend = FakeCANBackendForBridge()
        # 不注入 J1 传动 → J1 命令应被拒绝
        node = HardwareBridge(backend=backend, hardware_mode="live",
                              j1_transmission=None)
        q = [0.29, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        node._on_command(_make_joint_state(q))
        # J1 被拒绝，其余轴应该发送
        assert backend.send_count == 8  # J2-J9 only
        node.destroy_node()

    def test_estop_triggers_zero_hold(self, bridge_node) -> None:
        node, backend = bridge_node
        q = [0.29, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        node._on_command(_make_joint_state(q))
        # 模拟急停反馈
        node._inject_state_fault(estop_active=True)
        node._on_command(_make_joint_state(q))
        assert node._safety_gate.latched_stop_reason == "estop_active"

    def test_acknowledge_clears_latch(self, bridge_node) -> None:
        node, backend = bridge_node
        q = [0.29, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        node._on_command(_make_joint_state(q))
        node._inject_state_fault(estop_active=True)
        node._on_command(_make_joint_state(q))
        assert node._safety_gate.latched_stop_reason != ""
        # 恢复健康并人工确认
        node._inject_state_fault(estop_active=False)
        node.acknowledge_stop()
        assert node._safety_gate.latched_stop_reason == ""


class TestModeParameter:
    def test_shadow_mode_parameter(self, shadow_bridge) -> None:
        node, _ = shadow_bridge
        assert node.hardware_mode == "shadow"

    def test_live_mode_parameter(self, bridge_node) -> None:
        node, _ = bridge_node
        assert node.hardware_mode == "live"


def test_no_global_state_leak(ros_context) -> None:
    """两个 bridge 实例不共享状态。"""
    from robot_safecontrol_moveit.hardware_bridge import HardwareBridge
    b1 = FakeCANBackendForBridge()
    b2 = FakeCANBackendForBridge()
    j1 = TransmissionSpec(lead_mm_per_rev=10.0)
    n1 = HardwareBridge(backend=b1, hardware_mode="live", j1_transmission=j1)
    n2 = HardwareBridge(backend=b2, hardware_mode="live", j1_transmission=j1)
    q = [0.29, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    n1._on_command(_make_joint_state(q))
    assert b1.send_count > 0
    assert b2.send_count == 0  # n2 不受影响
    n1.destroy_node()
    n2.destroy_node()
