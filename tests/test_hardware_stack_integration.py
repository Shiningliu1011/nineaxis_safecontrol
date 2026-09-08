"""硬件栈集成测试 — 验证 contract → codec → conversion 接缝边界。

链路:
  关节位置 (URDF rad/m)
  → UnitConverter.joints_to_encoder() → 编码器读数 (deg)
  → drempower_can.encode_position() → CAN 帧 (8 bytes)
  → drempower_can.decode_feedback() → MotorFeedback
  → UnitConverter.encoder_to_joint() → 关节位置
  → HardwareState → CommandSafetyGate.evaluate() → PositionCommand

每一步都是纯函数/纯逻辑，无 I/O。
"""

import numpy as np
import pytest

from robot_safecontrol_moveit.drempower_can import (
    CMD_POSITION_ANGLE_MODE0,
    MotorFeedback,
    can_id,
    decode_feedback,
    encode_position,
)
from robot_safecontrol_moveit.hardware_contract import (
    CommandSafetyGate,
    HardwareState,
    PositionCommand,
    WatchdogConfig,
)
from robot_safecontrol_moveit.unit_conversion import (
    JointCalibrationTable,
    JointNodeMap,
    PerJointCalibration,
    TransmissionSpec,
    UnitConverter,
)


# ─── Fixtures ──────────────────────────────────────────────────────

@pytest.fixture
def node_map():
    return JointNodeMap.from_joint_list([f"J{i}" for i in range(1, 10)])


@pytest.fixture
def calibration():
    return JointCalibrationTable.from_entries([
        PerJointCalibration(f"J{i}", sign=1, zero_offset_deg=0.0)
        for i in range(1, 10)
    ])


@pytest.fixture
def j1_transmission():
    return TransmissionSpec(lead_mm_per_rev=10.0, ratio_motor_rev_per_lead_rev=1.0)


@pytest.fixture
def converter(node_map, calibration, j1_transmission):
    return UnitConverter(
        node_map=node_map, calibration=calibration,
        j1_transmission=j1_transmission)


@pytest.fixture
def safety_gate():
    cfg = WatchdogConfig(
        feedback_timeout_s=0.2,
        command_timeout_s=0.2,
        qdot_limit=np.full(9, 3.0),
        dq_per_command_limit=np.full(9, 0.05),
    )
    return CommandSafetyGate(cfg)


# ─── Round-trip: URDF → encoder → CAN → feedback → encoder → URDF ─

class TestEncoderRoundTrip:
    """关节位置 → 编码器 → CAN 帧 → 解码 → 编码器 → 关节位置"""

    def test_rotary_joint_roundtrip(self, converter):
        """J2（旋转关节）的 URDF↔编码器往返误差 < 0.01°。"""
        q_urdf = 0.5  # rad
        enc_deg = converter.joint_to_encoder("J2", q_urdf)
        q_back = converter.encoder_to_joint("J2", enc_deg)
        assert abs(q_back - q_urdf) < 1e-6

    def test_j1_prismatic_roundtrip(self, converter):
        """J1（棱柱关节）的 URDF↔编码器往返误差 < 1μm。"""
        q_urdf = 0.3  # m
        enc_deg = converter.joint_to_encoder("J1", q_urdf)
        q_back = converter.encoder_to_joint("J1", enc_deg)
        assert abs(q_back - q_urdf) < 1e-6

    def test_full_9dof_roundtrip(self, converter):
        """9 关节批量往返：encode → CAN → decode → compare。"""
        q_original = np.array([0.3, 0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7, -0.8])
        readings_deg = converter.joints_to_encoder(q_original)

        # 每关节编码为 CAN 帧再解码
        q_roundtrip = np.zeros(9)
        for i, (joint, reading) in enumerate(
                zip(["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"],
                    readings_deg)):
            node_id = i + 1
            frame = encode_position(node_id, reading, speed=100, filter_accel=50)
            fb = decode_feedback(can_id(node_id, CMD_POSITION_ANGLE_MODE0),
                                 frame)
            q_roundtrip[i] = converter.encoder_to_joint(joint, fb.pos_deg)

        np.testing.assert_allclose(q_roundtrip, q_original, atol=1e-4)


# ─── Safety gate integration ──────────────────────────────────────

class TestSafetyGateIntegration:
    """CAN 反馈 → HardwareState → SafetyGate → 安全命令"""

    def _make_state(self, q_urdf, converter, now_s=10.0, **overrides):
        """从 URDF 关节位置构造 HardwareState（模拟 CAN 反馈解码链）。"""
        readings_deg = converter.joints_to_encoder(q_urdf)
        kwargs = dict(
            q=q_urdf,
            qdot=np.zeros(9),
            stamp_s=now_s,
            feedback_ok=True,
        )
        kwargs.update(overrides)
        return HardwareState(**kwargs)

    def test_healthy_command_passes(self, converter, safety_gate):
        """健康反馈 + 合法命令 → 命令通过（不停车）。"""
        q = np.array([0.3, 0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7, -0.8])
        state = self._make_state(q, converter, now_s=10.0)
        cmd = PositionCommand(
            q=q + np.full(9, 0.001), stamp_s=10.0, valid_until_s=10.5,
            source="controller")
        result = safety_gate.evaluate(state, cmd, now_s=10.01)
        assert not result.is_stop
        assert result.source == "controller"

    def test_feedback_timeout_triggers_stop(self, converter, safety_gate):
        """反馈超时 → 零速保持 + 锁存原因。"""
        q = np.array([0.3, 0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7, -0.8])
        state = self._make_state(q, converter, now_s=9.0)  # 1s ago
        cmd = PositionCommand(
            q=q, stamp_s=10.0, valid_until_s=10.5, source="controller")
        result = safety_gate.evaluate(state, cmd, now_s=10.1)
        assert result.is_stop
        assert result.stop_reason == "feedback_timeout"
        # 零速保持：位置应等于反馈位置
        np.testing.assert_array_equal(result.q, q)

    def test_rate_limit_triggers_stop(self, converter, safety_gate):
        """相邻命令跳变超限 → 停车。"""
        q = np.array([0.3, 0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7, -0.8])
        state = self._make_state(q, converter, now_s=10.0)

        # 第一条命令建立基准
        cmd1 = PositionCommand(
            q=q, stamp_s=10.0, valid_until_s=10.5, source="controller")
        r1 = safety_gate.evaluate(state, cmd1, now_s=10.01)
        assert not r1.is_stop

        # 第二条命令跳变 0.1 rad（超过 0.05 限制）
        cmd2 = PositionCommand(
            q=q + 0.1, stamp_s=10.02, valid_until_s=10.5, source="controller")
        r2 = safety_gate.evaluate(state, cmd2, now_s=10.03)
        assert r2.is_stop
        assert r2.stop_reason == "command_rate_limit"

    def test_estop_triggers_stop(self, converter, safety_gate):
        """急停反馈 → 停车 + 锁存。"""
        q = np.array([0.3, 0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7, -0.8])
        state = self._make_state(q, converter, now_s=10.0, estop_active=True)
        cmd = PositionCommand(
            q=q, stamp_s=10.0, valid_until_s=10.5, source="controller")
        result = safety_gate.evaluate(state, cmd, now_s=10.01)
        assert result.is_stop
        assert result.stop_reason == "estop_active"

    def test_acknowledge_clears_latch(self, converter, safety_gate):
        """acknowledge_stop 在健康状态下清除锁存。"""
        q = np.array([0.3, 0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7, -0.8])
        # 先触发停车
        state_bad = self._make_state(q, converter, now_s=9.0)
        cmd = PositionCommand(
            q=q, stamp_s=10.0, valid_until_s=10.5, source="controller")
        safety_gate.evaluate(state_bad, cmd, now_s=10.1)
        assert safety_gate.latched_stop_reason

        # 健康状态下确认
        state_good = self._make_state(q, converter, now_s=10.2)
        cleared = safety_gate.acknowledge_stop(state_good, now_s=10.2)
        assert cleared
        assert not safety_gate.latched_stop_reason


# ─── CAN codec + unit conversion joint test ────────────────────────

class TestCanCodecUnitConversion:
    """CAN 编解码器 × 单位换算的联合验证。"""

    def test_position_encoding_precision(self, converter):
        """编码精度：CAN float32 位置 → 关节值误差 < 0.001 rad。"""
        q_target = 1.234  # rad (J2)
        enc_deg = converter.joint_to_encoder("J2", q_target)
        frame = encode_position(2, enc_deg, speed=100, filter_accel=50)
        fb = decode_feedback(can_id(2, CMD_POSITION_ANGLE_MODE0), frame)
        q_decoded = converter.encoder_to_joint("J2", fb.pos_deg)
        assert abs(q_decoded - q_target) < 0.001

    def test_node_id_mapping(self, node_map):
        """节点 ID 映射：J1=1, J2=2, ..., J9=9。"""
        for i in range(1, 10):
            assert node_map.node_of(f"J{i}") == i
            assert node_map.joint_of(i) == f"J{i}"
