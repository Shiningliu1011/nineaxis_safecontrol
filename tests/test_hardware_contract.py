"""安全网关（位置契约）测试。

语义沿袭参考项目 hardware_contract.py：网关只检验、不重塑；超时/故障/限幅
违例 → 零速保持命令（保持当前位置）+ 锁存停车原因 + 人工确认恢复；
J1 未标定时 watchdog 构造在硬件模式 fail-closed。
"""

import numpy as np
import pytest

from robot_safecontrol_moveit.hardware_contract import (
    CommandSafetyGate,
    HardwareState,
    PositionCommand,
    ShadowCommandRecorder,
    WatchdogConfig,
)


def _state(**overrides):
    kwargs = dict(
        q=np.array([0.1, -0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]),
        qdot=np.zeros(9),
        stamp_s=10.0,
        feedback_ok=True,
    )
    kwargs.update(overrides)
    return HardwareState(**kwargs)


def _command(q=None, **overrides):
    kwargs = dict(
        q=np.array([0.11, -0.21, 0.31, 0.41, 0.51, 0.61, 0.71, 0.81, 0.91])
        if q is None else q,
        stamp_s=10.0,
        valid_until_s=10.5,
        source="controller",
    )
    kwargs.update(overrides)
    return PositionCommand(**kwargs)


def _gate(dq_limit=0.01, qdot_limit=3.0, feedback_timeout=0.2, command_timeout=0.2):
    cfg = WatchdogConfig(
        feedback_timeout_s=feedback_timeout,
        command_timeout_s=command_timeout,
        qdot_limit=np.full(9, qdot_limit),
        dq_per_command_limit=np.full(9, dq_limit),
    )
    return CommandSafetyGate(cfg)


class TestContractsConstruct:
    def test_state_shape_mismatch_rejected(self) -> None:
        with pytest.raises(ValueError):
            HardwareState(q=np.zeros(3), qdot=np.zeros(9), stamp_s=0.0, feedback_ok=True)

    def test_watchdog_positive_timeouts_required(self) -> None:
        with pytest.raises(ValueError):
            WatchdogConfig(
                feedback_timeout_s=0.0, command_timeout_s=0.1,
                qdot_limit=np.ones(9), dq_per_command_limit=np.ones(9))

    def test_watchdog_limit_shapes_aligned(self) -> None:
        with pytest.raises(ValueError):
            WatchdogConfig(
                feedback_timeout_s=0.1, command_timeout_s=0.1,
                qdot_limit=np.ones(9), dq_per_command_limit=np.ones(3))

    def test_from_profile_hardware_mode_fails_closed(self) -> None:
        class ProfileStub:
            velocity_limits = np.full(9, 1.0)
            hardware_validated = np.array([False] + [True] * 8)

            def require_hardware_executable(self) -> None:
                if not self.hardware_validated.all():
                    raise RuntimeError("J1 传动未标定")

        with pytest.raises(RuntimeError, match="J1"):
            WatchdogConfig.from_actuator_limit_profile(
                ProfileStub(), feedback_timeout_s=0.1, command_timeout_s=0.1,
                dq_per_command_limit=np.full(9, 0.01),
                require_hardware_executable=True)

    def test_from_profile_shadow_mode_allows_sim_only(self) -> None:
        class ProfileStub:
            velocity_limits = np.full(9, 1.0)
            hardware_validated = np.array([False] + [True] * 8)

        cfg = WatchdogConfig.from_actuator_limit_profile(
            ProfileStub(), feedback_timeout_s=0.1, command_timeout_s=0.1,
            dq_per_command_limit=np.full(9, 0.01))
        assert np.allclose(cfg.qdot_limit, 1.0)


class TestGateHealthy:
    def test_passthrough_restamps(self) -> None:
        gate = _gate()
        out = gate.evaluate(_state(), _command(), now_s=10.05)
        assert np.allclose(out.q, _command().q)
        assert out.source == "controller"
        assert out.stop_reason == ""
        assert out.mode == "shadow"
        assert out.limit_status == "within_limits"

    def test_explicit_stop_reason_forwarded(self) -> None:
        gate = _gate()
        out = gate.evaluate(_state(), _command(stop_reason="operator_halt"), now_s=10.0)
        assert out.is_stop
        assert out.stop_reason == "operator_halt"


class TestGateStopReasons:
    @pytest.mark.parametrize("state_kwargs", [
        {"estop_active": True},
        {"fault_code": "ABC"},
        {"watchdog_ok": False},
        {"feedback_ok": False},
    ])
    def test_state_faults_stop(self, state_kwargs) -> None:
        gate = _gate()
        out = gate.evaluate(_state(**state_kwargs), _command(), now_s=10.0)
        assert out.is_stop
        assert out.limit_status != "within_limits"

    def test_feedback_timeout_stops(self) -> None:
        gate = _gate(feedback_timeout=0.2)
        out = gate.evaluate(_state(stamp_s=10.0), _command(), now_s=10.5)
        assert out.stop_reason == "feedback_timeout"

    def test_command_timeout_stops(self) -> None:
        gate = _gate(command_timeout=0.3)
        # 状态保持新鲜，只让命令过期
        out = gate.evaluate(_state(stamp_s=10.4), _command(stamp_s=10.0, valid_until_s=10.2), now_s=10.4)
        assert out.stop_reason == "command_timeout"

    def test_command_expiry_stops(self) -> None:
        gate = _gate()
        out = gate.evaluate(_state(), _command(valid_until_s=9.9), now_s=10.0)
        assert out.stop_reason == "command_timeout"

    def test_state_joint_count_mismatch_stops(self) -> None:
        gate = _gate()
        bad = _state(q=np.zeros(8), qdot=np.zeros(8))
        out = gate.evaluate(bad, _command(q=np.zeros(9)), now_s=10.0)
        assert out.stop_reason == "joint_count_mismatch"

    def test_command_joint_count_mismatch_stops(self) -> None:
        gate = _gate()
        out = gate.evaluate(_state(), _command(q=np.zeros(4)), now_s=10.0)
        assert out.stop_reason == "command_joint_count_mismatch"

    def test_nonfinite_command_stops(self) -> None:
        gate = _gate()
        out = gate.evaluate(_state(), _command(q=np.full(9, np.nan)), now_s=10.0)
        assert out.stop_reason == "command_not_finite"

    def test_rate_limit_stops(self) -> None:
        gate = _gate(dq_limit=0.01)
        assert not gate.evaluate(_state(), _command(), now_s=10.0).is_stop
        # 相邻命令跳变 0.5 >> 0.01 → 停车（网关不裁剪，只停）
        jump = _command(q=_command().q + 0.5)
        out = gate.evaluate(_state(), jump, now_s=10.05)
        assert out.stop_reason == "command_rate_limit"
        assert out.is_stop

    def test_state_velocity_limit_stops(self) -> None:
        gate = _gate(qdot_limit=1.0)
        out = gate.evaluate(_state(qdot=np.full(9, 2.0)), _command(), now_s=10.0)
        assert out.stop_reason == "state_velocity_limit"

    def test_state_not_finite_stops(self) -> None:
        gate = _gate()
        out = gate.evaluate(_state(q=np.full(9, np.nan)), _command(), now_s=10.0)
        assert out.stop_reason == "state_not_finite"
        out = gate.evaluate(_state(qdot=np.full(9, np.inf)), _command(), now_s=10.0)
        assert out.stop_reason == "state_not_finite"

    def test_stale_feedback_timeout_beats_velocity_violation(self) -> None:
        gate = _gate(feedback_timeout=0.2, qdot_limit=1.0)
        out = gate.evaluate(_state(qdot=np.full(9, 5.0), stamp_s=10.0), _command(), now_s=10.5)
        assert out.stop_reason == "feedback_timeout"


class TestGateLatch:
    def test_first_reason_latches_and_persists(self) -> None:
        gate = _gate()
        first = gate.evaluate(_state(estop_active=True), _command(), now_s=10.0)
        assert first.stop_reason == "estop_active"
        # 状态恢复健康后仍输出停车（锁存，不自动恢复）
        second = gate.evaluate(_state(), _command(), now_s=10.1)
        assert second.is_stop
        assert second.stop_reason == "estop_active"

    def test_stop_holds_current_position(self) -> None:
        gate = _gate()
        state = _state(q=np.array([0.42] * 9), estop_active=True)
        out = gate.evaluate(state, _command(), now_s=10.0)
        assert np.allclose(out.q, state.q)  # 零速保持 = 保持在当前位置

    def test_acknowledge_requires_healthy_state(self) -> None:
        gate = _gate()
        gate.evaluate(_state(estop_active=True), _command(), now_s=10.0)
        assert not gate.acknowledge_stop(_state(estop_active=True), now_s=10.1)
        # 状态健康 + 外部确认 → 恢复
        assert gate.acknowledge_stop(_state(stamp_s=10.2), now_s=10.2) is True
        out = gate.evaluate(_state(stamp_s=10.3), _command(stamp_s=10.3), now_s=10.3)
        assert not out.is_stop

    def test_evaluate_never_auto_clears_latch(self) -> None:
        gate = _gate()
        gate.evaluate(_state(fault_code="X"), _command(), now_s=10.0)
        for i in range(3):
            out = gate.evaluate(_state(), _command(), now_s=10.5 + i)
            assert out.is_stop


class TestRecorder:
    def test_records_and_counts_stops(self) -> None:
        rec = ShadowCommandRecorder()
        gate = _gate()
        rec.record(_state(), _command())
        rec.record(_state(), _command(stop_reason="x"))
        assert len(rec.records) == 2
        assert rec.stop_count == 1
        # 记录器没有发送 API
        assert not hasattr(rec, "send")


def test_no_global_state_between_gates() -> None:
    g1, g2 = _gate(), _gate()
    g1.evaluate(_state(estop_active=True), _command(), now_s=10.0)
    out = g2.evaluate(_state(), _command(), now_s=10.0)
    assert not out.is_stop  # g2 不受 g1 锁存影响
