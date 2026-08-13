"""A JAX rate-slack stop must discard the integrated candidate state."""

from contextlib import nullcontext
from types import SimpleNamespace

import numpy as np
import pytest

pytestmark = pytest.mark.skip(
    reason="depends on newaxis and HARD_STOP semantics (OSCBF_PORTING_GUIDE.md "
           "§4.7/§5.5); M7 removes HARD_STOP and re-establishes this contract"
)


def test_unverified_rate_slack_rolls_back_jax_path_state_and_emits_zero(monkeypatch):
    """The next CBF step must see the pose that the actuator actually kept.

    The fixed-shape JIT kernel returns ``q_next`` and updated path state in one
    result.  A rate slack without a measured braking contract is a hard stop,
    so accepting either state would create a false command/plant history.
    """
    import newaxis.tracking_execution as tracking
    from newaxis.hardware_stop_execution import HardwareStopExecutionMixin
    from newaxis.tracking_execution import TrackingExecutionMixin

    class _Timer:
        def record(self, *_args):
            return None

    class _Audit:
        @staticmethod
        def metrics():
            return {}

    class _JumpLog:
        def __init__(self):
            self.calls = []

        def update(self, *_args, **kwargs):
            self.calls.append(kwargs)

    class _Rviz:
        def publish_visualization(self, *_args, **_kwargs):
            return None

    task_ref = SimpleNamespace(
        pos=np.zeros(3), vel=np.zeros(3), R_des=np.eye(3),
        omega=np.zeros(3), t=0.4)

    class _Controller:
        last_nominal_diagnostics = {}

        @staticmethod
        def tracking_schedule_factors(_t):
            return {}

    class _JaxLoop:
        is_initialized = True
        last_rate_slack = 0.02
        last_qp_candidate = np.full(9, 0.3)
        last_qp_active_count = 0
        last_qp_iterations = 8
        last_qp_warm_start_used = False
        last_qp_primal_residual = 0.0
        last_qp_terminal_kkt_residual = 0.0
        last_qp_terminal_kkt_accepted = True
        last_qp_dual_max = 0.0
        last_cbf_h_delta_norm = 0.0
        last_cbf_grad_delta_norm = 0.0

        @staticmethod
        def path_tracking_step(*, q, path_state, **_kwargs):
            return SimpleNamespace(
                q_next=np.asarray(q, dtype=float) + 0.25,
                u_safe=np.full(9, 0.3),
                u_nom=np.full(9, 0.2),
                err_6d=np.zeros(6),
                ee_pos=np.zeros(3),
                ee_rot=np.eye(3),
                qp_ok=True,
                min_obs_dist=1.0,
                path_state=np.asarray(path_state, dtype=float) + 0.4,
            )

    class _Runner(HardwareStopExecutionMixin, TrackingExecutionMixin):
        def __init__(self):
            self.path_following_enabled = True
            self.controller = _Controller()
            self.jax_loop = _JaxLoop()
            self.q = np.linspace(-0.2, 0.2, 9)
            self._jax_path_state = np.array([0.12, 0.08, 0.01, 0.0])
            self._last_joint_velocity_cmd = np.zeros(9)
            self._driver_sim = None
            self._step_timer = _Timer()
            self._perception_enabled = False
            self._last_safety_snapshot_age_ms = 0.0
            self._last_perception_margin_m = 0.0
            self._posture_planner_metrics = {}
            self._hardware_metrics = {'hardware_stop': 0.0}
            self._control_stop_latch_reason = ''
            self.rate_slack_braking_profile_verified = False
            self.dynamic_obstacle_scenario = 'unit_test'
            self.kp_pos = self.kp_orient = self.kp_joint = 1.0
            self.q_des = np.zeros(9)
            self.nullspace_speed_limit = 1.0
            self.damping = 0.0
            self.kin = SimpleNamespace(
                joint_limits=SimpleNamespace(
                    q_min=np.full(9, -1.0), q_max=np.full(9, 1.0)),
                actuator_limit_profile=SimpleNamespace(
                    hardware_executable=False, j1_simulation_only=True),
            )
            self.ee_history = []
            self.error_history = []
            self.orient_error_history = []
            self._last_path_metrics = {}
            self._jump_diag = _JumpLog()
            self.rviz_pub = _Rviz()
            self.dt = 0.01
            self.gate_calls = []

        @staticmethod
        def _timed(_name):
            return nullcontext()

        @staticmethod
        def path_task_reference_for_control():
            return task_ref, task_ref.t

        @staticmethod
        def _apply_jax_path_result(_result):
            return task_ref

        def _collect_obstacle_arrays(self, **_kwargs):
            return (
                np.zeros((8, 3)), np.zeros(8), np.zeros(8), np.zeros((8, 3)),
                np.zeros(8), np.zeros(8), np.ones(8),
            )

        @staticmethod
        def _safety_snapshot_inputs():
            return {}

        @staticmethod
        def _merge_perception_tracks(*arrays):
            return arrays

        @staticmethod
        def _check_qp_health(_qp_ok, command):
            return np.asarray(command, dtype=float)

        def _apply_hardware_command_gate(self, _t, q_before, requested, *, source,
                                         stop_reason=''):
            self.gate_calls.append((np.asarray(q_before).copy(),
                                    np.asarray(requested).copy(), source, stop_reason))
            return np.asarray(requested, dtype=float)

        def _remember_joint_velocity_cmd(self, command):
            self._last_joint_velocity_cmd = np.asarray(command, dtype=float).copy()

        @staticmethod
        def _update_path_endpoint_settle(*_args):
            return None

    monkeypatch.setattr(tracking, 'audit_velocity_layers', lambda *_args: _Audit())
    monkeypatch.setattr(tracking, 'record_runner_joint_sample', lambda *_args: None)
    runner = _Runner()
    q_before = runner.q.copy()
    path_before = runner._jax_path_state.copy()

    runner._step_tracking(0.4)

    np.testing.assert_allclose(runner.q, q_before)
    np.testing.assert_allclose(runner._jax_path_state, path_before)
    np.testing.assert_allclose(runner._last_joint_velocity_cmd, 0.0)
    assert runner.gate_calls[0][2:] == (
        'jax_cbf_qp', 'rate_slack_without_verified_braking_profile')
    np.testing.assert_allclose(runner.gate_calls[0][1], 0.0)
    assert runner._last_control_safety_state == 'HARD_STOP'
    assert runner._last_control_stop_reason == 'rate_slack_without_verified_braking_profile'
    assert runner._last_metrics['control_hard_stop'] == 1.0
    np.testing.assert_allclose(
        [runner._last_metrics[f'u_safe{i}'] for i in range(9)], 0.0)
    np.testing.assert_allclose(
        [runner._last_metrics[f'u_qp{i}'] for i in range(9)], 0.3)
    assert runner._jump_diag.calls[0]['context']['control_safety_state'] == 'HARD_STOP'
