#!/usr/bin/env python3
"""JAX soft-rate-limit contract: CBF rows hard, rate rows elastic."""

import numpy as np
import pytest

pytestmark = pytest.mark.skip(
    reason="depends on newaxis (excluded by OSCBF_PORTING_GUIDE.md §4.7); "
           "rate-limit contract is re-evaluated in M7"
)


def test_jax_rate_limit_uses_fixed_slack_without_changing_cbf_qp_shape():
    pytest.importorskip("cbfpy")
    from work.jax_control_loop import JaxControlLoop, MAX_JAX_OBSTACLES

    du_max = np.full(9, 0.01)
    loop = JaxControlLoop(
        dt=0.002, temporal_lambda=0.2,
        rate_limit_du_max=du_max, rate_limit_penalty=1e3)
    loop.init_cbf()
    q = np.array([
        0.2303562, 0.1112539, 1.0167209, -0.6810303, -1.8294025,
        -0.4664294, 0.4743473, -1.0429228, 0.0289233,
    ])
    ee_pos = np.asarray(loop.robot.ee_position(q))
    ee_rot = np.asarray(loop.robot.ee_rotation(q))
    result = loop.tracking_step(
        q=q,
        task_pos=ee_pos + np.array([0.02, 0.0, 0.0]),
        task_vel=np.zeros(3), task_rot=ee_rot, task_omega=np.zeros(3),
        kp_pos=50.0, kp_orient=10.0, kp_joint=0.45, q_des=q,
        nullspace_speed_limit=0.18,
        obs_pos=np.zeros((MAX_JAX_OBSTACLES, 3)),
        obs_radii=np.zeros(MAX_JAX_OBSTACLES),
        obs_enabled=np.zeros(MAX_JAX_OBSTACLES),
        obs_d_safe=np.zeros(MAX_JAX_OBSTACLES),
        obs_vel=np.zeros((MAX_JAX_OBSTACLES, 3)),
        obs_radius_dot=np.zeros(MAX_JAX_OBSTACLES),
        obs_alpha=np.ones(MAX_JAX_OBSTACLES) * 10.0,
        u_safe_prev=np.zeros(9),
    )

    _, u_safe, _, _, _, _, qp_ok, _ = result
    assert qp_ok
    actual_relaxation = max(
        0.0, float(np.max(np.abs(loop.last_qp_candidate) - du_max)))
    assert loop.last_rate_constraint_violation == pytest.approx(actual_relaxation)
    # Compatibility readers keep receiving the physical relaxation, while
    # the raw qpax interior-point coordinate remains diagnostic-only.
    assert loop.last_rate_slack == pytest.approx(actual_relaxation)
    assert loop.last_rate_solver_slack >= -1.0e-6
    assert np.all(np.abs(u_safe) <= du_max + actual_relaxation + 1.0e-4)


def test_solver_slack_is_not_classified_as_a_rate_relaxation():
    """An inactive rate row must not stop solely from qpax centrality."""
    from newaxis.control_safety_state import TRACKING, classify_control_safety_state
    from work.jax_control_loop import JaxControlLoop

    loop = JaxControlLoop(rate_limit_du_max=np.full(9, 0.01))
    loop._update_qp_diagnostics(
        qp_ok=True, min_dist=1.0, min_esdf=1.0,
        rate_constraint_violation=0.0, rate_solver_slack=0.02,
        h_vals=None, cbf_grad=None, active_count=0,
        primal_residual=0.0, terminal_kkt_residual=0.0,
        terminal_kkt_accepted=True, dual_max=0.0, qp_iterations=7)

    assert loop.last_rate_constraint_violation == 0.0
    assert loop.last_rate_solver_slack == pytest.approx(0.02)
    decision = classify_control_safety_state(
        qp_ok=True, rate_slack_rad_s=loop.last_rate_constraint_violation)
    assert decision.state == TRACKING
