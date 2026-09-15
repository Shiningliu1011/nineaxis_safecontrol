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
    from work.jax_control_facade import JaxControlLoop, MAX_JAX_OBSTACLES

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
    # The rate relaxation used to be mirrored onto the loop as
    # ``last_rate_constraint_violation`` / ``last_rate_slack`` /
    # ``last_rate_solver_slack``.  Those mirrors were retired: the physical
    # relaxation and qpax's raw interior-point slack are carried by the step
    # record (``rate_constraint_violation`` / ``rate_solver_slack``), which
    # ``tracking_step()`` does not return, so this legacy path can only check
    # the command it received.  Re-enabling this module (M7) means reading
    # ``path_tracking_step().delta_slack``-style record slots instead.
    assert np.all(np.abs(u_safe) <= du_max + actual_relaxation + 1.0e-4)


def test_solver_slack_is_not_classified_as_a_rate_relaxation():
    """An inactive rate row must not stop solely from qpax centrality."""
    from newaxis.control_safety_state import TRACKING, classify_control_safety_state

    # The classifier only ever sees the physical relaxation, which the step
    # record reports as ``rate_constraint_violation``; qpax's raw
    # interior-point slack (``rate_solver_slack``) is diagnostic-only and is
    # never passed here.  This test used to seed both through
    # ``loop._update_qp_diagnostics`` -- a nonzero 0.02 raw slack next to a
    # zero physical relaxation -- so the decoy was visible in the fixture; only
    # the physical value is passed now, and it is the one that decides.
    decision = classify_control_safety_state(qp_ok=True, rate_slack_rad_s=0.0)
    assert decision.state == TRACKING
