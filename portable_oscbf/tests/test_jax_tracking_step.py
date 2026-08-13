#!/usr/bin/env python3
"""Regression tests for the fully-JIT nominal OSC tracking entry point."""

import numpy as np
import pytest


def test_tracking_step_runs_nominal_osc_and_qp_in_one_jitted_entry_point():
    pytest.importorskip("cbfpy")
    from work.jax_control_loop import JaxControlLoop, MAX_JAX_OBSTACLES

    loop = JaxControlLoop(dt=0.002, temporal_lambda=0.2)
    loop.init_cbf()
    assert loop._config.num_obstacle_constraints == loop._config.num_robot_collision_spheres
    # The all-zero folded configuration violates the model's self-collision
    # barrier.  Use the verified tracking start configuration instead.
    q = np.array([
        0.2303562, 0.1112539, 1.0167209, -0.6810303, -1.8294025,
        -0.4664294, 0.4743473, -1.0429228, 0.0289233,
    ])
    ee_pos = np.asarray(loop.robot.ee_position(q))
    ee_rot = np.asarray(loop.robot.ee_rotation(q))
    result = loop.tracking_step(
        q=q,
        task_pos=ee_pos,
        task_vel=np.zeros(3),
        task_rot=ee_rot,
        task_omega=np.zeros(3),
        kp_pos=50.0,
        kp_orient=10.0,
        kp_joint=0.45,
        q_des=q,
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

    q_next, u_safe, u_nom, err_6d, ee_pos_out, ee_rot_out, qp_ok, _ = result
    assert q_next.shape == (9,)
    assert u_safe.shape == (9,)
    assert u_nom.shape == (9,)
    assert err_6d.shape == (6,)
    assert ee_pos_out.shape == (3,)
    assert ee_rot_out.shape == (3, 3)
    assert bool(qp_ok)
    assert np.max(np.abs(np.asarray(u_nom))) < 1.0e-5
    assert loop.last_qp_active_count >= 0
    assert loop.last_qp_iterations >= 0
    assert loop.last_qp_primal_residual >= 0.0
    assert loop.last_qp_terminal_kkt_residual >= 0.0
    assert loop.last_qp_terminal_kkt_accepted
    assert loop.last_qp_dual_max >= 0.0
    # 预热必须覆盖真实的 Python/NumPy 调用约定。否则第一帧控制会在
    # 500 Hz 热路径里重新编译，产生秒级速度命令空窗。
    assert loop._tracking_fn._cache_size() == 1


def test_tracking_fast_path_preserves_safe_command_without_h_gradient_telemetry():
    pytest.importorskip("cbfpy")
    from work.jax_control_loop import JaxControlLoop, MAX_JAX_OBSTACLES

    loop = JaxControlLoop(dt=0.002, temporal_lambda=0.2)
    loop.init_cbf()
    q = np.array([
        0.2303562, 0.1112539, 1.0167209, -0.6810303, -1.8294025,
        -0.4664294, 0.4743473, -1.0429228, 0.0289233,
    ])
    common = dict(
        q=q,
        task_pos=np.asarray(loop.robot.ee_position(q)) + np.array([0.01, 0.0, 0.0]),
        task_vel=np.zeros(3), task_rot=np.asarray(loop.robot.ee_rotation(q)),
        task_omega=np.zeros(3), kp_pos=50.0, kp_orient=10.0,
        kp_joint=0.45, q_des=q, nullspace_speed_limit=0.18,
        obs_pos=np.zeros((MAX_JAX_OBSTACLES, 3)),
        obs_radii=np.zeros(MAX_JAX_OBSTACLES),
        obs_enabled=np.zeros(MAX_JAX_OBSTACLES),
        obs_d_safe=np.zeros(MAX_JAX_OBSTACLES),
        obs_vel=np.zeros((MAX_JAX_OBSTACLES, 3)),
        obs_radius_dot=np.zeros(MAX_JAX_OBSTACLES),
        obs_alpha=np.ones(MAX_JAX_OBSTACLES) * 10.0,
        u_safe_prev=np.zeros(9),
    )
    full_result = loop.tracking_step(**common)
    loop.collect_cbf_diagnostics = False
    fast_result = loop.tracking_step(**common)

    np.testing.assert_allclose(fast_result[1], full_result[1], atol=1e-10)
    assert fast_result[6]
    assert np.isnan(loop.last_cbf_h_delta_norm)
    assert np.isnan(loop.last_cbf_grad_delta_norm)
