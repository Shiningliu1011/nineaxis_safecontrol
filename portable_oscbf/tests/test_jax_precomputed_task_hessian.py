#!/usr/bin/env python3
"""Task-consistent Hessian reuse must preserve the OSCBF QP objective."""

import numpy as np
import pytest


def test_config_uses_supplied_task_hessian_for_p_and_q():
    pytest.importorskip("cbfpy")
    import jax.numpy as jnp

    from work.jax_control_facade import MAX_JAX_OBSTACLES
    from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
    from work.oscbf_velocity_config import NineaxisOSCBFVelocityConfig

    robot = NineaxisManipulatorJAX()
    config = NineaxisOSCBFVelocityConfig(robot, temporal_lambda=0.2)
    q = jnp.zeros(robot.num_joints)
    u_des = jnp.linspace(-0.2, 0.2, robot.num_joints)
    u_prev = jnp.linspace(0.1, -0.1, robot.num_joints)
    supplied_p_task = 3.0 * jnp.eye(robot.num_joints)
    args = (
        jnp.zeros((MAX_JAX_OBSTACLES, 3)),
        jnp.zeros(MAX_JAX_OBSTACLES),
        jnp.zeros(MAX_JAX_OBSTACLES),
        jnp.zeros(MAX_JAX_OBSTACLES),
        jnp.zeros((MAX_JAX_OBSTACLES, 3)),
        jnp.zeros(MAX_JAX_OBSTACLES),
        jnp.ones(MAX_JAX_OBSTACLES) * config.obstacle_h_baseline_alpha,
        u_prev,
        jnp.full((2, 2, 2), 10.0),
        jnp.zeros(3),
        jnp.asarray(1.0),
        jnp.asarray(0.0),
        jnp.asarray(config.d_safe_collision),
        supplied_p_task,
    )

    p_actual = config.P(q, u_des, *args)
    q_actual = config.q(q, u_des, *args)
    temporal_diag = config.temporal_lambda * jnp.diag(config.temporal_wu ** 2)

    assert np.allclose(p_actual, supplied_p_task + temporal_diag)
    assert np.allclose(
        q_actual,
        -supplied_p_task @ u_des
        - config.temporal_lambda * config.temporal_wu ** 2 * u_prev,
    )

