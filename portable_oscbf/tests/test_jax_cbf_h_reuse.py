#!/usr/bin/env python3
"""The CBF value used for diagnostics must be recoverable from QP data."""

import numpy as np
import pytest


def test_cbf_raw_h_matches_unmodified_qp_rhs_divided_by_alpha():
    pytest.importorskip("cbfpy")
    import jax.numpy as jnp
    from cbfpy import CBF

    from work.jax_control_facade import MAX_JAX_OBSTACLES
    from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
    from work.oscbf_velocity_config import NineaxisOSCBFVelocityConfig

    robot = NineaxisManipulatorJAX()
    config = NineaxisOSCBFVelocityConfig(
        robot, temporal_lambda=0.2, aggregate_dynamic_obstacles=True)
    cbf = CBF.from_config(config)
    q = jnp.array([
        0.2303562, 0.1112539, 1.0167209, -0.6810303, -1.8294025,
        -0.4664294, 0.4743473, -1.0429228, 0.0289233,
    ])
    args = (
        jnp.zeros((MAX_JAX_OBSTACLES, 3)),
        jnp.zeros(MAX_JAX_OBSTACLES),
        jnp.zeros(MAX_JAX_OBSTACLES),
        jnp.zeros(MAX_JAX_OBSTACLES),
        jnp.zeros((MAX_JAX_OBSTACLES, 3)),
        jnp.zeros(MAX_JAX_OBSTACLES),
        jnp.ones(MAX_JAX_OBSTACLES) * config.obstacle_h_baseline_alpha,
        jnp.zeros(robot.num_joints),
        jnp.full((2, 2, 2), 10.0),
        jnp.zeros(3),
        jnp.asarray(1.0),
        jnp.asarray(0.0),
        jnp.asarray(config.d_safe_collision),
        jnp.eye(robot.num_joints),
    )
    _, _, _, _, _, qp_h = cbf.qp_data(q, jnp.zeros(robot.num_joints), *args)
    raw_h = cbf.h(q, *args)

    assert np.allclose(
        qp_h[:cbf.num_cbf] / config.obstacle_h_baseline_alpha,
        raw_h,
        atol=1.0e-6,
    )

