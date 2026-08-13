#!/usr/bin/env python3
"""Regression tests for fixed-topology dynamic-primitive aggregation."""

import numpy as np
import pytest


def test_smooth_dynamic_barrier_has_one_conservative_row_per_robot_sphere():
    pytest.importorskip("cbfpy")
    import jax.numpy as jnp

    from work.jax_control_loop import MAX_JAX_OBSTACLES
    from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
    from work.oscbf_velocity_config import NineaxisOSCBFVelocityConfig

    robot = NineaxisManipulatorJAX()
    config = NineaxisOSCBFVelocityConfig(
        robot, aggregate_dynamic_obstacles=True)
    q = jnp.array([
        0.2303562, 0.1112539, 1.0167209, -0.6810303, -1.8294025,
        -0.4664294, 0.4743473, -1.0429228, 0.0289233,
    ])
    collision_data = np.asarray(robot.environment_collision_data(q))
    first_center = collision_data[0, :3]
    obs_pos = np.zeros((MAX_JAX_OBSTACLES, 3), dtype=np.float32)
    obs_pos[0] = first_center + np.array([0.08, 0.0, 0.0])
    obs_pos[1] = first_center + np.array([0.14, 0.0, 0.0])
    obs_radii = np.zeros(MAX_JAX_OBSTACLES, dtype=np.float32)
    enabled = np.zeros(MAX_JAX_OBSTACLES, dtype=np.float32)
    enabled[:2] = 1.0
    h = config.h_2(
        q, jnp.asarray(obs_pos), jnp.asarray(obs_radii), jnp.asarray(enabled),
        jnp.zeros(MAX_JAX_OBSTACLES),
        jnp.zeros((MAX_JAX_OBSTACLES, 3)),
        jnp.zeros(MAX_JAX_OBSTACLES),
        jnp.ones(MAX_JAX_OBSTACLES) * config.obstacle_h_baseline_alpha,
        jnp.zeros(robot.num_joints),
    )

    obstacle_h = h[config.obstacle_h_start:config.obstacle_h_stop]
    expected_first_min = min(
        0.08 - collision_data[0, 3],
        0.14 - collision_data[0, 3],
    )
    assert config.num_obstacle_constraints == config.num_robot_collision_spheres
    assert obstacle_h.shape == (config.num_robot_collision_spheres,)
    # soft-min is intentionally no larger than the exact minimum, so it is
    # conservative when a tracked primitive enters or leaves a slot.
    assert float(obstacle_h[0]) <= expected_first_min + 1.0e-6
