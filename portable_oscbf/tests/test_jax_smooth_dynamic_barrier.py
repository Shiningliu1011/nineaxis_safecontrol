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
    # The M2/M7 obstacle kernel aggregates to one conservative row per OBB
    # link, not per mesh-envelope collision sphere.  Compare the soft-min of
    # the first link against the exact per-slot DCOL clearances feeding it.
    assert config.num_obstacle_constraints == config.num_obb_links
    assert obstacle_h.shape == (config.num_obstacle_constraints,)

    from work.jax_barrier_terms import compute_dcol_obstacle_clearance
    h_obs, _ = compute_dcol_obstacle_clearance(
        q,
        jnp.asarray(obs_pos, dtype=q.dtype),
        jnp.asarray(obs_radii, dtype=q.dtype),
        jnp.zeros(MAX_JAX_OBSTACLES, dtype=q.dtype),        # obs_d_safe
        jnp.zeros((MAX_JAX_OBSTACLES, 3), dtype=q.dtype),   # obs_vel
        jnp.zeros(MAX_JAX_OBSTACLES, dtype=q.dtype),        # obs_radius_dot
    )
    expected_first_min = float(jnp.min(h_obs[0, :2]))
    # soft-min is intentionally no larger than the exact minimum, so it is
    # conservative when a tracked primitive enters or leaves a slot.
    assert float(obstacle_h[0]) <= expected_first_min + 1.0e-6
