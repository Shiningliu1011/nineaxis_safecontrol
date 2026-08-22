#!/usr/bin/env python3
"""Fixed-topology distance-field CBF tests."""

import numpy as np
import pytest


def test_esdf_adds_one_fixed_barrier_per_robot_collision_sphere():
    pytest.importorskip("cbfpy")
    import jax.numpy as jnp

    from work.jax_control_facade import MAX_JAX_OBSTACLES
    from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
    from work.oscbf_velocity_config import NineaxisOSCBFVelocityConfig
    from work.safety_snapshot import SafetyGridSpec, build_distance_field

    robot = NineaxisManipulatorJAX()
    spec = SafetyGridSpec(
        workspace_min=np.array([-1.0, -1.5, -0.5]),
        workspace_max=np.array([1.5, 1.5, 1.8]),
        voxel_size=0.05,
    )
    config = NineaxisOSCBFVelocityConfig(
        robot, sdf_shape=spec.shape, aggregate_dynamic_obstacles=True)
    assert config.num_robot_collision_spheres == 32
    assert config.num_obstacle_constraints == 10
    assert config.num_esdf_constraints == 32
    # M7: obstacle rows are 10 DCOL OBB rows; ESDF keeps 32 sphere rows.
    assert config.num_obstacle_constraints == 10
    assert config.num_cbf == 18 + 14 + 10 + 32 + 1
    q = jnp.array([
        0.2303562, 0.1112539, 1.0167209, -0.6810303, -1.8294025,
        -0.4664294, 0.4743473, -1.0429228, 0.0289233,
    ])
    collision_data = np.asarray(robot.environment_collision_data(q))
    field = build_distance_field(collision_data[:1, :3], spec)
    h = config.h_2(
        q,
        jnp.zeros((MAX_JAX_OBSTACLES, 3)),
        jnp.zeros(MAX_JAX_OBSTACLES),
        jnp.zeros(MAX_JAX_OBSTACLES),
        jnp.zeros(MAX_JAX_OBSTACLES),
        jnp.zeros((MAX_JAX_OBSTACLES, 3)),
        jnp.zeros(MAX_JAX_OBSTACLES),
        jnp.ones(MAX_JAX_OBSTACLES) * config.obstacle_h_baseline_alpha,
        jnp.zeros(robot.num_joints),
        jnp.asarray(field),
        jnp.asarray(spec.workspace_min),
        jnp.asarray(spec.voxel_size),
        jnp.asarray(1.0),
        jnp.asarray(0.03),
    )

    assert config.num_esdf_constraints == config.num_robot_collision_spheres
    assert len(h) == config.esdf_h_stop + 1
    assert float(h[config.esdf_h_start]) < 0.0


def test_jax_loop_accepts_a_fixed_shape_distance_snapshot_without_rejit_shape_change():
    pytest.importorskip("cbfpy")

    from work.jax_control_facade import JaxControlLoop
    from work.safety_snapshot import SafetyGridSpec, build_distance_field

    spec = SafetyGridSpec(
        workspace_min=np.array([-1.0, -1.5, -0.5]),
        workspace_max=np.array([1.5, 1.5, 1.8]),
        voxel_size=0.10,
    )
    loop = JaxControlLoop(dt=0.002, temporal_lambda=0.2, sdf_shape=spec.shape)
    loop.init_cbf()
    assert loop._config.num_cbf == 18 + 14 + 10 + 32 + 1
    q = np.array([
        0.2303562, 0.1112539, 1.0167209, -0.6810303, -1.8294025,
        -0.4664294, 0.4743473, -1.0429228, 0.0289233,
    ])
    first_center = np.asarray(loop.robot.environment_collision_data(q))[0, :3]
    field = build_distance_field(first_center[None, :], spec)
    loop.step(
        q, np.zeros(9), sdf_distance=field,
        sdf_origin=spec.workspace_min, sdf_voxel_size=spec.voxel_size,
        sdf_enabled=1.0, sdf_margin=0.03,
    )
    cache_size_after_first_snapshot = loop._step_fn._cache_size()

    moved_field = build_distance_field((first_center + 0.10)[None, :], spec)
    loop.step(
        q, np.zeros(9), sdf_distance=moved_field,
        sdf_origin=spec.workspace_min + 0.01, sdf_voxel_size=spec.voxel_size,
        sdf_enabled=1.0, sdf_margin=0.03,
    )

    assert loop.last_min_esdf_dist < 0.0
    # The ESDF content and origin may change each perception frame, but their
    # fixed shapes and canonical dtypes must keep the compiled kernel stable.
    assert loop._step_fn._cache_size() == cache_size_after_first_snapshot == 1


def test_path_tracking_reuses_one_cache_for_fixed_shape_esdf_snapshots():
    """The production path entry point must not compile per perception frame."""
    pytest.importorskip("cbfpy")

    from work.jax_control_facade import JaxControlLoop
    from work.path_following import PathFollowingConfig, PathGeometry
    from work.safety_snapshot import SafetyGridSpec, build_distance_field

    q = np.array([
        0.2303562, 0.1112539, 1.0167209, -0.6810303, -1.8294025,
        -0.4664294, 0.4743473, -1.0429228, 0.0289233,
    ])
    spec = SafetyGridSpec(
        workspace_min=np.array([-1.0, -1.5, -0.5]),
        workspace_max=np.array([1.5, 1.5, 1.8]),
        voxel_size=0.10,
    )
    loop = JaxControlLoop(dt=0.002, temporal_lambda=0.2, sdf_shape=spec.shape)
    start = np.asarray(loop.robot.ee_position(q))
    rotation = np.asarray(loop.robot.ee_rotation(q))
    geometry = PathGeometry.from_samples(
        np.stack((start, start + [0.01, 0.0, 0.0],
                  start + [0.02, 0.0, 0.0])),
        np.repeat(rotation[None, :, :], 3, axis=0),
        np.full(3, 0.02), np.array([0.0, 0.5, 1.0]),
    )
    loop.configure_path(geometry, PathFollowingConfig())
    loop.init_cbf()

    common = dict(
        q=q,
        kp_pos=50.0,
        kp_orient=10.0,
        kp_joint=0.45,
        q_des=q,
        nullspace_speed_limit=0.18,
        damping=1e-3,
        sdf_voxel_size=spec.voxel_size,
        sdf_enabled=1.0,
        sdf_margin=0.03,
    )
    first = loop.path_tracking_step(
        path_state=loop.initial_path_state(),
        sdf_distance=build_distance_field((start + [0.30, 0.0, 0.0])[None, :], spec),
        sdf_origin=spec.workspace_min,
        **common,
    )
    cache_size_after_first_snapshot = loop._path_tracking_fn._cache_size()
    second = loop.path_tracking_step(
        path_state=loop.initial_path_state(),
        sdf_distance=build_distance_field((start + [0.40, 0.0, 0.0])[None, :], spec),
        sdf_origin=spec.workspace_min + 0.01,
        **common,
    )

    assert first.qp_ok
    assert second.qp_ok
    assert loop._path_tracking_fn._cache_size() == cache_size_after_first_snapshot == 1
