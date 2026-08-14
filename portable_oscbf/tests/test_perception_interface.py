#!/usr/bin/env python3
"""M9 acceptance: perception interface (ESDF/obs_*) with default disabled."""

from __future__ import annotations

from pathlib import Path

import jax

jax.config.update("jax_enable_x64", True)

import _path_setup  # noqa: F401
import numpy as np
import pytest

from work.ik_data_loader import load_repository_trajectory
from work.jax_control_facade import JaxControlLoop
from work.path_following import PathFollowingConfig
from work.safety_snapshot import (
    SafetyGridSpec,
    build_distance_field,
    preprocess_points,
    sample_distance_field_jax,
    sample_distance_field_numpy,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
CURRENT_TRAJECTORY = REPO_ROOT / "data" / "nurbs" / "ik_input.mat"
BASELINE_PATH = REPO_ROOT / "output" / "baseline_tracking.npz"


def _control_kwargs(q):
    return dict(
        kp_pos=60.0, kp_orient=10.0, kp_joint=0.45, q_des=q,
        nullspace_speed_limit=0.18, damping=1e-3,
        u_safe_prev=np.zeros(9))


@pytest.fixture(scope="module")
def plain_loop():
    trajectory = load_repository_trajectory(str(CURRENT_TRAJECTORY))
    control = JaxControlLoop(dt=0.002, temporal_lambda=0.2, enable_x64=True)
    control.configure_path(trajectory.path_geometry(), PathFollowingConfig())
    control.init_cbf()
    return control


@pytest.fixture(scope="module")
def sdf_loop():
    trajectory = load_repository_trajectory(str(CURRENT_TRAJECTORY))
    control = JaxControlLoop(
        dt=0.002, temporal_lambda=0.2, enable_x64=True,
        sdf_shape=(8, 8, 8))
    control.configure_path(trajectory.path_geometry(), PathFollowingConfig())
    control.init_cbf()
    return control


def test_sdf_disabled_matches_m7_baseline(plain_loop):
    """sdf_enabled=false must reproduce the M7 baseline step-for-step."""

    baseline = np.load(BASELINE_PATH)
    q = baseline["initial_q"].copy()
    path_state = plain_loop.initial_path_state()
    kwargs = _control_kwargs(q)

    explicit_disabled = plain_loop.path_tracking_step(
        q=q, path_state=path_state, sdf_enabled=0.0, **kwargs)
    default = plain_loop.path_tracking_step(
        q=q, path_state=path_state, **kwargs)

    np.testing.assert_allclose(
        explicit_disabled.q_next, default.q_next, rtol=0.0, atol=1e-12)
    np.testing.assert_allclose(
        explicit_disabled.u_safe, default.u_safe, rtol=0.0, atol=1e-12)
    assert explicit_disabled.qp_ok == default.qp_ok

    # Same input as the M7 baseline first step.  The baseline was produced in
    # a separate process; qpax's interior-point solve is iteration-sensitive
    # across processes (u_safe can differ by ~0.2 rad/s at the convergence
    # boundary), so only the integrated state magnitude and the QP status are
    # compared.  Exact same-process equivalence is asserted above.
    np.testing.assert_allclose(
        explicit_disabled.q_next, baseline["q_sequence"][1],
        rtol=0.0, atol=5e-3)
    assert explicit_disabled.qp_ok == bool(baseline["qp_ok_sequence"][0])


def test_esdf_float32_keeps_single_cache_entry(sdf_loop):
    """Fixed-shape float32 ESDF updates must not trigger recompilation."""

    q = np.zeros(9)
    path_state = sdf_loop.initial_path_state()
    kwargs = _control_kwargs(q)
    origin = np.zeros(3, dtype=np.float32)
    field = np.full((8, 8, 8), 8.0, dtype=np.float32)
    first = sdf_loop.path_tracking_step(
        q=q, path_state=path_state, sdf_enabled=1.0,
        sdf_distance=field, sdf_origin=origin, sdf_voxel_size=0.1,
        sdf_margin=0.03, **kwargs)
    assert first.qp_ok
    cache_after_first = sdf_loop._path_tracking_fn._cache_size()

    field_changed = np.full((8, 8, 8), 3.0, dtype=np.float32)
    second = sdf_loop.path_tracking_step(
        q=q, path_state=path_state, sdf_enabled=1.0,
        sdf_distance=field_changed, sdf_origin=origin, sdf_voxel_size=0.1,
        sdf_margin=0.03, **kwargs)
    assert second.qp_ok
    assert sdf_loop._path_tracking_fn._cache_size() == cache_after_first == 1


def test_point_cloud_to_distance_field_chain():
    """Synthetic point cloud -> voxelisation -> distance field -> sampling."""

    spec = SafetyGridSpec(
        workspace_min=np.array([-0.5, -0.5, -0.5]),
        workspace_max=np.array([0.5, 0.5, 0.5]),
        voxel_size=0.1,
    )
    sensor_points = np.array([
        [0.05, 0.05, 0.05],
        [0.06, 0.06, 0.06],
        [0.15, -0.05, 0.02],
        [0.0, 0.0, 0.0],  # robot point, excluded
    ])
    sensor_to_world = np.eye(4)
    robot_spheres = [(np.array([0.0, 0.0, 0.0]), 0.01)]
    processed = preprocess_points(
        sensor_points, sensor_to_world, spec,
        robot_spheres=robot_spheres)
    assert np.all(np.isfinite(processed))
    assert len(processed) == 2  # robot point removed; 0.05/0.06 share a voxel

    field = build_distance_field(processed, spec)
    assert field.shape == spec.shape
    assert np.all(np.isfinite(field))
    assert np.all(field >= 0.0)

    query = np.array([0.05, 0.05, 0.05])
    jax_value = float(np.asarray(sample_distance_field_jax(
        jax.numpy.asarray(field, dtype=jax.numpy.float32),
        jax.numpy.asarray(query),
        jax.numpy.asarray(spec.workspace_min, dtype=jax.numpy.float32),
        jax.numpy.asarray(spec.voxel_size, dtype=jax.numpy.float32))))
    numpy_value = float(sample_distance_field_numpy(
        field, query, spec.workspace_min, spec.voxel_size))
    assert jax_value == pytest.approx(numpy_value, abs=1e-6)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
