#!/usr/bin/env python3
"""M4 acceptance: repository trajectory -> arc-length PathGeometry."""

from __future__ import annotations

from pathlib import Path
from time import perf_counter

import _path_setup  # noqa: F401
import numpy as np
import pytest

from work.ik_data_loader import load_repository_trajectory
from work.jax_control_facade import JaxControlLoop
from work.jax_path_following import PATH_STATE_SIZE
from work.path_following import PathFollowingConfig


REPO_ROOT = Path(__file__).resolve().parents[2]
CURRENT_TRAJECTORY = REPO_ROOT / "data" / "nurbs" / "ik_input.mat"
REFERENCE_TRAJECTORY = (
    Path(__file__).resolve().parents[1] / "data" / "ik_input.mat")


def _load(mat_path: Path):
    assert mat_path.is_file(), f"trajectory not found: {mat_path}"
    started = perf_counter()
    data = load_repository_trajectory(str(mat_path))
    geometry = data.path_geometry()
    elapsed = perf_counter() - started
    return data, geometry, elapsed


def test_current_trajectory_loads_with_expected_metadata():
    data, geometry, elapsed = _load(CURRENT_TRAJECTORY)

    assert data.num_points == 14992
    assert data.Ts == pytest.approx(0.002)
    assert data.num_blocks == 23
    assert geometry.num_points >= 2
    assert geometry.total_length_m > 0.0
    assert elapsed < 5.0, f"trajectory load took {elapsed:.2f}s (limit 5s)"


def test_path_geometry_is_finite_and_continuous():
    _, geometry, _ = _load(CURRENT_TRAJECTORY)

    positions = geometry.positions_m
    for name, values in (
        ("positions_m", positions),
        ("rotations", geometry.rotations),
        ("tangents", geometry.tangents),
        ("omega_per_m", geometry.omega_per_m),
        ("feedrate_m_s", geometry.feedrate_m_s),
        ("arc_length_m", geometry.arc_length_m),
    ):
        assert np.all(np.isfinite(values)), f"{name} contains NaN/Inf"

    segment_lengths = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    assert np.max(segment_lengths) < 1e-3, (
        f"adjacent path samples jump {np.max(segment_lengths) * 1000:.3f} mm")


def test_arc_length_is_strictly_increasing():
    _, geometry, _ = _load(CURRENT_TRAJECTORY)
    differences = np.diff(geometry.arc_length_m)
    assert np.all(differences > 0.0)
    assert geometry.arc_length_m[0] == 0.0


def test_path_state_contract_through_facade():
    _, geometry, _ = _load(CURRENT_TRAJECTORY)
    loop = JaxControlLoop(dt=0.002, enable_x64=True)
    loop.configure_path(geometry, PathFollowingConfig())
    path_state = loop.initial_path_state()
    assert path_state.shape == (PATH_STATE_SIZE,)
    assert np.all(np.isfinite(path_state))
    np.testing.assert_allclose(path_state, np.zeros(PATH_STATE_SIZE))


def test_reference_trajectory_still_loads_for_regression():
    data, geometry, _ = _load(REFERENCE_TRAJECTORY)
    assert data.num_points > 1000
    assert geometry.num_points >= 2
    assert geometry.total_length_m > 0.0
    assert np.all(np.isfinite(geometry.positions_m))


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
