"""Adapter contracts: full-path fitting, synchronized sampling, optional time."""

import numpy as np
import pytest
from scipy.io import savemat

from robot_safecontrol_moveit import oscbf_trajectory as trajectory
from robot_safecontrol_moveit.cylinder_geometry import fit_circle


@pytest.fixture
def source_path(tmp_path, monkeypatch):
    angles = np.linspace(0.0, 2.0 * np.pi, 40, endpoint=False)
    # Vary radius so fitting a short prefix would visibly change the answer.
    radii = 1.0 + 0.04 * np.sin(3.0 * angles)
    positions = np.column_stack((
        radii * np.cos(angles), np.linspace(-0.3, 0.8, 40),
        radii * np.sin(angles),
    ))
    times = np.arange(40, dtype=float) ** 2 * 0.01
    path = tmp_path / "trajectory.mat"
    savemat(path, {"ik_input": {
        "position_series": positions * 1000.0, "time_series": times,
    }})
    transform = np.eye(4)
    transform[:3, 3] = [0.2, 0.3, -0.4]
    monkeypatch.setattr(trajectory, "trajectory_to_base_transform", lambda _: transform)
    return path, times


@pytest.mark.parametrize("stride,maximum", [(1, 0), (3, 5), (50, 2), (2, -1), (-1, 0)])
def test_sampling_preserves_full_fit_and_source_time(source_path, stride, maximum):
    path, times = source_path
    full = trajectory.load_calibrated_path(path)
    indices = np.arange(0, len(full), stride)
    if maximum > 0:
        indices = indices[:maximum]
    positions, sampled_times = trajectory.load_calibrated_path_with_times(
        path, point_stride=stride, max_points=maximum,
    )
    np.testing.assert_array_equal(positions, full[indices])
    np.testing.assert_array_equal(sampled_times, times[indices])
    np.testing.assert_array_equal(positions, trajectory.load_calibrated_path(
        path, point_stride=stride, max_points=maximum,
    ))


def test_position_only_input_does_not_require_times(source_path):
    path, _ = source_path
    from scipy.io import loadmat
    data = loadmat(path)["ik_input"][0, 0]
    savemat(path, {"ik_input": {"position_series": data["position_series"]}})
    assert trajectory.load_calibrated_path(path).shape == (40, 3)
    with pytest.raises(ValueError, match="time_series"):
        trajectory.load_calibrated_path_with_times(path)


def test_axis_point_preserves_transverse_center_for_tilted_cylinder():
    axis = np.array([1.0, 2.0, 3.0])
    axis /= np.linalg.norm(axis)
    u = np.cross(axis, [0.0, 1.0, 0.0])
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    center = 0.7 * u - 0.4 * v
    angles = np.linspace(0, 2 * np.pi, 30, endpoint=False)
    points = center + np.cos(angles)[:, None] * u + np.sin(angles)[:, None] * v
    fit = fit_circle(points, axis)
    for coordinate in (-2.0, 0.0, 1.7):
        np.testing.assert_allclose(fit.axis_point(coordinate), center + coordinate * axis,
                                   atol=1e-14)
