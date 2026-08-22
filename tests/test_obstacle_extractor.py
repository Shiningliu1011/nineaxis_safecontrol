"""解析障碍物提取器测试：球/圆柱拟合、聚类分析、tracks 槽→obs_* 契约解码。

全部为合成点云（非物理数据），沉淀形状拟合与契约的数值基线。
"""

from dataclasses import dataclass

import numpy as np
import pytest

from robot_safecontrol_moveit.obstacle_extractor import (
    MAX_OBSTACLE_SLOTS,
    ObsArrays,
    ObstacleShape,
    analyze_cloud,
    fit_cylinder,
    fit_sphere,
    tracks_slots_to_obs_arrays,
)


@dataclass(frozen=True)
class FakeSpec:
    """与 SafetyGridSpec duck-typed 的最小 spec（.shape/.workspace_min/.voxel_size）。"""

    workspace_min: np.ndarray
    workspace_max: np.ndarray
    voxel_size: float

    @property
    def shape(self):
        ext = np.asarray(self.workspace_max) - np.asarray(self.workspace_min)
        return tuple(np.ceil(ext / self.voxel_size).astype(int) + 1)


def _spec():
    return FakeSpec(
        workspace_min=np.array([-1.0, -1.0, -0.5]),
        workspace_max=np.array([1.5, 1.5, 1.5]),
        voxel_size=0.02,
    )


def _sphere_points(center, radius, n=600, seed=0):
    rng = np.random.default_rng(seed)
    unit = rng.normal(size=(n, 3))
    unit = unit / np.linalg.norm(unit, axis=1, keepdims=True)
    return unit * radius + np.asarray(center)


def _cylinder_points(center, axis, radius, half_length, n=600, seed=0):
    rng = np.random.default_rng(seed)
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    # 构造与轴正交的基
    perp = np.cross(axis, [0.0, 0.0, 1.0])
    if np.linalg.norm(perp) < 1e-6:
        perp = np.cross(axis, [1.0, 0.0, 0.0])
    perp = perp / np.linalg.norm(perp)
    b2 = np.cross(axis, perp)
    theta = rng.uniform(0.0, 2.0 * np.pi, n)
    t = rng.uniform(-half_length, half_length, n)
    pts = (np.outer(np.cos(theta), perp) + np.outer(np.sin(theta), b2)) * radius
    pts = pts + np.outer(t, axis) + np.asarray(center)
    return pts


class TestFitSphere:
    def test_recovers_sphere(self) -> None:
        fit = fit_sphere(_sphere_points([0.5, -0.2, 1.0], 0.2))
        assert fit is not None
        assert np.linalg.norm(fit.center - [0.5, -0.2, 1.0]) < 0.02
        assert abs(fit.radius - 0.2) < 0.02
        assert fit.residual_ms < 0.02

    def test_degenerate_rejected(self) -> None:
        assert fit_sphere(np.zeros((3, 3))) is None


class TestFitCylinder:
    def test_recovers_vertical_cylinder(self) -> None:
        fit = fit_cylinder(_cylinder_points([0.0, 0.0, 0.5], [0, 0, 1], 0.05, 0.2))
        assert fit is not None
        assert np.linalg.norm(fit.centroid - [0.0, 0.0, 0.5]) < 0.02
        assert abs(abs(fit.axis[2]) - 1.0) < 1e-3
        assert abs(fit.radius - 0.05) < 0.01
        assert abs(fit.half_length - 0.2) < 0.02

    def test_cylinder_envelope_geometry(self) -> None:
        # 包络球 = 半径与半长度正交组合
        fit = fit_cylinder(_cylinder_points([0.0, 0.0, 0.5], [0, 0, 1], 0.05, 0.2))
        envelope = np.sqrt(fit.radius**2 + fit.half_length**2)
        assert envelope == pytest.approx(np.sqrt(0.0025 + 0.04), rel=1e-2)


class TestAnalyzeCloud:
    def test_single_sphere_cluster(self) -> None:
        shapes = analyze_cloud(_sphere_points([0.5, 0.0, 1.0], 0.15), _spec())
        assert len(shapes) == 1
        shape = shapes[0]
        assert shape.kind == "sphere"
        assert np.linalg.norm(shape.center - [0.5, 0.0, 1.0]) < 0.03
        assert abs(shape.envelope_radius - 0.15) < 0.03

    def test_cylinder_classified_and_enveloped(self) -> None:
        shapes = analyze_cloud(
            _cylinder_points([0.0, 0.0, 0.5], [0, 0, 1], 0.045, 0.28), _spec())
        assert len(shapes) == 1
        shape = shapes[0]
        assert shape.kind == "cylinder"
        assert shape.axis is not None
        assert abs(abs(shape.axis[2]) - 1.0) < 1e-3
        # 包络球含半长度：sqrt(0.045² + 0.28²) ≈ 0.284
        assert shape.envelope_radius == pytest.approx(np.sqrt(0.045**2 + 0.28**2), rel=0.05)

    def test_two_clusters_sorted_by_size(self) -> None:
        big = _sphere_points([0.5, 0.0, 1.0], 0.15, n=800, seed=1)
        small = _sphere_points([-0.5, 0.5, 0.2], 0.05, n=200, seed=2)
        shapes = analyze_cloud(np.vstack([big, small]), _spec())
        assert len(shapes) == 2
        assert shapes[0].n_points > shapes[1].n_points

    def test_empty_and_nan_safe(self) -> None:
        assert analyze_cloud(np.zeros((0, 3)), _spec()) == []
        assert analyze_cloud(
            np.array([[np.nan, 0.0, 0.0], [0.0, 0.0, 0.0]]), _spec()) == []

    def test_too_few_points(self) -> None:
        pts = np.random.default_rng(0).uniform(-1, 1, size=(3, 3))
        assert analyze_cloud(pts, _spec(), min_points=4) == []


def _slot(i, px, py, pz, r, vx=0.0, vy=0.0, vz=0.0, enabled=1.0, d_safe=0.02, alpha=1.5):
    return np.array([px, py, pz, r, vx, vy, vz, enabled, d_safe, alpha])


class TestTrackContract:
    def test_slots_decode_to_obs_arrays(self) -> None:
        slots = np.zeros((MAX_OBSTACLE_SLOTS, 10))
        slots[0] = _slot(0, 0.1, 0.2, 0.3, 0.1)
        slots[2] = _slot(2, 0.4, 0.5, 0.6, 0.2, vx=0.01, alpha=2.0)
        obs = tracks_slots_to_obs_arrays(slots)
        assert isinstance(obs, ObsArrays)
        assert obs.pos.shape == (MAX_OBSTACLE_SLOTS, 3)
        assert np.allclose(obs.pos[0], [0.1, 0.2, 0.3])
        assert obs.radii[0] == 0.1
        assert obs.enabled[0] == 1.0 and obs.enabled[1] == 0.0
        assert np.allclose(obs.vel[2], [0.01, 0.0, 0.0])
        assert obs.d_safe[0] == 0.02
        assert obs.alpha[2] == 2.0 and obs.alpha[0] == 1.5

    def test_radius_dot_from_history(self) -> None:
        slots = np.zeros((MAX_OBSTACLE_SLOTS, 10))
        slots[0] = _slot(0, 0.0, 0.0, 0.0, 0.10)
        obs = tracks_slots_to_obs_arrays(
            slots, dt_s=0.1, prev_radii=np.array([0.08] + [0.0] * 7))
        assert obs.radius_dot[0] == pytest.approx((0.10 - 0.08) / 0.1, abs=1e-9)
        assert obs.radius_dot[1] == 0.0

    def test_flat_input_accepted(self) -> None:
        slots = np.zeros(MAX_OBSTACLE_SLOTS * 10)
        slots[0:10] = _slot(0, 1.0, 1.0, 1.0, 0.05)
        obs = tracks_slots_to_obs_arrays(slots)
        assert np.allclose(obs.pos[0], [1.0, 1.0, 1.0])

    def test_bad_shape_rejected(self) -> None:
        with pytest.raises(ValueError):
            tracks_slots_to_obs_arrays(np.zeros((3, 7)))
