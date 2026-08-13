#!/usr/bin/env python3
"""感知管线纯计算单测: 配置加载 / 静态-动态分离 / 动态聚类。

不依赖 ROS。直接验证 portable_oscbf/work/ 里的感知模块。
"""

from __future__ import annotations

import _path_setup  # noqa: F401

import numpy as np
import pytest

from work.dynamic_clustering import cluster_into_tracks, empty_track_state
from work.perception_config import load_point_cloud_collision, spec_of
from work.safety_snapshot import MAX_DYNAMIC_TRACKS
from work.static_occupancy import StaticOccupancyTracker


@pytest.fixture(scope="module")
def spec():
    cfg = load_point_cloud_collision()
    return spec_of(cfg)


def _points_in(center, half_extent, n, seed=0):
    rng = np.random.default_rng(seed)
    return np.column_stack([
        rng.uniform(center[0] - half_extent, center[0] + half_extent, n),
        rng.uniform(center[1] - half_extent, center[1] + half_extent, n),
        rng.uniform(center[2] - half_extent, center[2] + half_extent, n),
    ])


# ---------------------------------------------------------------------------
# 配置加载 (obstacle_params.yaml 修正后的默认值)
# ---------------------------------------------------------------------------

def test_config_defaults_are_driver_correct():
    """source_topic / input_frame 必须与驱动实际话题/帧一致。"""
    cfg = load_point_cloud_collision()
    assert cfg.source_topic == "/camera/depth_registered/points"
    assert cfg.input_frame == "camera_color_optical_frame"
    assert cfg.world_frame == "base_link"
    assert cfg.max_points > 0 and cfg.voxel_size > 0.0


def test_spec_shape_positive(spec):
    shape = spec.shape
    assert len(shape) == 3
    assert all(s >= 2 for s in shape)


# ---------------------------------------------------------------------------
# 静态/动态分离
# ---------------------------------------------------------------------------

def test_persistent_points_become_static(spec):
    tracker = StaticOccupancyTracker(spec, keep_frames=8)
    env = _points_in(np.array([0.0, 0.4, 1.3]), 0.1, 800)
    static = dynamic = None
    for _ in range(12):
        static, dynamic = tracker.update(env)
    # 持续存在 -> 全部进 ESDF (静态)。
    assert len(static) > 0
    assert len(dynamic) == 0
    assert static.shape[1] == 3


def test_transient_points_stay_dynamic(spec):
    tracker = StaticOccupancyTracker(spec, keep_frames=8)
    env = _points_in(np.array([0.0, 0.4, 1.3]), 0.1, 800)
    person = _points_in(np.array([0.0, 0.5, 1.2]), 0.05, 300)
    # 人只出现 3 帧 (不足 keep_frames=8), 其余帧消失。
    static = dynamic = None
    for f in range(12):
        frame = np.vstack([env, person]) if f < 3 else env
        static, dynamic = tracker.update(frame)
    # 环境进 ESDF; 人已消失 -> dynamic 应为空。
    assert len(static) > 0
    assert len(dynamic) == 0
    # 人在场的那几帧应被标记为 dynamic。
    tracker2 = StaticOccupancyTracker(spec, keep_frames=8)
    for f in range(3):
        _, dynamic = tracker2.update(np.vstack([env, person]))
    assert len(dynamic) > 0


def test_dynamic_excludes_ramped_static(spec):
    tracker = StaticOccupancyTracker(spec, keep_frames=8)
    env = _points_in(np.array([0.0, 0.4, 1.3]), 0.1, 800)
    for _ in range(10):
        tracker.update(env)  # 先让环境变成静态
    _, dynamic = tracker.update(env)  # 环境仍在, 应无动态
    assert len(dynamic) == 0


# ---------------------------------------------------------------------------
# 动态聚类 → 固定 8 槽
# ---------------------------------------------------------------------------

def test_cluster_output_shapes(spec):
    pts = _points_in(np.array([0.0, 0.5, 1.2]), 0.06, 500)
    prev = empty_track_state(MAX_DYNAMIC_TRACKS)
    state, next_id = cluster_into_tracks(pts, prev, spec, next_id=1)
    assert state.pos.shape == (MAX_DYNAMIC_TRACKS, 3)
    assert state.radii.shape == (MAX_DYNAMIC_TRACKS,)
    assert state.vel.shape == (MAX_DYNAMIC_TRACKS, 3)
    assert state.enabled.shape == (MAX_DYNAMIC_TRACKS,)
    assert state.ids.shape == (MAX_DYNAMIC_TRACKS,)
    assert state.enabled[0] > 0.0
    assert state.radii[0] > 0.0
    assert next_id > 1


def test_cluster_velocity_from_finite_difference(spec):
    """同一团块跨帧移动 → 速度 = 位移/dt。"""
    pts1 = _points_in(np.array([-0.1, 0.5, 1.2]), 0.04, 400)
    pts2 = _points_in(np.array([0.1, 0.5, 1.2]), 0.04, 400)
    prev, nid = cluster_into_tracks(pts1, empty_track_state(8), spec, next_id=1)
    state, _ = cluster_into_tracks(pts2, prev, spec, dt_s=0.1, next_id=nid)
    assert state.enabled[0] > 0.0
    # 期望速度 ≈ (0.1 - (-0.1)) / 0.1 = 2 m/s 沿 x。
    assert abs(state.vel[0, 0] - 2.0) < 0.6
    # 同一 id 持续 (关联成功)。
    assert state.ids[0] == prev.ids[0]


def test_cluster_respects_max_tracks(spec):
    """4 个分离团块 → 只输出 4 个激活槽 (≤ MAX_DYNAMIC_TRACKS)。"""
    centers = [np.array([-0.3, 0.5, 1.2]), np.array([0.0, 0.5, 1.2]),
               np.array([0.3, 0.5, 1.2]), np.array([0.0, 0.6, 1.4])]
    pts = np.vstack([_points_in(c, 0.03, 300, seed=i) for i, c in enumerate(centers)])
    prev = empty_track_state(MAX_DYNAMIC_TRACKS)
    state, _ = cluster_into_tracks(pts, prev, spec, next_id=1)
    assert int(state.enabled.sum()) == 4
    assert int(state.enabled.sum()) <= MAX_DYNAMIC_TRACKS


def test_empty_input_produces_all_disabled(spec):
    state, nid = cluster_into_tracks(
        np.empty((0, 3)), empty_track_state(8), spec, next_id=1)
    assert np.all(state.enabled == 0.0)
    assert nid == 1
