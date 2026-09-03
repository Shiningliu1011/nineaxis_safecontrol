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
from work.static_occupancy import OccupancyTracker


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
# 三层占据分离: 连续占据升格为 static
# ---------------------------------------------------------------------------

def test_persistent_points_become_static(spec):
    """持续占据 >= static_confirm_s → 全部进 static 层。"""
    tracker = OccupancyTracker(spec, occupancy_timeout_s=2.0, static_confirm_s=3.0)
    env = _points_in(np.array([0.0, 0.4, 1.3]), 0.1, 800)
    static = unconfirmed = instant = None
    dt = 0.5  # 每帧间隔 0.5s, 12 帧 = 6s > static_confirm_s=3.0
    for i in range(12):
        static, unconfirmed, instant = tracker.update(env, stamp_s=i * dt)
    # 持续存在 -> 全部进 static (ESDF)。
    assert len(static) > 0
    assert len(unconfirmed) == 0
    assert static.shape[1] == 3
    # instant 始终返回全部有效点。
    assert len(instant) > 0


def test_transient_points_stay_unconfirmed(spec):
    """短暂出现 (< static_confirm_s) → 停留在 unconfirmed 层。"""
    tracker = OccupancyTracker(spec, occupancy_timeout_s=2.0, static_confirm_s=5.0)
    env = _points_in(np.array([0.0, 0.4, 1.3]), 0.1, 800)
    person = _points_in(np.array([0.0, 0.5, 1.2]), 0.05, 300)
    # 人只出现 1.5s (3 帧 × 0.5s), 不足 static_confirm_s=5.0。
    dt = 0.5
    static = unconfirmed = instant = None
    for f in range(12):
        frame = np.vstack([env, person]) if f < 3 else env
        static, unconfirmed, instant = tracker.update(frame, stamp_s=f * dt)
    # 环境进 static; 人已消失 -> unconfirmed 应为空。
    assert len(static) > 0
    assert len(unconfirmed) == 0
    # 人在场的那几帧应被标记为 unconfirmed。
    tracker2 = OccupancyTracker(spec, occupancy_timeout_s=2.0, static_confirm_s=5.0)
    for f in range(3):
        _, unconfirmed, _ = tracker2.update(np.vstack([env, person]), stamp_s=f * dt)
    assert len(unconfirmed) > 0


def test_unconfirmed_excludes_ramped_static(spec):
    """已达 static 阈值的体素不出现在 unconfirmed 层。"""
    tracker = OccupancyTracker(spec, occupancy_timeout_s=2.0, static_confirm_s=3.0)
    env = _points_in(np.array([0.0, 0.4, 1.3]), 0.1, 800)
    dt = 0.5
    for i in range(10):
        tracker.update(env, stamp_s=i * dt)  # 先让环境变成 static (5s > 3s)
    _, unconfirmed, _ = tracker.update(env, stamp_s=10 * dt)  # 环境仍在, 应无 unconfirmed
    assert len(unconfirmed) == 0


# ---------------------------------------------------------------------------
# 三层占据分离: 占据中断后重新计时
# ---------------------------------------------------------------------------

def test_gap_resets_timing(spec):
    """占据中断 → first_seen 重置, 下次重新计时。"""
    tracker = OccupancyTracker(spec, occupancy_timeout_s=2.0, static_confirm_s=3.0)
    env = _points_in(np.array([0.0, 0.4, 1.3]), 0.1, 800)
    dt = 0.5

    # 前 4 帧占据 (2s < 3s, 还是 unconfirmed)。
    for i in range(4):
        tracker.update(env, stamp_s=i * dt)

    # 中间跳 2 帧 (1s gap), 模拟物体消失。
    # 不调用 update 即可模拟 gap; 但要验证连续性检测, 需要调用一次空帧。
    tracker.update(np.empty((0, 3)), stamp_s=4 * dt)  # 空帧 → 占据中断

    # 重新出现, first_seen 应重置。
    static, unconfirmed, instant = tracker.update(env, stamp_s=5 * dt)
    # 只有 1 帧 (0.5s) 的连续占据, 远未达 3s → 应是 unconfirmed。
    assert len(unconfirmed) > 0
    assert len(static) == 0

    # 再持续 3s (6 帧), 才能升格为 static。
    for i in range(6):
        static, unconfirmed, instant = tracker.update(env, stamp_s=(6 + i) * dt)
    assert len(static) > 0
    assert len(unconfirmed) == 0


# ---------------------------------------------------------------------------
# 三层占据分离: 超时清理
# ---------------------------------------------------------------------------

def test_timeout_clears_state(spec):
    """occupancy_timeout_s 未见 → 清除 first_seen/last_seen。"""
    tracker = OccupancyTracker(spec, occupancy_timeout_s=1.0, static_confirm_s=3.0)
    env = _points_in(np.array([0.0, 0.4, 1.3]), 0.1, 800)
    dt = 0.5

    # 先占据 2 帧 (1s)。
    for i in range(2):
        tracker.update(env, stamp_s=i * dt)

    # 超过 timeout (1.0s) 不更新 → 内部状态应被清除。
    # 发一帧空帧, 时间戳超过 timeout。
    tracker.update(np.empty((0, 3)), stamp_s=10.0)

    # 验证内部状态已清除: first_seen 应全是 inf。
    assert np.all(np.isinf(tracker._first_seen))
    assert np.all(tracker._last_seen == -np.inf)


# ---------------------------------------------------------------------------
# 三层占据分离: prev_occupied 连续性检测
# ---------------------------------------------------------------------------

def test_prev_occupied_continuity(spec):
    """prev_occupied 正确追踪上一帧占据状态。"""
    tracker = OccupancyTracker(spec, occupancy_timeout_s=2.0, static_confirm_s=3.0)
    env = _points_in(np.array([0.0, 0.4, 1.3]), 0.1, 800)
    dt = 0.5

    # 初始 prev_occupied 应全 False。
    assert not np.any(tracker._prev_occupied)

    # 第一帧后, env 区域应变为 True。
    tracker.update(env, stamp_s=0.0)
    assert np.any(tracker._prev_occupied)

    # 保存 prev_occupied 快照。
    prev_snap = tracker._prev_occupied.copy()

    # 第二帧同样内容, prev_occupied 应更新但连续。
    tracker.update(env, stamp_s=dt)
    # 连续占据区域不应触发 newly_occupied (已连续)。
    # prev_snap & 当前 occupied = 连续区域。
    assert np.any(tracker._prev_occupied)

    # 空帧 → 占据中断, prev_occupied 应变全 False。
    tracker.update(np.empty((0, 3)), stamp_s=2 * dt)
    assert not np.any(tracker._prev_occupied)


def test_newly_occupied_detection(spec):
    """newly_occupied = occupied & ~prev_occupied 正确触发。"""
    tracker = OccupancyTracker(spec, occupancy_timeout_s=2.0, static_confirm_s=3.0)
    env1 = _points_in(np.array([0.0, 0.4, 1.3]), 0.1, 800, seed=0)
    env2 = _points_in(np.array([0.0, 0.4, 1.3]), 0.1, 800, seed=1)
    dt = 0.5

    # 第一帧: 全部是 newly_occupied。
    tracker.update(env1, stamp_s=0.0)
    first_snap = tracker._first_seen.copy()

    # 第二帧: 相同区域, first_seen 不应重置 (连续占据)。
    tracker.update(env1, stamp_s=dt)
    # 连续区域的 first_seen 应保持不变。
    continuous_mask = ~np.isinf(first_snap) & ~np.isinf(tracker._first_seen)
    if np.any(continuous_mask):
        np.testing.assert_array_equal(
            tracker._first_seen[continuous_mask], first_snap[continuous_mask])


def test_empty_input_returns_empty(spec):
    """空输入 → 三层都为空。"""
    tracker = OccupancyTracker(spec, occupancy_timeout_s=2.0, static_confirm_s=3.0)
    static, unconfirmed, instant = tracker.update(
        np.empty((0, 3)), stamp_s=0.0)
    assert len(static) == 0
    assert len(unconfirmed) == 0
    assert len(instant) == 0


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
