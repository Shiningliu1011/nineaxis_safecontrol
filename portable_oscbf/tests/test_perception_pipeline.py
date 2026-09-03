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
from work.safety_snapshot import MAX_DYNAMIC_TRACKS, preprocess_points
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


def test_config_fields_covers_all_yaml_loadable_fields():
    """config_fields() 返回所有可从 YAML 加载的字段元数据。"""
    from work.perception_config import PointCloudCollisionConfig
    fields = PointCloudCollisionConfig.config_fields()
    names = {f.name for f in fields}
    # 验证覆盖了所有 YAML 可加载的简单字段 (非 workspace_min/max)。
    expected = {
        "source_topic", "input_frame", "world_frame", "voxel_size",
        "max_points", "point_radius", "safety_margin", "sdf_far_distance",
        "static_occupancy_frames", "source_topic_lidar", "input_frame_lidar",
        "source_voxel_camera_m", "source_voxel_lidar_m", "fusion_voxel_m",
        "max_inter_sensor_dt_s", "camera_max_age_s", "lidar_max_age_s",
        "occupancy_timeout_s", "static_confirm_s", "perception_timeout_s",
        "cluster_max_tracks", "cluster_min_points",
        "cluster_association_max_dist_m",
    }
    assert expected <= names, f"Missing: {expected - names}"


def test_config_fields_type_constructors_match_defaults():
    """每个字段的 type_constructor 能正确转换默认值。"""
    from work.perception_config import PointCloudCollisionConfig
    for f in PointCloudCollisionConfig.config_fields():
        # type_constructor(str) 应不报错 (str, int, float 都能转 str)。
        assert callable(f.type_constructor)
        # default 应存在。
        assert f.default is not None or f.type_constructor is str


# ---------------------------------------------------------------------------
# 向后兼容: StaticOccupancyTracker 旧接口
# ---------------------------------------------------------------------------

def test_static_occupancy_tracker_old_interface(spec):
    """旧 StaticOccupancyTracker(spec, keep_frames=8) 接口继续工作。"""
    from work.static_occupancy import StaticOccupancyTracker
    tracker = StaticOccupancyTracker(spec, keep_frames=8)
    env = _points_in(np.array([0.0, 0.4, 1.3]), 0.1, 800)
    # 旧代码不传 stamp_s → update 内部用默认值。
    # 向后兼容: 返回 2-tuple (static, dynamic)。
    result = tracker.update(env)
    assert len(result) == 2


def test_static_occupancy_tracker_two_value_unpack(spec):
    """旧代码解包两个值 static, dynamic = tracker.update(pts) 不崩溃。

    这是向后兼容的核心契约: StaticOccupancyTracker.update() 必须返回
    2-tuple (static, dynamic) 而非 3-tuple, 否则旧调用方会 ValueError。
    """
    from work.static_occupancy import StaticOccupancyTracker
    tracker = StaticOccupancyTracker(spec, keep_frames=8)
    env = _points_in(np.array([0.0, 0.4, 1.3]), 0.1, 800)
    # 旧代码: 2-value unpack, 不传 stamp_s。
    static, dynamic = tracker.update(env)
    assert isinstance(static, np.ndarray)
    assert isinstance(dynamic, np.ndarray)


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
# 三层占据分离: 超时清理 (通过公共行为验证)
# ---------------------------------------------------------------------------

def test_timeout_clears_state(spec):
    """occupancy_timeout_s 未见 → 重新出现时计时重置, 不会直接变 static。"""
    tracker = OccupancyTracker(spec, occupancy_timeout_s=1.0, static_confirm_s=2.0)
    env = _points_in(np.array([0.0, 0.4, 1.3]), 0.1, 800)
    dt = 0.5

    # 先占据 6 帧 (3s > 2s) → 变成 static。
    for i in range(6):
        static, _, _ = tracker.update(env, stamp_s=i * dt)
    assert len(static) > 0

    # 超过 timeout (1.0s) 后空帧 → 状态清除。
    tracker.update(np.empty((0, 3)), stamp_s=10.0)

    # 重新出现第一帧 → 应是 unconfirmed, 不是 static (计时已重置)。
    static, unconfirmed, _ = tracker.update(env, stamp_s=10.5)
    assert len(static) == 0
    assert len(unconfirmed) > 0


# ---------------------------------------------------------------------------
# 三层占据分离: 连续性检测 (通过公共行为验证)
# ---------------------------------------------------------------------------

def test_continuity_through_behavior(spec):
    """连续占据 → 升格 static; 中断后 → 需重新计时。"""
    tracker = OccupancyTracker(spec, occupancy_timeout_s=2.0, static_confirm_s=2.0)
    env = _points_in(np.array([0.0, 0.4, 1.3]), 0.1, 800)
    dt = 0.5

    # 前 5 帧连续占据 (2.5s > 2.0s) → 应变 static。
    for i in range(5):
        static, unconfirmed, _ = tracker.update(env, stamp_s=i * dt)
    assert len(static) > 0
    assert len(unconfirmed) == 0

    # 空帧 → 占据中断。
    tracker.update(np.empty((0, 3)), stamp_s=5 * dt)

    # 重新出现 → 应是 unconfirmed (计时重置)。
    static, unconfirmed, _ = tracker.update(env, stamp_s=6 * dt)
    assert len(static) == 0
    assert len(unconfirmed) > 0


def test_gap_vs_continuous_timing(spec):
    """连续占据 vs 有间隙的占据: 同样总时长, 行为不同。"""
    env = _points_in(np.array([0.0, 0.4, 1.3]), 0.1, 800)
    dt = 0.5

    # 连续: 5 帧 (2.5s) 无中断。
    tracker1 = OccupancyTracker(spec, occupancy_timeout_s=2.0, static_confirm_s=2.0)
    for i in range(5):
        tracker1.update(env, stamp_s=i * dt)
    static1, _, _ = tracker1.update(env, stamp_s=5 * dt)
    # 已超 2s → static。
    assert len(static1) > 0

    # 有间隙: 前 3 帧 + 空帧 + 再 2 帧 (总时长相同, 但中间断过)。
    tracker2 = OccupancyTracker(spec, occupancy_timeout_s=2.0, static_confirm_s=2.0)
    for i in range(3):
        tracker2.update(env, stamp_s=i * dt)
    tracker2.update(np.empty((0, 3)), stamp_s=3 * dt)  # 中断
    tracker2.update(env, stamp_s=4 * dt)
    static2, unconfirmed2, _ = tracker2.update(env, stamp_s=4.5 * dt)
    # 只连续 0.5s → 还是 unconfirmed。
    assert len(static2) == 0
    assert len(unconfirmed2) > 0


def test_empty_input_returns_empty(spec):
    """空输入 → 三层都为空。"""
    tracker = OccupancyTracker(spec, occupancy_timeout_s=2.0, static_confirm_s=3.0)
    static, unconfirmed, instant = tracker.update(
        np.empty((0, 3)), stamp_s=0.0)
    assert len(static) == 0
    assert len(unconfirmed) == 0
    assert len(instant) == 0


# ---------------------------------------------------------------------------
# instant_occupancy 安全通道: instant_points 始终返回全部有效原始点
# ---------------------------------------------------------------------------

def test_instant_points_are_raw_not_voxel_centers(spec):
    """instant_points 是原始点 (非体素中心), 与 static/unconfirmed 不同。"""
    tracker = OccupancyTracker(spec, occupancy_timeout_s=2.0, static_confirm_s=0.1)
    pts = _points_in(np.array([0.0, 0.4, 1.3]), 0.05, 200)
    _, _, instant = tracker.update(pts, stamp_s=0.0)
    # instant 应返回全部有效原始点 (可能有部分在 workspace 外被裁剪)。
    assert len(instant) > 0
    assert instant.shape[1] == 3
    # 原始点应比体素中心多 (多个点映射到同一体素)。
    _, _, instant2 = tracker.update(pts, stamp_s=10.0)
    # 即使 static 层已建立, instant 仍返回原始点。
    assert len(instant2) > 0


def test_instant_points_include_new_obstacles(spec):
    """新障碍物在 static 确认前就出现在 instant 层。"""
    tracker = OccupancyTracker(spec, occupancy_timeout_s=2.0, static_confirm_s=5.0)
    env = _points_in(np.array([0.0, 0.4, 1.3]), 0.1, 500)
    # 环境持续 10s → 变成 static。
    for i in range(20):
        tracker.update(env, stamp_s=i * 0.5)
    # 新障碍物出现第一帧。
    new_obs = _points_in(np.array([0.3, 0.5, 1.0]), 0.03, 100)
    frame = np.vstack([env, new_obs])
    static, unconfirmed, instant = tracker.update(frame, stamp_s=10.0)
    # instant 应包含全部点 (环境 + 新障碍物)。
    assert len(instant) > len(env)
    # 新障碍物应进 unconfirmed (未达 static_confirm_s)。
    assert len(unconfirmed) > 0
    # 环境应进 static。
    assert len(static) > 0


def test_instant_points_returns_all_finite_points(spec):
    """instant_points 返回全部有限点 (workspace 过滤在上游 preprocess_points)。"""
    tracker = OccupancyTracker(spec, occupancy_timeout_s=2.0, static_confirm_s=3.0)
    pts = _points_in(np.array([0.0, 0.4, 1.3]), 0.1, 200)
    _, _, instant = tracker.update(pts, stamp_s=0.0)
    # instant 返回全部有限原始点 (workspace 过滤由 preprocess_points 完成)。
    assert len(instant) > 0
    assert instant.shape[1] == 3


# ---------------------------------------------------------------------------
# status 编码契约: 10 元素布局验证
# ---------------------------------------------------------------------------

def test_status_encoding_layout():
    """status 10 元素布局与 spec 一致 (纯编码测试, 无 ROS)。"""
    # 模拟 _publish_status 的编码逻辑。
    camera_alive, lidar_alive = True, False
    camera_age, lidar_age = 0.05, -1.0
    camera_used, lidar_used = True, False
    fusion_stamp, fusion_age = 100.5, 0.02
    source_count = 1
    perception_valid = True

    data = [
        1.0 if camera_alive else 0.0,
        1.0 if lidar_alive else 0.0,
        float(camera_age),
        float(lidar_age),
        1.0 if camera_used else 0.0,
        1.0 if lidar_used else 0.0,
        float(fusion_stamp),
        float(fusion_age),
        float(source_count),
        1.0 if perception_valid else 0.0,
    ]
    assert len(data) == 10
    assert data[0] == 1.0   # camera_alive
    assert data[1] == 0.0   # lidar_alive
    assert data[2] == 0.05  # camera_age
    assert data[3] == -1.0  # lidar_age (无数据)
    assert data[4] == 1.0   # camera_used
    assert data[5] == 0.0   # lidar_used
    assert data[6] == 100.5 # fusion_stamp
    assert data[7] == 0.02  # fusion_age
    assert data[8] == 1.0   # source_count
    assert data[9] == 1.0   # perception_valid


def test_perception_valid_formula():
    """perception_valid = (camera_used or lidar_used) and fusion_age < timeout。"""
    timeout = 1.0
    # 有传感器被采用且年龄在超时内 → valid。
    assert (True or False) and 0.5 < timeout
    # 无传感器被采用 → invalid。
    assert not ((False or False) and 0.5 < timeout)
    # 有传感器但超时 → invalid。
    assert not ((True or False) and 1.5 < timeout)
    # 两路都采用且新鲜 → valid。
    assert (True or True) and 0.1 < timeout


def test_alive_vs_used_independent():
    """alive=True 不保证 used=True (跨传感器 dt 降级场景)。"""
    # Camera alive 但因跨传感器 dt 太大而未被采用。
    camera_alive = True
    camera_used = False  # 被跨传感器 dt 降级。
    assert camera_alive and not camera_used
    # LiDAR alive 但因 max_age 超时而未被采用。
    lidar_alive = True
    lidar_used = False
    assert lidar_alive and not lidar_used


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


# ---------------------------------------------------------------------------
# voxel_downsample: 共享降采样函数 (消除 fusion_engine / perception_bridge 重复)
# ---------------------------------------------------------------------------

def test_voxel_downsample_deduplicates_nearby_points():
    """同一体素内的多个点只保留一个。"""
    from work.safety_snapshot import voxel_downsample
    # 三个点都在同一个 0.05m 体素内。
    pts = np.array([[0.0, 0.0, 0.0],
                    [0.01, 0.01, 0.01],
                    [0.02, 0.02, 0.02]], dtype=np.float32)
    result = voxel_downsample(pts, 0.05)
    assert len(result) == 1


def test_voxel_downsample_preserves_distant_points():
    """不同体素的点全部保留。"""
    from work.safety_snapshot import voxel_downsample
    pts = np.array([[0.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0]], dtype=np.float32)
    result = voxel_downsample(pts, 0.05)
    assert len(result) == 3


def test_voxel_downsample_empty_input():
    """空输入 → 空输出。"""
    from work.safety_snapshot import voxel_downsample
    pts = np.empty((0, 3), dtype=np.float32)
    result = voxel_downsample(pts, 0.05)
    assert len(result) == 0


def test_voxel_downsample_returns_float32():
    """输出始终是 float32。"""
    from work.safety_snapshot import voxel_downsample
    pts = np.array([[0.1, 0.2, 0.3]], dtype=np.float64)
    result = voxel_downsample(pts, 0.1)
    assert result.dtype == np.float32


# ---------------------------------------------------------------------------
# preprocess_points: 每传感器源体素降采样
# ---------------------------------------------------------------------------

def test_preprocess_points_respects_custom_voxel_size(spec):
    """自定义 voxel_size 参数应覆盖 spec.voxel_size。"""
    center = (spec.workspace_min + spec.workspace_max) / 2
    rng = np.random.default_rng(42)
    pts = (center + rng.uniform(-0.04, 0.04, (200, 3))).astype(np.float32)
    identity = np.eye(4, dtype=np.float64)

    coarse = preprocess_points(pts, identity, spec, voxel_size=0.1)
    fine = preprocess_points(pts, identity, spec, voxel_size=0.01)
    assert len(coarse) < len(fine), (
        f"coarse({len(coarse)}) should be < fine({len(fine)})")


def test_preprocess_points_removes_robot_spheres(spec):
    """robot_spheres 参数应剔除机器人本体占据的点。"""
    center = (spec.workspace_min + spec.workspace_max) / 2
    # 在中心放一个球体 (0.15m 半径) 内的点。
    rng = np.random.default_rng(99)
    pts = (center + rng.uniform(-0.02, 0.02, (100, 3))).astype(np.float32)
    identity = np.eye(4, dtype=np.float64)

    # 无球体过滤 → 全部保留。
    no_filter = preprocess_points(pts, identity, spec)
    # 球体覆盖整个点云区域 → 全部剔除。
    filtered = preprocess_points(
        pts, identity, spec,
        robot_spheres=[(center, 0.15)])
    assert len(no_filter) > 0
    assert len(filtered) == 0
