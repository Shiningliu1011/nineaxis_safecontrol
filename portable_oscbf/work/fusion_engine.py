#!/usr/bin/env python3
"""纯 Python 双传感器融合引擎 (零 ROS 依赖)。

从 perception_bridge.py 提取的核心融合逻辑, 供无硬件集成测试使用。
接口精简: feed_camera / feed_lidar 推入合成点云, fuse() 执行一次融合周期。

状态码 layout (10 floats, 与 perception_bridge 的 /perception/status 一致):
    [0] camera_alive   [1] lidar_alive
    [2] camera_age     [3] lidar_age
    [4] camera_used    [5] lidar_used
    [6] fusion_stamp   [7] fusion_age
    [8] source_count   [9] perception_valid
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, Optional, Tuple

import numpy as np

from work.dynamic_clustering import TrackState, cluster_into_tracks, empty_track_state
from work.safety_snapshot import (
    MAX_DYNAMIC_TRACKS,
    SafetyGridSpec,
    build_distance_field,
    preprocess_points,
    voxel_downsample,
)
from work.static_occupancy import OccupancyTracker


def _filter_robot_spheres(
    points: np.ndarray,
    robot_spheres: Iterable[Tuple[np.ndarray, float]],
) -> np.ndarray:
    """Remove points inside any robot sphere (world-frame filtering)."""
    if len(points) == 0:
        return points
    pts = np.asarray(points, dtype=np.float32)
    keep = np.ones(len(pts), dtype=bool)
    for center, radius in robot_spheres:
        d = np.linalg.norm(pts - np.asarray(center, dtype=np.float32), axis=1)
        keep &= d > float(radius)
    return pts[keep]


@dataclass
class FusionResult:
    """一次融合周期的全部输出。"""

    merged_points: np.ndarray          # 融合+降采样后的世界系点云
    tracks: TrackState                 # 动态聚类 8 槽 track
    distance_field: np.ndarray         # 静态层 ESDF
    instant_points: np.ndarray         # instant 安全通道原始点
    status: Dict[str, float]           # 10 元素状态 (与 /perception/status 对应)


@dataclass
class FusionEngine:
    """无状态融合管线 (每次 fuse() 独立, 内部保留占据/tracker 连续性)。

    用法::

        engine = FusionEngine(spec)
        engine.feed_camera(points, stamp_s)
        engine.feed_lidar(points, stamp_s)
        result = engine.fuse(now_s)
    """

    spec: SafetyGridSpec
    max_inter_sensor_dt_s: float = 0.1
    camera_max_age_s: float = 0.5
    lidar_max_age_s: float = 0.5
    perception_timeout_s: float = 1.0
    fusion_voxel_m: float = 0.03
    safety_margin: float = 0.08
    sdf_far: float = 10.0
    occupancy_timeout_s: float = 0.3
    static_confirm_s: float = 0.5
    cluster_max_tracks: int = MAX_DYNAMIC_TRACKS
    cluster_min_points: int = 4
    cluster_association_max_dist_m: float = 0.5
    camera_buffer_maxlen: int | None = None
    lidar_buffer_maxlen: int | None = None

    # --- 内部状态 (跨帧连续性) ---
    _occupancy_tracker: OccupancyTracker = field(init=False, repr=False)
    _prev_tracks: TrackState = field(init=False, repr=False)
    _next_track_id: int = field(default=1, init=False)
    _prev_fusion_stamps: tuple = field(default=(), init=False)

    # --- 传感器缓冲 ---
    _camera_buffer: deque = field(init=False, repr=False)
    _lidar_buffer: deque = field(init=False, repr=False)

    def __post_init__(self) -> None:
        from collections import deque as _dq
        self._camera_buffer = _dq(maxlen=self.camera_buffer_maxlen)
        self._lidar_buffer = _dq(maxlen=self.lidar_buffer_maxlen)
        self._occupancy_tracker = OccupancyTracker(
            self.spec,
            occupancy_timeout_s=self.occupancy_timeout_s,
            static_confirm_s=self.static_confirm_s,
        )
        self._prev_tracks = empty_track_state(self.cluster_max_tracks)

    # ------------------------------------------------------------------
    # 传感器数据输入
    # ------------------------------------------------------------------

    def feed_camera(
        self, points_world: np.ndarray, stamp_s: float, *,
        robot_spheres: Iterable[Tuple[np.ndarray, float]] = (),
    ) -> None:
        """推入一帧相机世界系点云 (经预处理后)。"""
        self._camera_buffer.append(
            (np.asarray(points_world, dtype=np.float32), float(stamp_s),
             list(robot_spheres)))

    def feed_lidar(
        self, points_world: np.ndarray, stamp_s: float, *,
        robot_spheres: Iterable[Tuple[np.ndarray, float]] = (),
    ) -> None:
        """推入一帧 LiDAR 世界系点云 (经预处理后)。"""
        self._lidar_buffer.append(
            (np.asarray(points_world, dtype=np.float32), float(stamp_s),
             list(robot_spheres)))

    def clear_buffers(self) -> None:
        """清空传感器缓冲 (测试用)。"""
        self._camera_buffer.clear()
        self._lidar_buffer.clear()

    # ------------------------------------------------------------------
    # 融合主循环 (与 perception_bridge._fusion_callback 对齐)
    # ------------------------------------------------------------------

    def fuse(self, now_s: float) -> FusionResult:
        """执行一次融合周期, 返回 FusionResult。

        逻辑与 perception_bridge._fusion_callback 严格对齐:
        1. 快照缓冲
        2. alive/used 检测 + 新鲜度检查
        3. 时间戳配对 (LiDAR 为参考, Camera 选最近)
        4. 跨传感器 dt 降级
        5. 重复帧保护
        6. 合并 + 融合体素降采样
        7. 三层占据分类
        8. ESDF + 动态聚类
        9. 状态编码
        """
        now = float(now_s)

        # 1. 快照缓冲。
        lidar_frame = self._lidar_buffer[-1] if self._lidar_buffer else None
        camera_snapshot = list(self._camera_buffer)

        # 2. Alive / used 检测 + 新鲜度。
        camera_alive = False
        camera_used = False
        camera_age = -1.0
        t_camera = None
        camera_pts = None

        if camera_snapshot:
            _, latest_cam_stamp, _ = camera_snapshot[-1]
            camera_age = now - latest_cam_stamp
            camera_alive = camera_age < self.camera_max_age_s

        lidar_alive = False
        lidar_used = False
        lidar_age = -1.0
        t_lidar = None
        lidar_pts = None

        lidar_spheres: list = []
        if lidar_frame is not None:
            _, stamp, lidar_spheres = lidar_frame
            lidar_age = now - stamp
            lidar_alive = lidar_age < self.lidar_max_age_s
            if lidar_alive:
                lidar_used = True
                t_lidar = stamp
                lidar_pts = lidar_frame[0]

        # 3. 时间戳配对: LiDAR 为参考, Camera 选最近。
        cam_spheres: list = []
        if lidar_used and camera_snapshot:
            best_dt = float("inf")
            best_cam = None
            for pts, stamp, spheres in camera_snapshot:
                dt = abs(stamp - t_lidar)
                if dt < best_dt:
                    best_dt = dt
                    best_cam = (pts, stamp, spheres)
            if best_cam is not None:
                cam_pts, cam_stamp, cam_spheres = best_cam
                cam_age = now - cam_stamp
                if cam_age < self.camera_max_age_s:
                    camera_used = True
                    t_camera = cam_stamp
                    camera_pts = cam_pts
        elif camera_snapshot:
            pts, stamp, cam_spheres = camera_snapshot[-1]
            cam_age = now - stamp
            if cam_age < self.camera_max_age_s:
                camera_used = True
                t_camera = stamp
                camera_pts = pts

        # 4. 跨传感器 dt 降级。
        if camera_used and lidar_used:
            dt = abs(t_camera - t_lidar)
            if dt > self.max_inter_sensor_dt_s:
                if t_camera > t_lidar:
                    lidar_used = False
                    lidar_pts = None
                else:
                    camera_used = False
                    camera_pts = None

        # 5. 融合时间戳 = max(参与传感器时间戳)。
        stamps_used = [
            t for t, used in
            [(t_camera, camera_used), (t_lidar, lidar_used)] if used
        ]
        if not stamps_used:
            return self._empty_result(now)

        fusion_stamp = max(stamps_used)

        # 6. 重复帧保护。
        combo = tuple(sorted(
            t for t, used in
            [(t_camera, camera_used), (t_lidar, lidar_used)] if used
        ))
        if combo == self._prev_fusion_stamps:
            return self._empty_result(now)
        self._prev_fusion_stamps = combo

        # 7. 合并 + 融合体素降采样。
        clouds = []
        if camera_used and camera_pts is not None:
            clouds.append(camera_pts)
        if lidar_used and lidar_pts is not None:
            clouds.append(lidar_pts)
        merged = np.vstack(clouds) if len(clouds) > 1 else clouds[0]
        merged = voxel_downsample(merged, self.fusion_voxel_m)

        # 7b. 机器人自体过滤 (世界系球体裁剪)。
        all_spheres = []
        if camera_used:
            all_spheres.extend(cam_spheres)
        if lidar_used:
            all_spheres.extend(lidar_spheres)
        if all_spheres and len(merged) > 0:
            merged = _filter_robot_spheres(merged, all_spheres)

        if len(merged) == 0:
            return self._empty_result(now)

        # 8. 三层占据分类。
        static_pts, unconfirmed_pts, instant_pts = \
            self._occupancy_tracker.update(merged, fusion_stamp)

        # 9. ESDF + 动态聚类。
        sdf = build_distance_field(static_pts, self.spec, far_distance=self.sdf_far)
        dt_s = fusion_stamp - (self._prev_fusion_stamps[0]
                               if self._prev_fusion_stamps else fusion_stamp)
        new_tracks, self._next_track_id = cluster_into_tracks(
            unconfirmed_pts, self._prev_tracks, self.spec,
            max_tracks=self.cluster_max_tracks,
            min_points=self.cluster_min_points,
            asso_max_dist_m=self.cluster_association_max_dist_m,
            dt_s=max(dt_s, 1.0 / 30.0),
            next_id=self._next_track_id,
        )
        self._prev_tracks = new_tracks

        # 10. 状态编码。
        fusion_age = now - fusion_stamp
        source_count = int(camera_used) + int(lidar_used)
        perception_valid = (
            (camera_used or lidar_used) and fusion_age < self.perception_timeout_s
        )

        status = {
            "camera_alive": 1.0 if camera_alive else 0.0,
            "lidar_alive": 1.0 if lidar_alive else 0.0,
            "camera_age": float(camera_age),
            "lidar_age": float(lidar_age),
            "camera_used": 1.0 if camera_used else 0.0,
            "lidar_used": 1.0 if lidar_used else 0.0,
            "fusion_stamp": float(fusion_stamp),
            "fusion_age": float(fusion_age),
            "source_count": float(source_count),
            "perception_valid": 1.0 if perception_valid else 0.0,
        }

        return FusionResult(
            merged_points=merged,
            tracks=new_tracks,
            distance_field=sdf,
            instant_points=instant_pts,
            status=status,
        )

    def _empty_result(self, now_s: float) -> FusionResult:
        """无数据时返回空结果 + 状态。"""
        camera_alive = False
        lidar_alive = False
        camera_age = -1.0
        lidar_age = -1.0
        if self._camera_buffer:
            _, stamp, _ = self._camera_buffer[-1]
            camera_age = now_s - stamp
            camera_alive = camera_age < self.camera_max_age_s
        if self._lidar_buffer:
            _, stamp, _ = self._lidar_buffer[-1]
            lidar_age = now_s - stamp
            lidar_alive = lidar_age < self.lidar_max_age_s

        return FusionResult(
            merged_points=np.empty((0, 3), dtype=np.float32),
            tracks=empty_track_state(self.cluster_max_tracks),
            distance_field=np.full(
                self.spec.shape, 10.0, dtype=np.float32),
            instant_points=np.empty((0, 3), dtype=np.float32),
            status={
                "camera_alive": 1.0 if camera_alive else 0.0,
                "lidar_alive": 1.0 if lidar_alive else 0.0,
                "camera_age": float(camera_age),
                "lidar_age": float(lidar_age),
                "camera_used": 0.0,
                "lidar_used": 0.0,
                "fusion_stamp": 0.0,
                "fusion_age": float(now_s),
                "source_count": 0.0,
                "perception_valid": 0.0,
            },
        )
