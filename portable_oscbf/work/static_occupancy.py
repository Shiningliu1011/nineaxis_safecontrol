#!/usr/bin/env python3
"""静态/动态障碍分离: 把世界系点云分成静态环境与瞬态团块。

静态环境 (桌、柜、墙等持续存在的物体) 应进入 ESDF 距离场;
瞬态物体 (人、移动的物体) 应进入动态聚类 → 8 槽 track。

判定原则: 体素占用计数。某体素连续被占据 >= keep_frames 帧后视为静态;
当前帧新出现、计数未达阈值的体素视为动态。
"""

from __future__ import annotations

from typing import Tuple

import numpy as np

try:
    from safety_snapshot import SafetyGridSpec
except ImportError:  # 包式导入回退
    from work.safety_snapshot import SafetyGridSpec


class StaticOccupancyTracker:
    """按体素占用时长区分静态/动态。

    用法
    ----
    >>> tracker = StaticOccupancyTracker(spec, keep_frames=8)
    >>> static, dynamic = tracker.update(points_world)   # 每帧调用一次
    """

    def __init__(self, spec: SafetyGridSpec, keep_frames: int = 8,
                 presence_gain: int = 2, absence_decay: int = 1) -> None:
        self.spec = spec
        self.keep_frames = max(1, int(keep_frames))
        self.presence_gain = max(1, int(presence_gain))
        self.absence_decay = max(1, int(absence_decay))
        # 每体素占用计分 (泄漏计数器, 抗深度噪声):
        #   出现 -> +presence_gain (封顶 keep_frames); 未出现 -> -absence_decay (归零)。
        self._counts = np.zeros(spec.shape, dtype=np.int16)

    def update(self, points_world: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """输入世界系 (M,3) 点云, 返回 (static_points, dynamic_points) 世界系 float32。

        - static_points: 已达阈值的静态体素中心 (供 build_distance_field)。
        - dynamic_points: 当前帧落在未达阈值体素内的原始点 (供聚类)。
        """
        points = np.asarray(points_world, dtype=np.float64).reshape(-1, 3)
        finite = points[np.all(np.isfinite(points), axis=1)]
        spec = self.spec

        # 体素占用: 出现则 +presence_gain (封顶 keep_frames), 未出现才 -absence_decay。
        # 泄漏计数器抗深度噪声: 表面体素偶尔缺席一帧不会立刻掉回动态。
        present = np.zeros(spec.shape, dtype=bool)
        valid = np.zeros(len(finite), dtype=bool)
        if len(finite):
            indices = np.floor(
                (finite - spec.workspace_min) / spec.voxel_size).astype(np.int64)
            upper = np.asarray(spec.shape, dtype=np.int64)
            valid = np.all((indices >= 0) & (indices < upper), axis=1)
            if np.any(valid):
                idx = indices[valid]
                unique_idx = np.unique(idx, axis=0)
                present[unique_idx[:, 0], unique_idx[:, 1], unique_idx[:, 2]] = True

        self._counts[present] = np.minimum(
            self._counts[present].astype(np.int32) + self.presence_gain,
            self.keep_frames).astype(np.int16)
        self._counts[~present] = np.maximum(
            self._counts[~present].astype(np.int32) - self.absence_decay,
            0).astype(np.int16)

        static_mask = self._counts >= self.keep_frames

        # 静态点: 静态体素的中心 (网格稳定, 便于 ESDF 复用)。
        static_idx = np.argwhere(static_mask)
        if len(static_idx):
            static_points = (
                spec.workspace_min + (static_idx.astype(np.float64) + 0.5)
                * spec.voxel_size).astype(np.float32)
        else:
            static_points = np.empty((0, 3), dtype=np.float32)

        # 动态点: 当前帧落在 "出现但未达静态阈值" 体素内的原始点。
        dynamic = np.zeros(len(finite), dtype=bool)
        if np.any(valid):
            present_this = present[idx[:, 0], idx[:, 1], idx[:, 2]]
            static_this = static_mask[idx[:, 0], idx[:, 1], idx[:, 2]]
            dynamic[valid] = present_this & ~static_this
        dynamic_points = finite[dynamic].astype(np.float32, copy=False)

        return static_points, dynamic_points
