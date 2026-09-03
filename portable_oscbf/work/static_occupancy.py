#!/usr/bin/env python3
"""静态/动态障碍分离: 把世界系点云分成三层占据模型。

静态环境 (桌、柜、墙等持续存在的物体) 应进入 ESDF 距离场;
瞬态物体 (人、移动的物体) 应进入动态聚类 → 8 槽 track。

判定原则: 时间戳驱动的体素占据。
  - instant: 当前帧全部点 (原始点)。
  - unconfirmed: 占据但未达静态阈值的体素中心。
  - static: 持续占据 >= static_confirm_s 的体素中心。

内部用 prev_occupied 连续性检测: 占据中断后 first_seen 重置为 inf,
下次重新计时。occupancy_timeout_s 未见的体素清除 first_seen/last_seen。
"""

from __future__ import annotations

from typing import Tuple

import numpy as np

from work.safety_snapshot import SafetyGridSpec


class OccupancyTracker:
    """时间戳驱动三层占据模型。

    用法
    ----
    >>> tracker = OccupancyTracker(spec, occupancy_timeout_s=2.0, static_confirm_s=5.0)
    >>> static, unconfirmed, instant = tracker.update(points_world, stamp_s)
    """

    def __init__(
        self,
        spec: SafetyGridSpec,
        occupancy_timeout_s: float = 2.0,
        static_confirm_s: float = 5.0,
    ) -> None:
        self.spec = spec
        self.occupancy_timeout_s = max(0.1, float(occupancy_timeout_s))
        self.static_confirm_s = max(0.1, float(static_confirm_s))

        shape = spec.shape
        # first_seen: 体素首次被连续占据的时间戳; inf 表示未占据。
        self._first_seen = np.full(shape, np.inf, dtype=np.float64)
        # last_seen: 体素最后一次被观测到的时间戳; -inf 表示从未见过。
        self._last_seen = np.full(shape, -np.inf, dtype=np.float64)
        # prev_occupied: 上一帧的占据掩码, 用于连续性检测。
        self._prev_occupied = np.zeros(shape, dtype=bool)

    def update(
        self, points_world: np.ndarray, stamp_s: float,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """输入世界系 (M,3) 点云和时间戳 (秒), 返回三层点云。

        - static_points:    持续占据 >= static_confirm_s 的体素中心 (供 ESDF)。
        - unconfirmed_points: 当前帧占据但未达静态阈值的体素中心 (供聚类)。
        - instant_points:   当前帧全部有效点 (原始点)。
        """
        points = np.asarray(points_world, dtype=np.float64).reshape(-1, 3)
        finite = points[np.all(np.isfinite(points), axis=1)]
        spec = self.spec
        stamp = float(stamp_s)

        # --- 当前帧占据掩码 ---
        occupied = np.zeros(spec.shape, dtype=bool)
        valid = np.zeros(len(finite), dtype=bool)
        idx = None
        if len(finite):
            indices = np.floor(
                (finite - spec.workspace_min) / spec.voxel_size).astype(np.int64)
            upper = np.asarray(spec.shape, dtype=np.int64)
            valid = np.all((indices >= 0) & (indices < upper), axis=1)
            if np.any(valid):
                idx = indices[valid]
                unique_idx = np.unique(idx, axis=0)
                occupied[unique_idx[:, 0], unique_idx[:, 1], unique_idx[:, 2]] = True

        # --- 连续性检测 ---
        # newly_occupied: 本帧新出现、上帧未占据的体素 → 重置 first_seen。
        newly_occupied = occupied & ~self._prev_occupied
        # 占据中断: 上帧占据、本帧未占据 → first_seen 重置为 inf。
        gap = ~occupied & self._prev_occupied

        # 更新 first_seen: 新占据的体素记录当前时间; 中断的重置为 inf。
        self._first_seen[newly_occupied] = stamp
        self._first_seen[gap] = np.inf

        # 更新 last_seen: 当前帧占据的体素更新; 超时的重置为 -inf。
        self._last_seen[occupied] = stamp
        timed_out = (~occupied) & ((stamp - self._last_seen) > self.occupancy_timeout_s)
        self._first_seen[timed_out] = np.inf
        self._last_seen[timed_out] = -np.inf

        # 更新 prev_occupied。
        self._prev_occupied = occupied

        # --- 三层分类 ---
        # static: 持续占据 >= static_confirm_s 的体素。
        static_mask = occupied & ((stamp - self._first_seen) >= self.static_confirm_s)
        # unconfirmed: 当前占据但未达静态阈值。
        unconfirmed_mask = occupied & ~static_mask

        # static_points: 静态体素的中心 (网格稳定, 便于 ESDF 复用)。
        static_points = _voxel_centers(spec, static_mask)
        # unconfirmed_points: 未确认体素的中心。
        unconfirmed_points = _voxel_centers(spec, unconfirmed_mask)

        # instant_points: 当前帧全部有效原始点。
        instant_points = finite.astype(np.float32, copy=False)

        return static_points, unconfirmed_points, instant_points


# ---------------------------------------------------------------------------
# 辅助函数
# ---------------------------------------------------------------------------

def _voxel_centers(spec: SafetyGridSpec, mask: np.ndarray) -> np.ndarray:
    """返回 mask 中 True 体素的世界坐标中心点 (N,3) float32。"""
    idx = np.argwhere(mask)
    if len(idx):
        return (spec.workspace_min + (idx.astype(np.float64) + 0.5)
                * spec.voxel_size).astype(np.float32)
    return np.empty((0, 3), dtype=np.float32)


# ---------------------------------------------------------------------------
# 向后兼容包装: 旧接口 StaticOccupancyTracker(spec, keep_frames=8)
# ---------------------------------------------------------------------------

class StaticOccupancyTracker(OccupancyTracker):
    """向后兼容包装。接受旧 keep_frames 参数, 内部转换为时间模型。

    旧调用方式继续工作::

        tracker = StaticOccupancyTracker(spec, keep_frames=8)
        static, dynamic = tracker.update(points_world)   # stamp_s 可省略

    注意: update() 仍返回三层 (static, unconfirmed, instant),
    旧代码解包两个值会拿到 (static, unconfirmed)。
    """

    def __init__(self, spec: SafetyGridSpec, keep_frames: int = 8,
                 presence_gain: int = 2, absence_decay: int = 1) -> None:
        # keep_frames 帧 → 近似转换为秒 (假设 ~30fps, 每帧 ~0.033s)。
        # 这只是粗略映射, 实际部署应直接用 OccupancyTracker 的时间参数。
        approx_frame_s = 0.033
        static_confirm_s = max(0.1, keep_frames * approx_frame_s)
        occupancy_timeout_s = max(0.1, keep_frames * approx_frame_s * 0.5)
        super().__init__(spec, occupancy_timeout_s=occupancy_timeout_s,
                         static_confirm_s=static_confirm_s)
        self._auto_stamp = 0.0

    def update(self, points_world, stamp_s=None):  # type: ignore[override]
        """旧接口兼容: stamp_s 可省略, 自动用单调递增时间。"""
        if stamp_s is None:
            stamp_s = self._auto_stamp
            self._auto_stamp += 0.033  # 假设 ~30fps
        return super().update(points_world, stamp_s)
