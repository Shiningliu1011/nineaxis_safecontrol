#!/usr/bin/env python3
"""动态障碍物聚类: 世界系瞬态点云 → 固定 8 槽 track。

用体素 6-连通域聚类, 取点数前 max_tracks 大的团块, 跨帧贪心关联求速度,
输出与 JaxControlLoop 的 obs_* 槽 (MAX_JAX_OBSTACLES=8) 兼容的固定 shape。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import ndimage

try:
    from safety_snapshot import MAX_DYNAMIC_TRACKS, SafetyGridSpec
except ImportError:  # 包式导入回退
    from work.safety_snapshot import MAX_DYNAMIC_TRACKS, SafetyGridSpec


@dataclass
class TrackState:
    """固定 8 槽的动态 track 状态 (与 SafetySnapshot.track_* 对应)。"""

    pos: np.ndarray       # (8, 3) 世界系质心
    radii: np.ndarray     # (8,)   包围半径
    vel: np.ndarray       # (8, 3) 世界系速度
    enabled: np.ndarray   # (8,)   float32, 1.0=激活 0.0=空槽
    ids: np.ndarray       # (8,)   int64 跟踪 id (0=无效)


def empty_track_state(max_tracks: int = MAX_DYNAMIC_TRACKS) -> TrackState:
    return TrackState(
        pos=np.zeros((max_tracks, 3), dtype=np.float32),
        radii=np.zeros(max_tracks, dtype=np.float32),
        vel=np.zeros((max_tracks, 3), dtype=np.float32),
        enabled=np.zeros(max_tracks, dtype=np.float32),
        ids=np.zeros(max_tracks, dtype=np.int64),
    )


def cluster_into_tracks(
    points_world: np.ndarray,
    prev: TrackState,
    spec: SafetyGridSpec,
    *,
    max_tracks: int = MAX_DYNAMIC_TRACKS,
    min_points: int = 4,
    asso_max_dist_m: float = 0.5,
    dt_s: float = 1.0 / 30.0,
    next_id: int = 1,
) -> tuple[TrackState, int]:
    """聚类 + 跨帧关联。

    返回 (new_state, next_id)。``next_id`` 用于持久 id 分配; 无新 track 时原样返回。
    """
    points = np.asarray(points_world, dtype=np.float64).reshape(-1, 3)
    state = empty_track_state(max_tracks)
    if len(points) < min_points:
        return state, next_id

    # ---- 体素 6-连通域聚类 ----
    occ = np.zeros(spec.shape, dtype=bool)
    indices = np.floor(
        (points - spec.workspace_min) / spec.voxel_size).astype(np.int64)
    upper = np.asarray(spec.shape, dtype=np.int64)
    valid = np.all((indices >= 0) & (indices < upper), axis=1)
    if not np.any(valid):
        return state, next_id
    idx = indices[valid]
    occ[idx[:, 0], idx[:, 1], idx[:, 2]] = True
    labels, _ = ndimage.label(occ, structure=np.ones((3, 3, 3), dtype=bool))
    if labels.max() == 0:
        return state, next_id

    label_at_point = labels[idx[:, 0], idx[:, 1], idx[:, 2]]
    components = []  # (centroid, radius, n_points)
    for lab in range(1, int(labels.max()) + 1):
        member_mask = label_at_point == lab
        n = int(member_mask.sum())
        if n < min_points:
            continue
        members = points[valid][member_mask]
        centroid = members.mean(axis=0)
        radius = float(np.max(np.linalg.norm(members - centroid, axis=1)))
        components.append((centroid, radius, n))
    if not components:
        return state, next_id

    # 取点数前 max_tracks 大的团块。
    components.sort(key=lambda c: -c[2])
    components = components[:max_tracks]

    # ---- 跨帧贪心关联 (新质心 → 最近旧激活 track) ----
    prev_pos = np.asarray(prev.pos, dtype=np.float64)
    prev_vel = np.asarray(prev.vel, dtype=np.float64)
    prev_enabled = np.asarray(prev.enabled, dtype=np.float32)
    prev_ids = np.asarray(prev.ids, dtype=np.int64)

    used_prev = np.zeros(max_tracks, dtype=bool)
    for i, (centroid, radius, n) in enumerate(components):
        # 最近未使用的旧激活 track。
        best_j, best_d = -1, float("inf")
        for j in range(max_tracks):
            if prev_enabled[j] <= 0.0 or used_prev[j]:
                continue
            d = float(np.linalg.norm(prev_pos[j] - centroid))
            if d < best_d:
                best_d, best_j = d, j
        if best_j >= 0 and best_d <= asso_max_dist_m:
            used_prev[best_j] = True
            state.pos[i] = centroid.astype(np.float32)
            state.radii[i] = radius
            state.vel[i] = ((centroid - prev_pos[best_j]) / max(dt_s, 1e-6)).astype(
                np.float32)
            state.enabled[i] = 1.0
            state.ids[i] = prev_ids[best_j]
        else:
            state.pos[i] = centroid.astype(np.float32)
            state.radii[i] = radius
            state.enabled[i] = 1.0
            state.ids[i] = next_id
            next_id += 1

    return state, next_id
