"""解析障碍物提取器——纯 numpy/scipy，无 ROS / 无 I/O。

职责（规格 D5 的纯计算层）：
1. 世界系点云 → 体素聚类 → 每簇解析几何（球最小二乘 / 圆柱 PCA 轴 +
   包络球），产出 :class:`ObstacleShape` 列表（按点数排序）。
2. 感知桥接节点发布的 tracks 槽（8 槽 × 10 float：
   px,py,pz, r, vx,vy,vz, enabled, d_safe, alpha）⇄ JAX 控制内核
   ``obs_*`` 数组（pos/radii/enabled/d_safe/vel/radius_dot/alpha）。

控制内核的障碍物界面是球体（位置+包围半径）——圆柱障碍物以**包络球**
进入 ``obs_*``（包络球 = 半径与半长度的正交组合，见
``fit_cylinder``）；稠密点云不直接进 CBF。

聚类网格沿袭内核 ``dynamic_clustering`` 的体素 6/26-连通模式；
本模块与内核解耦（不 import work/），通过 duck-type 的 spec
（.shape / .workspace_min / .voxel_size）消费网格参数。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np
from scipy import ndimage

# 与 JAX 内核 MAX_JAX_OBSTACLES=8、感知桥接 _TRACK_SLOT_FLOATS=10 对齐。
MAX_OBSTACLE_SLOTS = 8
TRACK_SLOT_FLOATS = 10


@dataclass(frozen=True)
class SphereFit:
    center: np.ndarray
    radius: float
    residual_ms: float


@dataclass(frozen=True)
class CylinderFit:
    centroid: np.ndarray
    axis: np.ndarray
    radius: float
    half_length: float
    envelope_radius: float
    residual_ms: float


@dataclass(frozen=True)
class ObstacleShape:
    """解析障碍物：形状类型 + 包络球（控制内核契约的最小单元）。"""

    kind: str  # 'sphere' | 'cylinder'
    center: np.ndarray
    envelope_radius: float
    axis: Optional[np.ndarray]
    n_points: int


@dataclass(frozen=True)
class ObsArrays:
    """与 JAX 控制内核 obs_* 参数一一对应的固定 shape 数组（float64）。"""

    pos: np.ndarray        # (8, 3)
    radii: np.ndarray      # (8,)
    enabled: np.ndarray    # (8,)
    d_safe: np.ndarray     # (8,)
    vel: np.ndarray        # (8, 3)
    radius_dot: np.ndarray  # (8,)
    alpha: np.ndarray      # (8,)


def fit_sphere(points: np.ndarray) -> Optional[SphereFit]:
    """最小二乘球拟合（线性化：2c·p + (r²-|c|²) = |p|²）。"""
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    if pts.shape[0] < 4 or not np.all(np.isfinite(pts)):
        return None
    a = np.column_stack([2.0 * pts, np.ones(pts.shape[0])])
    b = np.einsum("ij,ij->i", pts, pts)
    try:
        solution, *_ = np.linalg.lstsq(a, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    center = solution[:3]
    radius_sq = float(solution[3]) + float(center @ center)
    if radius_sq <= 0.0 or not np.isfinite(radius_sq):
        return None
    radius = float(np.sqrt(radius_sq))
    residual = float(np.mean(np.abs(np.linalg.norm(pts - center, axis=1) - radius)))
    return SphereFit(center=center, radius=radius, residual_ms=residual * 1000.0)


def fit_cylinder(points: np.ndarray) -> Optional[CylinderFit]:
    """圆柱拟合：PCA 主轴（点沿轴展开 ⇒ 最大方差方向）+ 均方半径。"""
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    if pts.shape[0] < 5 or not np.all(np.isfinite(pts)):
        return None
    centroid = pts.mean(axis=0)
    covariance = np.cov((pts - centroid).T)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    if not np.isfinite(eigenvalues).all():
        return None
    axis = eigenvectors[:, int(np.argmax(eigenvalues))]
    norm = float(np.linalg.norm(axis))
    if norm < 1e-9:
        return None
    axis = axis / norm
    offsets = pts - centroid
    parallel = offsets @ axis
    radial = offsets - np.outer(parallel, axis)
    radius = float(np.sqrt(np.mean(np.einsum("ij,ij->i", radial, radial))))
    half_length = float((parallel.max() - parallel.min()) / 2.0)
    envelope = float(np.sqrt(radius * radius + half_length * half_length))
    residual = float(np.mean(np.abs(np.linalg.norm(radial, axis=1) - radius)))
    return CylinderFit(
        centroid=centroid, axis=axis, radius=radius, half_length=half_length,
        envelope_radius=envelope, residual_ms=residual * 1000.0,
    )


def _envelope_radius(pts: np.ndarray, center: np.ndarray) -> float:
    return float(np.max(np.linalg.norm(pts - center, axis=1)))


def _cluster_points(points: np.ndarray, spec, *, min_points: int) -> list[np.ndarray]:
    """体素 26-连通聚类，返回每个达标簇的点集（点数降序）。"""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    pts = pts[np.all(np.isfinite(pts), axis=1)]
    if pts.shape[0] < min_points:
        return []
    indices = np.floor(
        (pts - np.asarray(spec.workspace_min)) / float(spec.voxel_size)).astype(np.int64)
    upper = np.asarray(spec.shape, dtype=np.int64)
    valid = np.all((indices >= 0) & (indices < upper), axis=1)
    if not np.any(valid):
        return []
    idx = indices[valid]
    occ = np.zeros(tuple(spec.shape), dtype=bool)
    occ[idx[:, 0], idx[:, 1], idx[:, 2]] = True
    labels, _ = ndimage.label(occ, structure=np.ones((3, 3, 3), dtype=bool))
    if labels.max() <= 0:
        return []
    parts: list[tuple[int, np.ndarray]] = []
    for lab in range(1, int(labels.max()) + 1):
        member_mask = labels[idx[:, 0], idx[:, 1], idx[:, 2]] == lab
        n = int(member_mask.sum())
        if n < min_points:
            continue
        parts.append((n, pts[valid][member_mask]))
    parts.sort(key=lambda item: -item[0])
    return [points for _, points in parts]


def analyze_cloud(
    points_world: np.ndarray, spec, *, min_points: int = 4,
    max_obstacles: int = MAX_OBSTACLE_SLOTS,
) -> list[ObstacleShape]:
    """世界系点云 → 解析障碍物形状列表（球/圆柱，按点数降序，上限截断）。"""
    shapes: list[ObstacleShape] = []
    for pts in _cluster_points(points_world, spec, min_points=min_points):
        sphere = fit_sphere(pts)
        cylinder = fit_cylinder(pts)
        if cylinder is not None and cylinder.half_length >= 2.0 * cylinder.radius:
            center = cylinder.centroid
            envelope = cylinder.envelope_radius
            kind, axis = "cylinder", cylinder.axis
        elif sphere is not None:
            center = sphere.center
            envelope = max(_envelope_radius(pts, center), sphere.radius)
            kind, axis = "sphere", None
        else:
            center = pts.mean(axis=0)
            envelope = _envelope_radius(pts, center)
            kind, axis = "sphere", None
        shapes.append(ObstacleShape(
            kind=kind, center=np.asarray(center, dtype=float).reshape(3),
            envelope_radius=float(envelope), axis=axis, n_points=int(pts.shape[0]),
        ))
        if len(shapes) >= max_obstacles:
            break
    return shapes


def tracks_slots_to_obs_arrays(
    slots: np.ndarray, *, dt_s: Optional[float] = None,
    prev_radii: Optional[np.ndarray] = None,
) -> ObsArrays:
    """8 槽 tracks（(8,10) 或展平）→ 控制内核 obs_* 数组。

    槽布局：px,py,pz, r, vx,vy,vz, enabled, d_safe, alpha；
    ``radius_dot`` 由上一帧半径 + ``dt_s`` 差分（缺省为 0）。
    """
    raw = np.asarray(slots, dtype=float)
    if raw.shape == (MAX_OBSTACLE_SLOTS * TRACK_SLOT_FLOATS,):
        raw = raw.reshape(MAX_OBSTACLE_SLOTS, TRACK_SLOT_FLOATS)
    if raw.shape != (MAX_OBSTACLE_SLOTS, TRACK_SLOT_FLOATS):
        raise ValueError(
            f"tracks 槽形状必须为 ({MAX_OBSTACLE_SLOTS}, {TRACK_SLOT_FLOATS}) "
            f"或展平，got {raw.shape}")
    pos = raw[:, 0:3]
    radii = raw[:, 3]
    vel = raw[:, 4:7]
    enabled = raw[:, 7]
    d_safe = raw[:, 8]
    alpha = raw[:, 9]

    radius_dot = np.zeros(MAX_OBSTACLE_SLOTS, dtype=float)
    if prev_radii is not None and dt_s is not None and dt_s > 0.0:
        previous = np.asarray(prev_radii, dtype=float).reshape(-1)
        if previous.shape != radii.shape:
            raise ValueError("prev_radii 形状必须与槽数一致")
        radius_dot = (radii - previous) / float(dt_s)
    return ObsArrays(
        pos=pos.astype(float), radii=radii.astype(float),
        enabled=enabled.astype(float), d_safe=d_safe.astype(float),
        vel=vel.astype(float), radius_dot=radius_dot, alpha=alpha.astype(float),
    )
