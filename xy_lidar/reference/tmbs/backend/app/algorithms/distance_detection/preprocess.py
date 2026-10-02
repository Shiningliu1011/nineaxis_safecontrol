"""点云预处理：体素下采样、统计离群值过滤与车辆 AABB 剔除 (PRD §§8.1、9.3)。

本模块不依赖 FastAPI、ORM、EventHub、ArtifactStore 或真实雷达。
使用 Open3D 作为算法后端，延迟导入避免模块级副作用。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# 默认最小点数：任一阶段过滤后点数低于此值视为无效。
_DEFAULT_MIN_POINTS = 10

# 零向量判断容差。
_ZERO_ATOL = 1e-12


@dataclass(frozen=True)
class Stage1Diagnostics:
    """预处理第一阶段诊断信息 (PRD §8.1 步骤 1–5)。"""

    input_points: int
    """输入点总数。"""

    removed_non_finite: int
    """删除的非有限点。"""

    removed_all_zero: int
    """删除的全零坐标点。"""

    after_clean: int
    """清洗后点数。"""

    after_range: int
    """点云范围过滤后点数。"""

    removed_by_range: int
    """点云范围过滤删除的点数。"""

    after_downsample: int
    """体素下采样后点数。"""

    after_filter: int
    """统计滤波后点数。"""

    removed_by_filter: int
    """统计滤波删除的点数。"""

    removed_by_aabb: int
    """车辆 AABB 剔除的点数。"""

    after_aabb: int
    """车辆 AABB 剔除后点数。"""

    passed: bool
    """所有阶段是否通过最低点数门槛。"""


def _make_o3d_point_cloud(points: np.ndarray) -> "open3d.geometry.PointCloud":  # type: ignore[name-defined]  # noqa: F821
    import open3d as o3d

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3].astype(np.float64))
    return pcd


def _o3d_points_to_array(pcd: "open3d.geometry.PointCloud") -> np.ndarray:  # type: ignore[name-defined]  # noqa: F821
    return np.asarray(pcd.points, dtype=np.float64)


def clean_points(points: np.ndarray) -> tuple[np.ndarray, int, int]:
    """删除非有限点和全零坐标点 (PRD §8.1 步骤 2)。

    Returns:
        (cleaned_points, n_removed_non_finite, n_removed_all_zero)
    """

    arr = np.asarray(points, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 3:
        raise ValueError(f"points must be N×3 or N×4, got shape {arr.shape}")

    # 非有限点
    finite_mask = np.all(np.isfinite(arr[:, :3]), axis=1)
    n_non_finite = int(np.sum(~finite_mask))

    # 全零坐标点
    nonzero_mask = ~np.all(np.isclose(arr[:, :3], 0.0, atol=_ZERO_ATOL), axis=1)
    n_all_zero = int(np.sum(finite_mask & ~nonzero_mask))

    clean = arr[finite_mask & nonzero_mask]
    return clean, n_non_finite, n_all_zero


def voxel_downsample(
    points: np.ndarray,
    voxel_size_m: float = 0.05,
) -> np.ndarray:
    """体素下采样 (PRD §8.1 步骤 3)。

    输入应为已清洗的 N×3 或 N×4 数组。返回 float64 N'×3 数组。
    """

    arr = np.asarray(points, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 3:
        raise ValueError(f"points must be N×3 or N×4, got shape {arr.shape}")
    if len(arr) == 0:
        return arr[:, :3].copy()

    pcd = _make_o3d_point_cloud(arr)
    down = pcd.voxel_down_sample(voxel_size=float(voxel_size_m))
    return _o3d_points_to_array(down)


def statistical_outlier_filter(
    points: np.ndarray,
    neighbors: int = 20,
    std_ratio: float = 2.0,
) -> tuple[np.ndarray, int]:
    """一次统计离群值过滤 (PRD §8.1 步骤 4)。

    Returns:
        (filtered_points, n_removed)
    """

    arr = np.asarray(points, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 3:
        raise ValueError(f"points must be N×3 or N×4, got shape {arr.shape}")

    n_before = len(arr)
    if n_before == 0:
        return arr[:, :3].copy(), 0

    pcd = _make_o3d_point_cloud(arr)
    pcd_clean, _ = pcd.remove_statistical_outlier(
        nb_neighbors=int(neighbors),
        std_ratio=float(std_ratio),
    )
    result = _o3d_points_to_array(pcd_clean)
    return result, n_before - len(result)


def remove_vehicle_aabb(
    points: np.ndarray,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
) -> tuple[np.ndarray, int]:
    """删除车辆 AABB 内部的点，包括边界点 (PRD §8.1 步骤 5)。

    边界点按「车体点」一并删除 (闭区间)。
    Returns: (outside_points, n_removed)
    """

    arr = np.asarray(points, dtype=np.float64)
    inside = (
        (arr[:, 0] >= x_min) & (arr[:, 0] <= x_max)
        & (arr[:, 1] >= y_min) & (arr[:, 1] <= y_max)
        & (arr[:, 2] >= z_min) & (arr[:, 2] <= z_max)
    )
    return arr[~inside], int(np.sum(inside))


def filter_point_cloud_range(
    points: np.ndarray,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
) -> tuple[np.ndarray, int]:
    """保留闭区间范围内的点，返回过滤结果和删除数量。"""

    arr = np.asarray(points, dtype=np.float64)
    inside = (
        (arr[:, 0] >= x_min) & (arr[:, 0] <= x_max)
        & (arr[:, 1] >= y_min) & (arr[:, 1] <= y_max)
        & (arr[:, 2] >= z_min) & (arr[:, 2] <= z_max)
    )
    return arr[inside], int(np.sum(~inside))


def preprocess_stage1(
    points: np.ndarray,
    voxel_size_m: float = 0.05,
    stat_neighbors: int = 20,
    stat_std_ratio: float = 2.0,
    vehicle_aabb: tuple[float, float, float, float, float, float] | None = None,
    min_points: int = _DEFAULT_MIN_POINTS,
    point_cloud_range: tuple[float, float, float, float, float, float] | None = None,
    stat_enabled: bool = True,
) -> tuple[np.ndarray, Stage1Diagnostics]:
    """执行预处理第一阶段：清洗 → 范围 → AABB → 体素 → 可选统计滤波。

    返回值：
        (filtered_points, diagnostics)

    任一阶段点数低于 min_points 时返回空数组且 passed=False，
    不会抛出异常。范围和 vehicle_aabb 为 None 时分别跳过对应过滤。"""

    arr = np.asarray(points, dtype=np.float64)
    n_input = len(arr)

    if n_input == 0:
        diag = Stage1Diagnostics(
            input_points=0,
            removed_non_finite=0,
            removed_all_zero=0,
            after_clean=0,
            after_range=0,
            removed_by_range=0,
            after_downsample=0,
            after_filter=0,
            removed_by_filter=0,
            removed_by_aabb=0,
            after_aabb=0,
            passed=False,
        )
        return np.empty((0, 3), dtype=np.float64), diag

    # Step 2: 清洗
    clean, n_non_finite, n_all_zero = clean_points(arr)
    n_clean = len(clean)
    if n_clean < min_points:
        diag = Stage1Diagnostics(
            input_points=n_input,
            removed_non_finite=n_non_finite,
            removed_all_zero=n_all_zero,
            after_clean=n_clean,
            after_range=0,
            removed_by_range=0,
            after_downsample=0,
            after_filter=0,
            removed_by_filter=0,
            removed_by_aabb=0,
            after_aabb=0,
            passed=False,
        )
        return np.empty((0, 3), dtype=np.float64), diag

    # The public pointCloudRange is intentionally applied before voxelization
    # so points outside the acquisition window cannot influence voxel centers.
    if point_cloud_range is not None:
        ranged, n_range_removed = filter_point_cloud_range(
            clean, *point_cloud_range
        )
    else:
        ranged = clean
        n_range_removed = 0
    n_range = len(ranged)
    if n_range < min_points:
        diag = Stage1Diagnostics(
            input_points=n_input,
            removed_non_finite=n_non_finite,
            removed_all_zero=n_all_zero,
            after_clean=n_clean,
            after_range=n_range,
            removed_by_range=n_range_removed,
            after_downsample=0,
            after_filter=0,
            removed_by_filter=0,
            removed_by_aabb=0,
            after_aabb=0,
            passed=False,
        )
        return np.empty((0, 3), dtype=np.float64), diag

    # Step 3: AABB 剔除。必须在体素化前完成，避免跨 AABB 边界的
    # 车体/环境点质心污染保留侧。
    if vehicle_aabb is not None:
        outside, n_aabb_removed = remove_vehicle_aabb(ranged, *vehicle_aabb)
    else:
        outside = ranged
        n_aabb_removed = 0
    n_after_aabb = len(outside)
    if n_after_aabb < min_points:
        diag = Stage1Diagnostics(
            input_points=n_input,
            removed_non_finite=n_non_finite,
            removed_all_zero=n_all_zero,
            after_clean=n_clean,
            after_range=n_range,
            removed_by_range=n_range_removed,
            after_downsample=0,
            after_filter=0,
            removed_by_filter=0,
            removed_by_aabb=n_aabb_removed,
            after_aabb=n_after_aabb,
            passed=False,
        )
        return np.empty((0, 3), dtype=np.float64), diag

    # Step 4: 体素下采样
    down = voxel_downsample(outside, voxel_size_m=voxel_size_m)
    n_down = len(down)
    if n_down < min_points:
        diag = Stage1Diagnostics(
            input_points=n_input,
            removed_non_finite=n_non_finite,
            removed_all_zero=n_all_zero,
            after_clean=n_clean,
            after_range=n_range,
            removed_by_range=n_range_removed,
            after_downsample=n_down,
            after_filter=0,
            removed_by_filter=0,
            removed_by_aabb=n_aabb_removed,
            after_aabb=n_after_aabb,
            passed=False,
        )
        return np.empty((0, 3), dtype=np.float64), diag

    # Step 5: 可选统计滤波
    if stat_enabled:
        filtered, n_removed = statistical_outlier_filter(
            down,
            neighbors=stat_neighbors,
            std_ratio=stat_std_ratio,
        )
    else:
        filtered = down
        n_removed = 0
    n_filtered = len(filtered)
    passed = n_filtered >= min_points

    diag = Stage1Diagnostics(
        input_points=n_input,
        removed_non_finite=n_non_finite,
        removed_all_zero=n_all_zero,
        after_clean=n_clean,
        after_range=n_range,
        removed_by_range=n_range_removed,
        after_downsample=n_down,
        after_filter=n_filtered,
        removed_by_filter=n_removed,
        removed_by_aabb=n_aabb_removed,
        after_aabb=n_after_aabb,
        passed=passed,
    )

    if not passed:
        return np.empty((0, 3), dtype=np.float64), diag

    return filtered[:, :3], diag
