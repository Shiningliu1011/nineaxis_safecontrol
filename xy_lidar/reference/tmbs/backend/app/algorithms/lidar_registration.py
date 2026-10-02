"""雷达阵列外参无目标自动标定（配准算法内核）。

从遗留 ``lidar_array_calibration`` 服务中拆出的纯算法部分：基于默认外参和
多尺度 ICP/GICP 为每台设备生成外参 1 候选，并输出质量门控结果。本模块不
访问数据库、不依赖驱动，可被平台标定流程与离线工具复用。
"""

from __future__ import annotations

import math
from typing import Any, Callable, Mapping, Optional

import numpy as np

from app.utils.calibration import (
    apply_matrix4x4,
    effective_lidar_matrix,
    matrix4x4_to_euler_zyx,
    normalize_lidar_calibration,
)
from app.utils.pcd import read_pcd
from app.algorithms.logging_port import get_algorithm_logger

logger = get_algorithm_logger(__name__, component="algorithm.registration")

DEFAULT_VOXEL_SCALES = [0.20, 0.10, 0.05]
MIN_DEVICE_POINTS = 100


def run_targetless_registration(
    *,
    pcd_path: str,
    reference_device_id: int,
    device_ids: Optional[list[int]],
    default_calibrations_by_device_id: Mapping[str, Any],
    voxel_scales: Optional[list[float]] = None,
    progress_cb: Optional[Callable[[Mapping[str, Any]], None]] = None,
) -> dict[str, Any]:
    """基于默认外参和多尺度 ICP/GICP 生成外参 1 候选。"""

    try:
        result = _compute_targetless_registration(
            pcd_path=pcd_path,
            reference_device_id=reference_device_id,
            device_ids=device_ids,
            default_calibrations_by_device_id=default_calibrations_by_device_id,
            voxel_scales=voxel_scales,
            progress_cb=progress_cb,
        )
        return {"ok": True, **result}
    except FileNotFoundError:
        logger.log_event(
            "algorithm.registration.input.missing",
            "配准输入 PCD 不可用",
            level="WARNING",
            context={"stage": "capture", "status": "invalid", "code": "PCD_NOT_FOUND"},
        )
        return {"ok": False, "error": "PCD 文件不存在"}
    except ValueError as exc:
        logger.log_event(
            "algorithm.registration.input.invalid",
            "配准输入无效",
            level="WARNING",
            context={"stage": "compute", "status": "invalid", "code": "INVALID_INPUT"},
        )
        return {"ok": False, "error": str(exc)}
    except ImportError as exc:
        logger.log_event(
            "algorithm.registration.compute.failed",
            "配准依赖加载失败",
            level="ERROR",
            context={"stage": "compute", "status": "failed", "code": "DEPENDENCY_ERROR"},
            exc_info=True,
        )
        return {"ok": False, "error": str(exc)}


def _compute_targetless_registration(
    *,
    pcd_path: str,
    reference_device_id: int,
    device_ids: Optional[list[int]],
    default_calibrations_by_device_id: Mapping[str, Any],
    voxel_scales: Optional[list[float]] = None,
    progress_cb: Optional[Callable[[Mapping[str, Any]], None]] = None,
) -> dict[str, Any]:
    """执行标定计算。"""

    scales = _normalize_voxel_scales(voxel_scales)
    pcd_data = read_pcd(pcd_path)
    dev_ids = _read_device_ids(pcd_data.attributes)
    requested_ids = _normalize_device_ids(device_ids or sorted(set(dev_ids.tolist())))
    if reference_device_id not in requested_ids:
        requested_ids.insert(0, int(reference_device_id))

    points_by_device = {
        device_id: pcd_data.points[dev_ids == device_id]
        for device_id in requested_ids
    }
    if points_by_device.get(reference_device_id, np.empty((0, 3))).shape[0] < MIN_DEVICE_POINTS:
        raise ValueError(f"referenceDeviceId={reference_device_id} 点数不足，无法标定")

    default_snapshots = {
        str(device_id): normalize_lidar_calibration(default_calibrations_by_device_id.get(str(device_id)) or {})
        for device_id in requested_ids
    }
    target_points = _apply_default(points_by_device[reference_device_id], default_snapshots[str(reference_device_id)])
    target_pcd = _preprocess_points(target_points, voxel_size=min(scales))

    registration_device_ids = [
        device_id
        for device_id in requested_ids
        if device_id != reference_device_id
        and points_by_device.get(device_id, np.empty((0, 3))).shape[0] >= MIN_DEVICE_POINTS
    ]
    total_steps = max(1, len(registration_device_ids) * len(scales))
    completed_before_device = 0
    results: dict[str, Any] = {}
    for device_id in requested_ids:
        if device_id == reference_device_id:
            continue
        source_raw = points_by_device.get(device_id, np.empty((0, 3)))
        if source_raw.shape[0] < MIN_DEVICE_POINTS:
            results[str(device_id)] = {
                "status": "failed",
                "reason": "device_points_too_few",
                "pointCount": int(source_raw.shape[0]),
            }
            continue
        default_calibration = default_snapshots[str(device_id)]
        source_default = _apply_default(source_raw, default_calibration)
        registration = _register_multiscale(
            source_default,
            target_points,
            voxel_scales=scales,
            progress_cb=(
                None
                if progress_cb is None
                else lambda scale_progress, did=device_id, base=completed_before_device: progress_cb({
                    "deviceId": int(did),
                    "deviceIndex": registration_device_ids.index(did) + 1,
                    "deviceCount": len(registration_device_ids),
                    "scaleIndex": int(scale_progress["scaleIndex"]),
                    "scaleCount": len(scales),
                    "voxelSize": float(scale_progress["voxelSize"]),
                    "method": scale_progress["method"],
                    "completedSteps": base + int(scale_progress["scaleIndex"]),
                    "totalSteps": total_steps,
                })
            ),
        )
        completed_before_device += len(scales)
        default_matrix = effective_lidar_matrix(default_calibration)
        delta = np.asarray(registration["deltaMatrix4x4"], dtype=np.float64)
        candidate_matrix = delta @ default_matrix
        candidate = normalize_lidar_calibration({
            "transformType": "matrix4x4",
            "sourceFrame": "lidar",
            "targetFrame": "reference_lidar",
            "matrix4x4": candidate_matrix.tolist(),
            "version": f"candidate-device-{device_id}",
            "note": "targetless ICP/GICP candidate",
        })
        results[str(device_id)] = {
            "status": "candidate",
            "defaultCalibration": default_calibration,
            "delta": registration,
            "candidateCalibration": candidate,
            "quality": _quality_gate(registration),
            "pointCount": int(source_raw.shape[0]),
        }

    return {
        "status": "completed",
        "method": "targetless_multiscale_icp",
        "referenceDeviceId": int(reference_device_id),
        "deviceIds": requested_ids,
        "referencePointCount": int(points_by_device[reference_device_id].shape[0]),
        "referenceDownPointCount": int(np.asarray(target_pcd.points).shape[0]),
        "results": results,
    }


def _read_device_ids(attributes: Mapping[str, np.ndarray]) -> np.ndarray:
    """从 PCD attributes 读取 devId 字段。"""

    dev = attributes.get("devId")
    if dev is None:
        dev = attributes.get("deviceId")
    if dev is None:
        raise ValueError("PCD 缺少 devId 字段，无法按设备拆分")
    return np.asarray(dev, dtype=np.int32)


def _normalize_device_ids(device_ids: list[int]) -> list[int]:
    """去重并校验设备 ID。"""

    normalized: list[int] = []
    seen: set[int] = set()
    for item in device_ids:
        value = int(item)
        if value <= 0:
            raise ValueError("deviceIds 必须为正整数")
        if value not in seen:
            seen.add(value)
            normalized.append(value)
    return normalized


def _normalize_voxel_scales(voxel_scales: Optional[list[float]]) -> list[float]:
    """校验多尺度体素参数。"""

    scales = [float(item) for item in (voxel_scales or DEFAULT_VOXEL_SCALES)]
    if not scales:
        raise ValueError("voxelScales 不能为空")
    if any(not math.isfinite(item) or item <= 0 for item in scales):
        raise ValueError("voxelScales 必须全部为正数")
    return scales


def _apply_default(points: np.ndarray, calibration: Mapping[str, Any]) -> np.ndarray:
    """应用默认外参。"""

    if not calibration.get("enabled", True):
        return points
    padded = np.zeros((points.shape[0], 4), dtype=np.float64)
    padded[:, :3] = points
    transformed = apply_matrix4x4(padded, effective_lidar_matrix(calibration).tolist())
    return transformed[:, :3]


def _preprocess_points(points: np.ndarray, *, voxel_size: float) -> Any:
    """构造并预处理 Open3D 点云。"""

    import open3d as o3d

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    if voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    if len(pcd.points) >= 4:
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=max(voxel_size * 3.0, 0.1),
                max_nn=30,
            )
        )
    return pcd


def _register_multiscale(
    source_points: np.ndarray,
    target_points: np.ndarray,
    *,
    voxel_scales: list[float],
    progress_cb: Optional[Callable[[Mapping[str, Any]], None]] = None,
) -> dict[str, Any]:
    """执行多尺度 point-to-plane ICP，优先尝试 GICP。"""

    import open3d as o3d

    transform = np.eye(4, dtype=np.float64)
    scale_metrics: list[dict[str, Any]] = []
    final_result = None
    for scale_index, voxel in enumerate(voxel_scales, start=1):
        source = _preprocess_points(source_points, voxel_size=voxel)
        target = _preprocess_points(target_points, voxel_size=voxel)
        threshold = max(voxel * 3.0, 0.05)
        if hasattr(o3d.pipelines.registration, "registration_generalized_icp"):
            result = o3d.pipelines.registration.registration_generalized_icp(
                source,
                target,
                threshold,
                transform,
                o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
            )
            method = "generalized_icp"
        else:
            result = o3d.pipelines.registration.registration_icp(
                source,
                target,
                threshold,
                transform,
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            )
            method = "point_to_plane_icp"
        transform = np.asarray(result.transformation, dtype=np.float64)
        final_result = result
        scale_metrics.append({
            "voxelSize": voxel,
            "method": method,
            "fitness": round(float(result.fitness), 6),
            "inlierRmse": round(float(result.inlier_rmse), 6),
            "sourcePoints": int(len(source.points)),
            "targetPoints": int(len(target.points)),
        })
        if progress_cb is not None:
            progress_cb({
                "scaleIndex": scale_index,
                "voxelSize": voxel,
                "method": method,
            })

    if final_result is None:
        raise ValueError("ICP/GICP 未执行")
    return {
        "deltaMatrix4x4": transform.tolist(),
        "deltaEuler": matrix4x4_to_euler_zyx(transform.tolist()),
        "fitness": round(float(final_result.fitness), 6),
        "inlierRmse": round(float(final_result.inlier_rmse), 6),
        "deltaTranslationNorm": round(float(np.linalg.norm(transform[:3, 3])), 6),
        "deltaRotationDeg": round(_rotation_angle_deg(transform[:3, :3]), 6),
        "scales": scale_metrics,
    }


def _rotation_angle_deg(rotation: np.ndarray) -> float:
    """计算旋转矩阵角度。"""

    value = (float(np.trace(rotation)) - 1.0) / 2.0
    value = min(1.0, max(-1.0, value))
    return math.degrees(math.acos(value))


def _quality_gate(registration: Mapping[str, Any]) -> dict[str, Any]:
    """生成候选质量门控结果。"""

    fitness = float(registration.get("fitness", 0.0))
    rmse = float(registration.get("inlierRmse", 999.0))
    delta_t = float(registration.get("deltaTranslationNorm", 999.0))
    delta_r = float(registration.get("deltaRotationDeg", 999.0))
    accepted = fitness >= 0.35 and rmse <= 0.15 and delta_t <= 0.30 and delta_r <= 8.0
    thresholds = {
        "fitnessMin": 0.35,
        "inlierRmseMax": 0.15,
        "deltaTranslationNormMax": 0.30,
        "deltaRotationDegMax": 8.0,
    }
    return {
        "accepted": accepted,
        "requiresManualReview": not accepted,
        "thresholds": thresholds,
        "checks": [
            {"name": "fitness", "value": fitness, "operator": ">=", "threshold": thresholds["fitnessMin"], "passed": fitness >= thresholds["fitnessMin"]},
            {"name": "inlierRmse", "value": rmse, "operator": "<=", "threshold": thresholds["inlierRmseMax"], "passed": rmse <= thresholds["inlierRmseMax"]},
            {"name": "deltaTranslationNorm", "value": delta_t, "operator": "<=", "threshold": thresholds["deltaTranslationNormMax"], "passed": delta_t <= thresholds["deltaTranslationNormMax"]},
            {"name": "deltaRotationDeg", "value": delta_r, "operator": "<=", "threshold": thresholds["deltaRotationDegMax"], "passed": delta_r <= thresholds["deltaRotationDegMax"]},
        ],
    }
