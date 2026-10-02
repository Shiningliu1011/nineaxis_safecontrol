"""标定参数规范化与坐标变换工具。"""

from __future__ import annotations

import math
from typing import Any, Mapping

import numpy as np

_IDENTITY_MATRIX = np.eye(4, dtype=np.float64)
_EULER_ZYX = "euler_zyx"
_MATRIX4X4 = "matrix4x4"
_SOURCE_FRAME_LIDAR = "lidar"
_TARGET_FRAME_REFERENCE = "reference_lidar"
_SOURCE_FRAME_REFERENCE = "reference_lidar"
_TARGET_FRAME_VEHICLE = "vehicle"


def euler_zyx_to_matrix4x4(
    tx: float,
    ty: float,
    tz: float,
    rx: float,
    ry: float,
    rz: float,
) -> list[list[float]]:
    """将 ZYX 欧拉角（度）和平移转换为 4x4 齐次矩阵。"""

    rx_rad = math.radians(rx)
    ry_rad = math.radians(ry)
    rz_rad = math.radians(rz)
    cx, sx = math.cos(rx_rad), math.sin(rx_rad)
    cy, sy = math.cos(ry_rad), math.sin(ry_rad)
    cz, sz = math.cos(rz_rad), math.sin(rz_rad)
    rotation = np.array(
        [
            [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
            [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
            [-sy, cy * sx, cy * cx],
        ],
        dtype=np.float64,
    )
    matrix = _IDENTITY_MATRIX.copy()
    matrix[:3, :3] = rotation
    matrix[:3, 3] = [tx, ty, tz]
    return matrix.tolist()


def matrix4x4_to_euler_zyx(matrix4x4: Any) -> dict[str, float | str]:
    """将 4x4 齐次矩阵转换为项目约定的 ZYX 欧拉角（度）和平移。

    旋转约定与 :func:`euler_zyx_to_matrix4x4` 一致，即
    ``R = Rz(rz) @ Ry(ry) @ Rx(rx)``。万向锁时固定 ``rz = 0``，
    以获得确定且有限的表示。
    """

    matrix = np.asarray(_coerce_matrix4x4(matrix4x4), dtype=np.float64)
    rotation = matrix[:3, :3]
    sy = math.hypot(float(rotation[0, 0]), float(rotation[1, 0]))
    if sy > 1e-8:
        rx = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        ry = math.atan2(-float(rotation[2, 0]), sy)
        rz = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    else:
        rx = math.atan2(-float(rotation[1, 2]), float(rotation[1, 1]))
        ry = math.atan2(-float(rotation[2, 0]), sy)
        rz = 0.0
    return {
        "tx": float(matrix[0, 3]),
        "ty": float(matrix[1, 3]),
        "tz": float(matrix[2, 3]),
        "rx": math.degrees(rx),
        "ry": math.degrees(ry),
        "rz": math.degrees(rz),
        "unit": "deg",
        "order": "zyx",
    }


def _coerce_matrix4x4(value: Any) -> list[list[float]]:
    """校验并转换 4x4 矩阵。"""

    arr = np.asarray(value, dtype=np.float64)
    if arr.shape != (4, 4):
        raise ValueError("matrix4x4 必须是 4x4 数值矩阵")
    if not np.all(np.isfinite(arr)):
        raise ValueError("matrix4x4 不能包含 NaN 或 Inf")
    if not np.allclose(arr[3], np.array([0.0, 0.0, 0.0, 1.0]), atol=1e-6):
        raise ValueError("matrix4x4 最后一行必须为 [0, 0, 0, 1]")
    rotation = arr[:3, :3]
    if not np.allclose(
        rotation.T @ rotation,
        np.eye(3, dtype=np.float64),
        atol=1e-6,
        rtol=0.0,
    ) or not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-6):
        raise ValueError("matrix4x4 旋转部分必须为刚体变换")
    return arr.tolist()


def validate_rigid_matrix4x4(value: Any) -> list[list[float]]:
    """Validate and normalize the public rigid-transform matrix payload."""

    return _coerce_matrix4x4(value)


def normalize_lidar_calibration(params: Mapping[str, Any] | None) -> dict[str, Any]:
    """规范化外参 1：单雷达到 reference_lidar。"""

    raw = dict(params or {})
    transform_type = raw.get("transformType") or raw.get("transform_type") or _EULER_ZYX
    source_frame = raw.get("sourceFrame") or raw.get("source_frame") or _SOURCE_FRAME_LIDAR
    target_frame = raw.get("targetFrame") or raw.get("target_frame") or _TARGET_FRAME_REFERENCE
    enabled = bool(raw.get("enabled", True))
    version = raw.get("version") or ""
    note = raw.get("note") or ""

    if transform_type == _MATRIX4X4:
        matrix = _coerce_matrix4x4(raw.get("matrix4x4") or raw.get("matrix_json"))
        euler = matrix4x4_to_euler_zyx(matrix)
    else:
        euler_payload = raw.get("euler") if isinstance(raw.get("euler"), dict) else raw
        euler = {
            "tx": float(euler_payload.get("tx", 0.0)),
            "ty": float(euler_payload.get("ty", 0.0)),
            "tz": float(euler_payload.get("tz", 0.0)),
            "rx": float(euler_payload.get("rx", 0.0)),
            "ry": float(euler_payload.get("ry", 0.0)),
            "rz": float(euler_payload.get("rz", 0.0)),
            "unit": "deg",
            "order": "zyx",
        }
        transform_type = _EULER_ZYX
        matrix = euler_zyx_to_matrix4x4(
            euler["tx"],
            euler["ty"],
            euler["tz"],
            euler["rx"],
            euler["ry"],
            euler["rz"],
        )

    normalized = {
        "transformType": transform_type,
        "sourceFrame": source_frame,
        "targetFrame": target_frame,
        "enabled": enabled,
        "euler": euler,
        "matrix4x4": matrix,
        "version": version,
        "note": note,
    }
    return normalized


def effective_lidar_matrix(calibration: Mapping[str, Any]) -> np.ndarray:
    """Return the effective matrix from an explicit matrix calibration document."""
    matrix = _coerce_matrix4x4(calibration.get("matrix4x4"))
    return np.asarray(matrix if calibration.get("enabled", True) else _IDENTITY_MATRIX,
                      dtype=np.float64)


def normalize_vehicle_mount_calibration(params: Mapping[str, Any] | None) -> dict[str, Any]:
    """规范化外参 2：reference_lidar 到 vehicle。"""

    raw = dict(params or {})
    raw.setdefault("transformType", raw.get("transform_type") or _MATRIX4X4)
    raw.setdefault("sourceFrame", raw.get("source_frame") or _SOURCE_FRAME_REFERENCE)
    raw.setdefault("targetFrame", raw.get("target_frame") or _TARGET_FRAME_VEHICLE)
    if raw["sourceFrame"] != _SOURCE_FRAME_REFERENCE:
        raise ValueError("外参 2 sourceFrame 必须为 reference_lidar")
    if raw["targetFrame"] != _TARGET_FRAME_VEHICLE:
        raise ValueError("外参 2 targetFrame 必须为 vehicle")
    if raw["transformType"] == _MATRIX4X4 and not (raw.get("matrix4x4") or raw.get("matrix_json")):
        raw["matrix4x4"] = _IDENTITY_MATRIX.tolist()
    calibration = normalize_lidar_calibration(raw)
    calibration["sourceFrame"] = _SOURCE_FRAME_REFERENCE
    calibration["targetFrame"] = _TARGET_FRAME_VEHICLE
    return calibration


def legacy_fields_from_calibration(calibration: Mapping[str, Any]) -> dict[str, float]:
    """从规范化标定中提取旧 tx/ty/tz/rx/ry/rz 字段。"""

    euler = calibration.get("euler") or {}
    return {
        "tx": float(euler.get("tx", 0.0)),
        "ty": float(euler.get("ty", 0.0)),
        "tz": float(euler.get("tz", 0.0)),
        "rx": float(euler.get("rx", 0.0)),
        "ry": float(euler.get("ry", 0.0)),
        "rz": float(euler.get("rz", 0.0)),
    }


def calibration_from_model(params_model: Any) -> dict[str, Any]:
    """从 ORM 外参对象构造规范化标定。"""

    if params_model is None:
        return normalize_lidar_calibration(None)
    return normalize_lidar_calibration({
        "tx": params_model.tx,
        "ty": params_model.ty,
        "tz": params_model.tz,
        "rx": params_model.rx,
        "ry": params_model.ry,
        "rz": params_model.rz,
        "transformType": params_model.transform_type or "euler_zyx",
        "matrix4x4": params_model.matrix_json,
        "sourceFrame": params_model.source_frame or "lidar",
        "targetFrame": params_model.target_frame or "reference_lidar",
        "enabled": params_model.enabled if params_model.enabled is not None else True,
        "version": params_model.version or "",
        "note": params_model.note or "",
    })


def apply_matrix4x4(points: np.ndarray, matrix4x4: Any) -> np.ndarray:
    """对点云前三列应用 4x4 齐次矩阵。"""

    matrix = np.asarray(_coerce_matrix4x4(matrix4x4), dtype=np.float32)
    result = points.copy()
    xyz = result[:, :3]
    result[:, :3] = xyz @ matrix[:3, :3].T + matrix[:3, 3]
    return result
