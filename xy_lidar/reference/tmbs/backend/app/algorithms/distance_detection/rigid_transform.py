"""刚体矩阵校验与点云坐标变换 (PRD §4.1)。

本模块不依赖 FastAPI、ORM、EventHub、ArtifactStore 或真实雷达。
仅使用 numpy，可在任意 Python 环境中独立运行。
"""

from __future__ import annotations

import numpy as np

# ── 常量 ────────────────────────────────────────────────────────────────

_IDENTITY_4X4 = np.eye(4, dtype=np.float64)
_LAST_ROW = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)

# 正交性检查容差。在数值稳定的合成矩阵（单位阵/纯旋转）上应严格通过；
# 同时容忍浮点累积误差。
_ORTHO_ATOL = 1e-10
_DET_ATOL = 1e-8


# ── 公开 API ───────────────────────────────────────────────────────────


def validate_rigid_matrix(matrix: np.ndarray) -> np.ndarray:
    """校验 4×4 刚体变换矩阵 (PRD §4.1)。

    要求：
    - 形状为 (4, 4)
    - 所有值为有限浮点数
    - 末行严格为 [0, 0, 0, 1]
    - 左上 3×3 旋转部分正交 (RᵀR ≈ I)
    - 旋转部分行列式约为 +1

    通过校验时返回 float64 副本；失败时抛出 ValueError。
    """

    arr = np.asarray(matrix, dtype=np.float64)

    if arr.shape != (4, 4):
        raise ValueError(
            f"rigid matrix must be 4×4, got {arr.shape[0]}×{arr.shape[1]}"
        )

    if not np.all(np.isfinite(arr)):
        raise ValueError("rigid matrix must not contain NaN or Inf")

    if not np.allclose(arr[3], _LAST_ROW, atol=1e-6):
        raise ValueError(
            f"last row must be [0, 0, 0, 1], got {arr[3].tolist()}"
        )

    rotation = arr[:3, :3]

    # RᵀR 应等于 I₃
    deviation = rotation.T @ rotation - np.eye(3)
    if not np.allclose(deviation, 0.0, atol=_ORTHO_ATOL):
        raise ValueError(
            "rotation submatrix is not orthogonal; "
            f"max |RᵀR - I| deviation = {float(np.max(np.abs(deviation))):.2e}"
        )

    det = float(np.linalg.det(rotation))
    if abs(det - 1.0) > _DET_ATOL:
        raise ValueError(
            f"rotation determinant must be ≈ +1, got {det:.6e}"
        )

    return arr.copy()


def identity_matrix() -> np.ndarray:
    """返回 float64 4×4 单位矩阵。"""

    return _IDENTITY_4X4.copy()


def apply_rigid_transform(
    points: np.ndarray,
    matrix: np.ndarray,
) -> np.ndarray:
    """对 N×3 或 N×4 点云的前三列应用 4×4 刚体变换。

    输入点云不会被修改；返回新的 float64 数组。
    矩阵会先通过 :func:`validate_rigid_matrix` 校验。
    """

    validated = validate_rigid_matrix(matrix)
    arr = np.asarray(points, dtype=np.float64)

    if arr.ndim != 2 or arr.shape[1] < 3:
        raise ValueError(
            f"points must be N×3 or N×4, got shape {arr.shape}"
        )

    rotation = validated[:3, :3]
    translation = validated[:3, 3]

    result = arr.copy()
    result[:, :3] = arr[:, :3] @ rotation.T + translation
    return result


def transform_lidar_to_vehicle(
    points: np.ndarray,
    lidar_to_reference: np.ndarray,
    reference_to_vehicle: np.ndarray,
) -> np.ndarray:
    """将点云从 lidar 坐标系变换到 vehicle 坐标系 (PRD §4.1)。

    变换链：lidar → reference_lidar → vehicle。
    两个矩阵分别通过 :func:`validate_rigid_matrix` 校验后，
    按 T_vehicle = T_ref→veh @ T_lidar→ref 复合。
    """

    T_lr = validate_rigid_matrix(lidar_to_reference)
    T_rv = validate_rigid_matrix(reference_to_vehicle)

    # 复合：先 lidar→ref，再 ref→vehicle
    T_lv = T_rv @ T_lr

    return apply_rigid_transform(points, T_lv)


def make_rigid_transform(
    rotation: np.ndarray,
    translation: np.ndarray,
) -> np.ndarray:
    """由 3×3 旋转矩阵和 3 元素平移向量构造 4×4 刚体变换矩阵。

    旋转矩阵必须通过正交性和行列式 ≈ +1 校验。"""

    rot = np.asarray(rotation, dtype=np.float64)
    trans = np.asarray(translation, dtype=np.float64)

    if rot.shape != (3, 3):
        raise ValueError(f"rotation must be 3×3, got {rot.shape}")
    if trans.shape != (3,):
        raise ValueError(f"translation must be length 3, got {trans.shape}")

    if not np.all(np.isfinite(rot)):
        raise ValueError("rotation must not contain NaN or Inf")
    if not np.all(np.isfinite(trans)):
        raise ValueError("translation must not contain NaN or Inf")

    deviation = rot.T @ rot - np.eye(3)
    if not np.allclose(deviation, 0.0, atol=_ORTHO_ATOL):
        raise ValueError(
            "rotation is not orthogonal; "
            f"max |RᵀR - I| deviation = {float(np.max(np.abs(deviation))):.2e}"
        )

    det = float(np.linalg.det(rot))
    if abs(det - 1.0) > _DET_ATOL:
        raise ValueError(f"rotation determinant must be ≈ +1, got {det:.6e}")

    matrix = _IDENTITY_4X4.copy()
    matrix[:3, :3] = rot
    matrix[:3, 3] = trans
    return matrix
