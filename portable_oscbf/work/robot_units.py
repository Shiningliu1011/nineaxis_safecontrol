import numpy as np
from numpy.typing import ArrayLike

from work.kinematics_data import N_JOINTS


def _joint_array(values: ArrayLike) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    if values.ndim < 1 or values.shape[-1] != N_JOINTS or not np.all(np.isfinite(values)):
        raise ValueError("关节数组的最后一维必须为 9，且所有数值必须有限")
    return values


def joints_mm_deg_to_si(values: ArrayLike) -> np.ndarray:
    """项目关节输入转换为控制内核单位：J1 mm → m，J2–J9 ° → rad。"""
    converted = _joint_array(values).copy()
    converted[..., 0] /= 1000.0
    converted[..., 1:] = np.deg2rad(converted[..., 1:])
    return converted


def joints_si_to_mm_deg(values: ArrayLike) -> np.ndarray:
    """控制内核关节数据转换为项目展示单位：J1 m → mm，J2–J9 rad → °。"""
    converted = _joint_array(values).copy()
    converted[..., 0] *= 1000.0
    converted[..., 1:] = np.rad2deg(converted[..., 1:])
    return converted
