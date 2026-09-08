#!/usr/bin/env python3
"""Per-control-step cache for robot kinematics used by OSCBF."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

import numpy as np


@dataclass(frozen=True)
class RobotStepCache:
    q: np.ndarray
    T_all: Dict[str, np.ndarray]
    ee_pos: np.ndarray
    ee_rot: np.ndarray
    J_s: np.ndarray
    J_pos: np.ndarray
    J_full: np.ndarray


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])


def point_jacobian_from_spatial(J_s: np.ndarray, link_idx: int,
                                point_world: np.ndarray, n_joints: int = 9) -> np.ndarray:
    """Compute a point linear-velocity Jacobian from a cached spatial Jacobian."""
    n_act = min(int(link_idx), int(n_joints))
    J_pos_full = J_s[3:, :] - _skew(point_world) @ J_s[:3, :]
    J_pos = np.zeros((3, int(n_joints)))
    J_pos[:, :n_act] = J_pos_full[:, :n_act]
    return J_pos


def build_robot_step_cache(kin, q: np.ndarray) -> RobotStepCache:
    """Build all kinematics commonly reused during one controller step.

    Uses fused FK+Jacobian when available (single-pass, ~40% faster).
    """
    q_copy = np.asarray(q, dtype=float).copy()

    # 融合正运动学+雅可比 (单遍遍历, 避免重复 _twist_exp)
    if hasattr(kin, 'forward_kinematics_and_jacobian'):
        T_all, J_s = kin.forward_kinematics_and_jacobian(q_copy)
    else:
        T_all = kin.forward_kinematics(q_copy)
        J_s = kin.compute_spatial_jacobian_world(q_copy)

    T_ee = T_all["ee_link"]
    J_pos = J_s[3:, :] - _skew(T_ee[:3, 3]) @ J_s[:3, :]
    J_full = np.vstack([J_pos, J_s[:3, :]])
    return RobotStepCache(
        q=q_copy,
        T_all={name: T.copy() for name, T in T_all.items()},
        ee_pos=T_ee[:3, 3].copy(),
        ee_rot=T_ee[:3, :3].copy(),
        J_s=J_s.copy(),
        J_pos=J_pos.copy(),
        J_full=J_full.copy(),
    )
