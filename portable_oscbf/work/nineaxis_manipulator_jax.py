#!/usr/bin/env python3
"""
nineaxis_manipulator_jax.py
===========================
9-DOF 冗余臂 JAX 运动学封装，适配 cbfpy 框架。

将当前 POE FK/Jacobian 实现为 JAX 版本，支持自动微分。
"""

import jax
import jax.numpy as jnp
import numpy as np
from typing import Tuple

from work.collision_envelope import (
    ENVIRONMENT_SPHERE_LINK_INDICES,
    ENVIRONMENT_SPHERE_LOCAL_CENTERS_M,
    ENVIRONMENT_SPHERE_RADII_M,
    NUM_ENVIRONMENT_COLLISION_SPHERES,
)


# URDF 关节链参数 — 单一数据源在 nineaxis_kinematics.py（M1 已验证其与
# URDF 一致）；此处只导入，禁止在本模块重新抄写。
from work.nineaxis_kinematics import JOINT_CHAIN


def _skew_jax(v):
    """3×3 反对称矩阵 (JAX)"""
    return jnp.array([[0, -v[2], v[1]],
                      [v[2], 0, -v[0]],
                      [-v[1], v[0], 0]])


def _adjoint_jax(T):
    """6×6 伴随变换矩阵 Ad_T (JAX)"""
    Rmat = T[:3, :3]
    p = T[:3, 3]
    Ad = jnp.zeros((6, 6))
    Ad = Ad.at[:3, :3].set(Rmat)
    Ad = Ad.at[3:, :3].set(_skew_jax(p) @ Rmat)
    Ad = Ad.at[3:, 3:].set(Rmat)
    return Ad


def _twist_exp_jax(screw, theta):
    """螺旋运动指数映射 → 4×4 齐次矩阵 (JAX 版本)"""
    omega = screw[:3]
    v = screw[3:]
    omega_norm = jnp.linalg.norm(omega)

    # 纯平动 (棱柱关节)
    T_prismatic = jnp.eye(4).at[:3, 3].set(v * theta)

    # 旋转关节
    omega_hat = omega / jnp.maximum(omega_norm, 1e-12)
    w_skew = _skew_jax(omega_hat)
    ct, st = jnp.cos(omega_norm * theta), jnp.sin(omega_norm * theta)
    Rmat = jnp.eye(3) + st * w_skew + (1 - ct) * w_skew @ w_skew
    G = (jnp.eye(3) * theta +
         (1 - ct) / jnp.maximum(omega_norm, 1e-12) * w_skew +
         (theta - st / jnp.maximum(omega_norm, 1e-12)) * w_skew @ w_skew)
    p_rot = G @ v
    T_rot = jnp.eye(4).at[:3, :3].set(Rmat).at[:3, 3].set(p_rot)

    # 根据 omega_norm 选择
    return jnp.where(omega_norm < 1e-12, T_prismatic, T_rot)


# 全局函数 (可用于 JAX jit)
def ee_position_fn(q, S_axes, M_zero):
    """末端位置 FK (纯函数)"""
    T = jnp.eye(4)
    for i in range(9):
        T = T @ _twist_exp_jax(S_axes[i], q[i])
    T_ee = T @ M_zero
    return T_ee[:3, 3]


def ee_rotation_fn(q, S_axes, M_zero):
    """末端姿态 FK (纯函数)"""
    T = jnp.eye(4)
    for i in range(9):
        T = T @ _twist_exp_jax(S_axes[i], q[i])
    T_ee = T @ M_zero
    return T_ee[:3, :3]


def ee_transform_fn(q, S_axes, M_zero):
    """末端齐次变换 (纯函数)"""
    T = jnp.eye(4)
    for i in range(9):
        T = T @ _twist_exp_jax(S_axes[i], q[i])
    return T @ M_zero


def spatial_jacobian_fn(q, S_axes):
    """空间雅可比 J_s ∈ R^{6×9} (纯函数)"""
    J_s = jnp.zeros((6, 9))
    T_acc = jnp.eye(4)
    for i in range(9):
        J_s = J_s.at[:, i].set(_adjoint_jax(T_acc) @ S_axes[i])
        T_acc = T_acc @ _twist_exp_jax(S_axes[i], q[i])
    return J_s


def position_jacobian_fn(q, S_axes, M_zero):
    """位置雅可比 J_pos ∈ R^{3×9} (纯函数)"""
    J_s = spatial_jacobian_fn(q, S_axes)
    p_ee = ee_position_fn(q, S_axes, M_zero)
    return J_s[3:, :] - _skew_jax(p_ee) @ J_s[:3, :]


def full_jacobian_fn(q, S_axes, M_zero):
    """完整雅可比 [J_pos; J_rot] ∈ R^{6×9} (纯函数)"""
    J_s = spatial_jacobian_fn(q, S_axes)
    p_ee = ee_position_fn(q, S_axes, M_zero)
    J_pos = J_s[3:, :] - _skew_jax(p_ee) @ J_s[:3, :]
    J_rot = J_s[:3, :]
    return jnp.vstack([J_pos, J_rot])


class NineaxisManipulatorJAX:
    """9-DOF 冗余臂 JAX 运动学封装"""

    def __init__(self):
        self.num_joints = 9

        # 关节限位 (基于电机模组参数)
        self.joint_lower_limits = jnp.array([
            0.0, -1.5708, -1.5708, -1.5708, -3.1416,
            -1.48353, -1.48353, -1.48353, -1.48353
        ])
        self.joint_upper_limits = jnp.array([
            0.585, 1.5708, 1.5708, 1.5708, 3.1416,
            1.48353, 1.48353, 1.48353, 1.48353
        ])
        # 扭矩限位 (基于电机模组参数)
        # B25-PD-36-G (J2-J6): 额定扭矩 25 Nm, 保守值 15.0 Nm (安全系数 0.6)
        # B06-PA-36-G (J7-J9): 额定扭矩 6 Nm, 保守值 3.6 Nm (安全系数 0.6)
        self.joint_max_forces = jnp.array([
            50.0, 15.0, 15.0, 15.0, 15.0, 15.0, 3.6, 3.6, 3.6
        ])

        # 从 NumPy 版本提取螺旋轴、零位矩阵和连杆变换
        from work.nineaxis_kinematics import NineaxisKinematics
        kin_np = NineaxisKinematics()
        # Keep the JAX box constraints identical to the NumPy/simulation path.
        self.joint_max_velocities = jnp.asarray(kin_np.joint_limits.dq_max)

        self.S_axes = jnp.array(np.array(kin_np._S_list))  # (9, 6)
        self.M_zero = jnp.array(kin_np.M)  # (4, 4)

        # 各连杆的零位变换 (用于碰撞球位置计算)
        self._M_links = {}
        for parent, child, jtype, *_ in JOINT_CHAIN:
            if hasattr(kin_np, '_M_link') and child in kin_np._M_link:
                self._M_links[child] = jnp.array(kin_np._M_link[child])
            else:
                self._M_links[child] = jnp.eye(4)

        # Environment and point-cloud safety use a mesh-conservative outer
        # envelope. The legacy 17-sphere model remains dedicated to the
        # separately calibrated self-collision pair topology below.
        self.num_environment_collision_spheres = NUM_ENVIRONMENT_COLLISION_SPHERES
        self._environment_sphere_link_indices = jnp.asarray(
            ENVIRONMENT_SPHERE_LINK_INDICES)
        self._environment_sphere_local_centers = jnp.asarray(
            ENVIRONMENT_SPHERE_LOCAL_CENTERS_M)
        self._environment_sphere_radii = jnp.asarray(
            ENVIRONMENT_SPHERE_RADII_M)

        # JIT 编译纯函数
        self._ee_position_jit = jax.jit(ee_position_fn, static_argnums=())
        self._ee_rotation_jit = jax.jit(ee_rotation_fn, static_argnums=())
        self._ee_transform_jit = jax.jit(ee_transform_fn, static_argnums=())
        self._full_jacobian_jit = jax.jit(full_jacobian_fn, static_argnums=())
        self._link_collision_data_jit = jax.jit(self.link_collision_data)

    def ee_position(self, q: jnp.ndarray) -> jnp.ndarray:
        """末端位置 FK"""
        return self._ee_position_jit(q, self.S_axes, self.M_zero)

    def ee_rotation(self, q: jnp.ndarray) -> jnp.ndarray:
        """末端姿态 FK"""
        return self._ee_rotation_jit(q, self.S_axes, self.M_zero)

    def ee_transform(self, q: jnp.ndarray) -> jnp.ndarray:
        """末端齐次变换"""
        return self._ee_transform_jit(q, self.S_axes, self.M_zero)

    def ee_jacobian(self, q: jnp.ndarray) -> jnp.ndarray:
        """6-DOF 雅可比 [J_pos; J_rot]"""
        return self._full_jacobian_jit(q, self.S_axes, self.M_zero)

    def self_collision_data(self, q: jnp.ndarray) -> jnp.ndarray:
        """Return the legacy 17-sphere data used by self-collision CBF pairs.

        This topology remains frozen while its replacement is calibrated
        against mesh self-clearance. It must not be used for environment or
        point-cloud clearance; use ``environment_collision_data`` there.
        """
        # 计算所有连杆的 FK 变换
        transforms = self._compute_all_link_transforms(q)

        # 关节球体 (与 fcl_collision.py JOINT_RADII 一致)
        joint_radii = jnp.array([0.080, 0.075, 0.075, 0.070, 0.065, 0.060, 0.055, 0.040])
        # Link2~ee_link 的位置 (indices 2~9 in transforms)
        joint_positions = transforms[2:10, :3, 3]  # (8, 3)
        joint_spheres = jnp.hstack([joint_positions, joint_radii[:, None]])  # (8, 4)

        # 连杆中点球体 (覆盖连杆轴, 与 CAPSULE_DEFS 对应)
        # Link2-Link3 中点
        mid_23 = 0.5 * (transforms[2, :3, 3] + transforms[3, :3, 3])
        # Link3-Link4 中点
        mid_34 = 0.5 * (transforms[3, :3, 3] + transforms[4, :3, 3])
        # Link4-Link5 中点
        mid_45 = 0.5 * (transforms[4, :3, 3] + transforms[5, :3, 3])
        # Link5-Link7 中点
        mid_57 = 0.5 * (transforms[5, :3, 3] + transforms[6, :3, 3])
        # Link7-Link8 中点
        mid_78 = 0.5 * (transforms[6, :3, 3] + transforms[7, :3, 3])
        # Link8-Link9 中点
        mid_89 = 0.5 * (transforms[7, :3, 3] + transforms[8, :3, 3])
        # Link9-ee 中点
        mid_9e = 0.5 * (transforms[8, :3, 3] + transforms[9, :3, 3])

        mid_radii = jnp.array([0.065, 0.055, 0.065, 0.065, 0.040, 0.060, 0.025])
        mid_positions = jnp.stack([mid_23, mid_34, mid_45, mid_57, mid_78, mid_89, mid_9e])
        mid_spheres = jnp.hstack([mid_positions, mid_radii[:, None]])  # (7, 4)

        # base_link 和 Link1 用简化球体 (替代 Box)
        base_center = transforms[0, :3, 3] + transforms[0, :3, :3] @ jnp.array([0.0, 0.006, 0.113])
        link1_center = transforms[1, :3, 3] + transforms[1, :3, :3] @ jnp.array([0.0, 0.189, -0.061])
        base_sphere = jnp.array([[base_center[0], base_center[1], base_center[2], 0.135]])
        link1_sphere = jnp.array([[link1_center[0], link1_center[1], link1_center[2], 0.100]])

        return jnp.vstack([base_sphere, link1_sphere, joint_spheres, mid_spheres])  # (17, 4)

    def environment_collision_data(self, q: jnp.ndarray) -> jnp.ndarray:
        """Return the fixed 32-sphere mesh outer envelope for environment CBFs.

        Every local sphere is transformed by its owning link pose. The result
        is differentiable with respect to ``q`` and has a constant shape, so
        dynamic obstacle count and point-cloud density cannot change QP size.
        """
        transforms = self._compute_all_link_transforms(q)
        selected = transforms[self._environment_sphere_link_indices]
        centers = (
            jnp.einsum(
                'nij,nj->ni', selected[:, :3, :3],
                self._environment_sphere_local_centers)
            + selected[:, :3, 3])
        return jnp.concatenate([
            centers, self._environment_sphere_radii[:, None]], axis=1)

    def link_collision_data(self, q: jnp.ndarray) -> jnp.ndarray:
        """Backward-compatible alias for the legacy self-collision topology."""
        return self.self_collision_data(q)

    def _compute_all_link_transforms(self, q: jnp.ndarray) -> jnp.ndarray:
        """计算所有连杆的世界系变换 (JAX)"""
        transforms = []
        T = jnp.eye(4)
        joint_idx = 0
        for parent, child, jtype, *_ in JOINT_CHAIN:
            if jtype in ("revolute", "prismatic"):
                T = T @ _twist_exp_jax(self.S_axes[joint_idx], q[joint_idx])
                joint_idx += 1
            transforms.append(T @ self._M_links.get(child, jnp.eye(4)))
        return jnp.stack(transforms)  # (11, 4, 4)

    def mass_matrix(self, q: jnp.ndarray) -> jnp.ndarray:
        """简化对角惯量矩阵"""
        M_diag = jnp.array([1.99, 2.35, 0.64, 1.16, 0.77, 0.66, 0.36, 0.40, 0.15])
        return jnp.diag(M_diag)

    def mass_matrix_inv(self, q: jnp.ndarray) -> jnp.ndarray:
        """惯量矩阵逆"""
        M_diag = jnp.array([1.99, 2.35, 0.64, 1.16, 0.77, 0.66, 0.36, 0.40, 0.15])
        return jnp.diag(1.0 / M_diag)
