#!/usr/bin/env python3
"""
nineaxis_kinematics.py
======================
九轴机械臂运动学: POE 正运动学、螺旋轴雅可比、牛顿-拉夫森逆运动学。

依赖: numpy, scipy
"""

import numpy as np

from work.actuator_limits import load_actuator_limit_profile
from scipy.spatial.transform import Rotation as R
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass


@dataclass
class JointLimits:
    """
    关节限位。

    q_min/q_max: 位置限位 (J1: m, J2-J9: rad) — 与 C++/YAML 对齐。
    dq_max/ddq_max: 电机规格限位 (rad/s, rad/s²), 反映实际电机能力。
        Python 仿真使用这些宽松限位。C++ 使用更保守的仿真限位:
          dq_sim = [±0.30, ±1.5, ±1.5, ±1.5, ±1.5, ±1.5, ±1.5, ±1.5, ±1.5]
          ddq_sim = [±1.0, ±4.0, ±4.0, ±4.0, ±4.0, ±4.0, ±4.0, ±4.0, ±4.0]
    """
    q_min: np.ndarray   # 9, rad (J1: m)
    q_max: np.ndarray
    dq_max: np.ndarray  # 速度限位 (电机规格)
    ddq_max: np.ndarray # 加速度限位 (电机规格)


# Shared kinematics data — single source of truth in kinematics_data.py.
# Re-exported here for backward compatibility with existing imports.
from work.kinematics_data import JOINT_CHAIN, N_JOINTS, LINK_NAMES

# 活动关节在 JOINT_CHAIN 中的索引
_ACTIVE_IDX = [i for i, (_, _, jt, _, _, _, _, _, _, _) in enumerate(JOINT_CHAIN)
               if jt in ("revolute", "prismatic")]


def _skew(v):
    """3×3 反对称矩阵"""
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def _adjoint(T):
    """6×6 伴随变换矩阵 Ad_T"""
    Rmat = T[:3, :3]
    p = T[:3, 3]
    Ad = np.zeros((6, 6))
    Ad[:3, :3] = Rmat
    Ad[3:, :3] = _skew(p) @ Rmat
    Ad[3:, 3:] = Rmat
    return Ad


def _twist_exp(screw, theta):
    """螺旋运动指数映射 → 4×4 齐次矩阵 (Rodrigues 公式, 优化版)"""
    omega = screw[:3]
    v = screw[3:]

    # 计算 omega 范数的平方，避免 sqrt
    omega_norm_sq = omega[0]*omega[0] + omega[1]*omega[1] + omega[2]*omega[2]

    if omega_norm_sq < 1e-24:
        # 纯平动 (棱柱关节)
        T = np.eye(4)
        T[0, 3] = v[0] * theta
        T[1, 3] = v[1] * theta
        T[2, 3] = v[2] * theta
        return T

    omega_norm = np.sqrt(omega_norm_sq)
    inv_norm = 1.0 / omega_norm
    wx = omega[0] * inv_norm
    wy = omega[1] * inv_norm
    wz = omega[2] * inv_norm

    # 旋转部分: Rodrigues (直接计算，避免矩阵运算)
    angle = omega_norm * theta
    ct, st = np.cos(angle), np.sin(angle)
    omct = 1.0 - ct

    # R = I + st * K + (1-ct) * K^2, K = skew(omega_hat)
    R00 = 1.0 - omct*(wy*wy + wz*wz)
    R01 = omct*wx*wy - st*wz
    R02 = omct*wx*wz + st*wy
    R10 = omct*wx*wy + st*wz
    R11 = 1.0 - omct*(wx*wx + wz*wz)
    R12 = omct*wy*wz - st*wx
    R20 = omct*wx*wz - st*wy
    R21 = omct*wy*wz + st*wx
    R22 = 1.0 - omct*(wx*wx + wy*wy)

    # 平动部分: G = theta*I + (1-ct)/norm * K + (theta - st/norm) * K^2
    # K = skew(omega_hat), K^2 = omega_hat*omega_hat^T - I
    # p = G @ v
    inv_norm2 = 1.0 / omega_norm
    a = theta
    b = omct * inv_norm2
    c = (theta - st * inv_norm2)

    # G @ v = a*v + b*(K @ v) + c*(K^2 @ v)
    # K @ v = cross(omega_hat, v)
    kv_x = wy*v[2] - wz*v[1]
    kv_y = wz*v[0] - wx*v[2]
    kv_z = wx*v[1] - wy*v[0]

    # K^2 @ v = cross(omega_hat, K @ v)
    k2v_x = wy*kv_z - wz*kv_y
    k2v_y = wz*kv_x - wx*kv_z
    k2v_z = wx*kv_y - wy*kv_x

    px = a*v[0] + b*kv_x + c*k2v_x
    py = a*v[1] + b*kv_y + c*k2v_y
    pz = a*v[2] + b*kv_z + c*k2v_z

    T = np.eye(4)
    T[0, 0] = R00; T[0, 1] = R01; T[0, 2] = R02; T[0, 3] = px
    T[1, 0] = R10; T[1, 1] = R11; T[1, 2] = R12; T[1, 3] = py
    T[2, 0] = R20; T[2, 1] = R21; T[2, 2] = R22; T[2, 3] = pz
    return T


def so3_log(R_mat: np.ndarray) -> np.ndarray:
    """SO(3) 对数映射: 3×3 旋转矩阵 → 3D 旋转向量"""
    theta = np.arccos(np.clip((np.trace(R_mat) - 1.0) / 2.0, -1.0, 1.0))
    if abs(theta) < 1e-10:
        return np.zeros(3)
    omega_hat = (R_mat - R_mat.T) / (2.0 * np.sin(theta))
    return np.array([omega_hat[2, 1], omega_hat[0, 2], omega_hat[1, 0]]) * theta


def _axis_from_pi_rotation(R_mat: np.ndarray) -> np.ndarray:
    """theta≈π 时从旋转矩阵提取旋转轴 (参照 se3_log.m: axis_from_pi_rotation)"""
    A = (R_mat + np.eye(3)) / 2.0
    idx = int(np.argmax(np.diag(A)))
    axis = np.zeros(3)
    axis[idx] = np.sqrt(max(A[idx, idx], 0.0))
    if axis[idx] < 1e-8:
        return np.array([1.0, 0.0, 0.0])
    for j in range(3):
        if j != idx:
            axis[j] = A[j, idx] / axis[idx]
    return axis / np.linalg.norm(axis)


def se3_log(T: np.ndarray) -> np.ndarray:
    """SE(3) 对数映射: 4×4 齐次变换 → 空间旋量 [omega; v] (参照 se3_log.m)

    用于空间系 NR IK 的 SE(3) 误差 xi = se3_log(T_target @ T_cur^{-1})。
    处理 theta≈0 (纯平移) 与 theta≈π (退化) 两种情形。
    """
    R_mat = T[:3, :3]
    p = T[:3, 3]
    theta = np.arccos(np.clip((np.trace(R_mat) - 1.0) / 2.0, -1.0, 1.0))
    if theta < 1e-12:
        w = np.zeros(3)
        v = p
    elif abs(np.pi - theta) < 1e-6:
        axis = _axis_from_pi_rotation(R_mat)
        w = theta * axis
        w_skew = _skew(w)
        G_inv = np.eye(3) - 0.5 * w_skew + (1.0 / (theta * theta)) * w_skew @ w_skew
        v = G_inv @ p
    else:
        w_skew_unscaled = (theta / (2.0 * np.sin(theta))) * (R_mat - R_mat.T)
        w = np.array([w_skew_unscaled[2, 1], w_skew_unscaled[0, 2], w_skew_unscaled[1, 0]])
        w_skew = _skew(w)
        G_inv = (np.eye(3) - 0.5 * w_skew
                 + (1.0 / (theta * theta) - (1.0 + np.cos(theta)) / (2.0 * theta * np.sin(theta)))
                 * w_skew @ w_skew)
        v = G_inv @ p
    return np.concatenate([w, v])


class NineaxisKinematics:
    """九轴机械臂运动学 (POE 指数积方法)"""

    def __init__(self):
        # 速度/加速度统一从执行器资料导出。J1 的值仅可用于仿真或影子模式。
        self.actuator_limit_profile = load_actuator_limit_profile()
        self.joint_limits = JointLimits(
            q_min=np.array([0.0, -1.5708, -1.5708, -1.5708, -3.1416,
                           -1.48353, -1.48353, -1.48353, -1.48353]),
            q_max=np.array([0.585, 1.5708, 1.5708, 1.5708, 3.1416,
                           1.48353, 1.48353, 1.48353, 1.48353]),
            dq_max=self.actuator_limit_profile.velocity_limits.copy(),
            ddq_max=self.actuator_limit_profile.acceleration_limits.copy(),
        )

        # 计算零位 FK (用 URDF 链方法，仅初始化一次)
        T_zero = self._compute_fk_zero()

        # 提取螺旋轴 S_i (世界系，q=0 位形)
        self._S_list = self._extract_screw_axes(T_zero)  # list of 9 arrays (6,)

        # M = 零位时末端执行器(ee_link)在世界系下的位姿
        self.M = T_zero["ee_link"].copy()

        # 各连杆在零位时的世界位姿 (用于 FK 加速)
        self._M_link = {name: T_zero[name].copy() for name in LINK_NAMES}

        # 每个连杆受几个活动关节影响 (用于 POE FK)
        self._link_active_count = {}
        cnt = 0
        for parent, child, jtype, *_ in JOINT_CHAIN:
            if jtype in ("revolute", "prismatic"):
                cnt += 1
            self._link_active_count[child] = cnt

    # ================================================================
    # 引导: URDF 链 FK (仅初始化用)
    # ================================================================
    @staticmethod
    def _joint_transform(x, y, z, roll, pitch, yaw, jtype, axis, q):
        """单关节齐次变换 T_parent_child (仅用于提取螺旋轴)"""
        T = np.eye(4)
        T[:3, 3] = [x, y, z]
        T[:3, :3] = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

        ax = np.array(axis, dtype=float)
        ax = ax / max(np.linalg.norm(ax), 1e-12)

        if jtype == "revolute":
            c, s = np.cos(q), np.sin(q)
            vv = 1.0 - c
            ux, uy, uz = ax
            R_j = np.array([
                [c + ux*ux*vv,  ux*uy*vv - uz*s, ux*uz*vv + uy*s],
                [uy*ux*vv + uz*s, c + uy*uy*vv,  uy*uz*vv - ux*s],
                [uz*ux*vv - uy*s, uz*uy*vv + ux*s, c + uz*uz*vv],
            ])
            T[:3, :3] = T[:3, :3] @ R_j
        elif jtype == "prismatic":
            T[:3, 3] += T[:3, :3] @ (ax * q)
        return T

    def _compute_fk_zero(self):
        """计算 q=0 时所有连杆的世界位姿"""
        T = {"world": np.eye(4)}
        for parent, child, jtype, x, y, z, roll, pitch, yaw, axis in JOINT_CHAIN:
            T[child] = T[parent] @ self._joint_transform(
                x, y, z, roll, pitch, yaw, jtype, axis, 0.0)
        return T

    def _extract_screw_axes(self, T_zero):
        """
        从零位 FK 提取世界系螺旋轴 S_i (空间坐标系)。

        对旋转关节: S = [ω; p × ω]  (p 是关节轴上一点)
        对棱柱关节: S = [0; v]       (v 是平动方向)
        """
        screws = []
        T_accum = np.eye(4)

        for parent, child, jtype, x, y, z, roll, pitch, yaw, axis in JOINT_CHAIN:
            # T_joint_at_q0 = 关节刚体变换 (不含关节运动)
            T_joint = self._joint_transform(
                x, y, z, roll, pitch, yaw, jtype, axis, 0.0)
            # T_world_joint = 子连杆零位在世界系下的位姿 (关节轴通过该原点)
            T_world_joint = T_accum @ T_joint

            if jtype in ("revolute", "prismatic"):
                ax = np.array(axis, dtype=float)
                ax = ax / max(np.linalg.norm(ax), 1e-12)
                # 轴在世界系下的方向 (经 parent 和 origin 旋转)
                axis_world = T_world_joint[:3, :3] @ ax
                axis_world = axis_world / np.linalg.norm(axis_world)

                if jtype == "revolute":
                    p_w = T_world_joint[:3, 3]
                    v_w = np.cross(p_w, axis_world)
                    screws.append(np.concatenate([axis_world, v_w]))
                else:  # prismatic
                    screws.append(np.concatenate([np.zeros(3), axis_world]))

            T_accum = T_world_joint  # 累积到下一个关节

        return screws  # list of 9 arrays, each shape (6,)

    # ================================================================
    # POE 正运动学
    # ================================================================
    def forward_kinematics(self, q: np.ndarray) -> Dict[str, np.ndarray]:
        """
        POE 正运动学: T_world_link = exp(S1*q1)*...*exp(Sk*qk) @ M_link

        返回: {"world": T, "base_link": T, "Link1": T, ..., "ee_link": T}
        """
        T = {"world": np.eye(4)}
        T_acc = np.eye(4)
        joint_idx = 0

        for parent, child, jtype, *_ in JOINT_CHAIN:
            if jtype in ("revolute", "prismatic"):
                T_acc = T_acc @ _twist_exp(self._S_list[joint_idx], q[joint_idx])
                joint_idx += 1
            T[child] = T_acc @ self._M_link[child]

        return T

    def link_poses(self, q: np.ndarray) -> Dict[str, np.ndarray]:
        return self.forward_kinematics(q)

    # ================================================================
    # 末端执行器
    # ================================================================
    def ee_pose(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """末端执行器位姿 → (pos 3D, rot 3×3)"""
        # 直接用 POE: T_ee = exp(S1q1)*...*exp(S9q9) * M
        T_ee = self._compute_ee_pose(q)
        return T_ee[:3, 3].copy(), T_ee[:3, :3].copy()

    def _compute_ee_pose(self, q: np.ndarray) -> np.ndarray:
        """POE: T_ee = exp(S1q1)*...*exp(S9q9) @ M"""
        T = np.eye(4)
        for i in range(N_JOINTS):
            T = T @ _twist_exp(self._S_list[i], q[i])
        return T @ self.M

    def ee_position(self, q: np.ndarray) -> np.ndarray:
        return self.ee_pose(q)[0]

    def ee_rotation(self, q: np.ndarray) -> np.ndarray:
        return self.ee_pose(q)[1]

    # ================================================================
    # POE 空间雅可比
    # ================================================================
    def compute_spatial_jacobian_world(self, q: np.ndarray) -> np.ndarray:
        """
        POE 空间雅可比 J_s ∈ R^{6×9}:
        J_s[:,i] = Ad_{exp(S1q1)...exp(S_{i-1}q_{i-1})} @ S_i
        """
        J_s = np.zeros((6, N_JOINTS))
        T_acc = np.eye(4)

        for i in range(N_JOINTS):
            J_s[:, i] = _adjoint(T_acc) @ self._S_list[i]
            T_acc = T_acc @ _twist_exp(self._S_list[i], q[i])

        return J_s

    def forward_kinematics_and_jacobian(self, q: np.ndarray):
        """
        融合正运动学 + 空间雅可比 — 单遍遍历 JOINT_CHAIN, 避免重复计算 _twist_exp。

        返回: (T_all, J_s) 与 forward_kinematics + compute_spatial_jacobian_world 相同
        """
        T_all = {"world": np.eye(4)}
        J_s = np.zeros((6, N_JOINTS))
        T_acc = np.eye(4)
        joint_idx = 0

        for parent, child, jtype, *_ in JOINT_CHAIN:
            if jtype in ("revolute", "prismatic"):
                S_i = self._S_list[joint_idx]
                J_s[:, joint_idx] = _adjoint(T_acc) @ S_i
                T_acc = T_acc @ _twist_exp(S_i, q[joint_idx])
                joint_idx += 1
            T_all[child] = T_acc @ self._M_link[child]

        return T_all, J_s

    def compute_position_jacobian(self, q: np.ndarray) -> np.ndarray:
        """
        位置雅可比 J_pos ∈ R^{3×9}。

        ṗ = J_pos @ qdot
        ṗ = v_s + ω_s × p  (空间速度 → 末端线速度)
        """
        J_s = self.compute_spatial_jacobian_world(q)
        p_ee = self.ee_position(q)
        # ṗ_i = v_s_i + ω_i × p_ee = v_s_i - skew(p_ee) @ ω_i
        J_pos = J_s[3:, :] - _skew(p_ee) @ J_s[:3, :]
        return J_pos

    def point_jacobian(self, q: np.ndarray, link_idx: int,
                       point_world: np.ndarray) -> np.ndarray:
        """连杆 link_idx 上某点 point_world 的线速度雅可比 (3×N_JOINTS)。

        前 link_idx 个活动关节影响该点 (上游), 下游关节列置零。
        用于自碰撞 CBF 的解析梯度 (避免有限差分的性能开销)。
        """
        J_s = self.compute_spatial_jacobian_world(q)
        n_act = min(int(link_idx), N_JOINTS)
        J_pos_full = J_s[3:, :] - _skew(point_world) @ J_s[:3, :]
        J_pos = np.zeros((3, N_JOINTS))
        J_pos[:, :n_act] = J_pos_full[:, :n_act]
        return J_pos

    def compute_full_jacobian(self, q: np.ndarray) -> np.ndarray:
        """6D task Jacobian ordered as [linear velocity; angular velocity]."""
        J_pos = self.compute_position_jacobian(q)
        J_rot = self.compute_spatial_jacobian_world(q)[:3, :]
        return np.vstack([J_pos, J_rot])

    # ================================================================
    # 奇异值
    # ================================================================
    def singularity_index(self, q: np.ndarray) -> float:
        J_pos = self.compute_position_jacobian(q)
        return self._singularity_index_from_jac(J_pos)

    @staticmethod
    def _singularity_index_from_jac(J_pos: np.ndarray) -> float:
        """Singularity index (product of singular values) from a pre-computed Jacobian."""
        try:
            _, s, _ = np.linalg.svd(J_pos)
            return float(np.prod(s))
        except np.linalg.LinAlgError:
            return 1e-10

    # ================================================================
    # 惯量矩阵 (简化对角)
    # ================================================================
    def mass_matrix(self, q: np.ndarray) -> np.ndarray:
        M_diag = np.array([1.99, 2.35, 0.64, 1.16, 0.77, 0.66, 0.36, 0.40, 0.15])
        return np.diag(M_diag)

    def mass_matrix_inv(self, q: np.ndarray) -> np.ndarray:
        return np.diag(1.0 / np.diag(self.mass_matrix(q)))

    # ================================================================
    # 牛顿-拉夫森逆运动学
    # ================================================================
    def ik(self, target_pos: np.ndarray, target_R: Optional[np.ndarray] = None,
           q_init: Optional[np.ndarray] = None, max_iter: int = 200,
           tol_pos: float = 1e-4, tol_rot: float = 1e-3,
           damping: float = 0.001) -> Optional[np.ndarray]:
        """
        牛顿-拉夫森 IK (阻尼最小二乘)。

        q_{k+1} = q_k + J^T (J J^T + λ²I)^{-1} e
        """
        if q_init is None:
            q_init = np.zeros(N_JOINTS)
        q = q_init.copy()
        q = np.clip(q, self.joint_limits.q_min, self.joint_limits.q_max)

        best_q = q.copy()
        best_err = float('inf')

        for _ in range(max_iter):
            T_ee = self._compute_ee_pose(q)
            pos = T_ee[:3, 3]
            rot = T_ee[:3, :3]

            dp = target_pos - pos
            if target_R is not None:
                # SE(3) 空间系误差 [omega; v] (参照 ik_nr_spatial.m: se3_log(T_target/T_cur))
                # 配空间雅可比 J_s=[omega; v], 顺序与坐标系一致。
                T_target = np.eye(4)
                T_target[:3, :3] = target_R
                T_target[:3, 3] = target_pos
                error = se3_log(T_target @ np.linalg.inv(T_ee))
                J = self.compute_spatial_jacobian_world(q)
            else:
                # 位置 IK: 末端位置雅可比 J_pos = J_s[3:] - skew(p_ee)·J_s[:3]
                # 配末端位置误差 dp。注意 J_s[3:] 是空间线速度, 不等于末端线速度,
                # 必须用 compute_position_jacobian (旧代码误用 J_s[3:] 导致不收敛)。
                error = dp
                J = self.compute_position_jacobian(q)

            err_norm = np.linalg.norm(error)
            if err_norm < best_err:
                best_err = err_norm
                best_q = q.copy()
            if err_norm < tol_pos:
                break

            # 阻尼伪逆: J^T (J J^T + λ²I)^{-1}
            m = J.shape[0]
            JJT = J @ J.T + damping * np.eye(m)
            dq = J.T @ np.linalg.solve(JJT, error)

            # 线搜索
            alpha = 1.0
            for _ in range(10):
                q_new = q + alpha * dq
                q_new = np.clip(q_new, self.joint_limits.q_min, self.joint_limits.q_max)
                T_new = self._compute_ee_pose(q_new)
                err_new = np.linalg.norm(target_pos - T_new[:3, 3])
                if err_new < err_norm:
                    break
                alpha *= 0.5

            q = q + alpha * dq
            q = np.clip(q, self.joint_limits.q_min, self.joint_limits.q_max)

        return best_q

    def ik_multiple(self, target_positions: List[np.ndarray],
                    q_init: Optional[np.ndarray] = None) -> List[np.ndarray]:
        if q_init is None:
            q_init = np.zeros(N_JOINTS)
        solutions = []
        q_prev = q_init
        for pos in target_positions:
            q_sol = self.ik(pos, q_init=q_prev)
            if q_sol is not None:
                solutions.append(q_sol)
                q_prev = q_sol
            else:
                solutions.append(q_prev.copy())
        return solutions


# ================================================================
# 测试
# ================================================================
if __name__ == "__main__":
    kin = NineaxisKinematics()

    q_zero = np.zeros(9)
    pos, rot = kin.ee_pose(q_zero)
    print(f"POE 零位 EE 位置: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}] m")
    print(f"POE 零位 EE 姿态 (RPY): {R.from_matrix(rot).as_euler('xyz')}")

    # FK 全连杆
    T_all = kin.forward_kinematics(q_zero)
    for name in LINK_NAMES:
        p = T_all[name][:3, 3]
        print(f"  {name:10s}: pos=[{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}]")

    # J1 方向验证
    q1 = np.zeros(9); q1[0] = 0.1
    pos1, _ = kin.ee_pose(q1)
    delta = pos1 - pos
    print(f"\nJ1=0.1 → delta=[{delta[0]:.4f} {delta[1]:.4f} {delta[2]:.4f}] (应沿世界 Z)")

    # 雅可比验证
    J_pos = kin.compute_position_jacobian(q_zero)
    print(f"\nJ_pos[:,0] (J1): {np.round(J_pos[:,0], 3)}")
    U, s, Vt = np.linalg.svd(J_pos, full_matrices=False)
    print(f"奇异值: {np.round(s, 4)}  条件数: {s[0]/s[-1]:.1f}")
    print(f"Manipulability: {kin.singularity_index(q_zero):.6f}")

    # IK 测试
    print(f"\nNewton-Raphson IK → 目标=零位EE")
    q_ik = kin.ik(pos)
    if q_ik is not None:
        pos_ik, _ = kin.ee_pose(q_ik)
        err = np.linalg.norm(pos_ik - pos)
        print(f"  IK 误差: {err:.2e} m  q={np.round(q_ik, 3)}")
