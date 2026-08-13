#!/usr/bin/env python3
"""
oscbf_torque_config.py
======================
9-DOF 冗余臂力矩级 OSCBF 配置，使用 cbfpy 框架 + frax 动力学。

参考 oscbf/core/oscbf_configs.py:21 - OSCBFTorqueConfig

状态: z = [q, qdot] (length = 18)
控制: u = [joint torques] (length = 9)
"""

import jax
import jax.numpy as jnp
import numpy as np
from cbfpy import CBFConfig, CBF
from work.frax_manipulator import FraxManipulator
from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
from work.oscbf_collision_config import (
    SELF_COLLISION_PAIRS, compute_self_collision_h, compute_obstacle_h)


class NineaxisOSCBFTorqueConfig(CBFConfig):
    """9-DOF 冗余臂力矩级 OSCBF 配置

    State: z = [q, qdot] (length = 18)
    Control: u = [joint torques] (length = 9)

    使用 frax 库提供完整的动力学支持:
    - 惯量矩阵 M(q)
    - 重力向量 g(q)
    - 科里奥利向量 c(q, qdot)
    - 碰撞检测数据
    """

    def __init__(self, robot: FraxManipulator,
                 obstacle_positions=None, obstacle_radii=None):
        self.robot = robot

        # 碰撞几何 (复用速度模式的 17 球模型; 动力学仍用 FraxManipulator)
        self._collision_robot = NineaxisManipulatorJAX()

        # 权重
        self.pos_obj_weight = 50.0
        self.rot_obj_weight = 20.0
        self.joint_obj_weight = 1.0

        # 约束参数
        self.alpha_joint_limit = 8.0
        self.alpha_velocity_limit = 5.0
        self.alpha_collision = 5.0
        self.alpha_singularity = 5.0
        self.d_safe_collision = 0.03
        self.singularity_tol = 0.005

        # 障碍物 (可选)
        self.obstacle_positions = obstacle_positions if obstacle_positions is not None else jnp.array([])
        self.obstacle_radii = obstacle_radii if obstacle_radii is not None else jnp.array([])

        super().__init__(
            n=robot.num_joints * 2,  # [q, qdot]
            m=robot.num_joints,      # tau
            u_min=-np.array(robot.joint_max_forces),
            u_max=np.array(robot.joint_max_forces),
        )

    def f(self, z):
        """动力学: f(z) = [qdot, -M^{-1}(g + c)]

        使用 frax 提供完整的动力学计算
        """
        q = z[:9]
        qdot = z[9:]

        # frax 动力学
        M_inv = self.robot.mass_matrix_inverse(q)
        h = self.robot.nonlinear_bias(q, qdot)  # h = g + c

        # f(z) = [qdot, -M^{-1} @ h]
        return jnp.concatenate([qdot, -M_inv @ h])

    def g(self, z):
        """控制输入矩阵: g(z) = [0; M^{-1}]"""
        q = z[:9]
        M_inv = self.robot.mass_matrix_inverse(q)

        # g(z) = [[0, ..., 0], [M^{-1}]]
        return jnp.vstack([
            jnp.zeros((9, 9)),
            M_inv
        ])

    def _P(self, z, *args, **kwargs):
        """OSCBF 任务一致性 P 矩阵 (力矩级)

        P = N^T @ W_joint^2 @ N + J^T @ W_task^2 @ J
        """
        q = z[:9]
        J = self.robot.ee_jacobian(q)

        # 阻尼伪逆
        lam = 1e-3
        JJT = J @ J.T + lam**2 * jnp.eye(6)
        J_hash = J.T @ jnp.linalg.inv(JJT)

        # 零空间投影
        N = jnp.eye(9) - J_hash @ J

        # 权重矩阵
        W_task_sq = jnp.diag(jnp.array([
            self.pos_obj_weight**2, self.pos_obj_weight**2, self.pos_obj_weight**2,
            self.rot_obj_weight**2, self.rot_obj_weight**2, self.rot_obj_weight**2
        ]))
        W_joint_sq = self.joint_obj_weight**2 * jnp.eye(9)

        # P 矩阵
        P_u = N.T @ W_joint_sq @ N + J.T @ W_task_sq @ J

        return 0.5 * (P_u + P_u.T)

    def P(self, z, u_des, *args, **kwargs):
        """QP 二次项"""
        return self._P(z)

    def q(self, z, u_des, *args, **kwargs):
        """QP 线性项"""
        return -u_des.T @ self._P(z)

    def h_2(self, z, *args, **kwargs):
        """所有 CBF 约束 (相对度 2)

        约束来源:
        1. 关节位置限位 (18 个)
        2. 关节速度限位 (18 个)
        3. 自碰撞避免
        4. 障碍物碰撞避免
        5. 奇异性避免 (1 个)
        """
        q = z[:9]
        qdot = z[9:]

        # 1. 关节位置限位
        h_pos_upper = self.robot.joint_upper_limits - q - 0.01
        h_pos_lower = q - self.robot.joint_lower_limits - 0.01
        h_pos = jnp.concatenate([h_pos_upper, h_pos_lower])

        # 2. 关节速度限位
        h_vel_upper = self.robot.joint_max_velocities - qdot
        h_vel_lower = qdot + self.robot.joint_max_velocities
        h_vel = jnp.concatenate([h_vel_upper, h_vel_lower])

        # 3. 自碰撞避免 (全身 17 球 × 14 对, 与速度模式一致)
        collision_data = self._collision_robot.link_collision_data(q)  # (17, 4)
        robot_positions = collision_data[:, :3]
        robot_radii = collision_data[:, 3]
        h_self_collision = compute_self_collision_h(
            robot_positions, robot_radii, SELF_COLLISION_PAIRS, self.d_safe_collision)

        # 4. 障碍物碰撞避免 (全身 17 球 vs 障碍物)
        h_obstacles = compute_obstacle_h(
            robot_positions, robot_radii,
            self.obstacle_positions, self.obstacle_radii,
            self.d_safe_collision)

        # 5. 奇异性
        J_pos = self.robot.ee_jacobian(q)[:3, :]
        sigma = jnp.prod(jnp.linalg.svd(J_pos, compute_uv=False))
        h_singularity = jnp.array([sigma - self.singularity_tol])

        return jnp.concatenate([h_pos, h_vel, h_self_collision, h_obstacles, h_singularity])

    def alpha(self, h):
        """CBF 增益函数"""
        return 10.0 * h


def create_torque_cbf(obstacle_positions=None, obstacle_radii=None):
    """创建力矩级 CBF 实例"""
    robot = FraxManipulator()
    config = NineaxisOSCBFTorqueConfig(
        robot,
        obstacle_positions=obstacle_positions,
        obstacle_radii=obstacle_radii
    )
    return CBF.from_config(config)
