#!/usr/bin/env python3
"""
oscbf_velocity_config.py
========================
9-DOF 冗余臂速度级 OSCBF 配置，使用 cbfpy 框架。

参考 oscbf/core/oscbf_configs.py:134 - OSCBFVelocityConfig

障碍物通过 h_2 的 *h_args 传入 (固定 shape + enabled mask),
不烘焙到闭包, 避免障碍物更新触发 JAX 重新编译。
"""

import jax
import jax.numpy as jnp
import numpy as np
from cbfpy import CBFConfig, CBF
from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
from work.oscbf_collision_config import (
    SELF_COLLISION_PAIRS, compute_self_collision_h, compute_obstacle_h)
from work.safety_snapshot import sample_distance_field_jax
from work.joint_limit_contract import JOINT_LIMIT_CBF_MARGIN


class NineaxisOSCBFVelocityConfig(CBFConfig):
    """9-DOF 冗余臂速度级 OSCBF 配置

    State: z = [q] (length = 9)
    Control: u = [joint velocities] (length = 9)

    障碍物通过 h_2(z, obs_pos, obs_radii, obs_enabled, obs_d_safe) 传入:
    - obs_pos: (MAX, 3) 障碍物中心
    - obs_radii: (MAX,) 障碍物半径
    - obs_enabled: (MAX,) 1=启用, 0=禁用 (未用槽位 h 设为极大值, 不激活)
    - obs_d_safe: (MAX,) 每个障碍物的安全距离。动态障碍物可大于静态点云。

    参考论文: Morton & Pavone, "Safe, Task-Consistent Manipulation with OSCBF" (IROS 2025)
    """

    def __init__(self, robot: NineaxisManipulatorJAX,
                 temporal_lambda: float = 0.0,
                 temporal_wu=None,
                 sdf_shape=None,
                 aggregate_dynamic_obstacles: bool = False,
                 smooth_min_temperature: float = 0.01,
                 solver_tol: float = 1e-3,
                 joint_limit_lower=None,
                 joint_limit_upper=None,
                 joint_limit_cbf_margin=None):
        self.robot = robot

        # A verified task profile may narrow the mechanical range to an
        # independently checked working envelope. It reuses the existing 18
        # joint-limit CBF rows, so the fixed QP topology is unchanged.
        hard_lower = np.asarray(robot.joint_lower_limits, dtype=float)
        hard_upper = np.asarray(robot.joint_upper_limits, dtype=float)
        if (joint_limit_lower is None) != (joint_limit_upper is None):
            raise ValueError(
                'joint_limit_lower and joint_limit_upper must be supplied together')
        lower = (hard_lower if joint_limit_lower is None
                 else np.asarray(joint_limit_lower, dtype=float).reshape(robot.num_joints))
        upper = (hard_upper if joint_limit_upper is None
                 else np.asarray(joint_limit_upper, dtype=float).reshape(robot.num_joints))
        margin = (JOINT_LIMIT_CBF_MARGIN if joint_limit_cbf_margin is None
                  else float(joint_limit_cbf_margin))
        if (not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper))
                or not np.isfinite(margin) or margin < 0.0):
            raise ValueError('joint-limit envelope and CBF margin must be finite')
        if np.any(lower < hard_lower - 1.0e-12) or np.any(upper > hard_upper + 1.0e-12):
            raise ValueError('joint-limit envelope must stay inside mechanical limits')
        if np.any(lower + margin >= upper - margin):
            raise ValueError('joint-limit envelope has no positive CBF interior')
        self.joint_limit_lower = jnp.asarray(lower)
        self.joint_limit_upper = jnp.asarray(upper)
        self.joint_limit_cbf_margin = margin

        # 权重 (与当前实现一致)
        self.pos_obj_weight = 20.0
        self.rot_obj_weight = 10.0
        self.joint_obj_weight = 0.1

        # 约束参数
        self.alpha_joint_limit = 8.0
        self.alpha_collision = 5.0
        self.alpha_singularity = 5.0
        self.d_safe_collision = 0.03
        self.activation_collision = 0.10
        self.singularity_tol = 0.005
        self.singularity_activation = 0.05

        # 自碰撞对 (共享模块, 力矩/速度同一真相源)
        self.self_collision_pairs = SELF_COLLISION_PAIRS

        # QP 中障碍物行的固定位置。JAX 闭环使用它定点修正动态项，
        # 不依赖运行时约束数量，也不会触发重新编译。
        # Dynamic obstacles and ESDF use the mesh-conservative outer envelope.
        # Self-collision retains its separately calibrated 17-sphere topology.
        self.num_robot_collision_spheres = int(robot.num_environment_collision_spheres)
        # M7: obstacle rows use one DCOL OBB per link (10 links) instead of the
        # 32-sphere environment envelope; ESDF keeps the 32-sphere rows.
        self.num_obb_links = 10
        self.obstacle_h_start = 2 * robot.num_joints + int(self.self_collision_pairs.shape[0])
        self.aggregate_dynamic_obstacles = bool(aggregate_dynamic_obstacles)
        self.smooth_min_temperature = float(smooth_min_temperature)
        if self.smooth_min_temperature <= 0.0:
            raise ValueError("smooth_min_temperature must be positive")
        self.num_obstacle_constraints = (
            self.num_obb_links
            if self.aggregate_dynamic_obstacles
            else self.num_obb_links * 8)
        self.obstacle_h_stop = self.obstacle_h_start + self.num_obstacle_constraints
        self.obstacle_h_baseline_alpha = 10.0
        self.sdf_shape = None if sdf_shape is None else tuple(int(v) for v in sdf_shape)
        self.enable_sdf = self.sdf_shape is not None
        if self.enable_sdf and (len(self.sdf_shape) != 3 or min(self.sdf_shape) < 2):
            raise ValueError("sdf_shape must contain three dimensions >= 2")
        self.esdf_h_start = self.obstacle_h_stop
        self.num_esdf_constraints = self.num_robot_collision_spheres if self.enable_sdf else 0
        self.esdf_h_stop = self.esdf_h_start + self.num_esdf_constraints

        # 与 OSCBFQPSolver 一致的时序近端项：
        # lambda / 2 * ||W_u (u - u_safe_prev)||^2。
        self.temporal_lambda = float(temporal_lambda)
        if temporal_wu is None:
            self.temporal_wu = jnp.ones(robot.num_joints)
        else:
            self.temporal_wu = jnp.asarray(temporal_wu).reshape(robot.num_joints)

        # 障碍物不再烘焙到 config; 通过 h_2 参数传入
        # init_args 提供空障碍物种子, 用于 CBFConfig 的初始化验证
        _n_obs = 8  # MAX_JAX_OBSTACLES, 与 jax_control_facade.py 一致
        init_args = (
            jnp.zeros((_n_obs, 3)),   # obs_pos
            jnp.zeros(_n_obs),        # obs_radii
            jnp.zeros(_n_obs),        # obs_enabled
            jnp.zeros(_n_obs),        # obs_d_safe
            jnp.zeros((_n_obs, 3)),   # obs_vel (由 JAX 控制环使用)
            jnp.zeros(_n_obs),        # obs_radius_dot (由 JAX 控制环使用)
            jnp.ones(_n_obs) * self.obstacle_h_baseline_alpha,  # obs_alpha
            jnp.zeros(robot.num_joints),  # u_safe_prev (时序近端项)
        )
        # 保持 h_args 的位置在所有 JAX 模式下一致。即使未启用 ESDF 也
        # 传入一个小占位栅格，最后一项是 TRACKING 已算出的任务 Hessian。
        # 这让 P/q 不必各自再次计算 Jacobian 和伪逆。
        _sdf_shape = self.sdf_shape if self.enable_sdf else (2, 2, 2)
        init_args = init_args + (
            jnp.full(_sdf_shape, 10.0),  # occupied-voxel distance field
            jnp.zeros(3),                # grid origin (world frame)
            jnp.asarray(0.05),           # voxel size (m)
            jnp.asarray(0.0),            # disabled during config validation
            jnp.asarray(0.03),           # safety margin (m)
            jnp.eye(robot.num_joints),   # precomputed task-consistent P block
        )
        super().__init__(
            n=robot.num_joints,
            m=robot.num_joints,
            u_min=-np.array(robot.joint_max_velocities),
            u_max=np.array(robot.joint_max_velocities),
            # M7: elastic QP.  Constraint conflicts are softened by the slack
            # (large penalty keeps slack ~0 in normal operation) instead of
            # triggering a controlled stop; delta_slack is a diagnostic.
            relax_cbf=True,
            cbf_relaxation_penalty=1e5,
            solver_tol=solver_tol,
            init_args=init_args,
        )

    def f(self, z, *args, **kwargs):
        """动力学 (速度级: 无自主动力学)"""
        return jnp.zeros(self.robot.num_joints)

    def g(self, z, *args, **kwargs):
        """控制输入矩阵 (速度级: 单位矩阵)"""
        return jnp.eye(self.robot.num_joints)

    def _P(self, z, *args, **kwargs):
        """OSCBF 任务一致性 P 矩阵

        P = N^T @ W_joint^2 @ N + J^T @ W_task^2 @ J

        参考论文公式 (15)
        """
        q = z
        J = self.robot.ee_jacobian(q)

        # 阻尼伪逆 (论文原文公式)
        lam = 1e-3
        JJT = J @ J.T + lam**2 * jnp.eye(6)
        J_hash = J.T @ jnp.linalg.inv(JJT)

        # 零空间投影
        N = jnp.eye(self.robot.num_joints) - J_hash @ J

        # 权重矩阵 (平方)
        W_task_sq = jnp.diag(jnp.array([
            self.pos_obj_weight**2, self.pos_obj_weight**2, self.pos_obj_weight**2,
            self.rot_obj_weight**2, self.rot_obj_weight**2, self.rot_obj_weight**2
        ]))
        W_joint_sq = self.joint_obj_weight**2 * jnp.eye(self.robot.num_joints)

        # P 矩阵
        P_u = N.T @ W_joint_sq @ N + J.T @ W_task_sq @ J

        # 对称化
        return 0.5 * (P_u + P_u.T)

    def _temporal_weight_sq(self):
        return self.temporal_lambda * self.temporal_wu**2

    def P(self, z, u_des, obs_pos=None, obs_radii=None, obs_enabled=None,
          obs_d_safe=None, obs_vel=None, obs_radius_dot=None, obs_alpha=None,
          u_safe_prev=None, sdf_distance=None, sdf_origin=None,
          sdf_voxel_size=None, sdf_enabled=None, sdf_margin=None,
          task_p=None, *args, **kwargs):
        """QP 二次项，含可选时序近端稳定化。"""
        p_task = self._P(z) if task_p is None else jnp.asarray(task_p)
        if self.temporal_lambda <= 0.0 or u_safe_prev is None:
            return p_task
        p_total = p_task + jnp.diag(self._temporal_weight_sq())
        return 0.5 * (p_total + p_total.T)

    def q(self, z, u_des, obs_pos=None, obs_radii=None, obs_enabled=None,
          obs_d_safe=None, obs_vel=None, obs_radius_dot=None, obs_alpha=None,
          u_safe_prev=None, sdf_distance=None, sdf_origin=None,
          sdf_voxel_size=None, sdf_enabled=None, sdf_margin=None,
          task_p=None, *args, **kwargs):
        """QP 线性项，严格对齐 OSCBFQPSolver 的近端项展开式。"""
        p_task = self._P(z) if task_p is None else jnp.asarray(task_p)
        q_task = -p_task @ u_des
        if self.temporal_lambda <= 0.0 or u_safe_prev is None:
            return q_task
        return q_task - self._temporal_weight_sq() * u_safe_prev

    def h_2(self, z, obs_pos=None, obs_radii=None, obs_enabled=None,
            obs_d_safe=None, obs_vel=None, obs_radius_dot=None,
            obs_alpha=None, u_safe_prev=None, sdf_distance=None,
            sdf_origin=None, sdf_voxel_size=None, sdf_enabled=None,
            sdf_margin=None, task_p=None, *args, **kwargs):
        """所有 CBF 约束 (相对度 2)

        参数
        ----
        obs_pos : (MAX, 3) or None
            障碍物中心位置。None 时无外部障碍物。
        obs_radii : (MAX,) or None
            障碍物半径。
        obs_enabled : (MAX,) or None
            1=启用, 0=禁用。disabled 槽位 h 设为极大值 (不激活)。
        obs_d_safe : (MAX,) or None
            每个障碍物的安全距离。None 时使用默认 30mm。

        约束来源:
        1. 关节限位 (18 个)
        2. 自碰撞避免 (全身碰撞球对)
        3. 障碍物碰撞避免 (全身碰撞球 vs 障碍物球, masked)
        4. 奇异性避免 (1 个)
        """
        q = z

        # 1. 关节限位
        h_joint_upper = self.joint_limit_upper - q - self.joint_limit_cbf_margin
        h_joint_lower = q - self.joint_limit_lower - self.joint_limit_cbf_margin
        h_joint = jnp.concatenate([h_joint_upper, h_joint_lower])

        # 2. 自碰撞避免 (frozen legacy pair topology)
        self_collision_data = self.robot.self_collision_data(q)
        h_self_collision = self._compute_self_collision_constraints(
            self_collision_data[:, :3], self_collision_data[:, 3])

        # 3/4. Environment safety uses the mesh-conservative outer envelope
        # (ESDF rows); obstacle primitives use the DCOL OBB-vs-sphere kernel.
        collision_data = self.robot.environment_collision_data(q)
        robot_positions = collision_data[:, :3]
        robot_radii = collision_data[:, 3]

        # 3. DCOL obstacle avoidance: one fixed-shape row per OBB (masked),
        # soft-min aggregated over the eight tracking slots when enabled.
        h_obstacles = self._compute_dcol_obstacle_constraints(
            q, obs_pos, obs_radii, obs_enabled, obs_d_safe, obs_vel,
            obs_radius_dot, obs_alpha)

        # 4. A point cloud is represented by exactly one distance-field CBF
        # per robot collision sphere.  Its row count never depends on point
        # cloud density or tracked-object count.
        h_esdf = self._compute_esdf_constraints(
            robot_positions, robot_radii, sdf_distance, sdf_origin,
            sdf_voxel_size, sdf_enabled, sdf_margin)

        # 5. 奇异性
        J_pos = self.robot.ee_jacobian(q)[:3, :]
        sigma = jnp.prod(jnp.linalg.svd(J_pos, compute_uv=False))
        h_singularity = jnp.array([sigma - self.singularity_tol])

        return jnp.concatenate([
            h_joint, h_self_collision, h_obstacles, h_esdf, h_singularity])

    def _compute_self_collision_constraints(self, robot_positions, robot_radii):
        """自碰撞约束 (委托共享模块 compute_self_collision_h)"""
        if len(self.self_collision_pairs) == 0:
            return jnp.array([])
        return compute_self_collision_h(
            robot_positions, robot_radii,
            self.self_collision_pairs, self.d_safe_collision)

    def _compute_dcol_obstacle_constraints(self, q, obs_pos, obs_radii,
                                           obs_enabled, obs_d_safe=None,
                                           obs_vel=None, obs_radius_dot=None,
                                           obs_alpha=None):
        """Fixed-shape DCOL OBB-vs-sphere obstacle rows (masked)."""
        if obs_pos is None or obs_enabled is None:
            return jnp.array([])

        from work.jax_barrier_terms import (
            aggregate_dynamic_obstacle_terms,
            compute_dcol_obstacle_clearance,
        )

        if obs_d_safe is None:
            obs_d_safe = jnp.full_like(obs_radii, self.d_safe_collision)
        if obs_vel is None:
            obs_vel = jnp.zeros_like(obs_pos)
        if obs_radius_dot is None:
            obs_radius_dot = jnp.zeros_like(obs_radii)
        h_obs, h_dot_obs = compute_dcol_obstacle_clearance(
            q, obs_pos, obs_radii, obs_d_safe, obs_vel, obs_radius_dot)

        if not self.aggregate_dynamic_obstacles:
            mask = obs_enabled[None, :] > 0.5
            return jnp.where(mask, h_obs, 1e3).ravel()

        alpha = (jnp.full_like(obs_radii, self.obstacle_h_baseline_alpha)
                 if obs_alpha is None else obs_alpha)
        h_aggregate, _, _ = aggregate_dynamic_obstacle_terms(
            h_obs, h_dot_obs, obs_enabled, alpha,
            self.smooth_min_temperature)
        return h_aggregate

    def _compute_esdf_constraints(self, positions, radii, sdf_distance,
                                  sdf_origin, sdf_voxel_size, sdf_enabled,
                                  sdf_margin):
        if not self.enable_sdf:
            return jnp.array([])
        if sdf_distance is None or sdf_enabled is None:
            return jnp.full(self.num_robot_collision_spheres, 1e3)
        sampled = jax.vmap(sample_distance_field_jax, in_axes=(None, 0, None, None))(
            sdf_distance, positions, sdf_origin, sdf_voxel_size)
        margin = self.d_safe_collision if sdf_margin is None else sdf_margin
        raw = sampled - radii - margin
        return jnp.where(sdf_enabled > 0.5, raw, jnp.full_like(raw, 1e3))

    def alpha(self, h, *args, **kwargs):
        """CBF 增益函数"""
        return self.obstacle_h_baseline_alpha * h
