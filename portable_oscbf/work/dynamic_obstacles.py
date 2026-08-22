#!/usr/bin/env python3
"""
dynamic_obstacles.py
====================
实时动态障碍物系统。所有障碍物的位置/速度/激活状态随时间变化，
参数来源于 ik_input.mat 数据 (位置/速度/加速度/进给率/弦误差/NURBS分块)。

依赖: numpy, cbf_types, controller_step_cache, point_cloud_obstacles_dynamic
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass

from work.controller_step_cache import point_jacobian_from_spatial
from work.cbf_types import CbfConstraint, _cbf_upper_bound  # 提取到独立模块，向后兼容 re-export
from work.point_cloud_obstacles_dynamic import (  # 提取到独立模块，向后兼容 re-export
    StaticObstacle, DynamicPointCloudObstacle, MovingObstacle,
)


class TrackingObstacle:
    """
    同轨迹追踪障碍物: 在 EE 前方沿同一 NURBS 轨迹移动。

    数据来源: ik_input.mat position_series, velocity_series
    """

    def __init__(self, traj_data, lead_time: float = 0.4, radius: float = 0.05,
                 r_ee: float = 0.06):
        """
        参数
        ----
        traj_data : IKTrajectoryData
        lead_time : float
            障碍物领先 EE 的时间 (s)。
        radius : float
            障碍物球半径 (m)。
        r_ee : float
            末端执行器等效半径 (m)。
        """
        self.traj = traj_data
        self.lead_time = lead_time
        self.radius = radius
        self.r_ee = r_ee

    def state_at(self, t: float) -> Tuple[np.ndarray, np.ndarray]:
        """返回 (pos_3d, vel_3d) 在世界系"""
        t_obs = min(t + self.lead_time, self.traj.total_time())
        return self.traj.pos_world_at(t_obs), self.traj.vel_world_at(t_obs)

    def get_cbf_constraint(self, t: float, ee_pos: np.ndarray,
                            J_pos: np.ndarray, d_safe: float,
                            alpha: float) -> CbfConstraint:
        """
        构建 CBF 约束: h = ||ee - obs|| - (r_ee + r_obs + d_safe)

        ∇h = (ee - obs)^T / ||ee - obs|| * J_pos
        """
        obs_pos, obs_vel = self.state_at(t)
        diff = ee_pos - obs_pos
        dist = np.linalg.norm(diff)

        r_total = self.r_ee + self.radius + d_safe

        h_val = dist - r_total
        active = h_val < 0.15  # 激活距离

        if dist < 1e-10:
            grad = np.zeros(9)
            h_dot_time = 0.0
        else:
            direction = diff / dist         # (3,)
            grad = direction @ J_pos        # (9,)
            h_dot_time = -float(direction @ obs_vel)

        return CbfConstraint(
            name="tracking_obs",
            G_row=-grad,
            h_bound=_cbf_upper_bound(alpha, h_val, h_dot_time),
            h_value=h_val,
            active=active,
        )


class CrossingObstacle:
    """
    交叉移动障碍物: 在 NURBS 分块边界处横向穿过的动态障碍物。

    数据来源: ik_input.mat boundary_point_index, block_index, position_series
    """

    def __init__(self, traj_data, every_n_blocks: int = 4,
                 speed: float = 0.15, radius: float = 0.04, r_ee: float = 0.06):
        """
        参数
        ----
        every_n_blocks : int
            每隔几个 block 边界设置一个交叉障碍物。
        speed : float
            横向穿行速度 (m/s)。
        radius : float
            障碍物球半径 (m)。
        r_ee : float
            末端执行器等效半径 (m)。
        """
        self.traj = traj_data
        self.speed = speed
        self.radius = radius
        self.r_ee = r_ee

        # 选择激活的块边界
        boundary_times = traj_data.get_boundary_times()
        n_boundaries = len(boundary_times)
        self.active_boundaries = list(range(0, n_boundaries, every_n_blocks))

        # 对每个激活边界，计算交叉位置和方向
        self.crossing_configs = []
        for bi in self.active_boundaries:
            t_b = boundary_times[bi]
            ee_pos = traj_data.pos_world_at(t_b)
            ee_vel = traj_data.vel_world_at(t_b)
            vel_norm = np.linalg.norm(ee_vel)

            # 交叉方向: 垂直于 EE 运动方向
            if vel_norm > 0.01:
                tangent = ee_vel / vel_norm
                # 选择一个垂直方向 (在水平面内)
                up = np.array([0., 0., 1.])
                cross_dir = np.cross(tangent, up)
                cross_norm = np.linalg.norm(cross_dir)
                if cross_norm < 0.1:
                    cross_dir = np.cross(tangent, np.array([1., 0., 0.]))
                    cross_norm = np.linalg.norm(cross_dir)
                cross_dir = cross_dir / cross_norm
            else:
                cross_dir = np.array([1., 0., 0.])

            # 交叉起始位置: 在轨迹点的一侧 (物理偏移 0.15m)
            offset = 0.15  # 起始偏移距离 (m)
            start_pos = ee_pos + cross_dir * offset
            travel_dist = 2.0 * offset

            # 激活时间窗口
            travel_time = travel_dist / self.speed
            t_start = t_b - travel_time / 2
            t_end = t_b + travel_time / 2

            self.crossing_configs.append({
                't_boundary': t_b,
                'cross_dir': cross_dir,
                'start_pos': start_pos,
                'travel_dist': travel_dist,
                't_start': t_start,
                't_end': t_end,
                'pos_at_boundary': ee_pos.copy(),
            })

    def get_active_obstacles(self, t: float) -> List[Tuple[np.ndarray, np.ndarray]]:
        """返回当前激活的交叉障碍物 [(pos, vel)]"""
        active = []
        for cfg in self.crossing_configs:
            if cfg['t_start'] <= t <= cfg['t_end']:
                tau = (t - cfg['t_start']) / (cfg['t_end'] - cfg['t_start'])
                progress = self.speed * (t - cfg['t_start'])
                pos = cfg['start_pos'] - cfg['cross_dir'] * progress
                vel = -cfg['cross_dir'] * self.speed
                active.append((pos, vel))
        return active

    def get_cbf_constraints(self, t: float, ee_pos: np.ndarray,
                             J_pos: np.ndarray, d_safe: float,
                             alpha: float) -> List[CbfConstraint]:
        constraints = []
        for obs_pos, obs_vel in self.get_active_obstacles(t):
            diff = ee_pos - obs_pos
            dist = np.linalg.norm(diff)
            r_total = self.r_ee + self.radius + d_safe
            h_val = dist - r_total

            if dist < 1e-10:
                grad = np.zeros(9)
                h_dot_time = 0.0
            else:
                direction = diff / dist
                grad = direction @ J_pos
                h_dot_time = -float(direction @ obs_vel)

            constraints.append(CbfConstraint(
                name="crossing_obs",
                G_row=-grad,
                h_bound=_cbf_upper_bound(alpha, h_val, h_dot_time, floor=-1.0),
                h_value=h_val,
                active=True,  # 窗口内始终激活
            ))
        return constraints


class SpeedScaledObstacle:
    """
    速度关联膨胀障碍物: 固定位置，有效半径随 EE 速度/加速度动态膨胀。

    数据来源: ik_input.mat speed_series, acceleration_norm_series
    """

    def __init__(self, position: np.ndarray, base_radius: float,
                 k_speed: float = 0.3, k_acc: float = 0.1, r_ee: float = 0.06):
        self.position = np.array(position)
        self.base_radius = base_radius
        self.k_speed = k_speed
        self.k_acc = k_acc
        self.r_ee = r_ee
        self.current_radius = base_radius

    def update(self, t: float, traj_data) -> None:
        """根据当前速度/加速度更新有效半径"""
        speed = traj_data.speed_at(t)
        acc_norm = traj_data.accel_norm_at(t)
        self.current_radius = self.base_radius + self.k_speed * speed + self.k_acc * acc_norm

    def get_cbf_constraint(self, ee_pos: np.ndarray, J_pos: np.ndarray,
                            d_safe: float, alpha: float) -> CbfConstraint:
        diff = ee_pos - self.position
        dist = np.linalg.norm(diff)
        r_total = self.r_ee + self.current_radius + d_safe
        h_val = dist - r_total
        active = h_val < 0.2

        if dist < 1e-10:
            grad = np.zeros(9)
        else:
            direction = diff / dist
            grad = direction @ J_pos

        return CbfConstraint(
            name="speed_scaled_obs",
            G_row=-grad,
            h_bound=alpha * h_val,
            h_value=h_val,
            active=active,
        )


class ChordErrorTube:
    """
    弦误差跟踪管约束: EE 必须保持在以参考轨迹为中心、半径正比于 chord_error 的动态管道内。

    数据来源: ik_input.mat chord_error_series

    这不属于障碍物，而是一个"安全域" (keep-in set) 约束。
    """

    def __init__(self, traj_data, k_chord: float = 3.0, min_radius: float = 0.005):
        """
        参数
        ----
        k_chord : float
            chord_error 放大系数 (管道半径 = k_chord * chord_error)。
        min_radius : float
            管道最小半径 (m)，防止直线段 (chord_error≈0) 约束过紧。
        """
        self.traj = traj_data
        self.k_chord = k_chord
        self.min_radius = min_radius

    def get_cbf_constraint(self, t: float, ee_pos: np.ndarray,
                            ref_pos: np.ndarray, J_pos: np.ndarray,
                            alpha: float) -> CbfConstraint:
        """h = tube_radius - ||ee_pos - ref_pos|| >= 0"""
        chord_err = self.traj.chord_error_at(t)
        tube_radius = max(self.k_chord * chord_err, self.min_radius)

        diff = ee_pos - ref_pos
        dist = np.linalg.norm(diff)
        h_val = tube_radius - dist
        active = True  # 始终激活

        if dist < 1e-10:
            grad = np.zeros(9)
        else:
            direction = -diff / dist       # ∇h 指向参考轨迹
            grad = direction @ J_pos

        return CbfConstraint(
            name="chord_tube",
            G_row=-grad,
            h_bound=alpha * h_val,
            h_value=h_val,
            active=active,
        )


class JerkAwareActivation:
    """
    高 Jerk 段预激活: 在 jerk_norm 大的区域提前扩安全距离和 CBF 增益。

    数据来源: ik_input.mat jerk_norm_series
    """

    def __init__(self, traj_data, threshold: float = 50.0,
                 dist_factor: float = 1.5, alpha_factor: float = 1.3):
        """
        参数
        ----
        threshold : float
            jerk_norm 阈值 (m/s³)。
        dist_factor : float
            碰撞激活距离膨胀系数。
        alpha_factor : float
            CBF α 增益系数。
        """
        self.traj = traj_data
        self.threshold = threshold
        self.dist_factor = dist_factor
        self.alpha_factor = alpha_factor

    def get_factors(self, t: float) -> Tuple[float, float]:
        """返回 (dist_multiplier, alpha_multiplier)"""
        jn = self.traj.jerk_norm_at(t)
        if jn > self.threshold:
            return self.dist_factor, self.alpha_factor
        return 1.0, 1.0


class DynamicObstacleManager:
    """
    统一管理障碍物的生命周期和 CBF 约束生成。

    障碍物策略 (修改后):
      - 所有障碍物在世界坐标系 (world) 中定义, 位置固定;
      - 障碍物分布在机械臂工作空间周围, 不跟随轨迹移动;
      - 不使用以轨迹点为中心的障碍物生成逻辑。
    """

    def __init__(self, traj_data):
        self.traj = traj_data

        # ---- 障碍物已关闭 (纯跟踪验证阶段) ----
        # 障碍物位置待后续阶段启用时根据实际轨迹范围重新标定
        self.static_obs = []      # StaticObstacle 列表
        self.dynamic_obs = []     # SpeedScaledObstacle 列表
        self.moving_obs = []      # MovingObstacle 列表 (可动障碍物)

        # ---- 弦误差跟踪管 (keep-in 约束, 非障碍物) ----
        self.chord_tube = ChordErrorTube(traj_data, k_chord=5.0, min_radius=0.03)
        self.enable_chord_tube = True

        # ---- Jerk 感知激活调制 ----
        self.jerk_activation = JerkAwareActivation(traj_data, threshold=50.0)

    def get_dynamic_d_safe(self, t: float, base: float = None) -> float:
        """进给率耦合安全距离"""
        if base is None:
            base = 0.03  # 3cm 基础安全距离
        feedrate = self.traj.feedrate_cmd_at(t)
        k_feed = 0.5
        return base + k_feed * max(feedrate, 0.001)

    def get_dynamic_alpha(self, t: float, base_alpha: float = 10.0) -> float:
        """Jerk 调制 CBF 增益"""
        jn = self.traj.jerk_norm_at(t)
        max_jn = max(np.max(self.traj._jerk_norm), 1.0)
        k_jerk = 0.5
        return base_alpha * (1.0 + k_jerk * jn / max_jn)

    def get_all_cbf_constraints(self, t: float, q: np.ndarray,
                                 ee_pos: np.ndarray, J_pos: np.ndarray,
                                 kinematics, T_all=None, J_s=None,
                                 ee_ref: np.ndarray = None,
                                 chord_reference_time: float = None) -> List[CbfConstraint]:
        """
        返回当前时刻所有激活的 CBF 约束。

        约束来源:
          1. 静态障碍物 (世界系固定位置, 分布在机械臂周围)
          2. 速度膨胀障碍物 (固定位置, 半径动态变化)
          3. 弦误差跟踪管 (keep-in 约束)

        Parameters
        ----------
        ee_ref : np.ndarray, required
            End-effector reference position (used for chord-tube CBF).
            Must be supplied by the caller to avoid monkey-patching
            ``self.traj.pos_world_at`` (P2-15).
        chord_reference_time : float, optional
            Source time of an arc-length reference. Dynamic obstacles still
            use ``t`` (physical scene time); only chord-error radius sampling
            follows this explicit path reference.
        """
        assert ee_ref is not None, 'ee_ref must be supplied explicitly (P2-15)'
        d_safe_base = self.get_dynamic_d_safe(t)
        alpha_base = self.get_dynamic_alpha(t)

        # Jerk 感知调制
        dist_factor, alpha_factor = self.jerk_activation.get_factors(t)
        d_safe = d_safe_base * dist_factor
        alpha = alpha_base * alpha_factor

        constraints = []

        # 1. 静态障碍物 (机械臂工作空间周围)
        for obs in self.static_obs:
            c = obs.get_cbf_constraint(ee_pos, J_pos, d_safe, alpha)
            if c.active:
                constraints.append(c)

        # 2. 速度膨胀障碍物
        for obs in self.dynamic_obs:
            obs.update(t, self.traj)
            c = obs.get_cbf_constraint(ee_pos, J_pos, d_safe, alpha)
            if c.active:
                constraints.append(c)

        # 2.5. 可动障碍物 (MovingObstacle: update(t) + 全身关节检测)
        for obs in self.moving_obs:
            obs.update(t)
            c_list = obs.get_cbf_constraint(ee_pos, J_pos, d_safe, alpha,
                                            q=q, kin=kinematics,
                                            T_all=T_all, J_s=J_s)
            constraints.extend([c for c in c_list if c.active])

        # 3. 弦误差跟踪管
        if self.enable_chord_tube:
            ref_pos = ee_ref
            chord_time = t if chord_reference_time is None else chord_reference_time
            c = self.chord_tube.get_cbf_constraint(
                chord_time, ee_pos, ref_pos, J_pos, alpha)
            constraints.append(c)

        return constraints

    def get_all_obstacle_poses(self, t: float) -> List[dict]:
        """返回所有障碍物当前可视化位姿 (供 RViz / PyBullet 使用)"""
        obstacles = []

        # 静态障碍物 — 蓝色
        for obs in self.static_obs:
            obstacles.append({
                'type': 'static', 'pos': obs.position,
                'vel': np.zeros(3), 'radius': obs.radius,
                'color': [0.3, 0.5, 1.0, 0.7],
            })

        # 速度膨胀障碍物 — 黄色
        for obs in self.dynamic_obs:
            obstacles.append({
                'type': 'dynamic', 'pos': obs.position,
                'vel': np.zeros(3), 'radius': obs.current_radius,
                'color': [1.0, 0.8, 0.1, 0.6],
            })

        # 可动障碍物 — 红色
        for obs in self.moving_obs:
            obstacles.append({
                'type': 'moving', 'pos': obs.position.copy(),
                'vel': obs.velocity.copy(), 'radius': obs.radius,
                'color': [1.0, 0.2, 0.1, 0.7],
            })

        return obstacles
