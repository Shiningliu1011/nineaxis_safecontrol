#!/usr/bin/env python3
"""
point_cloud_obstacles_dynamic.py
================================
独立的点云/动态障碍物类 — 从 dynamic_obstacles.py 提取。

包含:
  - StaticObstacle: 静态球体障碍物
  - DynamicPointCloudObstacle: 解析距离动态障碍物 (2球+1柱)
  - MovingObstacle: 沿路径移动的球体障碍物

依赖: numpy, cbf_types, controller_step_cache
"""

from typing import List, Optional

import numpy as np

from work.cbf_types import CbfConstraint, _cbf_upper_bound
from work.controller_step_cache import point_jacobian_from_spatial
from work.robot_geometry import BODY_SEGMENTS

# P2-28: 统一末端执行器碰撞半径 (原分散在 0.04/0.045/0.06 三处)
EE_COLLISION_RADIUS = 0.05

# P3-29: 统一 CBF h_bound floor (原 -0.5 / -0.2 / 无 不一致)
CBF_H_BOUND_FLOOR = -0.5


class StaticObstacle:
    """
    静态障碍物: 放置在机械臂本体周围的固定球体障碍物。

    不依赖轨迹数据，位置在世界系中固定，用于模拟工作空间中的
    真实障碍物 (如设备支架、防护笼壁等)。
    """

    def __init__(self, position: np.ndarray, radius: float = 0.06,
                 r_ee: float = EE_COLLISION_RADIUS, name: str = "static_obs"):  # P2-28
        self.name = name
        self.position = np.array(position)
        self.radius = radius
        self.r_ee = r_ee

    def get_cbf_constraint(self, ee_pos: np.ndarray, J_pos: np.ndarray,
                            d_safe: float, alpha: float) -> CbfConstraint:
        diff = ee_pos - self.position
        dist = np.linalg.norm(diff)
        r_total = self.r_ee + self.radius + d_safe
        h_val = dist - r_total
        active = h_val < 0.25  # 较大的激活距离 (静态障碍物需要提前感知)

        if dist < 1e-10:
            grad = np.zeros(9)
        else:
            direction = diff / dist
            grad = direction @ J_pos

        return CbfConstraint(
            name=self.name,
            G_row=-grad,
            h_bound=alpha * h_val,
            h_value=h_val,
            active=active,
        )


class DynamicPointCloudObstacle:
    """动态障碍物: 解析距离计算 (无点云采样, 无 FCL 查询)。

    三个基元 (2球 + 1柱), 位置/尺寸正弦振荡。
    直接计算障碍物到关节/连杆的解析距离, 比点云 FCL 快 100x+。
    """

    # 关节球半径 (用于解析距离减去)
    _JOINT_RADII = {
        0: 0.06,   # base_link (Box 近似)
        1: 0.05,   # Link1
        2: 0.080,  # Link2
        3: 0.075,  # Link3
        4: 0.070,  # Link4
        5: 0.070,  # Link5
        6: 0.060,  # Link6
        7: 0.055,  # Link7
        8: 0.055,  # Link8
        9: 0.050,  # Link9/ee
    }

    _BODY_POINTS = [
        ('base_link', 0, 0.060),
        ('Link1', 1, 0.050),
        ('Link2', 2, 0.080),
        ('Link3', 3, 0.075),
        ('Link4', 4, 0.070),
        ('Link5', 5, 0.070),
        ('Link6', 6, 0.060),
        ('Link7', 7, 0.055),
        ('Link8', 8, 0.055),
        ('Link9', 9, 0.050),
        ('ee_link', 9, EE_COLLISION_RADIUS),
    ]

    # _BODY_SEGMENTS removed: use shared BODY_SEGMENTS from work.robot_geometry

    def __init__(self, base_center_sphere1, base_center_sphere2, base_center_cylinder,
                 sphere1_base_radius=0.05, sphere2_base_radius=0.04,
                 cyl_base_radius=0.03, cyl_base_height=0.10,
                 scenario='side_sweep', max_constraints_per_obstacle=4,
                 enabled_obstacles=None):
        valid_scenarios = (
            'side_sweep',
            'sudden',
            'link5_sweep',
            'ee_crossing',
            'corridor_probe',
            'pop_up',
        )
        if scenario not in valid_scenarios:
            raise ValueError(f"Unknown dynamic obstacle scenario: {scenario}")
        self.scenario = scenario
        self.max_constraints_per_obstacle = int(max_constraints_per_obstacle)
        valid_obstacles = {"sphere1", "sphere2", "cylinder"}
        if enabled_obstacles is None:
            enabled_obstacles = valid_obstacles
        self.enabled_obstacles = set(enabled_obstacles)
        unknown = self.enabled_obstacles - valid_obstacles
        if unknown:
            raise ValueError(f"Unknown dynamic obstacle part(s): {sorted(unknown)}")
        self.sphere1_base_center = np.array(base_center_sphere1, dtype=float)
        self.sphere2_base_center = np.array(base_center_sphere2, dtype=float)
        self.cyl_base_center = np.array(base_center_cylinder, dtype=float)

        self.sphere1_base_radius = sphere1_base_radius
        self.sphere1_center = self.sphere1_base_center.copy()
        self.sphere1_radius = sphere1_base_radius
        self.sphere1_velocity = np.zeros(3)
        self.sphere1_radius_dot = 0.0

        self.sphere2_base_radius = sphere2_base_radius
        self.sphere2_center = self.sphere2_base_center.copy()
        self.sphere2_radius = sphere2_base_radius
        self.sphere2_velocity = np.zeros(3)
        self.sphere2_radius_dot = 0.0

        self.cyl_base_radius = cyl_base_radius
        self._cyl_base_radius_saved = cyl_base_radius  # P3-31: 保存原值
        self.cyl_base_height = cyl_base_height
        self.cyl_center = self.cyl_base_center.copy()
        self.cyl_height = cyl_base_height
        self.cyl_velocity = np.zeros(3)
        self.cyl_height_dot = 0.0

        # 振荡参数 - 沿臂延伸方向 (Z) 和垂直方向 (X)
        # 振幅 << h值(80mm), 保证 CBF QP 始终可行
        self._sphere1_amp_z = 0.02     # 球1 Z 振荡 (沿臂方向)
        self._sphere1_period_z = 4.0
        self._sphere1_amp_r = 0.005
        self._sphere1_period_r = 3.0

        self._sphere2_amp_x = 0.015    # 球2 X 振荡 (垂直臂方向)
        self._sphere2_period_x = 3.5
        self._sphere2_amp_z = 0.02     # 球2 Z 振荡 (沿臂方向)
        self._sphere2_period_z = 3.0
        self._sphere2_amp_r = 0.005
        self._sphere2_period_r = 2.5

        self._cyl_amp_z = 0.02         # 柱 Z 振荡 (沿臂方向)
        self._cyl_period_z = 4.0
        self._cyl_amp_h = 0.01
        self._cyl_period_h = 3.5
        self._sudden_appear_t = 5.0
        self._sudden_ramp_t = 0.4

    def is_obstacle_enabled(self, name: str) -> bool:
        return str(name) in self.enabled_obstacles

    def _zero_disabled_obstacles(self):
        if not self.is_obstacle_enabled("sphere1"):
            self.sphere1_radius = 0.0
            self.sphere1_velocity = np.zeros(3)
            self.sphere1_radius_dot = 0.0
        if not self.is_obstacle_enabled("sphere2"):
            self.sphere2_radius = 0.0
            self.sphere2_velocity = np.zeros(3)
            self.sphere2_radius_dot = 0.0
        if not self.is_obstacle_enabled("cylinder"):
            self.cyl_base_radius = 0.0
            self.cyl_height = 0.0
            self.cyl_velocity = np.zeros(3)
            self.cyl_height_dot = 0.0
        elif self.cyl_base_radius == 0.0 and hasattr(self, '_cyl_base_radius_saved'):
            # P3-31: 恢复被 _zero_disabled_obstacles 覆写的原值
            self.cyl_base_radius = self._cyl_base_radius_saved

    def update(self, t: float):
        """按时间 t 更新障碍物几何参数 (正弦振荡)。"""
        # 球1: Z 振荡 (沿臂方向) + 半径变化
        self.sphere1_center = self.sphere1_base_center.copy()
        w_s1_z = 2.0 * np.pi / self._sphere1_period_z
        self.sphere1_center[2] += self._sphere1_amp_z * np.sin(w_s1_z * t)
        self.sphere1_velocity = np.zeros(3)
        self.sphere1_velocity[2] = self._sphere1_amp_z * w_s1_z * np.cos(w_s1_z * t)
        w_s1_r = 2.0 * np.pi / self._sphere1_period_r
        self.sphere1_radius = (self.sphere1_base_radius
                               + self._sphere1_amp_r * np.sin(w_s1_r * t))
        self.sphere1_radius_dot = self._sphere1_amp_r * w_s1_r * np.cos(w_s1_r * t)

        # 球2: X+Z 振荡 + 半径变化
        self.sphere2_center = self.sphere2_base_center.copy()
        w_s2_x = 2.0 * np.pi / self._sphere2_period_x
        w_s2_z = 2.0 * np.pi / self._sphere2_period_z
        self.sphere2_center[0] += self._sphere2_amp_x * np.sin(w_s2_x * t)
        self.sphere2_center[2] += self._sphere2_amp_z * np.sin(w_s2_z * t)
        self.sphere2_velocity = np.zeros(3)
        self.sphere2_velocity[0] = self._sphere2_amp_x * w_s2_x * np.cos(w_s2_x * t)
        self.sphere2_velocity[2] = self._sphere2_amp_z * w_s2_z * np.cos(w_s2_z * t)
        w_s2_r = 2.0 * np.pi / self._sphere2_period_r
        self.sphere2_radius = (self.sphere2_base_radius
                               + self._sphere2_amp_r * np.sin(w_s2_r * t))
        self.sphere2_radius_dot = self._sphere2_amp_r * w_s2_r * np.cos(w_s2_r * t)

        # 柱: Z 振荡 (沿臂方向) + 高度变化
        self.cyl_center = self.cyl_base_center.copy()
        w_c_z = 2.0 * np.pi / self._cyl_period_z
        self.cyl_center[2] += self._cyl_amp_z * np.sin(w_c_z * t)
        self.cyl_velocity = np.zeros(3)
        self.cyl_velocity[2] = self._cyl_amp_z * w_c_z * np.cos(w_c_z * t)
        w_c_h = 2.0 * np.pi / self._cyl_period_h
        self.cyl_height = (self.cyl_base_height
                           + self._cyl_amp_h * np.sin(w_c_h * t))
        self.cyl_height_dot = self._cyl_amp_h * w_c_h * np.cos(w_c_h * t)

        if self.scenario == 'link5_sweep':
            w = 2.0 * np.pi / 2.8
            self.sphere1_center = self.sphere1_base_center.copy()
            self.sphere1_center[1] += 0.075 * np.sin(w * t)
            self.sphere1_velocity = np.array([0.0, 0.075 * w * np.cos(w * t), 0.0])
            self.cyl_center = self.cyl_base_center.copy()
            self.cyl_center[0] += 0.035 * np.sin(0.8 * w * t)
            self.cyl_center[2] += 0.030 * np.cos(0.8 * w * t)
            self.cyl_velocity = np.array([
                0.035 * 0.8 * w * np.cos(0.8 * w * t),
                0.0,
                -0.030 * 0.8 * w * np.sin(0.8 * w * t),
            ])
        elif self.scenario == 'ee_crossing':
            w = 2.0 * np.pi / 3.2
            self.sphere2_center = self.sphere2_base_center.copy()
            self.sphere2_center[0] += 0.090 * np.sin(w * t)
            self.sphere2_center[2] += 0.025 * np.cos(w * t)
            self.sphere2_velocity = np.array([
                0.090 * w * np.cos(w * t),
                0.0,
                -0.025 * w * np.sin(w * t),
            ])
        elif self.scenario == 'corridor_probe':
            w = 2.0 * np.pi / 4.0
            self.sphere1_center = self.sphere1_base_center + np.array([
                0.040 * np.sin(w * t),
                0.020 * np.cos(w * t),
                0.0,
            ])
            self.sphere1_velocity = np.array([
                0.040 * w * np.cos(w * t),
                -0.020 * w * np.sin(w * t),
                0.0,
            ])
            self.sphere2_center = self.sphere2_base_center + np.array([
                -0.040 * np.sin(w * t),
                0.020 * np.cos(w * t),
                0.0,
            ])
            self.sphere2_velocity = np.array([
                -0.040 * w * np.cos(w * t),
                -0.020 * w * np.sin(w * t),
                0.0,
            ])
            self.cyl_center = self.cyl_base_center.copy()
            self.cyl_center[2] += 0.040 * np.sin(0.7 * w * t)
            self.cyl_velocity = np.array([
                0.0,
                0.0,
                0.040 * 0.7 * w * np.cos(0.7 * w * t),
            ])

        if self.scenario == 'sudden':
            growth, growth_dot = self._smooth_appearance(t)
            radius_nominal = self.sphere2_radius
            radius_dot_nominal = self.sphere2_radius_dot
            self.sphere2_radius = growth * radius_nominal
            self.sphere2_radius_dot = (growth * radius_dot_nominal
                                       + growth_dot * radius_nominal)

        if self.scenario == 'pop_up':
            self._update_pop_up(t)

        self._zero_disabled_obstacles()

    def _smooth_appearance(self, t: float):
        """突然出现测试: 用短平滑 ramp 代替不连续半径跳变。"""
        if t <= self._sudden_appear_t:
            return 0.0, 0.0
        ramp = max(self._sudden_ramp_t, 1e-6)
        x = (t - self._sudden_appear_t) / ramp
        if x >= 1.0:
            return 1.0, 0.0
        value = x * x * (3.0 - 2.0 * x)
        value_dot = 6.0 * x * (1.0 - x) / ramp
        return value, value_dot

    # ---- pop_up 场景: 障碍物突然出现/消失 ----

    def _init_pop_up_events(self):
        """初始化 pop_up 事件列表。

        每个事件: (appear_t, disappear_t, center, radius)
        障碍物在 appear_t 时平滑出现, 在 disappear_t 时平滑消失。
        位置在机械臂连杆附近 (基于零位 FK 估计)。
        """
        # 机械臂连杆大致位置 (零位, Y-up)
        link_positions = [
            np.array([0.0, 0.34, 0.0]),     # Link2 附近
            np.array([0.0, 0.34, 0.22]),     # Link3 附近
            np.array([0.0, 0.34, 0.45]),     # Link4 附近
            np.array([0.0, 0.34, 0.79]),     # Link5 附近
            np.array([0.0, 0.34, 0.93]),     # Link7 附近
            np.array([0.0, 0.34, 1.04]),     # Link8 附近
            np.array([0.0, 0.34, 1.15]),     # Link9 附近
        ]

        # 事件: (出现时间, 消失时间, 位置偏移, 半径)
        # 在不同时间、不同连杆位置突然出现 (保持安全距离)
        # 注意: 位置基于零位 FK, 机械臂在轨迹中会移动, 需留足余量
        self._pop_up_events = [
            # t=2s: Link3 侧方 0.25m, 持续2s
            (2.0, 4.0, link_positions[1] + np.array([0.25, 0.0, 0.0]), 0.04),
            # t=5s: Link5 上方 0.2m, 持续2s
            (5.0, 7.0, link_positions[3] + np.array([0.0, 0.20, 0.0]), 0.04),
            # t=9s: Link7 侧方 0.25m, 持续1.5s
            (9.0, 10.5, link_positions[4] + np.array([0.25, 0.0, 0.0]), 0.035),
            # t=13s: Link4 对侧 0.3m, 持续1.5s
            (13.0, 14.5, link_positions[2] + np.array([-0.30, 0.0, 0.0]), 0.04),
            # t=17s: Link8 下方 0.2m, 持续1.5s
            (17.0, 18.5, link_positions[5] + np.array([0.0, -0.20, 0.0]), 0.035),
            # t=21s: Link5 侧方 0.35m, 持续1.5s
            (21.0, 22.5, link_positions[3] + np.array([0.35, 0.0, 0.0]), 0.03),
            # t=25s: Link6 侧方 0.25m, 持续1.5s
            (25.0, 26.5, link_positions[3] + np.array([0.25, 0.0, 0.0]), 0.035),
        ]
        self._pop_up_ramp = 0.3  # 出现/消失的平滑时间 (s)

    def _update_pop_up(self, t: float):
        """pop_up 场景更新: 在不同时间让障碍物在连杆附近突然出现/消失。"""
        if not hasattr(self, '_pop_up_events'):
            self._init_pop_up_events()

        # 找到当前活跃的事件 (取最近的一个)
        best_event = None
        best_growth = 0.0

        for appear_t, disappear_t, center, radius in self._pop_up_events:
            if t < appear_t:
                continue  # 还没到出现时间
            if t > disappear_t + self._pop_up_ramp:
                continue  # 已经完全消失

            # 计算生长因子 (出现/消失的平滑过渡)
            ramp = self._pop_up_ramp

            if t < appear_t + ramp:
                # 正在出现
                x = (t - appear_t) / ramp
                growth = x * x * (3.0 - 2.0 * x)
            elif t < disappear_t:
                # 完全可见
                growth = 1.0
            else:
                # 正在消失
                x = (disappear_t + ramp - t) / ramp
                growth = x * x * (3.0 - 2.0 * x)

            if growth > best_growth:
                best_growth = growth
                best_event = (center, radius, growth)

        if best_event is not None:
            center, radius, growth = best_event
            # 用 sphere1 表示 pop_up 障碍物
            self.sphere1_center = center.copy()
            self.sphere1_radius = growth * radius
            self.sphere1_radius_dot = 0.0  # 简化: 忽略半径变化率
            self.sphere1_velocity = np.zeros(3)
        else:
            # 没有活跃事件, 障碍物不可见
            self.sphere1_radius = 0.001  # 几乎为零
            self.sphere1_radius_dot = 0.0

    def get_points(self) -> np.ndarray:
        """采样表面点 (仅用于可视化, 不用于碰撞检测)。"""
        from work.point_cloud_obstacles import _sample_sphere_surface, _sample_cylinder_surface
        point_sets = []
        if self.is_obstacle_enabled("sphere1"):
            point_sets.append(_sample_sphere_surface(
                self.sphere1_center, self.sphere1_radius, n_points=200))
        if self.is_obstacle_enabled("sphere2"):
            point_sets.append(_sample_sphere_surface(
                self.sphere2_center, self.sphere2_radius, n_points=150))
        if self.is_obstacle_enabled("cylinder"):
            point_sets.append(_sample_cylinder_surface(
                self.cyl_center, self.cyl_base_radius, self.cyl_height, n_points=150))
        if not point_sets:
            return np.empty((0, 3), dtype=float)
        return np.vstack(point_sets)

    def get_all_points(self) -> np.ndarray:
        return self.get_points()

    @staticmethod
    def _point_to_segment_dist(p, a, b):
        """点 p 到线段 ab 的最短距离。"""
        ab = b - a
        ap = p - a
        t = np.dot(ap, ab) / max(np.dot(ab, ab), 1e-12)
        t = np.clip(t, 0.0, 1.0)
        closest = a + t * ab
        return np.linalg.norm(p - closest), closest, t

    @staticmethod
    def _segment_to_segment_closest(p1, q1, p2, q2):
        """两条线段的最近点: 返回 (distance, c1, c2, s, t)。"""
        d1 = q1 - p1
        d2 = q2 - p2
        r = p1 - p2
        a = float(d1 @ d1)
        e = float(d2 @ d2)
        f = float(d2 @ r)
        eps = 1e-12

        if a <= eps and e <= eps:
            c1, c2 = p1, p2
            return np.linalg.norm(c1 - c2), c1, c2, 0.0, 0.0
        if a <= eps:
            s = 0.0
            t = np.clip(f / max(e, eps), 0.0, 1.0)
        else:
            c = float(d1 @ r)
            if e <= eps:
                t = 0.0
                s = np.clip(-c / max(a, eps), 0.0, 1.0)
            else:
                b = float(d1 @ d2)
                denom = a * e - b * b
                if denom != 0.0:
                    s = np.clip((b * f - c * e) / denom, 0.0, 1.0)
                else:
                    s = 0.0
                t_nom = b * s + f
                if t_nom < 0.0:
                    t = 0.0
                    s = np.clip(-c / max(a, eps), 0.0, 1.0)
                elif t_nom > e:
                    t = 1.0
                    s = np.clip((b - c) / max(a, eps), 0.0, 1.0)
                else:
                    t = t_nom / e

        c1 = p1 + d1 * s
        c2 = p2 + d2 * t
        return np.linalg.norm(c1 - c2), c1, c2, float(s), float(t)

    def get_cbf_constraints(self, q, kin, fcl_collision=None,
                            alpha: float = 10.0, d_safe: float = 0.03,
                            activation: float = 0.15,
                            T_all=None, J_s=None) -> List[CbfConstraint]:
        """解析距离计算生成 CBF 约束 (无 FCL, 无点云采样)。"""
        if T_all is None:
            T_all = kin.forward_kinematics(q)
        if J_s is None and hasattr(kin, "compute_spatial_jacobian_world"):
            J_s = kin.compute_spatial_jacobian_world(q)
        body_points = []
        for body_name, link_idx, body_radius in self._BODY_POINTS:
            if body_name in T_all:
                body_points.append((body_name, link_idx, T_all[body_name][:3, 3],
                                    body_radius))

        # 三个障碍物: (center, radius, type, name_prefix)
        obstacles = []
        if self.is_obstacle_enabled("sphere1"):
            obstacles.append(
                (self.sphere1_center, self.sphere1_radius, 'sphere', 'dyn_s1',
                 self.sphere1_velocity, self.sphere1_radius_dot))
        if self.is_obstacle_enabled("sphere2"):
            obstacles.append(
                (self.sphere2_center, self.sphere2_radius, 'sphere', 'dyn_s2',
                 self.sphere2_velocity, self.sphere2_radius_dot))
        # 柱: 用轴线段表示 (center ± height/2 * Z_axis)
        cyl_half = self.cyl_height / 2.0
        cyl_top = self.cyl_center + np.array([0, 0, cyl_half])
        cyl_bot = self.cyl_center - np.array([0, 0, cyl_half])
        if self.is_obstacle_enabled("cylinder"):
            obstacles.append((self.cyl_center, self.cyl_base_radius, 'cylinder', 'dyn_cyl',
                              self.cyl_velocity, 0.0))

        best_by_obstacle = {}
        def add_constraint(name, robot_point, robot_radius, link_idx,
                           obs_center, obs_radius, obs_type, obs_velocity,
                           radius_dot, obs_name, obs_seg_t=None):
            if obs_type == 'sphere':
                dist_to_center = np.linalg.norm(robot_point - obs_center)
                dist = dist_to_center - obs_radius - robot_radius
                if dist_to_center > 1e-8:
                    normal = (robot_point - obs_center) / dist_to_center
                else:
                    normal = np.array([0.0, 0.0, 1.0])
                h_dot_time = -float(normal @ obs_velocity) - float(radius_dot)
            else:
                if obs_seg_t is None:
                    d, closest, obs_seg_t = self._point_to_segment_dist(
                        robot_point, cyl_bot, cyl_top)
                else:
                    d = np.linalg.norm(robot_point - obs_center)
                    closest = obs_center
                dist = d - obs_radius - robot_radius
                diff = robot_point - closest
                diff_norm = np.linalg.norm(diff)
                if diff_norm > 1e-8:
                    normal = diff / diff_norm
                else:
                    normal = np.array([0.0, 0.0, 1.0])
                closest_velocity = obs_velocity.copy()
                closest_velocity[2] += (2.0 * obs_seg_t - 1.0) * 0.5 * self.cyl_height_dot
                h_dot_time = -float(normal @ closest_velocity)

            h_val = dist - d_safe
            if h_val > activation:
                return

            if J_s is not None:
                J_point = point_jacobian_from_spatial(J_s, link_idx, robot_point)
            else:
                J_point = kin.point_jacobian(q, link_idx, robot_point)
            G_row = -normal @ J_point
            constraint = CbfConstraint(
                name=name,
                G_row=G_row,
                h_bound=_cbf_upper_bound(alpha, h_val, h_dot_time, floor=CBF_H_BOUND_FLOOR),
                h_value=h_val,
                active=True,
            )
            best_by_obstacle.setdefault(obs_name, []).append(constraint)

        for obs_center, obs_radius, obs_type, obs_name, obs_velocity, radius_dot in obstacles:
            for j_idx, (body_name, link_idx, jp, j_rad) in enumerate(body_points):
                if obs_type == 'cylinder':
                    d, closest, obs_seg_t = self._point_to_segment_dist(jp, cyl_bot, cyl_top)
                    obs_point = closest
                else:
                    obs_seg_t = None
                    obs_point = obs_center
                add_constraint(
                    f"{obs_name}_j{j_idx}", jp, j_rad, link_idx,
                    obs_point, obs_radius, obs_type, obs_velocity, radius_dot,
                    obs_name=obs_name, obs_seg_t=obs_seg_t)

            # 空间过滤: 障碍物到连杆的快速粗筛距离
            obs_max_reach = obs_radius + d_safe + activation + 0.15  # 粗筛裕量

            for a_name, b_name, link_idx, seg_radius in BODY_SEGMENTS:
                if a_name not in T_all or b_name not in T_all:
                    continue
                a = T_all[a_name][:3, 3]
                b = T_all[b_name][:3, 3]
                # 快速粗筛: 障碍物中心到线段中点距离 > 最大可达距离则跳过
                seg_mid = 0.5 * (a + b)
                if np.linalg.norm(obs_center - seg_mid) > obs_max_reach + np.linalg.norm(b - a):
                    continue
                if obs_type == 'sphere':
                    _, robot_point, _ = self._point_to_segment_dist(obs_center, a, b)
                    obs_point = obs_center
                    obs_seg_t = None
                else:  # cylinder
                    _, robot_point, obs_point, _, obs_seg_t = self._segment_to_segment_closest(
                        a, b, cyl_bot, cyl_top)
                add_constraint(
                    f"{obs_name}_seg_{a_name}_{b_name}", robot_point, seg_radius,
                    link_idx, obs_point, obs_radius, obs_type, obs_velocity,
                    radius_dot, obs_name=obs_name, obs_seg_t=obs_seg_t)

        constraints = []
        for candidates in best_by_obstacle.values():
            candidates.sort(key=lambda item: item.h_value)
            constraints.extend(candidates[:self.max_constraints_per_obstacle])
        return constraints


class MovingObstacle:
    """
    可动障碍物: 沿指定路径移动的球体, 用于测试实时避障能力。

    支持两种运动模式:
      - 'linear': p_start ↔ p_end 直线往返, 周期 T 秒 (匀速三角波)
      - 'sinusoidal': base + amplitude * sin(2π*t/period) 三维正弦振荡

    不依赖轨迹数据, 障碍物自行按时间更新位置。
    CBF 约束使用 EE 点保护公式 (与 StaticObstacle 相同), 并附加速度耦合安全距离膨胀。
    """

    def __init__(self, name: str = "moving_obs",
                 path_type: str = 'linear',
                 p_start: np.ndarray = None,
                 p_end: np.ndarray = None,
                 period: float = 5.0,
                 base: np.ndarray = None,
                 amplitude: np.ndarray = None,
                 radius: float = 0.04,
                 r_ee: float = EE_COLLISION_RADIUS):  # P2-28
        """
        参数
        ----
        name : str
            障碍物名称 (CBF 约束标识)。
        path_type : 'linear' | 'sinusoidal'
            运动模式。
        p_start/p_end : np.ndarray (3,)
            直线运动的两端点 (仅 linear 模式)。
        period : float
            往返周期 (秒)。
        base/amplitude : np.ndarray (3,)
            正弦振荡的基准位置和振幅 (仅 sinusoidal 模式)。
        radius : float
            障碍物球体半径 (m)。
        r_ee : float
            末端执行器等效包围半径 (m)。
        """
        self.name = name
        self.path_type = path_type
        self.period = period
        self.radius = radius
        self.r_ee = r_ee

        if path_type == 'linear':
            if p_start is None or p_end is None:
                raise ValueError("linear mode requires p_start and p_end")
            self.p_start = np.array(p_start, dtype=float)
            self.p_end = np.array(p_end, dtype=float)
            self.position = self.p_start.copy()
            self.velocity = np.zeros(3)
        elif path_type == 'sinusoidal':
            if base is None or amplitude is None:
                raise ValueError("sinusoidal mode requires base and amplitude")
            self.base = np.array(base, dtype=float)
            self.amplitude = np.array(amplitude, dtype=float)
            self.position = self.base.copy()
            self.velocity = np.zeros(3)
        else:
            raise ValueError(f"Unknown path_type: {path_type}")

    def update(self, t: float):
        """更新当前时刻的位置和速度 (由管理器每步调用)"""
        phase = (t % self.period) / self.period  # [0, 1)

        if self.path_type == 'linear':
            # 前半周期 p_start→p_end, 后半周期 p_end→p_start (三角波)
            if phase < 0.5:
                s = 2.0 * phase                          # [0, 1]
                self.position = self.p_start + s * (self.p_end - self.p_start)
                self.velocity = (2.0 / self.period) * (self.p_end - self.p_start)
            else:
                s = 2.0 * (phase - 0.5)                  # [0, 1]
                self.position = self.p_end + s * (self.p_start - self.p_end)
                self.velocity = -(2.0 / self.period) * (self.p_end - self.p_start)

        elif self.path_type == 'sinusoidal':
            omega = 2.0 * np.pi / self.period
            sin_val = np.sin(omega * t)
            cos_val = np.cos(omega * t)
            self.position = self.base + self.amplitude * sin_val
            self.velocity = omega * self.amplitude * cos_val

    # 全身检测用的关节半径 (与 DynamicPointCloudObstacle 共用)
    _FULL_BODY_RADII = {
        0: 0.06, 1: 0.05, 2: 0.080, 3: 0.075, 4: 0.070,
        5: 0.070, 6: 0.060, 7: 0.055, 8: 0.055, 9: 0.050,
    }
    _JOINT_FK_NAMES = ["base_link", "Link1", "Link2", "Link3", "Link4",
                       "Link5", "Link6", "Link7", "Link8", "Link9", "ee_link"]

    def get_cbf_constraint(self, ee_pos: np.ndarray, J_pos: np.ndarray,
                            d_safe: float, alpha: float,
                            q: np.ndarray = None, kin=None,
                            T_all=None, J_s=None) -> list:
        """构建 CBF 约束。若提供 q 和 kin，进行全身关节检测；否则仅 EE 点保护。
        返回 CbfConstraint 列表 (全身模式可能返回多个)。"""
        constraints = []
        speed = np.linalg.norm(self.velocity)
        k_speed = 0.10   # 速度→距离增益 (增大提前检测)
        activation_dist = 0.30 + k_speed * speed  # 增大激活距离, 提前反应

        if q is not None and kin is not None:
            # ---- 全身关节检测 ----
            if T_all is None:
                T_all = kin.forward_kinematics(q)
            if J_s is None and hasattr(kin, "compute_spatial_jacobian_world"):
                J_s = kin.compute_spatial_jacobian_world(q)
            for fk_name in self._JOINT_FK_NAMES:
                if fk_name not in T_all:
                    continue
                # 用 kinematics 内部的 _link_active_count 获取正确的关节索引
                link_idx = kin._link_active_count.get(fk_name, 0)
                jp = T_all[fk_name][:3, 3]
                j_rad = self._FULL_BODY_RADII.get(link_idx, 0.05)

                diff = jp - self.position
                dist = np.linalg.norm(diff)
                r_total = j_rad + self.radius + d_safe + k_speed * speed
                h_val = dist - r_total

                if h_val > activation_dist:
                    continue

                if dist < 1e-10:
                    normal = np.array([0, 0, 1.0])
                else:
                    normal = diff / dist
                h_dot_time = -float(normal @ self.velocity)

                if J_s is not None:
                    J_point = point_jacobian_from_spatial(J_s, link_idx, jp)
                else:
                    J_point = kin.point_jacobian(q, link_idx, jp)
                G_row = -normal @ J_point

                constraints.append(CbfConstraint(
                    name=f"{self.name}_l{link_idx}",
                    G_row=G_row,
                    h_bound=_cbf_upper_bound(alpha, h_val, h_dot_time, floor=CBF_H_BOUND_FLOOR),
                    h_value=h_val,
                    active=True,
                ))
        else:
            # ---- 仅 EE 点保护 (向后兼容) ----
            diff = ee_pos - self.position
            dist = np.linalg.norm(diff)
            r_total = self.r_ee + self.radius + d_safe + k_speed * speed
            h_val = dist - r_total
            active = h_val < activation_dist

            if dist < 1e-10:
                grad = np.zeros(9)
                h_dot_time = 0.0
            else:
                direction = diff / dist
                grad = direction @ J_pos
                h_dot_time = -float(direction @ self.velocity)

            constraints.append(CbfConstraint(
                name=self.name,
                G_row=-grad,
                h_bound=_cbf_upper_bound(alpha, h_val, h_dot_time),
                h_value=h_val,
                active=active,
            ))

        return constraints
