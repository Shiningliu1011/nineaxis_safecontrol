#!/usr/bin/env python3
"""
point_cloud_obstacles.py
========================
基于 FCL 包围体的环境点云碰撞检测。

复用 fcl_collision.py 的 Box/Sphere/Capsule 基元，
用 FCL distance API 计算每个包围体到点云的精确距离，
构建全身 CBF 约束 (非仅末端)。

依赖: numpy, scipy, python-fcl
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass

import fcl

from controller_step_cache import point_jacobian_from_spatial


# ================================================================
# 几何基元→点云采样
# ================================================================

def _sample_box_surface(center: np.ndarray, size: np.ndarray,
                        density: float = 5000.0) -> np.ndarray:
    """Box 6 个面均匀网格采样。"""
    sx, sy, sz = size
    areas = [sx * sy, sx * sy, sx * sz, sx * sz, sy * sz, sy * sz]
    total_area = 2 * (sx * sy + sx * sz + sy * sz)
    n_total = int(density * total_area)
    n_per_face = [max(4, int(n_total * a / total_area)) for a in areas]
    points = []
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    for face_idx in range(6):
        n_side = max(2, int(np.sqrt(n_per_face[face_idx])))
        u = np.linspace(-0.5, 0.5, n_side)
        v = np.linspace(-0.5, 0.5, n_side)
        uu, vv = np.meshgrid(u, v)
        uu, vv = uu.ravel(), vv.ravel()
        if face_idx == 0:
            pts = np.column_stack([uu * sx, vv * sy, np.full_like(uu, hz)])
        elif face_idx == 1:
            pts = np.column_stack([uu * sx, vv * sy, np.full_like(uu, -hz)])
        elif face_idx == 2:
            pts = np.column_stack([np.full_like(uu, hx), uu * sy, vv * sz])
        elif face_idx == 3:
            pts = np.column_stack([np.full_like(uu, -hx), uu * sy, vv * sz])
        elif face_idx == 4:
            pts = np.column_stack([uu * sx, np.full_like(uu, hy), vv * sz])
        else:
            pts = np.column_stack([uu * sx, np.full_like(uu, -hy), vv * sz])
        points.append(pts + center)
    return np.vstack(points)


def _sample_sphere_surface(center: np.ndarray, radius: float,
                           n_points: int = 1000) -> np.ndarray:
    """Fibonacci 球面均匀采样。"""
    golden_ratio = (1 + np.sqrt(5)) / 2
    indices = np.arange(n_points)
    theta = 2 * np.pi * indices / golden_ratio
    phi = np.arccos(1 - 2 * (indices + 0.5) / n_points)
    x = radius * np.sin(phi) * np.cos(theta)
    y = radius * np.sin(phi) * np.sin(theta)
    z = radius * np.cos(phi)
    return np.column_stack([x, y, z]) + center


def _sample_cylinder_surface(center: np.ndarray, radius: float, height: float,
                             n_points: int = 1500) -> np.ndarray:
    """圆柱面 + 顶底面均匀采样。"""
    side_area = 2 * np.pi * radius * height
    cap_area = np.pi * radius ** 2
    total_area = side_area + 2 * cap_area
    n_side = max(10, int(n_points * side_area / total_area))
    n_cap = max(4, int(n_points * cap_area / total_area))
    points = []
    hz = height / 2
    n_circ = max(8, int(np.sqrt(n_side * 2 * np.pi * radius / height)))
    n_height = max(4, n_side // n_circ)
    theta = np.linspace(0, 2 * np.pi, n_circ, endpoint=False)
    z_vals = np.linspace(-hz, hz, n_height)
    for z in z_vals:
        for th in theta:
            points.append([radius * np.cos(th), radius * np.sin(th), z])
    np.random.seed(42)
    for _ in range(n_cap):
        r = radius * np.sqrt(np.random.random())
        th = 2 * np.pi * np.random.random()
        points.append([r * np.cos(th), r * np.sin(th), hz])
    for _ in range(n_cap):
        r = radius * np.sqrt(np.random.random())
        th = 2 * np.pi * np.random.random()
        points.append([r * np.cos(th), r * np.sin(th), -hz])
    return np.array(points) + center


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """体素降采样: 每个体素保留质心。"""
    if len(points) == 0:
        return points
    voxel_indices = np.floor(points / voxel_size).astype(np.int32)
    voxel_map: Dict[tuple, list] = {}
    for i, vi in enumerate(voxel_indices):
        key = (vi[0], vi[1], vi[2])
        if key not in voxel_map:
            voxel_map[key] = []
        voxel_map[key].append(points[i])
    return np.array([np.mean(pts, axis=0) for pts in voxel_map.values()])


# ================================================================
# FCL 环境碰撞检测器
# ================================================================

# 从 fcl_collision.py 导入包围体参数
from fcl_collision import (
    JOINT_NAMES, JOINT_RADII, CAPSULE_DEFS,
    BASE_BOX_SIZE, BASE_BOX_CENTER,
    LINK1_BOX_SIZE, LINK1_BOX_CENTER,
    FclSelfCollisionChecker,
)


class FCLPointCloudCollision:
    """基于 FCL 包围体的环境点云碰撞检测。

    复用 fcl_collision.py 的 Box/Sphere/Capsule 基元，
    用 FCL distance API 计算每个包围体到点云的精确距离，
    构建全身 CBF 约束。
    """

    def __init__(self, max_candidates_per_body: int = 5):
        # ---- 创建 FCL 碰撞体 (与 FclSelfCollisionChecker 相同) ----
        self.max_candidates_per_body = max_candidates_per_body
        # Box: base_link
        self._base_box = fcl.CollisionObject(
            fcl.Box(*BASE_BOX_SIZE), fcl.Transform())
        # Box: Link1
        self._link1_box = fcl.CollisionObject(
            fcl.Box(*LINK1_BOX_SIZE), fcl.Transform())
        # Sphere: 关节球体
        self._spheres: Dict[str, fcl.CollisionObject] = {}
        for name in JOINT_NAMES:
            r = JOINT_RADII[name]
            self._spheres[name] = fcl.CollisionObject(
                fcl.Sphere(r), fcl.Transform())
        # Capsule: 连杆胶囊体
        self._capsules: Dict[Tuple[str, str], fcl.CollisionObject] = {}
        self._capsule_lengths: Dict[Tuple[str, str], float] = {}
        for n_from, n_to, radius, length in CAPSULE_DEFS:
            self._capsules[(n_from, n_to)] = fcl.CollisionObject(
                fcl.Capsule(radius, length), fcl.Transform())
            self._capsule_lengths[(n_from, n_to)] = length
        # Pre-created zero-radius sphere for point distance queries (reused per call)
        self._point_sphere = fcl.CollisionObject(fcl.Sphere(0.0), fcl.Transform())
        # Reusable FCL transform for point sphere (avoids allocation per candidate)
        self._pt_tf = fcl.Transform()
        # Cached KDTree for static point clouds
        self._cached_tree = None
        self._cached_tree_key = None
        # Temporal coherence cache: body_name → (point_index, last_distance)
        self._prev_nearest: Dict[str, Tuple[int, float]] = {}
        self._coherence_step = 0
        self._coherence_refresh_interval = 10  # full query every N steps

        self._rotation_align_z = FclSelfCollisionChecker._rotation_align_z

    def update_poses(self, T_all: Dict[str, np.ndarray]):
        """从 forward_kinematics(q) 结果更新所有 FCL 包围体位姿。"""
        # Box: base_link
        T_base = T_all.get("base_link")
        if T_base is not None:
            R_b = T_base[:3, :3]
            t_b = R_b @ np.array(BASE_BOX_CENTER) + T_base[:3, 3]
            tf = fcl.Transform()
            tf.setRotation(R_b)
            tf.setTranslation(t_b)
            self._base_box.setTransform(tf)

        # Box: Link1
        T_l1 = T_all.get("Link1")
        if T_l1 is not None:
            R_l1 = T_l1[:3, :3]
            t_l1 = R_l1 @ np.array(LINK1_BOX_CENTER) + T_l1[:3, 3]
            tf = fcl.Transform()
            tf.setRotation(R_l1)
            tf.setTranslation(t_l1)
            self._link1_box.setTransform(tf)

        # Sphere: 关节球体
        pos: Dict[str, np.ndarray] = {}
        for name in JOINT_NAMES:
            if name not in T_all:
                continue
            p = T_all[name][:3, 3]
            pos[name] = p
            tf = fcl.Transform()
            tf.setTranslation(p)
            self._spheres[name].setTransform(tf)

        # Capsule: 连杆胶囊体
        for (n_from, n_to), obj in self._capsules.items():
            if n_from not in pos or n_to not in pos:
                continue
            p_from = pos[n_from]
            p_to = pos[n_to]
            center = (p_from + p_to) / 2
            diff = p_to - p_from
            direction = diff / np.linalg.norm(diff)
            R_cap = self._rotation_align_z(direction)
            tf = fcl.Transform()
            tf.setRotation(R_cap)
            tf.setTranslation(center)
            obj.setTransform(tf)

    def get_all_bodies(self) -> List[Tuple[str, fcl.CollisionObject, str]]:
        """返回所有 FCL 包围体: [(body_name, fcl_obj, body_type)]。

        body_type: 'box', 'sphere', 'capsule'
        """
        bodies = [("base_box", self._base_box, "box"),
                  ("link1_box", self._link1_box, "box")]
        for name, obj in self._spheres.items():
            bodies.append((f"joint_{name}", obj, "sphere"))
        for (n_from, n_to), obj in self._capsules.items():
            bodies.append((f"capsule_{n_from}_{n_to}", obj, "capsule"))
        return bodies

    def compute_distances_to_points(self, points: np.ndarray,
                                     max_dist: float = 0.3,
                                     use_coherence: bool = True,
                                     ) -> List[Tuple[str, float, np.ndarray, np.ndarray]]:
        """计算每个 FCL 包围体到点云的最近距离。

        参数
        ----
        points : (N, 3) 点云
        max_dist : 最大表面距离 (m)，超过此距离的体不返回
        use_coherence : 启用时间一致性缓存 (跳过未变化的查询)
        返回
        ----
        list of (body_name, distance, nearest_on_body, nearest_on_point)
        """
        from scipy.spatial import cKDTree
        if len(points) == 0:
            return []

        # Cache KDTree for identical point arrays (common for static obstacles)
        tree_key = (points.shape[0], points.ctypes.data)
        if self._cached_tree is None or self._cached_tree_key != tree_key:
            self._cached_tree = cKDTree(points)
            self._cached_tree_key = tree_key
        tree = self._cached_tree
        req = fcl.DistanceRequest()
        results = []

        # Temporal coherence: force full refresh periodically
        do_full_refresh = not use_coherence or (
            self._coherence_step % self._coherence_refresh_interval == 0)
        self._coherence_step += 1

        # 各包围体最大半径 (用于 KDTree 粗筛裕量)
        BODY_MAX_RADIUS = {
            "box": 0.6,      # Box 对角线/2
            "sphere": 0.10,  # 最大球半径
            "capsule": 0.30, # 最大胶囊半径+半高
        }

        new_prev: Dict[str, Tuple[int, float]] = {}

        for body_name, obj, btype in self.get_all_bodies():
            # 获取包围体中心用于 KDTree 查询
            tf = obj.getTransform()
            center = np.array(tf.getTranslation())

            # Temporal coherence: check cached nearest point first
            cached = self._prev_nearest.get(body_name) if use_coherence else None
            if cached is not None and not do_full_refresh:
                cached_idx, cached_dist = cached
                # Quick check: is cached point still valid?
                if cached_idx < len(points):
                    pt = points[cached_idx]
                    self._pt_tf.setTranslation(pt.astype(np.float64))
                    self._point_sphere.setTransform(self._pt_tf)
                    res = fcl.DistanceResult()
                    dist = fcl.distance(obj, self._point_sphere, req, res)
                    if dist >= -0.5 and dist < max_dist:
                        # Cached point still valid — use it
                        new_prev[body_name] = (cached_idx, dist)
                        results.append((
                            body_name, dist,
                            np.array(res.nearest_points[0]),
                            np.array(res.nearest_points[1]),
                        ))
                        continue

            # Full KDTree query
            search_radius = max_dist + BODY_MAX_RADIUS.get(btype, 0.3)
            idxs = tree.query_ball_point(center, search_radius)
            if not idxs:
                continue

            # Per-body candidate cap: keep nearest to center + diversity along long axis
            if len(idxs) > self.max_candidates_per_body:
                pts_subset = points[idxs]
                dists_to_center = np.linalg.norm(pts_subset - center, axis=1)
                sorted_order = np.argsort(dists_to_center)
                k = self.max_candidates_per_body

                if btype == "box":
                    # Large boxes can have nearest surface points far from their center.
                    # Keep all KDTree candidates for boxes; only two box bodies exist.
                    pass
                elif btype == "capsule":
                    # For elongated capsules, also keep points near endpoints
                    # Capsule Z-axis is the long axis
                    R = obj.getTransform().getRotation()
                    rot_mat = np.array([[R[i, j] for j in range(3)] for i in range(3)])
                    z_axis = rot_mat[:, 2]  # long axis direction
                    # Get capsule half-length from stored definitions
                    # body_name format: "capsule_{n_from}_{n_to}"
                    parts = body_name.split("_", 2)
                    cap_key = (parts[1], parts[2]) if len(parts) == 3 else None
                    cap_length = self._capsule_lengths.get(cap_key, 0.3) if cap_key else 0.3
                    half_len = cap_length / 2
                    p0 = center - half_len * z_axis
                    p1 = center + half_len * z_axis
                    # Project candidates onto long axis
                    proj = (pts_subset - center) @ z_axis
                    # Keep: top K/2 nearest to center + top K/4 near each endpoint
                    k_center = k // 2
                    k_end = k - k_center
                    center_sel = sorted_order[:k_center]
                    # For endpoints: sort by distance to p0 and p1
                    dists_p0 = np.linalg.norm(pts_subset - p0, axis=1)
                    dists_p1 = np.linalg.norm(pts_subset - p1, axis=1)
                    end_order_p0 = np.argsort(dists_p0)[:k_end // 2]
                    end_order_p1 = np.argsort(dists_p1)[:k_end - k_end // 2]
                    combined = list(center_sel) + list(end_order_p0) + list(end_order_p1)
                    # Deduplicate while preserving order
                    seen = set()
                    deduped = []
                    for i in combined:
                        if i not in seen:
                            seen.add(i)
                            deduped.append(i)
                    idxs = [idxs[i] for i in deduped[:k]]
                else:
                    idxs = [idxs[i] for i in sorted_order[:k]]

            # 对候选点用 FCL 精确计算距离
            best_dist = float('inf')
            best_pt_body = None
            best_pt_cloud = None
            best_idx = None

            for idx in idxs:
                pt = points[idx]
                self._pt_tf.setTranslation(pt.astype(np.float64))
                self._point_sphere.setTransform(self._pt_tf)
                res = fcl.DistanceResult()
                dist = fcl.distance(obj, self._point_sphere, req, res)

                if dist < -0.5:
                    continue  # FCL bug 过滤
                if dist < best_dist:
                    best_dist = dist
                    best_pt_body = np.array(res.nearest_points[0])
                    best_pt_cloud = np.array(res.nearest_points[1])
                    best_idx = idx

            if best_dist < max_dist and best_pt_body is not None:
                results.append((body_name, best_dist, best_pt_body, best_pt_cloud))
                if best_idx is not None:
                    new_prev[body_name] = (best_idx, best_dist)

        self._prev_nearest = new_prev
        return results

    def get_cbf_constraints(self, q: np.ndarray, kin,
                             points: np.ndarray,
                             alpha: float = 10.0,
                             d_safe: float = 0.03,
                             activation: float = 0.15,
                             body_link_idx: Optional[Dict[str, int]] = None,
                             T_all=None,
                             J_s=None,
                             use_coherence: bool = True,
                             ) -> list:
        """生成全身 FCL 包围体 vs 点云的 CBF 约束。

        参数
        ----
        q : (9,) 关节角
        kin : NineaxisKinematics 实例
        points : (N, 3) 点云
        alpha : CBF 增益
        d_safe : 安全距离
        activation : 激活距离
        body_link_idx : 碰撞体名→link_idx 映射 (用于 point_jacobian)
        返回
        ----
        list of CbfConstraint
        """
        from dynamic_obstacles import CbfConstraint

        if body_link_idx is None:
            body_link_idx = _BODY_LINK_IDX

        # 更新 FCL 包围体位姿
        if T_all is None:
            T_all = kin.forward_kinematics(q)
        if J_s is None:
            J_s = kin.compute_spatial_jacobian_world(q)
        self.update_poses(T_all)

        # 计算距离
        dist_results = self.compute_distances_to_points(
            points, max_dist=activation + 0.1, use_coherence=use_coherence)

        constraints = []
        for body_name, dist, pt_body, pt_cloud in dist_results:
            h_val = dist - d_safe
            if h_val > activation:
                continue

            # 方向: 包围体→点云 (从机器人指向障碍物)
            diff = pt_cloud - pt_body
            norm_diff = np.linalg.norm(diff)
            if norm_diff < 1e-10:
                continue
            normal = diff / norm_diff

            # link_idx 用于 point_jacobian
            link_idx = body_link_idx.get(body_name, 0)

            # 点雅可比 (3×9)
            J_point = point_jacobian_from_spatial(J_s, link_idx, pt_body)

            # CBF 梯度: G_row = -dh/dq = normal @ J_point (normal 指向障碍物→身体)
            G_row = normal @ J_point  # (9,)

            constraints.append(CbfConstraint(
                name=f"env_{body_name}",
                G_row=G_row,
                h_bound=alpha * max(h_val, -0.5),
                h_value=h_val,
                active=True,
            ))

        return constraints


# 碰撞体名 → link_idx 映射 (用于 point_jacobian)
# link_idx = 影响该碰撞体的上游活动关节数
_BODY_LINK_IDX = {
    "base_box": 0,
    "link1_box": 1,
    "joint_Link2": 2,
    "joint_Link3": 3,
    "joint_Link4": 4,
    "joint_Link5": 5,
    "joint_Link7": 7,
    "joint_Link8": 8,
    "joint_Link9": 9,
    "joint_ee_link": 9,
    "capsule_Link2_Link3": 2,
    "capsule_Link3_Link4": 3,
    "capsule_Link4_Link5": 4,
    "capsule_Link5_Link7": 7,  # 中点受 J6+J7 影响, 需包含 joints 0-6 (P1-11)
    "capsule_Link7_Link8": 7,
    "capsule_Link8_Link9": 8,
    "capsule_Link9_ee_link": 9,
}


# ================================================================
# 点云管理器 (采样 + 存储 + 可视化数据)
# ================================================================

class PointCloudObstacleManager:
    """点云环境障碍物管理器。

    负责:
      1. 从几何基元生成点云
      2. 体素降采样
      3. 持有点云数据供可视化和碰撞检测
    """

    def __init__(self, points: np.ndarray, voxel_size: float = 0.02):
        self.raw_points = points.copy()
        self.voxel_size = voxel_size
        self.points = voxel_downsample(points, voxel_size) if voxel_size > 0 else points.copy()
        self.n_points = len(self.points)

        # FCL 碰撞检测器
        self.fcl_collision = FCLPointCloudCollision()

    @classmethod
    def from_obstacle_specs(cls, specs: list, voxel_size: float = 0.02,
                            seed: int = 42) -> 'PointCloudObstacleManager':
        """从几何基元规格列表生成点云。"""
        np.random.seed(seed)
        all_points = []
        for spec in specs:
            center = np.array(spec['center'])
            obs_type = spec['type']
            if obs_type == 'box':
                pts = _sample_box_surface(center, np.array(spec['size']),
                                          spec.get('density', 5000.0))
            elif obs_type == 'sphere':
                pts = _sample_sphere_surface(center, spec['radius'],
                                             spec.get('n_points', 1000))
            elif obs_type == 'cylinder':
                pts = _sample_cylinder_surface(center, spec['radius'], spec['height'],
                                               spec.get('n_points', 1500))
            else:
                raise ValueError(f"未知障碍物类型: {obs_type}")
            all_points.append(pts)
        combined = np.vstack(all_points) if all_points else np.empty((0, 3))
        return cls(combined, voxel_size)

    def get_cbf_constraints(self, q, kin, alpha=10.0, d_safe=0.03,
                            activation=0.15, T_all=None, J_s=None,
                            use_coherence: bool = True) -> list:
        """生成全身 FCL 包围体 vs 点云的 CBF 约束。"""
        return self.fcl_collision.get_cbf_constraints(
            q, kin, self.points, alpha, d_safe, activation,
            T_all=T_all, J_s=J_s, use_coherence=use_coherence)

    def get_obstacle_points(self) -> np.ndarray:
        """返回降采样后的点云 (N, 3)。"""
        return self.points
