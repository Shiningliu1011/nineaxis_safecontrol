#!/usr/bin/env python3
"""
fcl_collision.py
================
基于 python-fcl 基元的自碰撞检测器。

- base_link: Box 包围盒
- Link1: Box 包围盒 (沿 Link1→Link2 方向)
- Link2~Link9: 关节球体 + 连杆胶囊体

依赖: python-fcl, trimesh, numpy
"""

import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import fcl
import numpy as np
import trimesh


@dataclass
class FclCollisionPair:
    name_i: str
    name_j: str
    distance: float
    pt_i: np.ndarray
    pt_j: np.ndarray
    normal: np.ndarray
    # dpax 平滑梯度 (仅 dpax 后端填充, FCL 为 None)
    _dpax_grad: Optional[np.ndarray] = field(default=None, repr=False)
    # dpax CBF 约束边界 h_bound (仅 dpax 后端填充)
    _dpax_h_bound: Optional[float] = field(default=None, repr=False)


# ================================================================
# 关节名 (球体, 去掉 base_link 和 Link1, 它们用 Box)
# ================================================================
JOINT_NAMES = [
    "Link2",      # 0
    "Link3",      # 1
    "Link4",      # 2
    "Link5",      # 3 (=Link6)
    "Link7",      # 4
    "Link8",      # 5
    "Link9",      # 6
    "ee_link",    # 7
]

# 关节球体半径 (覆盖关节机构, 比胶囊体大)
JOINT_RADII = {
    "Link2":     0.080,
    "Link3":     0.075,
    "Link4":     0.075,
    "Link5":     0.070,
    "Link7":     0.065,
    "Link8":     0.060,
    "Link9":     0.055,
    "ee_link":   0.040,
}

# ================================================================
# 连杆胶囊体 (半径比球体小, 仅覆盖连杆轴)
# 注意: FCL 对 capsule+sphere 组合半径 >0.135 时返回 -1.0
# 因此胶囊半径需满足: capsule_r + sphere_r < 0.135
# ================================================================
CAPSULE_DEFS = [
    # (from_joint, to_joint, radius, length)
    ("Link2",     "Link3",   0.065, 0.225),  # Link3 sphere=0.075, 0.065+0.075=0.140>0.135, 需减小
    ("Link3",     "Link4",   0.055, 0.225),  # OK
    ("Link4",     "Link5",   0.065, 0.343),  # Link7 sphere=0.065, 0.065+0.065=0.130<0.135 ✓
    ("Link5",     "Link7",   0.065, 0.135),  # Link8 sphere=0.060, 0.065+0.060=0.125<0.135 ✓
    ("Link7",     "Link8",   0.040, 0.110),  # OK
    ("Link8",     "Link9",   0.060, 0.114),  # OK
    ("Link9",     "ee_link", 0.025, 0.235),  # OK
]

# 基元包络的已标定拓扑豁免。
# Link5-Link7 与 Link8-Link9 之间隔着固定的 Link7-Link8（110 mm），但
# 两个保守胶囊的半径和为 125 mm，因此无论构型如何都会在该连接端虚假
# 重叠。FCL 网格验证在零位及 oscbf-jax 过渡回放中均显示 Link5-Link8
# 实体没有接触（零位净空约 144 mm）。该对不能成为硬 CBF，否则会制造
# 永久不可行约束；网格验证后端仍保留真实 Link5-Link8 碰撞检查。
# 更换 URDF、网格或上述半径后必须重新标定，不能把此列表当作通用豁免。
CALIBRATED_PRIMITIVE_PAIR_EXCLUSIONS = frozenset({
    frozenset(("capsule_Link5_Link7", "capsule_Link8_Link9")),
    frozenset(("capsule_Link4_Link5", "joint_Link7")),  # 机械近邻, 零位5mm为基元保守近似
})

# ================================================================
# base_link Box (覆盖整个底座: z=-0.335 ~ z=0.562)
# ================================================================
BASE_BOX_SIZE = (0.160, 0.140, 0.900)  # (sx, sy, sz)
BASE_BOX_CENTER = (0.000, 0.006, 0.113)  # 局部坐标系中心

# ================================================================
# Link1 Box (沿 Link1→Link2 方向, 覆盖 mesh: y=-0.024~0.401, z=-0.185~0.062)
# ================================================================
LINK1_BOX_SIZE = (0.160, 0.430, 0.260)  # (sx, sy, sz)
LINK1_BOX_CENTER = (0.000, 0.189, -0.061)  # 局部坐标系中心


class FclSelfCollisionChecker:
    """FCL 自碰撞检查器: Box(base,Link1) + Sphere+Capsule(Link2~Link9)"""

    def __init__(self, mesh_dir: str):
        self._request = fcl.DistanceRequest()
        self._request.enable_nearest_points = True

        # ---- Box: base_link ----
        self._base_box = fcl.CollisionObject(fcl.Box(*BASE_BOX_SIZE))

        # ---- Box: Link1 ----
        self._link1_box = fcl.CollisionObject(fcl.Box(*LINK1_BOX_SIZE))

        # ---- 球体: 关节 ----
        self._spheres: Dict[str, fcl.CollisionObject] = {}
        for name in JOINT_NAMES:
            self._spheres[name] = fcl.CollisionObject(fcl.Sphere(JOINT_RADII[name]))

        # ---- 胶囊体: 连杆 ----
        self._capsules: Dict[Tuple[str, str], fcl.CollisionObject] = {}
        for n_from, n_to, radius, length in CAPSULE_DEFS:
            self._capsules[(n_from, n_to)] = fcl.CollisionObject(fcl.Capsule(radius, length))

        # ---- 碰撞对 (排除相邻) ----
        self._build_collision_pairs()

        # ---- 距离缓存 (自碰撞优化) ----
        self._prev_joint_positions: Dict[str, np.ndarray] = {}
        self._prev_pair_distances: Dict[Tuple[str, str], FclCollisionPair] = {}
        self._cache_move_threshold = 0.0005  # 0.5mm — 小于此值复用上一步距离

        # ---- 凸包 (可视化) ----
        self.mesh_vis: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        for name in ['base_link','Link1','Link2','Link3','Link4','Link5','Link6','Link7','Link8','Link9']:
            stl_path = os.path.join(mesh_dir, f"{name}.STL")
            if os.path.exists(stl_path):
                mesh = trimesh.load(stl_path)
                hull = mesh.convex_hull
                self.mesh_vis[name] = (hull.vertices.copy(), hull.faces.copy())

    def _build_collision_pairs(self):
        """构建碰撞对列表, 按关节分组, 相邻组跳过

        分组:
          0: base_box
          1: link1_box + joint_Link2 + capsule_Link2_Link3
          2: joint_Link3 + capsule_Link3_Link4
          3: joint_Link4 + capsule_Link4_Link5
          4: joint_Link5 + capsule_Link5_Link7
          5: joint_Link7 + capsule_Link7_Link8
          6: joint_Link8 + capsule_Link8_Link9
          7: joint_Link9 + capsule_Link9_ee_link
          8: joint_ee_link
        """
        # 每组的碰撞体名
        groups = [
            ["base_box"],                                          # 0
            ["link1_box", "joint_Link2", "capsule_Link2_Link3"],   # 1
            ["joint_Link3", "capsule_Link3_Link4"],                # 2
            ["joint_Link4", "capsule_Link4_Link5"],                # 3
            ["joint_Link5", "capsule_Link5_Link7"],                # 4
            ["joint_Link7", "capsule_Link7_Link8"],                # 5
            ["joint_Link8", "capsule_Link8_Link9"],                # 6
            ["joint_Link9", "capsule_Link9_ee_link"],              # 7
            ["joint_ee_link"],                                     # 8
        ]

        self._check_pairs: List[Tuple[str, str, object, object]] = []
        # (name_i, name_j, obj_i, obj_j) - obj 在 check() 中动态填入

        # 只检查距离较近的组 (|gi-gj| <= 3)，跳过远距离对
        max_group_gap = 3
        for gi in range(len(groups)):
            for gj in range(gi + 2, min(gi + max_group_gap + 1, len(groups))):
                for bi in groups[gi]:
                    for bj in groups[gj]:
                        if frozenset((bi, bj)) in CALIBRATED_PRIMITIVE_PAIR_EXCLUSIONS:
                            continue
                        self._check_pairs.append((bi, bj, None, None))

    @staticmethod
    def _rotation_align_z(direction: np.ndarray) -> np.ndarray:
        """旋转矩阵: Z 轴 → direction (优化版)"""
        # 归一化方向向量
        norm_sq = direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]
        if norm_sq < 1e-20:
            return np.eye(3)
        inv_norm = 1.0 / np.sqrt(norm_sq)
        dx = direction[0] * inv_norm
        dy = direction[1] * inv_norm
        dz = direction[2] * inv_norm

        # cos_a = dot(z, d) = dz
        cos_a = np.clip(dz, -1.0, 1.0)

        if cos_a > 0.999:
            return np.eye(3)
        if cos_a < -0.999:
            return np.array([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])

        # axis = cross(z, d) = [-dy, dx, 0]
        axis_norm_sq = dx*dx + dy*dy
        if axis_norm_sq < 1e-20:
            return np.eye(3)
        inv_axis_norm = 1.0 / np.sqrt(axis_norm_sq)
        ax = -dy * inv_axis_norm
        ay = dx * inv_axis_norm
        az = 0.0

        angle = np.arccos(cos_a)
        s = np.sin(angle)
        c = 1.0 - np.cos(angle)

        # K = [[0, -az, ay], [az, 0, -ax], [-ay, ax, 0]]
        # R = I + s*K + c*K@K
        K = np.array([[0.0, -az, ay],
                      [az, 0.0, -ax],
                      [-ay, ax, 0.0]])
        return np.eye(3) + s * K + c * (K @ K)

    def _get_obj(self, name: str):
        """从碰撞体名获取 FCL 对象"""
        if name == "base_box":
            return self._base_box
        if name == "link1_box":
            return self._link1_box
        if name.startswith("joint_"):
            jname = name[6:]  # "joint_Link2" -> "Link2"
            return self._spheres.get(jname)
        if name.startswith("capsule_"):
            parts = name.split("_")  # "capsule_Link2_Link3" -> ["capsule","Link2","Link3"]
            key = (parts[1], parts[2])
            return self._capsules.get(key)
        return None

    def check(self, transforms: Dict[str, np.ndarray],
              activation_dist: float = 0.1) -> List[FclCollisionPair]:
        """
        自碰撞检测: Box(base,Link1) + Sphere+Capsule(Link2~ee)
        按关节分组, 相邻组跳过。

        参数
        ----
        transforms : dict
            forward_kinematics(q) 返回的 {link_name: 4×4 齐次变换}
        activation_dist : float
            仅返回距离 < activation_dist 的碰撞对
        """
        # ---- 更新 Box: base_link ----
        T_base = transforms.get("base_link")
        if T_base is not None:
            R_b = T_base[:3, :3]
            t_b = R_b @ np.array(BASE_BOX_CENTER) + T_base[:3, 3]
            tf = fcl.Transform()
            tf.setRotation(R_b)
            tf.setTranslation(t_b)
            self._base_box.setTransform(tf)

        # ---- 更新 Box: Link1 ----
        T_l1 = transforms.get("Link1")
        if T_l1 is not None:
            R_l1 = T_l1[:3, :3]
            t_l1 = R_l1 @ np.array(LINK1_BOX_CENTER) + T_l1[:3, 3]
            tf = fcl.Transform()
            tf.setRotation(R_l1)
            tf.setTranslation(t_l1)
            self._link1_box.setTransform(tf)

        # ---- 更新球体 ----
        pos: Dict[str, np.ndarray] = {}
        for name in JOINT_NAMES:
            if name not in transforms:
                continue
            p = transforms[name][:3, 3]
            pos[name] = p
            tf = fcl.Transform()
            tf.setTranslation(p)
            self._spheres[name].setTransform(tf)

        # ---- 更新胶囊体 ----
        for (n_from, n_to), obj in self._capsules.items():
            if n_from not in pos or n_to not in pos:
                continue
            p_from, p_to = pos[n_from], pos[n_to]
            center = (p_from + p_to) / 2
            diff = p_to - p_from
            direction = diff / np.linalg.norm(diff)
            R_cap = self._rotation_align_z(direction)
            tf = fcl.Transform()
            tf.setRotation(R_cap)
            tf.setTranslation(center)
            obj.setTransform(tf)

        # ---- 检查关节位移, 标记可用缓存的碰撞对 ----
        cur_positions: Dict[str, np.ndarray] = dict(pos)
        # 也记录 box 中心
        if "base_box" in [n for n, *_ in self._check_pairs]:
            T_base = transforms.get("base_link")
            if T_base is not None:
                cur_positions["base_box"] = R_b @ np.array(BASE_BOX_CENTER) + T_base[:3, 3]
        if "link1_box" in [n for n, *_ in self._check_pairs]:
            T_l1 = transforms.get("Link1")
            if T_l1 is not None:
                cur_positions["link1_box"] = R_l1 @ np.array(LINK1_BOX_CENTER) + T_l1[:3, 3]

        # 判断每对是否可复用: 两端都移动 < threshold
        thresh = self._cache_move_threshold
        can_reuse: Dict[Tuple[str, str], bool] = {}
        for name_i, name_j, _, _ in self._check_pairs:
            pi_prev = self._prev_joint_positions.get(name_i)
            pj_prev = self._prev_joint_positions.get(name_j)
            pi_cur = cur_positions.get(name_i)
            pj_cur = cur_positions.get(name_j)
            if (pi_prev is not None and pj_prev is not None and
                pi_cur is not None and pj_cur is not None):
                di = np.linalg.norm(pi_cur - pi_prev)
                dj = np.linalg.norm(pj_cur - pj_prev)
                can_reuse[(name_i, name_j)] = (di < thresh and dj < thresh)
            else:
                can_reuse[(name_i, name_j)] = False

        # ---- 遍历碰撞对 ----
        results = []
        req = self._request
        new_pair_cache: Dict[Tuple[str, str], FclCollisionPair] = {}

        for name_i, name_j, _, _ in self._check_pairs:
            # 尝试复用缓存
            if can_reuse.get((name_i, name_j), False):
                cached = self._prev_pair_distances.get((name_i, name_j))
                if cached is not None:
                    results.append(cached)
                    new_pair_cache[(name_i, name_j)] = cached
                    continue

            obj_i = self._get_obj(name_i)
            obj_j = self._get_obj(name_j)
            if obj_i is None or obj_j is None:
                continue

            dr = fcl.DistanceResult()
            fcl.distance(obj_i, obj_j, req, dr)
            dist = dr.min_distance
            pt_i, pt_j = dr.nearest_points

            # FCL bug 修复: 用最近点实际距离校验
            # 1. 哨兵值 -1.0 直接丢弃
            if dist < -0.5:
                continue
            # 2. 负距离时, 检查最近点距离是否合理 (差值 > 5mm 视为 bug)
            if dist < 0:
                actual_dist = np.linalg.norm(pt_j - pt_i)
                if abs(dist - actual_dist) > 0.005:
                    continue  # FCL 返回值不可信, 跳过

            if dist < activation_dist:
                diff = pt_j - pt_i
                n = np.linalg.norm(diff)
                normal = diff / n if n > 1e-12 else np.array([1.0, 0, 0])
                pair = FclCollisionPair(
                    name_i, name_j, dist, pt_i, pt_j, normal)
                results.append(pair)
                new_pair_cache[(name_i, name_j)] = pair

        # 更新缓存
        self._prev_joint_positions = cur_positions
        self._prev_pair_distances = new_pair_cache

        return results


# ================================================================
# 测试
# ================================================================
if __name__ == "__main__":
    import time, sys
    sys.path.insert(0, os.path.dirname(__file__))
    from nineaxis_kinematics import NineaxisKinematics

    mesh_dir = os.path.join(os.path.dirname(__file__),
                            '..', 'assets', 'ninezzhouURDF', 'meshes')
    checker = FclSelfCollisionChecker(mesh_dir)
    kin = NineaxisKinematics()

    print(f"碰撞体: 2 Box + {len(checker._spheres)} Sphere + {len(checker._capsules)} Capsule")

    q = np.zeros(9)
    T_all = kin.forward_kinematics(q)
    t0 = time.perf_counter()
    for _ in range(1000):
        pairs = checker.check(T_all, activation_dist=0.10)
    dt = (time.perf_counter() - t0) / 1000 * 1000
    print(f"\n零位: {len(pairs)} 对, {dt:.3f} ms/call")
    for p in pairs:
        print(f"  {p.name_i} - {p.name_j}: dist={p.distance*1000:.1f}mm")
