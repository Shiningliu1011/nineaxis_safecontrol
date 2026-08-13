#!/usr/bin/env python3
"""
traj_to_base_transform.py
==========================
计算轨迹坐标系到世界坐标系的齐次仿射变换矩阵 T_traj_to_world。

核心目标:
  将 ik_input.mat 中的 NURBS 轨迹变换到世界系, 并使:
    desired_traj[0] == current_ee_position
  即轨迹起点严格对齐当前末端执行器原点, 机械臂末端沿轨迹运动。

对齐关系 (旋转):
  轨迹系 X⁺  ∥  世界 Z⁺
  轨迹系 Y⁺  ∥  世界 X⁺
  轨迹系 Z⁺  ∥  世界 Y⁺

变换方案:
  T[:3, :3] = scale * R_traj                                    (旋转+等比缩放)
  T[:3, 3]  = ee_start - scale * (R_traj @ raw_traj_first)      (起点对齐EE)
  → 变换后 desired_traj[0] ≡ ee_start (精确重合)

不可达性处理:
  原始 NURBS 轨迹经刚体变换后跨度 ~1.6m, 九轴机械臂可达包络有限,
  J1 棱柱行程仅 0.58m → 必须等比缩放使轨迹落入可达工作空间。
"""

import os
import numpy as np
from nineaxis_kinematics import NineaxisKinematics


# ================================================================
# 放置参数 (可调)
# ================================================================
DEFAULT_SCALE = 0.22
# 缩放系数: rpy=(-π/2,0,0), 轴映射 X→X, Y→Y, Z→Z
# 原始 Z(0.961m) → 世界 Z(J1方向), scale=0.22 → Z跨度 ~0.21m
# EE Z 起点=1.152, Z_max≈1.152+0.21=1.36 < 1.64(工作空间上限)

# 默认 EE 起始位置 (仅在未传入 ee_start 时使用)
# q=0 EE≈[0.343, 0, 1.152] m (世界系)
DEFAULT_EE_START = np.array([0.343, 0.0, 1.152])

# J1 中位 (用于计算 Link9 参考位姿, 仅影响 T_world_9 返回值)
J1_MID = 0.29  # J1 范围 [0, 0.58] 的中点


def _load_raw_first_point(mat_path: str) -> np.ndarray:
    """加载 ik_input.mat, 返回轨迹系下第一个轨迹点 (m)"""
    import scipy.io as sio
    mat = sio.loadmat(mat_path)
    data = mat['ik_input'][0, 0]
    raw_pos = data['position_series'] / 1000.0   # N×3, mm → m
    return raw_pos[0].copy()  # 第一个点


def compute_traj_to_world(mat_path: str = None,
                          scale: float = DEFAULT_SCALE,
                          ee_start: np.ndarray = None):
    """
    计算轨迹坐标系到世界坐标系的齐次仿射变换矩阵 (含等比缩放)。

    关键: 变换后 desired_traj[0] == ee_start (轨迹起点精确对齐EE原点)。

    参数
    ----
    mat_path : str
        ik_input.mat 路径。None 时回退到默认路径。
    scale : float
        轨迹等比缩放系数。
    ee_start : (3,) np.ndarray or None
        当前机械臂末端执行器原点在世界系下的位置 (m)。
        None 时使用 DEFAULT_EE_START。

    返回
    ----
    T_traj_to_world : (4,4) 仿射变换 (旋转块含缩放)
    T_world_9 : (4,4) Link9 在 J1=J1_MID 时的世界位姿 (参考用)
    T_frames : dict 全连杆世界位姿
    first_point_world : (3,) 变换后第一个轨迹点的世界坐标 (应 == ee_start)
    """
    kin = NineaxisKinematics()

    if ee_start is None:
        ee_start = DEFAULT_EE_START

    # ---- J1=中位 时的 Link9 参考位姿 (仅用于 T_world_9 返回) ----
    q_mid = np.zeros(9)
    q_mid[0] = J1_MID
    T_frames = kin.forward_kinematics(q_mid)
    T_world_9 = T_frames["Link9"]

    # ---- 旋转: 轨迹系 → 世界系 轴映射 ----
    # world = base_link (无旋转)
    # 轨迹直接映射: X→X, Y→Y, Z→Z
    X_T_in_world = np.array([1.0, 0.0, 0.0])
    Y_T_in_world = np.array([0.0, 1.0, 0.0])
    Z_T_in_world = np.array([0.0, 0.0, 1.0])
    R_traj = np.column_stack([X_T_in_world, Y_T_in_world, Z_T_in_world])

    # ---- 平移: 使轨迹第一个点对齐到 EE 原点 ----
    if mat_path is None:
        here = os.path.dirname(os.path.abspath(__file__))
        mat_path = os.path.join(here, '..', 'nurbs_data', 'ik_input.mat')
    raw_first = _load_raw_first_point(mat_path)
    rotated_first = R_traj @ raw_first
    # 平移量: ee_start = scale * rotated_first + t → t = ee_start - scale * rotated_first
    t_traj = ee_start - scale * rotated_first

    # ---- 组装仿射变换 (旋转块含缩放) ----
    T_traj_to_world = np.eye(4)
    T_traj_to_world[:3, :3] = scale * R_traj
    T_traj_to_world[:3, 3] = t_traj

    # 验证: 变换后第一个点是否等于 ee_start
    first_point_world = T_traj_to_world[:3, :3] @ raw_first + T_traj_to_world[:3, 3]

    return T_traj_to_world, T_world_9, T_frames, first_point_world


# 向后兼容别名
def compute_traj_to_base(mat_path: str = None,
                         scale: float = DEFAULT_SCALE,
                         place_target: np.ndarray = None,
                         ee_start: np.ndarray = None):
    """
    向后兼容接口。优先使用 ee_start (精确对齐), 否则回退到 place_target (质心对齐)。

    P2-40: 旋转映射说明:
      - ee_start 路径 → compute_traj_to_world (恒等映射, 当前 Y-up 坐标系)
      - place_target 回退路径 → 遗留置换映射 (X→Z, Y→X, Z→Y), 仅用于无 ee_start
        的旧调用路径。主运行器 (run_oscbf_rviz_newaxis) 使用 _compute_traj_transform
        自行计算变换, 恒等映射与 URDF Y-up 一致。
    """
    if ee_start is not None:
        T, T9, frames, fp = compute_traj_to_world(mat_path, scale, ee_start)
        return T, T9, frames
    # 回退: 使用质心对齐 (遗留置换映射, 仅向后兼容)
    if place_target is None:
        place_target = np.array([0.40, 0.0, 1.15])
    if mat_path is None:
        here = os.path.dirname(os.path.abspath(__file__))
        mat_path = os.path.join(here, '..', 'nurbs_data', 'ik_input.mat')

    import scipy.io as sio
    kin = NineaxisKinematics()
    q_mid = np.zeros(9); q_mid[0] = J1_MID
    T_frames = kin.forward_kinematics(q_mid)
    T_world_9 = T_frames["Link9"]

    # 遗留旋转映射: 轨迹系 (Z-up) → 旧 world 系
    X_T = np.array([0.0, 0.0, 1.0])
    Y_T = np.array([1.0, 0.0, 0.0])
    Z_T = np.array([0.0, 1.0, 0.0])
    R_traj = np.column_stack([X_T, Y_T, Z_T])

    mat = sio.loadmat(mat_path)
    data = mat['ik_input'][0, 0]
    raw_pos = data['position_series'] / 1000.0
    raw_centroid = raw_pos.mean(axis=0)
    rotated_centroid = R_traj @ raw_centroid
    t_traj = place_target - scale * rotated_centroid

    T_traj_to_base = np.eye(4)
    T_traj_to_base[:3, :3] = scale * R_traj
    T_traj_to_base[:3, 3] = t_traj
    return T_traj_to_base, T_world_9, T_frames


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True, linewidth=120)

    here = os.path.dirname(os.path.abspath(__file__))
    mat_path = os.path.join(here, '..', 'nurbs_data', 'ik_input.mat')

    # 使用默认 EE 起点
    T, T_world_9, _, first_pt = compute_traj_to_world(mat_path)

    print("=" * 72)
    print("轨迹坐标系 → 世界坐标系 (起点对齐 EE 原点)")
    print("=" * 72)
    print(f"  scale = {DEFAULT_SCALE}")
    print(f"  ee_start (默认) = {DEFAULT_EE_START} m")
    print(f"  first_point_world = {first_pt} m")
    print(f"  起点误差 = {np.linalg.norm(first_pt - DEFAULT_EE_START)*1000:.3f} mm")
    print(f"  轨迹 X⁺ ∥ 世界 Z⁺,  Y⁺ ∥ 世界 X⁺,  Z⁺ ∥ 世界 Y⁺")

    print(f"\n齐次仿射变换 T_traj_to_world (旋转块含缩放 {DEFAULT_SCALE}):")
    for i in range(4):
        print("  [" + ", ".join(f"{T[i,j]:12.6f}" for j in range(4)) + "]")
