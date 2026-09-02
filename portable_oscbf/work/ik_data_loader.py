#!/usr/bin/env python3
"""
ik_data_loader.py
=================
加载 ik_input.mat 全部数据，统一转换到世界坐标系 (单位: 米, m/s, m/s², m/s³)，
并生成可选的末端 6-DOF 姿态参考。

依赖: numpy, scipy
"""

import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


def reference_trajectory_transform(
        mat_path: str,
        align_rotation: np.ndarray,
        ee_center: np.ndarray,
        kinematics: Optional["NineaxisKinematics"] = None,
        max_span_fraction: float = 0.6) -> np.ndarray:
    """Build the reference trajectory-to-base transform (M4/M5 shared helper).

    Mirrors the reference runner's ``_compute_traj_transform``: rotate the raw
    trajectory by ``align_rotation``, scale it so its largest span covers 60%
    of the J1 prismatic stroke, and align its centroid to ``ee_center``.  The
    simple translation used by the ROS display path is intentionally NOT used
    here because it places the path start outside the reachable workspace.

    Returns a 4x4 homogeneous transform ``T_traj_to_base`` (uniform scale in
    the rotation block, matching ``IKTrajectoryData``'s scale handling).
    """

    import scipy.io as sio

    if kinematics is None:
        from work.nineaxis_kinematics import NineaxisKinematics

        kinematics = NineaxisKinematics()
    raw = np.asarray(sio.loadmat(mat_path)["ik_input"][0, 0]["position_series"],
                     dtype=float) / 1000.0
    rotation = np.asarray(align_rotation, dtype=float)
    rotated = (rotation @ raw.T).T

    ee_min = kinematics.ee_position(np.zeros(9))
    q_j1_max = np.zeros(9)
    q_j1_max[0] = kinematics.joint_limits.q_max[0]
    ee_max = kinematics.ee_position(q_j1_max)
    ws_z_extent = ee_max[2] - ee_min[2]
    max_extent = max(
        rotated[:, axis].max() - rotated[:, axis].min() for axis in range(3))
    scale = ws_z_extent / max_extent * max_span_fraction

    centroid = scale * rotated.mean(axis=0)
    translation = np.asarray(ee_center, dtype=float) - centroid

    transform = np.eye(4)
    transform[:3, :3] = scale * rotation
    transform[:3, 3] = translation
    return transform


def load_repository_trajectory(
        mat_path: str,
        config_yaml_path: Optional[str] = None,
        feedrate_scale: float = 1.0) -> "IKTrajectoryData":
    """Load the repository trajectory with the reference calibration.

    Uses ``config/nineaxis.yaml`` kinematics (align rotation, ee_center and
    fixed orientation) so the path lies inside the reachable workspace, then
    selects the fixed orientation mode.  Shared by M4/M5 tests and the ROS
    integration layer.
    """

    import yaml

    if config_yaml_path is None:
        config_yaml_path = str(
            Path(__file__).resolve().parents[1] / "config" / "nineaxis.yaml")
    with open(config_yaml_path, encoding="utf-8") as stream:
        kinematics_config = yaml.safe_load(stream)["kinematics"]
    transform = reference_trajectory_transform(
        mat_path,
        np.asarray(kinematics_config["trajectory_align_rotation"], dtype=float),
        np.asarray(kinematics_config["ee_center"], dtype=float),
    )
    data = IKTrajectoryData(mat_path, transform, feedrate_scale=feedrate_scale)
    data.set_orientation_mode(
        "fixed", fixed_R=np.asarray(
            kinematics_config["fixed_orientation"], dtype=float))
    return data


@dataclass
class TaskReference:
    """6-DOF 任务参考 (操作空间)"""
    t: float
    pos: np.ndarray      # 3D 期望位置 (m)
    vel: np.ndarray      # 3D 期望线速度 (m/s)
    accel: np.ndarray    # 3D 期望线加速度 (m/s²)
    R_des: np.ndarray    # 3×3 期望姿态矩阵
    omega: np.ndarray    # 3D 期望角速度 (rad/s)


class IKTrajectoryData:
    """加载并管理 ik_input.mat 全部数据，提供世界系下的采样接口"""

    ORIENTATION_FIXED = "fixed"
    ORIENTATION_TOOL_FORWARD_TANGENT_LIMITED = "tool_forward_tangent_limited"
    # 旧名称曾把平面蝴蝶曲线的“X 轴向下”约定写进接口名。实际算法始终
    # 保持传入 fixed_R 的 X 轴；保留这个别名只是为了不破坏历史命令。
    LEGACY_TOOL_DOWN_TANGENT_LIMITED = "tool_down_tangent_limited"
    TOOL_TANGENT_LIMITED_RATE = 0.6  # rad/s, 工具轴角速率限幅 (0.2 时被圆柱曲率压到 ~0.04 m/s, 跟踪过慢)
    TOOL_TANGENT_ACCEL_LIMIT = 10.0  # rad/s^2, 平滑 omega feedforward

    def __init__(self, mat_path: str, T_traj_to_base: np.ndarray,
                 feedrate_scale: float = 1.0):
        """
        参数
        ----
        mat_path : str
            ik_input.mat 文件路径。
        T_traj_to_base : np.ndarray (4×4)
            轨迹坐标系 → 基坐标系的齐次变换矩阵。
        feedrate_scale : float
            参考进给缩放。源数据 feedrate_cmd_series 按旧 tool-axis 限速
            生成, 常把整条路径压到 ~0.02-0.04 m/s; 运行时放大可整体提速,
            rate/joint 等物理限速 (cap) 仍会兜底。
        """
        import scipy.io as sio
        mat = sio.loadmat(mat_path)
        data = mat['ik_input'][0, 0]

        self.Ts = float(data['Ts'][0, 0])           # 采样周期 (s)
        self.num_points = int(data['num_points'][0, 0])
        self.num_blocks = int(data['num_blocks'][0, 0])
        self.T_traj_to_base = T_traj_to_base
        self.R_traj = T_traj_to_base[:3, :3]        # 旋转部分 (含等比缩放)
        self.t_traj = T_traj_to_base[:3, 3]         # 平移部分 (m)
        # 等比缩放因子 (T 旋转块 = scale·R_traj, R_traj 正交故列范数 = scale)。
        # 向量 (_vel/_acc/_jerk world) 经 _transform_vectors 已乘 scale;
        # 标量须同乘 scale 以保持量纲一致 (供 dynamic_obstacles 的 d_safe/radius 使用)。
        self.scale = float(np.linalg.norm(T_traj_to_base[:3, 0]))

        # ---- 加载原始数据 (轨迹坐标系, mm 单位) ----
        self._raw_pos = data['position_series']         # N×3, mm
        self._raw_vel = data['velocity_series']         # N×3, mm/s
        self._raw_acc = data['acceleration_series']     # N×3, mm/s²
        self._raw_jerk = data['jerk_series']            # N×3, mm/s³
        self._raw_speed = data['speed_series'].ravel()  # N, mm/s
        self._raw_acc_norm = data['acceleration_norm_series'].ravel()
        self._raw_jerk_norm = data['jerk_norm_series'].ravel()
        self._raw_tangent_acc_cmd = data['tangent_acc_cmd_series'].ravel()
        self._raw_tangent_acc_projection = data['tangent_acc_projection_series']
        self._raw_tangent_jerk_cmd = data['tangent_jerk_cmd_series'].ravel()
        self._raw_tangent_jerk_projection = data['tangent_jerk_projection_series']
        self._raw_feedrate = data['feedrate_cmd_series'].ravel()
        self._raw_chord_err = data['chord_error_series'].ravel()
        self._raw_block_idx = data['block_index'].ravel().astype(int)
        self._raw_point_idx = data['point_index'].ravel().astype(int)
        self._raw_time = data['time_series'].ravel()
        self._raw_u = data['u_series'].ravel()

        # 分块边界
        self.boundary_indices = data['boundary_point_index'].ravel().astype(int) - 1
        self.boundary_step_indices = data['boundary_step_index'].ravel().astype(int) - 1
        self.boundary_times = self._raw_time[self.boundary_indices]

        # ---- 转换到世界系 (单位: 米) ----
        self._pos_world = self._transform_points(self._raw_pos / 1000.0)
        self._vel_world = self._transform_vectors(self._raw_vel / 1000.0)
        self._acc_world = self._transform_vectors(self._raw_acc / 1000.0)
        self._jerk_world = self._transform_vectors(self._raw_jerk / 1000.0)
        self._tangent_acc_projection_world = self._transform_vectors(
            self._raw_tangent_acc_projection / 1000.0)
        self._tangent_jerk_projection_world = self._transform_vectors(
            self._raw_tangent_jerk_projection / 1000.0)

        # 标量 (与坐标系无关；转换单位并乘缩放因子, 与向量经 T 变换的缩放一致)
        s = self.scale
        self._speed = self._raw_speed / 1000.0 * s
        self._acc_norm = self._raw_acc_norm / 1000.0 * s
        self._jerk_norm = self._raw_jerk_norm / 1000.0 * s
        self._tangent_acc_cmd = self._raw_tangent_acc_cmd / 1000.0 * s
        self._tangent_jerk_cmd = self._raw_tangent_jerk_cmd / 1000.0 * s
        self._feedrate = self._raw_feedrate / 1000.0 * s * float(feedrate_scale)
        self._chord_err = self._raw_chord_err / 1000.0 * s

        # Some generated .mat files contain isolated NaNs at block/tail
        # samples. Interpolate them once at load time so the final control
        # cycle cannot inject NaN into the nominal velocity or QP matrices.
        for attr in (
            "_pos_world", "_vel_world", "_acc_world", "_jerk_world",
            "_tangent_acc_projection_world", "_tangent_jerk_projection_world",
            "_speed", "_acc_norm", "_jerk_norm", "_tangent_acc_cmd",
            "_tangent_jerk_cmd", "_feedrate", "_chord_err",
        ):
            setattr(self, attr, self._fill_nonfinite(getattr(self, attr)))

        # ---- 贴合圆柱表面 ----
        # 圆柱轴 (0,1,0) 与 controller/viewer/runtime 配置一致; 径向分量吸附
        # 到拟合半径, 保证控制路径本就工作在圆柱表面 (任何消费侧再吸附幂等)。
        _axis = np.array([0.0, 1.0, 0.0])
        _centre, _radius = self._fit_surface_with_radius(_axis)
        self._snap_path_onto_cylinder(_axis, _centre, _radius)

        # ---- 预计算姿态参考标架 ----
        self._path_geometry = None
        self.set_orientation_mode("fixed")

        print(f"[IKTrajectoryData] 加载完成: {self.num_points} 点, "
              f"{self.num_blocks} 块, Ts={self.Ts}s, 总时长={self.total_time():.3f}s")

    # ================================================================
    # 坐标变换辅助
    # ================================================================
    def _transform_points(self, pts_mm: np.ndarray) -> np.ndarray:
        """将轨迹系下的点变换到世界系"""
        N = len(pts_mm)
        pts_h = np.hstack([pts_mm, np.ones((N, 1))])
        return (self.T_traj_to_base @ pts_h.T).T[:, :3]

    def _transform_vectors(self, vecs_mm: np.ndarray) -> np.ndarray:
        """将轨迹系下的向量 (速度/加速度) 变换到世界系 (仅旋转)"""
        return (self.R_traj @ vecs_mm.T).T

    @staticmethod
    def _fill_nonfinite(series: np.ndarray, fallback: float = 0.0) -> np.ndarray:
        """Fill isolated NaN/Inf samples by linear interpolation."""
        arr = np.asarray(series, dtype=float).copy()
        if np.all(np.isfinite(arr)):
            return arr
        if arr.ndim == 1:
            idx = np.arange(arr.shape[0])
            finite = np.isfinite(arr)
            if np.any(finite):
                arr[~finite] = np.interp(idx[~finite], idx[finite], arr[finite])
            else:
                arr[:] = fallback
            return arr
        flat = arr.reshape(arr.shape[0], -1)
        for col in range(flat.shape[1]):
            flat[:, col] = IKTrajectoryData._fill_nonfinite(flat[:, col], fallback)
        return flat.reshape(arr.shape)

    # ================================================================
    # 时间索引
    # ================================================================
    def _time_to_idx(self, t: float) -> int:
        """将时间映射到最近的数据索引"""
        idx = int(round(t / self.Ts))
        return max(0, min(self.num_points - 1, idx))

    def total_time(self) -> float:
        return self._raw_time[-1]

    # ================================================================
    # 世界系采样接口
    # ================================================================
    def pos_world_at(self, t: float) -> np.ndarray:
        return self._pos_world[self._time_to_idx(t)]

    def vel_world_at(self, t: float) -> np.ndarray:
        return self._vel_world[self._time_to_idx(t)]

    def accel_world_at(self, t: float) -> np.ndarray:
        return self._acc_world[self._time_to_idx(t)]

    def jerk_world_at(self, t: float) -> np.ndarray:
        return self._jerk_world[self._time_to_idx(t)]

    # ================================================================
    # 标量特征
    # ================================================================
    def speed_at(self, t: float) -> float:
        return float(self._speed[self._time_to_idx(t)])

    def accel_norm_at(self, t: float) -> float:
        return float(self._acc_norm[self._time_to_idx(t)])

    def jerk_norm_at(self, t: float) -> float:
        return float(self._jerk_norm[self._time_to_idx(t)])

    def feedrate_cmd_at(self, t: float) -> float:
        return float(self._feedrate[self._time_to_idx(t)])

    def chord_error_at(self, t: float) -> float:
        idx = self._time_to_idx(t)
        if idx < len(self._chord_err):
            return float(self._chord_err[idx])
        return float(self._chord_err[-1])

    # ================================================================
    # 姿态参考标架
    # ================================================================
    @staticmethod
    def _normalize_vector(v: np.ndarray, fallback: np.ndarray) -> np.ndarray:
        """归一化向量；退化时使用 fallback。"""
        v = np.asarray(v, dtype=float)
        n = float(np.linalg.norm(v))
        if n > 1e-10:
            return v / n
        fallback = np.asarray(fallback, dtype=float)
        nf = float(np.linalg.norm(fallback))
        if nf > 1e-10:
            return fallback / nf
        return np.array([1.0, 0.0, 0.0])

    @staticmethod
    def _orthonormalize_rotation(R_in: np.ndarray) -> np.ndarray:
        """将近似旋转矩阵投影到 SO(3)。"""
        R_in = np.asarray(R_in, dtype=float)
        U, _, Vt = np.linalg.svd(R_in)
        R_out = U @ Vt
        if np.linalg.det(R_out) < 0.0:
            U[:, -1] *= -1.0
            R_out = U @ Vt
        return R_out

    @staticmethod
    def _default_fixed_orientation() -> np.ndarray:
        """机械臂自然姿态，保持旧版默认行为。"""
        return np.array([
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ])

    @classmethod
    def canonical_orientation_mode(cls, mode: str) -> str:
        """Return the public canonical name while accepting the legacy alias.

        ``fixed_R[:, 0]`` is the only source of truth for the held tool axis.
        Therefore the canonical non-fixed name deliberately says neither
        “down” nor another world direction: a calibrated caller may choose
        the fixed axis, while the cylinder contract supplies world ``+Z``.
        """
        normalized = str(mode).strip().lower()
        if normalized == cls.LEGACY_TOOL_DOWN_TANGENT_LIMITED:
            return cls.ORIENTATION_TOOL_FORWARD_TANGENT_LIMITED
        if normalized in (
                cls.ORIENTATION_FIXED,
                cls.ORIENTATION_TOOL_FORWARD_TANGENT_LIMITED):
            return normalized
        raise ValueError(f"unknown orientation mode: {mode}")

    def set_orientation_mode(self, mode: str = "fixed",
                             fixed_R: Optional[np.ndarray] = None,
                             max_roll_rate: Optional[float] = None,
                             max_roll_accel: Optional[float] = None):
        """
        设置轨迹姿态模式。

        fixed:
            全程保持 fixed_R。
        tool_forward_tangent_limited:
            保持 fixed_R 的 X 轴不变，绕该工具轴按轨迹切向投影缓慢滚转。
            圆柱模式传入的 fixed_R 规定 X = 世界 +Z（朝前）。

        ``tool_down_tangent_limited`` 仍接受为历史兼容别名，但不会再作为
        运行时状态或日志名称。
        """
        mode = self.canonical_orientation_mode(mode)

        fixed_R = self._default_fixed_orientation() if fixed_R is None else fixed_R
        fixed_R = self._orthonormalize_rotation(fixed_R)
        self.orientation_mode = mode

        if mode == self.ORIENTATION_FIXED:
            self._R_des_series = self._compute_fixed_frames(fixed_R)
        else:
            roll_rate = (self.TOOL_TANGENT_LIMITED_RATE
                         if max_roll_rate is None else float(max_roll_rate))
            roll_accel = (self.TOOL_TANGENT_ACCEL_LIMIT
                          if max_roll_accel is None else float(max_roll_accel))
            if not np.isfinite(roll_rate) or roll_rate <= 0.0:
                raise ValueError("max_roll_rate must be finite and positive")
            if not np.isfinite(roll_accel) or roll_accel <= 0.0:
                raise ValueError("max_roll_accel must be finite and positive")
            self._R_des_series = self._compute_tool_axis_tangent_frames(
                fixed_R, max_roll_rate=roll_rate, max_roll_accel=roll_accel)

        self._omega_series = self._compute_angular_velocity()
        # Path geometry contains the orientation series and must be rebuilt
        # whenever the public orientation mode changes.
        self._path_geometry = None
        return self

    def set_surface_normal_orientation(
        self, axis_direction: Sequence[float],
        axis_point: Optional[Sequence[float]] = None,
    ) -> None:
        """Point the tool X-axis at the cylinder centre along the whole path.

        Frame convention (matches ``compute_surface_normal_orientations``):
        X = inward radial, perpendicular to the cylinder surface and pointing
        toward the cylinder axis; Y = cylinder axis; Z = X x Y (tangent).
        The outward normal is deliberately not used: it points below the
        reachable workspace of this arm.

        ``axis_point`` is one point on the cylinder axis.  When omitted it is
        fitted from the trajectory itself (a least-squares circle in the
        plane normal to ``axis_direction``), which is how the transition
        planner and the MuJoCo viewer locate the same cylinder.  Assuming the
        axis passes through the origin gives a wrong inward normal whenever
        the cylinder centre is offset (the butterfly cylinder is offset by
        roughly +0.27 m in Z), so callers should normally leave it unset.
        """
        axis = np.asarray(axis_direction, dtype=float).ravel()
        axis_len = float(np.linalg.norm(axis))
        if axis_len < 1e-12:
            raise ValueError("axis_direction must be a non-zero 3-vector")
        axis = axis / axis_len
        if axis_point is None:
            centre, radius = self._fit_surface_with_radius(axis)
        else:
            centre = np.asarray(axis_point, dtype=float).ravel()
            if centre.shape != (3,):
                raise ValueError("axis_point must be a 3-vector")
            radius = self._fit_surface_with_radius(axis)[1]

        # 吸附到拟合圆柱表面 (镜像 cylinder_geometry.snap_path_to_cylindrical_surface):
        # 生成的原始轨迹相对拟合圆柱有毫米级径向起伏, 直接把每个点的径向分量
        # 钉到拟合半径上, 末端工具因此始终工作在圆柱表面, 不伸入也不脱离。
        # 保存拟合圆柱几何 (供末端径向侵入诊断与约束使用)。
        self.surface_axis = np.asarray(axis, dtype=float).copy()
        self.surface_centre = np.asarray(centre, dtype=float).copy()
        self.surface_radius = float(radius)
        self._snap_path_onto_cylinder(axis, centre, radius)

        frames = np.zeros((self.num_points, 3, 3))
        for index, point in enumerate(self._pos_world):
            relative = point - centre
            axial = axis * float(np.dot(relative, axis))
            radial = relative - axial
            radial_len = float(np.linalg.norm(radial))
            if radial_len < 1e-12:
                frames[index] = np.eye(3)
                continue
            x_axis = -radial / radial_len       # toward the centre line
            y_axis = axis.copy()                # cylinder axis
            z_axis = np.cross(x_axis, y_axis)
            z_axis /= np.linalg.norm(z_axis)
            y_axis = np.cross(z_axis, x_axis)   # re-orthogonalise
            frames[index] = np.column_stack((x_axis, y_axis, z_axis))

        self.orientation_mode = "surface_normal"
        self._R_des_series = frames
        self._omega_series = self._compute_angular_velocity()
        self._path_geometry = None

    def _fit_surface_axis_point(self, axis: np.ndarray) -> np.ndarray:
        """Fit one point on the cylinder axis from the loaded world points.

        Mirrors ``task_target.compute_surface_normal_orientations`` and the
        MuJoCo viewer's cylinder fit so every consumer agrees on the same
        cylinder centre.  Returns a point ``c`` such that
        ``c + t * axis`` is the fitted axis line.
        """
        return self._fit_surface_with_radius(axis)[0]

    def _fit_surface_with_radius(self, axis: np.ndarray) -> tuple[np.ndarray, float]:
        """Fit the cylinder axis point and the surface radius in one pass.

        ``c + t * axis`` is the fitted axis line and the return value is the
        least-squares surface radius, used for snapping the path onto the
        cylindrical surface.
        """
        points = np.asarray(self._pos_world, dtype=float)
        helper = np.array([1.0, 0.0, 0.0])
        if abs(float(np.dot(helper, axis))) > 0.9:
            helper = np.array([0.0, 0.0, 1.0])
        u = np.cross(axis, helper)
        u /= np.linalg.norm(u)
        v = np.cross(axis, u)
        v /= np.linalg.norm(v)

        plane_x = points @ u
        plane_y = points @ v
        design = np.column_stack((plane_x, plane_y, np.ones(points.shape[0])))
        rhs = -(plane_x * plane_x + plane_y * plane_y)
        coeff, *_ = np.linalg.lstsq(design, rhs, rcond=None)
        centre_u = -0.5 * coeff[0]
        centre_v = -0.5 * coeff[1]

        axial_values = points @ axis
        axial_centre = 0.5 * (float(axial_values.min()) + float(axial_values.max()))
        centre = centre_u * u + centre_v * v + axial_centre * axis
        radius = float(
            np.sqrt(centre_u * centre_u + centre_v * centre_v - coeff[2])
        )
        return centre, radius

    def _snap_path_onto_cylinder(
        self, axis: np.ndarray, centre: np.ndarray, radius: float
    ) -> None:
        """Radially project the stored world path (and its feedforward).

        The axial and temporal coordinates are untouched; the radial
        coordinate of every position is pinned to ``radius`` so the tool
        tracks the surface exactly.  Velocity/acceleration/jerk feedforward
        vectors get their radial component projected off so they stay tangent
        to the surface.
        """
        relative = self._pos_world - centre
        axial = np.outer(relative @ axis, axis)
        radial = relative - axial
        radial_len = np.linalg.norm(radial, axis=1)
        radial_len = np.maximum(radial_len, 1e-12)
        radial_dir = radial / radial_len[:, None]
        self._pos_world = (
            centre + axial + (radius / radial_len[:, None]) * radial
        )
        for attr in ("_vel_world", "_acc_world", "_jerk_world"):
            values = np.asarray(getattr(self, attr), dtype=float)
            radial_component = np.sum(values * radial_dir, axis=1, keepdims=True)
            setattr(self, attr, values - radial_component * radial_dir)

    def _compute_frenet_frames(self) -> np.ndarray:
        """
        兼容旧接口：返回固定自然姿态。
        """
        return self._compute_fixed_frames(self._default_fixed_orientation())

    def _compute_fixed_frames(self, fixed_R: np.ndarray) -> np.ndarray:
        return np.tile(fixed_R, (self.num_points, 1, 1))

    def _path_tangents(self) -> np.ndarray:
        """由速度序列和位置差分得到连续单位切向。"""
        N = self.num_points
        if N <= 0:
            return np.zeros((0, 3))

        pos = np.asarray(getattr(self, "_pos_world", np.zeros((N, 3))), dtype=float)
        vel = np.asarray(getattr(self, "_vel_world", np.zeros((N, 3))), dtype=float)
        if vel.shape != (N, 3):
            vel = np.zeros((N, 3))

        diff = np.zeros((N, 3))
        if N == 1:
            diff[0] = vel[0]
        else:
            diff[0] = pos[1] - pos[0]
            diff[-1] = pos[-1] - pos[-2]
            if N > 2:
                diff[1:-1] = pos[2:] - pos[:-2]

        first_fallback = np.array([1.0, 0.0, 0.0])
        for arr in (vel, diff):
            norms = np.linalg.norm(arr, axis=1)
            nz = np.where(norms > 1e-10)[0]
            if len(nz) > 0:
                first_fallback = arr[nz[0]]
                break

        tangents = np.zeros((N, 3))
        fallback = first_fallback
        for i in range(N):
            candidate = vel[i] if np.linalg.norm(vel[i]) > 1e-10 else diff[i]
            tangents[i] = self._normalize_vector(candidate, fallback)
            fallback = tangents[i]
        return tangents

    def _compute_tool_axis_tangent_frames(self, fixed_R: np.ndarray,
                                          max_roll_rate: Optional[float] = None,
                                          max_roll_accel: Optional[float] = None) -> np.ndarray:
        """Build frames with a fixed tool X axis and limited roll about it."""
        tangents = self._path_tangents()
        x_axis = fixed_R[:, 0]
        z_fallback = fixed_R[:, 2]
        frames = np.zeros((self.num_points, 3, 3))
        z_targets = np.zeros((self.num_points, 3))
        for i, tangent in enumerate(tangents):
            z_axis = tangent - np.dot(tangent, x_axis) * x_axis
            if np.linalg.norm(z_axis) < 1e-10:
                z_axis = z_fallback - np.dot(z_fallback, x_axis) * x_axis
            z_axis = self._normalize_vector(z_axis, z_fallback)
            if np.dot(z_axis, z_fallback) < 0.0:
                z_axis = -z_axis
            z_targets[i] = z_axis
            z_fallback = z_axis

        if max_roll_rate is not None and self.num_points > 0:
            y0 = fixed_R[:, 1]
            z0 = fixed_R[:, 2]
            yaw_target = np.unwrap(np.arctan2(z_targets @ y0, z_targets @ z0))
            yaw = self._rate_and_accel_limited_yaw(
                yaw_target,
                max_yaw_rate=abs(float(max_roll_rate)),
                max_yaw_accel=(self.TOOL_TANGENT_ACCEL_LIMIT
                               if max_roll_accel is None
                               else abs(float(max_roll_accel))),
            )
            z_targets = (
                np.cos(yaw)[:, None] * z0[None, :]
                + np.sin(yaw)[:, None] * y0[None, :]
            )

        for i, z_axis in enumerate(z_targets):
            y_axis = np.cross(z_axis, x_axis)
            y_axis = self._normalize_vector(y_axis, fixed_R[:, 1])
            z_axis = np.cross(x_axis, y_axis)
            frames[i] = np.column_stack((x_axis, y_axis, z_axis))
        return frames

    # Private compatibility wrapper for out-of-tree analysis scripts.  New
    # code must use the axis-neutral method above.
    def _compute_tool_down_tangent_frames(self, fixed_R: np.ndarray,
                                          max_yaw_rate: Optional[float] = None) -> np.ndarray:
        return self._compute_tool_axis_tangent_frames(
            fixed_R, max_roll_rate=max_yaw_rate)

    def _rate_and_accel_limited_yaw(self, yaw_target: np.ndarray,
                                    max_yaw_rate: float,
                                    max_yaw_accel: float) -> np.ndarray:
        """Track target yaw with bounded speed and acceleration.

        The previous limiter bounded only yaw rate. At direction changes it could
        switch the feedforward angular velocity from +0.5 to 0 to -0.5 rad/s in
        adjacent 2 ms samples. This second-order limiter keeps the same speed
        cap while ramping yaw rate smoothly.
        """
        yaw_target = np.asarray(yaw_target, dtype=float)
        yaw = np.empty_like(yaw_target)
        if yaw_target.size == 0:
            return yaw
        yaw[0] = yaw_target[0]
        yaw_rate = 0.0
        dt = float(self.Ts)
        max_rate = abs(float(max_yaw_rate))
        max_rate_delta = abs(float(max_yaw_accel)) * dt

        for i in range(1, yaw_target.size):
            err = float(yaw_target[i] - yaw[i - 1])
            desired_rate = np.clip(err / dt, -max_rate, max_rate)
            yaw_rate += float(np.clip(
                desired_rate - yaw_rate,
                -max_rate_delta,
                max_rate_delta,
            ))
            yaw_rate = float(np.clip(yaw_rate, -max_rate, max_rate))
            yaw[i] = yaw[i - 1] + yaw_rate * dt
        return yaw

    def orientation_at(self, t: float) -> np.ndarray:
        """返回 3×3 期望姿态矩阵 R_des(t)"""
        return self._R_des_series[self._time_to_idx(t)]

    def pose_at(self, t: float) -> np.ndarray:
        """返回 4×4 齐次变换矩阵 T_des(t) (位置 + 姿态)"""
        idx = self._time_to_idx(t)
        T = np.eye(4)
        T[:3, :3] = self._R_des_series[idx]
        T[:3, 3] = self._pos_world[idx]
        return T

    def _compute_angular_velocity(self) -> np.ndarray:
        """从相邻标架差分估计世界系角速度 ω"""
        N = self.num_points
        omega = np.zeros((N, 3))
        dt = self.Ts

        for i in range(1, N - 1):
            R_prev = self._R_des_series[i - 1]
            R_next = self._R_des_series[i + 1]
            dR = R_next @ R_prev.T
            omega[i] = R.from_matrix(dR).as_rotvec() / (2 * dt)

        # 边界处理
        if N >= 2:
            dR = self._R_des_series[1] @ self._R_des_series[0].T
            omega[0] = R.from_matrix(dR).as_rotvec() / dt
            dR = self._R_des_series[-1] @ self._R_des_series[-2].T
            omega[-1] = R.from_matrix(dR).as_rotvec() / dt

        return omega

    def angular_velocity_at(self, t: float) -> np.ndarray:
        return self._omega_series[self._time_to_idx(t)]

    def path_geometry(self):
        """Return the shared arc-length geometry for path-following control.

        The result is cached after orientation construction.  It uses the
        already transformed world-frame samples, so callers must not apply the
        trajectory-to-base transform a second time.
        """
        if self._path_geometry is None:
            from work.path_following import PathGeometry

            self._path_geometry = PathGeometry.from_samples(
                self._pos_world,
                self._R_des_series,
                self._feedrate,
                self._raw_time,
            )
        return self._path_geometry

    # ================================================================
    # 6-DOF 任务参考
    # ================================================================
    def task_reference_at(self, t: float) -> TaskReference:
        """返回完整 6-DOF 任务参考 (位置来自数据, 姿态来自当前模式)"""
        idx = self._time_to_idx(t)
        return TaskReference(
            t=t,
            pos=self._pos_world[idx].copy(),
            vel=self._vel_world[idx].copy(),
            accel=self._acc_world[idx].copy(),
            R_des=self._R_des_series[idx].copy(),
            omega=self._omega_series[idx].copy(),
        )

    def task_reference_at_continuous(self, t: float) -> TaskReference:
        """Return a continuous-time 6-DOF reference using adjacent samples.

        Exact sample times intentionally reuse task_reference_at() so the
        existing indexed behavior remains bit-for-bit stable at k*Ts.
        """
        last_t = (self.num_points - 1) * self.Ts
        if t <= 0.0:
            return self.task_reference_at(0.0)
        if t >= last_t:
            return self.task_reference_at(last_t)

        sample = t / self.Ts
        nearest = round(sample)
        if abs(sample - nearest) < 1e-10:
            return self.task_reference_at(nearest * self.Ts)

        i0 = int(np.floor(sample))
        i1 = min(i0 + 1, self.num_points - 1)
        tau = float((t - i0 * self.Ts) / self.Ts)
        dt = float(self.Ts)

        p0 = self._pos_world[i0]
        p1 = self._pos_world[i1]
        v0 = self._vel_world[i0]
        v1 = self._vel_world[i1]

        h00 = 2.0 * tau**3 - 3.0 * tau**2 + 1.0
        h10 = tau**3 - 2.0 * tau**2 + tau
        h01 = -2.0 * tau**3 + 3.0 * tau**2
        h11 = tau**3 - tau**2

        dh00 = 6.0 * tau**2 - 6.0 * tau
        dh10 = 3.0 * tau**2 - 4.0 * tau + 1.0
        dh01 = -6.0 * tau**2 + 6.0 * tau
        dh11 = 3.0 * tau**2 - 2.0 * tau

        ddh00 = 12.0 * tau - 6.0
        ddh10 = 6.0 * tau - 4.0
        ddh01 = -12.0 * tau + 6.0
        ddh11 = 6.0 * tau - 2.0

        pos = h00 * p0 + h10 * dt * v0 + h01 * p1 + h11 * dt * v1
        vel = (dh00 * p0 + dh10 * dt * v0 + dh01 * p1 + dh11 * dt * v1) / dt
        accel = (ddh00 * p0 + ddh10 * dt * v0 + ddh01 * p1 + ddh11 * dt * v1) / (dt * dt)

        R0 = self._R_des_series[i0]
        R1 = self._R_des_series[i1]
        if np.allclose(R0, R1, atol=1e-12):
            R_des = R0.copy()
        else:
            slerp = Slerp([0.0, 1.0], R.from_matrix([R0, R1]))
            R_des = slerp([tau]).as_matrix()[0]
        omega = (1.0 - tau) * self._omega_series[i0] + tau * self._omega_series[i1]

        return TaskReference(
            t=t,
            pos=pos.copy(),
            vel=vel.copy(),
            accel=accel.copy(),
            R_des=R_des.copy(),
            omega=omega.copy(),
        )

    def motion_scalars_at(self, t: float) -> Dict[str, float]:
        """Return scalar schedule data from the .mat trajectory at time t.

        Units are SI after the same transform/scale used for the task
        reference: speed/feedrate in m/s, acceleration in m/s^2, jerk in
        m/s^3, chord error in m.
        """
        last_t = (self.num_points - 1) * self.Ts
        if t <= 0.0:
            i0 = i1 = 0
            tau = 0.0
        elif t >= last_t:
            i0 = i1 = self.num_points - 1
            tau = 0.0
        else:
            sample = t / self.Ts
            i0 = int(np.floor(sample))
            i1 = min(i0 + 1, self.num_points - 1)
            tau = float(sample - i0)

        def lerp(series):
            series = np.asarray(series)
            j0 = min(i0, len(series) - 1)
            j1 = min(i1, len(series) - 1)
            return float((1.0 - tau) * series[j0] + tau * series[j1])

        def vec_norm_lerp(series):
            series = np.asarray(series)
            j0 = min(i0, len(series) - 1)
            j1 = min(i1, len(series) - 1)
            v = (1.0 - tau) * series[j0] + tau * series[j1]
            return float(np.linalg.norm(v))

        # The trajectory start/end are always valid schedule boundaries even
        # when the source .mat exposes only interior or terminal blocks.
        # Returning ``inf`` here leaks into gain scheduling and makes a
        # finite trajectory look malformed near its tail.
        boundary_times = np.unique(np.concatenate([
            np.asarray(self.boundary_times, dtype=float),
            np.array([0.0, last_t], dtype=float),
        ]))
        prev_times = boundary_times[boundary_times <= t]
        next_times = boundary_times[boundary_times >= t]
        prev_boundary_s = float(t - prev_times[-1])
        next_boundary_s = float(next_times[0] - t)
        boundary_distance_s = min(prev_boundary_s, next_boundary_s)

        return {
            "speed": lerp(self._speed),
            "feedrate": lerp(self._feedrate),
            "acc_norm": lerp(self._acc_norm),
            "jerk_norm": lerp(self._jerk_norm),
            "tangent_acc_cmd": lerp(self._tangent_acc_cmd),
            "tangent_acc_projection_norm": vec_norm_lerp(self._tangent_acc_projection_world),
            "tangent_jerk_cmd": lerp(self._tangent_jerk_cmd),
            "tangent_jerk_projection_norm": vec_norm_lerp(self._tangent_jerk_projection_world),
            "chord_err": lerp(self._chord_err),
            "u": lerp(self._raw_u),
            "point_index": float(self._raw_point_idx[i0]),
            "block_index": float(self._raw_block_idx[i0]),
            "prev_boundary_s": prev_boundary_s,
            "next_boundary_s": next_boundary_s,
            "boundary_distance_s": boundary_distance_s,
        }

    def task_reference_sequence(self) -> List[TaskReference]:
        """返回全部时间点的任务参考序列"""
        return [self.task_reference_at(t) for t in self._raw_time]

    # ================================================================
    # NURBS 分块
    # ================================================================
    def block_index_at(self, t: float) -> int:
        return int(self._raw_block_idx[self._time_to_idx(t)])

    def is_block_boundary(self, t: float) -> bool:
        """检查当前时刻是否在分块边界 (前后索引跨越边界)"""
        idx = self._time_to_idx(t)
        if idx == 0 or idx >= self.num_points - 1:
            return False
        return self._raw_block_idx[idx] != self._raw_block_idx[idx + 1]

    def get_block_ranges(self) -> List[Tuple[int, int, int]]:
        """返回 [(block_id, start_idx, end_idx), ...] 共 num_blocks 段"""
        ranges = []
        for i in range(len(self.boundary_indices) - 1):
            block_id = i + 1
            start = int(self.boundary_indices[i])
            end = int(self.boundary_indices[i + 1])
            ranges.append((block_id, start, end))
        return ranges

    def get_boundary_times(self) -> np.ndarray:
        """返回分块边界时间 (s)"""
        return self.boundary_times.copy()

    # ================================================================
    # 路点提取 (供 OMPL 使用)
    # ================================================================
    def extract_key_waypoints(self, n_waypoints: int = 80) -> List[TaskReference]:
        """
        从 12,765 点中提取关键路点:
        - 优先在分块边界处放置路点 (22 个边界 = 已含 22 个路点)
        - 在高曲率 / 高加速度段增加密度
        - 确保间距 ≥ 0.2s, ≤ 0.5s
        """
        # 策略: 在分块边界之间按动态重要性均匀采样
        waypoints = []

        for block_id, start, end in self.get_block_ranges():
            # 分块起始点 (边界)
            t_start = self._raw_time[start]
            waypoints.append(self.task_reference_at(t_start))

            # 块内部: 根据动态特性决定采样数
            block_duration = self._raw_time[end] - self._raw_time[start]
            avg_accel = np.mean(self._acc_norm[start:end + 1])
            avg_jerk = np.mean(self._jerk_norm[start:end + 1])

            # 高动态 → 更多路点
            dyn_factor = 1.0 + 2.0 * (avg_accel / max(np.max(self._acc_norm), 1e-6))
            dyn_factor += 2.0 * (avg_jerk / max(np.max(self._jerk_norm), 1e-6))

            # 最少 1 个内部点，最多根据块大小
            n_inner = max(1, min(int(block_duration / 0.3 * dyn_factor), 5))

            inner_indices = np.linspace(start + 1, end - 1, n_inner, dtype=int)
            for idx in inner_indices:
                if idx > start and idx < end:
                    t = self._raw_time[idx]
                    waypoints.append(self.task_reference_at(t))

        # 降采样到目标数量
        if len(waypoints) > n_waypoints:
            indices = np.linspace(0, len(waypoints) - 1, n_waypoints, dtype=int)
            waypoints = [waypoints[i] for i in indices]

        print(f"[extract_key_waypoints] 提取 {len(waypoints)} 个关键路点 (目标 {n_waypoints})")
        return waypoints
