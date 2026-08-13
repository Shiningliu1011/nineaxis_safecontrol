"""MuJoCo simulation demo: JaxControlLoop tracks the NURBS trajectory while
avoiding obstacles derived from the LIVE camera via the perception bridge.

The perception bridge transforms the camera cloud into ``base_link`` (via
``camera_to_world_static``) and publishes fixed-shape ESDF + 8 dynamic slots.
This node subscribes to those topics, feeds ``sdf_*`` / ``obs_*`` into
``JaxControlLoop.path_tracking_step`` each control step, and renders the arm
plus the camera obstacle points in MuJoCo.

Stand up::

    # terminal 1 — camera driver
    ros2 launch orbbec_camera gemini_330_series.launch.py \\
        depth_width:=640 depth_height:=480 depth_fps:=30 \\
        color_width:=640 color_height:=480 color_fps:=30 \\
        depth_registration:=true enable_colored_point_cloud:=true
    # terminal 2 — perception bridge
    ros2 run robot_safecontrol_moveit perception_bridge
    # terminal 3 — this demo
    ros2 run robot_safecontrol_moveit perception_demo

Walk in front of the camera: the mapped obstacle appears near the arm path and
the arm deviates to avoid it.
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
from sensor_msgs_py import point_cloud2 as pc2

import mujoco

_REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO_ROOT / "portable_oscbf"))
sys.path.insert(0, str(_REPO_ROOT / "portable_oscbf" / "work"))

from work.perception_config import load_point_cloud_collision, spec_of  # noqa: E402
from work.ik_data_loader import load_repository_trajectory  # noqa: E402
from work.jax_control_facade import JaxControlLoop  # noqa: E402
from work.nineaxis_kinematics import NineaxisKinematics  # noqa: E402
from work.path_following import PathFollowingConfig  # noqa: E402
from work.safety_snapshot import MAX_DYNAMIC_TRACKS  # noqa: E402

from .mujoco_viewer_with_cylinder import MuJoCoJointStateViewer  # noqa: E402

DEFAULT_JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9")
N_OBS_GEOMS = 120  # display-only obstacle spheres in the MuJoCo scene


def decode_esdf_message(msg) -> tuple[np.ndarray, tuple]:
    """Float32MultiArray(/perception/esdf) -> (grid float32, shape)."""
    shape = tuple(int(d.size) for d in msg.layout.dim)
    if len(shape) != 3 or min(shape) < 2:
        raise ValueError(f"invalid esdf layout dims: {shape}")
    data = msg.data
    arr = (np.frombuffer(data, dtype=np.float32).reshape(shape)
           if isinstance(data, bytes) else np.asarray(
               data, dtype=np.float32).reshape(shape))
    return arr, shape


def decode_tracks_message(msg) -> np.ndarray:
    """Float32MultiArray(/perception/tracks) -> (8, 10) float32 slots."""
    arr = np.asarray(msg.data, dtype=np.float32)
    if arr.size < MAX_DYNAMIC_TRACKS * 10:
        raise ValueError(
            f"tracks payload too small: {arr.size} < {MAX_DYNAMIC_TRACKS * 10}")
    return arr.reshape(MAX_DYNAMIC_TRACKS, 10)


def _source_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _resolve_path(param_value: str, default_relative: str) -> Path:
    if param_value.strip():
        return Path(param_value).expanduser()
    for base in (Path.cwd(), _source_root()):
        candidate = base / default_relative
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(f"File not found: {default_relative}")


def _resolve_dir(default_relative: str) -> Path:
    for base in (Path.cwd(), _source_root()):
        candidate = base / default_relative
        if candidate.is_dir():
            return candidate
    raise FileNotFoundError(f"Directory not found: {default_relative}")


def _work_start_configuration(trajectory) -> np.ndarray:
    """IK the trajectory start pose (mirrors the M5 baseline runner)."""
    target_position = trajectory.pos_world_at(0.0)
    target_rotation = trajectory.orientation_at(0.0)
    kinematics = NineaxisKinematics()
    rng = np.random.default_rng(0)
    seeds = [np.zeros(9)]
    lower = np.asarray(kinematics.joint_limits.q_min)
    upper = np.asarray(kinematics.joint_limits.q_max)
    for _ in range(24):
        seeds.append(rng.uniform(lower + 0.1, upper - 0.1))
    best_q = None
    best_error = float("inf")
    for seed in seeds:
        candidate = kinematics.ik(target_position, target_rotation, q_init=seed)
        if candidate is None:
            continue
        error = float(np.linalg.norm(
            kinematics.ee_position(candidate) - target_position))
        if error < best_error:
            best_error = error
            best_q = candidate
    if best_q is None or best_error > 1e-3:
        raise RuntimeError(
            f"IK failed to reach trajectory start (best error {best_error:.4f} m)")
    return np.asarray(best_q, dtype=np.float64)


class PerceptionDemo(Node):
    """Sim arm tracking the NURBS path, avoiding live-camera obstacles."""

    def __init__(self) -> None:
        super().__init__("perception_demo")
        self._declare_parameters()

        self._cfg = load_point_cloud_collision()
        self._spec = spec_of(self._cfg)
        self._control_dt = float(self.get_parameter("control_dt_s").value)
        self._sdf_margin = float(self.get_parameter("sdf_margin").value)
        self._joint_names = tuple(
            str(n) for n in self.get_parameter("joint_names").value)

        # --- trajectory + controller -------------------------------------
        traj_path = _resolve_path(
            str(self.get_parameter("trajectory_mat").value),
            "data/nurbs/ik_input.mat")
        self.get_logger().info(f"PERCEPTION_DEMO loading trajectory {traj_path}")
        trajectory = load_repository_trajectory(str(traj_path))
        geometry = trajectory.path_geometry()
        initial_q = _work_start_configuration(trajectory)

        self._loop = JaxControlLoop(
            dt=self._control_dt, temporal_lambda=0.2, enable_x64=True,
            sdf_shape=self._spec.shape)
        self._loop.configure_path(geometry, PathFollowingConfig())
        self._loop.init_cbf()

        self._q = initial_q.copy()
        self._path_state = np.asarray(self._loop.initial_path_state())
        self._u_safe = np.zeros(9, dtype=np.float64)
        self._done = False

        # --- perception state (guarded by lock) ---------------------------
        self._plock = threading.Lock()
        self._esdf_grid = None          # float32 spec.shape
        self._esdf_origin = None        # (3,) float32
        self._esdf_voxel = None
        self._esdf_stamp = 0.0
        self._tracks = np.zeros((MAX_DYNAMIC_TRACKS, 10), dtype=np.float32)
        self._tracks_stamp = 0.0
        self._cloud_pts = np.empty((0, 3), dtype=np.float32)

        # --- subscriptions (ReentrantCallbackGroup → own thread) ----------
        group = ReentrantCallbackGroup()
        self.create_subscription(
            Float32MultiArray, "/perception/esdf", self._esdf_cb,
            qos_profile_sensor_data, callback_group=group)
        self.create_subscription(
            Float32MultiArray, "/perception/esdf_meta", self._meta_cb,
            qos_profile_sensor_data, callback_group=group)
        self.create_subscription(
            Float32MultiArray, "/perception/tracks", self._tracks_cb,
            qos_profile_sensor_data, callback_group=group)
        self.create_subscription(
            PointCloud2, "/perception/cloud_world", self._cloud_cb,
            qos_profile_sensor_data, callback_group=group)

        # --- MuJoCo model --------------------------------------------------
        model_xml = self._build_mjcf(geometry)
        self._model = mujoco.MjModel.from_xml_string(model_xml)
        self._data = mujoco.MjData(self._model)
        self._obs_geom_ids = [self._model.geom(f"obs_{i}").id
                              for i in range(N_OBS_GEOMS)]

        self.get_logger().info(
            f"PERCEPTION_DEMO_READY sdf_shape={self._spec.shape} "
            f"voxel={self._spec.voxel_size} control_dt={self._control_dt}")

    def _declare_parameters(self) -> None:
        self.declare_parameter("control_dt_s", 0.002)
        self.declare_parameter("render_hz", 60.0)
        self.declare_parameter("sdf_margin", 0.08)
        self.declare_parameter("trajectory_mat", "")
        self.declare_parameter("joint_names", list(DEFAULT_JOINT_NAMES))
        self.declare_parameter("enable_dynamic_slots", True)

    # ------------------------------------------------------------------
    # Perception callbacks
    # ------------------------------------------------------------------
    def _esdf_cb(self, msg: Float32MultiArray) -> None:
        try:
            grid, _ = decode_esdf_message(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"esdf decode failed: {exc}")
            return
        with self._plock:
            self._esdf_grid = grid
            self._esdf_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _meta_cb(self, msg: Float32MultiArray) -> None:
        arr = np.asarray(msg.data, dtype=np.float32)
        if arr.size < 5 or arr[4] <= 0.0:
            return
        with self._plock:
            self._esdf_origin = arr[:3].copy()
            self._esdf_voxel = float(arr[3])

    def _tracks_cb(self, msg: Float32MultiArray) -> None:
        try:
            arr = decode_tracks_message(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"tracks decode failed: {exc}")
            return
        with self._plock:
            self._tracks = arr.copy()
            self._tracks_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _cloud_cb(self, msg: PointCloud2) -> None:
        try:
            arr = pc2.read_points_numpy(msg, field_names=["x", "y", "z"])
            pts = np.asarray(arr, dtype=np.float32).reshape(-1, 3)
        except Exception:  # noqa: BLE001
            return
        with self._plock:
            self._cloud_pts = pts

    # ------------------------------------------------------------------
    # Control step
    # ------------------------------------------------------------------
    def control_step(self) -> dict:
        """One JaxControlLoop.path_tracking_step against the latest perception."""
        with self._plock:
            grid = self._esdf_grid
            origin = self._esdf_origin
            voxel = self._esdf_voxel
            tracks = self._tracks.copy()

        sdf_distance = None
        sdf_enabled = 0.0
        if grid is not None and origin is not None and voxel is not None:
            if grid.shape != self._spec.shape:
                # 陈旧/不匹配的 bridge 消息: 本步禁用 ESDF, 避免 JIT shape 校验崩溃。
                self.get_logger().warn(
                    f"esdf shape {grid.shape} != spec {self._spec.shape}; "
                    f"skipping ESDF this step")
            else:
                sdf_distance = np.asarray(grid, dtype=np.float32)
                sdf_enabled = 1.0

        obs_pos = tracks[:, 0:3].astype(np.float64)
        obs_radii = tracks[:, 3].astype(np.float64)
        obs_vel = tracks[:, 4:7].astype(np.float64)
        obs_enabled = tracks[:, 7].astype(np.float64)
        obs_d_safe = tracks[:, 8].astype(np.float64)
        obs_alpha = tracks[:, 9].astype(np.float64)
        if not bool(self.get_parameter("enable_dynamic_slots").value):
            obs_enabled = np.zeros(MAX_DYNAMIC_TRACKS, dtype=np.float64)

        result = self._loop.path_tracking_step(
            q=self._q,
            path_state=self._path_state,
            kp_pos=50.0, kp_orient=10.0, kp_joint=0.45,
            q_des=self._q.copy(),
            nullspace_speed_limit=0.18, damping=1e-3,
            obs_pos=obs_pos, obs_radii=obs_radii, obs_enabled=obs_enabled,
            obs_d_safe=obs_d_safe, obs_vel=obs_vel,
            obs_radius_dot=np.zeros(MAX_DYNAMIC_TRACKS, dtype=np.float64),
            obs_alpha=obs_alpha, u_safe_prev=self._u_safe,
            sdf_distance=sdf_distance,
            sdf_origin=origin,
            sdf_voxel_size=voxel,
            sdf_enabled=sdf_enabled,
            sdf_margin=self._sdf_margin,
        )
        self._q = np.asarray(result.q_next)
        self._path_state = np.asarray(result.path_state)
        self._u_safe = np.asarray(result.u_safe)
        if float(np.asarray(result.path_state)[4]) > 0.5:
            self._done = True
        return {
            "min_dist": float(result.min_obs_dist),
            "min_esdf": float(self._loop.last_min_esdf_dist),
            "qp_ok": bool(result.qp_ok),
            "progress_m": float(np.asarray(result.path_state)[0]),
            "done": self._done,
        }

    # ------------------------------------------------------------------
    # MuJoCo model + rendering
    # ------------------------------------------------------------------
    def _build_mjcf(self, geometry):
        urdf_path = _resolve_path("", "models/ninezzhou/urdf/ninezzhou.urdf")
        mesh_dir = _resolve_dir("models/ninezzhou/meshes")
        xml = MuJoCoJointStateViewer._urdf_to_mjcf(urdf_path, mesh_dir)
        xml = MuJoCoJointStateViewer._inject_display_scene(
            xml, target_path=(), tracking_cylinder=None,
            joint_names=self._joint_names)
        # Display-only obstacle spheres (repositioned per frame).
        obs_xml = "".join(
            f'<geom name="obs_{i}" type="sphere" size="0.008" pos="0 0 0" '
            f'rgba="0.9 0.15 0.1 0.85" contype="0" conaffinity="0"/>\n'
            for i in range(N_OBS_GEOMS))
        return xml.replace(
            "</worldbody>", "    <body name=\"camera_obstacles\">\n"
            f"    {obs_xml}    </body>\n  </worldbody>", 1)

    def apply_to(self, data) -> None:
        """Write the latest joint config + obstacle points into MuJoCo data."""
        data.qpos[:9] = self._q
        mujoco.mj_forward(self._model, data)
        with self._plock:
            pts = self._cloud_pts
        if len(pts):
            if len(pts) > N_OBS_GEOMS:
                keep = np.random.choice(len(pts), N_OBS_GEOMS, replace=False)
                pts = pts[keep]
            pts = pts[:N_OBS_GEOMS]
        else:
            pts = np.empty((0, 3))
        for i, geom_id in enumerate(self._obs_geom_ids):
            if i < len(pts):
                data.geom_xpos[geom_id] = pts[i]
            else:
                data.geom_xpos[geom_id] = [100.0, 100.0, 100.0]


def main(args=None) -> int:
    rclpy.init(args=args)
    node = PerceptionDemo()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    render_hz = float(node.get_parameter("render_hz").value)
    period_s = 1.0 / max(render_hz, 1.0)
    steps_per_render = max(1, int(round(period_s / node._control_dt)))

    try:
        with mujoco.viewer.launch_passive(node._model, node._data) as viewer:
            last_render = time.monotonic()
            last_log = time.monotonic()
            while rclpy.ok() and viewer.is_running():
                # 非阻塞收感知回调 (独立线程 executor)。
                executor.spin_once(timeout_sec=0.0)
                for _ in range(steps_per_render):
                    try:
                        metrics = node.control_step()
                    except Exception as exc:  # noqa: BLE001 — 不因单步错误退出
                        node.get_logger().warn(f"control_step failed: {exc}")
                        break
                    if node._done:
                        break
                now = time.monotonic()
                if now - last_render >= period_s:
                    with viewer.lock():
                        node.apply_to(node._data)
                    viewer.sync()
                    last_render = now
                if now - last_log >= 2.0:
                    node.get_logger().info(
                        f"min_obs={metrics['min_dist']:.3f} "
                        f"min_esdf={metrics['min_esdf']:.3f} "
                        f"qp_ok={metrics['qp_ok']} "
                        f"progress={metrics['progress_m']:.2f}/{node._loop.last_path_metrics.get('path_progress_m', 0.0):.2f}")
                    last_log = now
                if node._done:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
