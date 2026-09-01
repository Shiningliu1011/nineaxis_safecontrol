"""OSCBF safe-control ROS 2 node (M10).

The node is deliberately independent of MoveIt: it loads the repository
trajectory, runs the pure-JAX OSCBF control kernel, and publishes the safe
joint command onto the dedicated command stream (``/oscbf_command``), separate
from the plant state stream it subscribes to.  The ``portable_oscbf`` source
tree is shipped in the package share directory; the node adds it (and the
vendored ``dpax``) to ``sys.path`` before importing ``work``.
"""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import List, Optional, Sequence

import numpy as np
import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger

from .oscbf_trajectory import bootstrap_portable
from .robot_spec import DEFAULT_JOINT_NAMES
from .ros_conventions import (
    JOINT_STATE_TOPIC,
    OSCBF_COMMAND_TOPIC,
    state_stream_qos,
)


def _default_share_dir() -> Path:
    """Installed share directory, or the repository root in source trees."""
    try:
        return Path(get_package_share_directory("robot_safecontrol_moveit"))
    except PackageNotFoundError:
        return Path.cwd()


class OscbfController(Node):
    """Run the JAX OSCBF kernel on the MuJoCo joint-state stream."""

    def __init__(
        self,
        *,
        node_name: str = "oscbf_controller",
        parameter_overrides: Optional[List] = None,
        context=None,
    ) -> None:
        super().__init__(
            node_name,
            context=context,
            parameter_overrides=parameter_overrides,
        )
        self._declare_parameters()
        self._resolve_empty_path_parameters()
        self._validate_parameters()
        self._step_durations: List[float] = []
        self._qp_fail_count = 0
        self._latest_q: Optional[np.ndarray] = None
        self._received_any_state = False
        self._last_state_time: Optional[float] = None
        self._last_result = None
        self._trajectory_duration_s = 30.0
        self._completion_logged = False
        self._tracking_started = not bool(
            self.get_parameter("wait_for_start").value
        )
        self._log_throttle = 0.0

        portable_root = Path(
            str(self.get_parameter("portable_oscbf_root").value)
        )
        bootstrap_portable(portable_root)
        self._build_controller(portable_root)

        joint_state_topic = str(
            self.get_parameter("joint_state_topic").value
        )
        publish_topic = str(
            self.get_parameter("publish_joint_state_topic").value
        )
        # Same QoS as the MuJoCo viewer, which owns the joint-state stream.
        self.create_subscription(
            JointState,
            joint_state_topic,
            self._joint_state_callback,
            state_stream_qos(),
        )
        self._publisher = self.create_publisher(
            JointState, publish_topic, qos_profile_sensor_data
        )
        period_s = 1.0 / float(
            self.get_parameter("publish_frequency_hz").value
        )
        self._timer = self.create_timer(period_s, self._control_tick)
        self._telemetry_timer = self.create_timer(1.0, self._telemetry_tick)

        # 感知障碍物订阅（默认 disabled，不影响现有行为）
        self._obs_state: dict = {}
        self._enable_obs = bool(
            self.get_parameter("enable_perception_obstacles").value)
        if self._enable_obs:
            tracks_topic = str(
                self.get_parameter("perception_tracks_topic").value)
            self.create_subscription(
                Float32MultiArray, tracks_topic,
                self._tracks_callback, qos_profile_sensor_data)
            self.get_logger().info(f"perception obstacles enabled: {tracks_topic}")

        if not self._tracking_started:
            self._start_service = self.create_service(
                Trigger, "/oscbf_controller/start_tracking",
                self._start_tracking_callback,
            )
        else:
            self._start_service = None
        self.get_logger().info(
            "oscbf_controller ready: trajectory="
            f"{self.get_parameter('trajectory_mat').value}, "
            f"subscribe={joint_state_topic}, publish={publish_topic} @ "
            f"{float(self.get_parameter('publish_frequency_hz').value):.1f} Hz, "
            f"tracking={'auto-start' if self._tracking_started else 'waiting for /oscbf_controller/start_tracking'}"
        )

    # ------------------------------------------------------------------
    # Parameter handling
    # ------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        share_dir = _default_share_dir()
        default_portable_root = share_dir / "portable_oscbf"
        defaults = {
            "dt": 0.002,
            "publish_frequency_hz": 100.0,
            "trajectory_mat": str(share_dir / "data" / "nurbs" / "ik_input.mat"),
            "portable_oscbf_root": str(default_portable_root),
            "portable_config_yaml": str(
                default_portable_root / "config" / "nineaxis.yaml"
            ),
            "joint_names": list(DEFAULT_JOINT_NAMES),
            "joint_state_topic": JOINT_STATE_TOPIC,
            "publish_joint_state_topic": OSCBF_COMMAND_TOPIC,
            "kp_pos": 60.0,
            "kp_orient": 10.0,
            "kp_joint": 0.45,
            "nullspace_speed_limit": 0.18,
            "damping": 1e-3,
            "w_pos": 20.0,
            "w_orient": 10.0,
            "w_joint": 0.1,
            "temporal_lambda": 0.2,
            "enable_x64": True,
            "solver_tol": 1e-3,
            "task_mode": "tool_axis_5d",
            "use_nullspace_policy": False,
            "reference_lead_m": 0.005,
            "orientation_mode": "surface_normal",
            "cylinder_axis_direction": [0.0, 1.0, 0.0],
            "cylinder_center": [],
            "wait_for_start": False,
            "telemetry_period_s": 1.0,
            "perf_report_path": "output/oscbf_m10_perf.md",
            # 感知障碍物（默认 disabled，不影响现有行为）
            "enable_perception_obstacles": False,
            "perception_tracks_topic": "/perception/tracks",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._default_parameters = defaults

    def _resolve_empty_path_parameters(self) -> None:
        """Empty path overrides fall back to the computed defaults."""
        for name in (
            "trajectory_mat",
            "portable_oscbf_root",
            "portable_config_yaml",
        ):
            value = str(self.get_parameter(name).value)
            if not value:
                default = self._default_parameters[name]
                self.set_parameters([rclpy.parameter.Parameter(
                    name, rclpy.parameter.Parameter.Type.STRING, default
                )])

    def _validate_parameters(self) -> None:
        dt = float(self.get_parameter("dt").value)
        frequency_hz = float(self.get_parameter("publish_frequency_hz").value)
        if not 0.0 < dt <= 0.1:
            raise ValueError(f"dt must be in (0, 0.1], got {dt}")
        if not 1.0 <= frequency_hz <= 1000.0:
            raise ValueError(
                f"publish_frequency_hz must be in [1, 1000], got {frequency_hz}"
            )
        for name in ("kp_pos", "kp_orient", "kp_joint", "nullspace_speed_limit",
                     "w_pos", "w_orient", "w_joint"):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive, got {value}")
        damping = float(self.get_parameter("damping").value)
        solver_tol = float(self.get_parameter("solver_tol").value)
        if not math.isfinite(damping) or damping <= 0.0:
            raise ValueError(f"damping must be positive, got {damping}")
        if not 0.0 < solver_tol < 1.0:
            raise ValueError(f"solver_tol must be in (0, 1), got {solver_tol}")
        joint_names = [
            str(name) for name in self.get_parameter("joint_names").value
        ]
        if len(joint_names) != 9 or len(set(joint_names)) != 9:
            raise ValueError(
                "joint_names must contain exactly 9 unique joint names"
            )
        cylinder_center = list(self.get_parameter("cylinder_center").value)
        if cylinder_center and len(cylinder_center) != 3:
            raise ValueError(
                f"cylinder_center must be empty or a 3-vector, got {cylinder_center}"
            )
        task_mode = str(self.get_parameter("task_mode").value)
        if task_mode not in ("pose6d", "tool_axis_5d"):
            raise ValueError(
                f"task_mode must be 'pose6d' or 'tool_axis_5d', got {task_mode!r}"
            )
        trajectory_mat = Path(str(self.get_parameter("trajectory_mat").value))
        if not trajectory_mat.is_file():
            raise FileNotFoundError(
                f"trajectory_mat not found: {trajectory_mat}"
            )

    # ------------------------------------------------------------------
    # Controller construction
    # ------------------------------------------------------------------

    def _build_controller(self, portable_root: Path) -> None:
        from work.ik_data_loader import load_repository_trajectory
        from work.jax_control_facade import JaxControlLoop
        from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
        from work.nullspace_policy import ManipulabilityGradientPolicy
        from work.path_following import PathFollowingConfig

        self.get_logger().info("Loading repository trajectory ...")
        trajectory_mat = str(self.get_parameter("trajectory_mat").value)
        config_yaml = str(self.get_parameter("portable_config_yaml").value)
        trajectory = load_repository_trajectory(
            trajectory_mat, config_yaml_path=config_yaml
        )
        self._trajectory_duration_s = float(
            trajectory.num_points * trajectory.Ts
        )
        orientation_mode = str(
            self.get_parameter("orientation_mode").value
        )
        if orientation_mode == "surface_normal":
            center = list(self.get_parameter("cylinder_center").value)
            axis_point = (
                None if len(center) == 0 else center
            )
            trajectory.set_surface_normal_orientation(
                self.get_parameter("cylinder_axis_direction").value,
                axis_point=axis_point,
            )
        elif orientation_mode != "fixed":
            raise ValueError(
                "orientation_mode must be 'fixed' or 'surface_normal', "
                f"got {orientation_mode!r}"
            )
        geometry = trajectory.path_geometry()

        policy = None
        if bool(self.get_parameter("use_nullspace_policy").value):
            robot = NineaxisManipulatorJAX()
            policy = ManipulabilityGradientPolicy(robot)

        self._loop = JaxControlLoop(
            dt=float(self.get_parameter("dt").value),
            w_pos=float(self.get_parameter("w_pos").value),
            w_orient=float(self.get_parameter("w_orient").value),
            w_joint=float(self.get_parameter("w_joint").value),
            temporal_lambda=float(self.get_parameter("temporal_lambda").value),
            enable_x64=bool(self.get_parameter("enable_x64").value),
            solver_tol=float(self.get_parameter("solver_tol").value),
            task_mode=str(self.get_parameter("task_mode").value),
            nullspace_policy=policy,
        )
        self._loop.configure_path(
            geometry,
            PathFollowingConfig(
                reference_lead_m=float(
                    self.get_parameter("reference_lead_m").value
                )
            ),
        )
        self.get_logger().info("Warming up the JAX control kernel ...")
        self._loop.init_cbf()
        self.get_logger().info("JAX control kernel warm-up complete")
        self._path_state = self._loop.initial_path_state()
        self._joint_names = [
            str(name) for name in self.get_parameter("joint_names").value
        ]
        self._limits = (
            np.asarray(self._loop.robot.joint_lower_limits, dtype=float),
            np.asarray(self._loop.robot.joint_upper_limits, dtype=float),
        )

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _extract_positions(self, message: JointState) -> Optional[np.ndarray]:
        if len(message.position) == 9 and not message.name:
            return np.asarray(message.position, dtype=float)
        if set(message.name) != set(self._joint_names):
            return None
        order = {name: index for index, name in enumerate(message.name)}
        positions = np.asarray(
            [message.position[order[name]] for name in self._joint_names],
            dtype=float,
        )
        return positions

    def _joint_state_callback(self, message: JointState) -> None:
        positions = self._extract_positions(message)
        if positions is None or not np.all(np.isfinite(positions)):
            return
        self._received_any_state = True
        self._latest_q = positions
        self._last_state_time = time.monotonic()

    def _tracks_callback(self, message: Float32MultiArray) -> None:
        """解码 /perception/tracks（8×10 float）→ obs_* 数组缓存。"""
        from .obstacle_extractor import MAX_OBSTACLE_SLOTS, TRACK_SLOT_FLOATS
        arr = np.asarray(message.data, dtype=np.float32)
        expected = MAX_OBSTACLE_SLOTS * TRACK_SLOT_FLOATS
        if arr.size < expected:
            return
        slots = arr[:expected].reshape(MAX_OBSTACLE_SLOTS, TRACK_SLOT_FLOATS)
        self._obs_state = {
            "obs_pos": slots[:, 0:3].astype(np.float64),
            "obs_radii": slots[:, 3].astype(np.float64),
            "obs_enabled": slots[:, 7].astype(np.float64),
            "obs_d_safe": slots[:, 8].astype(np.float64),
            "obs_vel": slots[:, 4:7].astype(np.float64),
            "obs_alpha": slots[:, 9].astype(np.float64),
        }

    def step_once(self, q: np.ndarray, *, obs_kwargs: dict | None = None) -> dict:
        """Advance the control kernel by one step (pure method for tests)."""
        kwargs = dict(
            q=np.asarray(q, dtype=float),
            path_state=self._path_state,
            kp_pos=float(self.get_parameter("kp_pos").value),
            kp_orient=float(self.get_parameter("kp_orient").value),
            kp_joint=float(self.get_parameter("kp_joint").value),
            q_des=np.asarray(q, dtype=float),
            nullspace_speed_limit=float(
                self.get_parameter("nullspace_speed_limit").value
            ),
            damping=float(self.get_parameter("damping").value),
        )
        if obs_kwargs:
            kwargs.update(obs_kwargs)
        result = self._loop.path_tracking_step(**kwargs)
        self._path_state = np.asarray(result.path_state, dtype=float)
        self._last_result = result
        return {
            "q_next": np.asarray(result.q_next, dtype=float),
            "u_safe": np.asarray(result.u_safe, dtype=float),
            "err_6d": np.asarray(result.err_6d, dtype=float),
            "qp_ok": bool(result.qp_ok),
            "min_obs_dist": float(result.min_obs_dist),
            "delta_slack": float(result.delta_slack),
        }

    def _control_tick(self) -> None:
        if not self._tracking_started or self._latest_q is None:
            return

        start = time.perf_counter()
        obs_kwargs = dict(self._obs_state) if self._enable_obs and self._obs_state else None
        step = self.step_once(self._latest_q, obs_kwargs=obs_kwargs)
        duration_ms = (time.perf_counter() - start) * 1000.0
        self._step_durations.append(duration_ms)
        self._latest_q = None

        q_next = step["q_next"]
        lower, upper = self._limits
        valid = np.all(np.isfinite(q_next)) and np.all(q_next >= lower - 1e-9) \
            and np.all(q_next <= upper + 1e-9)
        if not valid:
            self.get_logger().error(
                f"discarding invalid safe state: {q_next.tolist()}"
            )
            return
        if not step["qp_ok"]:
            self._qp_fail_count += 1
            self.get_logger().warn(
                f"QP failed at step {len(self._step_durations)}; "
                "holding current state"
            )

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self._joint_names)
        message.position = [float(value) for value in q_next]
        self._publisher.publish(message)

    def progress_snapshot(self) -> dict:
        """One-shot progress/latency snapshot for logs, tests and tooling."""
        durations = self._step_durations
        result = self._last_result
        if result is None:
            return {
                "tracking_started": self._tracking_started,
                "steps": 0,
                "ready": False,
                "qp_fail_count": self._qp_fail_count,
            }
        source_time = float(result.reference_source_time_s)
        if durations:
            p50 = float(np.percentile(durations, 50))
            p95 = float(np.percentile(durations, 95))
            maximum = float(np.max(durations))
        else:
            p50 = p95 = maximum = float("nan")
        return {
            "tracking_started": self._tracking_started,
            "steps": len(durations),
            "ready": True,
            "err_6d": np.asarray(result.err_6d, dtype=float),
            "pos_error_m": float(np.linalg.norm(result.err_6d[:3])),
            "orient_error_rad": float(np.linalg.norm(result.err_6d[3:])),
            "source_time_s": source_time,
            "trajectory_duration_s": self._trajectory_duration_s,
            "arc_fraction": min(
                max(source_time / self._trajectory_duration_s, 0.0), 1.0
            ),
            "cross_track_error_m": float(result.cross_track_error_m),
            "feedrate_m_s": float(result.feedrate_m_s),
            "limiting_reason_code": int(result.limiting_reason_code),
            "at_endpoint": bool(result.reference_at_endpoint),
            "qp_ok": bool(result.qp_ok),
            "delta_slack": float(result.delta_slack),
            "latency_p50_ms": p50,
            "latency_p95_ms": p95,
            "latency_max_ms": maximum,
            "qp_fail_count": self._qp_fail_count,
        }

    def _telemetry_tick(self) -> None:
        now = time.monotonic()
        if self._last_state_time is None:
            self.get_logger().warn(
                "waiting for joint states on "
                f"{self.get_parameter('joint_state_topic').value}"
            )
            return
        if now - self._last_state_time > 5.0:
            self.get_logger().warn(
                f"joint-state stream stalled for {now - self._last_state_time:.0f}s"
            )
        snapshot = self.progress_snapshot()
        if not snapshot.get("ready"):
            return
        self.get_logger().info(
            "progress "
            f"source_time={snapshot['source_time_s']:.2f}/"
            f"{snapshot['trajectory_duration_s']:.1f}s "
            f"arc={snapshot['arc_fraction']*100:.1f}% "
            f"pos_err={snapshot['pos_error_m']*1000:.3f}mm "
            f"orient_err={snapshot['orient_error_rad']*180/np.pi:.4f}deg "
            f"cross_track={snapshot['cross_track_error_m']*1000:.3f}mm "
            f"feedrate={snapshot['feedrate_m_s']:.4f}m/s "
            f"limit={snapshot['limiting_reason_code']} "
            f"qp_ok={snapshot['qp_ok']} slack={snapshot['delta_slack']:.2e} "
            f"latency p50/p95/max="
            f"{snapshot['latency_p50_ms']:.2f}/"
            f"{snapshot['latency_p95_ms']:.2f}/"
            f"{snapshot['latency_max_ms']:.2f}ms "
            f"qp_fail={snapshot['qp_fail_count']}"
        )
        if snapshot["at_endpoint"] and not self._completion_logged:
            self.get_logger().info(
                f"TRAJECTORY_COMPLETE: {self._trajectory_duration_s:.1f}s "
                f"tracked in {snapshot['steps']} steps"
            )
            self._completion_logged = True

    def _start_tracking_callback(self, request, response):
        if not self._tracking_started:
            self._tracking_started = True
            self._path_state = self._loop.initial_path_state()
            self.get_logger().info(
                "TRACKING_STARTED: beginning path tracking from the current "
                "plant state"
            )
            response.success = True
            response.message = "TRACKING_STARTED"
        else:
            response.success = True
            response.message = "ALREADY_TRACKING"
        return response

    def write_perf_report(self) -> None:
        """Write the M10 performance evidence file (p95 step latency)."""
        report_path = Path(str(self.get_parameter("perf_report_path").value))
        if not report_path.is_absolute():
            report_path = Path.cwd() / report_path
        report_path.parent.mkdir(parents=True, exist_ok=True)
        if self._step_durations:
            p95 = float(np.percentile(self._step_durations, 95))
            p50 = float(np.percentile(self._step_durations, 50))
            maximum = float(np.max(self._step_durations))
        else:
            p95 = p50 = maximum = float("nan")
        report_path.write_text(
            "# M10 oscbf_controller 性能证据\n\n"
            f"- 控制频率: {self.get_parameter('publish_frequency_hz').value} Hz\n"
            f"- 步数: {len(self._step_durations)}\n"
            f"- `path_tracking_step` 延迟 p50: {p50:.3f} ms\n"
            f"- `path_tracking_step` 延迟 p95: {p95:.3f} ms（阈值 < 10 ms）\n"
            f"- 单步最大: {maximum:.3f} ms\n"
            f"- QP 失败次数: {self._qp_fail_count}\n",
            encoding="utf-8",
        )
        if rclpy.ok():
            self.get_logger().info(f"wrote performance report to {report_path}")


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = OscbfController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_perf_report()
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                # The launch framework may have already shut the context down
                # on SIGINT; the controller itself has exited cleanly.
                pass


if __name__ == "__main__":
    main()
