"""OSCBF safe-control ROS 2 node (M10).

The node is deliberately independent of MoveIt: it loads the repository
trajectory, runs the pure-JAX OSCBF control kernel, and publishes the safe
joint command onto the dedicated command stream (``/oscbf_command``), separate
from the plant state stream it subscribes to.  The ``portable_oscbf`` source
tree is shipped in the package share directory; the node adds it (and the
vendored ``dpax``) to ``sys.path`` before importing ``work``.
"""

from __future__ import annotations

import time
from dataclasses import replace
from pathlib import Path
from threading import Lock
from typing import List, Optional, Sequence, TYPE_CHECKING

import numpy as np
import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from rclpy.exceptions import InvalidTopicNameException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger

if TYPE_CHECKING:
    # ``work`` is importable only after ``bootstrap_portable`` extends
    # ``sys.path``; the annotation is a string, so this never runs.
    from work.jax_control_facade import JaxPathTrackingResult

from .oscbf_trajectory import bootstrap_portable
from .production_config import (
    MANAGED_PARAMETERS,
    build_effective_configuration,
    load_production_profile,
)
from .runtime_snapshot import (
    collect_software_identity,
    persist_runtime_snapshot,
    sha256_bytes,
)
from .ros_conventions import (
    command_stream_qos,
    state_stream_qos,
)
from .tracking_contract import EvidenceContext
from .tracking_run import TrackingRun


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
        share_dir = _default_share_dir()
        self.declare_parameter(
            "production_config_yaml",
            str(share_dir / "config" / "oscbf_controller.yaml"),
        )
        self._production_profile = load_production_profile(
            Path(str(self.get_parameter("production_config_yaml").value)),
            node_name=node_name,
        )
        explicit_overrides = {
            name: parameter.value
            for name, parameter in self._parameter_overrides.items()
            if name != "production_config_yaml"
            and name != "use_sim_time"
            and not name.startswith("qos_overrides.")
        }
        self._effective = build_effective_configuration(
            self._production_profile,
            explicit_overrides,
            share_dir=share_dir,
        )
        self._runtime_config = self._effective.values
        self._topic_connections = self._resolve_topic_connections()
        self._declare_parameters(self._runtime_config)
        self.add_on_set_parameters_callback(
            self._reject_runtime_configuration_changes
        )
        self._last_state_time: Optional[float] = None
        self._trajectory_duration_s = 30.0
        self._completion_logged = False
        self._hold_reported = False
        self._reported_writer_state = "idle"
        self._control_lock = Lock()
        self._closing = False

        portable_root = Path(
            str(self._runtime_config["portable_oscbf_root"])
        )
        bootstrap_portable(portable_root)
        self._build_controller(portable_root)

        # This snapshot is a startup gate.  Command-facing ROS entities must
        # not exist unless the exact effective configuration can be persisted.
        self.runtime_snapshot_path = self._write_runtime_snapshot(portable_root)
        snapshot = str(self.runtime_snapshot_path)
        self._run = TrackingRun(
            loop=self._loop, geometry=self._evaluation_geometry,
            config=self._runtime_config, trajectory_duration_s=self._trajectory_duration_s,
            evidence=EvidenceContext(
                run_id=self.runtime_snapshot_path.stem, kind="model", boundary="kernel_candidate",
                model_id=snapshot + "#software", config_id=snapshot,
                trajectory_id="sha256:" + self._evaluation_geometry_hash,
                data_id=snapshot + "#kernel_step_sequence",
                scenario="configured path; obstacles=" + str(bool(self._runtime_config["enable_perception_obstacles"])),
                measurement="post-integration model q_next and command reference; QP rows at solve input; before command filter; no execution feedback",
                time_basis="perf_counter sample start; step_once includes kernel and host diagnostics",
            ),
            log=self.get_logger(),
            surface=(self._surface_axis, self._surface_centre, self._surface_radius)
            if self._surface_centre is not None else None,
        )

        joint_state_topic = str(self._runtime_config["joint_state_topic"])
        publish_topic = str(self._runtime_config["publish_joint_state_topic"])
        # Subscribe to the configured joint-state stream using the canonical
        # state-stream QoS.
        self.create_subscription(
            JointState,
            joint_state_topic,
            self._joint_state_callback,
            state_stream_qos(),
        )
        self._publisher = self.create_publisher(
            JointState, publish_topic, command_stream_qos()
        )
        period_s = 1.0 / float(
            self._runtime_config["publish_frequency_hz"]
        )
        self._timer = self.create_timer(period_s, self._control_tick)
        self._telemetry_timer = self.create_timer(
            float(self._runtime_config["telemetry_period_s"]),
            self._telemetry_tick,
        )

        # 感知障碍物订阅（默认 disabled，不影响现有行为）
        self._obs_state: dict = {}
        self._enable_obs = bool(
            self._runtime_config["enable_perception_obstacles"])
        if self._enable_obs:
            tracks_topic = str(self._runtime_config["perception_tracks_topic"])
            self.create_subscription(
                Float32MultiArray, tracks_topic,
                self._tracks_callback, qos_profile_sensor_data)
            self.get_logger().info(f"perception obstacles enabled: {tracks_topic}")

        if self.execution_state == "waiting":
            self._start_service = self.create_service(
                Trigger, "/oscbf_controller/start_tracking",
                self._start_tracking_callback,
            )
        else:
            self._start_service = None
        self.get_logger().info(
            "oscbf_controller ready: trajectory="
            f"{self._runtime_config['trajectory_mat']}, "
            "subscribe="
            f"{self._topic_connections['joint_state_topic']['resolved_topic']}, "
            "publish="
            f"{self._topic_connections['publish_joint_state_topic']['resolved_topic']} @ "
            f"{float(self._runtime_config['publish_frequency_hz']):.1f} Hz, "
            f"tracking={'auto-start' if self.execution_state == 'tracking' else 'waiting for /oscbf_controller/start_tracking'}"
        )

    # ------------------------------------------------------------------
    # Parameter handling
    # ------------------------------------------------------------------

    def _declare_parameters(self, defaults) -> None:
        for name, value in defaults.items():
            self.declare_parameter(name, value, ignore_override=True)

    def effective_configuration(self) -> dict:
        """Return the fixed startup values and provenance for diagnostics."""
        diagnostics = self._effective.diagnostics()
        diagnostics["production_config_path"] = str(self._production_profile.path)
        diagnostics["topic_connections"] = {
            name: dict(connection)
            for name, connection in self._topic_connections.items()
        }
        return diagnostics

    def _resolve_topic_connections(self) -> dict[str, dict[str, str]]:
        """Validate and record the final ROS topic names after remapping."""
        connections: dict[str, dict[str, str]] = {}
        for name in (
            "joint_state_topic",
            "publish_joint_state_topic",
            "perception_tracks_topic",
        ):
            raw = str(self._runtime_config[name])
            try:
                resolved = self.resolve_topic_name(raw)
            except InvalidTopicNameException as exc:
                raise ValueError(
                    f"{name} must be a valid ROS topic: {raw!r}: {exc}"
                ) from exc
            connections[name] = {
                "raw_value": raw,
                "resolved_topic": resolved,
            }

        state = connections["joint_state_topic"]["resolved_topic"]
        command = connections["publish_joint_state_topic"]["resolved_topic"]
        if state == command:
            raise ValueError(
                "final state and command topics must differ: " + state
            )
        perception = connections["perception_tracks_topic"]["resolved_topic"]
        if bool(self._runtime_config["enable_perception_obstacles"]) and (
            perception == state or perception == command
        ):
            raise ValueError(
                "enabled perception topic must differ from state and command topics: "
                + perception
            )
        return connections

    def _reject_runtime_configuration_changes(self, parameters) -> SetParametersResult:
        managed = sorted(
            parameter.name
            for parameter in parameters
            if parameter.name in MANAGED_PARAMETERS
            or parameter.name == "production_config_yaml"
        )
        if managed:
            return SetParametersResult(
                successful=False,
                reason=(
                    "managed production parameters are immutable at runtime; "
                    "restart with an explicit override: " + ", ".join(managed)
                ),
            )
        return SetParametersResult(successful=True)

    def runtime_configuration_diagnostics(self) -> dict:
        """Report values at the real facade, tracking and timer consumers."""
        return {
            "facade": {
                "dt": self._loop.dt,
                "dt_path": self._loop.dt_path,
                "w_pos": self._loop.w_pos,
                "w_orient": self._loop.w_orient,
                "w_joint": self._loop.w_joint,
                "temporal_lambda": self._loop.temporal_lambda,
                "enable_x64": self._loop.enable_x64,
                "solver_tol": self._loop.solver_tol,
                "task_mode": self._loop.task_mode,
            },
            "path_tracking_step": {
                name: self._runtime_config[name]
                for name in (
                    "kp_pos",
                    "kp_orient",
                    "kp_joint",
                    "nullspace_speed_limit",
                    "damping",
                )
            },
            "timer_period_s": self._timer.timer_period_ns / 1.0e9,
            "telemetry_period_s": self._telemetry_timer.timer_period_ns / 1.0e9,
        }

    def _write_runtime_snapshot(self, portable_root: Path) -> Path:
        """Persist the configuration consumed by the warmed control kernel."""
        perf_path = Path(str(self._runtime_config["perf_report_path"]))
        if not perf_path.is_absolute():
            perf_path = Path.cwd() / perf_path
        snapshot_dir = perf_path.resolve().parent / "runtime_snapshots"

        center = list(self._runtime_config["cylinder_center"])
        if self._surface_centre is not None:
            resolved_center = self._surface_centre.tolist()
            center_source = "trajectory_fit" if not center else "explicit_config"
        else:
            resolved_center = center or None
            center_source = "explicit_config" if center else "unavailable"
        geometry = {
            "cylinder_center_source": center_source,
            "resolved_center": resolved_center,
            "resolved_axis_direction": (
                self._surface_axis.tolist()
                if self._surface_axis is not None
                else list(self._runtime_config["cylinder_axis_direction"])
            ),
            "resolved_radius_m": self._surface_radius,
        }

        repository_hint = Path(__file__).resolve().parents[2]
        software = collect_software_identity(
            repository_hint=repository_hint,
            source_paths=(
                Path(__file__),
                Path(__file__).with_name("production_config.py"),
                Path(__file__).with_name("runtime_snapshot.py"),
                Path(__file__).with_name("tracking_evaluator.py"),
                Path(__file__).with_name("tracking_contract.py"),
                Path(__file__).with_name("tracking_report_writer.py"),
                Path(__file__).with_name("tracking_run.py"),
                portable_root / "work",
            ),
        )
        effective = self._effective.diagnostics()
        payload = {
            "schema_version": 1,
            "node_name": self.get_name(),
            "final_values": effective["values"],
            "parameter_sources": effective["sources"],
            "override_chains": effective["override_chains"],
            "resources": effective["resources"],
            "topic_connections": {
                name: dict(connection)
                for name, connection in self._topic_connections.items()
            },
            "production_config": {
                "path": str(self._production_profile.path),
                "sha256": sha256_bytes(self._production_profile.content),
            },
            "software": software,
            "geometry": geometry,
            "evaluation": {
                "trajectory_geometry_sha256": self._evaluation_geometry_hash,
                "total_length_m": self._evaluation_geometry.total_length_m,
                "path_array_dtype": self._evaluation_path_dtype,
                "boundary": "kernel_candidate",
                "scope": "full_path",
            },
            "obstacle_alpha_contract": {
                "baseline_value": float(
                    self._loop.obstacle_h_baseline_alpha
                ),
                "baseline_source": "NineaxisOSCBFVelocityConfig",
                "runtime_value_source": "perception track slot 10",
            },
        }
        try:
            snapshot_path = persist_runtime_snapshot(payload, snapshot_dir)
        except Exception as exc:
            raise RuntimeError(
                f"runtime snapshot persistence failed: {exc}"
            ) from exc
        self.get_logger().info(
            "runtime configuration locked: "
            f"dt={self._runtime_config['dt']}, "
            f"dt_path={self._runtime_config['dt_path']}, "
            f"publish_frequency_hz={self._runtime_config['publish_frequency_hz']}; "
            f"snapshot={snapshot_path}"
        )
        return snapshot_path

    # ------------------------------------------------------------------
    # Controller construction
    # ------------------------------------------------------------------

    def _build_controller(self, portable_root: Path) -> None:
        from work.ik_data_loader import load_repository_trajectory
        from work.jax_control_facade import JaxControlLoop
        from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
        from work.nullspace_policy import ManipulabilityGradientPolicy
        from work.path_following import PathFollowingConfig
        import jax

        # Optional policy construction also creates JAX robot constants.
        jax.config.update('jax_enable_x64', bool(self._runtime_config["enable_x64"]))

        self.get_logger().info("Loading repository trajectory ...")
        trajectory_mat = str(self._runtime_config["trajectory_mat"])
        config_yaml = str(self._runtime_config["portable_config_yaml"])
        trajectory = load_repository_trajectory(
            trajectory_mat,
            config_yaml_path=config_yaml,
            feedrate_scale=float(
                self._runtime_config["reference_feedrate_scale"]
            ),
        )
        self._trajectory_duration_s = float(
            trajectory.num_points * trajectory.Ts
        )
        orientation_mode = str(
            self._runtime_config["orientation_mode"]
        )
        if orientation_mode == "surface_normal":
            center = list(self._runtime_config["cylinder_center"])
            axis_point = (
                None if len(center) == 0 else center
            )
            trajectory.set_surface_normal_orientation(
                self._runtime_config["cylinder_axis_direction"],
                axis_point=axis_point,
            )
        elif orientation_mode != "fixed":
            raise ValueError(
                "orientation_mode must be 'fixed' or 'surface_normal', "
                f"got {orientation_mode!r}"
            )
        geometry = trajectory.path_geometry()

        policy = None
        if bool(self._runtime_config["use_nullspace_policy"]):
            robot = NineaxisManipulatorJAX()
            policy = ManipulabilityGradientPolicy(robot)

        self._loop = JaxControlLoop(
            dt=float(self._runtime_config["dt"]),
            dt_path=float(self._runtime_config["dt_path"]),
            w_pos=float(self._runtime_config["w_pos"]),
            w_orient=float(self._runtime_config["w_orient"]),
            w_joint=float(self._runtime_config["w_joint"]),
            temporal_lambda=float(self._runtime_config["temporal_lambda"]),
            enable_x64=bool(self._runtime_config["enable_x64"]),
            solver_tol=float(self._runtime_config["solver_tol"]),
            task_mode=str(self._runtime_config["task_mode"]),
            nullspace_policy=policy,
        )
        self._loop.configure_path(
            geometry,
            PathFollowingConfig(
                reference_lead_m=float(
                    self._runtime_config["reference_lead_m"]
                ),
                maximum_tool_axis_speed_rad_s=float(
                    self._runtime_config["max_tool_axis_speed_rad_s"]
                ),
            ),
        )
        arrays = self._loop.path_geometry_arrays()
        self._evaluation_path_dtype = str(arrays["arc_length_m"].dtype)
        # Widen the actual kernel values losslessly for host projection. This
        # also binds explicit float32 runs to their real quantized path length.
        self._evaluation_geometry = replace(geometry, **{
            name: np.asarray(value, dtype=float) for name, value in arrays.items()
        })
        self._evaluation_geometry_hash = sha256_bytes(b"".join(
            np.asarray(arrays[name], dtype="<f8").tobytes() for name in sorted(arrays)
        ))
        self.get_logger().info("Warming up the JAX control kernel ...")
        self._loop.init_cbf()
        # 拟合圆柱几何 (surface_normal 模式下由轨迹数据拟合), 供径向
        # 侵入诊断使用: 末端到轴线的径向距离减去半径, 负值=侵入内部。
        self._surface_axis = None
        self._surface_centre = None
        self._surface_radius = None
        if hasattr(trajectory, "surface_centre"):
            self._surface_axis = np.asarray(trajectory.surface_axis, dtype=float)
            self._surface_centre = np.asarray(trajectory.surface_centre, dtype=float)
            self._surface_radius = float(trajectory.surface_radius)
        self.get_logger().info("JAX control kernel warm-up complete")
        self._joint_names = [
            str(name) for name in self._runtime_config["joint_names"]
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
        with self._control_lock:
            if self._closing:
                return
            self._run.receive_state(positions)
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

    def step_once(self, q: np.ndarray, *, obs_kwargs: dict | None = None
                  ) -> "JaxPathTrackingResult":
        """执行控制步；轨迹状态和距离测量语义由 TrackingRun 管理。"""
        with self._control_lock:
            return self._run.step_once(q, obs_kwargs=obs_kwargs)

    @property
    def execution_state(self) -> str:
        return self._run.execution_state

    def _poll_tracking_report(self) -> None:
        status = self._run.report_status()
        if status["state"] == self._reported_writer_state:
            return
        self._reported_writer_state = status["state"]
        if status["state"] == "failed":
            self.get_logger().error(f"tracking report failed: {status['error']}")
        else:
            self.get_logger().info(f"tracking report {status['state']}: {status['path']}")

    def _control_tick(self) -> None:
        with self._control_lock:
            if self._closing:
                return
            self._run.tick(self._publish_positions, obs_kwargs=self._obs_state)
            self._poll_tracking_report()

    def _publish_positions(self, positions: np.ndarray) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self._joint_names)
        message.position = [float(value) for value in positions]
        self._publisher.publish(message)

    def progress_snapshot(self) -> dict:
        """One-shot progress/latency snapshot for logs, tests and tooling."""
        return self._run.progress_snapshot()

    def _telemetry_tick(self) -> None:
        if self._closing:
            return
        # Poll before any hold/stale-state return so persistence failures remain
        # visible even after sampling and feedback have stopped.
        self._poll_tracking_report()
        now = time.monotonic()
        if self._last_state_time is None:
            self.get_logger().warn(
                "waiting for joint states on "
                f"{self._runtime_config['joint_state_topic']}"
            )
            return
        if now - self._last_state_time > 5.0:
            self.get_logger().warn(
                f"joint-state stream stalled for {now - self._last_state_time:.0f}s"
            )
        snapshot = self.progress_snapshot()
        if not snapshot.get("ready"):
            return
        if self.execution_state == "holding":
            if not self._hold_reported:
                self._hold_reported = True
                self.get_logger().info(
                    "HELD: tracking frozen at final pose, holding command active"
                )
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
            f"prog={snapshot['path_progress_m']:.4f}m steps={snapshot['steps']} "
            f"limit={snapshot['limiting_reason_code']} "
            f"qp_ok={snapshot['qp_ok']} slack={snapshot['delta_slack']:.2e} "
            f"latency p50/p95/max="
            f"{snapshot['latency_p50_ms']:.2f}/"
            f"{snapshot['latency_p95_ms']:.2f}/"
            f"{snapshot['latency_max_ms']:.2f}ms "
            f"qp_fail={snapshot['qp_fail_count']}"
        )
        if snapshot["at_endpoint"] and not self._completion_logged:
            self._completion_logged = True
            done_pct = snapshot["arc_fraction"] * 100.0
            self.get_logger().info(
                f"REFERENCE_AT_ENDPOINT: projected_arc={done_pct:.1f}% "
                f"duration={self._trajectory_duration_s:.1f}s "
                f"steps={snapshot['steps']} "
                f"at_endpoint={snapshot['at_endpoint']}"
            )

    def tracking_report(self):
        """返回跟踪评价报告（TrackingReport），未开始跟踪时返回 None。"""
        return self._run.tracking_report()

    def write_tracking_report(self, path: str | None = None) -> str:
        """Synchronous export for explicit callers outside ROS callbacks."""
        path = self._run.write_tracking_report(path)
        self.get_logger().info(f"tracking report written to {path}")
        return path

    def destroy_node(self):
        try:
            self._timer.cancel()
            self._telemetry_timer.cancel()
            with self._control_lock:
                self._closing = True
                self._run.close()
            self._poll_tracking_report()
        finally:
            destroyed = super().destroy_node()
        return destroyed

    def _start_tracking_callback(self, request, response):
        with self._control_lock:
            if self._closing:
                response.success = False
                response.message = "TRACKING_CLOSED"
                return response
            response.message = self._run.start()
        response.success = True
        if response.message == "TRACKING_STARTED":
            self.get_logger().info(
                "TRACKING_STARTED: beginning path tracking from the current "
                "plant state"
            )
        return response

    def write_perf_report(self) -> None:
        """Write the M10 performance evidence file (p95 step latency)."""
        report_path = Path(str(self._runtime_config["perf_report_path"]))
        if not report_path.is_absolute():
            report_path = Path.cwd() / report_path
        report_path.parent.mkdir(parents=True, exist_ok=True)
        stats = self._run.performance_snapshot()
        p95, p50 = stats["latency_p95_ms"], stats["latency_p50_ms"]
        maximum, budget = stats["latency_max_ms"], stats["budget_ms"]
        miss_count, miss_rate = stats["miss_count"], stats["miss_rate"]
        report_path.write_text(
            "# M10 oscbf_controller 性能证据\n\n"
            f"- 控制频率: {self._runtime_config['publish_frequency_hz']} Hz\n"
            f"- 步数: {stats['steps']}\n"
            f"- `path_tracking_step` 延迟 p50: {p50:.3f} ms\n"
            f"- `path_tracking_step` 延迟 p95: {p95:.3f} ms（01B 口径: "
            f"50Hz 预算 = {budget:.0f} ms）\n"
            f"- 单步最大: {maximum:.3f} ms\n"
            f"- 超预算(>{budget:.0f}ms)步数: {miss_count} "
            f"(miss rate = {miss_rate * 100:.2f}%, 上限 1%)\n"
            f"- QP 失败次数: {stats['qp_fail_count']}\n",
            encoding="utf-8",
        )
        if rclpy.ok():
            self.get_logger().info(f"wrote performance report to {report_path}")


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node: OscbfController | None = None
    try:
        node = OscbfController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.write_perf_report()
            finally:
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
