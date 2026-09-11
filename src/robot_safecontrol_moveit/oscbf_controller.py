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
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger

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
    state_stream_qos,
)
from .tracking_evaluator import TrackingEvaluator


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
        self._declare_parameters(self._runtime_config)
        self.add_on_set_parameters_callback(
            self._reject_runtime_configuration_changes
        )
        self._step_durations: List[float] = []
        self._qp_fail_count = 0
        self._latest_q: Optional[np.ndarray] = None
        self._received_any_state = False
        self._last_state_time: Optional[float] = None
        self._last_result = None
        self._trajectory_duration_s = 30.0
        self._completion_logged = False
        self._hold_q: Optional[np.ndarray] = None
        self._hold_reported = False
        self._stall_since: Optional[float] = None
        self._last_pos_err: Optional[float] = None
        self._q_cmd_smooth: Optional[np.ndarray] = None
        self._last_state0: Optional[float] = None
        # (monotonic, pos_err) 滚动 5 s 窗口, 用于卡死增强判据。
        from collections import deque
        self._pos_err_hist: deque = deque()
        self._src_hist: deque = deque()
        self._tracking_started = not bool(
            self._runtime_config["wait_for_start"]
        )
        self._log_throttle = 0.0
        self._evaluator: TrackingEvaluator | None = None

        portable_root = Path(
            str(self._runtime_config["portable_oscbf_root"])
        )
        bootstrap_portable(portable_root)
        self._build_controller(portable_root)

        # This snapshot is a startup gate.  Command-facing ROS entities must
        # not exist unless the exact effective configuration can be persisted.
        self.runtime_snapshot_path = self._write_runtime_snapshot(portable_root)

        joint_state_topic = str(self._runtime_config["joint_state_topic"])
        publish_topic = str(self._runtime_config["publish_joint_state_topic"])
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

        if not self._tracking_started:
            self._start_service = self.create_service(
                Trigger, "/oscbf_controller/start_tracking",
                self._start_tracking_callback,
            )
        else:
            self._start_service = None
        self.get_logger().info(
            "oscbf_controller ready: trajectory="
            f"{self._runtime_config['trajectory_mat']}, "
            f"subscribe={joint_state_topic}, publish={publish_topic} @ "
            f"{float(self._runtime_config['publish_frequency_hz']):.1f} Hz, "
            f"tracking={'auto-start' if self._tracking_started else 'waiting for /oscbf_controller/start_tracking'}"
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
        return diagnostics

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
            "production_config": {
                "path": str(self._production_profile.path),
                "sha256": sha256_bytes(self._production_profile.content),
            },
            "software": software,
            "geometry": geometry,
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
        self._path_state = self._loop.initial_path_state()
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
            kp_pos=float(self._runtime_config["kp_pos"]),
            kp_orient=float(self._runtime_config["kp_orient"]),
            kp_joint=float(self._runtime_config["kp_joint"]),
            q_des=np.asarray(q, dtype=float),
            nullspace_speed_limit=float(
                self._runtime_config["nullspace_speed_limit"]
            ),
            damping=float(self._runtime_config["damping"]),
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
            "ee_pos": np.asarray(result.ee_pos, dtype=float),
            "reference_position_m": np.asarray(
                result.reference_position_m, dtype=float),
            "qp_ok": bool(result.qp_ok),
            "min_obs_dist": float(result.min_obs_dist),
            "delta_slack": float(result.delta_slack),
            # TrackingEvaluator 依赖这些键计算完成度/横偏/速率统计,
            # 缺失会让 report 的 done=0% 而 progress 行却显示 100%。
            "reference_source_time_s": float(result.reference_source_time_s),
            "reference_at_endpoint": bool(result.reference_at_endpoint),
            "cross_track_error_m": float(result.cross_track_error_m),
            "feedrate_m_s": float(result.feedrate_m_s),
            "feedrate_nominal_m_s": float(result.feedrate_nominal_m_s),
            "feedrate_joint_limit_m_s": float(result.feedrate_joint_limit_m_s),
            "feedrate_cbf_limit_m_s": float(result.feedrate_cbf_limit_m_s),
            "feedrate_rate_limit_m_s": float(result.feedrate_rate_limit_m_s),
            "feedrate_tool_axis_limit_m_s": float(
                result.feedrate_tool_axis_limit_m_s),
            "feedrate_endpoint_brake_limit_m_s": float(
                result.feedrate_endpoint_brake_limit_m_s),
            "gamma": float(result.gamma),
            "limiting_reason_code": int(result.limiting_reason_code),
        }

    def _control_tick(self) -> None:
        if not self._tracking_started or self._latest_q is None:
            return

        # 完成后冻结: 不再运行反馈循环, 只把最终位姿保持发布给执行器。
        # 路径到达端点后进给前馈消失, 高增益位置反馈 (kp_pos=80) 与 plant
        # 自身位置环 (kp=80) 串联会自激振荡 (观测: 完成后 pos_err 5mm →
        # 570mm), 冻结命令是行为上的硬闸门。
        if self._hold_q is not None:
            self._publish_positions(self._hold_q)
            return

        # 首次跟踪步：初始化评价器
        if self._evaluator is None:
            self._evaluator = TrackingEvaluator(
                trajectory_duration_s=self._trajectory_duration_s)

        start = time.perf_counter()
        q_now = np.asarray(self._latest_q, dtype=float)
        obs_kwargs = dict(self._obs_state) if self._enable_obs and self._obs_state else None
        step = self.step_once(q_now, obs_kwargs=obs_kwargs)
        duration_ms = (time.perf_counter() - start) * 1000.0
        self._step_durations.append(duration_ms)
        self._latest_q = None

        # 累积跟踪指标
        self._evaluator.update(step, wall_time_s=start)

        lower, upper = self._limits
        # 跳变诊断: 误差与上一步相比 >0.25 m 时, 记录一步的完整输入输出,
        # 用于区分 "参考跳变" 与 "命令跳变" (观测: 4s 处一秒内 7mm→967mm)。
        pos_err_now = float(np.linalg.norm(step["err_6d"][:3]))
        if (self._last_pos_err is not None
                and abs(pos_err_now - self._last_pos_err) > 0.25):
            self.get_logger().warning(
                f"POS_JUMP d_err={pos_err_now - self._last_pos_err:.3f}m "
                f"u_safe=[{', '.join(f'{v:.3f}' for v in step['u_safe'])}] "
                f"q_next-qmax={np.max(np.abs(step['q_next'] - q_now)):.4f} "
                f"ee=[{', '.join(f'{v:.3f}' for v in step['ee_pos'])}] "
                f"ref=[{', '.join(f'{v:.3f}' for v in step['reference_position_m'])}]"
            )
        self._last_pos_err = pos_err_now
        # 进给分项诊断: 每 300 步打印各 cap, 直接观察是谁在压进给。
        if len(self._step_durations) % 300 == 0:
            state0 = float(self._path_state[0])
            state1 = float(self._path_state[1])
            delta0 = (state0 - self._last_state0
                      if self._last_state0 is not None else float("nan"))
            self._last_state0 = state0
            self.get_logger().info(
                f"DETAIL steps={len(self._step_durations)} "
                f"prog={state0:.6f} proj={state1:.6f} "
                f"lead={state0 - state1:.6f} dprog300={delta0:.6f} "
                f"feed={step['feedrate_m_s']:.5f} "
                f"nom={step['feedrate_nominal_m_s']:.5f} "
                f"gamma={step['gamma']:.3f} lim={step['limiting_reason_code']} "
                f"cap_j={step['feedrate_joint_limit_m_s'] if step['feedrate_joint_limit_m_s'] < 1e9 else 9.99:.4f} "
                f"cap_cbf={step['feedrate_cbf_limit_m_s'] if step['feedrate_cbf_limit_m_s'] < 1e9 else 9.99:.4f} "
                f"cap_rate={step['feedrate_rate_limit_m_s'] if step['feedrate_rate_limit_m_s'] < 1e9 else 9.99:.4f} "
                f"cap_tool={step['feedrate_tool_axis_limit_m_s'] if step['feedrate_tool_axis_limit_m_s'] < 1e9 else 9.99:.4f} "
                f"cap_brake={step['feedrate_endpoint_brake_limit_m_s']:.4f} "
                f"u_max={float(np.max(np.abs(step['u_safe']))):.5f} "
                f"dq_max={float(np.max(np.abs(step['q_next'] - q_now))):.6f} "
                f"radial={self._radial_error_m(step['ee_pos'])*1e3:.2f}mm "
                f"ref_radial={self._radial_error_m(step['reference_position_m'])*1e3:.2f}mm "
                f"cap_cbf={step['feedrate_cbf_limit_m_s'] if step['feedrate_cbf_limit_m_s'] < 1e9 else 9.99:.4f} "
                f"src={step['reference_source_time_s']:.4f}"
            )
        # 卡死检测: 参考进给归零且横断误差持续超限 (再紧的非端点位置)
        # 时, 反馈拉回与参考停滞会形成长期摆动; 连续超过 1 s 即冻结,
        # 行为与完成冻结一致 (安全胜过继续挣扎)。
        if (not bool(step["reference_at_endpoint"])
                and float(step["feedrate_m_s"]) <= 1e-3
                and float(step["cross_track_error_m"]) > 5e-3):
            if self._stall_since is None:
                self._stall_since = start
            elif start - self._stall_since > 1.0:
                hold = np.clip(q_now, lower, upper)
                self._hold_q = hold
                self.get_logger().warn(
                    "TRACKING_STALLED: reference feedrate=0 with cross-track "
                    f"={float(step['cross_track_error_m'])*1e3:.1f}mm for "
                    f"{start - self._stall_since:.2f}s; holding current pose"
                )
                self._publish_positions(hold)
                return
        else:
            self._stall_since = None

        # 卡死增强判据: 参考源时间 5 s 不变 (参考完全停滞) + 低进给,
        # 说明系统停在 "参考停走 + 末端无法收回" 的等待态 (如 CBF 曲率段
        # 封顶), 与横断超限判据互补。误差爬升率在卡死后只有 1-2 mm/s,
        # 因此不能依赖误差阈值。
        self._pos_err_hist.append((start, pos_err_now))
        self._src_hist.append((start, float(step["reference_source_time_s"])))
        while self._pos_err_hist and start - self._pos_err_hist[0][0] > 5.0:
            self._pos_err_hist.popleft()
        while self._src_hist and start - self._src_hist[0][0] > 5.0:
            self._src_hist.popleft()
        if (self._hold_q is None
                and len(self._src_hist) >= 2
                and float(step["feedrate_m_s"]) < 0.05
                and float(step["reference_source_time_s"])
                - self._src_hist[0][1] < 0.01
                and pos_err_now > 0.005):
            hold = np.clip(q_now, lower, upper)
            self._hold_q = hold
            self.get_logger().warn(
                "TRACKING_STALLED: reference source-time frozen for 5s "
                f"(feed={float(step['feedrate_m_s']):.4f}m/s, "
                f"pos_err={pos_err_now*1e3:.1f}mm); holding current pose"
            )
            self._publish_positions(hold)
            return

        if bool(step["reference_at_endpoint"]) and self._hold_q is None:
            hold = np.clip(q_now, lower, upper)
            self._hold_q = hold
            self.get_logger().info(
                "END_OF_TRACKING: holding final pose "
                f"pos_err={float(np.linalg.norm(step['err_6d'][:3]))*1e3:.1f}mm"
            )
            self._publish_positions(hold)
            return

        q_next = step["q_next"]
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

        self._publish_positions(q_next)

    def _publish_positions(self, positions: np.ndarray) -> None:
        # 一阶低通 (tau=0.02 s): QP 输出逐 tick 的高频微抖 (20-50 Hz,
        # 0.1-0.9 rad/s) 直接下发对电机是颤振; 平滑后仅引入约 2-3 tick
        # 相位滞后, 由位置环与参考前馈吸收, 稳态无偏差。
        dt = 1.0 / float(self._runtime_config["publish_frequency_hz"])
        alpha = dt / (dt + 0.02)
        target = np.asarray(positions, dtype=float)
        if self._q_cmd_smooth is None:
            self._q_cmd_smooth = target.copy()
        else:
            self._q_cmd_smooth = (
                self._q_cmd_smooth + alpha * (target - self._q_cmd_smooth))
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self._joint_names)
        message.position = [float(value) for value in self._q_cmd_smooth]
        self._publisher.publish(message)

    def _radial_error_m(self, ee_pos: np.ndarray) -> float:
        """径向偏差: 末端到圆柱轴线的距离 - 半径 (负=侵入表面内部)。"""
        if self._surface_centre is None:
            return float("nan")
        rel = np.asarray(ee_pos, dtype=float) - self._surface_centre
        axial = self._surface_axis * float(np.dot(rel, self._surface_axis))
        radial = rel - axial
        return float(np.linalg.norm(radial) - self._surface_radius)

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
            "path_progress_m": float(np.asarray(result.path_state)[0]),
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
        if self._hold_q is not None:
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
            report = self._evaluator.report() if self._evaluator is not None else None
            done_pct = (
                report.completion_fraction * 100.0 if report is not None else 0.0
            )
            self.get_logger().info(
                f"TRAJECTORY_COMPLETE: done={done_pct:.1f}% "
                f"duration={self._trajectory_duration_s:.1f}s "
                f"steps={snapshot['steps']} "
                f"at_endpoint={snapshot['at_endpoint']}"
            )
            # 输出跟踪评价报告摘要
            if report is not None:
                self.get_logger().info(
                    f"TRACKING_REPORT: {report.summary()}")
                # 写入报告文件
                try:
                    self.write_tracking_report()
                except Exception as exc:
                    self.get_logger().warn(f"failed to write tracking report: {exc}")

    def tracking_report(self):
        """返回跟踪评价报告（TrackingReport），未开始跟踪时返回 None。"""
        if self._evaluator is None:
            return None
        return self._evaluator.report()

    def write_tracking_report(self, path: str | None = None) -> str:
        """写入 Markdown 格式的跟踪评价报告，返回文件路径。"""
        from pathlib import Path
        report = self._evaluator.report()
        if path is None:
            path = str(Path(self._runtime_config["perf_report_path"]).parent
                       / "tracking_report.md")
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(report.markdown(), encoding="utf-8")
        self.get_logger().info(f"tracking report written to {path}")
        return path

    def _start_tracking_callback(self, request, response):
        if not self._tracking_started:
            self._tracking_started = True
            self._path_state = self._loop.initial_path_state()
            self._hold_q = None
            self._hold_reported = False
            self._completion_logged = False
            self._stall_since = None
            self._q_cmd_smooth = None
            self._last_state0 = None
            self._pos_err_hist.clear()
            self._src_hist.clear()
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
        report_path = Path(str(self._runtime_config["perf_report_path"]))
        if not report_path.is_absolute():
            report_path = Path.cwd() / report_path
        report_path.parent.mkdir(parents=True, exist_ok=True)
        if self._step_durations:
            p95 = float(np.percentile(self._step_durations, 95))
            p50 = float(np.percentile(self._step_durations, 50))
            maximum = float(np.max(self._step_durations))
            budget = float(self._runtime_config["latency_budget_ms"])
            miss_count = int(np.count_nonzero(
                np.asarray(self._step_durations) > budget))
            miss_rate = miss_count / len(self._step_durations)
        else:
            p95 = p50 = maximum = float("nan")
            budget = float(self._runtime_config["latency_budget_ms"])
            miss_count = 0
            miss_rate = 0.0
        report_path.write_text(
            "# M10 oscbf_controller 性能证据\n\n"
            f"- 控制频率: {self._runtime_config['publish_frequency_hz']} Hz\n"
            f"- 步数: {len(self._step_durations)}\n"
            f"- `path_tracking_step` 延迟 p50: {p50:.3f} ms\n"
            f"- `path_tracking_step` 延迟 p95: {p95:.3f} ms（01B 口径: "
            f"50Hz 预算 = {budget:.0f} ms）\n"
            f"- 单步最大: {maximum:.3f} ms\n"
            f"- 超预算(>{budget:.0f}ms)步数: {miss_count} "
            f"(miss rate = {miss_rate * 100:.2f}%, 上限 1%)\n"
            f"- QP 失败次数: {self._qp_fail_count}\n",
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
