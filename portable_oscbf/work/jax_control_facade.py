"""Host-side facade for the fixed-shape JAX OSCBF control kernels.

This class owns input normalisation, JIT warmup, small host diagnostics and
the stable ``JaxControlLoop`` public API.  The mathematical kernel itself is
implemented in :mod:`work.jax_kernel_factory`.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Tuple

import jax
import jax.numpy as jnp
import numpy as np

from work.cross_step_state import (
    advance_cross_step_state,
    initial_cross_step_state,
)
from work.jax_barrier_terms import MAX_JAX_OBSTACLES
from work.jax_kernel_factory import build_jax_control_kernels
from work.jax_path_following import (
    as_jax_path_config,
    as_jax_path_geometry,
    initial_path_state_jax,
)
from work.jax_posture_reference import as_jax_path_posture_reference
from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
from work.path_following import PathFollowingConfig, PathGeometry
from work.path_posture_reference import PathPostureReference
from work.tool_axis_task import SUPPORTED_TASK_MODES, TASK_MODE_POSE_6D


@dataclass
class ObstacleState:
    """Bundle obstacle + SDF inputs for JaxControlLoop methods.

    Replaces the15 individual keyword arguments that were previously passed
    to every public step method.  All fields default to "no obstacles" so
    callers only set what they need.
    """

    obs_pos: np.ndarray | None = None
    obs_radii: np.ndarray | None = None
    obs_enabled: np.ndarray | None = None
    obs_d_safe: np.ndarray | None = None
    obs_vel: np.ndarray | None = None
    obs_radius_dot: np.ndarray | None = None
    obs_alpha: np.ndarray | None = None
    u_safe_prev: np.ndarray | None = None
    sdf_distance: np.ndarray | None = None
    sdf_origin: np.ndarray | None = None
    sdf_voxel_size: float | None = None
    sdf_enabled: float = 0.0
    sdf_margin: float | None = None


@dataclass(frozen=True)
class JaxPathTrackingResult:
    """Host-readable output from one fixed-shape JAX path control step."""

    q_next: np.ndarray
    u_safe: np.ndarray
    u_nom: np.ndarray
    err_6d: np.ndarray
    ee_pos: np.ndarray
    ee_rot: np.ndarray
    qp_ok: bool
    min_obs_dist: float | None
    path_state: np.ndarray
    reference_position_m: np.ndarray
    reference_rotation: np.ndarray
    posture_reference: np.ndarray
    reference_tangent: np.ndarray
    reference_omega_per_m: np.ndarray
    reference_source_time_s: float
    cross_track_error_m: float
    gamma: float
    feedrate_nominal_m_s: float
    feedrate_m_s: float
    feedrate_joint_limit_m_s: float
    feedrate_cbf_limit_m_s: float
    feedrate_rate_limit_m_s: float
    feedrate_tool_axis_limit_m_s: float
    feedrate_endpoint_brake_limit_m_s: float
    limiting_reason_code: int
    actual_tangent_speed_m_s: float
    reference_at_endpoint: bool
    delta_slack: float = 0.0
    constraint_metrics: dict | None = None
    ee_pos_before: np.ndarray | None = None
    qp_primal_residual: float | None = None
    min_esdf_dist: float | None = None
    min_obs_dist_measured: bool = False
    min_esdf_dist_measured: bool = False


class JaxControlLoop:
    """Backward-compatible facade around the fixed-shape JAX control kernels.

    The compiled chain contains FK, Jacobian, P-only OSC, collision-sphere
    geometry, CBF-QP and simulation integration.  ROS, PointCloud2 decoding,
    ESDF construction and command transport deliberately remain outside this
    class because they are not fixed-shape numerical kernels.
    """

    def __init__(self, dt: float = 0.002,
                 dt_path: float | None = None,
                 w_pos: float = 20.0, w_orient: float = 10.0,
                 w_joint: float = 0.1,
                 temporal_lambda: float = 0.0,
                 temporal_wu: np.ndarray = None,
                 sdf_shape=None,
                 rate_limit_du_max: np.ndarray = None,
                 rate_limit_penalty: float = 1e3,
                 enable_x64: bool = True,
                 solver_tol: float = 1e-3,
                 collect_cbf_diagnostics: bool = True,
                 task_mode: str = TASK_MODE_POSE_6D,
                 joint_limit_lower: np.ndarray | None = None,
                 joint_limit_upper: np.ndarray | None = None,
                 joint_limit_cbf_margin: float | None = None,
                 nullspace_policy=None):
        self.dt = float(dt)
        # 参考路径推进步长默认与 QP 积分步一致; 控制周期 0.01s 时传 dt_path=0.01
        self.dt_path = self.dt if dt_path is None else float(dt_path)
        self.w_pos = float(w_pos)
        self.w_orient = float(w_orient)
        self.w_joint = float(w_joint)
        self.temporal_lambda = float(temporal_lambda)
        self.temporal_wu = None if temporal_wu is None else np.asarray(temporal_wu)
        self.sdf_shape = None if sdf_shape is None else tuple(int(v) for v in sdf_shape)
        self.enable_sdf = self.sdf_shape is not None
        self.aggregate_dynamic_obstacles = True
        self.rate_limit_du_max = (
            None if rate_limit_du_max is None
            else np.asarray(rate_limit_du_max, dtype=np.float32).reshape(9))
        self.rate_limit_penalty = float(rate_limit_penalty)
        self.enable_x64 = bool(enable_x64)
        # Apply precision before constructing robot constants, limits or paths.
        # Enabling x64 only in init_cbf cannot recover already-rounded arrays.
        jax.config.update('jax_enable_x64', self.enable_x64)
        self.solver_tol = float(solver_tol)
        self.task_mode = str(task_mode)
        self.nullspace_policy = nullspace_policy
        if self.task_mode not in SUPPORTED_TASK_MODES:
            raise ValueError(
                f'task_mode must be one of {SUPPORTED_TASK_MODES}, got {self.task_mode!r}')
        self.collect_cbf_diagnostics = bool(collect_cbf_diagnostics)
        self.enable_rate_limit = bool(
            self.rate_limit_du_max is not None
            and np.any(self.rate_limit_du_max > 0.0))
        if self.rate_limit_penalty <= 0.0:
            raise ValueError('rate_limit_penalty must be positive')
        if self.solver_tol <= 0.0:
            raise ValueError('solver_tol must be positive')
        if self.enable_sdf and (len(self.sdf_shape) != 3 or min(self.sdf_shape) < 2):
            raise ValueError('sdf_shape must have three dimensions >= 2')

        self.robot = NineaxisManipulatorJAX()
        self.hard_q_min = np.asarray(self.robot.joint_lower_limits, dtype=float)
        self.hard_q_max = np.asarray(self.robot.joint_upper_limits, dtype=float)
        if (joint_limit_lower is None) != (joint_limit_upper is None):
            raise ValueError(
                'joint_limit_lower and joint_limit_upper must be supplied together')
        tracking_lower = (
            self.hard_q_min if joint_limit_lower is None else
            np.asarray(joint_limit_lower, dtype=float).reshape(9))
        tracking_upper = (
            self.hard_q_max if joint_limit_upper is None else
            np.asarray(joint_limit_upper, dtype=float).reshape(9))
        if (not np.all(np.isfinite(tracking_lower))
                or not np.all(np.isfinite(tracking_upper))
                or np.any(tracking_lower >= tracking_upper)):
            raise ValueError('tracking joint-limit envelope must be finite and ordered')
        if (np.any(tracking_lower < self.hard_q_min - 1.0e-12)
                or np.any(tracking_upper > self.hard_q_max + 1.0e-12)):
            raise ValueError('tracking joint-limit envelope exceeds mechanical limits')
        self.q_min = jnp.asarray(tracking_lower)
        self.q_max = jnp.asarray(tracking_upper)
        self.joint_limit_cbf_margin = joint_limit_cbf_margin
        self.uses_inner_joint_envelope = bool(
            not np.array_equal(tracking_lower, self.hard_q_min)
            or not np.array_equal(tracking_upper, self.hard_q_max))
        self.dq_max = self.robot.joint_max_velocities

        self._cbf = None
        self._config = None
        self._default_jax_inputs = None
        self._default_jax_inputs_key = None
        self._step_fn = None
        self._tracking_fn = None
        self._tracking_fast_fn = None
        self._path_tracking_fn = None
        self._task_p_fn = None
        self._qp_problem_fn = None
        self._qp_core_fn = None
        self._warmed_up = False
        self._path_geometry = None
        self._path_config = None
        self._path_posture_reference = None

        # ``last_min_esdf_dist`` is the one measured distance that keeps a
        # compatibility mirror here: ``perception_demo`` prints it.  Its twins
        # now live on the step record only.
        self.last_min_esdf_dist = 1.0
        # The rate relaxation is carried by the step record
        # (``rate_constraint_violation``) together with qpax's raw
        # interior-point slack (``rate_solver_slack``).  Their former
        # ``last_rate_slack`` / ``last_rate_constraint_violation`` /
        # ``last_rate_solver_slack`` mirrors, and with them the documented
        # "public compatibility alias" promise, were retired: nothing outside a
        # skipped module read them.
        self.last_qp_active_count = 0
        self.last_qp_iterations = 0
        self.last_qp_primal_residual = 0.0
        self.last_qp_dual_max = 0.0
        self.last_qp_terminal_kkt_residual = 0.0
        self.last_qp_terminal_kkt_accepted = False
        self.cross_step_state = initial_cross_step_state(9)
        self.last_qp_candidate = np.zeros(9)
        self.last_path_metrics = {}

    # ``portable_oscbf/tests/test_jax_default_input_cache.py`` seeds the
    # previous command through this attribute to pin the "cacheable compiled
    # inputs, refreshed live command" contract, and the archived redundancy
    # probe does the same.  Keep it as the one writable door into the
    # cross-step state rather than storing a second copy beside it.
    @property
    def _last_u_safe(self) -> np.ndarray:
        return self.cross_step_state.u_safe_prev

    @_last_u_safe.setter
    def _last_u_safe(self, value: np.ndarray) -> None:
        self.cross_step_state = self.cross_step_state._replace(
            u_safe_prev=np.asarray(value))

    def configure_path(self, geometry: PathGeometry,
                       config: PathFollowingConfig,
                       posture_reference: np.ndarray | None = None) -> None:
        """Register immutable path data before JAX kernel construction.

        Reconfiguring a compiled controller would replace closure constants
        and compile another executable, so the operation is intentionally
        rejected after :meth:`init_cbf`.  Runtime path state remains a fixed
        five-value JAX array passed to every control step.  When supplied,
        ``posture_reference`` must have one 9-joint target for every compacted
        path point and is captured by the JIT closure.  It affects only the
        OSC null-space target; CBF topology and task dimensions do not change.
        """
        if self.is_initialized:
            raise RuntimeError('configure_path() must run before init_cbf()')
        # The ROS runner imports ``work/`` as a top-level module path while
        # tests commonly import it as the ``work`` package.  Reject malformed
        # values by interface, not by Python module identity, so the same
        # immutable geometry is accepted in both supported import layouts.
        geometry_fields = (
            'positions_m', 'quaternions_xyzw', 'arc_length_m', 'tangents',
            'omega_per_m', 'feedrate_m_s', 'source_time_s')
        config_fields = (
            'projection_half_window_segments', 'max_projection_speed_m_s',
            'reference_lead_m', 'cross_track_stop_m',
            'endpoint_braking_deceleration_m_s2', 'endpoint_settle_s')
        if not all(hasattr(geometry, field) for field in geometry_fields):
            raise TypeError('geometry must expose the PathGeometry fields')
        if not all(hasattr(config, field) for field in config_fields):
            raise TypeError('config must expose the PathFollowingConfig fields')
        jax.config.update('jax_enable_x64', self.enable_x64)
        self._path_geometry = as_jax_path_geometry(geometry)
        self._path_config = as_jax_path_config(config)
        if posture_reference is None:
            self._path_posture_reference = None
        else:
            host_reference = PathPostureReference.from_path_geometry(
                geometry,
                posture_reference,
                q_min=np.asarray(self.q_min),
                q_max=np.asarray(self.q_max),
            )
            self._path_posture_reference = as_jax_path_posture_reference(
                host_reference)

    def path_geometry_arrays(self) -> dict[str, np.ndarray]:
        """Startup snapshot of the exact arrays captured by the path kernel."""
        if self._path_geometry is None:
            raise RuntimeError('configure_path() must run before path_geometry_arrays()')
        return {name: np.array(value, copy=True)
                for name, value in self._path_geometry._asdict().items()}

    @property
    def path_is_configured(self) -> bool:
        """Whether this loop owns a compiled arc-length path kernel."""
        return self._path_geometry is not None and self._path_config is not None

    @property
    def obstacle_h_baseline_alpha(self) -> float:
        """Configured baseline used to reconstruct obstacle CBF upper bounds."""
        if self._config is None:
            raise RuntimeError("control kernel has not been initialised")
        return float(self._config.obstacle_h_baseline_alpha)

    @property
    def path_posture_reference_enabled(self) -> bool:
        """Whether the compiled path kernel owns static null-space targets."""
        return self._path_posture_reference is not None

    def initial_path_state(self) -> np.ndarray:
        """Return a fresh fixed-shape JAX path-state value for the runner."""
        if not self.path_is_configured:
            raise RuntimeError('configure_path() must run before initial_path_state()')
        return np.asarray(initial_path_state_jax())

    def init_cbf(self):
        """Build and warm the static-shape CBF-QP kernels before control starts."""
        from cbfpy import CBF
        from work.oscbf_velocity_config import NineaxisOSCBFVelocityConfig

        jax.config.update('jax_enable_x64', self.enable_x64)

        # 启用 JAX 持久化编译缓存（JAX 0.4.30 实验性，部分版本不生效）。
        # 首次编译后缓存到磁盘，后续启动理论上秒开。
        import os
        cache_dir = os.environ.get(
            "JAX_COMPILATION_CACHE_DIR",
            os.path.join(os.path.expanduser("~"), ".cache", "jax_cache"))
        try:
            from jax.experimental.compilation_cache import compilation_cache
            compilation_cache.set_cache_dir(cache_dir)
        except Exception:
            try:
                jax.config.update("jax_compilation_cache_dir", cache_dir)
            except Exception:
                pass  # 不支持时忽略

        self._config = NineaxisOSCBFVelocityConfig(
            self.robot,
            temporal_lambda=self.temporal_lambda,
            temporal_wu=self.temporal_wu,
            sdf_shape=self.sdf_shape,
            aggregate_dynamic_obstacles=self.aggregate_dynamic_obstacles,
            solver_tol=self.solver_tol,
            joint_limit_lower=self.q_min,
            joint_limit_upper=self.q_max,
            joint_limit_cbf_margin=self.joint_limit_cbf_margin)
        self._cbf = CBF.from_config(self._config)
        self._build_step_fn()
        self._warmup()

    def _build_step_fn(self):
        kernels = build_jax_control_kernels(
            cbf=self._cbf,
            robot=self.robot,
            controller_config=self._config,
            dt=self.dt,
            dt_path=self.dt_path,
            q_min=self.q_min,
            q_max=self.q_max,
            aggregate_dynamic_obstacles=self.aggregate_dynamic_obstacles,
            enable_rate_limit=self.enable_rate_limit,
            rate_limit_du_max=self.rate_limit_du_max,
            rate_limit_penalty=self.rate_limit_penalty,
            enable_sdf=self.enable_sdf,
            path_geometry=self._path_geometry,
            path_config=self._path_config,
            path_posture_reference=self._path_posture_reference,
            task_mode=self.task_mode,
            nullspace_policy=self.nullspace_policy)
        self._step_fn = kernels.step
        self._tracking_fn = kernels.tracking
        self._tracking_fast_fn = kernels.tracking_fast
        self._path_tracking_fn = kernels.path_tracking
        self._task_p_fn = kernels.task_hessian
        self._qp_problem_fn = kernels.qp_problem
        self._qp_core_fn = kernels.qp_core

    def _warmup(self):
        """Compile public call shapes before a real tracking command is accepted."""
        if self._warmed_up:
            return
        print('  JAX 固定形状控制内核 JIT 预热中...', flush=True)
        t0 = time.perf_counter()
        q_warm = np.clip(
            np.zeros(9, dtype=np.float64),
            np.asarray(self.q_min, dtype=np.float64),
            np.asarray(self.q_max, dtype=np.float64))
        obs_pos = np.zeros((MAX_JAX_OBSTACLES, 3), dtype=np.float64)
        obs_radii = np.zeros(MAX_JAX_OBSTACLES, dtype=np.float64)
        obs_enabled = np.zeros(MAX_JAX_OBSTACLES, dtype=np.float64)
        obs_d_safe = np.zeros(MAX_JAX_OBSTACLES, dtype=np.float64)
        obs_vel = np.zeros((MAX_JAX_OBSTACLES, 3), dtype=np.float64)
        obs_radius_dot = np.zeros(MAX_JAX_OBSTACLES, dtype=np.float64)
        obs_alpha = np.full(
            MAX_JAX_OBSTACLES, self._config.obstacle_h_baseline_alpha,
            dtype=np.float64)
        u_safe_prev = np.zeros(9, dtype=np.float64)
        sdf_shape = self.sdf_shape if self.enable_sdf else (2, 2, 2)
        sdf_distance = np.full(sdf_shape, 10.0, dtype=np.float32)
        sdf_origin = np.zeros(3, dtype=np.float32)

        # 仅预热生产内核 path_tracking_step（9 个 JIT 子模块）。
        # 遗留 step()/tracking_step() 不再预热——它们仅用于测试，
        # 在首次调用时自行 JIT 编译，不影响生产跟踪性能。
        if self.path_is_configured:
            self.path_tracking_step(
                q=q_warm,
                path_state=self.initial_path_state(),
                kp_pos=50.0,
                kp_orient=10.0,
                kp_joint=0.45,
                q_des=np.zeros(9, dtype=np.float64),
                nullspace_speed_limit=0.18,
                damping=1e-3,
                obs_pos=obs_pos,
                obs_radii=obs_radii,
                obs_enabled=obs_enabled,
                obs_d_safe=obs_d_safe,
                obs_vel=obs_vel,
                obs_radius_dot=obs_radius_dot,
                obs_alpha=obs_alpha,
                u_safe_prev=u_safe_prev,
                sdf_distance=sdf_distance,
                sdf_origin=sdf_origin,
                sdf_voxel_size=1.0,
                sdf_enabled=0.0,
                sdf_margin=self._config.d_safe_collision)
        elapsed = time.perf_counter() - t0
        print(f'  JAX 固定形状控制内核 JIT 预热完成 ({elapsed:.1f}s)',
              flush=True)
        self._warmed_up = True

    def step(self, q: np.ndarray, u_des: np.ndarray,
             obs_pos: np.ndarray = None, obs_radii: np.ndarray = None,
             obs_enabled: np.ndarray = None, obs_d_safe: np.ndarray = None,
             obs_vel: np.ndarray = None, obs_radius_dot: np.ndarray = None,
             obs_alpha: np.ndarray = None, u_safe_prev: np.ndarray = None,
             sdf_distance: np.ndarray = None, sdf_origin: np.ndarray = None,
             sdf_voxel_size: float = None, sdf_enabled: float = 0.0,
             sdf_margin: float = None,
             obs: ObstacleState | None = None) -> Tuple[np.ndarray, np.ndarray]:
        """Run one fixed-shape CBF-QP step from a supplied nominal velocity."""
        if not self.is_initialized:
            raise RuntimeError('init_cbf() must complete before step()')
        jx = self._prepare_jax_inputs(
            obs, obs_pos=obs_pos, obs_radii=obs_radii, obs_enabled=obs_enabled,
            obs_d_safe=obs_d_safe, obs_vel=obs_vel, obs_radius_dot=obs_radius_dot,
            obs_alpha=obs_alpha, u_safe_prev=u_safe_prev,
            sdf_distance=sdf_distance, sdf_origin=sdf_origin,
            sdf_voxel_size=sdf_voxel_size, sdf_enabled=sdf_enabled,
            sdf_margin=sdf_margin)
        q_jax = jnp.asarray(q)
        task_p_jax = self._task_p_fn(q_jax)
        result = self._step_fn(
            q_jax, jnp.asarray(u_des),
            jx['obs_pos'], jx['obs_radii'], jx['obs_enabled'],
            jx['obs_d_safe'], jx['obs_vel'], jx['obs_radius_dot'],
            jx['obs_alpha'], jx['u_safe_prev'],
            jx['sdf_distance'], jx['sdf_origin'], jx['sdf_voxel_size'],
            jx['sdf_enabled'], jx['sdf_margin'], task_p_jax)
        (q_next, u_safe, u_candidate, qp_ok, min_dist, min_esdf,
         rate_constraint_violation, rate_solver_slack, h_vals, cbf_grad,
         active_count, primal_residual,
         terminal_kkt_residual, terminal_kkt_accepted, dual_max,
         qp_iterations, _delta_slack) = result
        self._update_qp_diagnostics(
            min_esdf, h_vals, cbf_grad,
            active_count, primal_residual, terminal_kkt_residual,
            terminal_kkt_accepted, dual_max, qp_iterations)
        self._last_u_safe = np.asarray(u_safe)
        self.last_qp_candidate = np.asarray(u_candidate)
        return np.asarray(q_next), np.asarray(u_safe)

    def freeze_qp_problem(self, q: np.ndarray, u_des: np.ndarray,
                          obs_pos: np.ndarray = None, obs_radii: np.ndarray = None,
                          obs_enabled: np.ndarray = None, obs_d_safe: np.ndarray = None,
                          obs_vel: np.ndarray = None, obs_radius_dot: np.ndarray = None,
                          obs_alpha: np.ndarray = None, u_safe_prev: np.ndarray = None,
                          sdf_distance: np.ndarray = None, sdf_origin: np.ndarray = None,
                          sdf_voxel_size: float = None, sdf_enabled: float = 0.0,
                          sdf_margin: float = None,
                          obs: ObstacleState | None = None):
        """Return the exact qpax matrix problem for one real control sample.

        This offline audit API shares the production CBF, dynamic-RHS and
        rate-slack assembly path.  It does not change controller state and it
        does not invoke a hardware gateway.  Benchmark callers must block on
        the returned JAX values before timing the solver.
        """
        if not self.is_initialized or self._qp_problem_fn is None:
            raise RuntimeError('init_cbf() must complete before freeze_qp_problem()')
        if self._qp_core_fn is None:
            raise RuntimeError(
                'frozen QP benchmarking is unavailable in the default '
                'elastic-QP mode; enable rate limiting to audit the hard '
                'rate-slack QP')
        jx = self._prepare_jax_inputs(
            obs, obs_pos=obs_pos, obs_radii=obs_radii, obs_enabled=obs_enabled,
            obs_d_safe=obs_d_safe, obs_vel=obs_vel, obs_radius_dot=obs_radius_dot,
            obs_alpha=obs_alpha, u_safe_prev=u_safe_prev,
            sdf_distance=sdf_distance, sdf_origin=sdf_origin,
            sdf_voxel_size=sdf_voxel_size, sdf_enabled=sdf_enabled,
            sdf_margin=sdf_margin)
        q_jax = jnp.asarray(q)
        task_p = self._task_p_fn(q_jax)
        return self._qp_problem_fn(
            q_jax, jnp.asarray(u_des),
            jx['obs_pos'], jx['obs_radii'], jx['obs_enabled'],
            jx['obs_d_safe'], jx['obs_vel'], jx['obs_radius_dot'],
            jx['obs_alpha'], jx['u_safe_prev'],
            jx['sdf_distance'], jx['sdf_origin'], jx['sdf_voxel_size'],
            jx['sdf_enabled'], jx['sdf_margin'], task_p)

    def solve_frozen_qp_problem(self, problem):
        """Run upstream qpax on :meth:`freeze_qp_problem` output.

        Device arrays are intentionally returned unchanged.  This makes a
        caller use ``block_until_ready`` explicitly, instead of measuring
        asynchronous JAX dispatch as if it were the numerical QP solve.
        """
        if self._qp_core_fn is None:
            raise RuntimeError(
                'frozen QP benchmarking is unavailable in the default '
                'elastic-QP mode; enable rate limiting to audit the hard '
                'rate-slack QP')
        return self._qp_core_fn(*problem)

    def tracking_step(self, *, q: np.ndarray, task_pos: np.ndarray,
                      task_vel: np.ndarray, task_rot: np.ndarray,
                      task_omega: np.ndarray, kp_pos: float,
                      kp_orient: float, kp_joint: float, q_des: np.ndarray,
                      nullspace_speed_limit: float, damping: float = 1e-3,
                      obs_pos: np.ndarray = None, obs_radii: np.ndarray = None,
                      obs_enabled: np.ndarray = None, obs_d_safe: np.ndarray = None,
                      obs_vel: np.ndarray = None, obs_radius_dot: np.ndarray = None,
                      obs_alpha: np.ndarray = None, u_safe_prev: np.ndarray = None,
                      sdf_distance: np.ndarray = None, sdf_origin: np.ndarray = None,
                      sdf_voxel_size: float = None, sdf_enabled: float = 0.0,
                      sdf_margin: float = None,
                      obs: ObstacleState | None = None):
        """Run nominal P-only OSC and CBF-QP inside one JIT entry point."""
        if not self.is_initialized or self._tracking_fn is None:
            raise RuntimeError('init_cbf() must complete before tracking_step()')
        jx = self._prepare_jax_inputs(
            obs, obs_pos=obs_pos, obs_radii=obs_radii, obs_enabled=obs_enabled,
            obs_d_safe=obs_d_safe, obs_vel=obs_vel, obs_radius_dot=obs_radius_dot,
            obs_alpha=obs_alpha, u_safe_prev=u_safe_prev,
            sdf_distance=sdf_distance, sdf_origin=sdf_origin,
            sdf_voxel_size=sdf_voxel_size, sdf_enabled=sdf_enabled,
            sdf_margin=sdf_margin)
        tracking_fn = (
            self._tracking_fn if self.collect_cbf_diagnostics
            else self._tracking_fast_fn)
        result = tracking_fn(
            jnp.asarray(q), jnp.asarray(task_pos), jnp.asarray(task_vel),
            jnp.asarray(task_rot), jnp.asarray(task_omega), jnp.asarray(kp_pos),
            jnp.asarray(kp_orient), jnp.asarray(kp_joint), jnp.asarray(q_des),
            jnp.asarray(nullspace_speed_limit), jnp.asarray(damping),
            jx['obs_pos'], jx['obs_radii'], jx['obs_enabled'],
            jx['obs_d_safe'], jx['obs_vel'], jx['obs_radius_dot'],
            jx['obs_alpha'], jx['u_safe_prev'],
            jx['sdf_distance'], jx['sdf_origin'], jx['sdf_voxel_size'],
            jx['sdf_enabled'], jx['sdf_margin'])
        if self.collect_cbf_diagnostics:
            (q_next, u_safe, u_candidate, u_nom, err_6d, ee_pos, ee_rot,
             qp_ok, min_dist, min_esdf, rate_constraint_violation,
             rate_solver_slack, h_vals, cbf_grad,
             active_count, primal_residual, terminal_kkt_residual,
             terminal_kkt_accepted, dual_max, qp_iterations,
             # Unpacked for arity only: the frozen-QP slack reaches callers
             # through the step record, which the legacy tuple API cannot carry.
             _delta_slack) = result
        else:
            (q_next, u_safe, u_candidate, u_nom, err_6d, ee_pos, ee_rot,
             qp_ok, min_dist, min_esdf, rate_constraint_violation,
            rate_solver_slack, active_count,
             primal_residual, terminal_kkt_residual, terminal_kkt_accepted,
             dual_max, qp_iterations) = result
            h_vals = None
            cbf_grad = None
        self._update_qp_diagnostics(
            min_esdf, h_vals, cbf_grad,
            active_count, primal_residual, terminal_kkt_residual,
            terminal_kkt_accepted, dual_max, qp_iterations)
        self._last_u_safe = np.asarray(u_safe)
        self.last_qp_candidate = np.asarray(u_candidate)
        return (
            np.asarray(q_next), np.asarray(u_safe), np.asarray(u_nom),
            np.asarray(err_6d), np.asarray(ee_pos), np.asarray(ee_rot),
            bool(qp_ok), float(min_dist))

    def path_tracking_step(self, *, q: np.ndarray, path_state: np.ndarray,
                           kp_pos: float, kp_orient: float, kp_joint: float,
                           q_des: np.ndarray, nullspace_speed_limit: float,
                           damping: float = 1e-3,
                           obs_pos: np.ndarray = None, obs_radii: np.ndarray = None,
                           obs_enabled: np.ndarray = None, obs_d_safe: np.ndarray = None,
                           obs_vel: np.ndarray = None, obs_radius_dot: np.ndarray = None,
                           obs_alpha: np.ndarray = None, u_safe_prev: np.ndarray = None,
                           sdf_distance: np.ndarray = None, sdf_origin: np.ndarray = None,
                           sdf_voxel_size: float = None, sdf_enabled: float = 0.0,
                           sdf_margin: float = None,
                           obs: ObstacleState | None = None) -> JaxPathTrackingResult:
        """Run one fully-JIT arc-length tracking, CBF-QP, and integration step."""
        if (not self.is_initialized or self._path_tracking_fn is None
                or not self.path_is_configured):
            raise RuntimeError(
                'configure_path() and init_cbf() must complete before path_tracking_step()')
        state = np.asarray(path_state, dtype=float).reshape(-1)
        if state.shape != (5,):
            raise ValueError(f'path_state must have shape (5,), got {state.shape}')
        jx = self._prepare_jax_inputs(
            obs, obs_pos=obs_pos, obs_radii=obs_radii, obs_enabled=obs_enabled,
            obs_d_safe=obs_d_safe, obs_vel=obs_vel, obs_radius_dot=obs_radius_dot,
            obs_alpha=obs_alpha, u_safe_prev=u_safe_prev,
            sdf_distance=sdf_distance, sdf_origin=sdf_origin,
            sdf_voxel_size=sdf_voxel_size, sdf_enabled=sdf_enabled,
            sdf_margin=sdf_margin)
        result = self._path_tracking_fn(
            # The outer function is already jitted.  Keep array conversion at
            # the two public vector boundaries, while passing scalar gains and
            # the normalized path state through unchanged; the JIT boundary
            # handles conversion without separate jnp.asarray dispatches.
            jnp.asarray(q), state, kp_pos,
            kp_orient, kp_joint, jnp.asarray(q_des),
            nullspace_speed_limit, damping,
            jx['obs_pos'], jx['obs_radii'], jx['obs_enabled'],
            jx['obs_d_safe'], jx['obs_vel'], jx['obs_radius_dot'],
            jx['obs_alpha'], jx['u_safe_prev'],
            jx['sdf_distance'], jx['sdf_origin'], jx['sdf_voxel_size'],
            jx['sdf_enabled'], jx['sdf_margin'])
        self._update_qp_diagnostics(
            result.min_esdf_dist, result.h_vals, result.cbf_grad,
            result.active_count, result.primal_residual,
            result.terminal_kkt_residual, result.terminal_kkt_accepted,
            result.dual_max, result.qp_iterations)
        self._last_u_safe = np.asarray(result.u_safe)
        self.last_qp_candidate = np.asarray(result.u_candidate)
        next_state_np = np.asarray(result.path_state)
        self.last_path_metrics = {
            'path_progress_m': float(next_state_np[0]),
            'path_projection_m': float(next_state_np[1]),
            'path_reference_lead_m': float(next_state_np[0] - next_state_np[1]),
            'path_cross_track_error_m': float(result.cross_track_error_m),
            'path_gamma': float(result.gamma),
            'path_feedrate_nominal_m_s': float(result.feedrate_nominal_m_s),
            'path_feedrate_m_s': float(result.feedrate_m_s),
            'path_feedrate_joint_limit_m_s': float(
                result.feedrate_joint_limit_m_s),
            'path_feedrate_cbf_limit_m_s': float(
                result.feedrate_cbf_limit_m_s),
            'path_feedrate_rate_limit_m_s': float(
                result.feedrate_rate_limit_m_s),
            'path_feedrate_tool_axis_limit_m_s': float(
                result.feedrate_tool_axis_limit_m_s),
            'path_feedrate_endpoint_brake_limit_m_s': float(
                result.feedrate_endpoint_brake_limit_m_s),
            'path_actual_tangent_speed_m_s': float(
                result.actual_tangent_speed_m_s),
            'path_endpoint_hold_s': float(next_state_np[3]),
            'path_completed': float(next_state_np[4] > 0.5),
            'path_limit_code': float(result.limiting_reason_code),
            'path_reference_at_endpoint': float(result.reference_at_endpoint),
            'path_posture_reference_enabled': float(
                self.path_posture_reference_enabled),
            'path_delta_slack': float(result.delta_slack),
        }
        return JaxPathTrackingResult(
            q_next=np.asarray(result.q_next),
            u_safe=np.asarray(result.u_safe),
            u_nom=np.asarray(result.u_nom),
            err_6d=np.asarray(result.err_6d),
            ee_pos=np.asarray(result.ee_pos),
            ee_rot=np.asarray(result.ee_rot),
            qp_ok=bool(result.qp_ok),
            min_obs_dist=float(result.min_obs_dist),
            min_obs_dist_measured=bool(result.min_obs_dist_measured),
            min_esdf_dist=float(result.min_esdf_dist),
            min_esdf_dist_measured=bool(result.min_esdf_dist_measured),
            path_state=next_state_np,
            reference_position_m=np.asarray(result.reference_position_m),
            reference_rotation=np.asarray(result.reference_rotation),
            posture_reference=np.asarray(result.posture_reference),
            reference_tangent=np.asarray(result.reference_tangent),
            reference_omega_per_m=np.asarray(result.reference_omega_per_m),
            reference_source_time_s=float(result.reference_source_time_s),
            cross_track_error_m=float(result.cross_track_error_m),
            gamma=float(result.gamma),
            feedrate_nominal_m_s=float(result.feedrate_nominal_m_s),
            feedrate_m_s=float(result.feedrate_m_s),
            feedrate_joint_limit_m_s=float(result.feedrate_joint_limit_m_s),
            feedrate_cbf_limit_m_s=float(result.feedrate_cbf_limit_m_s),
            feedrate_rate_limit_m_s=float(result.feedrate_rate_limit_m_s),
            feedrate_tool_axis_limit_m_s=float(
                result.feedrate_tool_axis_limit_m_s),
            feedrate_endpoint_brake_limit_m_s=float(
                result.feedrate_endpoint_brake_limit_m_s),
            limiting_reason_code=int(result.limiting_reason_code),
            actual_tangent_speed_m_s=float(result.actual_tangent_speed_m_s),
            reference_at_endpoint=bool(result.reference_at_endpoint),
            delta_slack=float(result.delta_slack),
            qp_primal_residual=float(result.primal_residual),
            constraint_metrics=self._path_constraint_metrics(
                result.constraint_residuals, result.h_vals, jx),
            ee_pos_before=np.asarray(result.ee_pos_before),
        )

    def _path_constraint_metrics(self, residuals, static_margins, inputs) -> dict:
        """Group measured rows by quantity/unit; disabled geometry stays unmeasured.

        h_vals is the static CBF RHS divided by the baseline gain, before
        dynamic obstacle corrections. It is explicitly not physical clearance.
        """
        config = self._config
        joint_count = self.robot.num_joints
        from work.kinematics_data import JOINT_CHAIN
        joint_types = [row[2] for row in JOINT_CHAIN if row[2] != 'fixed']
        linear = [i for i, kind in enumerate(joint_types) if kind == 'prismatic']
        angular = [i for i, kind in enumerate(joint_types) if kind == 'revolute']
        linear = linear + [i + joint_count for i in linear]
        angular = angular + [i + joint_count for i in angular]
        groups = (
            ("joint_linear", linear, "m", True),
            ("joint_angular", angular, "rad", True),
            ("self_collision", slice(2 * joint_count, config.obstacle_h_start), "m", True),
            ("obstacle", slice(config.obstacle_h_start, config.obstacle_h_stop), "m",
             bool(np.any(np.asarray(inputs['obs_enabled']) > 0.5))),
            ("esdf", slice(config.esdf_h_start, config.esdf_h_stop), "m",
             bool(config.enable_sdf and float(inputs['sdf_enabled']) > 0.5)),
            ("singularity", slice(config.esdf_h_stop, config.esdf_h_stop + 1), "model_manipulability", True),
        )
        rows, margins = np.asarray(residuals), np.asarray(static_margins)
        metrics = {}
        for name, indices, unit, enabled in groups:
            metrics[name + '.residual'] = {
                'value': float(np.maximum(np.max(rows[indices]), 0.0)) if enabled else None,
                'quantity': 'max_positive_unrelaxed_G_u_candidate_minus_h',
                'unit': unit + '/s', 'active': enabled,
                'source': 'QP solve state; dynamic-corrected RHS; before health gate/integration/filter',
            }
            metrics[name + '.static_margin'] = {
                'value': float(np.min(margins[indices])) if enabled else None,
                'quantity': 'static_cbf_rhs_div_baseline_alpha',
                'unit': unit, 'active': enabled,
                'source': 'QP solve state; static RHS before dynamic correction; model metric, not physical clearance',
            }
        return metrics

    def _update_qp_diagnostics(self, min_esdf, h_vals, cbf_grad, active_count,
                               primal_residual, terminal_kkt_residual,
                               terminal_kkt_accepted, dual_max, qp_iterations):
        """Route one step's diagnostics to their two destinations.

        The step record is the carrier the node, the evaluator and the
        telemetry read, so what remains here is only the pair that cannot live
        in a step record: the per-attribute mirrors that a test or a research
        script reads *by name*, and the cross-step CBF memory, which by
        definition spans two steps.  Assigning both kinds from one place is
        what made the old ``last_delta_slack`` side channel look initialised
        when it was not.
        """
        self._mirror_qp_diagnostics(
            min_esdf, active_count, primal_residual, terminal_kkt_residual,
            terminal_kkt_accepted, dual_max, qp_iterations)
        self._advance_cbf_telemetry(h_vals, cbf_grad)

    def _mirror_qp_diagnostics(self, min_esdf, active_count, primal_residual,
                               terminal_kkt_residual, terminal_kkt_accepted,
                               dual_max, qp_iterations):
        """Keep the named attributes alive for the readers that still use them.

        A mirror is justified by a reader, not by a value: ``last_min_esdf_dist``
        has a production reader in ``perception_demo``, the rest are read by
        tests and archived research scripts.  What the step record already
        carries is not mirrored here.
        """
        self.last_min_esdf_dist = float(min_esdf)
        self.last_qp_active_count = int(active_count)
        self.last_qp_iterations = int(qp_iterations)
        self.last_qp_primal_residual = float(primal_residual)
        self.last_qp_terminal_kkt_residual = float(terminal_kkt_residual)
        self.last_qp_terminal_kkt_accepted = bool(terminal_kkt_accepted)
        self.last_qp_dual_max = float(dual_max)

    def _advance_cbf_telemetry(self, h_vals, cbf_grad) -> None:
        """Fold this step's CBF telemetry into the cross-step state.

        No telemetry on this step (the fast path) drops the previous sample
        instead of keeping it, so the two delta norms report ``NaN``,
        unmeasured, rather than a stale difference.
        """
        self.cross_step_state = advance_cross_step_state(
            self.cross_step_state, h_vals=h_vals, cbf_grad=cbf_grad)

    def _prepare_jax_inputs(self, obs: ObstacleState | None = None,
                            *, obs_pos=None, obs_radii=None, obs_enabled=None,
                            obs_d_safe=None, obs_vel=None, obs_radius_dot=None,
                            obs_alpha=None, u_safe_prev=None,
                            sdf_distance=None, sdf_origin=None,
                            sdf_voxel_size=None, sdf_enabled=0.0,
                            sdf_margin=None):
        """Normalise obstacle + SDF inputs and convert to JAX arrays.

        Accepts either an :class:`ObstacleState` bundle or the individual
        keyword arguments (backward-compatible).  Returns a dict of JAX
        arrays ready to pass to the compiled kernel.
        """
        use_defaults = (
            obs is None
            and all(value is None for value in (
                obs_pos, obs_radii, obs_enabled, obs_d_safe, obs_vel,
                obs_radius_dot, obs_alpha, sdf_distance, sdf_origin,
                sdf_voxel_size, sdf_margin))
            and isinstance(sdf_enabled, (int, float)) and sdf_enabled == 0.0)
        if use_defaults:
            defaults_key = (self._config.d_safe_collision,
                            self._config.obstacle_h_baseline_alpha,
                            self.enable_sdf, self.sdf_shape,
                            jax.config.x64_enabled)
            if (self._default_jax_inputs is not None
                    and self._default_jax_inputs_key == defaults_key):
                # JAX arrays are immutable. Copy only the mapping and always
                # refresh the previous control used by the rate constraints.
                return dict(self._default_jax_inputs, u_safe_prev=jnp.asarray(
                    self._last_u_safe if u_safe_prev is None else u_safe_prev))
        if obs is not None:
            obs_pos = obs.obs_pos; obs_radii = obs.obs_radii
            obs_enabled = obs.obs_enabled; obs_d_safe = obs.obs_d_safe
            obs_vel = obs.obs_vel; obs_radius_dot = obs.obs_radius_dot
            obs_alpha = obs.obs_alpha; u_safe_prev = obs.u_safe_prev
            sdf_distance = obs.sdf_distance; sdf_origin = obs.sdf_origin
            sdf_voxel_size = obs.sdf_voxel_size; sdf_enabled = obs.sdf_enabled
            sdf_margin = obs.sdf_margin
        (obs_pos, obs_radii, obs_enabled, obs_d_safe, obs_vel,
         obs_radius_dot, obs_alpha, u_safe_prev) = self._normalise_obstacle_inputs(
            obs_pos, obs_radii, obs_enabled, obs_d_safe, obs_vel,
            obs_radius_dot, obs_alpha, u_safe_prev)
        (sdf_distance, sdf_origin, sdf_voxel_size, sdf_enabled,
         sdf_margin) = self._normalise_sdf_inputs(
            sdf_distance, sdf_origin, sdf_voxel_size, sdf_enabled, sdf_margin)
        result = {
            'obs_pos': jnp.asarray(obs_pos),
            'obs_radii': jnp.asarray(obs_radii),
            'obs_enabled': jnp.asarray(obs_enabled),
            'obs_d_safe': jnp.asarray(obs_d_safe),
            'obs_vel': jnp.asarray(obs_vel),
            'obs_radius_dot': jnp.asarray(obs_radius_dot),
            'obs_alpha': jnp.asarray(obs_alpha),
            'u_safe_prev': jnp.asarray(u_safe_prev),
            'sdf_distance': jnp.asarray(sdf_distance),
            'sdf_origin': jnp.asarray(sdf_origin),
            'sdf_voxel_size': jnp.asarray(sdf_voxel_size),
            'sdf_enabled': jnp.asarray(sdf_enabled),
            'sdf_margin': jnp.asarray(sdf_margin),
        }
        if use_defaults:
            self._default_jax_inputs = {
                key: value for key, value in result.items() if key != 'u_safe_prev'}
            self._default_jax_inputs_key = defaults_key
        return result

    def _normalise_obstacle_inputs(self, obs_pos, obs_radii, obs_enabled,
                                   obs_d_safe, obs_vel, obs_radius_dot,
                                   obs_alpha, u_safe_prev):
        if obs_pos is None:
            obs_pos = np.zeros((MAX_JAX_OBSTACLES, 3))
            obs_radii = np.zeros(MAX_JAX_OBSTACLES)
            obs_enabled = np.zeros(MAX_JAX_OBSTACLES)
        if obs_d_safe is None:
            obs_d_safe = np.full(
                MAX_JAX_OBSTACLES, self._config.d_safe_collision)
        if obs_vel is None:
            obs_vel = np.zeros((MAX_JAX_OBSTACLES, 3))
        if obs_radius_dot is None:
            obs_radius_dot = np.zeros(MAX_JAX_OBSTACLES)
        if obs_alpha is None:
            obs_alpha = np.full(
                MAX_JAX_OBSTACLES, self._config.obstacle_h_baseline_alpha)
        if u_safe_prev is None:
            u_safe_prev = self._last_u_safe
        return (obs_pos, obs_radii, obs_enabled, obs_d_safe, obs_vel,
                obs_radius_dot, obs_alpha, u_safe_prev)

    def _normalise_sdf_inputs(self, sdf_distance, sdf_origin, sdf_voxel_size,
                              sdf_enabled, sdf_margin):
        """Return the canonical ESDF host representation for the JAX kernel.

        ``SafetySnapshot`` intentionally stores grid samples and their origin
        as ``float32``.  Enforcing that contract here also keeps callers that
        construct a grid directly from accidentally creating a second JIT
        executable solely because NumPy chose ``float64`` for the origin.
        The control state and QP remain ``float64``; only the fixed ESDF
        storage format is canonicalised.
        """
        if sdf_distance is None:
            shape = self.sdf_shape if self.enable_sdf else (2, 2, 2)
            sdf_distance = np.full(shape, 10.0, dtype=np.float32)
        else:
            sdf_distance = np.asarray(sdf_distance, dtype=np.float32)
        if sdf_origin is None:
            sdf_origin = np.zeros(3, dtype=np.float32)
        else:
            sdf_origin = np.asarray(sdf_origin, dtype=np.float32).reshape(3)
        if sdf_voxel_size is None:
            sdf_voxel_size = 1.0
        if sdf_margin is None:
            sdf_margin = self._config.d_safe_collision
        if self.enable_sdf and tuple(np.asarray(sdf_distance).shape) != self.sdf_shape:
            raise ValueError(
                f'expected fixed sdf shape {self.sdf_shape}, got '
                f'{np.asarray(sdf_distance).shape}')
        return sdf_distance, sdf_origin, sdf_voxel_size, sdf_enabled, sdf_margin

    @property
    def is_initialized(self) -> bool:
        return self._cbf is not None
