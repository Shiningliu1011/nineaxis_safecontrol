"""Factory for the fixed-shape JAX OSCBF control kernels.

The module owns the compiled mathematical kernels only.  It has no ROS,
PointCloud2, filesystem, NumPy host-state, or command-gateway behavior.
``JaxControlLoop`` remains the backwards-compatible facade that normalises
inputs and exposes diagnostics.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import jax
import jax.numpy as jnp
from qpax import solve_qp as qpax_solve_qp

from work.jax_barrier_terms import (
    aggregate_dynamic_obstacle_terms,
    apply_aggregated_dynamic_cbf_terms,
    apply_dynamic_obstacle_cbf_terms,
    apply_qp_health_gate,
    compute_dcol_obstacle_clearance,
)
from work.jax_path_following import (
    JaxPathFollowingConfig,
    JaxPathGeometry,
    advance_path_state_jax,
    feedrate_limit_from_box_jax,
    feedrate_limit_from_inequalities_jax,
    reconcile_path_state_after_motion_jax,
    sample_path_jax,
)
from work.jax_posture_reference import (
    JaxPathPostureReference,
    sample_path_posture_reference_jax,
)
from work.qp_solver_health import terminal_qp_health
from work.safety_snapshot import sample_distance_field_jax
from work.tool_axis_task import (
    TASK_MODE_POSE_6D,
    TASK_MODE_TOOL_AXIS_5D,
    rotation_error_rotvec_jax,
    task_error_5d_jax,
    task_error_report_6d_jax,
    task_jacobian_5d_jax,
    tool_axis_angular_velocity_2d_jax,
)

# 位置反馈误差的饱和半径: 正常跟踪误差 (mm 级) 内线性增益; 超过后反馈
# 速度封顶为 kp_pos * 0.005 (kp=80 时 0.4 m/s), 防止大误差时反馈随误差
# 增长、与 plant 位置环串联后自激振荡。
_POSITION_FB_SATURATION_M = 0.005
_EPS = 1e-12


@dataclass(frozen=True)
class JaxControlKernels:
    """Compiled entry points used by the lightweight JAX control facade."""

    step: Callable
    tracking: Callable
    tracking_fast: Callable
    task_hessian: Callable
    path_tracking: Callable | None = None
    qp_problem: Callable | None = None
    qp_core: Callable | None = None
    path_modules: tuple = ()


def build_jax_control_kernels(*, cbf, robot, controller_config, dt,
                              dt_path: float | None = None,
                              q_min, q_max, aggregate_dynamic_obstacles: bool,
                              enable_rate_limit: bool,
                              rate_limit_du_max, rate_limit_penalty: float,
                              enable_sdf: bool,
                              path_geometry: JaxPathGeometry | None = None,
                              path_config: JaxPathFollowingConfig | None = None,
                              path_posture_reference: JaxPathPostureReference | None = None,
                              task_mode: str = TASK_MODE_POSE_6D,
                              nullspace_policy=None,
                              ) -> JaxControlKernels:
    """Build static-shape QP and tracking kernels without changing CBF semantics.

    ``dt`` is the QP integration step (kept at the trajectory sampling period
    for the stable numeric path); ``dt_path`` is the reference path advance
    step and must match the real control tick (e.g. 1/publish_frequency).
    Defaults to ``dt`` when omitted, preserving legacy callers.
    """
    dt_path = dt if dt_path is None else float(dt_path)
    if not 0.0 < dt_path < float("inf"):
        raise ValueError(f'dt_path must be finite and positive, got {dt_path}')
    obstacle_h_start = controller_config.obstacle_h_start
    obstacle_h_baseline_alpha = controller_config.obstacle_h_baseline_alpha
    smooth_min_temperature = controller_config.smooth_min_temperature
    rate_limit = (jnp.asarray(rate_limit_du_max)
                  if enable_rate_limit else None)

    # Soft rate limiting is a hard-CBF mode: its augmented problem keeps the
    # CBF and velocity-box rows hard and only relaxes the rate-of-change rows
    # through the two shared slacks (see config/nineaxis.yaml soft_rate_limit).
    # The M7 elastic solver (cbf.qp_solver == solve_qp_elastic when
    # relax_cbf=True) has neither the equality-aware signature nor the
    # per-row penalty semantics this formulation needs, so the rate-limited
    # path always uses the hard qpax solver below.  The default (non-rate
    # limited) control loop remains the M7 elastic QP.

    if task_mode not in (TASK_MODE_POSE_6D, TASK_MODE_TOOL_AXIS_5D):
        raise ValueError(f'unsupported task_mode: {task_mode!r}')

    pose_task_weights_sq = jnp.array([
        controller_config.pos_obj_weight**2,
        controller_config.pos_obj_weight**2,
        controller_config.pos_obj_weight**2,
        controller_config.rot_obj_weight**2,
        controller_config.rot_obj_weight**2,
        controller_config.rot_obj_weight**2,
    ])
    tool_axis_task_weights_sq = jnp.array([
        controller_config.pos_obj_weight**2,
        controller_config.pos_obj_weight**2,
        controller_config.pos_obj_weight**2,
        controller_config.rot_obj_weight**2,
        controller_config.rot_obj_weight**2,
    ])
    tracking_task_weights_sq = (
        tool_axis_task_weights_sq
        if task_mode == TASK_MODE_TOOL_AXIS_5D
        else pose_task_weights_sq)

    def task_hessian_from_jacobian(jacobian, task_weights_sq):
        """Match OSCBF task weighting for a known 5-D or 6-D Jacobian."""
        task_damping = 1e-3
        jacobian_hash = jacobian.T @ jnp.linalg.inv(
            jacobian @ jacobian.T + task_damping**2 * jnp.eye(jacobian.shape[0]))
        null_projector = jnp.eye(robot.num_joints) - jacobian_hash @ jacobian
        task_weights_sq = jnp.diag(task_weights_sq)
        joint_weight_sq = (
            controller_config.joint_obj_weight**2 * jnp.eye(robot.num_joints))
        p_task = (
            null_projector.T @ joint_weight_sq @ null_projector
            + jacobian.T @ task_weights_sq @ jacobian)
        return 0.5 * (p_task + p_task.T)

    def task_hessian_from_q(q):
        # ``step()`` has no desired orientation input, so preserve its legacy
        # 6-D task weighting even when a tracking loop uses the 5-D mode.
        return task_hessian_from_jacobian(robot.ee_jacobian(q), pose_task_weights_sq)

    def task_geometry_terms(ee_pos, ee_rot, full_jacobian, task_pos, task_rot):
        """Return task error, fixed-shape telemetry error, and task Jacobian."""
        if task_mode == TASK_MODE_TOOL_AXIS_5D:
            task_error = task_error_5d_jax(
                ee_pos, task_pos, ee_rot, task_rot)
            return (
                task_error,
                task_error_report_6d_jax(task_error),
                task_jacobian_5d_jax(full_jacobian, ee_rot, task_rot),
            )
        pos_err = ee_pos - task_pos
        rot_err = rotation_error_rotvec_jax(ee_rot, task_rot)
        task_error = jnp.concatenate([pos_err, rot_err])
        return task_error, task_error, full_jacobian

    def task_velocity_from_world(linear_velocity, angular_velocity, task_rot):
        """Express a world-frame reference twist in the active task rows."""
        if task_mode == TASK_MODE_TOOL_AXIS_5D:
            return jnp.concatenate([
                linear_velocity,
                tool_axis_angular_velocity_2d_jax(task_rot, angular_velocity),
            ])
        return jnp.concatenate([linear_velocity, angular_velocity])

    def make_h_args(obs_pos, obs_radii, obs_enabled, obs_d_safe,
                    obs_vel, obs_radius_dot, obs_alpha, u_safe_prev,
                    sdf_distance, sdf_origin, sdf_voxel_size, sdf_enabled,
                    sdf_margin, task_p):
        """Keep every cbfpy argument fixed-shape and in one canonical order."""
        return (
            obs_pos, obs_radii, obs_enabled, obs_d_safe, obs_vel,
            obs_radius_dot, obs_alpha, u_safe_prev, sdf_distance,
            sdf_origin, sdf_voxel_size, sdf_enabled, sdf_margin, task_p)

    def build_constraint_terms(q, h_args):
        """Construct hard CBF rows once, independently of ``u_des``.

        cbfpy's ``G_qp`` and ``h_qp`` depend on the state and barrier inputs,
        not the nominal control.  Path following uses these same rows to
        derive a conservative feedrate cap before the final QP solve.
        """
        zero_u = jnp.zeros(cbf.m)
        G = cbf.G_qp(q, zero_u, *h_args)
        h_qp = cbf.h_qp(q, zero_u, *h_args)
        h_vals = h_qp[:cbf.num_cbf] / obstacle_h_baseline_alpha
        cbf_grad = -G[:cbf.num_cbf]

        (obs_pos, obs_radii, obs_enabled, obs_d_safe, obs_vel,
         obs_radius_dot, obs_alpha, _u_safe_prev, sdf_distance,
         sdf_origin, sdf_voxel_size, sdf_enabled, sdf_margin,
         _task_p) = h_args
        collision_data = robot.environment_collision_data(q)
        positions = collision_data[:, :3]
        radii = collision_data[:, 3]
        h_obs, h_dot_obs = compute_dcol_obstacle_clearance(
            q, obs_pos, obs_radii, obs_d_safe, obs_vel, obs_radius_dot)
        if aggregate_dynamic_obstacles:
            h_qp = apply_aggregated_dynamic_cbf_terms(
                h_qp, h_obs, h_dot_obs, obs_enabled, obs_alpha,
                obstacle_start=obstacle_h_start,
                baseline_alpha=obstacle_h_baseline_alpha,
                temperature=smooth_min_temperature)
        else:
            h_qp = apply_dynamic_obstacle_cbf_terms(
                h_qp, h_obs, h_dot_obs, obs_enabled, obs_alpha,
                obstacle_start=obstacle_h_start,
                baseline_alpha=obstacle_h_baseline_alpha)

        h_obs_masked = jnp.where(obs_enabled[None, :] > 0.5, h_obs, 1e3)
        min_obs_dist = jnp.where(
            jnp.any(obs_enabled > 0.5), jnp.min(h_obs_masked), 1.0)
        if enable_sdf:
            h_esdf = jax.vmap(sample_distance_field_jax, in_axes=(None, 0, None, None))(
                sdf_distance, positions, sdf_origin, sdf_voxel_size)
            h_esdf = h_esdf - radii - sdf_margin
            min_esdf_dist = jnp.where(sdf_enabled > 0.5, jnp.min(h_esdf), 1.0)
        else:
            min_esdf_dist = jnp.asarray(1.0)
        return G, h_qp, h_vals, cbf_grad, min_obs_dist, min_esdf_dist

    def assemble_qp_data(q, u_des, h_args, G, h_qp):
        """Build exactly the matrix problem consumed by the qpax solve.

        Production control and frozen-input benchmarking share this helper.
        In particular, the two rate slacks remain part of the benchmarked
        problem; a timing result must not silently measure an easier QP.
        """
        P = cbf.P_qp(q, u_des, *h_args)
        q_qp = cbf.q_qp(q, u_des, *h_args)
        A = jnp.zeros((0, cbf.m))
        b = jnp.zeros(0)
        u_safe_prev = h_args[7]

        if enable_rate_limit:
            # CBF and velocity-box rows stay hard.  Only two shared rate
            # slacks are elastic, preserving a constant QP dimension.
            n_controls = cbf.m
            p_aug = jnp.zeros((n_controls + 2, n_controls + 2))
            p_aug = p_aug.at[:n_controls, :n_controls].set(P)
            p_aug = p_aug.at[n_controls:, n_controls:].set(
                rate_limit_penalty * jnp.eye(2))
            q_aug = jnp.concatenate([q_qp, jnp.zeros(2)])
            g_base = jnp.concatenate([G, jnp.zeros((G.shape[0], 2))], axis=1)
            eye = jnp.eye(n_controls)
            zeros_col = jnp.zeros((n_controls, 1))
            rate_lower = jnp.concatenate(
                [-eye, -jnp.ones((n_controls, 1)), zeros_col], axis=1)
            rate_upper = jnp.concatenate(
                [eye, zeros_col, -jnp.ones((n_controls, 1))], axis=1)
            slack_nonnegative = jnp.concatenate(
                [jnp.zeros((2, n_controls)), -jnp.eye(2)], axis=1)
            g_aug = jnp.concatenate(
                [g_base, rate_lower, rate_upper, slack_nonnegative], axis=0)
            h_aug = jnp.concatenate([
                h_qp,
                rate_limit - u_safe_prev,
                rate_limit + u_safe_prev,
                jnp.zeros(2),
            ])
            a_aug = jnp.zeros((0, n_controls + 2))
            b_aug = jnp.zeros(0)
            return p_aug, q_aug, a_aug, b_aug, g_aug, h_aug

        return P, q_qp, A, b, G, h_qp

    def solve_from_constraint_terms(q, u_des, h_args, G, h_qp, h_vals,
                                    cbf_grad, min_obs_dist, min_esdf_dist):
        """Solve the unchanged fixed-shape CBF-QP from already-built rows."""
        P_solve, q_solve, A_solve, b_solve, G_solve, h_solve = assemble_qp_data(
            q, u_des, h_args, G, h_qp)
        u_safe_prev = h_args[7]

        if enable_rate_limit:
            n_controls = cbf.m
            x_qp, slack_qp, dual_qp, equality_dual_qp, converged, qp_iterations = qpax_solve_qp(
                P_solve, q_solve, A_solve, b_solve, G_solve, h_solve,
                solver_tol=cbf.solver_tol)
            terminal_health = terminal_qp_health(
                P_solve, q_solve, A_solve, b_solve, G_solve, h_solve,
                x_qp, slack_qp, dual_qp, equality_dual_qp,
                solver_tol=cbf.solver_tol)
            solver_accepted = converged | terminal_health.accepted
            u_candidate = x_qp[:n_controls]
            # qpax is an interior-point method.  Its non-negative slack
            # variables can remain slightly positive even when every rate
            # row is inactive.  Only a candidate that actually exceeds the
            # rate box represents a physical continuity relaxation.
            rate_solver_slack = jnp.max(x_qp[n_controls:])
            rate_relaxation = jnp.maximum(
                jnp.max(jnp.abs(u_candidate - u_safe_prev) - rate_limit),
                jnp.asarray(0.0))
            dual_max = jnp.max(jnp.abs(dual_qp))
            delta_slack = jnp.asarray(0.0)
            terminal_kkt_residual = terminal_health.kkt_residual
            terminal_kkt_accepted = terminal_health.accepted
        elif cbf.relax_cbf:
            x_qp, _t_qp, s1_qp, _s2_qp, _z1_qp, _z2_qp, converged, qp_iterations = cbf.qp_solver(
                P_solve, q_solve, G_solve, h_solve, cbf.cbf_relaxation_penalty,
                solver_tol=cbf.solver_tol)
            u_candidate = x_qp[:cbf.m]
            delta_slack = jnp.max(s1_qp)
            rate_relaxation = jnp.asarray(0.0)
            rate_solver_slack = jnp.asarray(0.0)
            dual_max = jnp.asarray(0.0)
            solver_accepted = converged
            terminal_kkt_residual = jnp.asarray(jnp.nan)
            terminal_kkt_accepted = jnp.asarray(False)
        else:
            x_qp, slack_qp, dual_qp, equality_dual_qp, converged, qp_iterations = cbf.qp_solver(
                P_solve, q_solve, A_solve, b_solve, G_solve, h_solve,
                solver_tol=cbf.solver_tol)
            terminal_health = terminal_qp_health(
                P_solve, q_solve, A_solve, b_solve, G_solve, h_solve,
                x_qp, slack_qp, dual_qp, equality_dual_qp,
                solver_tol=cbf.solver_tol)
            solver_accepted = converged | terminal_health.accepted
            u_candidate = x_qp[:cbf.m]
            delta_slack = jnp.asarray(0.0)
            rate_relaxation = jnp.asarray(0.0)
            rate_solver_slack = jnp.asarray(0.0)
            dual_max = jnp.max(jnp.abs(dual_qp))
            terminal_kkt_residual = terminal_health.kkt_residual
            terminal_kkt_accepted = terminal_health.accepted

        u_finite = jnp.all(jnp.isfinite(u_candidate))
        if cbf.relax_cbf and not enable_rate_limit:
            # Elastic QP (M7): a negative barrier is softened by the slack
            # instead of triggering a controlled stop; delta_slack is the
            # diagnostic.  Only convergence and finiteness gate the command.
            qp_ok = u_finite & solver_accepted
        else:
            # Rate-limited mode keeps the documented hard-CBF gate: the two
            # shared slacks relax only u - u_safe_prev, never the barrier.
            h_ok = jnp.all(h_vals >= -1e-3)
            qp_ok = u_finite & h_ok & solver_accepted
        q_next, u_safe = apply_qp_health_gate(
            q, u_candidate, qp_ok, dt=dt, q_min=q_min, q_max=q_max)
        qp_margin = G @ u_candidate - h_qp
        active_count = jnp.sum(jnp.abs(qp_margin) < 1e-4)
        primal_residual = jnp.maximum(jnp.max(qp_margin), 0.0)

        return (q_next, u_safe, u_candidate, qp_ok, min_obs_dist, min_esdf_dist,
                rate_relaxation, rate_solver_slack, h_vals, cbf_grad, active_count,
                primal_residual, terminal_kkt_residual,
                terminal_kkt_accepted, dual_max, qp_iterations, delta_slack)

    def qp_problem_step(q, u_des, obs_pos, obs_radii, obs_enabled, obs_d_safe,
                        obs_vel, obs_radius_dot, obs_alpha, u_safe_prev,
                        sdf_distance, sdf_origin, sdf_voxel_size, sdf_enabled,
                        sdf_margin, task_p):
        """Return the production QP matrices for offline timing and audit."""
        h_args = make_h_args(
            obs_pos, obs_radii, obs_enabled, obs_d_safe, obs_vel,
            obs_radius_dot, obs_alpha, u_safe_prev, sdf_distance,
            sdf_origin, sdf_voxel_size, sdf_enabled, sdf_margin, task_p)
        G, h_qp, *_ = build_constraint_terms(q, h_args)
        return assemble_qp_data(q, u_des, h_args, G, h_qp)

    def qp_core_step(P, q_qp, A, b, G, h_qp):
        """Run upstream cbfpy/qpax on preassembled fixed-shape matrices."""
        return qpax_solve_qp(
            P, q_qp, A, b, G, h_qp, solver_tol=cbf.solver_tol)

    def solve_step(q, u_des, obs_pos, obs_radii, obs_enabled, obs_d_safe,
                   obs_vel, obs_radius_dot, obs_alpha, u_safe_prev,
                   sdf_distance, sdf_origin, sdf_voxel_size, sdf_enabled,
                   sdf_margin, task_p):
        """Fixed-shape CBF-QP, health gate, and one state integration step."""
        h_args = make_h_args(
            obs_pos, obs_radii, obs_enabled, obs_d_safe, obs_vel,
            obs_radius_dot, obs_alpha, u_safe_prev, sdf_distance,
            sdf_origin, sdf_voxel_size, sdf_enabled, sdf_margin, task_p)
        terms = build_constraint_terms(q, h_args)
        return solve_from_constraint_terms(q, u_des, h_args, *terms)

    def tracking_step(q, task_pos, task_vel, task_rot, task_omega,
                      kp_pos, kp_orient, kp_joint, q_des,
                      nullspace_speed_limit, damping,
                      obs_pos, obs_radii, obs_enabled, obs_d_safe,
                      obs_vel, obs_radius_dot, obs_alpha, u_safe_prev,
                      sdf_distance, sdf_origin, sdf_voxel_size,
                      sdf_enabled, sdf_margin):
        """JIT hot path: P-only OSC followed by the fixed-shape CBF-QP."""
        ee_pos = robot.ee_position(q)
        ee_rot = robot.ee_rotation(q)
        full_jacobian = robot.ee_jacobian(q)
        task_error, err_report, jacobian = task_geometry_terms(
            ee_pos, ee_rot, full_jacobian, task_pos, task_rot)
        jacobian_hash = jacobian.T @ jnp.linalg.inv(
            jacobian @ jacobian.T + damping * jnp.eye(jacobian.shape[0]))
        task_gain = jnp.concatenate([
            jnp.full(3, kp_pos),
            jnp.full(task_error.shape[0] - 3, kp_orient)])
        task_velocity = (
            task_velocity_from_world(task_vel, task_omega, task_rot)
            - task_gain * task_error)
        qdot_task = jacobian_hash @ task_velocity
        null_projector = jnp.eye(robot.num_joints) - jacobian_hash @ jacobian
        if nullspace_policy is None:
            qdot_null = jnp.clip(
                -kp_joint * (q - q_des),
                -nullspace_speed_limit,
                nullspace_speed_limit,
            )
        else:
            qdot_null = nullspace_policy.nullspace_velocity(
                q, full_jacobian, jacobian_hash, null_projector,
                robot.joint_max_velocities)
        u_nom = qdot_task + null_projector @ qdot_null
        u_nom = jnp.clip(
            u_nom, -robot.joint_max_velocities,
            robot.joint_max_velocities)
        task_p = task_hessian_from_jacobian(jacobian, tracking_task_weights_sq)
        (q_next, u_safe, u_candidate, qp_ok, min_obs_dist, min_esdf_dist,
         rate_relaxation, rate_solver_slack, h_vals, cbf_grad, active_count, primal_residual,
         terminal_kkt_residual, terminal_kkt_accepted,
         dual_max, qp_iterations, delta_slack) = solve_step(
            q, u_nom, obs_pos, obs_radii, obs_enabled, obs_d_safe,
            obs_vel, obs_radius_dot, obs_alpha, u_safe_prev, sdf_distance,
            sdf_origin, sdf_voxel_size, sdf_enabled, sdf_margin, task_p)
        return (q_next, u_safe, u_candidate, u_nom, err_report, ee_pos, ee_rot, qp_ok,
                min_obs_dist, min_esdf_dist, rate_relaxation, rate_solver_slack,
                h_vals, cbf_grad,
                active_count, primal_residual, terminal_kkt_residual,
                terminal_kkt_accepted, dual_max, qp_iterations, delta_slack)

    def tracking_fast_step(*args):
        """Same calculation without returning the large h/gradient telemetry."""
        (q_next, u_safe, u_candidate, u_nom, err_6d, ee_pos, ee_rot, qp_ok,
         min_obs_dist, min_esdf_dist, rate_relaxation, rate_solver_slack,
         _h_vals, _cbf_grad,
         active_count, primal_residual, terminal_kkt_residual,
         terminal_kkt_accepted, dual_max, qp_iterations, _delta_slack) = tracking_step(*args)
        return (q_next, u_safe, u_candidate, u_nom, err_6d, ee_pos, ee_rot, qp_ok,
                min_obs_dist, min_esdf_dist, rate_relaxation, rate_solver_slack,
                active_count,
                primal_residual, terminal_kkt_residual,
                terminal_kkt_accepted, dual_max, qp_iterations)

    path_tracking = None
    path_modules: tuple = ()
    if path_geometry is not None and path_config is not None:
        # ------------------------------------------------------------------
        # Modular JIT (M6): the path step is a host scheduler that calls a
        # small set of independently compiled jitted modules.  Each module
        # keeps exactly the arithmetic of the former whole-kernel function so
        # the M5 baseline remains the numerical reference (< 1e-12).
        # ------------------------------------------------------------------

        def _posture_target(progress_m, fallback_q_des):
            if path_posture_reference is None:
                return fallback_q_des
            return sample_path_posture_reference_jax(
                path_posture_reference, progress_m)

        @jax.jit
        def _path_kinematics(q):
            return robot.ee_position(q), robot.ee_rotation(q), robot.ee_jacobian(q)

        @jax.jit
        def _path_sample(progress_m):
            return sample_path_jax(path_geometry, progress_m)

        @jax.jit
        def _path_cap_nominal(ee_pos, ee_rot, full_jacobian, sample, q,
                              kp_pos, kp_orient, kp_joint,
                              nullspace_speed_limit, damping, q_des):
            cap_task_error, _cap_err_report, cap_jacobian = task_geometry_terms(
                ee_pos, ee_rot, full_jacobian,
                sample.position_m, sample.rotation)
            cap_pos_err = cap_task_error[:3]
            cap_orientation_err = cap_task_error[3:]
            cap_jacobian_hash = cap_jacobian.T @ jnp.linalg.inv(
                cap_jacobian @ cap_jacobian.T
                + damping * jnp.eye(cap_jacobian.shape[0]))
            cap_null_projector = (
                jnp.eye(robot.num_joints) - cap_jacobian_hash @ cap_jacobian)
            cap_q_des = _posture_target(sample.progress_m, q_des)
            if nullspace_policy is None:
                cap_qdot_null = jnp.clip(
                    -kp_joint * (q - cap_q_des),
                    -nullspace_speed_limit,
                    nullspace_speed_limit,
                )
            else:
                cap_qdot_null = nullspace_policy.nullspace_velocity(
                    q, full_jacobian, cap_jacobian_hash, cap_null_projector,
                    robot.joint_max_velocities)
            tangent_error = sample.tangent * jnp.dot(
                sample.tangent, cap_pos_err)
            transverse_pos_err = cap_pos_err - tangent_error
            position_feedback_error = jnp.where(
                sample.at_endpoint, cap_pos_err, transverse_pos_err)
            # Saturate the position correction at a small arc-length radius:
            # beyond it the pull-back speed stays bounded (kp * sat) instead
            # of growing with the error, so a transient that exceeds the
            # cross-track stop threshold (feedrate -> 0, pure feedback) can
            # never cascade into an unstable loop with the plant's own
            # position servo.
            fb_norm = jnp.linalg.norm(position_feedback_error)
            fb_scale = jnp.where(
                fb_norm > _POSITION_FB_SATURATION_M,
                _POSITION_FB_SATURATION_M / jnp.maximum(fb_norm, _EPS),
                jnp.asarray(1.0),
            )
            endpoint_orient_scale = jnp.where(sample.at_endpoint, 0.1, 1.0)
            # Endpoint hold mode must not re-open the full 3-D position
            # feedback at full gain: with the plant's own position loop
            # (kp=80) the two cascade into a self-oscillating loop the
            # moment the feedforward drops to zero at the endpoint.  Scale
            # the position correction like the orientation one.
            endpoint_pos_scale = jnp.where(sample.at_endpoint, 0.1, 1.0)
            twist_bias = jnp.concatenate([
                -endpoint_pos_scale * kp_pos
                * fb_scale * position_feedback_error,
                -endpoint_orient_scale * kp_orient * cap_orientation_err,
            ])
            twist_per_m = task_velocity_from_world(
                sample.tangent, sample.omega_per_m, sample.rotation)
            u_bias = (cap_jacobian_hash @ twist_bias
                      + cap_null_projector @ cap_qdot_null)
            u_per_m = cap_jacobian_hash @ twist_per_m
            return u_bias, u_per_m

        @jax.jit
        def _path_constraint_terms(q, h_args):
            return build_constraint_terms(q, h_args)

        @jax.jit
        def _path_feed_caps(u_bias, u_per_m, G, h_qp, u_safe_prev):
            joint_cap = feedrate_limit_from_box_jax(
                u_bias, u_per_m,
                -robot.joint_max_velocities,
                robot.joint_max_velocities,
            )
            cbf_cap = feedrate_limit_from_inequalities_jax(
                G[:cbf.num_cbf], h_qp[:cbf.num_cbf], u_bias, u_per_m)
            if enable_rate_limit:
                rate_cap = feedrate_limit_from_box_jax(
                    u_bias, u_per_m,
                    u_safe_prev - rate_limit,
                    u_safe_prev + rate_limit,
                )
            else:
                rate_cap = jnp.asarray(jnp.inf)
            return joint_cap, cbf_cap, rate_cap

        @jax.jit
        def _path_advance(path_state, ee_pos, joint_cap, cbf_cap, rate_cap):
            return advance_path_state_jax(
                path_geometry,
                path_config,
                path_state,
                ee_pos,
                dt_s=dt_path,
                feedrate_joint_limit_m_s=joint_cap,
                feedrate_cbf_limit_m_s=cbf_cap,
                feedrate_rate_limit_m_s=rate_cap,
            )

        @jax.jit
        def _path_control_nominal(ee_pos, ee_rot, full_jacobian, sample, q,
                                  kp_pos, kp_orient, kp_joint,
                                  nullspace_speed_limit, damping, q_des,
                                  feedrate_m_s):
            task_error, _err_report, control_jacobian = task_geometry_terms(
                ee_pos, ee_rot, full_jacobian,
                sample.position_m, sample.rotation)
            pos_err = task_error[:3]
            orientation_err = task_error[3:]
            tangent_error = sample.tangent * jnp.dot(sample.tangent, pos_err)
            transverse_pos_err = pos_err - tangent_error
            position_feedback_error = jnp.where(
                sample.at_endpoint, pos_err, transverse_pos_err)
            fb_norm = jnp.linalg.norm(position_feedback_error)
            fb_scale = jnp.where(
                fb_norm > _POSITION_FB_SATURATION_M,
                _POSITION_FB_SATURATION_M / jnp.maximum(fb_norm, _EPS),
                jnp.asarray(1.0),
            )
            endpoint_orient_scale = jnp.where(sample.at_endpoint, 0.1, 1.0)
            # Endpoint hold mode: same low-gain position correction as in
            # _path_cap_nominal (see note there on cascaded loop stability).
            endpoint_pos_scale = jnp.where(sample.at_endpoint, 0.1, 1.0)
            control_twist_bias = jnp.concatenate([
                -endpoint_pos_scale * kp_pos
                * fb_scale * position_feedback_error,
                -endpoint_orient_scale * kp_orient * orientation_err,
            ])
            control_twist_per_m = task_velocity_from_world(
                sample.tangent, sample.omega_per_m, sample.rotation)
            control_jacobian_hash = control_jacobian.T @ jnp.linalg.inv(
                control_jacobian @ control_jacobian.T
                + damping * jnp.eye(control_jacobian.shape[0]))
            control_null_projector = (
                jnp.eye(robot.num_joints)
                - control_jacobian_hash @ control_jacobian)
            control_q_des = _posture_target(sample.progress_m, q_des)
            if nullspace_policy is None:
                control_qdot_null = jnp.clip(
                    -kp_joint * (q - control_q_des),
                    -nullspace_speed_limit,
                    nullspace_speed_limit,
                )
            else:
                control_qdot_null = nullspace_policy.nullspace_velocity(
                    q, full_jacobian, control_jacobian_hash,
                    control_null_projector, robot.joint_max_velocities)
            control_u_bias = (
                control_jacobian_hash @ control_twist_bias
                + control_null_projector @ control_qdot_null)
            control_u_per_m = control_jacobian_hash @ control_twist_per_m
            u_nom = control_u_bias + control_u_per_m * feedrate_m_s
            u_nom = jnp.clip(
                u_nom, -robot.joint_max_velocities,
                robot.joint_max_velocities)
            control_task_p = task_hessian_from_jacobian(
                control_jacobian, tracking_task_weights_sq)
            return u_nom, control_task_p, control_q_des

        @jax.jit
        def _path_solve(q, u_nom, h_args, G, h_qp, h_vals, cbf_grad,
                        min_obs_dist, min_esdf_dist):
            return solve_from_constraint_terms(
                q, u_nom, h_args, G, h_qp, h_vals, cbf_grad,
                min_obs_dist, min_esdf_dist)

        @jax.jit
        def _path_finalize(q_next, ee_pos, ee_rot, full_jacobian, sample,
                           u_safe, path_state):
            ee_pos_next = robot.ee_position(q_next)
            ee_rot_next = robot.ee_rotation(q_next)
            next_path_state = reconcile_path_state_after_motion_jax(
                path_geometry,
                path_config,
                path_state,
                ee_pos_next,
                dt_s=dt_path,
            )
            _pos_err_next = ee_pos_next - sample.position_m
            _next_task_error, err_report_next, _next_jacobian = task_geometry_terms(
                ee_pos_next, ee_rot_next, full_jacobian,
                sample.position_m, sample.rotation)
            actual_tangent_speed = jnp.dot(
                full_jacobian[:3, :] @ u_safe, sample.tangent)
            return (next_path_state, err_report_next, actual_tangent_speed,
                    ee_pos_next, ee_rot_next)

        path_modules = (
            _path_kinematics, _path_sample, _path_cap_nominal,
            _path_constraint_terms, _path_feed_caps, _path_advance,
            _path_control_nominal, _path_solve, _path_finalize,
        )

        def path_tracking_step(q, path_state,
                               kp_pos, kp_orient, kp_joint, q_des,
                               nullspace_speed_limit, damping,
                               obs_pos, obs_radii, obs_enabled, obs_d_safe,
                               obs_vel, obs_radius_dot, obs_alpha, u_safe_prev,
                               sdf_distance, sdf_origin, sdf_voxel_size,
                               sdf_enabled, sdf_margin):
            """One modular-JIT path-following OSCBF step (host scheduler)."""
            ee_pos, ee_rot, full_jacobian = _path_kinematics(q)
            cap_sample = _path_sample(path_state[0])
            u_bias, u_per_m = _path_cap_nominal(
                ee_pos, ee_rot, full_jacobian, cap_sample, q,
                kp_pos, kp_orient, kp_joint, nullspace_speed_limit,
                damping, q_des)
            h_args = make_h_args(
                obs_pos, obs_radii, obs_enabled, obs_d_safe, obs_vel,
                obs_radius_dot, obs_alpha, u_safe_prev, sdf_distance,
                sdf_origin, sdf_voxel_size, sdf_enabled, sdf_margin,
                jnp.eye(robot.num_joints))
            G, h_qp, h_vals, cbf_grad, min_obs_dist, min_esdf_dist = (
                _path_constraint_terms(q, h_args))
            joint_cap, cbf_cap, rate_cap = _path_feed_caps(
                u_bias, u_per_m, G, h_qp, u_safe_prev)
            path = _path_advance(
                path_state, ee_pos, joint_cap, cbf_cap, rate_cap)
            sample = path.sample
            u_nom, control_task_p, control_q_des = _path_control_nominal(
                ee_pos, ee_rot, full_jacobian, sample, q,
                kp_pos, kp_orient, kp_joint, nullspace_speed_limit,
                damping, q_des, path.feedrate_m_s)
            control_h_args = make_h_args(
                obs_pos, obs_radii, obs_enabled, obs_d_safe, obs_vel,
                obs_radius_dot, obs_alpha, u_safe_prev, sdf_distance,
                sdf_origin, sdf_voxel_size, sdf_enabled, sdf_margin,
                control_task_p)
            (q_next, u_safe, u_candidate, qp_ok, _min_obs_dist,
             _min_esdf_dist, rate_relaxation, rate_solver_slack, _h_vals,
             _cbf_grad, active_count,
             primal_residual, terminal_kkt_residual,
             terminal_kkt_accepted, dual_max, qp_iterations,
             delta_slack) = _path_solve(
                q, u_nom, control_h_args, G, h_qp, h_vals, cbf_grad,
                min_obs_dist, min_esdf_dist)
            (next_path_state, err_report_next, actual_tangent_speed,
             ee_pos_next, ee_rot_next) = _path_finalize(
                q_next, ee_pos, ee_rot, full_jacobian, sample, u_safe,
                path.state)
            return (
                q_next, u_safe, u_candidate, u_nom, err_report_next,
                ee_pos_next, ee_rot_next,
                qp_ok, min_obs_dist, min_esdf_dist, rate_relaxation,
                rate_solver_slack, h_vals,
                cbf_grad, active_count, primal_residual,
                terminal_kkt_residual, terminal_kkt_accepted, dual_max,
                qp_iterations, delta_slack, next_path_state,
                sample.position_m, sample.rotation,
                sample.tangent, sample.omega_per_m, sample.source_time_s,
                path.cross_track_error_m, path.gamma, path.feedrate_nominal_m_s,
                path.feedrate_m_s,
                path.feedrate_joint_limit_m_s, path.feedrate_cbf_limit_m_s,
                path.feedrate_rate_limit_m_s,
                path.feedrate_tool_axis_limit_m_s,
                path.feedrate_endpoint_brake_limit_m_s,
                path.limiting_reason_code,
                actual_tangent_speed, sample.at_endpoint, control_q_des,
            )

        # Compatibility surface used by legacy tests that assert a single
        # JIT cache entry: report the maximum module cache size (1 after the
        # first call, i.e. no recompilation).
        def _path_cache_size():
            return max(module._cache_size() for module in path_modules)

        path_tracking_step._cache_size = _path_cache_size
        path_tracking = path_tracking_step

    return JaxControlKernels(
        step=jax.jit(solve_step),
        tracking=jax.jit(tracking_step),
        tracking_fast=jax.jit(tracking_fast_step),
        task_hessian=jax.jit(task_hessian_from_q),
        path_tracking=path_tracking,
        qp_problem=jax.jit(qp_problem_step),
        qp_core=(
            None if (cbf.relax_cbf and not enable_rate_limit)
            else jax.jit(qp_core_step)),
        path_modules=path_modules,
    )
