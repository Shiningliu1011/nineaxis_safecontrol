"""Pure fixed-shape JAX geometry and CBF RHS operations.

These functions have no ROS, NumPy host state or solver lifecycle.  Keeping
them separate makes it possible to test a barrier geometry change without
rebuilding the control-loop facade.
"""

from __future__ import annotations

import jax
import jax.numpy as jnp


# Unused slots are masked rather than removed, so obstacle count never changes
# the JIT/QP matrix shape.
MAX_JAX_OBSTACLES = 8


def compute_obstacle_clearance(center_deltas, robot_radii, obs_radii, obs_d_safe):
    """Return the dynamic obstacle clearance ``h(q, t)`` for every sphere/slot."""
    distances = jnp.linalg.norm(center_deltas, axis=-1)
    return distances - robot_radii[:, None] - obs_radii[None, :] - obs_d_safe[None, :]


def compute_dcol_obstacle_clearance(q, obs_pos, obs_radii, obs_d_safe,
                                    obs_vel, obs_radius_dot):
    """DCOL OBB-vs-sphere clearance and time terms (M7 hot path).

    Returns ``(h_obs, h_dot_obs)`` shaped ``(N_obb, N_obs)``: the surface
    distance minus the per-slot safety margin, and the dynamic time terms
    from the DCOL distance gradient (zero for static obstacles).
    """

    from work.dpax_collision import obb_sphere_clearance

    return obb_sphere_clearance(
        q, obs_pos, obs_radii, obs_vel, obs_radius_dot, obs_d_safe)


def compute_obstacle_time_terms(center_deltas, obs_vel, obs_radius_dot):
    """Return ``dh_dt = -n.T @ v_obs - r_obs_dot`` for every sphere/slot."""
    distances = jnp.linalg.norm(center_deltas, axis=-1, keepdims=True)
    normals = jnp.where(
        distances > 1e-9,
        center_deltas / jnp.maximum(distances, 1e-9),
        jnp.zeros_like(center_deltas),
    )
    return -jnp.sum(normals * obs_vel[None, :, :], axis=-1) - obs_radius_dot[None, :]


def apply_dynamic_obstacle_cbf_terms(qp_h, clearance, time_terms,
                                     obs_enabled, obs_alpha, *,
                                     obstacle_start, baseline_alpha):
    """Patch fixed per-obstacle CBF rows with gain and time-derivative terms."""
    row_count = clearance.size
    enabled_rows = jnp.broadcast_to(
        obs_enabled[None, :] > 0.5, clearance.shape).ravel()
    correction = (
        (obs_alpha[None, :] - baseline_alpha) * clearance + time_terms
    ).ravel()
    correction = jnp.where(enabled_rows, correction, 0.0)
    return qp_h.at[obstacle_start:obstacle_start + row_count].add(correction)


def aggregate_dynamic_obstacle_terms(clearance, time_terms, obs_enabled,
                                     obs_alpha, temperature):
    """Build one conservative soft-min CBF term per robot collision sphere."""
    temp = jnp.asarray(temperature, dtype=clearance.dtype)
    masked = jnp.where(obs_enabled[None, :] > 0.5, clearance, 1e3)
    weights = jax.nn.softmax(-masked / temp, axis=1)
    h_aggregate = -temp * jax.scipy.special.logsumexp(-masked / temp, axis=1)
    dh_dt_aggregate = jnp.sum(weights * time_terms, axis=1)
    alpha_aggregate = jnp.sum(weights * obs_alpha[None, :], axis=1)
    return h_aggregate, dh_dt_aggregate, alpha_aggregate


def apply_aggregated_dynamic_cbf_terms(qp_h, clearance, time_terms,
                                       obs_enabled, obs_alpha, *,
                                       obstacle_start, baseline_alpha,
                                       temperature):
    """Patch cbfpy's static obstacle rows with fixed-shape aggregate terms."""
    h_aggregate, dh_dt_aggregate, alpha_aggregate = aggregate_dynamic_obstacle_terms(
        clearance, time_terms, obs_enabled, obs_alpha, temperature)
    has_obstacle = jnp.any(obs_enabled > 0.5)
    correction = (
        (alpha_aggregate - baseline_alpha) * h_aggregate + dh_dt_aggregate)
    correction = jnp.where(has_obstacle, correction, jnp.zeros_like(correction))
    row_count = clearance.shape[0]
    return qp_h.at[obstacle_start:obstacle_start + row_count].add(correction)


def apply_qp_health_gate(q, u_candidate, qp_ok, *, dt, q_min, q_max):
    """Integrate only a candidate accepted by the hard-CBF solver health gate."""
    u_applied = jnp.where(qp_ok, u_candidate, jnp.zeros_like(u_candidate))
    q_next = jnp.clip(q + u_applied * dt, q_min, q_max)
    return q_next, u_applied
