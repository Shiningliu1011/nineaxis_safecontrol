"""Fixed-shape JAX helpers for arc-length Cartesian path following.

The host-side :mod:`work.path_following` module owns validation and readable
NumPy state.  This module mirrors its numerical state machine with JAX arrays
so reference sampling, local projection, and feedrate limiting can stay in
the same compiled control entry point as OSC, CBF-QP, and integration.

All distances are metres, path speeds are metres per second, and angular
rates are radians per metre.  The path arrays are static closure data: a
runtime step never changes their shape or the QP row count.
"""

from __future__ import annotations

from typing import NamedTuple

import jax.numpy as jnp
import numpy as np

from work.path_following import PathFollowingConfig, PathGeometry


_EPS = 1.0e-9


class JaxPathGeometry(NamedTuple):
    """Array-only immutable path geometry captured by a JIT closure."""

    positions_m: jnp.ndarray
    quaternions_xyzw: jnp.ndarray
    arc_length_m: jnp.ndarray
    tangents: jnp.ndarray
    omega_per_m: jnp.ndarray
    feedrate_m_s: jnp.ndarray
    source_time_s: jnp.ndarray


class JaxPathFollowingConfig(NamedTuple):
    """Static scalar controls for the compiled one-way path state machine."""

    projection_half_window_segments: int
    max_projection_speed_m_s: float
    reference_lead_m: float
    cross_track_stop_m: float
    endpoint_braking_deceleration_m_s2: float
    endpoint_settle_s: float
    maximum_tool_axis_speed_rad_s: float


class JaxPathSample(NamedTuple):
    """One interpolated task reference returned from a JAX path sample."""

    progress_m: jnp.ndarray
    position_m: jnp.ndarray
    tangent: jnp.ndarray
    rotation: jnp.ndarray
    omega_per_m: jnp.ndarray
    feedrate_m_s: jnp.ndarray
    source_time_s: jnp.ndarray
    segment_index: jnp.ndarray
    at_endpoint: jnp.ndarray


class JaxPathAdvance(NamedTuple):
    """One path state update and its feedrate diagnostics."""

    state: jnp.ndarray
    sample: JaxPathSample
    cross_track_error_m: jnp.ndarray
    gamma: jnp.ndarray
    feedrate_nominal_m_s: jnp.ndarray
    feedrate_m_s: jnp.ndarray
    feedrate_joint_limit_m_s: jnp.ndarray
    feedrate_cbf_limit_m_s: jnp.ndarray
    feedrate_rate_limit_m_s: jnp.ndarray
    feedrate_tool_axis_limit_m_s: jnp.ndarray
    feedrate_endpoint_brake_limit_m_s: jnp.ndarray
    limiting_reason_code: jnp.ndarray


# State layout is intentionally a simple fixed vector so it can be passed
# through jax.jit without Python objects or shape changes.
PATH_STATE_REFERENCE_PROGRESS = 0
PATH_STATE_PROJECTED_PROGRESS = 1
PATH_STATE_PROJECTION_SEGMENT = 2
PATH_STATE_ENDPOINT_HOLD = 3
PATH_STATE_COMPLETED = 4
PATH_STATE_SIZE = 5


PATH_LIMIT_NOMINAL = 0
PATH_LIMIT_JOINT = 1
PATH_LIMIT_CBF = 2
PATH_LIMIT_RATE = 3
PATH_LIMIT_CROSS_TRACK = 4
PATH_LIMIT_ENDPOINT = 5
PATH_LIMIT_ENDPOINT_BRAKE = 6


def as_jax_path_geometry(geometry: PathGeometry) -> JaxPathGeometry:
    """Convert validated host geometry once, before JIT warmup."""
    return JaxPathGeometry(
        positions_m=jnp.asarray(geometry.positions_m),
        quaternions_xyzw=jnp.asarray(geometry.quaternions_xyzw),
        arc_length_m=jnp.asarray(geometry.arc_length_m),
        tangents=jnp.asarray(geometry.tangents),
        omega_per_m=jnp.asarray(geometry.omega_per_m),
        feedrate_m_s=jnp.asarray(geometry.feedrate_m_s),
        source_time_s=jnp.asarray(geometry.source_time_s),
    )


def as_jax_path_config(config: PathFollowingConfig) -> JaxPathFollowingConfig:
    """Copy validated host configuration into static JAX-kernel scalars."""
    return JaxPathFollowingConfig(
        projection_half_window_segments=int(config.projection_half_window_segments),
        max_projection_speed_m_s=float(config.max_projection_speed_m_s),
        reference_lead_m=float(config.reference_lead_m),
        cross_track_stop_m=float(config.cross_track_stop_m),
        endpoint_braking_deceleration_m_s2=float(
            config.endpoint_braking_deceleration_m_s2),
        endpoint_settle_s=float(config.endpoint_settle_s),
        maximum_tool_axis_speed_rad_s=float(config.maximum_tool_axis_speed_rad_s),
    )


def initial_path_state_jax(dtype=None) -> jnp.ndarray:
    """Return the initial one-way progress state as a fixed-length vector."""
    return (jnp.zeros(PATH_STATE_SIZE)
            if dtype is None else jnp.zeros(PATH_STATE_SIZE, dtype=dtype))


def sample_path_jax(geometry: JaxPathGeometry, progress_m: jnp.ndarray) -> JaxPathSample:
    """Interpolate a 6-D path reference at clamped arc-length progress."""
    n_segments = geometry.positions_m.shape[0] - 1
    total = geometry.arc_length_m[-1]
    progress = jnp.clip(progress_m, 0.0, total)
    segment = jnp.clip(
        jnp.searchsorted(geometry.arc_length_m, progress, side="right") - 1,
        0,
        n_segments - 1,
    ).astype(jnp.int32)
    start = geometry.arc_length_m[segment]
    span = geometry.arc_length_m[segment + 1] - start
    fraction = jnp.clip((progress - start) / jnp.maximum(span, _EPS), 0.0, 1.0)

    position = _lerp(geometry.positions_m[segment], geometry.positions_m[segment + 1], fraction)
    tangent = _normalize(
        _lerp(geometry.tangents[segment], geometry.tangents[segment + 1], fraction),
        geometry.tangents[segment],
    )
    quaternion = _slerp(
        geometry.quaternions_xyzw[segment],
        geometry.quaternions_xyzw[segment + 1],
        fraction,
    )
    return JaxPathSample(
        progress_m=progress,
        position_m=position,
        tangent=tangent,
        rotation=_quaternion_to_rotation(quaternion),
        omega_per_m=_lerp(
            geometry.omega_per_m[segment], geometry.omega_per_m[segment + 1], fraction),
        feedrate_m_s=jnp.maximum(
            _lerp(geometry.feedrate_m_s[segment], geometry.feedrate_m_s[segment + 1], fraction),
            0.0,
        ),
        source_time_s=_lerp(
            geometry.source_time_s[segment], geometry.source_time_s[segment + 1], fraction),
        segment_index=segment,
        at_endpoint=progress >= total - _EPS,
    )


def project_local_path_jax(geometry: JaxPathGeometry, position_m: jnp.ndarray,
                           anchor_segment: jnp.ndarray,
                           half_window_segments: int) -> tuple[jnp.ndarray, jnp.ndarray]:
    """Project onto a constant-size local segment window without branch jumps."""
    n_segments = geometry.positions_m.shape[0] - 1
    offsets = jnp.arange(-half_window_segments, half_window_segments + 1)
    anchor = jnp.clip(anchor_segment.astype(jnp.int32), 0, n_segments - 1)
    segments = jnp.clip(anchor + offsets, 0, n_segments - 1).astype(jnp.int32)
    starts = geometry.positions_m[segments]
    deltas = geometry.positions_m[segments + 1] - starts
    lengths_sq = jnp.sum(deltas * deltas, axis=1)
    fractions = jnp.sum((position_m[None, :] - starts) * deltas, axis=1)
    fractions = jnp.clip(fractions / jnp.maximum(lengths_sq, _EPS), 0.0, 1.0)
    closest = starts + fractions[:, None] * deltas
    best = jnp.argmin(jnp.sum((closest - position_m[None, :]) ** 2, axis=1))
    segment = segments[best]
    progress = geometry.arc_length_m[segment] + fractions[best] * (
        geometry.arc_length_m[segment + 1] - geometry.arc_length_m[segment]
    )
    return progress, segment


def advance_path_state_jax(geometry: JaxPathGeometry,
                           config: JaxPathFollowingConfig,
                           state: jnp.ndarray,
                           ee_position_m: jnp.ndarray,
                           *,
                           dt_s: float,
                           feedrate_joint_limit_m_s: jnp.ndarray,
                           feedrate_cbf_limit_m_s: jnp.ndarray,
                           feedrate_rate_limit_m_s: jnp.ndarray) -> JaxPathAdvance:
    """Advance toward the predicted end-of-cycle projection after feed caps."""
    reference_progress = state[PATH_STATE_REFERENCE_PROGRESS]
    projected_progress = state[PATH_STATE_PROJECTED_PROGRESS]
    projection_segment = state[PATH_STATE_PROJECTION_SEGMENT]
    endpoint_hold = state[PATH_STATE_ENDPOINT_HOLD]
    completed_before = state[PATH_STATE_COMPLETED] > 0.5
    sample = sample_path_jax(geometry, reference_progress)

    desired_minus_actual = sample.position_m - ee_position_m
    tangent_component = sample.tangent * jnp.dot(sample.tangent, desired_minus_actual)
    lateral = desired_minus_actual - tangent_component
    error_for_progress = jnp.where(sample.at_endpoint, desired_minus_actual, lateral)
    cross_track_error = jnp.linalg.norm(error_for_progress)
    gamma = jnp.clip(1.0 - cross_track_error / config.cross_track_stop_m, 0.0, 1.0)

    projection_raw, _ = project_local_path_jax(
        geometry,
        ee_position_m,
        projection_segment,
        config.projection_half_window_segments,
    )
    max_projected = jnp.minimum(
        geometry.arc_length_m[-1],
        projected_progress + config.max_projection_speed_m_s * dt_s,
    )
    projected = jnp.clip(projection_raw, projected_progress, max_projected)
    projected_segment = jnp.clip(
        jnp.searchsorted(geometry.arc_length_m, projected, side="right") - 1,
        0,
        geometry.positions_m.shape[0] - 2,
    )

    joint_cap = _finite_nonnegative(feedrate_joint_limit_m_s)
    cbf_cap = _finite_nonnegative(feedrate_cbf_limit_m_s)
    rate_cap = _finite_nonnegative(feedrate_rate_limit_m_s)
    endpoint_brake_cap = jnp.sqrt(jnp.maximum(
        2.0 * config.endpoint_braking_deceleration_m_s2 * (
            geometry.arc_length_m[-1] - sample.progress_m),
        0.0,
    ))
    omega_norm = jnp.linalg.norm(sample.omega_per_m)
    tool_axis_cap = jnp.where(
        omega_norm > _EPS,
        config.maximum_tool_axis_speed_rad_s / omega_norm,
        jnp.inf,
    )
    feed_probe_progress = jnp.minimum(
        geometry.arc_length_m[-1],
        reference_progress + config.max_projection_speed_m_s * dt_s,
    )
    feedrate_nominal = jnp.where(
        sample.feedrate_m_s <= _EPS,
        sample_path_jax(geometry, feed_probe_progress).feedrate_m_s,
        sample.feedrate_m_s,
    )
    cap = jnp.minimum(
        feedrate_nominal,
        jnp.minimum(
            joint_cap,
            jnp.minimum(
                cbf_cap,
                jnp.minimum(rate_cap, jnp.minimum(tool_axis_cap, endpoint_brake_cap)),
            ),
        ),
    )
    endpoint = sample.at_endpoint | completed_before
    feedrate = jnp.where(endpoint, 0.0, gamma * cap)
    # Bounded-lead scheduling: the reference follows the feed schedule
    # (reference + feedrate*dt_s) but may never lead the end-effector
    # projection by more than reference_lead_m.  This keeps the feed at the
    # nominal speed while the end effector keeps up, yet freezes the
    # reference when the effector lags (a stalled projection must not leave
    # the reference racing ahead).  It replaces the old anchor chain, which
    # pinned the reference to the projection advance and throttled the feed
    # ~20x.  Safety semantics are unchanged: gamma and the feed caps freeze
    # the reference when the cross-track error exceeds the stop threshold or
    # near the endpoint.
    candidate = jnp.minimum(
        geometry.arc_length_m[-1], reference_progress + feedrate * dt_s)
    lead_capped = jnp.minimum(
        geometry.arc_length_m[-1], projected + config.reference_lead_m)
    next_reference_progress = jnp.maximum(
        reference_progress, jnp.minimum(candidate, lead_capped))
    completed = next_reference_progress >= geometry.arc_length_m[-1] - _EPS
    next_endpoint_hold = jnp.where(completed, endpoint_hold + dt_s, 0.0)
    next_state = jnp.array([
        next_reference_progress,
        projected,
        projected_segment.astype(reference_progress.dtype),
        next_endpoint_hold,
        completed.astype(reference_progress.dtype),
    ])
    control_sample = sample_path_jax(geometry, next_reference_progress)

    limiting_reason = _limiting_reason_code(
        feedrate_nominal, joint_cap, cbf_cap, rate_cap, tool_axis_cap,
        endpoint_brake_cap, gamma, endpoint)
    return JaxPathAdvance(
        state=next_state,
        sample=control_sample,
        cross_track_error_m=cross_track_error,
        gamma=gamma,
        feedrate_nominal_m_s=feedrate_nominal,
        feedrate_m_s=feedrate,
        feedrate_joint_limit_m_s=joint_cap,
        feedrate_cbf_limit_m_s=cbf_cap,
        feedrate_rate_limit_m_s=rate_cap,
        feedrate_tool_axis_limit_m_s=tool_axis_cap,
        feedrate_endpoint_brake_limit_m_s=endpoint_brake_cap,
        limiting_reason_code=limiting_reason,
    )


def reconcile_path_state_after_motion_jax(
    geometry: JaxPathGeometry,
    config: JaxPathFollowingConfig,
    state: jnp.ndarray,
    ee_position_m: jnp.ndarray,
    *,
    dt_s: float,
) -> jnp.ndarray:
    """Write the measured post-QP local projection into fixed path state.

    The command reference remains fixed for the completed sample.  For the
    next cycle the virtual phase catches up to a bounded projection that has
    moved ahead, preventing a persistent one-cycle tangent lag.
    """
    projected_before = state[PATH_STATE_PROJECTED_PROGRESS]
    anchor_before = state[PATH_STATE_PROJECTION_SEGMENT]
    raw_projection, _ = project_local_path_jax(
        geometry,
        ee_position_m,
        anchor_before,
        config.projection_half_window_segments,
    )
    maximum = jnp.minimum(
        geometry.arc_length_m[-1],
        projected_before + config.max_projection_speed_m_s * dt_s,
    )
    projected = jnp.clip(raw_projection, projected_before, maximum)
    segment = jnp.clip(
        jnp.searchsorted(geometry.arc_length_m, projected, side="right") - 1,
        0,
        geometry.positions_m.shape[0] - 2,
    ).astype(state.dtype)
    # The reference is advanced by advance_path_state_jax only; the measured
    # projection must not pull it backwards (or catch up with it).  This keeps
    # the reference on its feed schedule even when the ee projection lags by
    # the steady-state tracking error.
    reference = state[PATH_STATE_REFERENCE_PROGRESS]
    completed = reference >= geometry.arc_length_m[-1] - _EPS
    return state.at[PATH_STATE_REFERENCE_PROGRESS].set(reference).at[
        PATH_STATE_PROJECTED_PROGRESS].set(projected).at[
        PATH_STATE_PROJECTION_SEGMENT].set(segment).at[
        PATH_STATE_COMPLETED].set(completed.astype(state.dtype))


def feedrate_limit_from_box_jax(u_bias: jnp.ndarray, u_per_m: jnp.ndarray,
                                lower: jnp.ndarray, upper: jnp.ndarray) -> jnp.ndarray:
    """Largest non-negative feedrate satisfying ``lower <= bias+s*ell_dot <= upper``."""
    slope = u_per_m
    positive = slope > _EPS
    negative = slope < -_EPS
    stationary_bad = (~positive & ~negative) & (
        (u_bias < lower - _EPS) | (u_bias > upper + _EPS)
    )
    positive_cap = (upper - u_bias) / jnp.where(positive, slope, 1.0)
    negative_cap = (lower - u_bias) / jnp.where(negative, slope, -1.0)
    caps = jnp.where(positive, positive_cap, jnp.where(negative, negative_cap, jnp.inf))
    cap = jnp.minimum(jnp.min(caps), jnp.inf)
    cap = jnp.where(jnp.any(stationary_bad), 0.0, cap)
    return jnp.maximum(cap, 0.0)


def feedrate_limit_from_inequalities_jax(G: jnp.ndarray, h: jnp.ndarray,
                                         u_bias: jnp.ndarray,
                                         u_per_m: jnp.ndarray) -> jnp.ndarray:
    """Largest non-negative feedrate satisfying hard ``G @ u <= h`` rows."""
    residual = h - G @ u_bias
    slope = G @ u_per_m
    positive = slope > _EPS
    stationary_bad = (~positive) & (jnp.abs(slope) <= _EPS) & (residual < -_EPS)
    caps = jnp.where(positive, residual / jnp.where(positive, slope, 1.0), jnp.inf)
    cap = jnp.min(caps)
    cap = jnp.where(jnp.any(stationary_bad), 0.0, cap)
    return jnp.maximum(cap, 0.0)


PATH_LIMIT_TOOL_AXIS = 7


def _limiting_reason_code(nominal: jnp.ndarray, joint_cap: jnp.ndarray,
                          cbf_cap: jnp.ndarray, rate_cap: jnp.ndarray,
                          tool_axis_cap: jnp.ndarray,
                          endpoint_brake_cap: jnp.ndarray,
                          gamma: jnp.ndarray, endpoint: jnp.ndarray) -> jnp.ndarray:
    """Return a stable numeric reason for reports without string host logic."""
    code = jnp.asarray(PATH_LIMIT_NOMINAL, dtype=jnp.int32)
    code = jnp.where(joint_cap < nominal, PATH_LIMIT_JOINT, code)
    code = jnp.where(cbf_cap < jnp.minimum(nominal, joint_cap), PATH_LIMIT_CBF, code)
    code = jnp.where(rate_cap < jnp.minimum(nominal, jnp.minimum(joint_cap, cbf_cap)), PATH_LIMIT_RATE, code)
    code = jnp.where(
        tool_axis_cap < jnp.minimum(
            nominal, jnp.minimum(joint_cap, jnp.minimum(cbf_cap, rate_cap))),
        PATH_LIMIT_TOOL_AXIS,
        code,
    )
    code = jnp.where(
        endpoint_brake_cap < jnp.minimum(
            nominal, jnp.minimum(joint_cap, jnp.minimum(
                cbf_cap, jnp.minimum(rate_cap, tool_axis_cap)))),
        PATH_LIMIT_ENDPOINT_BRAKE,
        code,
    )
    code = jnp.where(gamma <= _EPS, PATH_LIMIT_CROSS_TRACK, code)
    return jnp.where(endpoint, PATH_LIMIT_ENDPOINT, code)


def _finite_nonnegative(value: jnp.ndarray) -> jnp.ndarray:
    return jnp.where(jnp.isnan(value) | (value < 0.0), 0.0, value)


def _lerp(a: jnp.ndarray, b: jnp.ndarray, fraction: jnp.ndarray) -> jnp.ndarray:
    return (1.0 - fraction) * a + fraction * b


def _normalize(vector: jnp.ndarray, fallback: jnp.ndarray) -> jnp.ndarray:
    norm = jnp.linalg.norm(vector)
    fallback_norm = jnp.linalg.norm(fallback)
    unit_first_axis = jnp.zeros_like(vector).at[0].set(1.0)
    safe_fallback = jnp.where(
        fallback_norm > _EPS,
        fallback / jnp.maximum(fallback_norm, _EPS),
        unit_first_axis,
    )
    return jnp.where(norm > _EPS, vector / norm, safe_fallback)


def _slerp(q0: jnp.ndarray, q1: jnp.ndarray, fraction: jnp.ndarray) -> jnp.ndarray:
    dot = jnp.clip(jnp.dot(q0, q1), -1.0, 1.0)
    q1 = jnp.where(dot < 0.0, -q1, q1)
    dot = jnp.abs(dot)
    theta = jnp.arccos(jnp.clip(dot, -1.0, 1.0))
    sin_theta = jnp.sin(theta)
    linear = _normalize(_lerp(q0, q1, fraction), q0)
    spherical = (
        jnp.sin((1.0 - fraction) * theta) / jnp.maximum(sin_theta, _EPS) * q0
        + jnp.sin(fraction * theta) / jnp.maximum(sin_theta, _EPS) * q1
    )
    return _normalize(jnp.where(dot > 0.9995, linear, spherical), q0)


def _quaternion_to_rotation(quaternion: jnp.ndarray) -> jnp.ndarray:
    x, y, z, w = _normalize(quaternion, jnp.array([0.0, 0.0, 0.0, 1.0]))
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return jnp.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ])


__all__ = [
    "JaxPathAdvance",
    "JaxPathFollowingConfig",
    "JaxPathGeometry",
    "JaxPathSample",
    "PATH_LIMIT_CBF",
    "PATH_LIMIT_CROSS_TRACK",
    "PATH_LIMIT_ENDPOINT",
    "PATH_LIMIT_ENDPOINT_BRAKE",
    "PATH_LIMIT_JOINT",
    "PATH_LIMIT_NOMINAL",
    "PATH_LIMIT_RATE",
    "PATH_LIMIT_TOOL_AXIS",
    "PATH_STATE_COMPLETED",
    "PATH_STATE_ENDPOINT_HOLD",
    "PATH_STATE_PROJECTION_SEGMENT",
    "PATH_STATE_PROJECTED_PROGRESS",
    "PATH_STATE_REFERENCE_PROGRESS",
    "as_jax_path_config",
    "as_jax_path_geometry",
    "advance_path_state_jax",
    "feedrate_limit_from_box_jax",
    "feedrate_limit_from_inequalities_jax",
    "initial_path_state_jax",
    "reconcile_path_state_after_motion_jax",
    "sample_path_jax",
]
