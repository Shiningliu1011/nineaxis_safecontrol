"""Geometry and state helpers for one-way Cartesian path following.

This module is deliberately independent from ROS, QP solvers, and robot
kinematics.  It converts a sampled task trajectory into an arc-length path
and exposes the small state machine shared by the NumPy and JAX controllers.
All distances are metres and all path speeds are metres per second.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


_EPS = 1.0e-9


# Numeric values are written to CSV so JAX and NumPy runs can use the same
# compact reason field without changing a compiled output shape.
PATH_LIMIT_NOMINAL = 0
PATH_LIMIT_JOINT = 1
PATH_LIMIT_CBF = 2
PATH_LIMIT_RATE = 3
PATH_LIMIT_CROSS_TRACK = 4
PATH_LIMIT_ENDPOINT = 5
PATH_LIMIT_ENDPOINT_BRAKE = 6


@dataclass(frozen=True)
class PathFollowingConfig:
    """Fixed path-following limits with explicit physical units."""

    projection_half_window_segments: int = 96
    max_projection_speed_m_s: float = 0.12
    reference_lead_m: float = 1.0e-5
    cross_track_stop_m: float = 1.0e-3
    endpoint_braking_deceleration_m_s2: float = 5.0e-2
    endpoint_settle_s: float = 0.5
    maximum_tool_axis_speed_rad_s: float = 0.15
    maximum_reference_feedrate_step_m_s: float = 0.005

    def __post_init__(self) -> None:
        if int(self.projection_half_window_segments) < 1:
            raise ValueError("projection_half_window_segments must be positive")
        for name in (
            "max_projection_speed_m_s",
            "reference_lead_m",
            "cross_track_stop_m",
            "endpoint_braking_deceleration_m_s2",
            "endpoint_settle_s",
            "maximum_tool_axis_speed_rad_s",
            "maximum_reference_feedrate_step_m_s",
        ):
            value = float(getattr(self, name))
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class PathReference:
    """A continuous 6-D reference sampled at one arc-length progress."""

    progress_m: float
    position_m: np.ndarray
    tangent: np.ndarray
    rotation: np.ndarray
    omega_per_m: np.ndarray
    feedrate_m_s: float
    source_time_s: float
    segment_index: int
    at_endpoint: bool


@dataclass(frozen=True)
class PathFollowerState:
    """Persistent one-way path progress state.

    ``reference_progress_m`` is the virtual point used by OSC.  The projected
    progress is derived from the actual end-effector position but never moves
    backwards, so a self-intersection cannot switch the task to an old branch.
    """

    reference_progress_m: float = 0.0
    projected_progress_m: float = 0.0
    projection_segment: int = 0
    endpoint_hold_s: float = 0.0
    completed: bool = False


@dataclass(frozen=True)
class PathFollowingStep:
    """Reference and diagnostics for one control period."""

    reference: PathReference
    next_state: PathFollowerState
    cross_track_error_m: float
    gamma: float
    feedrate_nominal_m_s: float
    feedrate_m_s: float
    feedrate_joint_limit_m_s: float
    feedrate_cbf_limit_m_s: float
    feedrate_rate_limit_m_s: float
    feedrate_tool_axis_limit_m_s: float
    feedrate_endpoint_brake_limit_m_s: float
    limiting_reason: str


@dataclass(frozen=True)
class PathGeometry:
    """Arc-length parameterized immutable task path."""

    positions_m: np.ndarray
    rotations: np.ndarray
    quaternions_xyzw: np.ndarray
    arc_length_m: np.ndarray
    tangents: np.ndarray
    omega_per_m: np.ndarray
    feedrate_m_s: np.ndarray
    source_time_s: np.ndarray

    @classmethod
    def from_samples(
        cls,
        positions_m: np.ndarray,
        rotations: np.ndarray,
        feedrate_m_s: np.ndarray,
        source_time_s: np.ndarray,
        *,
        min_segment_m: float = 1.0e-7,
    ) -> "PathGeometry":
        """Build a path after removing numerically zero-length segments."""
        positions = np.asarray(positions_m, dtype=float)
        rotation_series = np.asarray(rotations, dtype=float)
        feedrate = np.asarray(feedrate_m_s, dtype=float).reshape(-1)
        source_time = np.asarray(source_time_s, dtype=float).reshape(-1)
        if positions.ndim != 2 or positions.shape[1] != 3:
            raise ValueError("positions_m must have shape (N, 3)")
        count = positions.shape[0]
        if count < 2:
            raise ValueError("at least two path samples are required")
        if rotation_series.shape != (count, 3, 3):
            raise ValueError("rotations must have shape (N, 3, 3)")
        if feedrate.shape != (count,) or source_time.shape != (count,):
            raise ValueError("feedrate_m_s and source_time_s must have length N")
        if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(rotation_series)):
            raise ValueError("path geometry must be finite")
        if not np.all(np.isfinite(feedrate)) or not np.all(np.isfinite(source_time)):
            raise ValueError("path schedule must be finite")
        if min_segment_m <= 0.0:
            raise ValueError("min_segment_m must be positive")

        keep = [0]
        for index in range(1, count):
            if np.linalg.norm(positions[index] - positions[keep[-1]]) >= min_segment_m:
                keep.append(index)
        if len(keep) < 2:
            raise ValueError("path contains no non-degenerate segment")

        indices = np.asarray(keep, dtype=int)
        positions = positions[indices].copy()
        rotation_series = rotation_series[indices].copy()
        feedrate = np.maximum(feedrate[indices], 0.0)
        source_time = source_time[indices].copy()
        segment_lengths = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        if np.any(segment_lengths < min_segment_m):
            raise AssertionError("degenerate segment escaped compaction")
        arc_length = np.concatenate([[0.0], np.cumsum(segment_lengths)])

        tangents = _point_tangents(positions)
        quaternions = Rotation.from_matrix(rotation_series).as_quat()
        quaternions = _canonicalize_quaternion_signs(quaternions)
        omega_per_m = _point_omega_per_m(rotation_series, segment_lengths)
        return cls(
            positions_m=positions,
            rotations=rotation_series,
            quaternions_xyzw=quaternions,
            arc_length_m=arc_length,
            tangents=tangents,
            omega_per_m=omega_per_m,
            feedrate_m_s=feedrate,
            source_time_s=source_time,
        )

    @property
    def num_points(self) -> int:
        return int(self.positions_m.shape[0])

    @property
    def num_segments(self) -> int:
        return self.num_points - 1

    @property
    def total_length_m(self) -> float:
        return float(self.arc_length_m[-1])

    def sample(self, progress_m: float) -> PathReference:
        """Interpolate one 6-D path reference at a clamped arc length."""
        progress = float(np.clip(progress_m, 0.0, self.total_length_m))
        segment, fraction = self._segment_fraction(progress)
        p0, p1 = self.positions_m[segment:segment + 2]
        tangent = _normalize(
            (1.0 - fraction) * self.tangents[segment]
            + fraction * self.tangents[segment + 1],
            self.tangents[segment],
        )
        quaternion = _slerp(
            self.quaternions_xyzw[segment],
            self.quaternions_xyzw[segment + 1],
            fraction,
        )
        rotation = Rotation.from_quat(quaternion).as_matrix()
        omega_per_m = (
            (1.0 - fraction) * self.omega_per_m[segment]
            + fraction * self.omega_per_m[segment + 1]
        )
        feedrate = float(
            (1.0 - fraction) * self.feedrate_m_s[segment]
            + fraction * self.feedrate_m_s[segment + 1]
        )
        source_time = float(
            (1.0 - fraction) * self.source_time_s[segment]
            + fraction * self.source_time_s[segment + 1]
        )
        return PathReference(
            progress_m=progress,
            position_m=(1.0 - fraction) * p0 + fraction * p1,
            tangent=tangent,
            rotation=rotation,
            omega_per_m=omega_per_m,
            feedrate_m_s=max(feedrate, 0.0),
            source_time_s=source_time,
            segment_index=segment,
            at_endpoint=progress >= self.total_length_m - _EPS,
        )

    def project_local(
        self,
        position_m: np.ndarray,
        *,
        anchor_segment: int,
        half_window_segments: int,
    ) -> tuple[float, int]:
        """Project a point onto a fixed local segment window.

        The caller owns the monotonic progress rule.  This method only avoids
        a global nearest-point jump at a self-intersection.
        """
        point = np.asarray(position_m, dtype=float).reshape(3)
        half_window = int(half_window_segments)
        if half_window < 1:
            raise ValueError("half_window_segments must be positive")
        anchor = int(np.clip(anchor_segment, 0, self.num_segments - 1))
        first = max(0, anchor - half_window)
        last = min(self.num_segments, anchor + half_window + 1)
        starts = self.positions_m[first:last]
        deltas = self.positions_m[first + 1:last + 1] - starts
        lengths_sq = np.einsum("ij,ij->i", deltas, deltas)
        fractions = np.einsum("ij,ij->i", point - starts, deltas) / lengths_sq
        fractions = np.clip(fractions, 0.0, 1.0)
        closest = starts + fractions[:, None] * deltas
        local = int(np.argmin(np.einsum("ij,ij->i", closest - point, closest - point)))
        segment = first + local
        progress = self.arc_length_m[segment] + fractions[local] * (
            self.arc_length_m[segment + 1] - self.arc_length_m[segment]
        )
        return float(progress), int(segment)

    def _segment_fraction(self, progress_m: float) -> tuple[int, float]:
        segment = int(np.searchsorted(self.arc_length_m, progress_m, side="right") - 1)
        segment = int(np.clip(segment, 0, self.num_segments - 1))
        start = self.arc_length_m[segment]
        span = self.arc_length_m[segment + 1] - start
        return segment, float(np.clip((progress_m - start) / span, 0.0, 1.0))


class PathFollower:
    """Small host-side state holder for the non-JAX controller path."""

    def __init__(self, geometry: PathGeometry, config: PathFollowingConfig):
        self.geometry = geometry
        self.config = config
        self.state = PathFollowerState()

    def reset(self) -> None:
        self.state = PathFollowerState()

    def reset_to_position(self, ee_position_m: np.ndarray) -> PathFollowerState:
        """Seed progress from the measured end-effector position.

        Path tracking starts after the zero-to-work transition, not at process
        start.  Seeding the projection and permitted reference lead prevents
        an artificial first-cycle tangent error.
        """
        self.state = initial_path_follower_state(
            self.geometry, self.config, ee_position_m)
        return self.state

    def step(
        self,
        ee_position_m: np.ndarray,
        *,
        dt_s: float,
        feedrate_joint_limit_m_s: float = float("inf"),
        feedrate_cbf_limit_m_s: float = float("inf"),
        feedrate_rate_limit_m_s: float = float("inf"),
    ) -> PathFollowingStep:
        step = advance_path_state(
            self.geometry,
            self.config,
            self.state,
            ee_position_m,
            dt_s=dt_s,
            feedrate_joint_limit_m_s=feedrate_joint_limit_m_s,
            feedrate_cbf_limit_m_s=feedrate_cbf_limit_m_s,
            feedrate_rate_limit_m_s=feedrate_rate_limit_m_s,
        )
        self.state = step.next_state
        return step

    def reconcile_after_motion(self, ee_position_m: np.ndarray, *,
                               dt_s: float) -> PathFollowerState:
        """Store the measured post-command projection for the next cycle.

        The NumPy/OSQP compatibility path calls this after its explicit
        integration.  The JAX path applies the identical operation inside the
        compiled kernel after integrating the QP command.
        """
        self.state = reconcile_path_state_after_motion(
            self.geometry, self.config, self.state, ee_position_m, dt_s=dt_s)
        return self.state


def initial_path_follower_state(
    geometry: PathGeometry,
    config: PathFollowingConfig,
    ee_position_m: np.ndarray,
) -> PathFollowerState:
    """Create a one-way state aligned with a measured initial pose.

    ``reference_progress_m`` is at most ``reference_lead_m`` ahead of the
    measured local projection.  This initialization is outside the periodic
    control loop, so it intentionally does not apply a per-cycle advance cap.
    """
    actual = np.asarray(ee_position_m, dtype=float).reshape(3)
    projected, segment = geometry.project_local(
        actual,
        anchor_segment=0,
        half_window_segments=config.projection_half_window_segments,
    )
    reference = min(geometry.total_length_m, projected + config.reference_lead_m)
    completed = reference >= geometry.total_length_m - _EPS
    return PathFollowerState(
        reference_progress_m=float(reference),
        projected_progress_m=float(projected),
        projection_segment=int(segment),
        endpoint_hold_s=0.0,
        completed=bool(completed),
    )


def reconcile_path_state_after_motion(
    geometry: PathGeometry,
    config: PathFollowingConfig,
    state: PathFollowerState,
    ee_position_m: np.ndarray,
    *,
    dt_s: float,
) -> PathFollowerState:
    """Update the actual projection and bounded phase catch-up after one step.

    The reference used to calculate the just-finished command is unchanged.
    The *next* state, however, must catch up when the measured projection is
    farther ahead; otherwise each cycle retains a deterministic tangent lag.
    ``projected`` is still bounded by the existing per-cycle projection speed
    limit before it can update reference progress.
    """
    dt = float(dt_s)
    if not np.isfinite(dt) or dt <= 0.0:
        raise ValueError("dt_s must be finite and positive")
    actual = np.asarray(ee_position_m, dtype=float).reshape(3)
    raw_projection, _ = geometry.project_local(
        actual,
        anchor_segment=state.projection_segment,
        half_window_segments=config.projection_half_window_segments,
    )
    maximum = min(
        geometry.total_length_m,
        state.projected_progress_m + config.max_projection_speed_m_s * dt,
    )
    projected = float(np.clip(raw_projection, state.projected_progress_m, maximum))
    segment, _ = geometry._segment_fraction(projected)
    reference = max(state.reference_progress_m, projected)
    completed = bool(state.completed or reference >= geometry.total_length_m - _EPS)
    return PathFollowerState(
        reference_progress_m=reference,
        projected_progress_m=projected,
        projection_segment=int(segment),
        endpoint_hold_s=state.endpoint_hold_s,
        completed=completed,
    )


def advance_path_state(
    geometry: PathGeometry,
    config: PathFollowingConfig,
    state: PathFollowerState,
    ee_position_m: np.ndarray,
    *,
    dt_s: float,
    feedrate_joint_limit_m_s: float = float("inf"),
    feedrate_cbf_limit_m_s: float = float("inf"),
    feedrate_rate_limit_m_s: float = float("inf"),
) -> PathFollowingStep:
    """Advance the reference toward the predicted end-of-cycle projection.

    The input pose is measured at the beginning of this sample.  Because the
    selected velocity is integrated during the same sample, the reference is
    lead-limited against the predicted end projection rather than the stale
    beginning projection.  The caller then records the measured result via
    :func:`reconcile_path_state_after_motion`.
    """
    dt = float(dt_s)
    if not np.isfinite(dt) or dt <= 0.0:
        raise ValueError("dt_s must be finite and positive")
    reference = geometry.sample(state.reference_progress_m)
    actual = np.asarray(ee_position_m, dtype=float).reshape(3)
    error_des_minus_actual = reference.position_m - actual
    if reference.at_endpoint:
        lateral_error = error_des_minus_actual
    else:
        lateral_error = error_des_minus_actual - reference.tangent * float(
            np.dot(reference.tangent, error_des_minus_actual)
        )
    cross_track = float(np.linalg.norm(lateral_error))
    gamma = float(np.clip(1.0 - cross_track / config.cross_track_stop_m, 0.0, 1.0))

    projection_raw, segment_raw = geometry.project_local(
        actual,
        anchor_segment=state.projection_segment,
        half_window_segments=config.projection_half_window_segments,
    )
    max_projected = min(
        geometry.total_length_m,
        state.projected_progress_m + config.max_projection_speed_m_s * dt,
    )
    projected = float(np.clip(projection_raw, state.projected_progress_m, max_projected))
    projection_segment, _ = geometry._segment_fraction(projected)

    omega_norm = float(np.linalg.norm(reference.omega_per_m))
    if omega_norm > _EPS:
        tool_axis_cap = float(config.maximum_tool_axis_speed_rad_s) / omega_norm
    else:
        tool_axis_cap = float("inf")
    limits = {
        "joint": _finite_nonnegative(feedrate_joint_limit_m_s),
        "cbf": _finite_nonnegative(feedrate_cbf_limit_m_s),
        "rate": _finite_nonnegative(feedrate_rate_limit_m_s),
        "tool_axis": tool_axis_cap,
        # This is a virtual-reference deceleration cap, not an actuator
        # model.  It gives the P-only endpoint hold a continuously shrinking
        # tangent feed instead of an abrupt 0.03 m/s -> 0 transition.
        "endpoint_brake": endpoint_braking_feedrate_limit(
            geometry.total_length_m - reference.progress_m,
            config.endpoint_braking_deceleration_m_s2,
        ),
    }
    # A MAT trajectory can start with exactly zero feedrate. Sampling only at
    # ell=0 would make progress an absorbing state even though the next sample
    # contains planned acceleration. Probe ahead only for that zero-start
    # case. Taking max(current, future) everywhere would skip an intentional
    # local slowdown in a time-parameterized profile.
    feed_probe_progress = min(
        geometry.total_length_m,
        state.reference_progress_m + config.max_projection_speed_m_s * dt,
    )
    nominal = reference.feedrate_m_s
    if nominal <= _EPS:
        nominal = geometry.sample(feed_probe_progress).feedrate_m_s
    if reference.at_endpoint or state.completed:
        feedrate = 0.0
        reason = "endpoint"
    else:
        minimum = min(nominal, *limits.values())
        feedrate = gamma * minimum
        reason = _limiting_reason(nominal, limits, gamma)

    candidate = min(geometry.total_length_m, state.reference_progress_m + feedrate * dt)
    predicted_projection = min(
        geometry.total_length_m,
        projected + min(feedrate, config.max_projection_speed_m_s) * dt,
    )
    lead_limited = min(
        geometry.total_length_m,
        predicted_projection + config.reference_lead_m,
    )
    scheduled_progress = max(
        state.reference_progress_m, min(candidate, lead_limited))
    # The reference may not lead the measured projection by more than the
    # configured allowance, but it must not remain behind it either.  During
    # a 5-D redundant motion a small null-space/geometry discrepancy can move
    # the physical end effector farther along the path than the scheduled
    # feed.  Ignoring tangent error is intentional for path following, so an
    # uncorrected phase lag would otherwise accumulate as a false full-pose
    # position error.  ``projected`` already has the per-cycle projection
    # speed cap, making this a bounded measurement phase correction rather
    # than a new velocity command or an output filter.
    reference_progress = max(scheduled_progress, projected)
    completed = reference_progress >= geometry.total_length_m - _EPS
    endpoint_hold = state.endpoint_hold_s + dt if completed else 0.0
    next_state = PathFollowerState(
        reference_progress_m=float(reference_progress),
        projected_progress_m=projected,
        projection_segment=int(projection_segment),
        endpoint_hold_s=float(endpoint_hold),
        completed=bool(completed),
    )
    # The control cycle uses the reference predicted for the end of this
    # sample. Returning the previous sample would add one full control-period
    # of path distance to the Cartesian error at a practical 100 Hz rate.
    control_reference = geometry.sample(reference_progress)
    return PathFollowingStep(
        reference=control_reference,
        next_state=next_state,
        cross_track_error_m=cross_track,
        gamma=gamma,
        feedrate_nominal_m_s=float(nominal),
        feedrate_m_s=float(feedrate),
        feedrate_joint_limit_m_s=limits["joint"],
        feedrate_cbf_limit_m_s=limits["cbf"],
        feedrate_rate_limit_m_s=limits["rate"],
        feedrate_tool_axis_limit_m_s=limits["tool_axis"],
        feedrate_endpoint_brake_limit_m_s=limits["endpoint_brake"],
        limiting_reason=reason,
    )


def feedrate_limit_from_box(
    u_bias: np.ndarray,
    u_per_m: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
) -> float:
    """Return the largest non-negative path speed satisfying box bounds."""
    bias = np.asarray(u_bias, dtype=float).reshape(-1)
    direction = np.asarray(u_per_m, dtype=float).reshape(-1)
    lower = np.asarray(lower, dtype=float).reshape(bias.shape)
    upper = np.asarray(upper, dtype=float).reshape(bias.shape)
    if np.any(lower > upper):
        raise ValueError("lower bounds must not exceed upper bounds")
    cap = float("inf")
    for base, slope, lo, hi in zip(bias, direction, lower, upper):
        if slope > _EPS:
            cap = min(cap, float((hi - base) / slope))
        elif slope < -_EPS:
            cap = min(cap, float((lo - base) / slope))
        elif base < lo - _EPS or base > hi + _EPS:
            return 0.0
    return max(float(cap), 0.0)


def feedrate_limit_from_joint_velocity(
    u_bias: np.ndarray,
    u_per_m: np.ndarray,
    velocity_limits: np.ndarray,
) -> float:
    """Derive a feedrate bound from the actuator velocity profile."""
    limits = np.asarray(velocity_limits, dtype=float).reshape(-1)
    if np.any(~np.isfinite(limits)) or np.any(limits <= 0.0):
        raise ValueError("velocity_limits must be finite and positive")
    return feedrate_limit_from_box(u_bias, u_per_m, -limits, limits)


def feedrate_limit_from_inequalities(
    G: np.ndarray,
    h: np.ndarray,
    u_bias: np.ndarray,
    u_per_m: np.ndarray,
) -> float:
    """Derive a feedrate cap from hard inequalities ``G @ u <= h``."""
    matrix = np.asarray(G, dtype=float)
    bound = np.asarray(h, dtype=float).reshape(-1)
    bias = np.asarray(u_bias, dtype=float).reshape(-1)
    direction = np.asarray(u_per_m, dtype=float).reshape(-1)
    if matrix.ndim != 2 or matrix.shape != (bound.size, bias.size):
        raise ValueError("G/h/u dimensions are inconsistent")
    residual = bound - matrix @ bias
    slope = matrix @ direction
    if np.any((np.abs(slope) <= _EPS) & (residual < -_EPS)):
        return 0.0
    positive = slope > _EPS
    if not np.any(positive):
        return float("inf")
    return max(float(np.min(residual[positive] / slope[positive])), 0.0)


def endpoint_braking_feedrate_limit(
    remaining_distance_m: float,
    deceleration_m_s2: float,
) -> float:
    """Return a terminal path-speed cap from ``v^2 = 2 a s``.

    The result schedules the *virtual path reference* to stop at the final
    arc-length sample.  It does not claim a physical actuator braking bound;
    actuator and CBF limits remain independent hard caps in the QP path.
    """
    remaining = max(float(remaining_distance_m), 0.0)
    deceleration = float(deceleration_m_s2)
    if not np.isfinite(deceleration) or deceleration <= 0.0:
        raise ValueError("deceleration_m_s2 must be finite and positive")
    return float(np.sqrt(2.0 * deceleration * remaining))


def _point_tangents(positions: np.ndarray) -> np.ndarray:
    tangents = np.empty_like(positions)
    tangents[0] = _normalize(positions[1] - positions[0], np.array([1.0, 0.0, 0.0]))
    tangents[-1] = _normalize(positions[-1] - positions[-2], tangents[0])
    for index in range(1, len(positions) - 1):
        tangents[index] = _normalize(positions[index + 1] - positions[index - 1], tangents[index - 1])
    return tangents


def _point_omega_per_m(rotations: np.ndarray, segment_lengths: np.ndarray) -> np.ndarray:
    segment_omega = np.empty((len(segment_lengths), 3), dtype=float)
    for index, length in enumerate(segment_lengths):
        relative = rotations[index + 1] @ rotations[index].T
        segment_omega[index] = Rotation.from_matrix(relative).as_rotvec() / length
    omega = np.empty((len(segment_lengths) + 1, 3), dtype=float)
    omega[0] = segment_omega[0]
    omega[-1] = segment_omega[-1]
    if len(omega) > 2:
        omega[1:-1] = 0.5 * (segment_omega[:-1] + segment_omega[1:])
    return omega


def _canonicalize_quaternion_signs(quaternions: np.ndarray) -> np.ndarray:
    result = np.asarray(quaternions, dtype=float).copy()
    for index in range(1, len(result)):
        if float(np.dot(result[index - 1], result[index])) < 0.0:
            result[index] *= -1.0
    return result


def _slerp(q0: np.ndarray, q1: np.ndarray, fraction: float) -> np.ndarray:
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    dot = float(np.clip(np.dot(q0, q1), -1.0, 1.0))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        return _normalize((1.0 - fraction) * q0 + fraction * q1, q0)
    theta = float(np.arccos(dot))
    sin_theta = float(np.sin(theta))
    return (
        np.sin((1.0 - fraction) * theta) / sin_theta * q0
        + np.sin(fraction * theta) / sin_theta * q1
    )


def _normalize(vector: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    value = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(value))
    if norm > _EPS:
        return value / norm
    fallback = np.asarray(fallback, dtype=float)
    fallback_norm = float(np.linalg.norm(fallback))
    if fallback_norm > _EPS:
        return fallback / fallback_norm
    return np.array([1.0, 0.0, 0.0])


def _finite_nonnegative(value: float) -> float:
    value = float(value)
    if np.isnan(value) or value < 0.0:
        return 0.0
    return value


def _limiting_reason(nominal: float, limits: dict[str, float], gamma: float) -> str:
    if gamma <= _EPS:
        return "cross_track"
    candidates = {"nominal": nominal, **limits}
    return min(candidates, key=candidates.get)
