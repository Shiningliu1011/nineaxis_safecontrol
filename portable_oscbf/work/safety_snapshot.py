#!/usr/bin/env python3
"""Fixed-shape point-cloud safety snapshots shared by perception and JAX.

ROS messages and point-cloud sizes are inherently dynamic.  The control loop
must never see those dynamic objects directly: it consumes a validated,
fixed-shape distance field plus a fixed number of tracked-object slots.
"""

from __future__ import annotations

from dataclasses import dataclass
from threading import Lock
from typing import Iterable, Optional, Sequence, Tuple

import jax.numpy as jnp
import numpy as np


MAX_DYNAMIC_TRACKS = 8
OUT_OF_GRID_DISTANCE = -1.0


@dataclass(frozen=True)
class SafetyGridSpec:
    """Static world-frame grid geometry used by every JAX safety snapshot."""

    workspace_min: np.ndarray
    workspace_max: np.ndarray
    voxel_size: float

    def __post_init__(self) -> None:
        lower = np.asarray(self.workspace_min, dtype=np.float64).reshape(3)
        upper = np.asarray(self.workspace_max, dtype=np.float64).reshape(3)
        voxel = float(self.voxel_size)
        if voxel <= 0.0:
            raise ValueError("voxel_size must be positive")
        if np.any(upper <= lower):
            raise ValueError("workspace_max must be greater than workspace_min")
        object.__setattr__(self, "workspace_min", lower)
        object.__setattr__(self, "workspace_max", upper)
        object.__setattr__(self, "voxel_size", voxel)

    @property
    def shape(self) -> Tuple[int, int, int]:
        extent = self.workspace_max - self.workspace_min
        return tuple(np.ceil(extent / self.voxel_size).astype(int) + 1)


@dataclass(frozen=True)
class SafetySnapshot:
    """A single perception result safe to hand to the fixed-shape JAX loop."""

    distance_field: np.ndarray
    origin: np.ndarray
    voxel_size: float
    stamp_s: float
    valid: bool
    track_positions: np.ndarray
    track_radii: np.ndarray
    track_velocities: np.ndarray
    track_enabled: np.ndarray
    source_latency_s: float = 0.0
    source_stamp_ns: int | None = None
    track_overflow: bool = False
    frame_id: int = 0
    geometric_error_m: float = 0.0
    calibration_error_m: float = 0.0
    max_obstacle_speed_m_s: float = 0.0
    max_obstacle_accel_m_s2: float = 0.0
    braking_time_s: float = 0.0
    untracked_dynamic_point_count: int = 0
    track_ids: np.ndarray | None = None
    track_handoff_inflation_m: np.ndarray | None = None
    handoff_max_inflation_m: float = 0.0

    def __post_init__(self) -> None:
        field = np.asarray(self.distance_field, dtype=np.float32)
        if field.ndim != 3 or min(field.shape) < 2:
            raise ValueError("distance_field must be a 3-D grid with every axis >= 2")
        object.__setattr__(self, "distance_field", field)
        object.__setattr__(self, "origin", np.asarray(self.origin, dtype=np.float32).reshape(3))
        object.__setattr__(self, "voxel_size", float(self.voxel_size))
        object.__setattr__(self, "track_positions", _fixed_array(self.track_positions, (MAX_DYNAMIC_TRACKS, 3)))
        object.__setattr__(self, "track_radii", _fixed_array(self.track_radii, (MAX_DYNAMIC_TRACKS,)))
        object.__setattr__(self, "track_velocities", _fixed_array(self.track_velocities, (MAX_DYNAMIC_TRACKS, 3)))
        object.__setattr__(self, "track_enabled", _fixed_array(self.track_enabled, (MAX_DYNAMIC_TRACKS,)))
        source_stamp = self.source_stamp_ns
        if source_stamp is not None:
            if isinstance(source_stamp, bool):
                raise ValueError("source_stamp_ns must be an integer or None")
            source_stamp_int = int(source_stamp)
            if source_stamp_int < 0 or source_stamp_int != source_stamp:
                raise ValueError("source_stamp_ns must be a non-negative integer or None")
            source_stamp = source_stamp_int
        object.__setattr__(self, "source_stamp_ns", source_stamp)
        track_ids = (np.zeros(MAX_DYNAMIC_TRACKS, dtype=np.int64)
                     if self.track_ids is None else self.track_ids)
        handoff_inflation = (np.zeros(MAX_DYNAMIC_TRACKS, dtype=np.float32)
                             if self.track_handoff_inflation_m is None
                             else self.track_handoff_inflation_m)
        object.__setattr__(self, "track_ids", _fixed_integer_array(
            track_ids, (MAX_DYNAMIC_TRACKS,)))
        object.__setattr__(self, "track_handoff_inflation_m", _fixed_array(
            handoff_inflation, (MAX_DYNAMIC_TRACKS,)))
        handoff_max = float(self.handoff_max_inflation_m)
        if not np.isfinite(handoff_max) or handoff_max < 0.0:
            raise ValueError("handoff_max_inflation_m must be finite and non-negative")
        object.__setattr__(self, "handoff_max_inflation_m", handoff_max)
        untracked_count = int(self.untracked_dynamic_point_count)
        if untracked_count < 0:
            raise ValueError("untracked_dynamic_point_count must be non-negative")
        object.__setattr__(self, "untracked_dynamic_point_count", untracked_count)

    @classmethod
    def empty(cls, spec: SafetyGridSpec, *, stamp_s: float,
              valid: bool = True, far_distance: float = 10.0) -> "SafetySnapshot":
        return cls(
            distance_field=np.full(spec.shape, far_distance, dtype=np.float32),
            origin=spec.workspace_min,
            voxel_size=spec.voxel_size,
            stamp_s=float(stamp_s),
            valid=bool(valid),
            track_positions=np.zeros((MAX_DYNAMIC_TRACKS, 3), dtype=np.float32),
            track_radii=np.zeros(MAX_DYNAMIC_TRACKS, dtype=np.float32),
            track_velocities=np.zeros((MAX_DYNAMIC_TRACKS, 3), dtype=np.float32),
            track_enabled=np.zeros(MAX_DYNAMIC_TRACKS, dtype=np.float32),
        )

    def is_fresh(self, now_s: float, max_age_s: float) -> bool:
        return bool(self.valid and float(now_s) - float(self.stamp_s) <= float(max_age_s))

    def dynamic_margin(self, now_s: float) -> float:
        """Return the auditable perception margin at a control timestamp."""
        age_s = max(0.0, float(now_s) - float(self.stamp_s))
        elapsed_s = age_s + max(0.0, float(self.source_latency_s))
        elapsed_s += max(0.0, float(self.braking_time_s))
        return float(
            max(0.0, float(self.geometric_error_m))
            + max(0.0, float(self.calibration_error_m))
            + max(0.0, float(self.max_obstacle_speed_m_s)) * elapsed_s
            + 0.5 * max(0.0, float(self.max_obstacle_accel_m_s2)) * elapsed_s**2)


class SafetySnapshotStore:
    """Atomic latest-snapshot handoff from the perception thread to control."""

    def __init__(self, max_age_s: float = 0.10) -> None:
        if max_age_s <= 0.0:
            raise ValueError("max_age_s must be positive")
        self.max_age_s = float(max_age_s)
        self._lock = Lock()
        self._latest: Optional[SafetySnapshot] = None

    def publish(self, snapshot: SafetySnapshot) -> None:
        with self._lock:
            self._latest = snapshot

    def latest(self, now_s: float) -> Optional[SafetySnapshot]:
        with self._lock:
            snapshot = self._latest
        if snapshot is None or not snapshot.is_fresh(now_s, self.max_age_s):
            return None
        return snapshot


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Floor-unique voxel downsample: keep one point per occupied voxel cell."""
    if len(points) == 0:
        return np.empty((0, 3), dtype=np.float32)
    pts = np.asarray(points, dtype=np.float32)
    mn = pts.min(axis=0)
    indices = np.floor((pts - mn) / voxel_size).astype(np.int64)
    _, first = np.unique(indices, axis=0, return_index=True)
    return pts[np.sort(first)]


def preprocess_points(points_sensor: np.ndarray, sensor_to_world: np.ndarray,
                      spec: SafetyGridSpec, *,
                      robot_spheres: Iterable[Tuple[np.ndarray, float]] = (),
                      voxel_size: Optional[float] = None) -> np.ndarray:
    """Transform, crop, voxel-downsample, and remove supplied robot spheres.

    ``voxel_size`` overrides ``spec.voxel_size`` for the downsample step
    (used for per-sensor source-voxel sizing)."""
    points = np.asarray(points_sensor, dtype=np.float64).reshape(-1, 3)
    transform = np.asarray(sensor_to_world, dtype=np.float64).reshape(4, 4)
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) == 0:
        return np.empty((0, 3), dtype=np.float32)

    homogeneous = np.c_[points, np.ones(len(points))]
    world = (transform @ homogeneous.T).T[:, :3]
    inside = np.all((world >= spec.workspace_min) & (world <= spec.workspace_max), axis=1)
    world = world[inside]

    for center, radius in robot_spheres:
        center_arr = np.asarray(center, dtype=np.float64).reshape(3)
        radius_f = float(radius)
        if radius_f <= 0.0 or len(world) == 0:
            continue
        world = world[np.linalg.norm(world - center_arr, axis=1) > radius_f]

    if len(world) == 0:
        return np.empty((0, 3), dtype=np.float32)
    vs = float(voxel_size) if voxel_size is not None else spec.voxel_size
    voxel_indices = np.floor((world - spec.workspace_min) / vs).astype(np.int64)
    _, first_indices = np.unique(voxel_indices, axis=0, return_index=True)
    return world[np.sort(first_indices)].astype(np.float32, copy=False)


def build_distance_field(points_world: np.ndarray, spec: SafetyGridSpec,
                         *, far_distance: float = 10.0) -> np.ndarray:
    """Build a conservative occupied-voxel Euclidean distance field.

    This is deliberately an unsigned clearance field for the first sensor
    milestone.  Surface points are expanded by the CBF margin, while a query
    outside the known grid is handled as unsafe by ``sample_distance_field_jax``.
    """
    points = np.asarray(points_world, dtype=np.float64).reshape(-1, 3)
    occupied = np.zeros(spec.shape, dtype=bool)
    finite = points[np.all(np.isfinite(points), axis=1)]
    if len(finite):
        indices = np.floor((finite - spec.workspace_min) / spec.voxel_size).astype(np.int64)
        upper = np.asarray(spec.shape, dtype=np.int64)
        valid = np.all((indices >= 0) & (indices < upper), axis=1)
        indices = indices[valid]
        if len(indices):
            occupied[indices[:, 0], indices[:, 1], indices[:, 2]] = True
    if not np.any(occupied):
        return np.full(spec.shape, float(far_distance), dtype=np.float32)

    from scipy.ndimage import distance_transform_edt

    return (distance_transform_edt(~occupied) * spec.voxel_size).astype(np.float32)


def sample_distance_field_jax(distance_field, position, origin, voxel_size):
    """Trilinearly sample a fixed grid; leaving its coverage is unsafe."""
    field = jnp.asarray(distance_field)
    pos = jnp.asarray(position)
    origin = jnp.asarray(origin)
    voxel = jnp.asarray(voxel_size, dtype=field.dtype)
    shape = jnp.asarray(field.shape, dtype=field.dtype)
    grid = (pos - origin) / voxel
    lower = jnp.floor(grid).astype(jnp.int32)
    frac = grid - lower.astype(field.dtype)
    max_lower = jnp.asarray(field.shape, dtype=jnp.int32) - 2
    in_bounds = jnp.all(lower >= 0) & jnp.all(lower <= max_lower)
    lower = jnp.clip(lower, 0, max_lower)
    upper = lower + 1

    c000 = field[lower[0], lower[1], lower[2]]
    c100 = field[upper[0], lower[1], lower[2]]
    c010 = field[lower[0], upper[1], lower[2]]
    c110 = field[upper[0], upper[1], lower[2]]
    c001 = field[lower[0], lower[1], upper[2]]
    c101 = field[upper[0], lower[1], upper[2]]
    c011 = field[lower[0], upper[1], upper[2]]
    c111 = field[upper[0], upper[1], upper[2]]
    tx, ty, tz = frac
    c00 = c000 * (1.0 - tx) + c100 * tx
    c10 = c010 * (1.0 - tx) + c110 * tx
    c01 = c001 * (1.0 - tx) + c101 * tx
    c11 = c011 * (1.0 - tx) + c111 * tx
    value = (c00 * (1.0 - ty) + c10 * ty) * (1.0 - tz) + (c01 * (1.0 - ty) + c11 * ty) * tz
    return jnp.where(in_bounds, value, jnp.asarray(OUT_OF_GRID_DISTANCE, dtype=field.dtype))


def sample_distance_field_numpy(distance_field, position, origin, voxel_size):
    """NumPy equivalent used only by low-rate posture-plan validation."""
    field = np.asarray(distance_field, dtype=np.float32)
    pos = np.asarray(position, dtype=float).reshape(3)
    origin = np.asarray(origin, dtype=float).reshape(3)
    grid = (pos - origin) / float(voxel_size)
    lower = np.floor(grid).astype(np.int64)
    max_lower = np.asarray(field.shape, dtype=np.int64) - 2
    if np.any(lower < 0) or np.any(lower > max_lower):
        return float(OUT_OF_GRID_DISTANCE)
    fraction = grid - lower
    upper = lower + 1
    tx, ty, tz = fraction
    c000 = field[lower[0], lower[1], lower[2]]
    c100 = field[upper[0], lower[1], lower[2]]
    c010 = field[lower[0], upper[1], lower[2]]
    c110 = field[upper[0], upper[1], lower[2]]
    c001 = field[lower[0], lower[1], upper[2]]
    c101 = field[upper[0], lower[1], upper[2]]
    c011 = field[lower[0], upper[1], upper[2]]
    c111 = field[upper[0], upper[1], upper[2]]
    c00 = c000 * (1.0 - tx) + c100 * tx
    c10 = c010 * (1.0 - tx) + c110 * tx
    c01 = c001 * (1.0 - tx) + c101 * tx
    c11 = c011 * (1.0 - tx) + c111 * tx
    return float((c00 * (1.0 - ty) + c10 * ty) * (1.0 - tz)
                 + (c01 * (1.0 - ty) + c11 * ty) * tz)


def sample_distance_field_numpy_batch(distance_field, positions, origin, voxel_size):
    """Vectorized NumPy SDF sampling for the low-rate perception pipeline."""
    field = np.asarray(distance_field, dtype=np.float32)
    points = np.asarray(positions, dtype=float).reshape(-1, 3)
    origin = np.asarray(origin, dtype=float).reshape(3)
    grid = (points - origin) / float(voxel_size)
    lower = np.floor(grid).astype(np.int64)
    max_lower = np.asarray(field.shape, dtype=np.int64) - 2
    in_bounds = np.all((lower >= 0) & (lower <= max_lower), axis=1)
    clipped = np.clip(lower, 0, max_lower)
    upper = clipped + 1
    fraction = grid - lower
    tx, ty, tz = fraction[:, 0], fraction[:, 1], fraction[:, 2]
    c000 = field[clipped[:, 0], clipped[:, 1], clipped[:, 2]]
    c100 = field[upper[:, 0], clipped[:, 1], clipped[:, 2]]
    c010 = field[clipped[:, 0], upper[:, 1], clipped[:, 2]]
    c110 = field[upper[:, 0], upper[:, 1], clipped[:, 2]]
    c001 = field[clipped[:, 0], clipped[:, 1], upper[:, 2]]
    c101 = field[upper[:, 0], clipped[:, 1], upper[:, 2]]
    c011 = field[clipped[:, 0], upper[:, 1], upper[:, 2]]
    c111 = field[upper[:, 0], upper[:, 1], upper[:, 2]]
    c00 = c000 * (1.0 - tx) + c100 * tx
    c10 = c010 * (1.0 - tx) + c110 * tx
    c01 = c001 * (1.0 - tx) + c101 * tx
    c11 = c011 * (1.0 - tx) + c111 * tx
    values = ((c00 * (1.0 - ty) + c10 * ty) * (1.0 - tz)
              + (c01 * (1.0 - ty) + c11 * ty) * tz)
    return np.where(in_bounds, values, float(OUT_OF_GRID_DISTANCE))


def _fixed_array(value, shape: Sequence[int]) -> np.ndarray:
    array = np.asarray(value, dtype=np.float32)
    if array.shape != tuple(shape):
        raise ValueError(f"expected shape {tuple(shape)}, got {array.shape}")
    return array


def _fixed_integer_array(value, shape: Sequence[int]) -> np.ndarray:
    array = np.asarray(value, dtype=np.int64)
    if array.shape != tuple(shape):
        raise ValueError(f"expected shape {tuple(shape)}, got {array.shape}")
    if np.any(array < 0):
        raise ValueError("track IDs must be non-negative")
    return array
