"""Adapter exposing a fixed arc-length path through the runner trajectory API.

The legacy runner was built around MAT-backed ``IKTrajectoryData``.  A
validated Stage-2 process profile already owns explicit world-frame geometry,
so this adapter supplies the small compatible read-only surface needed by
RViz, path following, and diagnostics without pretending that the profile is
another MAT trajectory or regenerating it at runtime.
"""

from __future__ import annotations

import numpy as np

from work.ik_data_loader import TaskReference
from work.path_following import PathGeometry


class FixedPathTrajectory:
    """Read-only time facade for one immutable :class:`PathGeometry`.

    ``source_time_s`` is retained only for existing visualisation and legacy
    diagnostics.  The control path itself uses arc length through
    ``path_geometry()``.  All positions use metres and all angular velocities
    use radians per second.
    """

    def __init__(self, geometry: PathGeometry) -> None:
        source_time = np.asarray(geometry.source_time_s, dtype=float)
        if source_time.shape != (geometry.num_points,):
            raise ValueError('path source_time_s shape does not match geometry')
        if np.any(~np.isfinite(source_time)) or np.any(np.diff(source_time) <= 0.0):
            raise ValueError('process path source_time_s must be strictly increasing')
        self._path_geometry = geometry
        self._raw_time = source_time.copy()
        self._pos_world = np.asarray(geometry.positions_m, dtype=float).copy()
        self._R_des_series = np.asarray(geometry.rotations, dtype=float).copy()
        self._feedrate = np.asarray(geometry.feedrate_m_s, dtype=float).copy()
        self._omega_series = (np.asarray(geometry.omega_per_m, dtype=float)
                              * self._feedrate[:, None])
        self._vel_world = np.asarray(geometry.tangents, dtype=float) * self._feedrate[:, None]
        self._acc_world = self._finite_difference(self._vel_world, self._raw_time)
        self._jerk_world = self._finite_difference(self._acc_world, self._raw_time)
        self._acc_norm = np.linalg.norm(self._acc_world, axis=1)
        self._jerk_norm = np.linalg.norm(self._jerk_world, axis=1)
        tangents = np.asarray(geometry.tangents, dtype=float)
        self._tangent_acc_cmd = np.einsum('ij,ij->i', self._acc_world, tangents)
        self._tangent_acc_projection_world = (
            self._tangent_acc_cmd[:, None] * tangents)
        self._tangent_jerk_cmd = np.einsum('ij,ij->i', self._jerk_world, tangents)
        self._tangent_jerk_projection_world = (
            self._tangent_jerk_cmd[:, None] * tangents)
        self._chord_err = np.zeros(geometry.num_points, dtype=float)
        self.num_points = geometry.num_points
        self.Ts = float(np.median(np.diff(self._raw_time)))
        self.orientation_mode = 'profile_fixed'

    @staticmethod
    def _finite_difference(values: np.ndarray, times: np.ndarray) -> np.ndarray:
        if values.shape[0] < 3:
            return np.zeros_like(values)
        return np.gradient(values, times, axis=0, edge_order=1)

    def _progress_at_time(self, time_s: float) -> float:
        clamped = float(np.clip(time_s, self._raw_time[0], self._raw_time[-1]))
        return float(np.interp(
            clamped, self._raw_time, self._path_geometry.arc_length_m))

    def _reference_at_time(self, time_s: float):
        return self._path_geometry.sample(self._progress_at_time(time_s))

    def _time_to_idx(self, time_s: float) -> int:
        clamped = float(np.clip(time_s, self._raw_time[0], self._raw_time[-1]))
        index = int(np.searchsorted(self._raw_time, clamped, side='right') - 1)
        return int(np.clip(index, 0, self.num_points - 1))

    def set_orientation_mode(self, mode: str = 'fixed', fixed_R=None):
        """Reject runtime frame generation; the validated profile owns its frame."""
        if str(mode).strip().lower() != 'fixed':
            raise ValueError(
                'a fixed Stage-2 process profile cannot use a generated orientation mode')
        if fixed_R is not None:
            candidate = np.asarray(fixed_R, dtype=float).reshape(3, 3)
            if not np.allclose(candidate[:, 0], self._R_des_series[0, :, 0], atol=1.0e-6):
                raise ValueError(
                    'fixed_R tool axis differs from the validated process profile')
        return self

    def path_geometry(self) -> PathGeometry:
        """Return the exact immutable geometry captured by the JAX path kernel."""
        return self._path_geometry

    def total_time(self) -> float:
        return float(self._raw_time[-1])

    def pos_world_at(self, time_s: float) -> np.ndarray:
        return np.asarray(self._reference_at_time(time_s).position_m, dtype=float).copy()

    def vel_world_at(self, time_s: float) -> np.ndarray:
        reference = self._reference_at_time(time_s)
        return np.asarray(reference.tangent, dtype=float) * reference.feedrate_m_s

    def orientation_at(self, time_s: float) -> np.ndarray:
        return np.asarray(self._reference_at_time(time_s).rotation, dtype=float).copy()

    def angular_velocity_at(self, time_s: float) -> np.ndarray:
        reference = self._reference_at_time(time_s)
        return np.asarray(reference.omega_per_m, dtype=float) * reference.feedrate_m_s

    def feedrate_cmd_at(self, time_s: float) -> float:
        return float(self._reference_at_time(time_s).feedrate_m_s)

    def chord_error_at(self, time_s: float) -> float:
        del time_s
        return 0.0

    def jerk_norm_at(self, time_s: float) -> float:
        return float(np.interp(
            float(np.clip(time_s, self._raw_time[0], self._raw_time[-1])),
            self._raw_time, self._jerk_norm))

    def motion_scalars_at(self, time_s: float) -> dict[str, float]:
        """Expose finite schedule metrics required by the shared runner layer.

        The Stage-2 runner admission gate fixes ``tracking_schedule=fixed``;
        these values therefore do not scale its gains.  They remain complete
        and finite so the common diagnostics and future explicit schedule
        rejection behave consistently with MAT-backed trajectories.
        """
        clamped = float(np.clip(time_s, self._raw_time[0], self._raw_time[-1]))

        def scalar(series: np.ndarray) -> float:
            return float(np.interp(clamped, self._raw_time, series))

        index = self._time_to_idx(clamped)
        next_index = min(index + 1, self.num_points - 1)
        previous_boundary = float(self._raw_time[index])
        next_boundary = float(self._raw_time[next_index])
        return {
            'speed': scalar(np.linalg.norm(self._vel_world, axis=1)),
            'feedrate': scalar(self._feedrate),
            'acc_norm': scalar(self._acc_norm),
            'jerk_norm': scalar(self._jerk_norm),
            'tangent_acc_cmd': scalar(self._tangent_acc_cmd),
            'tangent_acc_projection_norm': scalar(
                np.linalg.norm(self._tangent_acc_projection_world, axis=1)),
            'tangent_jerk_cmd': scalar(self._tangent_jerk_cmd),
            'tangent_jerk_projection_norm': scalar(
                np.linalg.norm(self._tangent_jerk_projection_world, axis=1)),
            'chord_err': 0.0,
            'u': clamped,
            'point_index': float(index),
            'block_index': 0.0,
            'prev_boundary_s': max(clamped - previous_boundary, 0.0),
            'next_boundary_s': max(next_boundary - clamped, 0.0),
            'boundary_distance_s': min(
                max(clamped - previous_boundary, 0.0),
                max(next_boundary - clamped, 0.0)),
        }

    def task_reference_at(self, time_s: float) -> TaskReference:
        reference = self._reference_at_time(time_s)
        feedrate = float(reference.feedrate_m_s)
        return TaskReference(
            t=float(reference.source_time_s),
            pos=np.asarray(reference.position_m, dtype=float).copy(),
            vel=np.asarray(reference.tangent, dtype=float) * feedrate,
            accel=np.zeros(3, dtype=float),
            R_des=np.asarray(reference.rotation, dtype=float).copy(),
            omega=np.asarray(reference.omega_per_m, dtype=float) * feedrate,
        )

    def task_reference_at_continuous(self, time_s: float) -> TaskReference:
        """The profile geometry already performs continuous arc-length sampling."""
        return self.task_reference_at(time_s)
