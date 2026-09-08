"""Immutable joint-space posture references indexed by path arc length.

This module owns only host-side validation and interpolation.  It deliberately
does not know about ROS, FK, CBF, QP, or hardware.  A reference is useful
only after every waypoint has been validated by the Stage-2 path checks; this
class cannot turn an arbitrary joint sequence into a safe motion by itself.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np


if TYPE_CHECKING:
    from work.path_following import PathGeometry


_EPS = 1.0e-9
_NUM_JOINTS = 9


@dataclass(frozen=True)
class PathPostureReference:
    """A fixed-size 9-joint reference sampled on the task path's arc length.

    ``q_waypoints`` has the same number of rows as the already compacted task
    path.  J1 is in metres and J2--J9 are in radians, so a Euclidean norm of
    the vector is intentionally not treated as a physical distance metric.

    When ``dq_des_dell`` is provided, it stores the derivative of the desired
    joint configuration with respect to arc length.  This enables the path
    kernel to include a feedforward null-space velocity term:
    ``qdot_des = dq_des_dell * ell_dot``.
    """

    arc_length_m: np.ndarray
    q_waypoints: np.ndarray
    dq_des_dell: np.ndarray | None = None

    def __post_init__(self) -> None:
        arc_length = np.asarray(self.arc_length_m, dtype=float).reshape(-1)
        q_values = np.asarray(self.q_waypoints, dtype=float)
        if arc_length.ndim != 1 or arc_length.size < 2:
            raise ValueError('arc_length_m must contain at least two samples')
        if np.any(~np.isfinite(arc_length)) or arc_length[0] < -_EPS:
            raise ValueError('arc_length_m must be finite and start at zero')
        if np.any(np.diff(arc_length) <= 0.0):
            raise ValueError('arc_length_m must be strictly increasing')
        if q_values.shape != (arc_length.size, _NUM_JOINTS):
            raise ValueError(
                'q_waypoints must have shape '
                f'({arc_length.size}, {_NUM_JOINTS}), got {q_values.shape}')
        if not np.all(np.isfinite(q_values)):
            raise ValueError('q_waypoints must contain only finite values')

        # Validate dq_des_dell if provided
        dq_values = None
        if self.dq_des_dell is not None:
            dq_values = np.asarray(self.dq_des_dell, dtype=float)
            if dq_values.shape != (arc_length.size, _NUM_JOINTS):
                raise ValueError(
                    'dq_des_dell must have shape '
                    f'({arc_length.size}, {_NUM_JOINTS}), got {dq_values.shape}')
            if not np.all(np.isfinite(dq_values)):
                raise ValueError('dq_des_dell must contain only finite values')

        arc_copy = arc_length.copy()
        q_copy = q_values.copy()
        arc_copy.setflags(write=False)
        q_copy.setflags(write=False)
        object.__setattr__(self, 'arc_length_m', arc_copy)
        object.__setattr__(self, 'q_waypoints', q_copy)
        if dq_values is not None:
            dq_values.setflags(write=False)
            object.__setattr__(self, 'dq_des_dell', dq_values)

    @classmethod
    def from_path_geometry(cls, geometry: 'PathGeometry', q_waypoints: np.ndarray,
                           *, q_min: np.ndarray | None = None,
                           q_max: np.ndarray | None = None,
                           dq_des_dell: np.ndarray | None = None) -> 'PathPostureReference':
        """Bind posture samples to one compacted task geometry.

        Bounds are optional because offline profile generation may happen
        before a runtime kinematics object exists.  The JAX facade supplies
        the runtime bounds, so control callers cannot accidentally capture an
        out-of-limit static reference.

        When ``dq_des_dell`` is provided, it stores the derivative of the
        desired joint configuration with respect to arc length for use as
        a feedforward null-space velocity term.
        """
        reference = cls(
            arc_length_m=np.asarray(geometry.arc_length_m, dtype=float),
            q_waypoints=q_waypoints,
            dq_des_dell=dq_des_dell)
        if (q_min is None) != (q_max is None):
            raise ValueError('q_min and q_max must be supplied together')
        if q_min is not None:
            lower = np.asarray(q_min, dtype=float).reshape(_NUM_JOINTS)
            upper = np.asarray(q_max, dtype=float).reshape(_NUM_JOINTS)
            if np.any(~np.isfinite(lower)) or np.any(~np.isfinite(upper)):
                raise ValueError('joint bounds must be finite')
            if np.any(lower >= upper):
                raise ValueError('joint lower bounds must be below upper bounds')
            _tol = 1.0e-8
            if (np.any(reference.q_waypoints < lower - _tol)
                    or np.any(reference.q_waypoints > upper + _tol)):
                raise ValueError('q_waypoints contains a hard-joint-limit violation')
            # Clamp to bounds to prevent sub-nanometre floating-point leakage.
            reference = cls(
                arc_length_m=reference.arc_length_m,
                q_waypoints=np.clip(reference.q_waypoints, lower, upper),
                dq_des_dell=reference.dq_des_dell)
        return reference

    @property
    def num_points(self) -> int:
        return int(self.q_waypoints.shape[0])

    @property
    def total_length_m(self) -> float:
        return float(self.arc_length_m[-1])

    def sample(self, progress_m: float) -> np.ndarray | tuple[np.ndarray, np.ndarray]:
        """Linearly interpolate the joint target at clamped path progress.

        Returns:
            If ``dq_des_dell`` is None: ``(q_des,)`` as a single ndarray.
            If ``dq_des_dell`` is provided: ``(q_des, dq_des_dell)`` tuple.
        """
        progress = float(np.clip(progress_m, 0.0, self.total_length_m))
        segment = int(np.searchsorted(self.arc_length_m, progress, side='right') - 1)
        segment = int(np.clip(segment, 0, self.num_points - 2))
        start = self.arc_length_m[segment]
        span = self.arc_length_m[segment + 1] - start
        fraction = float(np.clip((progress - start) / max(span, _EPS), 0.0, 1.0))
        q_des = ((1.0 - fraction) * self.q_waypoints[segment]
                 + fraction * self.q_waypoints[segment + 1])
        if self.dq_des_dell is not None:
            dq_des = ((1.0 - fraction) * self.dq_des_dell[segment]
                      + fraction * self.dq_des_dell[segment + 1])
            return q_des, dq_des
        return q_des
