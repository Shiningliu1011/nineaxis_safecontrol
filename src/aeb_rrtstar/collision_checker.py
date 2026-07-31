"""Collision checking and motion validation for the standalone OMPL benchmark.

Uses a *simplified* collision model that is consistent across all planners
in the benchmark.  The MoveIt + FCL pipeline provides the authoritative
collision geometry; this module approximates it for fair algorithm comparison.

Key checks:
1. Joint limits (exact – from URDF)
2. End-effector vs obstacle clearance (approximate – FK + geometric primitives)
3. Link swept-volume approximation (sparse FK chain points vs obstacles)
"""

from __future__ import annotations

from math import sqrt
from typing import Sequence

import numpy as np

import ompl.base as ob

from .robot_model import (
    DIMENSION,
    JOINT_LIMITS,
    OBSTACLES,
    BoxObstacle,
    CylinderObstacle,
    SphereObstacle,
    approximate_link_positions,
)


# ---------------------------------------------------------------------------
#  Low-level geometric queries
# ---------------------------------------------------------------------------


def _closest_point_box(
    point: np.ndarray,
    centre: np.ndarray,
    half_extents: np.ndarray,
) -> float:
    """Squared distance from *point* to an axis-aligned box."""
    delta = np.abs(point - centre) - half_extents
    outside = np.maximum(delta, 0.0)
    return float(np.dot(outside, outside))


def _closest_point_sphere(point: np.ndarray, centre: np.ndarray, radius: float) -> float:
    d = float(np.linalg.norm(point - centre))
    return max(0.0, d - radius)


def _closest_point_cylinder(
    point: np.ndarray,
    centre: np.ndarray,
    radius: float,
    half_height: float,
) -> float:
    """Distance from *point* to a Z-aligned cylinder (the obstacle YAML convention)."""
    rel = point - centre
    radial_dist = sqrt(rel[0] ** 2 + rel[1] ** 2)
    axial_dist = abs(rel[2])
    d_radial = max(0.0, radial_dist - radius)
    d_axial = max(0.0, axial_dist - half_height)
    if radial_dist <= radius:
        return d_axial
    if axial_dist <= half_height:
        return d_radial
    return sqrt(d_radial ** 2 + d_axial ** 2)


def _min_obstacle_clearance(point: np.ndarray) -> float:
    """Return the signed clearance (negative = penetration) to all obstacles."""
    best = float("inf")
    for obs in OBSTACLES:
        if isinstance(obs, BoxObstacle):
            d = _closest_point_box(
                point, np.asarray(obs.centre), np.asarray(obs.half_extents)
            )
        elif isinstance(obs, SphereObstacle):
            d = _closest_point_sphere(point, np.asarray(obs.centre), obs.radius)
        elif isinstance(obs, CylinderObstacle):
            d = _closest_point_cylinder(
                point, np.asarray(obs.centre), obs.radius, obs.half_height
            )
        else:
            continue
        if d < best:
            best = d
    return best


# Minimum clearance required at each sampled body point (metres).
# Relaxed clearance — the FK approximation uses sparse joint-origin points
# that cannot fully represent mesh collision geometry.  The authoritative
# collision check is MoveIt 2 + FCL.  This value keeps the standalone
# benchmark usable without being too conservative.
_LINK_CLEARANCE_M = 0.001  # 1 mm


def is_configuration_valid(joint_positions: Sequence[float]) -> bool:
    """Check joint limits and approximate obstacle collision.

    Returns True when:
    - Every joint is within its URDF-specified limits.
    - All FK chain points (joint origins + tool0) have positive
      clearance to every known obstacle.
    """
    if len(joint_positions) != DIMENSION:
        return False

    # --- joint limits -------------------------------------------------------
    for i, (low, high) in enumerate(JOINT_LIMITS):
        value = float(joint_positions[i])
        if value < low - 1e-10 or value > high + 1e-10:
            return False

    # --- obstacle clearance (best-effort approximate FK) --------------------
    try:
        points = approximate_link_positions(joint_positions)
    except Exception:
        return False

    for pt in points:
        if _min_obstacle_clearance(pt) < _LINK_CLEARANCE_M:
            return False

    return True


def is_motion_valid(
    from_joints: Sequence[float],
    to_joints: Sequence[float],
    steps: int = 16,
) -> bool:
    """Discretised motion check between two joint configurations.

    At least *steps* intermediate interpolations are tested.  We use linear
    interpolation in joint space (which MoveIt's default motion validator
    also uses for the continuous collision check).
    """
    if not is_configuration_valid(to_joints):
        return False

    n = max(steps, 2)
    for i in range(1, n):
        t = i / n
        interp = tuple(
            float(f) + t * (float(tgt) - float(f))
            for f, tgt in zip(from_joints, to_joints)
        )
        if not is_configuration_valid(interp):
            return False

    return True


# ======================================================================
#  OMPL StateValidityChecker and MotionValidator wrappers
# ======================================================================


def _state_to_tuple(state: ob.State, space: ob.RealVectorStateSpace) -> tuple[float, ...]:
    return tuple(float(state[i]) for i in range(DIMENSION))


class RobotStateValidityChecker(ob.StateValidityChecker):
    """OMPL validity checker using the simplified robot collision model.

    The SpaceInformation is stored explicitly because the Python bindings
    do not expose the C++ ``si_`` member.
    """

    def __init__(self, si: ob.SpaceInformation) -> None:
        super().__init__(si)
        self._si = si

    def isValid(self, state: ob.State) -> bool:
        joints = _state_to_tuple(state, self._si.getStateSpace())  # type: ignore[arg-type]
        return is_configuration_valid(joints)


class RobotMotionValidator(ob.MotionValidator):
    """OMPL motion validator using the simplified robot collision model.

    The SpaceInformation is stored explicitly because the Python bindings
    do not expose the C++ ``si_`` member.
    """

    def __init__(self, si: ob.SpaceInformation) -> None:
        super().__init__(si)
        self._si = si

    def checkMotion(  # type: ignore[override]
        self,
        s1: ob.State,
        s2: ob.State,
        lastValid: ob.State | None = None,
        steps: int | None = None,
    ) -> bool:
        """Check continuous validity between two states.

        OMPL may call with *lastValid* to report the last valid state on
        partial failure.  This implementation returns a boolean; partial
        validity reporting is not yet exposed.
        """
        _ = lastValid
        steps_int = steps if steps is not None and steps > 0 else 16
        joints1 = _state_to_tuple(s1, self._si.getStateSpace())  # type: ignore[arg-type]
        joints2 = _state_to_tuple(s2, self._si.getStateSpace())  # type: ignore[arg-type]
        return is_motion_valid(joints1, joints2, steps=steps_int)
