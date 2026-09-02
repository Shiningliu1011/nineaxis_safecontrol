"""Shared cylinder geometry: circle fit, projection helpers, surface normals.

The tracker path lies on a cylinder whose axis direction is configured (and
whose centre is fitted from the trajectory itself, because the axis is offset
from the world origin).  The viewer, the transition pipeline, and the OSCBF
controller must all agree on this fit — a mismatch once produced a 180-degree
orientation error at the handoff (see LESSONS_LEARNED.md).
"""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class CylinderFit:
    """Least-squares circle fitted in the plane normal to ``axis``.

    ``center_xy`` is the 2-D centre in the orthonormal (``u``, ``v``) basis of
    the axis plane; ``radius_squared`` may be non-positive for a degenerate
    fit, callers that need a display radius must check it.
    """

    axis: np.ndarray
    u: np.ndarray
    v: np.ndarray
    center_xy: tuple[float, float]
    radius_squared: float

    @property
    def radius(self) -> float:
        return sqrt(max(self.radius_squared, 0.0))


def fit_circle(
    points: Sequence[Sequence[float]],
    axis_direction: Sequence[float],
) -> CylinderFit:
    """Fit a circle to *points* in the plane perpendicular to *axis_direction*."""
    values = np.asarray(points, dtype=float)
    axis = np.asarray(axis_direction, dtype=float)
    axis_length = float(np.linalg.norm(axis))
    if axis_length < 1e-12:
        raise ValueError("cylinder_axis_direction must be a non-zero 3-vector")
    axis = axis / axis_length

    helper = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(helper, axis))) > 0.9:
        helper = np.array([0.0, 0.0, 1.0])
    u = np.cross(axis, helper)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    v /= np.linalg.norm(v)

    plane_x = values @ u
    plane_y = values @ v
    matrix = np.column_stack((plane_x, plane_y, np.ones(len(values))))
    rhs = -(plane_x * plane_x + plane_y * plane_y)
    coeff, *_ = np.linalg.lstsq(matrix, rhs, rcond=None)
    d_val, e_val, f_val = coeff
    center_x = -0.5 * d_val
    center_y = -0.5 * e_val
    radius_squared = center_x * center_x + center_y * center_y - f_val
    return CylinderFit(
        axis=axis,
        u=u,
        v=v,
        center_xy=(float(center_x), float(center_y)),
        radius_squared=float(radius_squared),
    )


def snap_path_to_cylindrical_surface(
    points: Sequence[Sequence[float]],
    axis_direction: Sequence[float] | np.ndarray,
) -> tuple[np.ndarray, tuple[float, float, float], float]:
    """Radially project *points* onto the least-squares fitted cylinder.

    The raw trajectory is generated independently of the cylinder that is
    fitted to it later, so its points wobble a few millimetres to almost two
    centimetres around the fitted surface.  The OSCBF controller, the
    transition first-target and the viewer display all consume the same
    calibrated path, so this module is where the snap must happen once:
    projecting radially keeps the axial (along-axis) coordinate and the
    ordering untouched while pinning the radial coordinate to the fitted
    radius, which keeps the tool on the work surface instead of penetrating
    it or floating off it.

    Returns ``(snapped_points, axis_point, radius)``.
    """
    values = np.asarray(points, dtype=float).reshape(-1, 3)
    fit = fit_circle(values, axis_direction)
    radius = fit.radius
    axial_vals = values @ fit.axis
    axial_centre = 0.5 * (float(axial_vals.min()) + float(axial_vals.max()))
    centre = (
        fit.center_xy[0] * fit.u
        + fit.center_xy[1] * fit.v
        + axial_centre * fit.axis
    )
    relative = values - centre
    axial = np.outer(relative @ fit.axis, fit.axis)
    radial = relative - axial
    radial_len = np.linalg.norm(radial, axis=1)
    radial_len = np.maximum(radial_len, 1e-12)
    snapped = centre + axial + (radius / radial_len)[:, None] * radial
    return snapped, tuple(float(c) for c in centre), float(radius)


def _rotation_matrix_to_quaternion_xyzw(
    matrix: np.ndarray,
) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to a normalised xyzw quaternion."""
    m = np.asarray(matrix, dtype=float)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s
    length = sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    return (
        float(qx / length),
        float(qy / length),
        float(qz / length),
        float(qw / length),
    )


def compute_surface_normal_orientations(
    points: list[tuple[float, float, float]],
    axis_direction: tuple[float, float, float],
    *,
    fit_points: Sequence[Sequence[float]] | None = None,
) -> list[tuple[float, float, float, float]]:
    """Return xyzw quaternion per point with X-axis = inward radial (surface normal).

    Fit the cylinder centre using *fit_points* (the full trajectory) so that
    near-stationary waypoint segments don't degenerate the circle fit.

    The X-axis points *toward* the cylinder axis (inward radial): the
    mathematical outward normal lies below the robot's reachable workspace.
    """
    values = np.asarray(points, dtype=float)
    fit_values = (
        np.asarray(fit_points, dtype=float) if fit_points is not None else values
    )
    if fit_values.ndim != 2 or fit_values.shape[1] != 3 or len(fit_values) < 3:
        raise ValueError("fit_points must be an (N, 3) array with at least 3 samples")

    fit = fit_circle(fit_values, axis_direction)

    axial_vals = fit_values @ fit.axis
    axial_centre = 0.5 * (float(axial_vals.min()) + float(axial_vals.max()))
    centre = (
        fit.center_xy[0] * fit.u
        + fit.center_xy[1] * fit.v
        + axial_centre * fit.axis
    )

    orientations: list[tuple[float, float, float, float]] = []
    for point in values:
        rel = point - centre
        axial = fit.axis * float(np.dot(rel, fit.axis))
        radial = rel - axial
        rlen = float(np.linalg.norm(radial))
        if rlen < 1e-12:
            # Point lies on the cylinder axis — fall back to identity.
            orientations.append((0.0, 0.0, 0.0, 1.0))
            continue
        col_x = -radial / rlen
        col_y = fit.axis
        col_z = np.cross(col_x, col_y)
        col_z /= np.linalg.norm(col_z)
        # Re-orthogonalise Y against X and Z
        col_y = np.cross(col_z, col_x)
        rot = np.column_stack((col_x, col_y, col_z))
        orientations.append(_rotation_matrix_to_quaternion_xyzw(rot))

    return orientations
