"""Shared trajectory target helpers: load, first-target IK, surface-normal orientation.

Both the command-line ``plan_transition`` node and the persistent
``transition_planning_server`` need the same trajectory-loading and
first-target computation logic.  This module is the single source of truth.
"""

from __future__ import annotations

from math import isfinite, sqrt
from pathlib import Path
from typing import Sequence

import numpy as np
import scipy.io
from pymoveit2 import MoveIt2
from sensor_msgs.msg import JointState

from .continuous_ik import ContinuousIK, IKError, IKOptions


# ---------------------------------------------------------------------------
#  Shared trajectory loading
# ---------------------------------------------------------------------------


def load_mat_trajectory(
    path: Path,
    offset_m: Sequence[float],
    max_points: int,
    point_stride: int,
) -> tuple[list[tuple[float, float, float]], list[float]]:
    """Load the MAT position path in ``base_link`` coordinates (metres)."""
    mat_data = scipy.io.loadmat(path)
    ik_input = mat_data["ik_input"][0, 0]
    positions_mm = np.asarray(ik_input["position_series"], dtype=float)
    times_s = np.asarray(ik_input["time_series"], dtype=float).reshape(-1)
    if positions_mm.ndim != 2 or positions_mm.shape[1] != 3:
        raise ValueError("position_series must have shape (N, 3)")
    indices = np.arange(0, len(positions_mm), point_stride, dtype=int)
    if max_points > 0:
        indices = indices[:max_points]
    offset = np.asarray(offset_m, dtype=float)
    positions_m = positions_mm[indices] / 1000.0 + offset
    selected_times = times_s[indices]
    return (
        [tuple(float(v) for v in point) for point in positions_m],
        [float(v) for v in selected_times],
    )


# ---------------------------------------------------------------------------
#  Surface-normal orientation computation
# ---------------------------------------------------------------------------


def _rotation_matrix_to_quaternion_xyzw(
    matrix: np.ndarray,
) -> tuple[float, float, float, float]:
    """Convert a 3×3 rotation matrix to a normalised xyzw quaternion."""
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
    return (float(qx / length), float(qy / length), float(qz / length), float(qw / length))


def compute_surface_normal_orientations(
    points: list[tuple[float, float, float]],
    axis_direction: tuple[float, float, float],
    *,
    fit_points: Sequence[Sequence[float]] | None = None,
) -> list[tuple[float, float, float, float]]:
    """Return xyzw quaternion per point with X-axis = inward radial (surface normal).

    Fit the cylinder centre using *fit_points* (the full trajectory) so that
    near-stationary waypoint segments don't degenerate the circle fit.
    """
    values = np.asarray(points, dtype=float)
    axis = np.asarray(axis_direction, dtype=float)
    axis_len = float(np.linalg.norm(axis))
    if axis_len < 1e-12:
        raise ValueError("cylinder_axis_direction must be a non-zero 3-vector")
    axis /= axis_len

    fit_values = (
        np.asarray(fit_points, dtype=float)
        if fit_points is not None
        else values
    )
    if fit_values.ndim != 2 or fit_values.shape[1] != 3 or len(fit_values) < 3:
        raise ValueError("fit_points must be an (N, 3) array with at least 3 samples")

    helper = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(helper, axis))) > 0.9:
        helper = np.array([0.0, 0.0, 1.0])
    u = np.cross(axis, helper)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    v /= np.linalg.norm(v)

    plane_x = fit_values @ u
    plane_y = fit_values @ v
    A = np.column_stack((plane_x, plane_y, np.ones(len(fit_values))))
    rhs = -(plane_x * plane_x + plane_y * plane_y)
    coeff, *_ = np.linalg.lstsq(A, rhs, rcond=None)
    d_val, e_val, _f_val = coeff
    cx = -0.5 * d_val
    cy = -0.5 * e_val

    axial_vals = fit_values @ axis
    axial_centre = 0.5 * (float(axial_vals.min()) + float(axial_vals.max()))
    centre = cx * u + cy * v + axial_centre * axis

    orientations: list[tuple[float, float, float, float]] = []
    for point in values:
        rel = point - centre
        axial = axis * float(np.dot(rel, axis))
        radial = rel - axial
        rlen = float(np.linalg.norm(radial))
        if rlen < 1e-12:
            orientations.append((0.0, 0.0, 0.0, 1.0))
            continue
        col_x = -radial / rlen
        col_y = axis
        col_z = np.cross(col_x, col_y)
        col_z /= np.linalg.norm(col_z)
        col_y = np.cross(col_z, col_x)
        rot = np.column_stack((col_x, col_y, col_z))
        orientations.append(_rotation_matrix_to_quaternion_xyzw(rot))

    return orientations


# ---------------------------------------------------------------------------
#  First-target helpers
# ---------------------------------------------------------------------------


def load_first_task_target(
    trajectory_mat: Path,
    offset_m: tuple[float, float, float],
    max_points: int,
    point_stride: int,
) -> tuple[list[tuple[float, float, float]], list[float]]:
    """Load trajectory positions and times used for first-target IK.

    Returns (positions, times).
    """
    return load_mat_trajectory(trajectory_mat, offset_m, max_points, point_stride)


def compute_first_task_orientation(
    positions: list[tuple[float, float, float]],
    *,
    align_tool_x_to_surface_normal: bool,
    cylinder_axis_direction: tuple[float, float, float],
    orientation_xyzw: tuple[float, float, float, float],
    trajectory_mat: Path | None = None,
    offset_m: tuple[float, float, float] = (0.0, 0.343, 1.587),
) -> tuple[tuple[float, float, float, float], list[tuple[float, float, float, float]] | None]:
    """Compute first-target orientation (and optionally all per-point orientations).

    Returns (first_orientation, per_point_orientations_or_None).
    """
    per_point: list[tuple[float, float, float, float]] | None = None
    if align_tool_x_to_surface_normal:
        if trajectory_mat is not None and trajectory_mat.is_file():
            full_positions, _ = load_mat_trajectory(
                trajectory_mat, offset_m, 0, 1
            )
        else:
            full_positions = positions
        per_point = compute_surface_normal_orientations(
            positions,
            cylinder_axis_direction,
            fit_points=full_positions,
        )
        return (per_point[0], per_point)
    return (orientation_xyzw, None)


def solve_first_task_state(
    moveit: MoveIt2,
    joint_names: tuple[str, ...],
    tool_link: str,
    positions: list[tuple[float, float, float]],
    start_state: JointState,
    first_orientation: tuple[float, float, float, float],
    per_point_orientations: list[tuple[float, float, float, float]] | None,
    max_joint_delta: float,
    ik_service_timeout_s: float,
    base_frame: str = "base_link",
    planning_group: str = "arm",
    logger: object = None,
) -> JointState:
    """Solve IK for the trajectory and return the first waypoint's joint state.

    Raises IKError on failure.
    """
    if logger is not None:
        logger.info(
            f"IK_REQUEST "
            f"position=({positions[0][0]:.4f},{positions[0][1]:.4f},"
            f"{positions[0][2]:.4f}) "
            f"orientation_xyzw=({first_orientation[0]:.6f},"
            f"{first_orientation[1]:.6f},{first_orientation[2]:.6f},"
            f"{first_orientation[3]:.6f}) "
            f"base_frame={base_frame} "
            f"tool_link={tool_link} "
            f"planning_group={planning_group} "
            f"seed_names={list(joint_names)} "
            f"seed_positions=[{', '.join(f'{v:.3f}' for v in start_state.position)}] "
            f"avoid_collisions=true "
            f"timeout={ik_service_timeout_s:.3f}s "
            f"align_tool_x_to_surface_normal="
            f"{'true' if per_point_orientations else 'false'}"
        )

    ik = ContinuousIK(
        moveit,
        joint_names,
        IKOptions(
            tool_link=tool_link,
            orientation_xyzw=first_orientation,
            planning_group=planning_group,
            base_frame=base_frame,
            max_joint_delta=max_joint_delta,
            service_timeout_s=ik_service_timeout_s,
        ),
    )
    ik_path = ik.solve(
        positions,
        start_state,
        orientations=per_point_orientations,
    )
    return ik_path.first_state
