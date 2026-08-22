"""Shared trajectory target helpers: load, first-target IK, surface-normal orientation.

The transition pipeline needs trajectory loading and first-target
computation; this module is the single source of truth for those helpers.
The cylinder-fit math itself lives in :mod:`cylinder_geometry` so the
viewer and the planner share one implementation.
"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

import numpy as np
import scipy.io
from pymoveit2 import MoveIt2
from sensor_msgs.msg import JointState

from .continuous_ik import ContinuousIK, IKError, IKOptions
from .cylinder_geometry import compute_surface_normal_orientations


# ---------------------------------------------------------------------------
#  Shared trajectory loading
# ---------------------------------------------------------------------------


def load_mat_trajectory(
    path: Path,
    offset_m: Sequence[float],
    max_points: int,
    point_stride: int,
) -> tuple[list[tuple[float, float, float]], list[float]]:
    """Load the MAT position path in ``base_link`` coordinates (metres).

    The legacy MuJoCo-only Y-up→Z-up conversion is intentionally absent.  The
    URDF, MoveIt PlanningScene, and the calibration offset all use the same
    ``base_link`` coordinates.
    """
    mat_data = scipy.io.loadmat(path)
    try:
        ik_input = mat_data["ik_input"][0, 0]
        positions_mm = np.asarray(ik_input["position_series"], dtype=float)
        times_s = np.asarray(ik_input["time_series"], dtype=float).reshape(-1)
    except (KeyError, IndexError, TypeError) as error:
        raise ValueError(f"{path} is not a supported ik_input.mat file") from error

    if positions_mm.ndim != 2 or positions_mm.shape[1] != 3:
        raise ValueError("position_series must have shape (N, 3)")
    if positions_mm.shape[0] != times_s.shape[0]:
        raise ValueError("position_series and time_series have different lengths")
    if not np.isfinite(positions_mm).all() or not np.isfinite(times_s).all():
        raise ValueError("MAT trajectory contains non-finite values")

    indices = np.arange(0, len(positions_mm), point_stride, dtype=int)
    if max_points > 0:
        indices = indices[:max_points]
    if len(indices) == 0:
        raise ValueError("Trajectory selection produced no waypoints")

    offset = np.asarray(offset_m, dtype=float)
    positions_m = positions_mm[indices] / 1000.0 + offset
    selected_times = times_s[indices]
    return (
        [tuple(float(v) for v in point) for point in positions_m],
        [float(v) for v in selected_times],
    )


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

    ``offset_m`` is retained for call-site compatibility but is ignored: the
    first target must sit exactly on the OSCBF controller's calibrated path.

    Returns (positions, times).
    """
    from .oscbf_trajectory import load_calibrated_path_with_times

    positions, times = load_calibrated_path_with_times(
        trajectory_mat,
        max_points=max_points,
        point_stride=point_stride,
    )
    return (
        [tuple(float(value) for value in point) for point in positions],
        [float(value) for value in times],
    )


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
            from .oscbf_trajectory import load_calibrated_path

            full_positions = load_calibrated_path(
                trajectory_mat, max_points=0, point_stride=1
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
