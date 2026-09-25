"""Single source of truth for the butterfly trajectory placement.

The OSCBF controller tracks ``data/nurbs/ik_input.mat`` through the reference
runner's calibrated transform (rotate -> scale to fit the J1 prismatic stroke
-> align the centroid to ``ee_center``).  The MuJoCo viewer's displayed target
path and the transition server's first-task target use this same transform.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Sequence

import numpy as np


def default_portable_root() -> Path:
    """Installed ``portable_oscbf``, or the source tree in a dev checkout."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(
            get_package_share_directory("robot_safecontrol_moveit")
        ) / "portable_oscbf"
    except Exception:
        return Path(__file__).resolve().parents[2] / "portable_oscbf"


def bootstrap_portable(portable_root: Path) -> None:
    """Make ``work`` and the vendored ``dpax`` importable."""
    work_dir = portable_root / "work"
    vendor_dpax = portable_root / "vendor" / "dpax"
    if not (work_dir / "__init__.py").is_file():
        raise FileNotFoundError(
            f"portable_oscbf/work not found under {portable_root}"
        )
    for entry in (portable_root, work_dir, vendor_dpax):
        text = str(entry)
        if text not in sys.path:
            sys.path.insert(0, text)


def trajectory_to_base_transform(
    mat_path: Path | str,
    config_yaml_path: Path | str | None = None,
) -> np.ndarray:
    """4x4 ``T_traj_to_base`` matching the OSCBF controller exactly."""
    portable_root = default_portable_root()
    bootstrap_portable(portable_root)
    from work.ik_data_loader import (
        load_kinematics_config,
        reference_trajectory_transform,
    )

    if config_yaml_path is None:
        config_yaml_path = portable_root / "config" / "nineaxis.yaml"
    kinematics_config = load_kinematics_config(config_yaml_path)
    return reference_trajectory_transform(
        str(mat_path),
        np.asarray(
            kinematics_config["trajectory_align_rotation"], dtype=float
        ),
        np.asarray(kinematics_config["ee_center"], dtype=float),
    )


def load_calibrated_path(
    mat_path: Path | str,
    *,
    max_points: int = 0,
    point_stride: int = 1,
    cylinder_axis_direction: Sequence[float] = (0.0, 1.0, 0.0),
) -> np.ndarray:
    """Return the calibrated trajectory positions in ``base_link`` (N, 3).

    The points are radially projected onto the least-squares fitted tracking
    cylinder so that every consumer (controller, transition first-target,
    viewer display) agrees the tool works *on* the cylindrical surface rather
    than wobbling inside or outside it.
    """
    _, positions, indices = _load_calibrated_samples(
        mat_path, max_points, point_stride, cylinder_axis_direction
    )
    return positions[indices]


def load_calibrated_path_with_times(
    mat_path: Path | str,
    *,
    max_points: int = 0,
    point_stride: int = 1,
    cylinder_axis_direction: Sequence[float] = (0.0, 1.0, 0.0),
) -> tuple[np.ndarray, np.ndarray]:
    """Calibrated positions (N, 3) and raw source times (N,) in seconds.

    Positions are radially projected onto the fitted tracking cylinder, same
    as :func:`load_calibrated_path`.
    """
    data, positions, indices = _load_calibrated_samples(
        mat_path, max_points, point_stride, cylinder_axis_direction
    )
    times = np.asarray(data["time_series"], dtype=float).reshape(-1)
    return positions[indices], times[indices]


def _load_calibrated_samples(
    mat_path: Path | str,
    max_points: int,
    point_stride: int,
    cylinder_axis_direction: Sequence[float],
) -> tuple[np.void, np.ndarray, np.ndarray]:
    """Load, transform and project the full path before selecting samples.

    Keep source fields available to the timed adapter without requiring a
    time_series field for position-only consumers. Fitting after subsampling
    would change the cylinder and break controller/viewer/transition agreement.
    """
    import scipy.io as sio

    from .cylinder_geometry import snap_path_to_cylindrical_surface

    transform = trajectory_to_base_transform(mat_path)
    data = sio.loadmat(mat_path)["ik_input"][0, 0]
    raw = np.asarray(data["position_series"], dtype=float) / 1000.0
    calibrated = apply_trajectory_transform(raw, transform)
    positions, _, _ = snap_path_to_cylindrical_surface(
        calibrated, cylinder_axis_direction
    )
    indices = np.arange(0, len(positions), point_stride, dtype=int)
    if max_points > 0:
        indices = indices[:max_points]
    return data, positions, indices


def apply_trajectory_transform(
    points_m: Sequence[Sequence[float]],
    transform: np.ndarray,
) -> np.ndarray:
    """Apply ``T_traj_to_base`` to (N, 3) raw trajectory points."""
    points = np.asarray(points_m, dtype=float).reshape(-1, 3)
    homogeneous = np.hstack([points, np.ones((len(points), 1))])
    return (np.asarray(transform) @ homogeneous.T).T[:, :3]
