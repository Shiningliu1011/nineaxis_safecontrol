"""Single source of truth for the butterfly trajectory placement.

The OSCBF controller tracks ``data/nurbs/ik_input.mat`` through the reference
runner's calibrated transform (rotate -> scale to fit the J1 prismatic stroke
-> align the centroid to ``ee_center``).  Every other consumer of the same
``.mat`` — the MuJoCo viewer's displayed target path and the transition
server's first-task target — MUST use this same transform.  Mixing it with
the legacy ``[0, 0.343, 1.587]`` translation made the displayed butterfly sit
~0.22 m above the curve the controller actually tracked, so tool0 never
touched the visible path.
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
    import yaml

    portable_root = default_portable_root()
    bootstrap_portable(portable_root)
    from work.ik_data_loader import reference_trajectory_transform

    if config_yaml_path is None:
        config_yaml_path = portable_root / "config" / "nineaxis.yaml"
    with open(config_yaml_path, encoding="utf-8") as stream:
        kinematics_config = yaml.safe_load(stream)["kinematics"]
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
    import scipy.io as sio

    from .cylinder_geometry import snap_path_to_cylindrical_surface

    transform = trajectory_to_base_transform(mat_path)
    data = sio.loadmat(mat_path)["ik_input"][0, 0]
    raw = np.asarray(data["position_series"], dtype=float) / 1000.0
    homogeneous = np.hstack([raw, np.ones((len(raw), 1))])
    # 先用全集拟合圆柱并吸附, 再按 stride/max_points 抽样。若先抽样后吸附,
    # 小样本(如仅轨迹开头的 64 点)会拟合出一个错误的圆, 与控制器/查看器
    # 用全路径拟合的结果不一致。
    calibrated = (transform @ homogeneous.T).T[:, :3]
    snapped, _, _ = snap_path_to_cylindrical_surface(
        calibrated, cylinder_axis_direction
    )
    indices = np.arange(0, len(snapped), point_stride, dtype=int)
    if max_points > 0:
        indices = indices[:max_points]
    return snapped[indices]


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
    import scipy.io as sio

    from .cylinder_geometry import snap_path_to_cylindrical_surface

    transform = trajectory_to_base_transform(mat_path)
    data = sio.loadmat(mat_path)["ik_input"][0, 0]
    raw = np.asarray(data["position_series"], dtype=float) / 1000.0
    times = np.asarray(data["time_series"], dtype=float).reshape(-1)
    homogeneous = np.hstack([raw, np.ones((len(raw), 1))])
    calibrated = (transform @ homogeneous.T).T[:, :3]
    # 全量吸附后再抽样 (见 load_calibrated_path 的说明)。
    positions, _, _ = snap_path_to_cylindrical_surface(
        calibrated, cylinder_axis_direction
    )
    indices = np.arange(0, len(positions), point_stride, dtype=int)
    if max_points > 0:
        indices = indices[:max_points]
    return positions[indices], times[indices]


def apply_trajectory_transform(
    points_m: Sequence[Sequence[float]],
    transform: np.ndarray,
) -> np.ndarray:
    """Apply ``T_traj_to_base`` to (N, 3) raw trajectory points."""
    points = np.asarray(points_m, dtype=float).reshape(-1, 3)
    homogeneous = np.hstack([points, np.ones((len(points), 1))])
    return (np.asarray(transform) @ homogeneous.T).T[:, :3]
