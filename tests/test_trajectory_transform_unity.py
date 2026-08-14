"""G1 acceptance gate: one butterfly, one transform, everywhere.

The MuJoCo viewer's displayed target path, the transition server's
first-task target, and the OSCBF controller's reference path must all come
from the same calibrated ``T_traj_to_base``.  A mismatch made the visible
curve sit ~0.22 m above the curve tool0 actually followed.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


REPO_ROOT = Path(__file__).resolve().parents[1]
_SRC = REPO_ROOT / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))
_PORTABLE = REPO_ROOT / "portable_oscbf"
for _entry in (_PORTABLE, _PORTABLE / "work", _PORTABLE / "vendor" / "dpax"):
    if str(_entry) not in sys.path:
        sys.path.insert(0, str(_entry))

MAT_PATH = REPO_ROOT / "data" / "nurbs" / "ik_input.mat"


def test_viewer_display_path_matches_controller_path():
    from robot_safecontrol_moveit.oscbf_trajectory import load_calibrated_path
    from work.ik_data_loader import load_repository_trajectory

    display = load_calibrated_path(MAT_PATH, max_points=0, point_stride=1)
    control = load_repository_trajectory(
        str(MAT_PATH)
    ).path_geometry().positions_m

    # PathGeometry drops stationary duplicate samples (14984 of 14992), so a
    # pointwise index comparison is meaningless.  The geometric gate is curve
    # coincidence: every displayed sample must lie on the controlled curve.
    assert abs(len(display) - len(control)) <= 8
    distances, _ = cKDTree(control).query(display)
    max_delta = float(np.max(distances))
    assert max_delta < 1e-3, (
        f"displayed and controlled paths differ by {max_delta * 1000:.3f} mm"
    )


def test_transition_first_target_is_controller_path_start():
    from robot_safecontrol_moveit.task_target import load_first_task_target
    from work.ik_data_loader import load_repository_trajectory

    positions, times = load_first_task_target(MAT_PATH, (0.0, 0.0, 0.0), 64, 1)
    start = load_repository_trajectory(
        str(MAT_PATH)
    ).path_geometry().positions_m[0]

    assert len(positions) == 64
    assert len(times) == 64
    assert float(np.linalg.norm(np.asarray(positions[0]) - start)) < 1e-6, (
        "transition target does not sit on the controller's path start"
    )


def test_viewer_display_samples_lie_on_controller_path():
    from robot_safecontrol_moveit.oscbf_trajectory import load_calibrated_path
    from work.ik_data_loader import load_repository_trajectory

    control = load_repository_trajectory(
        str(MAT_PATH)
    ).path_geometry().positions_m
    full = load_calibrated_path(MAT_PATH, max_points=0, point_stride=1)

    # Mirror the viewer's ``_sample_display_path`` subsampling.
    maximum_points = 512
    last = len(full) - 1
    indices = [
        round(index * last / (maximum_points - 1))
        for index in range(maximum_points)
    ]
    sampled = full[indices]
    distances, _ = cKDTree(control).query(sampled)
    assert float(np.max(distances)) < 1e-3, (
        "viewer display samples drift off the controlled path"
    )
