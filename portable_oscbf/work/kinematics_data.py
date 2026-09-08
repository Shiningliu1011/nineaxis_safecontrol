"""Shared kinematics data constants for the nine-axis robot arm.

This module is the single source of truth for the kinematic chain definition,
joint count, and link names used across ``nineaxis_kinematics`` (NumPy),
``nineaxis_manipulator_jax`` (JAX), and test suites.

``LINK_NAMES`` is re-exported from ``robot_geometry`` to avoid duplication.
"""

from work.robot_geometry import LINK_NAMES

# ================================================================
# URDF joint chain parameters (used to extract screw axes and zero-pose config)
# ================================================================
JOINT_CHAIN = [
    # (parent, child, jtype, x, y, z, roll, pitch, yaw, axis)
    # Matches URDF: assets/ninezzhouURDF/urdf/ninezzhou.urdf
    ("world",     "base_link", "fixed",      0.0,   0.0,   0.0,   0.0,     0.0,      0.0,      (0, 0, 1)),
    ("base_link", "Link1",     "prismatic",  0.0,   0.0,   0.0,   0.0,     0.0,      0.0,      (0, 0, 1)),
    ("Link1",     "Link2",     "revolute",   0.0,   0.343, 0.0,   1.5708, -1.5708,   0.0,      (0, 0, 1)),
    ("Link2",     "Link3",     "revolute",   0.225, 0.0,   0.0,   0.0,     0.0,      0.0,      (0, 0, 1)),
    ("Link3",     "Link4",     "revolute",   0.225, 0.0,   0.0,   0.0,     0.0,      1.5708,   (0, 0, 1)),
    ("Link4",     "Link5",     "revolute",   0.0,  -0.343, 0.0,  -1.5708,  0.0,     -3.1416,   (0, 0, 1)),
    ("Link5",     "Link6",     "revolute",   0.0,   0.0,   0.0,   1.5708, -1.5708,   0.0,      (0, 0, 1)),
    ("Link6",     "Link7",     "revolute",   0.135, 0.0,   0.0,  -1.5708,  0.0,      0.0,      (0, 0, 1)),
    ("Link7",     "Link8",     "revolute",   0.11,  0.0,   0.0,   1.5708,  0.0,      0.0,      (0, 0, 1)),
    ("Link8",     "Link9",     "revolute",   0.114, 0.0,   0.0,  -1.5708,  0.0,      0.0,      (0, 0, 1)),
    # ee_link: translate 0.235m along X from Link9
    # Zero-pose EE position: [0, 343, 1387] mm
    ("Link9",     "ee_link",   "fixed",      0.235, 0.0,   0.0,   0.0,     0.0,      0.0,      (0, 0, 1)),
]

N_JOINTS = 9  # 1 prismatic + 8 revolute
