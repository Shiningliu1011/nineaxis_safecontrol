"""Joint limits and configuration for the ninezzhou 9-DOF robot arm.

Joint-space planning uses RealVectorStateSpace(9).  All values are in the
coordinate convention of the URDF (Y-up), consistent with MoveIt's
PlanningScene and the obstacle YAML.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

# ---------------------------------------------------------------------------
# Joint limits from ninezzhou.urdf
# ---------------------------------------------------------------------------

JOINT_LIMITS: tuple[tuple[float, float], ...] = (
    (0.0, 0.585),          # J1  – prismatic  (m)
    (-1.5708, 1.5708),     # J2  – revolute   (rad)
    (-1.5708, 1.5708),     # J3  – revolute   (rad)
    (-1.5708, 1.5708),     # J4  – revolute   (rad)
    (-3.1416, 3.1416),     # J5  – revolute   (rad) — full circle
    (-1.48353, 1.48353),   # J6  – revolute   (rad)
    (-1.48353, 1.48353),   # J7  – revolute   (rad)
    (-1.48353, 1.48353),   # J8  – revolute   (rad)
    (-1.48353, 1.48353),   # J9  – revolute   (rad)
)

JOINT_NAMES: tuple[str, ...] = (
    "J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9",
)

JOINT_TYPES: tuple[str, ...] = (
    "prismatic",          # J1
    "revolute",           # J2
    "revolute",           # J3
    "revolute",           # J4
    "revolute",           # J5
    "revolute",           # J6
    "revolute",           # J7
    "revolute",           # J8
    "revolute",           # J9
)

# J5 wraps; other revolute joints do not.
WRAPPING_JOINT_INDICES: tuple[int, ...] = (4,)  # 0-based

DIMENSION = 9

# ---------------------------------------------------------------------------
# Workspace obstacles – from config/obstacles.yaml, in base_link (Y-up) coords
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class BoxObstacle:
    centre: tuple[float, float, float]
    half_extents: tuple[float, float, float]  # half-sizes


@dataclass(frozen=True)
class SphereObstacle:
    centre: tuple[float, float, float]
    radius: float


@dataclass(frozen=True)
class CylinderObstacle:
    centre: tuple[float, float, float]
    radius: float
    half_height: float  # half-height along cylinder axis


Obstacle = BoxObstacle | SphereObstacle | CylinderObstacle

# Converted from obstacles.yaml (full dims → half-size where applicable).
# cylinder YAML: dimensions = [height, radius]
OBSTACLES: tuple[Obstacle, ...] = (
    BoxObstacle(
        centre=(0.25, 0.243, 0.4),
        half_extents=(0.04, 0.04, 0.08),  # dims [0.08, 0.08, 0.16]
    ),
    SphereObstacle(
        centre=(-0.25, 0.343, 0.6),
        radius=0.05,
    ),
    CylinderObstacle(
        centre=(0.22, 0.30, 0.9),
        radius=0.03,
        half_height=0.08,  # dim [0.16 height, 0.03 radius]
    ),
    BoxObstacle(
        centre=(-0.1, 0.15, 0.9),
        half_extents=(0.05, 0.05, 0.05),  # dims [0.10, 0.10, 0.10]
    ),
)

# ---------------------------------------------------------------------------
# Forward kinematics helpers — simplified for collision approximation
# ---------------------------------------------------------------------------
# DH parameters extracted from the URDF (Y-up, Z-axis revolute joints).
# These are approximate; exact collision geometry is handled by MoveIt/FCL.
# This simplified FK is used ONLY for obstacle end-effector clearance checks
# in the standalone benchmark, not in the ROS pipeline.
#
# URDF joint origin translations (parent → child, in parent frame):
#   J1:   (0, 0, 0)            prismatic along Z
#   J2:   (0, 0.343, 0)        revolute about Z
#   J3:   (0.225, 0, 0)        revolute about Z
#   J4:   (0.225, 0, 0)        revolute about Z
#   J5:   (0, -0.343, 0)       revolute about Z
#   J6:   (0, 0, 0)            revolute about Z
#   J7:   (0.135, 0, 0)        revolute about Z
#   J8:   (0.11, 0, 0)         revolute about Z
#   J9:   (0.114, 0, 0)        revolute about Z
#   tool0: (0.235, 0, 0)       fixed

import numpy as np


def _rot_z(theta: float) -> np.ndarray:
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1],
    ])


def _translate_z(d: float) -> np.ndarray:
    return np.array([0.0, 0.0, d])


# Link offsets in parent frame (Y-up convention following URDF)
_LINK_OFFSETS: list[np.ndarray] = [
    np.array([0.0, 0.343, 0.0]),    # J2 offset (from J1 end)
    np.array([0.225, 0.0, 0.0]),     # J3 offset
    np.array([0.225, 0.0, 0.0]),     # J4 offset
    np.array([0.0, -0.343, 0.0]),    # J5 offset
    np.array([0.0, 0.0, 0.0]),       # J6 offset
    np.array([0.135, 0.0, 0.0]),     # J7 offset
    np.array([0.11, 0.0, 0.0]),      # J8 offset
    np.array([0.114, 0.0, 0.0]),     # J9 offset
    np.array([0.235, 0.0, 0.0]),     # tool0 offset
]


def forward_kinematics(
    joint_positions: Sequence[float],
) -> list[np.ndarray]:
    """Return Cartesian positions of each link frame in base_link coordinates.

    Returns a list of 11 3-D positions: [J1_origin, J2_origin, ..., J9_origin, tool0].
    All positions are in the URDF Y-up frame.
    """
    positions = []
    # J1: prismatic along Z
    p = np.array([0.0, 0.0, float(joint_positions[0])])
    positions.append(p.copy())

    R = np.eye(3)
    for i in range(1, DIMENSION):
        # Apply the link offset from previous joint (in current frame, then
        # rotate about Z by the joint angle)
        p = p + R @ _LINK_OFFSETS[i - 1]
        theta = float(joint_positions[i])
        R = R @ _rot_z(theta)
        positions.append(p.copy())

    # tool0: final fixed offset
    p = p + R @ _LINK_OFFSETS[-1]
    positions.append(p.copy())

    return positions


def end_effector_position(joint_positions: Sequence[float]) -> np.ndarray:
    """Return the tool0 Cartesian position in base_link coordinates."""
    return forward_kinematics(joint_positions)[-1]


def approximate_link_positions(
    joint_positions: Sequence[float],
) -> list[np.ndarray]:
    """Return a sparse set of Cartesian points along the arm for obstacle checks.

    Returns positions at J1..J9 joint origins + tool0 (11 points total).
    """
    return forward_kinematics(joint_positions)
