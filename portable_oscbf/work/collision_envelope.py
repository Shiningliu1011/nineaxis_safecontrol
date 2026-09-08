"""Fixed mesh-conservative outer sphere envelope for environment CBFs.

The velocity-level JAX controller needs a fixed number of smooth collision
queries. These spheres are a conservative outer approximation of the visual
and collision STL meshes, not a fitted average shape. Each local mesh AABB is
partitioned into cells and every cell is enclosed by one sphere with 2 mm of
numerical/model padding. The same data source is used by the NumPy point-cloud
self-filter and the JAX CBF path.

Units are metres. Centres are expressed in the named link's local URDF frame.
This module is controller-critical: change it only with the mesh-coverage
regression and an HRI headless run.
"""

from __future__ import annotations

import numpy as np


ENVIRONMENT_ENVELOPE_VERSION = "mesh_aabb_outer_spheres_v1"
MESH_ENVELOPE_PADDING_M = 0.002

# Transform indices match NineaxisManipulatorJAX._compute_all_link_transforms:
# base_link, Link1, Link2, Link3, Link4, Link5, Link6, Link7, Link8, Link9.
# Values are generated from STL local AABB partitions:
# base=(1,1,6), Link1=(1,3,2), Link2=(3,1,1), Link3=(3,1,1),
# Link4=(1,2,1), Link5=(1,1,3), Link6=(2,1,1), Link7=(2,1,1),
# Link8=(2,1,1), Link9=(3,1,1).
ENVIRONMENT_SPHERE_LINK_NAMES = np.asarray([
    "base_link", "base_link", "base_link", "base_link", "base_link", "base_link",
    "Link1", "Link1", "Link1", "Link1", "Link1", "Link1",
    "Link2", "Link2", "Link2",
    "Link3", "Link3", "Link3",
    "Link4", "Link4",
    "Link5", "Link5", "Link5",
    "Link6", "Link6",
    "Link7", "Link7",
    "Link8", "Link8",
    "Link9", "Link9", "Link9",
], dtype=object)

ENVIRONMENT_SPHERE_LINK_INDICES = np.asarray([
    0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1, 1,
    2, 2, 2,
    3, 3, 3,
    4, 4,
    5, 5, 5,
    6, 6,
    7, 7,
    8, 8,
    9, 9, 9,
], dtype=np.int32)

ENVIRONMENT_SPHERE_LOCAL_CENTERS_M = np.asarray([
    (-0.000001363, 0.006000355, -0.260250037),
    (-0.000001363, 0.006000355, -0.110750034),
    (-0.000001363, 0.006000355, 0.038749968),
    (-0.000001363, 0.006000355, 0.188249970),
    (-0.000001363, 0.006000355, 0.337749973),
    (-0.000001363, 0.006000355, 0.487249975),
    (0.000000000, 0.047083333, -0.123124991),
    (0.000000000, 0.047083333, 0.000625003),
    (0.000000000, 0.188649997, -0.123124991),
    (0.000000000, 0.188649997, 0.000625003),
    (0.000000000, 0.330216661, -0.123124991),
    (0.000000000, 0.330216661, 0.000625003),
    (0.010799993, -0.000000002, -0.000000007),
    (0.112499993, -0.000000002, -0.000000007),
    (0.214199994, -0.000000002, -0.000000007),
    (-0.004132878, 0.000000000, -0.000000007),
    (0.112499993, 0.000000000, -0.000000007),
    (0.229132865, 0.000000000, -0.000000007),
    (0.000000000, -0.101755387, -0.000000007),
    (0.000000000, -0.001266141, -0.000000007),
    (0.000000002, -0.000000004, -0.209424914),
    (0.000000002, -0.000000004, -0.101274733),
    (0.000000002, -0.000000004, 0.006875449),
    (0.006973481, -0.000000002, 0.000000004),
    (0.118877413, -0.000000002, 0.000000004),
    (0.002031040, -0.000000004, 0.000000000),
    (0.108010366, -0.000000004, 0.000000000),
    (0.006599066, -0.000035441, -0.000000011),
    (0.099866330, -0.000035441, -0.000000011),
    (0.016250209, -0.000000889, -0.013815091),
    (0.103750596, -0.000000889, -0.013815091),
    (0.191250983, -0.000000889, -0.013815091),
], dtype=np.float64)

ENVIRONMENT_SPHERE_RADII_M = np.asarray([
    0.125728788, 0.125728788, 0.125728788,
    0.125728788, 0.125728788, 0.125728788,
    0.123840245, 0.123840245, 0.123840245,
    0.123840245, 0.123840245, 0.123840245,
    0.104886939, 0.104886939, 0.104886939,
    0.119077139, 0.119077139, 0.119077139,
    0.113913009, 0.113913009,
    0.097111066, 0.097111066, 0.097111066,
    0.100379996, 0.100379996,
    0.095047860, 0.095047860,
    0.085790581, 0.085790581,
    0.063400694, 0.063400694, 0.063400694,
], dtype=np.float64)

NUM_ENVIRONMENT_COLLISION_SPHERES = int(len(ENVIRONMENT_SPHERE_RADII_M))

if not (
        ENVIRONMENT_SPHERE_LINK_NAMES.shape == (NUM_ENVIRONMENT_COLLISION_SPHERES,)
        and ENVIRONMENT_SPHERE_LINK_INDICES.shape == (NUM_ENVIRONMENT_COLLISION_SPHERES,)
        and ENVIRONMENT_SPHERE_LOCAL_CENTERS_M.shape == (NUM_ENVIRONMENT_COLLISION_SPHERES, 3)
        and np.all(ENVIRONMENT_SPHERE_RADII_M > 0.0)):
    raise RuntimeError("invalid fixed environment collision envelope")


def environment_spheres_for_link(link_name: str) -> tuple[np.ndarray, np.ndarray]:
    """Return local outer spheres for one visual mesh link."""
    mask = ENVIRONMENT_SPHERE_LINK_NAMES == str(link_name)
    return (ENVIRONMENT_SPHERE_LOCAL_CENTERS_M[mask],
            ENVIRONMENT_SPHERE_RADII_M[mask])


def environment_collision_spheres_from_transforms(transforms) -> np.ndarray:
    """Transform the fixed local outer envelope to the world/base frame.

    ``transforms`` is the NumPy FK map returned by ``NineaxisKinematics``.
    The result has shape ``(32, 4)`` and rows ``[x, y, z, radius]``.
    """
    rows = []
    for link_name, center_local, radius in zip(
            ENVIRONMENT_SPHERE_LINK_NAMES,
            ENVIRONMENT_SPHERE_LOCAL_CENTERS_M,
            ENVIRONMENT_SPHERE_RADII_M):
        transform = np.asarray(transforms[str(link_name)], dtype=float).reshape(4, 4)
        center_world = transform[:3, :3] @ center_local + transform[:3, 3]
        rows.append((center_world[0], center_world[1], center_world[2], radius))
    return np.asarray(rows, dtype=float)
