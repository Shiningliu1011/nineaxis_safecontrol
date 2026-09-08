"""Shared robot geometry data — single source of truth for collision modules.

This module centralises the structural geometry of the ninezzhou robot that was
previously duplicated across ``fcl_collision``, ``fcl_collision_mesh``,
``point_cloud_obstacles_dynamic``, and ``nineaxis_kinematics``.  Collision
backends import from here instead of declaring their own tables.

Calibration-specific data (pair exclusions, environment sphere envelopes, OBB
models) remains in its respective backend module because it is backend-specific
and often auto-generated.

Units: metres.
"""

from __future__ import annotations

from typing import List, Tuple

# ─── Link names in kinematic chain order ────────────────────────────
LINK_NAMES: List[str] = [
    "world", "base_link", "Link1", "Link2", "Link3", "Link4",
    "Link5", "Link6", "Link7", "Link8", "Link9", "ee_link",
]

N_LINKS = 10  # base_link … Link9 (excludes world and ee_link)

# ─── Body segment topology ─────────────────────────────────────────
# Each segment connects two link origins; the robot body between them must be
# considered for collision checking.  ``link_idx`` is the downstream link's
# active-joint index (used by the point-cloud and analytical distance paths).
# ``default_radius`` is a conservative capsule/segment radius (metres).
#
# ``CAPSULE_DEFS`` in fcl_collision.py derives from the same topology but uses
# per-backend calibrated radii; it is *not* replaced by this table.

BODY_SEGMENTS: List[Tuple[str, str, int, float]] = [
    ("base_link", "Link1",   1, 0.055),
    ("Link1",     "Link2",   2, 0.065),
    ("Link2",     "Link3",   3, 0.075),
    ("Link3",     "Link4",   4, 0.070),
    ("Link4",     "Link5",   5, 0.070),
    ("Link5",     "Link6",   6, 0.060),
    ("Link6",     "Link7",   7, 0.055),
    ("Link7",     "Link8",   8, 0.052),
    ("Link8",     "Link9",   9, 0.050),
    ("Link9",     "ee_link", 9, 0.050),
]

# ─── Box envelope dimensions ───────────────────────────────────────
# Conservative box approximations for base_link and Link1 (used by the FCL
# primitive collision path).  Centres are in the link's local URDF frame.

BASE_BOX_SIZE = (0.160, 0.140, 0.900)      # (sx, sy, sz)
BASE_BOX_CENTER = (0.000, 0.006, 0.113)    # local frame centre

LINK1_BOX_SIZE = (0.160, 0.430, 0.260)
LINK1_BOX_CENTER = (0.000, 0.189, -0.061)
