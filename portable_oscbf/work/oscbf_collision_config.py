#!/usr/bin/env python3
"""OSCBF collision constraint shared configuration (single source of truth for
velocity and torque modes).

Extracted from ``oscbf_velocity_config.py`` so ``oscbf_torque_config.py`` can
reuse the same collision geometry without duplicating the 17-sphere model or
the 14 self-collision pairs.

Reference
---------
Morton & Pavone, "Safe, Task-Consistent Manipulation with OSCBF" (IROS 2025)
OSCBF authors' ``franka_collision_model.py`` / ``cluttered_tabletop.py``
"""

import jax.numpy as jnp

# ---------------------------------------------------------------------------
# 14 self-collision pairs (adjacent links are skipped).
# Indices reference ``NineaxisManipulatorJAX.link_collision_data``'s 17-sphere
# ordering: 0=base, 1=link1, 2=Link2, 3=Link3, 4=Link4, 5=Link5,
# 6=Link7, 7=Link8, 8=Link9, 9=ee, 10-16=midpoints.
# ---------------------------------------------------------------------------
SELF_COLLISION_PAIRS = jnp.array([
    [0, 4], [0, 5], [0, 6], [0, 7], [0, 8],   # base vs rear links
    [1, 5], [1, 6], [1, 7], [1, 8],            # Link1 vs rear links
    [2, 6], [2, 7], [2, 8],                    # Link2 vs rear links
    [3, 7], [3, 8],                            # Link3 vs rear links
], dtype=jnp.int32)  # 14 pairs total


def compute_self_collision_h(positions, radii, pairs, d_safe):
    """Per-pair self-collision CBF: h = dist - r_a - r_b - d_safe.

    Parameters
    ----------
    positions : (N, 3)  Link-sphere world positions.
    radii     : (N,)    Link-sphere radii.
    pairs     : (K, 2)  Index pairs to check.
    d_safe    : float   Safety margin added to the sum of radii.

    Returns
    -------
    h : (K,)  CBF values (positive = safe).
    """
    pos_a = positions[pairs[:, 0]]
    pos_b = positions[pairs[:, 1]]
    rad_a = radii[pairs[:, 0]]
    rad_b = radii[pairs[:, 1]]
    distances = jnp.linalg.norm(pos_a - pos_b, axis=-1)
    return distances - rad_a - rad_b - d_safe


def compute_obstacle_h(positions, radii, obs_pos, obs_rad, d_safe):
    """Per-link × per-obstacle CBF: h = dist - r_robot - r_obs - d_safe.

    Parameters
    ----------
    positions : (N, 3)    Link-sphere world positions.
    radii     : (N,)      Link-sphere radii.
    obs_pos   : (M, 3)    Obstacle centres (may be empty (0,) when M=0).
    obs_rad   : (M,)      Obstacle radii.
    d_safe    : float     Safety margin.

    Returns
    -------
    h : (N*M,)  CBF values, ravel'd so each link vs each obstacle is a
        separate constraint.  Returns empty (0,) when M = 0.
    """
    if obs_pos.size == 0:
        return jnp.array([])
    # (N, M, 3)
    center_deltas = positions[:, None, :] - obs_pos[None, :, :]
    # (N, M)
    distances = jnp.linalg.norm(center_deltas, axis=-1)
    radii_sums = radii[:, None] + obs_rad[None, :]
    # (N*M,)
    return (distances - radii_sums - d_safe).ravel()
