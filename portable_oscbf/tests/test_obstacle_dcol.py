#!/usr/bin/env python3
"""M7 acceptance: DCOL obstacle sphere constraints in the hot path."""

from __future__ import annotations

from pathlib import Path

import jax

jax.config.update("jax_enable_x64", True)

import _path_setup  # noqa: F401
import numpy as np
import pytest

from work.ik_data_loader import load_repository_trajectory
from work.jax_control_facade import JaxControlLoop
from work.obb_collision_model import (
    OBB_HALF_EXTENTS_M,
    OBB_LOCAL_CENTERS_M,
)
from work.path_following import PathFollowingConfig


REPO_ROOT = Path(__file__).resolve().parents[2]
CURRENT_TRAJECTORY = REPO_ROOT / "data" / "nurbs" / "ik_input.mat"

# Base-link OBB (link index 0) in the zero configuration: centre z + half z.
BASE_OBB_TOP_Z = float(OBB_LOCAL_CENTERS_M[0][2] + OBB_HALF_EXTENTS_M[0][2])
OBS_RADIUS = 0.05
D_SAFE = 0.03


@pytest.fixture(scope="module")
def loop():
    trajectory = load_repository_trajectory(str(CURRENT_TRAJECTORY))
    control = JaxControlLoop(dt=0.002, temporal_lambda=0.2, enable_x64=True)
    control.configure_path(trajectory.path_geometry(), PathFollowingConfig())
    control.init_cbf()
    return control


def _step(loop, q, path_state, obstacle_z):
    obs = dict(
        obs_pos=np.zeros((8, 3)),
        obs_radii=np.zeros(8),
        obs_enabled=np.zeros(8),
        obs_d_safe=np.zeros(8),
        obs_vel=np.zeros((8, 3)),
        obs_radius_dot=np.zeros(8),
        obs_alpha=np.full(8, 10.0),
    )
    obs["obs_pos"][0] = [0.0, 0.006, obstacle_z]
    obs["obs_radii"][0] = OBS_RADIUS
    obs["obs_enabled"][0] = 1.0
    obs["obs_d_safe"][0] = D_SAFE
    result = loop.path_tracking_step(
        q=q, path_state=path_state, kp_pos=50.0, kp_orient=10.0,
        kp_joint=0.45, q_des=q, nullspace_speed_limit=0.18,
        damping=1e-3, **obs)
    return result


def test_approaching_obstacle_keeps_positive_margin_then_uses_slack(loop):
    q = np.zeros(9)
    path_state = loop.initial_path_state()

    # Far away: margin large, slack zero.
    far = _step(loop, q, path_state, BASE_OBB_TOP_Z + OBS_RADIUS + D_SAFE + 0.10)
    assert far.qp_ok
    assert far.min_obs_dist > 0.10
    assert far.delta_slack < 1e-6

    # Approaching: margin still positive, slack stays small.
    near = _step(loop, q, path_state, BASE_OBB_TOP_Z + OBS_RADIUS + D_SAFE + 0.005)
    assert near.qp_ok
    assert near.min_obs_dist > 0.0
    assert near.delta_slack < 1e-3

    # Penetrating: slack engages (elastic QP) instead of a hard stop.
    penetrating = _step(
        loop, q, path_state, BASE_OBB_TOP_Z + OBS_RADIUS - 0.005)
    assert penetrating.qp_ok
    assert penetrating.delta_slack > 1e-3
    assert np.all(np.isfinite(penetrating.u_safe))


def test_obstacle_updates_do_not_recompile(loop):
    q = np.zeros(9)
    path_state = loop.initial_path_state()
    for offset in (0.10, 0.08, 0.06, 0.04):
        result = _step(
            loop, q, path_state,
            BASE_OBB_TOP_Z + OBS_RADIUS + D_SAFE + offset)
        assert result.qp_ok
    assert loop._path_tracking_fn._cache_size() == 1


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
