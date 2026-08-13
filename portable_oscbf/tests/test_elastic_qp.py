#!/usr/bin/env python3
"""M7 acceptance: elastic QP, controller-internal clipping, no hard stop."""

from __future__ import annotations

from pathlib import Path

import jax

jax.config.update("jax_enable_x64", True)

import _path_setup  # noqa: F401
import numpy as np
import pytest

from work.ik_data_loader import load_repository_trajectory
from work.jax_control_facade import JaxControlLoop
from work.path_following import PathFollowingConfig


REPO_ROOT = Path(__file__).resolve().parents[2]
CURRENT_TRAJECTORY = REPO_ROOT / "data" / "nurbs" / "ik_input.mat"
NUM_STEPS = 300


@pytest.fixture(scope="module")
def loop():
    trajectory = load_repository_trajectory(str(CURRENT_TRAJECTORY))
    control = JaxControlLoop(dt=0.002, temporal_lambda=0.2, enable_x64=True)
    control.configure_path(trajectory.path_geometry(), PathFollowingConfig())
    control.init_cbf()
    assert control._config.relax_cbf is True
    assert control._config.cbf_relaxation_penalty == pytest.approx(1e5)
    return control


def _empty_obstacles():
    return dict(
        obs_pos=np.zeros((8, 3)),
        obs_radii=np.zeros(8),
        obs_enabled=np.zeros(8),
        obs_d_safe=np.zeros(8),
        obs_vel=np.zeros((8, 3)),
        obs_radius_dot=np.zeros(8),
        obs_alpha=np.full(8, 10.0),
    )


def test_elastic_qp_no_stop_and_slack_zero_in_normal_operation(loop):
    q = np.zeros(9)
    path_state = loop.initial_path_state()
    limits = np.asarray(loop.robot.joint_max_velocities)
    max_slack = 0.0
    for _ in range(NUM_STEPS):
        result = loop.path_tracking_step(
            q=q, path_state=path_state, kp_pos=50.0, kp_orient=10.0,
            kp_joint=0.45, q_des=q, nullspace_speed_limit=0.18,
            damping=1e-3, **_empty_obstacles())
        path_state = result.path_state
        q = result.q_next
        # AC7.2: no QP failure on the obstacle-free trajectory.
        assert result.qp_ok, f"qp_ok=False at step with no obstacles"
        # AC7.1: slack ~0 in normal operation.
        max_slack = max(max_slack, float(result.delta_slack))
        # AC7.4: controller-internal clip keeps u_nom inside the box.
        assert np.all(np.asarray(result.u_nom) >= -limits - 1e-9)
        assert np.all(np.asarray(result.u_nom) <= limits + 1e-9)
        # AC7.2: positive dynamic margin (no obstacles -> sentinel 1.0).
        assert result.min_obs_dist > 0.0
    assert max_slack < 1e-6, f"max delta_slack {max_slack:.3e} >= 1e-6"


def test_elastic_qp_jit_cache_stays_one(loop):
    q = np.zeros(9)
    path_state = loop.initial_path_state()
    for _ in range(3):
        result = loop.path_tracking_step(
            q=q, path_state=path_state, kp_pos=50.0, kp_orient=10.0,
            kp_joint=0.45, q_des=q, nullspace_speed_limit=0.18,
            damping=1e-3, **_empty_obstacles())
        path_state = result.path_state
        q = result.q_next
    assert loop._path_tracking_fn._cache_size() == 1


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
