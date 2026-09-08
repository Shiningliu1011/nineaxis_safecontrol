"""Integration coverage for q_posture_ref(ell) in the JAX null space only."""

import numpy as np
import pytest

from work.jax_control_facade import JaxControlLoop
from work.path_following import PathFollowingConfig, PathGeometry
from work.tool_axis_task import task_jacobian_5d


def _safe_q() -> np.ndarray:
    return np.array([
        0.25, 0.16, -0.98, 0.53, -2.64,
        -0.85, -0.16, -0.97, 1.18,
    ])


def _stationary_geometry(loop: JaxControlLoop, q: np.ndarray) -> PathGeometry:
    position = np.asarray(loop.robot.ee_position(q))
    rotation = np.asarray(loop.robot.ee_rotation(q))
    return PathGeometry.from_samples(
        np.stack((position, position + np.array([0.001, 0.0, 0.0]))),
        np.repeat(rotation[None, :, :], 2, axis=0),
        np.zeros(2),
        np.array([0.0, 0.1]),
    )


def _nullspace_target(loop: JaxControlLoop, q: np.ndarray) -> np.ndarray:
    rotation = np.asarray(loop.robot.ee_rotation(q))
    jacobian = task_jacobian_5d(
        np.asarray(loop.robot.ee_jacobian(q)), rotation, rotation)
    null_direction = np.linalg.svd(jacobian, full_matrices=True)[2][-1]
    return q + 0.05 * null_direction / np.max(np.abs(null_direction))


def test_static_path_posture_reference_changes_only_the_nullspace_nominal_term():
    pytest.importorskip('cbfpy')
    q = _safe_q()
    loop = JaxControlLoop(dt=0.01, task_mode='tool_axis_5d')
    target = _nullspace_target(loop, q)
    geometry = _stationary_geometry(loop, q)
    loop.configure_path(
        geometry,
        PathFollowingConfig(),
        posture_reference=np.repeat(target[None, :], geometry.num_points, axis=0),
    )
    loop.init_cbf()

    result = loop.path_tracking_step(
        q=q,
        path_state=loop.initial_path_state(),
        kp_pos=50.0,
        kp_orient=10.0,
        kp_joint=0.45,
        # A different runtime q_des proves that the configured static path
        # reference, not the legacy value, selects the null-space target.
        q_des=q,
        nullspace_speed_limit=0.18,
    )

    assert loop.path_posture_reference_enabled is True
    assert result.qp_ok
    np.testing.assert_allclose(result.posture_reference, target, atol=1.0e-9)
    # The task reference is initially exact and feedrate is zero.  Any nominal
    # motion is therefore from the static null-space posture target.
    assert np.linalg.norm(result.u_nom) > 1.0e-5
    assert np.linalg.norm(result.err_6d[:3]) < 1.0e-5
    assert np.linalg.norm(result.err_6d[3:5]) < 1.0e-5

    second = loop.path_tracking_step(
        q=q,
        path_state=loop.initial_path_state(),
        kp_pos=50.0,
        kp_orient=10.0,
        kp_joint=0.45,
        q_des=q,
        nullspace_speed_limit=0.18,
    )
    assert second.qp_ok
    assert loop._path_tracking_fn._cache_size() == 1


def test_path_posture_reference_is_rejected_before_jit_when_a_waypoint_exceeds_limits():
    q = _safe_q()
    loop = JaxControlLoop(dt=0.01, task_mode='tool_axis_5d')
    geometry = _stationary_geometry(loop, q)
    invalid = np.repeat(q[None, :], geometry.num_points, axis=0)
    invalid[0, 0] = float(np.asarray(loop.q_max)[0]) + 1.0e-3

    with pytest.raises(ValueError, match='hard-joint-limit'):
        loop.configure_path(
            geometry, PathFollowingConfig(), posture_reference=invalid)
