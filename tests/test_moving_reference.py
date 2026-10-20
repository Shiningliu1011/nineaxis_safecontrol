from pathlib import Path

import jax
import numpy as np
import pytest
from scipy.spatial.transform import Rotation, Slerp

from robot_safecontrol_moveit.oscbf_trajectory import bootstrap_portable

bootstrap_portable(Path(__file__).resolve().parents[1] / "portable_oscbf")

from work.jax_path_following import as_jax_path_geometry
from work.moving_reference_task import moving_reference_terms_jax, true_axis_angle
from work.nineaxis_kinematics import NineaxisKinematics
from work.path_following import PathGeometry


@pytest.mark.parametrize("angle", [0.0, 1e-10, 0.8, np.pi - 1e-8, np.pi])
def test_true_angle_covers_entire_direction_sphere(angle):
    current = Rotation.from_rotvec([0, 0, angle]).as_matrix()[:, 0]
    assert true_axis_angle(current, np.array([1., 0, 0])) == pytest.approx(angle, abs=1e-12)


def _geometry(kind):
    if kind == "line":
        positions = np.array([[0, 0, 0], [0.2, 0, 0], [0.4, 0, 0]])
    else:
        theta = np.array([0., 0.3, 0.6])
        positions = np.column_stack([0.3 * np.cos(theta), 0.3 * np.sin(theta), theta * 0])
    rotations = Rotation.from_rotvec([[0.1, 0.2, 0.], [0.2, 0.3, 0.2], [0.3, 0.1, 0.4]])
    return PathGeometry.from_samples(positions, rotations.as_matrix(), np.ones(3), np.arange(3.))


def _independent_error(kin, q, geometry, s):
    position, rotation = kin.ee_pose(q)
    desired = Slerp(geometry.arc_length_m, Rotation.from_matrix(geometry.rotations))([s]).as_matrix()[0]
    reference = np.array([np.interp(s, geometry.arc_length_m, geometry.positions_m[:, i]) for i in range(3)])
    cross = np.cross(desired[:, 0], rotation[:, 0])
    sine = np.linalg.norm(cross)
    angle = np.arctan2(sine, desired[:, 0] @ rotation[:, 0])
    orientation = desired[:, 1:].T @ (cross * angle / sine)
    return np.r_[position - reference, orientation]


@pytest.mark.parametrize("kind", ["line", "arc"])
@pytest.mark.parametrize("at_join", [False, True])
def test_moving_reference_derivatives_match_independent_fk(kind, at_join):
    jax.config.update("jax_enable_x64", True)
    geometry = _geometry(kind)
    s = geometry.arc_length_m[1] if at_join else 0.43 * geometry.arc_length_m[1]
    q = np.array([.22, .14, -.95, .49, -2.61, -.82, -.14, -.93, 1.13])
    kin = NineaxisKinematics()
    position, rotation = kin.ee_pose(q)
    terms = moving_reference_terms_jax(
        position, rotation, kin.compute_full_jacobian(q), as_jax_path_geometry(geometry), s,
    )
    assert bool(terms.valid)
    epsilon = 1e-7
    finite_q = np.column_stack([
        (_independent_error(kin, q + step, geometry, s) -
         _independent_error(kin, q - step, geometry, s)) / (2 * epsilon)
        for step in np.eye(9) * epsilon
    ])
    if at_join:
        finite_s = (_independent_error(kin, q, geometry, s + epsilon) -
                    _independent_error(kin, q, geometry, s)) / epsilon
    else:
        finite_s = (_independent_error(kin, q, geometry, s + epsilon) -
                    _independent_error(kin, q, geometry, s - epsilon)) / (2 * epsilon)
    np.testing.assert_allclose(terms.error, _independent_error(kin, q, geometry, s), atol=1e-10)
    np.testing.assert_allclose(terms.jacobian, finite_q, atol=2e-6)
    np.testing.assert_allclose(-terms.reference_rate, finite_s, atol=2e-5)


def test_roll_is_free_and_antipodal_direction_cannot_authorize_motion():
    jax.config.update("jax_enable_x64", True)
    geometry = _geometry("arc")
    s = 0.43 * geometry.arc_length_m[1]
    sample = geometry.sample(s)
    jacobian = np.eye(6, 9)
    geometry_jax = as_jax_path_geometry(geometry)
    roll = sample.rotation @ Rotation.from_rotvec([1.4, 0, 0]).as_matrix()
    terms = moving_reference_terms_jax(sample.position_m, roll, jacobian, geometry_jax, s)
    np.testing.assert_allclose(terms.error, np.zeros(5), atol=1e-12)
    np.testing.assert_allclose(terms.jacobian[3:] @ np.r_[np.zeros(3), sample.rotation[:, 0], np.zeros(3)], 0, atol=1e-12)
    assert bool(terms.valid)
    opposite = sample.rotation @ Rotation.from_rotvec([0, np.pi, 0]).as_matrix()
    invalid = moving_reference_terms_jax(sample.position_m, opposite, jacobian, geometry_jax, s)
    assert float(invalid.angle_rad) == pytest.approx(np.pi)
    assert not bool(invalid.valid)


def test_control_record_reference_and_feed_use_same_progress():
    from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig
    from work.jax_control_facade import JaxControlLoop
    from work.path_following import PathFollowingConfig

    root = Path(__file__).resolve().parents[1]
    config = ExperimentConfig.load(root / "config/static_experiment_s0.yaml", resource_root=root)
    geometry, _ = config.load_path()
    loop = JaxControlLoop(
        dt=.01, dt_path=.02, task_mode="tool_axis_5d", shared_path_reference=True,
    )
    loop.configure_path(geometry, PathFollowingConfig(reference_lead_m=1e-5))
    loop.init_cbf()
    result = loop.path_tracking_step(
        q=config.initial_position, path_state=np.zeros(5),
        kp_pos=240., kp_orient=40., kp_joint=.45, q_des=config.initial_position,
        nullspace_speed_limit=.18, damping=.05,
    )
    assert result.qp_ok
    reference = geometry.sample(result.path_state[0])
    np.testing.assert_allclose(result.reference_position_m, reference.position_m, atol=1e-12)
    np.testing.assert_allclose(result.reference_rotation, reference.rotation, atol=1e-12)
    assert result.feedrate_m_s * .02 == pytest.approx(result.path_state[0], abs=1e-12)


def test_invalid_shared_task_preserves_reference_and_joint_state():
    from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig
    from work.jax_control_facade import JaxControlLoop
    from work.path_following import PathFollowingConfig

    root = Path(__file__).resolve().parents[1]
    config = ExperimentConfig.load(root / "config/static_experiment_s0.yaml", resource_root=root)
    position, rotation = NineaxisKinematics().ee_pose(config.initial_position)
    opposite = rotation @ Rotation.from_rotvec([0., np.pi, 0.]).as_matrix()
    geometry = PathGeometry.from_samples(
        position + np.array([[0, 0, 0], [.01, 0, 0], [.02, 0, 0]]),
        np.repeat(opposite[None], 3, axis=0), np.full(3, .01), np.arange(3.),
    )
    loop = JaxControlLoop(dt=.01, dt_path=.02, task_mode="tool_axis_5d", shared_path_reference=True)
    loop.configure_path(geometry, PathFollowingConfig(reference_lead_m=.002))
    loop.init_cbf()
    result = loop.path_tracking_step(
        q=config.initial_position, path_state=np.zeros(5),
        kp_pos=240., kp_orient=40., kp_joint=.45, q_des=config.initial_position,
        nullspace_speed_limit=.18, damping=.05,
    )
    assert not result.qp_ok
    np.testing.assert_array_equal(result.q_next, config.initial_position)
    np.testing.assert_array_equal(result.u_safe, np.zeros(9))
    assert result.path_state[0] == 0.
    assert result.feedrate_m_s == 0.
    np.testing.assert_allclose(result.reference_position_m, position, atol=1e-12)
