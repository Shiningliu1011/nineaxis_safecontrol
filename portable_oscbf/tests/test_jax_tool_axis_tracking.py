"""JAX integration coverage for the roll-free 5-D tool-axis task."""

import numpy as np
from scipy.spatial.transform import Rotation

from work.jax_control_facade import JaxControlLoop
from work.path_following import PathFollowingConfig, PathGeometry
from work.tool_axis_task import rotation_error_rotvec, rotation_error_rotvec_jax


def test_rotation_error_rotvec_has_no_180_degree_blind_spot():
    identity = np.eye(3)
    flipped = np.array([
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
    ])  # 180 degrees about world X

    error = rotation_error_rotvec(identity, flipped)
    assert abs(float(np.linalg.norm(error)) - np.pi) < 1.0e-9, (
        f"exact 180-degree offset must report pi, got {np.linalg.norm(error)}"
    )
    # The legacy cross-sum formula is exactly zero at a half turn, which is
    # what made a flipped transition handoff invisible to the controller.
    legacy = -0.5 * (
        np.cross(identity[:, 0], flipped[:, 0])
        + np.cross(identity[:, 1], flipped[:, 1])
        + np.cross(identity[:, 2], flipped[:, 2])
    )
    assert float(np.linalg.norm(legacy)) < 1.0e-12

    import jax.numpy as jnp

    jax_error = rotation_error_rotvec_jax(
        jnp.asarray(identity), jnp.asarray(flipped)
    )
    # This test module runs with the default x32 JAX mode.
    assert abs(float(np.asarray(jnp.linalg.norm(jax_error))) - np.pi) < 1.0e-3


def test_rotation_error_rotvec_matches_cross_sum_for_small_angles():
    import jax.numpy as jnp

    desired = Rotation.from_rotvec(np.array([0.02, -0.01, 0.015])).as_matrix()
    identity = np.eye(3)
    exact = rotation_error_rotvec(identity, desired)
    legacy = -0.5 * (
        np.cross(identity[:, 0], desired[:, 0])
        + np.cross(identity[:, 1], desired[:, 1])
        + np.cross(identity[:, 2], desired[:, 2])
    )
    np.testing.assert_allclose(exact, legacy, atol=2.0e-4)

    exact_jax = rotation_error_rotvec_jax(
        jnp.asarray(identity), jnp.asarray(desired)
    )
    np.testing.assert_allclose(np.asarray(exact_jax), exact, atol=1.0e-3)


def test_tool_axis_mode_ignores_pure_roll_in_tracking_error_and_nominal_command():
    loop = JaxControlLoop(dt=0.01, task_mode='tool_axis_5d')
    loop.init_cbf()
    q = np.array([0.25, 0.16, -0.98, 0.53, -2.64, -0.85, -0.16, -0.97, 1.18])
    position = np.asarray(loop.robot.ee_position(q))
    current_rotation = np.asarray(loop.robot.ee_rotation(q))
    # Right multiplication rotates about the local tool X axis and preserves
    # its world direction, so this is intentionally free in the 5-D task.
    roll_only_target = current_rotation @ Rotation.from_rotvec([0.7, 0.0, 0.0]).as_matrix()

    (_q_next, _u_safe, u_nom, error_report, _ee_pos, _ee_rot,
     qp_ok, _min_dist) = loop.tracking_step(
         q=q,
         task_pos=position,
         task_vel=np.zeros(3),
         task_rot=roll_only_target,
         task_omega=np.zeros(3),
         kp_pos=50.0,
         kp_orient=10.0,
         kp_joint=0.45,
         q_des=q,
         nullspace_speed_limit=0.18,
     )

    assert qp_ok
    np.testing.assert_allclose(error_report, np.zeros(6), atol=1.0e-8)
    np.testing.assert_allclose(u_nom, np.zeros(9), atol=1.0e-8)


def test_invalid_task_mode_is_rejected_before_kernel_initialization():
    try:
        JaxControlLoop(task_mode='not_a_task')
    except ValueError as exc:
        assert 'task_mode' in str(exc)
    else:
        raise AssertionError('invalid task mode must be rejected')


def test_tool_axis_path_kernel_ignores_roll_only_reference_at_path_start():
    loop = JaxControlLoop(dt=0.01, task_mode='tool_axis_5d')
    q = np.array([0.25, 0.16, -0.98, 0.53, -2.64, -0.85, -0.16, -0.97, 1.18])
    position = np.asarray(loop.robot.ee_position(q))
    current_rotation = np.asarray(loop.robot.ee_rotation(q))
    roll_only_target = current_rotation @ Rotation.from_rotvec(
        [0.7, 0.0, 0.0]).as_matrix()
    geometry = PathGeometry.from_samples(
        np.array([position, position + np.array([0.001, 0.0, 0.0])]),
        np.array([roll_only_target, roll_only_target]),
        np.zeros(2),
        np.array([0.0, 0.1]),
    )
    loop.configure_path(geometry, PathFollowingConfig())
    loop.init_cbf()

    result = loop.path_tracking_step(
        q=q,
        path_state=loop.initial_path_state(),
        kp_pos=50.0,
        kp_orient=10.0,
        kp_joint=0.45,
        q_des=q,
        nullspace_speed_limit=0.18,
    )

    assert result.qp_ok
    np.testing.assert_allclose(result.err_6d, np.zeros(6), atol=1.0e-8)
    np.testing.assert_allclose(result.u_nom, np.zeros(9), atol=1.0e-8)
