"""JAX integration coverage for the roll-free 5-D tool-axis task."""

import numpy as np
from scipy.spatial.transform import Rotation

from work.jax_control_facade import JaxControlLoop
from work.path_following import PathFollowingConfig, PathGeometry


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
