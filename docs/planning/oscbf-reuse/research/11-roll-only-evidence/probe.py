"""JAX integration coverage for the roll-free 5-D tool-axis task."""

import numpy as np
from scipy.spatial.transform import Rotation

from work.jax_control_facade import JaxControlLoop
from work.path_following import PathFollowingConfig, PathGeometry
from work.tool_axis_task import rotation_error_rotvec, rotation_error_rotvec_jax


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
    print("PATH_DTYPE_BEFORE_INIT", loop._path_geometry.positions_m.dtype)
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

    from work.tool_axis_task import task_error_5d
    print("PRE_ERROR", task_error_5d(position, position, current_rotation, roll_only_target))
    print("SAFE_MAX", np.max(np.abs(result.u_safe)))
    print("DQ_MAX", np.max(np.abs(result.q_next-q)))
    assert result.qp_ok
    print("ERR", result.err_6d, "MAX", np.max(np.abs(result.err_6d)))
    print("U_NOM", result.u_nom, "MAX", np.max(np.abs(result.u_nom)))

test_tool_axis_path_kernel_ignores_roll_only_reference_at_path_start()
