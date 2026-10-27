import numpy as np
import pytest

from work.jax_control_facade import JaxControlLoop
from work.nineaxis_kinematics import NineaxisKinematics
from work.path_following import PathFollowingConfig, PathGeometry
from work.tool_axis_task import TASK_MODE_POSE_6D, TASK_MODE_TOOL_AXIS_5D


@pytest.fixture(scope="module", params=[TASK_MODE_POSE_6D, TASK_MODE_TOOL_AXIS_5D])
def stationary_reference(request):
    kinematics = NineaxisKinematics()
    q_reference = np.array([
        0.2303562, 0.1112539, 1.0167209, -0.6810303, -1.8294025,
        -0.4664294, 0.4743473, -1.0429228, 0.0289233,
    ])
    # 路径全部由关节范围内的 J1 平移生成，末端姿态保持一致。
    waypoints = np.repeat(q_reference[None, :], 3, axis=0)
    waypoints[:, 0] += np.array([-0.01, 0.0, 0.01])
    limits = kinematics.joint_limits
    assert np.all(waypoints >= limits.q_min)
    assert np.all(waypoints <= limits.q_max)
    poses = [kinematics.ee_pose(q) for q in waypoints]
    geometry = PathGeometry.from_samples(
        np.array([pose[0] for pose in poses]),
        np.array([pose[1] for pose in poses]),
        np.zeros(3), np.array([0.0, 0.5, 1.0]),
    )
    loop = JaxControlLoop(dt=0.002, temporal_lambda=0.2,
                          enable_x64=True, task_mode=request.param)
    loop.configure_path(geometry, PathFollowingConfig())
    loop.init_cbf()
    return loop, geometry, kinematics, q_reference


@pytest.mark.parametrize("along_error_m", [-0.003, 0.003], ids=["behind", "ahead"])
def test_path_feedback_converges_from_both_sides_of_stationary_reference(
        stationary_reference, along_error_m):
    loop, geometry, kinematics, q_reference = stationary_reference
    q = q_reference.copy()
    q[0] += along_error_m
    reference_progress = geometry.arc_length_m[1]
    reference = geometry.sample(reference_progress)
    state = loop.initial_path_state().copy()
    state[0] = reference_progress
    state[1] = reference_progress + along_error_m
    state[2] = 0 if along_error_m < 0.0 else 1
    previous_command = np.zeros(9)
    limits = kinematics.joint_limits
    initial_position = kinematics.ee_position(q)
    assert np.linalg.norm(initial_position - reference.position_m) == pytest.approx(0.003)

    for step in range(100):
        result = loop.path_tracking_step(
            q=q, path_state=state, kp_pos=60.0, kp_orient=10.0,
            kp_joint=0.45, q_des=q_reference, nullspace_speed_limit=0.18,
            damping=1e-3, u_safe_prev=previous_command,
        )
        assert result.qp_ok
        np.testing.assert_allclose(result.reference_position_m, reference.position_m, atol=1e-12)
        assert np.all(result.q_next >= limits.q_min)
        assert np.all(result.q_next <= limits.q_max)
        assert np.all(np.abs(result.u_safe) <= limits.dq_max + 1e-6)
        if step == 0:
            motion = kinematics.ee_position(result.q_next) - initial_position
            assert np.dot(motion, reference.tangent) * along_error_m < 0.0
        q, state = result.q_next, result.path_state
        previous_command = result.u_safe

    assert np.linalg.norm(kinematics.ee_position(q) - reference.position_m) < 0.05e-3
