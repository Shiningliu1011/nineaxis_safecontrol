"""Tests for the 5-D position plus tool-axis task geometry.

The tool axis is the end-effector X axis in this robot's fixed-orientation
contract.  A pure roll about that axis must remain free.
"""

import jax
import numpy as np
from scipy.spatial.transform import Rotation

from work.nineaxis_kinematics import NineaxisKinematics
from work.tool_axis_task import (
    TOOL_AXIS_INDEX,
    task_jacobian_5d,
    task_jacobian_5d_jax,
    tool_axis_error_2d,
)


def test_roll_about_tool_axis_has_zero_5d_orientation_error():
    desired = np.eye(3)
    current = Rotation.from_rotvec([0.83, 0.0, 0.0]).as_matrix()

    error = tool_axis_error_2d(current, desired)

    np.testing.assert_allclose(error, np.zeros(2), atol=1.0e-12)


def test_axis_error_feedback_sign_opposes_small_axis_tilt():
    desired = np.eye(3)
    # A positive rotation about world Y tilts the tool X axis toward -Z.
    current = Rotation.from_rotvec([0.0, 1.0e-4, 0.0]).as_matrix()

    error = tool_axis_error_2d(current, desired)

    assert error[0] > 0.0
    np.testing.assert_allclose(error[1], 0.0, atol=1.0e-10)


def test_5d_axis_jacobian_matches_finite_difference():
    kin = NineaxisKinematics()
    q = np.array([0.25, 0.16, -0.98, 0.53, -2.64, -0.85, -0.16, -0.97, 1.18])
    _, desired_rotation = kin.ee_pose(q)
    jacobian_6d = kin.compute_full_jacobian(q)
    jacobian_5d = task_jacobian_5d(jacobian_6d, desired_rotation, desired_rotation)

    epsilon = 1.0e-6
    finite_difference = np.zeros((2, 9))
    for joint in range(9):
        dq = np.zeros(9)
        dq[joint] = epsilon
        _, rotation_plus = kin.ee_pose(q + dq)
        _, rotation_minus = kin.ee_pose(q - dq)
        finite_difference[:, joint] = (
            tool_axis_error_2d(rotation_plus, desired_rotation)
            - tool_axis_error_2d(rotation_minus, desired_rotation)
        ) / (2.0 * epsilon)

    assert jacobian_5d.shape == (5, 9)
    np.testing.assert_allclose(jacobian_5d[:3], jacobian_6d[:3], atol=1.0e-12)
    np.testing.assert_allclose(jacobian_5d[3:], finite_difference, atol=3.0e-5)


def test_jax_and_numpy_5d_jacobians_agree():
    kin = NineaxisKinematics()
    q = np.array([0.22, 0.14, -0.95, 0.49, -2.61, -0.82, -0.14, -0.93, 1.13])
    _, rotation = kin.ee_pose(q)
    jacobian_6d = kin.compute_full_jacobian(q)

    expected = task_jacobian_5d(jacobian_6d, rotation, rotation)
    actual = np.asarray(jax.jit(task_jacobian_5d_jax, static_argnums=3)(
        jacobian_6d, rotation, rotation, TOOL_AXIS_INDEX))

    # Standalone JAX tests may run with x64 disabled, while the control loop
    # explicitly enables x64 before compiling.  This assertion checks the
    # algebraic mirror rather than imposing an unrelated global dtype policy.
    np.testing.assert_allclose(actual, expected, atol=2.0e-7, rtol=2.0e-7)
