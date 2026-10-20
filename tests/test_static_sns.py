from pathlib import Path

import numpy as np
import pytest
from scipy.optimize import linprog

from robot_safecontrol_moveit.oscbf_trajectory import bootstrap_portable

bootstrap_portable(Path(__file__).resolve().parents[1] / "portable_oscbf")

from work.static_sns import generalized_sns, add_posture, damped_pseudoinverse


@pytest.mark.parametrize("generalized", [False, True])
def test_generalized_saturation_matches_independent_maximum_task_scaling(generalized):
    J = np.array([[1., 0., 1.], [0., 1., 1.]])
    desired = np.array([2., 1.])
    A = np.eye(3)
    lower, upper = -np.array([.6, .5, .4]), np.array([.6, .5, .4])
    if generalized:
        A = np.vstack([A, [1., 1., 0.]])
        lower, upper = np.r_[lower, -1.1], np.r_[upper, .3]
    solution = generalized_sns(J, desired, A, lower, upper)
    direct = linprog(
        [0., 0., 0., -1.], A_ub=np.vstack([
            np.column_stack([A, np.zeros(len(A))]),
            np.column_stack([-A, np.zeros(len(A))]),
        ]), b_ub=np.r_[upper, -lower],
        A_eq=np.column_stack([J, -desired]), b_eq=np.zeros(2),
        bounds=[(None, None)] * 3 + [(0., 1.)], method="highs",
    )
    assert direct.success
    assert solution.task_scale == pytest.approx(direct.x[-1], abs=1e-9)
    assert solution.task_scale == pytest.approx(1.1 / 3 if generalized else .5, abs=1e-9)
    np.testing.assert_allclose(J @ solution.velocity, solution.task_scale * desired, atol=1e-9)
    assert np.all(A @ solution.velocity >= lower - 1e-9)
    assert np.all(A @ solution.velocity <= upper + 1e-9)
    assert solution.saturated


def test_posture_preserves_primary_task_and_respects_generalized_limits():
    J = np.array([[1., 0., 0.]])
    A, lower, upper = np.eye(3), -np.ones(3), np.ones(3)
    original = generalized_sns(J, [.2], A, lower, upper)
    adjusted = add_posture(original, J, [5., 2., -1.], A, lower, upper)
    np.testing.assert_allclose(adjusted.velocity, [.2, 1., -.5], atol=1e-10)
    np.testing.assert_allclose(J @ adjusted.velocity, [.2], atol=1e-10)


def test_zero_task_and_fully_blocked_task_remain_feasible():
    A, J = np.eye(3), np.array([[1., 0., 0.]])
    zero = generalized_sns(J, [0.], A, -np.ones(3), np.ones(3))
    np.testing.assert_array_equal(zero.velocity, 0.)
    blocked = generalized_sns(J, [1.], A, -np.ones(3), np.array([0., 1., 1.]))
    assert blocked.task_scale == 0.
    np.testing.assert_array_equal(blocked.velocity, 0.)


def test_author_damping_parameters_and_invalid_input():
    matrix = np.diag([1., .05])
    expected = np.diag(np.array([1., .05]) / (np.array([1., .05]) ** 2 + .75e-6))
    np.testing.assert_allclose(damped_pseudoinverse(matrix), expected, atol=1e-12)
    with pytest.raises(ValueError):
        generalized_sns([[1.]], [1.], [[1.]], [.1], [1.])
