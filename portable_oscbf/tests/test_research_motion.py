from dataclasses import replace

import numpy as np
import pytest
from ruckig import ControlInterface, InputParameter, Result, Ruckig, Trajectory
from work.nineaxis_model import NineaxisModel
from work.process_limits import ProcessLimits
from work.static_motion import ActuatorState, MotionPlanner


@pytest.fixture(scope="module")
def bound():
    return ProcessLimits.load().bind(NineaxisModel.load(), path_hash="a" * 64,
        workpiece_hash="b" * 64, scene_hash="c" * 64, path_length_m=2., phase="tracking", evidence_scope="research_simulation")


def independent_trajectory(state, target, bound, minimum_duration):
    inputs = InputParameter(9)
    inputs.control_interface = ControlInterface.Velocity
    inputs.current_position = state.position.tolist()
    inputs.current_velocity = state.velocity.tolist()
    inputs.current_acceleration = state.acceleration.tolist()
    inputs.target_velocity = target.tolist()
    inputs.target_acceleration = [0.] * 9
    inputs.max_velocity = bound.max_velocity.tolist()
    inputs.max_acceleration = bound.max_acceleration.tolist()
    # 需求给定的数值独立换算，不从 planner 对象获取。
    inputs.max_jerk = [.001 * 62500] + [np.deg2rad(600)] * 5 + [np.deg2rad(750)] * 3
    inputs.minimum_duration = minimum_duration
    algorithm, result = Ruckig(9, .002), Trajectory(9)
    assert algorithm.calculate(inputs, result) in (Result.Working, Result.Finished)
    return result


@pytest.mark.parametrize("axis", range(9))
def test_each_axis_ruckig_consumes_confirmed_jerk_and_stops_continuously(bound, axis):
    planner = MotionPlanner.from_limits(bound, minimum_duration_s=.08)
    initial = ActuatorState((bound.position_lower + bound.position_upper) / 2, np.zeros(9), np.zeros(9))
    target = np.zeros(9)
    target[axis] = bound.max_velocity[axis] * .2
    motion = planner.propose(initial, target, period_s=.12)
    expected = independent_trajectory(initial, target, bound, .08)
    for time_s in np.linspace(0., .12, 5):
        actual = motion.at_time(time_s)
        np.testing.assert_allclose(np.asarray([actual.position, actual.velocity, actual.acceleration]), expected.at_time(time_s), atol=1e-14, rtol=0)
    stop_initial = ActuatorState(*expected.at_time(.12))
    expected_stop = independent_trajectory(stop_initial, np.zeros(9), bound, .08)
    for time_s in np.linspace(0., expected_stop.duration, 7):
        actual = motion.at_time(.12 + time_s)
        np.testing.assert_allclose(np.asarray([actual.position, actual.velocity, actual.acceleration]), expected_stop.at_time(time_s), atol=2e-14, rtol=0)
    assert motion.bounds.max_jerk[axis] <= bound.max_jerk[axis] + 1e-12
    final = motion.at_time(motion.duration_s)
    np.testing.assert_allclose(final.velocity, 0., atol=1e-14)
    np.testing.assert_allclose(final.acceleration, 0., atol=1e-14)
    assert motion.limits_identity == bound.identity
    assert bound.check_joint_motion(motion).accepted
    reconstructed = planner.from_record(motion.to_record())
    np.testing.assert_allclose(reconstructed.at_time(.05).position, motion.at_time(.05).position)
    changed = MotionPlanner.from_limits(replace(bound, phase="stopping"), minimum_duration_s=.08)
    with pytest.raises(ValueError, match="limits_identity"):
        changed.from_record(motion.to_record())


def test_full_joint_motion_check_includes_stop_position_and_identity(bound):
    planner = MotionPlanner.from_limits(bound, minimum_duration_s=.08)
    position = (bound.position_lower + bound.position_upper) / 2
    position[0] = bound.position_upper[0] - .0001
    initial = ActuatorState(position, np.array([.05] + [0.] * 8), np.zeros(9))
    motion = planner.propose(initial, np.zeros(9), period_s=.02)
    report = bound.check_joint_motion(motion)
    assert not report.accepted and "joint_position" in report.violations
    with pytest.raises(ValueError, match="limits_identity"):
        bound.check_joint_motion(replace(motion, limits_identity="0" * 64))


def test_explicit_jerk_and_existing_jerk_time_have_unambiguous_validation(bound):
    arguments = (bound.position_lower, bound.position_upper, bound.max_velocity, bound.max_acceleration)
    legacy = MotionPlanner(*arguments, jerk_time=.08, minimum_duration_s=.08)
    np.testing.assert_array_equal(legacy.max_jerk, bound.max_acceleration / .08)
    for jerk in (np.ones(8), np.zeros(9), np.full(9, np.nan)):
        with pytest.raises(ValueError):
            MotionPlanner(*arguments, max_jerk=jerk, minimum_duration_s=.08)
    with pytest.raises(ValueError, match="jerk_time"):
        MotionPlanner(*arguments, max_jerk=bound.max_jerk, jerk_time=.08, minimum_duration_s=.08)
    with pytest.raises(ValueError, match="jerk"):
        MotionPlanner(*arguments, minimum_duration_s=.08)
