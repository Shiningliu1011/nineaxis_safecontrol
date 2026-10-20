from pathlib import Path

import numpy as np
import pytest
from ruckig import ControlInterface, InputParameter, Result, Ruckig, Trajectory

from robot_safecontrol_moveit.oscbf_trajectory import bootstrap_portable
from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig

ROOT = Path(__file__).resolve().parents[1]
bootstrap_portable(ROOT / "portable_oscbf")

from work.actuator_limits import load_actuator_limit_profile
from work.nineaxis_kinematics import NineaxisKinematics
from work.static_collision import StaticGeometry
from work.static_motion import ActuatorState, MotionPlanner, certify_motion


@pytest.fixture(scope="module")
def resources():
    config = ExperimentConfig.load(ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT)
    kin = NineaxisKinematics()
    profile = load_actuator_limit_profile()
    planner = MotionPlanner(
        kin.joint_limits.q_min, kin.joint_limits.q_max,
        profile.velocity_limits, profile.acceleration_limits, jerk_time=.08,
        minimum_duration_s=.08,
    )
    return config, planner, StaticGeometry(config.scene())


def test_motion_and_stop_match_independent_ruckig_calculation(resources):
    config, planner, geometry = resources
    initial = ActuatorState(config.initial_position, np.zeros(9), np.zeros(9))
    requested = np.array([.002, .003, -.002, .001, -.003, .002, -.001, .002, -.001])
    motion = planner.propose(initial, requested, period_s=.02)
    direct = InputParameter(9)
    direct.control_interface = ControlInterface.Velocity
    direct.current_position = initial.position.tolist()
    direct.current_velocity = initial.velocity.tolist()
    direct.current_acceleration = initial.acceleration.tolist()
    direct.target_velocity = requested.tolist()
    direct.target_acceleration = [0.] * 9
    direct.max_velocity = planner.max_velocity.tolist()
    direct.max_acceleration = planner.max_acceleration.tolist()
    direct.max_jerk = planner.max_jerk.tolist()
    direct.minimum_duration = .08
    trajectory = Trajectory(9)
    assert Ruckig(9, .002).calculate(direct, trajectory) in (Result.Working, Result.Finished)
    for time_s in np.arange(0, .0201, .002):
        expected = trajectory.at_time(float(time_s))
        state = motion.at_time(float(time_s))
        np.testing.assert_allclose(state.position, expected[0], atol=1e-14)
        np.testing.assert_allclose(state.velocity, expected[1], atol=1e-14)
        np.testing.assert_allclose(state.acceleration, expected[2], atol=1e-14)
    final = motion.at_time(motion.duration_s + 1)
    np.testing.assert_allclose(final.velocity, 0, atol=1e-12)
    np.testing.assert_allclose(final.acceleration, 0, atol=1e-12)
    certificate = certify_motion(motion, geometry, planner)
    assert certificate.accepted
    assert certificate.checked_intervals > 0
    for time_s in np.linspace(0, motion.duration_s, 101):
        state = motion.at_time(float(time_s))
        assert np.all(state.position >= planner.lower)
        assert np.all(state.position <= planner.upper)
        assert all(
            row.distance.valid and row.distance.distance_m >= row.required_m
            for row in geometry.query(state.position)
        )


def test_current_velocity_and_acceleration_are_preserved(resources):
    config, planner, _ = resources
    state = ActuatorState(config.initial_position, np.array([.02] + [0.] * 8),
                          np.array([.05] + [0.] * 8))
    motion = planner.propose(state, np.zeros(9), period_s=.02)
    np.testing.assert_array_equal(motion.at_time(0).velocity, state.velocity)
    np.testing.assert_array_equal(motion.at_time(0).acceleration, state.acceleration)
    assert motion.at_time(.002).position[0] > state.position[0]


def test_processing_can_reject_a_velocity_inside_its_bounds(resources):
    config, planner, geometry = resources
    q = config.initial_position.copy()
    q[0] = planner.upper[0] - .0001
    state = ActuatorState(q, np.array([.05] + [0.] * 8), np.zeros(9))
    motion = planner.propose(state, np.zeros(9), period_s=.02)
    certificate = certify_motion(motion, geometry, planner)
    assert not certificate.accepted
    assert certificate.reason == "joint_position"


def test_nonfinite_state_cannot_create_motion():
    with pytest.raises(ValueError):
        ActuatorState(np.full(9, np.nan), np.zeros(9), np.zeros(9))


@pytest.mark.parametrize("period", [.02, .1])
def test_continuous_joint_bounds_include_turning_points_and_truncated_tracking(resources, period):
    config, planner, _ = resources
    state = ActuatorState(
        config.initial_position, np.array([.02, .01] + [0.] * 7),
        np.array([.05, .1] + [0.] * 7),
    )
    motion = planner.propose(state, np.array([-.015, -.02] + [0.] * 7), period_s=period)
    bounds = motion.bounds
    for start, end in ((0., period), (period, motion.duration_s), (0., motion.duration_s)):
        low, high = bounds.position_range(start, end)
        samples = [motion.at_time(float(value)) for value in np.linspace(start, end, 1001)]
        positions = np.array([value.position for value in samples])
        velocity = np.abs([value.velocity for value in samples])
        acceleration = np.abs([value.acceleration for value in samples])
        assert np.all(positions >= low - 1e-14)
        assert np.all(positions <= high + 1e-14)
        assert np.all(velocity <= bounds.max_velocity + 1e-12)
        assert np.all(acceleration <= bounds.max_acceleration + 1e-12)
    if period == .1:
        assert bounds.upper[0] > max(state.position[0], motion.at_time(motion.duration_s).position[0]) + 1e-6
    else:
        assert motion.tracking.duration > period
        assert bounds.upper[0] == motion.at_time(motion.duration_s).position[0]
