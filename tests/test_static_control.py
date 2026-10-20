from pathlib import Path

import jax
import numpy as np
import pytest

from robot_safecontrol_moveit.oscbf_trajectory import bootstrap_portable
from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig

ROOT = Path(__file__).resolve().parents[1]
bootstrap_portable(ROOT / "portable_oscbf")
jax.config.update("jax_enable_x64", True)

from work.actuator_limits import load_actuator_limit_profile
from work.nineaxis_kinematics import NineaxisKinematics
from work.static_collision import StaticGeometry
from work.static_control import StaticControl
from work.static_motion import ActuatorState, MotionPlanner
from work.static_velocity_solver import build_constraints


@pytest.mark.parametrize("method", ["A", "B", "C", "D", "E"])
def test_method_matrix_uses_common_processed_motion_and_advances_from_feedback(method):
    config = ExperimentConfig.load(
        ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT, method=method,
    )
    profile, kin = load_actuator_limit_profile(), NineaxisKinematics()
    planner = MotionPlanner(
        kin.joint_limits.q_min, kin.joint_limits.q_max, profile.velocity_limits,
        profile.acceleration_limits, jerk_time=.08, minimum_duration_s=.08,
    )
    control = StaticControl(config.load_path()[0], StaticGeometry(config.scene()), planner, config.values["control"])
    state = ActuatorState(config.initial_position, np.zeros(9), np.zeros(9))
    progress, feed = 0., 0.
    for index in range(6):
        candidate = control.propose(state, progress, time_s=index * .1, executed_feed=feed)
        assert candidate is not None and candidate.certificate.accepted
        feed = candidate.solution.feed_m_s
        if candidate.phase != "RUNNING":
            assert feed == 0.
        state = candidate.motion.at_time(.1)
        progress += feed * .1
    assert progress > 0.
    if method == "E":
        assert candidate.solution.solver == "sns"
        assert candidate.solution.kkt_residual is None
        assert 0 <= candidate.solution.task_scale <= 1.


def test_full_path_starts_from_stationary_profile_and_executes_approved_intervals():
    config = ExperimentConfig.load(ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT)
    path, _ = config.load_path()
    assert path.feedrate_m_s[0] <= 1e-12
    profile, kin = load_actuator_limit_profile(), NineaxisKinematics()
    planner = MotionPlanner(
        kin.joint_limits.q_min, kin.joint_limits.q_max, profile.velocity_limits,
        profile.acceleration_limits, jerk_time=.08, minimum_duration_s=.08,
    )
    control = StaticControl(path, StaticGeometry(config.scene()), planner, config.values["control"])
    state = ActuatorState(config.initial_position, np.zeros(9), np.zeros(9))
    progress, executed_feed = 0., 0.
    period = config.values["control"]["period_s"]
    for index in range(4):
        candidate = control.propose(state, progress, time_s=index * period, executed_feed=executed_feed)
        assert candidate is not None and candidate.certificate.accepted
        assert candidate.solution.feed_m_s > 0
        assert candidate.phase == "RUNNING"
        state = candidate.motion.at_time(period)
        executed_feed = candidate.solution.feed_m_s
        progress += executed_feed * period
    assert progress > 0
    assert np.linalg.norm(state.position - config.initial_position) > 0


def test_s1_near_joint_limit_retains_a_certified_stopping_motion():
    config = ExperimentConfig.load(ROOT / "config/static_experiment_s1.yaml", resource_root=ROOT)
    profile, kin = load_actuator_limit_profile(), NineaxisKinematics()
    planner = MotionPlanner(
        kin.joint_limits.q_min, kin.joint_limits.q_max, profile.velocity_limits,
        profile.acceleration_limits, jerk_time=.08, minimum_duration_s=.08,
    )
    control = StaticControl(config.load_path()[0], StaticGeometry(config.scene()), planner, config.values["control"])
    # 来自 S1 实际停止状态的关节位置及参考进度。
    state = ActuatorState(np.array([
        .4708505282478995, -1.2838444178875443, .44274328581177236,
        1.4206174701248417, 1.0995780313018346, .4412914091871797,
        .863354548132695, -1.4818299363328082, -.9465893568841603,
    ]), np.zeros(9), np.zeros(9))
    candidate = control.propose(state, 3.570794304352301e-5, time_s=10.3, executed_feed=0.)
    assert candidate is not None and candidate.certificate.accepted
    positions = np.array([
        candidate.motion.at_time(time_s).position
        for time_s in np.linspace(0, candidate.motion.duration_s, 101)
    ])
    assert np.all(positions >= planner.lower) and np.all(positions <= planner.upper)


def test_s1_joint_recovery_handles_small_starting_feed_bounds():
    config = ExperimentConfig.load(
        ROOT / "config/static_experiment_s1.yaml", resource_root=ROOT,
        solver_mode="joint", recovery_mode="improved",
    )
    profile, kin = load_actuator_limit_profile(), NineaxisKinematics()
    planner = MotionPlanner(
        kin.joint_limits.q_min, kin.joint_limits.q_max, profile.velocity_limits,
        profile.acceleration_limits, jerk_time=.08, minimum_duration_s=.08,
    )
    control = StaticControl(config.load_path()[0], StaticGeometry(config.scene()), planner, config.values["control"])
    state = ActuatorState(config.initial_position, np.zeros(9), np.zeros(9))
    progress, feed = 0., 0.
    for index in range(6):
        candidate = control.propose(state, progress, time_s=index * .1, executed_feed=feed)
        assert candidate is not None and candidate.certificate.accepted
        state = candidate.motion.at_time(.1)
        feed = candidate.solution.feed_m_s
        assert feed >= 0
        progress += feed * .1
    assert progress > 0


def test_s4_recovers_control_clearance_reserve_from_recorded_boundary_position():
    config = ExperimentConfig.load(
        ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT,
        method="D", clearance_reserve_m=.0001,
    )
    scene = config.scene()
    scene["obstacles"] = [{
        "name": "obstacle_1", "radius_m": .06,
        "center_m": [-.3329816780253617, .34379819878847984, 1.0157373322340988],
    }]
    profile, kin = load_actuator_limit_profile(), NineaxisKinematics()
    planner = MotionPlanner(
        kin.joint_limits.q_min, kin.joint_limits.q_max, profile.velocity_limits,
        profile.acceleration_limits, jerk_time=.08, minimum_duration_s=.08,
    )
    geometry = StaticGeometry(scene)
    control = StaticControl(config.load_path()[0], geometry, planner, config.values["control"])
    # 关节位置来自 s4-joint-development-08 的 sequence=5114，检查静止后的间距恢复。
    state = ActuatorState(np.array([
        .11265206494784251, -.3676176768590687, .480671785341436,
        .5931371775968358, -.020964420606434334, -.6403831043027763,
        -.22504548016947482, -.3747881652445478, .2276471440728936,
    ]), np.zeros(9), np.zeros(9))
    initial = geometry.query(state.position)
    pair = ("Link7", "obstacle_1")
    before = next(row.distance.distance_m for row in initial if row.pair == pair)
    assert .03 < before < .0301
    constraints = build_constraints(
        state.position, control.limits, initial, period_s=.1, alpha=5.,
        clearance_reserve_m=.0001,
    )
    index = constraints.labels.index(pair)
    assert constraints.h[index] < 0.
    candidate = control.propose(state, 1.1067713704829663, time_s=0., executed_feed=0.)
    assert candidate is not None and candidate.certificate.accepted
    assert candidate.solution.feed_m_s == 0.
    assert constraints.G[index] @ candidate.solution.velocity <= constraints.h[index] + 1e-9
    after = geometry.query(candidate.motion.at_time(.1).position)
    assert next(row.distance.distance_m for row in after if row.pair == pair) > before


@pytest.mark.parametrize("method", ["A", "B", "C", "D", "E"])
def test_last_reference_interval_is_executed_before_endpoint_hold(method):
    config = ExperimentConfig.load(
        ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT, method=method,
    )
    path, _ = config.load_path()
    profile, kin = load_actuator_limit_profile(), NineaxisKinematics()
    planner = MotionPlanner(
        kin.joint_limits.q_min, kin.joint_limits.q_max, profile.velocity_limits,
        profile.acceleration_limits, jerk_time=.08, minimum_duration_s=.08,
    )
    control = StaticControl(path, StaticGeometry(config.scene()), planner, config.values["control"])
    # 来自 s4-joint-development-10 的停止位置和已结算参考进度。
    state = ActuatorState(np.array([
        .1306941177379043, -.7060779958979104, .19350705137335877,
        .7804435751336972, 1.0790893334269591, .05484058431642864,
        .4397736851681241, -.22787388223368704, -.6662958160755214,
    ]), np.zeros(9), np.zeros(9))
    progress, feed = 1.8167597440584302, 0.
    assert 0. < path.total_length_m - progress < 1e-12
    for index in range(6):
        candidate = control.propose(state, progress, time_s=index * .1, executed_feed=feed)
        assert candidate is not None and candidate.certificate.accepted
        if candidate.phase == "ENDPOINT_HOLD":
            assert progress == path.total_length_m
            break
        feed = candidate.solution.feed_m_s
        state = candidate.motion.at_time(.1)
        progress += feed * .1
    else:
        pytest.fail("最后参考区间未完成实际进给结算")
