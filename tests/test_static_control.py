from dataclasses import asdict
from pathlib import Path

import jax
import numpy as np
import pytest
from scipy.optimize import LinearConstraint, minimize

from robot_safecontrol_moveit.oscbf_trajectory import bootstrap_portable
from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig

ROOT = Path(__file__).resolve().parents[1]
bootstrap_portable(ROOT / "portable_oscbf")
jax.config.update("jax_enable_x64", True)

from work.actuator_limits import load_actuator_limit_profile
from work.nineaxis_kinematics import NineaxisKinematics
from work.static_collision import StaticGeometry
from work.static_control import StaticControl
from work.static_motion import ActuatorState, MotionPlanner, certify_motion
from work.static_velocity_solver import build_constraints, common_posture_velocity, solve_velocity


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
        .4708505282478995 - .085, -1.2838444178875443, .44274328581177236,
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
        .11265206494784251 - .085, -.3676176768590687, .480671785341436,
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


def test_s5_repeated_velocity_trials_keep_independent_qp_and_motion_evidence():
    config = ExperimentConfig.load(
        ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT,
        method="D", clearance_reserve_m=.0001,
    )
    scene = config.scene()
    scene["obstacles"] = [{
        "name": "path_block", "radius_m": .06,
        "center_m": [.0002744757528524406, .27357448955295105, 1.3404106022559328],
    }]
    profile, kin = load_actuator_limit_profile(), NineaxisKinematics()
    planner = MotionPlanner(
        kin.joint_limits.q_min, kin.joint_limits.q_max, profile.velocity_limits,
        profile.acceleration_limits, jerk_time=.08, minimum_duration_s=.08,
    )
    geometry = StaticGeometry(scene)
    control = StaticControl(config.load_path()[0], geometry, planner, config.values["control"])
    # 来自本机 s5-current-01 的最终实际候选，已停止且正向运动受阻。
    state = ActuatorState(np.array([
        .4546786057344995 - .085, -1.285951296455923, .4853365101649853,
        1.3546770709178174, 1.075906905721642, .4645223562096255,
        .8542537781206053, -1.4448177304276972, -.9157218150096683,
    ]), np.zeros(9), np.zeros(9))
    result = control.propose(state, .0037895319972561063, time_s=0., executed_feed=0.)
    assert result is None
    assert len(control.last_trials) == 9
    velocities = [row["velocity"] for row in control.last_trials]
    assert any(np.array_equal(first, second) for index, first in enumerate(velocities)
               for second in velocities[index + 1:])
    for trial in control.last_trials:
        problem, solution = trial["problem"], np.asarray(trial["solution"])
        matrix, vector = np.asarray(problem["P"]), np.asarray(problem["c"])
        constraints, bounds = np.asarray(problem["G"]), np.asarray(problem["h"])
        assert np.max(constraints @ solution - bounds) <= 1e-9
        dual = np.asarray(trial["dual"])
        assert np.min(dual) >= -1e-8
        assert np.max(abs(matrix @ solution + vector + constraints.T @ dual)) <= 1e-8
        assert np.max(abs((bounds - constraints @ solution) * dual)) <= 1e-8
        independent_qp = minimize(
            lambda value: (.5 * value @ matrix @ value + vector @ value,
                           matrix @ value + vector),
            solution, jac=True, method="SLSQP",
            constraints=[LinearConstraint(constraints, -np.inf, bounds)],
            options={"ftol": 1e-13, "maxiter": 1000},
        )
        assert independent_qp.success
        assert np.max(constraints @ independent_qp.x - bounds) <= 1e-9
        cost = .5 * solution @ matrix @ solution + vector @ solution
        assert cost == pytest.approx(independent_qp.fun, abs=1e-8)
        motion = planner.propose(state, trial["velocity"], period_s=.1)
        independent = certify_motion(motion, geometry, planner)
        assert trial["certificate"] == asdict(independent)
        assert not independent.accepted
    original_trials = control.last_trials
    assert control.propose(state, .0037895319972561063, time_s=.1, executed_feed=0.) is None
    assert len(control.last_trials) == len(original_trials)
    for index, trial in enumerate(control.last_trials):
        assert trial["iterations"] == 0
        assert trial["qp_reused_from_proposal"] == {
            "proposal_sequence": original_trials[index]["proposal_sequence"],
            "proposal_time_s": 0., "trial": index,
        }
        assert trial["motion_reused_from_proposal"]["proposal_time_s"] == 0.
        for key in ("problem", "solution", "dual", "velocity", "certificate", "kkt_residual"):
            assert trial[key] == original_trials[index][key]
    control.propose(state, .0037995319972561063, time_s=.2, executed_feed=0.)
    assert all(trial["qp_reused_from_proposal"] is None for trial in control.last_trials)
    accelerated = ActuatorState(state.position, state.velocity, np.full(9, 1e-8))
    control.propose(accelerated, .0037995319972561063, time_s=.3, executed_feed=0.)
    assert all(trial["motion_reused_from_proposal"] is None for trial in control.last_trials)
    assert control.propose(state, .0037895319972561063, time_s=30.1, executed_feed=0.) is None
    assert control.recovery.phase == "FAULT"
    assert control.recovery.reason == "pause_timeout"
    settings = config.values["control"]
    constraints = build_constraints(
        state.position, control.limits, control.last_measurements, period_s=.1,
        alpha=settings["cbf_alpha"], clearance_reserve_m=.0001,
    )
    posture = common_posture_velocity(
        state.position, control.limits, control.last_terms.jacobian, control.last_measurements,
        speed_limit=settings["posture_speed_limit"], influence_m=settings["posture_influence_m"],
        joint_margin_fraction=settings["posture_joint_margin_fraction"],
    )
    arguments = {
        "previous_velocity": state.velocity, "posture_velocity": posture, "mode": "joint",
        "gains": settings["task_gains"], "task_weights": settings["task_weights"],
        "posture_weight": settings["posture_weight"], "smooth_weight": settings["smooth_weight"],
        "feed_weight": settings["feed_weight"], "max_velocity": planner.max_velocity,
    }
    boundary = solve_velocity(control.last_terms, constraints, nominal_feed=.0001875, **arguments)
    assert abs(boundary.feed_m_s) <= np.finfo(float).eps * .0001875
    # 独立求解精确零进给，验证后续 QP 对该边界候选的 KKT 检查。
    boundary = solve_velocity(control.last_terms, constraints, nominal_feed=0., **arguments)
    assert boundary.feed_m_s == 0.
    for nominal in (.00009375, .003):
        reused = solve_velocity(
            control.last_terms, constraints, nominal_feed=nominal,
            previous_zero_feed=boundary, **arguments,
        )
        independent = solve_velocity(control.last_terms, constraints, nominal_feed=nominal, **arguments)
        matrix, vector = reused.problem["P"], reused.problem["c"]
        actual_cost = .5 * reused.solution @ matrix @ reused.solution + vector @ reused.solution
        expected_cost = .5 * independent.solution @ matrix @ independent.solution + vector @ independent.solution
        assert actual_cost == pytest.approx(expected_cost, abs=1e-8)
        if nominal == .003:
            assert reused.feed_m_s > 1e-8
            assert not reused.reused_zero_feed


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
    # 关节位置来自 s4-joint-development-10；测试当前连续路径的最后可表示区间。
    state = ActuatorState(np.array([
        .1306941177379043 - .085, -.7060779958979104, .19350705137335877,
        .7804435751336972, 1.0790893334269591, .05484058431642864,
        .4397736851681241, -.22787388223368704, -.6662958160755214,
    ]), np.zeros(9), np.zeros(9))
    progress, feed = np.nextafter(path.total_length_m, 0.), 0.
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
