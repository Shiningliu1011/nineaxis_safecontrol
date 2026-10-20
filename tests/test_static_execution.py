from pathlib import Path

import numpy as np

from robot_safecontrol_moveit.oscbf_trajectory import bootstrap_portable
from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig

ROOT = Path(__file__).resolve().parents[1]
bootstrap_portable(ROOT / "portable_oscbf")

from work.actuator_limits import load_actuator_limit_profile
from work.nineaxis_kinematics import NineaxisKinematics
from work.static_collision import StaticGeometry
from work.static_execution import ExecutionSchedule, ScheduledMotion
from work.static_motion import ActuatorState, MotionPlanner, certify_motion


def test_reference_settlement_requires_accepted_active_motion():
    config = ExperimentConfig.load(ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT)
    profile, kin = load_actuator_limit_profile(), NineaxisKinematics()
    planner = MotionPlanner(
        kin.joint_limits.q_min, kin.joint_limits.q_max, profile.velocity_limits,
        profile.acceleration_limits, jerk_time=.08, minimum_duration_s=.08,
    )
    geometry = StaticGeometry(config.scene())
    schedule = ExecutionSchedule(ActuatorState(config.initial_position, np.zeros(9), np.zeros(9)), planner, .02)
    motion = planner.propose(schedule.active.state_at_step(10), np.zeros(9), period_s=.02)
    certificate = certify_motion(motion, geometry, planner)
    assert certificate.accepted
    request = ScheduledMotion(1, 10, 0., .006, "RUNNING", motion)
    assert schedule.queue(request, certificate) == "accepted"
    for _ in range(9):
        assert schedule.advance()["reference_progress_m"] == 0
    assert schedule.advance()["active_command_id"] == 1
    for index in range(1, 11):
        snapshot = schedule.advance()
        np.testing.assert_allclose(snapshot["reference_progress_m"], index * .002 * .006, atol=1e-15)
    # 已执行周期之后的自动停止不继续增加参考进度。
    for _ in range(50):
        snapshot = schedule.advance()
        np.testing.assert_allclose(snapshot["reference_progress_m"], .02 * .006, atol=1e-15)
    late = ScheduledMotion(2, 60, .02 * .006, .01, "RUNNING", motion)
    assert schedule.queue(late, certificate) == "late_command"
    assert schedule.pending is None
    assert schedule.advance()["reference_progress_m"] == .02 * .006
    wrong = ScheduledMotion(3, 100, .5, .01, "RUNNING", motion)
    assert schedule.queue(wrong, certificate) == "reference_mismatch"


def test_control_start_advances_without_repeating_rejected_execution_step():
    config = ExperimentConfig.load(ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT)
    profile, kin = load_actuator_limit_profile(), NineaxisKinematics()
    planner = MotionPlanner(
        kin.joint_limits.q_min, kin.joint_limits.q_max, profile.velocity_limits,
        profile.acceleration_limits, jerk_time=.08, minimum_duration_s=.08,
    )
    schedule = ExecutionSchedule(
        ActuatorState(config.initial_position, np.zeros(9), np.zeros(9)), planner, .1,
    )
    motion = schedule.active
    assert motion.next_control_step(1, previous_start_step=None) == 50
    assert motion.next_control_step(2, previous_start_step=50) is None
    assert motion.next_control_step(5, previous_start_step=50) is None
    assert motion.next_control_step(6, previous_start_step=50) == 100
    assert motion.next_control_step(50, previous_start_step=100) is None
    assert motion.next_control_step(51, previous_start_step=100) == 150
    assert motion.next_control_step(202, previous_start_step=150) == 300
    previous = None
    starts = []
    for step in range(1, 501):
        start = motion.next_control_step(step, previous_start_step=previous)
        if start is not None:
            starts.append(start)
            previous = start
    assert starts == list(range(50, 551, 50))
    assert all(start > step for step, start in (
        (1, 50), (6, 100), (51, 150), (202, 300),
    ))
