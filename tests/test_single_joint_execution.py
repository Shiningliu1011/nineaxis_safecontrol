from dataclasses import replace
from math import nan
from pathlib import Path
import struct

import pytest

from robot_safecontrol_moveit.drempower_joint_control import DrEmpowerJointControl, JointFeedback
from robot_safecontrol_moveit.unit_conversion import JOINT_NAMES, JointNodeMap
from robot_safecontrol_moveit.single_joint_execution import (
    ExecutionLimits, MotionRejected, ObservedFeedback, SingleJointMonitor,
    decode_observation,
)


URDF = Path(__file__).resolve().parents[1] / "models/ninezzhou/urdf/ninezzhou.urdf"


def sample(t=1.0, angle=0.0, speed=0.0, done=True):
    # 状态机数值输入，测试不创建或替代任何硬件传输对象。
    return ObservedFeedback(2, JointFeedback(angle, speed), t, t + 0.001, done, False)


def monitor(**kwargs):
    return SingleJointMonitor(2, ExecutionLimits(-90, 90, **kwargs))


def test_urdf_limits_and_linear_joint_rejected():
    limits = ExecutionLimits.from_urdf(URDF, "J2")
    assert limits.upper_deg == pytest.approx(90, abs=0.001)
    assert limits.lower_deg == pytest.approx(-90, abs=0.001)
    with pytest.raises(ValueError):
        ExecutionLimits.from_urdf(URDF, "J1")


@pytest.mark.parametrize("delta", [5.01, -5.01, 0, nan])
def test_invalid_delta_latches(delta):
    guard = monitor()
    with pytest.raises(MotionRejected, match="step_limit"):
        guard.start(sample(), delta, 1.002)
    assert guard.phase == "aborted"
    with pytest.raises(MotionRejected, match="step_limit"):
        guard.start(sample(2), 1, 2.002)


@pytest.mark.parametrize("feedback,now,reason", [
    (replace(sample(), node_id=3), 1.002, "feedback_node_mismatch"),
    (replace(sample(), axis_error=True), 1.002, "axis_error"),
    (sample(angle=91), 1.002, "joint_limit"),
    (sample(speed=0.21), 1.002, "velocity_limit"),
    (sample(speed=0.03), 1.002, "initial_joint_moving"),
    (sample(angle=nan), 1.002, "nonfinite_feedback"),
    (sample(), 1.3, "feedback_timeout"),
    (sample(), 0.9, "feedback_clock_invalid"),
    (replace(sample(), request_s=2), 2.1, "feedback_clock_invalid"),
])
def test_feedback_admission(feedback, now, reason):
    with pytest.raises(MotionRejected, match=reason):
        monitor().start(feedback, 5, now)


def test_target_limit_checked_before_motion():
    with pytest.raises(MotionRejected, match="target_joint_limit"):
        monitor().start(sample(angle=89), 5, 1.002)


def test_repeated_feedback_rejected():
    guard = monitor()
    guard.start(sample(), 5, 1.002)
    with pytest.raises(MotionRejected, match="feedback_repeated"):
        guard.observe(sample(), 1.003)


def test_position_jump_rejected_even_with_zero_reported_speed():
    guard = monitor()
    guard.start(sample(), 5, 1.002)
    with pytest.raises(MotionRejected, match="feedback_position_jump"):
        guard.observe(sample(1.02, 4), 1.022)


def test_wrong_direction_rejected():
    guard = monitor()
    guard.start(sample(), 5, 1.002)
    with pytest.raises(MotionRejected, match="motion_outside_requested_interval"):
        guard.observe(sample(1.2, -0.2), 1.202)


def test_motion_timeout_with_fresh_feedback():
    guard = monitor(motion_timeout_s=1)
    guard.start(sample(), 5, 1.002)
    for i in range(1, 10):
        guard.observe(sample(1 + i * 0.1), 1.002 + i * 0.1)
    with pytest.raises(MotionRejected, match="motion_timeout"):
        guard.observe(sample(2.01), 2.012)


def test_position_arrival_and_velocity_settling_have_separate_deadlines():
    guard = monitor()
    guard.start(sample(), 5, 1.002)
    for index in range(1, 211):
        t = 1 + index * 0.1
        angle = min(5, index * 0.03)
        assert not guard.observe(sample(t, angle, 0.03, angle == 5), t + 0.002)
    assert guard.phase == "settling"
    for t in (22.1, 22.2, 22.3, 22.4, 22.5):
        assert not guard.observe(sample(t, 5), t + 0.002)
    assert guard.observe(sample(22.7, 5), 22.702)
    assert guard.phase == "completed"


def test_settling_still_has_a_deadline():
    guard = monitor()
    guard.start(sample(), 5, 1.002)
    with pytest.raises(MotionRejected, match="settling_timeout"):
        for index in range(1, 251):
            t = 1 + index * 0.1
            angle = min(5, index * 0.03)
            guard.observe(sample(t, angle, 0.03, angle == 5), t + 0.002)


def test_velocity_limit_remains_active_during_settling():
    guard = monitor()
    guard.start(sample(), 5, 1.002)
    for index in range(1, 170):
        t = 1 + index * 0.1
        angle = min(5, index * 0.03)
        guard.observe(sample(t, angle, 0.03, angle == 5), t + 0.002)
    with pytest.raises(MotionRejected, match="velocity_limit"):
        guard.observe(sample(18, 5, 0.21), 18.002)


def test_fresh_sample_after_polling_gap_does_not_resume_motion():
    guard = monitor()
    guard.start(sample(), 5, 1.002)
    with pytest.raises(MotionRejected, match="feedback_gap"):
        guard.observe(sample(1.5), 1.502)


@pytest.mark.parametrize("direction", [-1, 1])
def test_continuous_arrival_required(direction):
    guard = monitor()
    guard.start(sample(), direction, 1.002)
    for i in range(1, 21):
        assert not guard.observe(sample(1 + i * 0.1, direction * i / 20, 0.08, False), 1.002 + i * 0.1)
    for t in (3.1, 3.2, 3.3):
        assert not guard.observe(sample(t, direction), t + 0.002)
    assert not guard.observe(sample(3.4, direction, done=False), 3.402)
    for t in (3.5, 3.6, 3.7, 3.8, 3.9):
        assert not guard.observe(sample(t, direction), t + 0.002)
    assert guard.observe(sample(4.1, direction), 4.102)
    assert guard.phase == "completed"
    with pytest.raises(MotionRejected):
        guard.start(sample(5), 1, 5.002)


@pytest.mark.parametrize("kwargs", [
    {"speed_rpm": 1}, {"max_step_deg": 6}, {"feedback_timeout_s": 1},
    {"settle_s": 0.01}, {"tolerance_deg": 2}, {"max_feedback_rpm": nan},
    {"accel_rpm_s": 0}, {"stopped_rpm": 0.15},
])
def test_invalid_limits(kwargs):
    with pytest.raises(ValueError):
        monitor(**kwargs)


def test_can_feedback_decode_preserves_source_and_applies_direction():
    control = DrEmpowerJointControl(
        URDF.parents[3] / "config/hardware_joint_zero.yaml",
        JointNodeMap.from_joint_list(JOINT_NAMES),
    )
    offset = control.converter.calibration.for_joint("J2").zero_offset_deg
    data = struct.pack("<fhh", offset - 5.0, -10, 0)
    result = decode_observation(control, "J2", (2 << 5) | 3, data, 1, 1.001)
    assert result.state.angle_deg == pytest.approx(5)
    assert result.state.speed_rpm == pytest.approx(0.1)
    assert result.request_s == 1
    assert result.received_s == 1.001
    assert result.traj_done
    assert not result.axis_error
    for frame_id in ((3 << 5) | 3, (2 << 5) | 0x1F, 3):
        with pytest.raises((ValueError, MotionRejected)):
            decode_observation(control, "J2", frame_id, data, 1, 1.001)
    with pytest.raises(ValueError):
        decode_observation(control, "J2", (2 << 5) | 3, data[:4], 1, 1.001)
    fault = decode_observation(control, "J2", (2 << 5) | 7, data, 1, 1.001)
    with pytest.raises(MotionRejected, match="axis_error"):
        monitor().start(fault, 5, 1.002)
