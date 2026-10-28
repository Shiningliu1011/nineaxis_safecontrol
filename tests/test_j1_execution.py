from dataclasses import replace
import json
from pathlib import Path

import pytest

from robot_safecontrol_moveit.drempower_can import CMD_PROPERTY_READ
from robot_safecontrol_moveit.drempower_joint_control import JointFeedback
from robot_safecontrol_moveit.drempower_serial import DrEmpowerSerial, SERIAL_CAN_FRAME
from robot_safecontrol_moveit.j1_execution import J1ScrewControl
from robot_safecontrol_moveit.motor_motion_limits import validate_motion_parameters
from robot_safecontrol_moveit.single_joint_execution import (
    ArrivalWindow, ExecutionLimits, MotionRejected, ObservedFeedback, SingleJointMonitor,
    VelocityWindow, checked_speed, decode_observation,
)


ROOT = Path(__file__).resolve().parents[1]
TRACE = ROOT / "tests/data/j1_static_feedback_20261028.json"
ABORT_TRACE = ROOT / "tests/data/j1_motion_abort_20261028.json"


@pytest.mark.parametrize("position_mm,encoder_deg", [(0, 0), (5, -360), (50, -3600), (500, -36000)])
def test_confirmed_linear_position_and_feedback(position_mm, encoder_deg):
    control = J1ScrewControl.from_root(ROOT)
    arguments = control.position_arguments(
        "J1", position_mm * control.degrees_per_mm, speed_rpm=3, accel_rpm_s=0.6,
    )
    assert arguments == dict(id_num=1, angle=encoder_deg, speed=3, param=0.6, mode=1)
    state = control.decode_state("J1", encoder_deg, -3)
    assert state.angle_deg / control.degrees_per_mm == pytest.approx(position_mm)
    assert state.speed_rpm / 12 == pytest.approx(0.25)


@pytest.mark.parametrize("position_mm", [-0.00001, 500.00001, float("nan"), float("inf")])
def test_linear_command_rejects_outside_confirmed_travel(position_mm):
    control = J1ScrewControl.from_root(ROOT)
    with pytest.raises(ValueError, match="0–500"):
        control.position_arguments("J1", position_mm * control.degrees_per_mm,
                                   speed_rpm=3, accel_rpm_s=0.6)


def test_linear_speed_conversion_matches_wire_precision_and_deadline():
    control = J1ScrewControl.from_root(ROOT)
    limits = control.limits(speed_mm_s=0.25, accel_mm_s2=0.05)
    assert limits.speed_rpm == 3
    assert limits.accel_rpm_s == 0.6
    assert limits.for_motion(50 * control.degrees_per_mm).motion_timeout_s == pytest.approx(207)
    with pytest.raises(ValueError):
        validate_motion_parameters(3, 0.6)
    with pytest.raises(ValueError):
        replace(limits, max_feedback_rpm=3.61)
    with pytest.raises(ValueError):
        control.limits(speed_mm_s=0.251, accel_mm_s2=0.05)


def test_real_j1_preflight_trace_preserves_stationary_admission():
    report = json.loads(TRACE.read_text(encoding="utf-8"))
    assert report["motion_commands_sent"] == 0
    assert report["properties"]["node_id"] == 1
    for row in report["frames"]:
        frame = SERIAL_CAN_FRAME.parse(bytes.fromhex(row["raw_hex"]))
        assert frame.frame_id >> 5 == 1
        if row["direction"] == "tx":
            assert frame.frame_id & 0x1F == CMD_PROPERTY_READ
    control = J1ScrewControl.from_root(ROOT)
    limits = control.limits(speed_mm_s=0.25, accel_mm_s2=0.05)
    window = VelocityWindow(limits.velocity_window_s)
    guard = SingleJointMonitor(1, limits)
    admitted = False
    for row in report["samples"]:
        sample = window.add(ObservedFeedback(
            1, control.decode_state("J1", row["pos_deg"], row["vel_rpm"]),
            row["request_s"], row["received_s"], row["traj_done"], row["axis_error"],
        ))
        if sample.window_duration_s < limits.velocity_window_s:
            continue
        if not admitted:
            if checked_speed(sample) <= limits.stopped_rpm:
                target = guard.start(sample, 3600, sample.received_s + 0.00001)
                assert target - sample.state.angle_deg == 3600
                admitted = True
        else:
            guard.validate_feedback(sample, sample.received_s + 0.00001)
    assert admitted
    with pytest.raises(MotionRejected, match="feedback_timeout"):
        guard.validate_feedback(sample, sample.received_s + 0.3)


def test_rotary_profile_remains_default_and_rejects_linear_speed():
    assert ExecutionLimits(-90, 90).axis_type == "revolute"
    with pytest.raises(ValueError):
        ExecutionLimits(-90, 90, speed_rpm=3, accel_rpm_s=0.6)


@pytest.mark.parametrize("node,axis_type", [(1, "revolute"), (2, "prismatic"), (True, "prismatic")])
def test_mismatched_motion_node_rejected_before_opening_transport(node, axis_type):
    with pytest.raises(ValueError):
        DrEmpowerSerial(str(ROOT / "setup.py"), command_node=node, axis_type=axis_type)


def test_real_j1_motion_pair_preserves_position_change_protection():
    report = json.loads(ABORT_TRACE.read_text(encoding="utf-8"))
    control = J1ScrewControl.from_root(ROOT)
    guard = SingleJointMonitor(1, ExecutionLimits(**report["limits"]))
    samples = []
    for event, raw in zip(report["trigger_pair"], report["trigger_raw_frames"], strict=True):
        frame = SERIAL_CAN_FRAME.parse(bytes.fromhex(raw["raw_hex"]))
        recorded = event["sample"]
        sample = decode_observation(control, "J1", frame.frame_id, frame.data,
                                    recorded["request_s"], recorded["received_s"])
        assert sample.state.angle_deg == pytest.approx(recorded["state"]["angle_deg"])
        samples.append(replace(sample, mean_speed_rpm=recorded["mean_speed_rpm"],
                               position_speed_rpm=recorded["position_speed_rpm"],
                               window_duration_s=recorded["window_duration_s"]))
    guard.validate_feedback(samples[0], report["trigger_pair"][0]["monotonic_s"])
    with pytest.raises(MotionRejected, match="feedback_position_jump"):
        guard.validate_feedback(samples[1], report["trigger_pair"][1]["monotonic_s"])
    assert guard.phase == "aborted"
    assert report["result"]["reason"] == guard.reason


def test_real_j1_hold_trace_preserves_external_stop_requirement():
    report = json.loads(ABORT_TRACE.read_text(encoding="utf-8"))
    limits = ExecutionLimits(**report["limits"])
    target = report["hold_requested"]["sample"]["state"]["angle_deg"]
    arrival = ArrivalWindow(target, limits)
    guard = SingleJointMonitor(1, limits)
    for event in report["hold_feedback"]:
        row = event["sample"]
        sample = ObservedFeedback(**{**row, "state": JointFeedback(**row["state"])})
        guard.validate_feedback(sample, event["monotonic_s"])
        assert not arrival.observe(sample, event["monotonic_s"])
    assert report["result"]["stop_result"] == "external_stop_required"
    assert report["result"]["stop_error"] == "hold_observation_timeout"
    assert report["result"]["monotonic_s"] - report["hold_observation_started"]["monotonic_s"] >= limits.hold_timeout_s
    assert sample.traj_done and not sample.axis_error
    assert abs(sample.state.angle_deg - target) < limits.tolerance_deg
    assert sample.mean_speed_rpm > limits.stopped_rpm
