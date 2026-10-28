import json
from pathlib import Path
import struct
import subprocess
import sys
import shutil

from construct import ConstructError
import pytest

from robot_safecontrol_moveit.drempower_can import encode_trapezoid_position
from robot_safecontrol_moveit.drempower_joint_control import JointFeedback
from robot_safecontrol_moveit.drempower_serial import SERIAL_CAN_FRAME
from robot_safecontrol_moveit.single_joint_execution import (
    ArrivalWindow, ExecutionLimits, MotionRejected, ObservedFeedback, SingleJointMonitor,
    VelocityWindow, checked_speed,
)
from robot_safecontrol_moveit.motor_stop_latch import MotorStopLatch
from robot_safecontrol_moveit.motor_motion_limits import validate_motion_parameters


TRACE = Path(__file__).with_name("data") / "motor_static_feedback_20261021.json"


def test_trapezoid_command_matches_manufacturer_layout():
    payload = encode_trapezoid_position(2, -5, speed=0.1, accel=0.1)
    assert payload == struct.pack("<fhh", -5, 10, 10)
    raw = SERIAL_CAN_FRAME.build(dict(frame_id=(2 << 5) | 0x1A, data=payload))
    assert raw.hex() == "aa0000080000005a0000a0c00a000a00"
    decoded = SERIAL_CAN_FRAME.parse(raw)
    assert decoded.frame_id == 0x5A
    assert decoded.data == payload


@pytest.mark.parametrize("raw", [bytes(16), bytes.fromhex("aa00000800000043"),
                                      bytes.fromhex("aa00000800000043000000000000000000")])
def test_serial_layout_rejects_invalid_frames(raw):
    with pytest.raises(ConstructError):
        SERIAL_CAN_FRAME.parse(raw)


@pytest.mark.parametrize("node,angle,speed,accel", [
    (0, 1, 0.1, 0.1), (2, float("nan"), 0.1, 0.1),
    (2, 1, 0, 0.1), (2, 1, 0.1, 0),
])
def test_invalid_trapezoid_rejected(node, angle, speed, accel):
    with pytest.raises(ValueError):
        encode_trapezoid_position(node, angle, speed=speed, accel=accel)


@pytest.mark.parametrize("joint", [f"J{i}" for i in range(2, 10)])
def test_recorded_stationary_motor_trace(joint):
    report = json.loads(TRACE.read_text())
    assert report["motion_commands_sent"] == 0
    node = int(joint[1:])
    limits = ExecutionLimits(-90, 90, max_feedback_rpm=0.2)
    guard = SingleJointMonitor(node, limits)
    window = VelocityWindow(limits.velocity_window_s)
    admitted = False
    raw_rejected_by_old_threshold = False
    for request, received, angle, raw_speed, done, error in report["samples"][joint]:
        raw_rejected_by_old_threshold |= abs(raw_speed) > limits.max_feedback_rpm
        sample = window.add(ObservedFeedback(node, JointFeedback(angle, raw_speed),
                                             request, received, done, error))
        if sample.window_duration_s < limits.velocity_window_s:
            continue
        if not admitted:
            if checked_speed(sample) <= limits.stopped_rpm:
                guard.start(sample, 5, received + 0.0001)
                admitted = True
        else:
            assert not guard.observe(sample, received + 0.0001)
    assert admitted
    assert raw_rejected_by_old_threshold


def test_position_speed_detects_overspeed_with_zero_reported_velocity():
    window = VelocityWindow(1)
    for index in range(60):
        t = 1 + index * 0.02
        feedback = ObservedFeedback(2, JointFeedback(index * 0.06, 0), t, t + 0.001, False, False)
        sample = window.add(feedback)
    assert sample.position_speed_rpm == pytest.approx(0.5)
    with pytest.raises(MotionRejected, match="velocity_limit"):
        SingleJointMonitor(2, ExecutionLimits(-90, 90)).start(sample, 5, t + 0.002)


def test_mean_velocity_detects_inconsistent_speed_with_fixed_position():
    window = VelocityWindow(1)
    for index in range(60):
        t = 1 + index * 0.02
        sample = window.add(ObservedFeedback(2, JointFeedback(0, 0.5), t, t + 0.001, False, False))
    with pytest.raises(MotionRejected, match="velocity_limit"):
        SingleJointMonitor(2, ExecutionLimits(-90, 90)).start(sample, 5, t + 0.002)


def observed(data):
    return ObservedFeedback(**(data | {"state": JointFeedback(**data["state"])}))


@pytest.mark.parametrize("node,target_deg", [(2, None), (4, 9)])
def test_real_motion_transaction_ranges_do_not_claim_completion(node, target_deg):
    report = json.loads(TRACE.with_name(f"motor_J{node}_abort_20261021.json").read_text())
    events = report["events"]
    guard = SingleJointMonitor(node, ExecutionLimits(**events[0]["limits"]))
    command_index = next(i for i, event in enumerate(events) if event["event"] == "command_requested")
    initial = observed(events[command_index - 1]["sample"])
    delta = 5 if target_deg is None else target_deg - initial.state.angle_deg
    guard.start(initial, delta, events[command_index - 1]["monotonic_s"])
    for event in events[command_index + 1:]:
        if event["event"] == "feedback":
            assert not guard.observe(observed(event["sample"]), event["monotonic_s"])
    assert guard.phase == "moving"
    assert events[-1]["reason"] == "feedback_position_jump"
    assert guard.target_deg > 4.9


def test_real_hold_trace_does_not_claim_stopped():
    events = json.loads(TRACE.with_name("motor_J2_abort_20261021.json").read_text())["events"]
    start = next(row for row in events if row["event"] == "hold_requested")
    limits = ExecutionLimits(**events[0]["limits"])
    arrival = ArrivalWindow(start["sample"]["state"]["angle_deg"], limits)
    for event in events:
        if event["event"] == "hold_feedback":
            assert not arrival.observe(observed(event["sample"]), event["monotonic_s"])
    assert limits.hold_timeout_s > limits.velocity_window_s + limits.settle_s
    assert limits.max_feedback_rpm == 0.2
    assert limits.position_noise_deg == 0.01


def test_arrival_requires_complete_continuous_interval():
    limits = ExecutionLimits(-90, 90)
    window = ArrivalWindow(5, limits)
    for t in (0.0, 0.1, 0.2, 0.3, 0.4):
        assert not window.observe(ObservedFeedback(2, JointFeedback(5, 0), t, t, True, False), t)
    assert window.observe(ObservedFeedback(2, JointFeedback(5, 0), 0.5, 0.5, True, False), 0.5)


def test_stop_latch_preserves_first_reason(tmp_path):
    latch = MotorStopLatch(tmp_path / "state/stop.json")
    latch.require_clear()
    latch.trip(reason="first_fault", log_path=tmp_path / "first.jsonl")
    latch.trip(reason="second_fault", log_path=tmp_path / "second.jsonl")
    with pytest.raises(RuntimeError, match="first_fault"):
        latch.require_clear()


def test_stop_latch_reports_recorded_hardware_condition(tmp_path):
    path = tmp_path / "stop.json"
    path.write_text(json.dumps(dict(reason="feedback_position_jump",
                                    hardware_blocker="J7 moves after motor power cut")))
    with pytest.raises(RuntimeError, match="J7 moves after motor power cut"):
        MotorStopLatch(path).require_clear()


def test_cli_latch_rejects_before_opening_port(tmp_path):
    root = Path(__file__).resolve().parents[1]
    (tmp_path / "config").mkdir()
    for name in ("drempower.yaml", "hardware_joint_zero.yaml"):
        shutil.copyfile(root / "config" / name, tmp_path / "config" / name)
    model_dir = tmp_path / "models/ninezzhou/urdf"
    model_dir.mkdir(parents=True)
    shutil.copyfile(root / "models/ninezzhou/urdf/ninezzhou.urdf", model_dir / "ninezzhou.urdf")
    MotorStopLatch(tmp_path / ".scratch/motor_control/stop.json").trip(
        reason="review_required", log_path=tmp_path / "aborted.jsonl")
    result = subprocess.run([
        sys.executable, "-m", "robot_safecontrol_moveit.motor_cli", "--root", str(tmp_path),
        "--port", str(tmp_path / "absent_serial_port"), "--output", str(tmp_path / "motion.jsonl"),
        "move", "--joint", "J2", "--delta-deg", "5", "--execute",
    ], capture_output=True, text=True)
    assert result.returncode != 0
    assert "review_required" in result.stderr
    assert "FileNotFoundError" not in result.stderr
    assert not (tmp_path / "motion.jsonl").exists()


@pytest.mark.parametrize("joint,target", [("J2", 10), ("J2", -10), ("J7", 4.380497336387634)])
def test_absolute_target_preview_does_not_open_serial_port(tmp_path, joint, target):
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run([
        sys.executable, "-m", "robot_safecontrol_moveit.motor_cli", "--root", str(root),
        "--port", str(tmp_path / "absent_serial_port"), "--output", str(tmp_path / "unused.jsonl"),
        "move", "--joint", joint, "--target-deg", str(target),
    ], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    assert report["delta_deg"] is None
    assert report["target_deg"] == pytest.approx(target)
    assert report["motion_timeout_basis"] == "feedback_distance"
    assert report["status"] == "plan_only"
    assert not (tmp_path / "unused.jsonl").exists()


def test_slower_motion_preview_preserves_protection_and_scales_travel_time(tmp_path):
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run([
        sys.executable, "-m", "robot_safecontrol_moveit.motor_cli", "--root", str(root),
        "--port", str(tmp_path / "absent_serial_port"), "--output", str(tmp_path / "unused.jsonl"),
        "move", "--joint", "J4", "--delta-deg", "10", "--speed-rpm", "0.03",
        "--accel-rpm-s", "0.02",
    ], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    limits = json.loads(result.stdout)["limits"]
    assert limits["speed_rpm"] == 0.03
    assert limits["accel_rpm_s"] == 0.02
    assert limits["motion_timeout_s"] == pytest.approx(10 / (6 * 0.03) + 0.03 / 0.02 + 2)
    defaults = ExecutionLimits(-90, 90)
    for name in ("max_step_deg", "feedback_timeout_s", "max_feedback_rpm", "position_noise_deg",
                 "tolerance_deg", "stopped_rpm", "settle_s"):
        assert limits[name] == getattr(defaults, name)
    assert not (tmp_path / "unused.jsonl").exists()


@pytest.mark.parametrize("delta", [-10, 10])
def test_larger_relative_motion_preview_does_not_open_serial_port(tmp_path, delta):
    root = Path(__file__).resolve().parents[1]
    output = tmp_path / "unused.jsonl"
    result = subprocess.run([
        sys.executable, "-m", "robot_safecontrol_moveit.motor_cli", "--root", str(root),
        "--port", str(tmp_path / "absent_serial_port"), "--output", str(output),
        "move", "--joint", "J2", "--delta-deg", str(delta),
    ], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    assert report["limits"]["max_step_deg"] is None
    assert report["limits"]["max_feedback_rpm"] * 6 == pytest.approx(2.5)
    assert report["limits"]["motion_timeout_s"] == pytest.approx(abs(delta) / 0.3 + 3)
    assert report["motion_timeout_basis"] == "delta_deg"
    assert not output.exists()


@pytest.mark.parametrize("option,value,message", [
    ("--delta-deg", "0", "相对增量必须为有限非零数值"),
    ("--delta-deg", "nan", "相对增量必须为有限非零数值"),
    ("--delta-deg", "181", "相对增量超过 URDF 关节总行程"),
    ("--target-deg", "91", "绝对目标必须位于 URDF 关节范围内"),
])
def test_invalid_motion_rejected_before_connection(tmp_path, option, value, message):
    root = Path(__file__).resolve().parents[1]
    output = tmp_path / "unused.jsonl"
    result = subprocess.run([
        sys.executable, "-m", "robot_safecontrol_moveit.motor_cli", "--root", str(root),
        "--port", str(tmp_path / "absent_serial_port"), "--output", str(output),
        "move", "--joint", "J2", option, value, "--execute",
    ], capture_output=True, text=True)
    assert result.returncode != 0
    assert message in result.stderr
    assert "FileNotFoundError" not in result.stderr
    assert not output.exists()


@pytest.mark.parametrize("option,value,field", [
    ("--speed-rpm", "0.029", "speed_rpm"),
    ("--speed-rpm", "0.039", "speed_rpm"),
    ("--speed-rpm", "0.029999999999999995", "speed_rpm"),
    ("--speed-rpm", str(2.5 / 6), "speed_rpm"),
    ("--accel-rpm-s", "0.019", "accel_rpm_s"),
    ("--accel-rpm-s", "0.049999999999999996", "accel_rpm_s"),
])
def test_unrepresentable_motion_parameters_rejected_before_connection(tmp_path, option, value, field):
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run([
        sys.executable, "-m", "robot_safecontrol_moveit.motor_cli", "--root", str(root),
        "--port", str(tmp_path / "absent_serial_port"), "--output", str(tmp_path / "unused.jsonl"),
        "move", "--joint", "J4", "--delta-deg", "5", option, value, "--execute",
    ], capture_output=True, text=True)
    assert result.returncode != 0
    assert f"{field} 必须为 0.01 的整数倍" in result.stderr
    assert "FileNotFoundError" not in result.stderr
    assert not (tmp_path / "unused.jsonl").exists()


@pytest.mark.parametrize("speed,accel", [
    (0.03, 0.01), (0.05, 0.05), (0.07, 0.07), (0.1, 0.1), (0.41, 0.05),
])
def test_motion_preview_limits_match_wire_parameters(tmp_path, speed, accel):
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run([
        sys.executable, "-m", "robot_safecontrol_moveit.motor_cli", "--root", str(root),
        "--port", str(tmp_path / "absent_serial_port"), "--output", str(tmp_path / "unused.jsonl"),
        "move", "--joint", "J4", "--delta-deg", "5", "--speed-rpm", str(speed),
        "--accel-rpm-s", str(accel),
    ], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    limits = json.loads(result.stdout)["limits"]
    payload = encode_trapezoid_position(4, 5, speed=limits["speed_rpm"], accel=limits["accel_rpm_s"])
    _, encoded_speed, encoded_accel = struct.unpack("<fhh", payload)
    assert encoded_speed / 100 == limits["speed_rpm"] == speed
    assert encoded_accel / 100 == limits["accel_rpm_s"] == accel
    travel_s = 5 / (6 * (encoded_speed / 100)) + encoded_speed / encoded_accel
    assert limits["motion_timeout_s"] >= travel_s + 2
    assert not (tmp_path / "unused.jsonl").exists()


def test_maximum_representable_speed_uses_same_parameter_validation():
    validate_motion_parameters(0.41, 0.05)
    limits = ExecutionLimits(-180, 180, speed_rpm=0.41)
    payload = encode_trapezoid_position(5, 90, speed=limits.speed_rpm, accel=limits.accel_rpm_s)
    _, speed, _ = struct.unpack("<fhh", payload)
    assert speed == 41
    assert speed / 100 * 6 == pytest.approx(2.46)
    with pytest.raises(ValueError, match="speed_rpm"):
        validate_motion_parameters(0.42, 0.05)
    with pytest.raises(ValueError, match="0.01 的整数倍"):
        validate_motion_parameters(2.5 / 6, 0.05)


def test_speed_above_feedback_limit_is_rejected_before_connection(tmp_path):
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run([
        sys.executable, "-m", "robot_safecontrol_moveit.motor_cli", "--root", str(root),
        "--port", str(tmp_path / "absent_serial_port"), "--output", str(tmp_path / "unused.jsonl"),
        "move", "--joint", "J5", "--target-deg", "90", "--speed-rpm", "0.42", "--execute",
    ], capture_output=True, text=True)
    assert result.returncode != 0
    assert "speed_rpm 必须在" in result.stderr
    assert "FileNotFoundError" not in result.stderr
