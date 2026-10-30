import csv
import io
import json
import os
import struct
import subprocess
import sys
from dataclasses import replace
from pathlib import Path
from zipfile import ZipFile

import numpy as np
import pytest
from test_joint_table_execution import MuJoCoObservations, ready
from work.nineaxis_model import NineaxisModel

from robot_safecontrol_moveit.joint_positioning import (
    PositioningOperator,
    positioning_limits,
    single_target_table,
)
from robot_safecontrol_moveit.joint_table import TABLE_COLUMNS, JointTable
from robot_safecontrol_moveit.joint_table_execution import JointTableExecutor
from robot_safecontrol_moveit.drempower_joint_control import JointFeedback
from robot_safecontrol_moveit.robot_spec import DEFAULT_JOINT_NAMES
from robot_safecontrol_moveit.single_joint_execution import (
    ExecutionLimits, MotionRejected, ObservedFeedback, SingleJointMonitor, VelocityWindow,
)

ROOT = Path(__file__).resolve().parents[1]
SPEED = dict(zip(DEFAULT_JOINT_NAMES, [.25, *([2.46] * 8)]))
ACCELERATION = dict(zip(DEFAULT_JOINT_NAMES, [.05, *([.3] * 8)]))
TARGET = dict.fromkeys(DEFAULT_JOINT_NAMES, 0.)


@pytest.fixture(scope="module")
def model():
    return NineaxisModel.load(root=ROOT)


def positioning_session(model, path):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(TABLE_COLUMNS)
        writer.writerows([["a", *([0.] * 9)], ["b", .1, *([.4] * 8)]])
    executor = JointTableExecutor(JointTable.load(path, model), model, measurement_kind="positioning",
                                  task_id="manual-positioning-test", feedback_source="simulation_feedback",
                                  manual_positioning=True, measurement_timeout_s=.1)
    return executor, MuJoCoObservations(executor)


def test_project_units_effective_protocol_values_and_target_identity(model):
    target = {**TARGET, "J1": 10., "J2": -5.}
    table = single_target_table(target, model)
    np.testing.assert_allclose(table.rows[0].q_si, [.01, np.deg2rad(-5), *([0] * 7)])
    assert single_target_table(dict(reversed(list(target.items()))), model).source_sha256 == table.source_sha256
    limits = positioning_limits(ROOT, {**SPEED, "J2": 2.}, ACCELERATION)
    preview = table.preview(model, measurement_kind="positioning", execution_limits=limits)
    assert preview["rows"][0]["device_commands"][0]["arguments"]["angle"] == -720
    assert preview["rows"][0]["device_commands"][1]["arguments"]["speed"] == .33
    assert all(row["arguments"]["mode"] == 1 for row in preview["rows"][0]["device_commands"])
    assert limits["J1"].speed_rpm == 3
    assert limits["J1"].accel_rpm_s == .6
    assert limits["J2"].speed_rpm == .33
    assert limits["J9"].accel_rpm_s == .05
    assert limits["J1"].tolerance_deg == 72.
    assert all(limits[joint].tolerance_deg == .1 for joint in DEFAULT_JOINT_NAMES[1:])


def test_actual_native_j1_start_feedback_uses_linear_position_tolerance():
    with ZipFile(ROOT / "tests/data/direct-position-01_20261030.zip") as archive:
        events = [json.loads(line) for line in archive.read("direct-position-01.jsonl").splitlines()]
        report = json.loads(archive.read("direct-position-01.report.json"))
    original = ExecutionLimits(**report["limits"]["J1"])
    corrected = replace(original, tolerance_deg=positioning_limits(ROOT, SPEED, ACCELERATION)["J1"].tolerance_deg)
    guards = [SingleJointMonitor(1, limits) for limits in (original, corrected)]
    window = VelocityWindow(original.velocity_window_s, original.stationary_window_s)
    moving = False
    rejection = None
    for event in events:
        if event["event"] == "command_requested" and event["joint"] == "J1":
            for guard in guards:
                guard.start_absolute(sample, 144., event["monotonic_s"])
            moving = True
        if event["event"] != "feedback":
            continue
        value = event["samples"]["J1"]
        sample = window.add(ObservedFeedback(**{**value, "state": JointFeedback(**value["state"])}))
        if not moving:
            continue
        guards[1].observe(sample, event["monotonic_s"])
        try:
            guards[0].observe(sample, event["monotonic_s"])
        except MotionRejected as error:
            rejection = str(error)
            break
    assert rejection == "motion_outside_requested_interval"
    assert guards[1].phase == "moving"
    assert guards[1].start_deg is not None
    reverse_mm = (guards[1].start_deg - sample.state.angle_deg) / 72
    assert original.tolerance_deg / 72 < reverse_mm < .002
    assert reverse_mm < corrected.tolerance_deg / 72
    with pytest.raises(ValueError, match="到位误差"):
        replace(corrected, tolerance_deg=72.01)


def test_protocol_integer_steps_preserve_editable_decimal_speed(model):
    limits = positioning_limits(ROOT, {**SPEED, "J2": 1.74}, ACCELERATION)
    table = single_target_table(TARGET, model)
    command = table.preview(model, measurement_kind="positioning", execution_limits=limits)["rows"][0][
        "device_commands"][1]
    assert limits["J2"].speed_rpm == .29
    assert struct.unpack("<fhh", bytes.fromhex(command["payload_hex"]))[1:] == (29, 5)


@pytest.mark.parametrize("joint,value", [("J1", -1), ("J1", 501), ("J2", float("nan")), ("J9", True)])
def test_invalid_editable_targets_fail_at_input(model, joint, value):
    with pytest.raises(ValueError):
        single_target_table({**TARGET, joint: value}, model)


@pytest.mark.parametrize("value", [0, -1, float("nan"), True, 1000])
def test_invalid_editable_motion_parameters_fail_at_input(value):
    with pytest.raises(ValueError):
        positioning_limits(ROOT, {**SPEED, "J2": value}, ACCELERATION)


def test_real_pipe_enter_wait_and_two_mujoco_groups(model, tmp_path):
    executor, plant = positioning_session(model, tmp_path / "targets.csv")
    read_fd, write_fd = os.pipe()
    output = io.StringIO()
    with os.fdopen(read_fd, "r") as stream, os.fdopen(write_fd, "wb", buffering=0) as keys:
        operator = PositioningOperator(stream, output=output, start_immediately=False)
        keys.write(b"\n")
        for _ in range(110):
            plant.observe()
            assert operator.step(executor, now_s=plant.data.time) == []
        assert executor.phase == "ready"
        assert not executor.attempts
        keys.write(b"\n\n")
        first = operator.step(executor, now_s=plant.data.time)
        assert len(first) == 9
        keys.write(b"\n")
        for _ in range(100):
            plant.observe()
            assert operator.step(executor, now_s=plant.data.time) == []
        assert executor.phase == "holding_position"
        assert executor.index == 0
        assert len(executor.positioning_records) == 1
        assert not executor.records
        # 等待超过测量期限，真实动力学反馈继续更新，位置目标不变。
        for _ in range(400):
            plant.observe()
            assert operator.step(executor, now_s=plant.data.time) == []
        keys.write(b"\n")
        second = operator.step(executor, now_s=plant.data.time)
        assert len(second) == 9
        assert executor.index == 1
        target = np.asarray(executor.table.rows[1].q_si)
        for index in range(1000):
            fraction = min((index + 1) / 400, 1.)
            plant.observe(target=target * fraction, trajectory_done=fraction == 1.)
            assert operator.step(executor, now_s=plant.data.time) == []
            if executor.phase == "holding_position":
                break
        assert executor.phase == "holding_position"
        assert np.all(plant.max_motion > 0)
        assert len(executor.positioning_records) == 2
        keys.write(b"\n")
        assert operator.step(executor, now_s=plant.data.time) == []
        assert executor.phase == "holding_position"
        keys.write(b"q\n")
        assert operator.step(executor, now_s=plant.data.time) == []
        assert executor.phase == "completed"
        assert executor.termination["disable_command_sent"] is False
    assert "最后一组保持中" in output.getvalue()
    assert "下一组 b" in output.getvalue()
    assert not executor.records
    assert all(attempt["status"] == "completed" for attempt in executor.attempts)
    assert executor.report()["all_positioning_targets_completed"] is True
    assert executor.report()["return_to_zero"] is None
    assert executor.termination["returned_to_zero"] is False


def test_confirmed_return_from_nonzero_pose_and_continuous_zero_monitoring(model, tmp_path):
    executor, plant = positioning_session(model, tmp_path / "targets.csv")
    ready(executor, plant)
    executor.begin(now_s=plant.data.time)
    for _ in range(100):
        plant.observe()
    executor.advance_positioning(now_s=plant.data.time)
    executor.begin(now_s=plant.data.time)
    read_fd, write_fd = os.pipe()
    output = io.StringIO()
    with os.fdopen(read_fd, "r") as stream, os.fdopen(write_fd, "wb", buffering=0) as keys:
        operator = PositioningOperator(stream, output=output, start_immediately=False)
        keys.write(b"r\n")
        target = np.asarray(executor.table.rows[1].q_si)
        for index in range(1000):
            fraction = min((index + 1) / 400, 1.)
            plant.observe(target=target * fraction, trajectory_done=fraction == 1.)
            assert operator.step(executor, now_s=plant.data.time) == []
            if executor.phase == "holding_position":
                break
        assert executor.phase == "holding_position"
        assert executor.return_to_zero_attempt is None
        assert np.all(plant.data.qpos[plant.qpos] > 0)
        assert len(executor.attempts) == 2
        keys.write(b"r\n")
        commands = operator.step(executor, now_s=plant.data.time)
        assert len(commands) == 9
        for command in commands:
            joint = command["joint"]
            assert command["arguments"] == executor.control.position_arguments(joint, 0., executor.limits[joint])
        assert executor.phase == "moving"
        assert executor.current_attempt() is executor.return_to_zero_attempt
        assert executor.attempts[1]["status"] == "completed"
        keys.write(b"\nr\n")
        start = plant.data.qpos[plant.qpos].copy()
        for index in range(1000):
            fraction = min((index + 1) / 400, 1.)
            plant.observe(target=start * (1 - fraction), trajectory_done=fraction == 1.)
            assert operator.step(executor, now_s=plant.data.time) == []
            if executor.phase == "holding_zero":
                break
        assert executor.phase == "holding_zero"
        assert executor.return_to_zero_attempt["status"] == "holding_zero"
        for joint, actual in zip(DEFAULT_JOINT_NAMES, plant.data.qpos[plant.qpos]):
            assert abs(executor.control.target_deg(joint, actual)) <= executor.limits[joint].tolerance_deg
        keys.write(b"\nr\n")
        for _ in range(400):
            plant.observe()
            assert operator.step(executor, now_s=plant.data.time) == []
        assert executor.phase == "holding_zero"
        assert len(executor.attempts) == 3
        assert len(executor.positioning_records) == 2
        assert executor.index == 1
        assert len(executor.table.rows) == 2
        keys.write(b"q\n")
        assert operator.step(executor, now_s=plant.data.time) == []
    report = executor.report()
    assert report["phase"] == "completed"
    assert report["return_to_zero"]["status"] == "completed"
    assert report["return_to_zero"]["target_mm_deg"] == [0.] * 9
    assert report["return_to_zero"]["feedback_stopped"] is True
    assert report["feedback_source"] == "simulation_feedback"
    assert report["all_positioning_targets_completed"] is True
    assert report["termination"]["returned_to_zero"] is True
    assert report["termination"]["disable_command_sent"] is False
    assert not report["records"]
    assert "九轴回零完成" in output.getvalue()
    executor.write_report(tmp_path / "report.json")
    assert json.loads((tmp_path / "report.json").read_text())["return_to_zero"]["motion_kind"] == "return_to_zero"


@pytest.mark.parametrize("completed_count", [0, 1])
def test_return_before_remaining_table_targets_does_not_resume_or_change_counts(model, tmp_path, completed_count):
    executor, plant = positioning_session(model, tmp_path / "targets.csv")
    ready(executor, plant)
    if completed_count:
        executor.begin(now_s=plant.data.time)
        for _ in range(100):
            plant.observe()
    read_fd, write_fd = os.pipe()
    with os.fdopen(read_fd, "r") as stream, os.fdopen(write_fd, "wb", buffering=0) as keys:
        operator = PositioningOperator(stream, output=io.StringIO(), start_immediately=False)
        keys.write(b"r\n")
        assert operator.step(executor, now_s=plant.data.time) == []
        assert executor.return_to_zero_attempt is None
        keys.write(b"r\n")
        assert len(operator.step(executor, now_s=plant.data.time)) == 9
        for _ in range(100):
            plant.observe()
            assert operator.step(executor, now_s=plant.data.time) == []
        assert executor.phase == "holding_zero"
        keys.write(b"\nr\n")
        assert operator.step(executor, now_s=plant.data.time) == []
        assert len(executor.attempts) == completed_count + 1
        keys.write(b"q\n")
        assert operator.step(executor, now_s=plant.data.time) == []
    report = executor.report()
    assert report["phase"] == "completed"
    assert report["positioning_completed_count"] == completed_count
    assert report["positioning_total_count"] == 2
    assert report["all_positioning_targets_completed"] is False
    assert executor.index == 0


@pytest.mark.parametrize("phase", ["moving", "holding_zero"])
def test_return_to_zero_keeps_feedback_timeout_and_fault_attempt(model, tmp_path, phase):
    executor, plant = positioning_session(model, tmp_path / "targets.csv")
    ready(executor, plant)
    executor.begin_return_to_zero(now_s=plant.data.time)
    if phase == "holding_zero":
        for _ in range(100):
            plant.observe()
    assert executor.phase == phase
    with pytest.raises(MotionRejected, match="feedback_timeout"):
        executor.check_time(plant.data.time + .201)
    assert executor.phase == "aborted"
    assert executor.report()["return_to_zero"]["status"] == "aborted"
    assert executor.report()["return_to_zero"]["rejection_reason"] == executor.reason
    assert not executor.positioning_records
    with pytest.raises(ValueError, match="正常停车"):
        executor.begin_return_to_zero(now_s=plant.data.time)


def test_zero_hold_rejects_physical_motion(model, tmp_path):
    executor, plant = positioning_session(model, tmp_path / "targets.csv")
    ready(executor, plant)
    executor.begin_return_to_zero(now_s=plant.data.time)
    for _ in range(100):
        plant.observe()
    assert executor.phase == "holding_zero"
    target = np.zeros(9)
    target[1] = np.deg2rad(.3)
    with pytest.raises(MotionRejected):
        for _ in range(100):
            plant.observe(target=target)
    assert executor.phase == "aborted"
    assert executor.return_to_zero_attempt["status"] == "aborted"
    assert not executor.positioning_records


def test_return_confirmation_cannot_use_stale_feedback(model, tmp_path):
    executor, plant = positioning_session(model, tmp_path / "targets.csv")
    ready(executor, plant)
    read_fd, write_fd = os.pipe()
    with os.fdopen(read_fd, "r") as stream, os.fdopen(write_fd, "wb", buffering=0) as keys:
        operator = PositioningOperator(stream, output=io.StringIO(), start_immediately=False)
        operator.step(executor, now_s=plant.data.time)
        keys.write(b"r\n")
        with pytest.raises(MotionRejected, match="feedback_timeout"):
            operator.step(executor, now_s=plant.data.time + .201)
    assert executor.phase == "aborted"
    assert not executor.attempts


def test_q_before_next_group_preserves_completed_pose_and_reports_remaining_targets(model, tmp_path):
    executor, plant = positioning_session(model, tmp_path / "targets.csv")
    ready(executor, plant)
    read_fd, write_fd = os.pipe()
    with os.fdopen(read_fd, "r") as stream, os.fdopen(write_fd, "wb", buffering=0) as keys:
        operator = PositioningOperator(stream, output=io.StringIO(), start_immediately=True)
        assert len(operator.step(executor, now_s=plant.data.time)) == 9
        for _ in range(100):
            plant.observe()
            assert operator.step(executor, now_s=plant.data.time) == []
        keys.write(b"q\n")
        assert operator.step(executor, now_s=plant.data.time) == []
    report = executor.report()
    assert report["phase"] == "completed"
    assert report["positioning_completed_count"] == 1
    assert report["positioning_total_count"] == 2
    assert report["all_positioning_targets_completed"] is False
    assert len(executor.attempts) == 1


def test_partial_terminal_line_never_blocks_feedback_and_q_interrupts_motion(model, tmp_path):
    executor, plant = positioning_session(model, tmp_path / "targets.csv")
    ready(executor, plant)
    read_fd, write_fd = os.pipe()
    with os.fdopen(read_fd, "r") as stream, os.fdopen(write_fd, "wb", buffering=0) as keys:
        operator = PositioningOperator(stream, output=io.StringIO(), start_immediately=True)
        assert len(operator.step(executor, now_s=plant.data.time)) == 9
        keys.write(b"q")
        plant.observe()
        assert operator.step(executor, now_s=plant.data.time) == []
        assert executor.phase == "moving"
        keys.write(b"\n")
        with pytest.raises(KeyboardInterrupt, match="operator_stop"):
            operator.step(executor, now_s=plant.data.time)


def test_holding_position_still_rejects_missing_feedback(model, tmp_path):
    executor, plant = positioning_session(model, tmp_path / "targets.csv")
    ready(executor, plant)
    executor.begin(now_s=plant.data.time)
    for _ in range(100):
        plant.observe()
    assert executor.phase == "holding_position"
    for _ in range(12):
        plant.step()
    with pytest.raises(MotionRejected, match="feedback_timeout"):
        executor.check_time(plant.data.time)
    assert executor.phase == "aborted"
    assert len(executor.positioning_records) == 1
    assert not executor.records


def test_holding_position_rejects_physical_motion(model, tmp_path):
    executor, plant = positioning_session(model, tmp_path / "targets.csv")
    ready(executor, plant)
    executor.begin(now_s=plant.data.time)
    for _ in range(100):
        plant.observe()
    target = np.zeros(9)
    target[1] = np.deg2rad(.3)
    with pytest.raises(MotionRejected):
        for _ in range(100):
            plant.observe(target=target)
    assert executor.phase == "aborted"
    assert executor.index == 0


def test_manual_positioning_cannot_repurpose_measurement_session(model, tmp_path):
    table = single_target_table(TARGET, model)
    with pytest.raises(ValueError, match="手动定位"):
        JointTableExecutor(table, model, measurement_kind="camera", task_id="test",
                           feedback_source="simulation_feedback", manual_positioning=True)
    with pytest.raises(ValueError, match="仅支持一组"):
        JointTableExecutor(replace(table, rows=(table.rows[0], replace(table.rows[0], pose_id="second"))),
                           model, measurement_kind="positioning",
                           task_id="test", feedback_source="simulation_feedback")


def test_return_to_zero_requires_manual_stopped_session(model, tmp_path):
    table = single_target_table(TARGET, model)
    for kind in ("camera", "positioning"):
        executor = JointTableExecutor(table, model, measurement_kind=kind, task_id="test",
                                     feedback_source="simulation_feedback")
        plant = MuJoCoObservations(executor)
        ready(executor, plant)
        with pytest.raises(ValueError, match="正常停车"):
            executor.begin_return_to_zero(now_s=plant.data.time)
        assert not executor.attempts
    executor, plant = positioning_session(model, tmp_path / "targets.csv")
    with pytest.raises(ValueError, match="正常停车"):
        executor.begin_return_to_zero(now_s=plant.data.time)
    ready(executor, plant)
    executor.begin(now_s=plant.data.time)
    with pytest.raises(ValueError, match="正常停车"):
        executor.begin_return_to_zero(now_s=plant.data.time)
    assert len(executor.attempts) == 1
    assert executor.return_to_zero_attempt is None


def test_editable_script_previews_both_input_modes_and_rejects_missing_policy(tmp_path):
    script = ROOT / "scripts/joint_positioning.py"
    for extra in ([], ["--csv", "config/calibration_joint_table.example.csv"]):
        result = subprocess.run([sys.executable, str(script), "--preview", *extra],
                                cwd=ROOT, capture_output=True, text=True, check=True)
        report = json.loads(result.stdout)
        assert report["hardware_executable"] is False
        assert report["continue_monitoring_after_last_pose"] is True
        assert len(report["rows"]) == (2 if extra else 1)
        assert report["parameters"]["J1"]["effective_speed"] == .25
        assert report["return_to_zero_requires_confirmation"] is True
        assert report["return_to_zero_command"] == "r"
        assert report["continue_monitoring_after_return_to_zero"] is True
        assert report["return_to_zero"]["target_mm_deg"] == [0.] * 9
        assert report["return_to_zero"]["pose_id"] == "return_to_zero"
        for zero, target in zip(report["return_to_zero"]["device_commands"], report["rows"][0]["device_commands"]):
            assert zero["arguments"]["speed"] == target["arguments"]["speed"]
            assert zero["arguments"]["param"] == target["arguments"]["param"]
    log = tmp_path / "not-created/run.jsonl"
    result = subprocess.run([sys.executable, str(script), "--port", "/absent", "--output", str(log),
                             "--experiment-policy", str(tmp_path / "missing-policy.json")],
                            cwd=ROOT, capture_output=True, text=True, check=False)
    assert result.returncode != 0
    assert "missing-policy.json" in result.stderr
    assert not log.parent.exists()


def test_positioning_import_does_not_load_serial_or_open_devices():
    subprocess.run([sys.executable, "-c", ("import sys; import robot_safecontrol_moveit.joint_positioning; "
                    "assert 'serial' not in sys.modules; assert 'construct' not in sys.modules")],
                   cwd=ROOT, capture_output=True, text=True, check=True)


def test_explicit_nine_joint_values_preview_native_position_commands():
    target = [2., 1., -1., 1., -1., 1., 1., -1., 1.]
    result = subprocess.run([
        sys.executable, str(ROOT / "scripts/joint_positioning.py"), "--preview",
        "--joints", *map(str, target), "--port", "/absent"],
        cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    assert report["input_kind"] == "joints"
    assert report["rows"][0]["target_mm_deg"] == pytest.approx(target)
    assert report["first_pose_requires_enter"] is False
    commands = report["rows"][0]["device_commands"]
    assert len(commands) == 9
    assert all(row["arguments"]["mode"] == 1 for row in commands)
    assert commands[0]["arguments"]["angle"] == -144.


@pytest.mark.parametrize("arguments", [
    ["--joints", *(["0"] * 8)],
    ["--joints", *(["0"] * 10)],
    ["--joints", "nan", *(["0"] * 8)],
    ["--joints", *(["0"] * 9), "--csv", "config/calibration_joint_table.example.csv"],
])
def test_explicit_joint_input_rejects_invalid_count_values_or_csv_conflict(arguments, tmp_path):
    output = tmp_path / "not-created.json"
    result = subprocess.run([
        sys.executable, str(ROOT / "scripts/joint_positioning.py"), "--preview",
        "--port", "/absent", "--output", str(output), *arguments],
        cwd=ROOT, capture_output=True, text=True)
    assert result.returncode != 0
    assert not output.exists()


def test_real_recording_preview_has_common_progress_and_complete_reference(tmp_path):
    archive_path = ROOT / "tests/data/j1_bounded_counter_experiments_20261030.zip"
    with ZipFile(archive_path) as archive:
        source = archive.read("final-nineaxis-readonly.json")
    recording = tmp_path / "current-readonly.json"
    recording.write_bytes(source)
    output = tmp_path / "synchronized-preview.json"
    result = subprocess.run([
        sys.executable, str(ROOT / "scripts/joint_positioning.py"), "--preview",
        "--start-readonly", str(recording), "--output", str(output),
    ], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    report = json.loads(output.read_bytes())
    plan = report["synchronized_plan"]
    assert plan["status"] == "planned_reference_only"
    assert plan["duration_s"] > 200
    assert plan["hardware_executable"] is False
    assert plan["coordinate_rules_verified"] is False
    assert plan["common_progress_verified"] is True
    assert plan["motion_commands_sent"] == 0
    assert plan["parameter_writes_sent"] == 0
    rows = plan["rows"]
    assert len(rows) == 1
    samples = rows[0]["samples"]
    assert len(samples) >= 129
    start = np.asarray(rows[0]["start_mm_deg"])
    goal = np.array([50., 10., -20., 30., -30., 60., 10., 20., 30.])
    original = json.loads(source)
    original_last = {row["joint"]: row for row in original["samples"][-9:]}
    expected = [original_last[joint]["state"]["angle_deg"] / (72 if joint == "J1" else 1)
                for joint in DEFAULT_JOINT_NAMES]
    np.testing.assert_allclose(start, expected, atol=1e-10)
    np.testing.assert_allclose(samples[0]["position_mm_deg"], start, atol=1e-10)
    np.testing.assert_allclose(samples[-1]["position_mm_deg"], goal, atol=1e-9)
    previous = -1.
    moving = goal != start
    for sample in samples:
        assert sample["time_s"] > previous
        previous = sample["time_s"]
        fractions = (np.asarray(sample["position_mm_deg"])[moving] - start[moving]) / (goal - start)[moving]
        np.testing.assert_allclose(fractions, sample["progress"], atol=1e-9)
        assert -.000000001 <= sample["progress"] <= 1.000000001
        assert np.all(np.abs(sample["velocity_mm_deg_s"]) <= np.array([.25, *([2.46] * 8)]) + 1e-10)
        assert np.all(np.abs(sample["acceleration_mm_deg_s2"]) <= np.array([.05, *([.3] * 8)]) + 1e-10)
        assert np.asarray(sample["tcp_position_mm"]).shape == (3,)
    for endpoint in (samples[0], samples[-1]):
        np.testing.assert_allclose(endpoint["velocity_mm_deg_s"], 0., atol=1e-10)
        np.testing.assert_allclose(endpoint["acceleration_mm_deg_s2"], 0., atol=1e-10)
    assert rows[0]["reference_profiles"]
    assert plan["start_evidence"]["device_sample_time_known"] is False
    assert report["hardware_executable"] is False


def test_common_preview_preserves_zero_displacement_axes_and_static_rows(tmp_path):
    with ZipFile(ROOT / "tests/data/j1_bounded_counter_experiments_20261030.zip") as archive:
        source = archive.read("final-nineaxis-readonly.json")
    recording = tmp_path / "recording.json"
    recording.write_bytes(source)
    last = {row["joint"]: row["state"]["angle_deg"] for row in json.loads(source)["samples"][-9:]}
    start = [last[joint] / (72 if joint == "J1" else 1) for joint in DEFAULT_JOINT_NAMES]
    target = [start[0] + 5, start[1] - 1, *start[2:]]
    table_path = tmp_path / "targets.csv"
    with table_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(TABLE_COLUMNS)
        writer.writerows([["mixed", *target], ["static", *target]])
    result = subprocess.run([
        sys.executable, str(ROOT / "scripts/joint_positioning.py"), "--preview", "--csv", str(table_path),
        "--start-readonly", str(recording),
    ], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)["synchronized_plan"]
    moving, stationary = report["rows"]
    for sample in moving["samples"]:
        np.testing.assert_allclose(sample["position_mm_deg"][2:], start[2:], atol=1e-12)
        np.testing.assert_array_equal(sample["velocity_mm_deg_s"][2:], 0.)
        np.testing.assert_array_equal(sample["acceleration_mm_deg_s2"][2:], 0.)
        assert sample["velocity_mm_deg_s"][0] >= -1e-12
        assert sample["velocity_mm_deg_s"][1] <= 1e-12
    for sample in stationary["samples"]:
        np.testing.assert_allclose(sample["position_mm_deg"], target, atol=1e-12)
        np.testing.assert_allclose(sample["velocity_mm_deg_s"], 0., atol=1e-12)
        np.testing.assert_allclose(sample["acceleration_mm_deg_s2"], 0., atol=1e-12)


def test_recorded_out_of_range_start_is_rejected_without_clipping_or_creating_output(tmp_path):
    with ZipFile(ROOT / "tests/data/motor_positioning_research_20261030.zip") as archive:
        source = archive.read("nineaxis-current-readonly.json")
    recording = tmp_path / "recording.json"
    recording.write_bytes(source)
    output = tmp_path / "absent/plan.json"
    result = subprocess.run([
        sys.executable, str(ROOT / "scripts/joint_positioning.py"), "--preview",
        "--start-readonly", str(recording), "--output", str(output),
    ], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode != 0
    assert "原始反馈起点超出共同模型范围" in result.stderr
    assert not output.parent.exists()


def test_preview_certifies_full_route_and_rejects_collision_between_clear_endpoints(model, tmp_path):
    from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig
    from work.obb_collision_model import OBB_LINK_NAMES, OBB_LOCAL_CENTERS_M
    from work.static_collision import StaticGeometry

    with ZipFile(ROOT / "tests/data/j1_bounded_counter_experiments_20261030.zip") as archive:
        source = archive.read("final-nineaxis-readonly.json")
    recording = tmp_path / "recording.json"
    recording.write_bytes(source)
    last = {row["joint"]: row["state"]["angle_deg"] for row in json.loads(source)["samples"][-9:]}
    start = [last[joint] / (72 if joint == "J1" else 1) for joint in DEFAULT_JOINT_NAMES]
    target = [start[0], 30., *start[2:]]
    table_path = tmp_path / "target.csv"
    with table_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(TABLE_COLUMNS)
        writer.writerow(["swing", *target])
    scene = ExperimentConfig.load(ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT).scene()
    scene.pop("config_sha256")
    scene["workpiece"]["center_m"] = [10., 10., 10.]
    scene["table"]["center_m"] = [10., -10., 10.]
    scene_path = tmp_path / "declared-simulation-scene.json"
    from work.robot_units import joints_mm_deg_to_si

    first, last_si = joints_mm_deg_to_si(start), joints_mm_deg_to_si(target)
    midpoint = (first + last_si) / 2
    link_pose = model.link_poses(midpoint)["Link9"]
    center = link_pose[:3, :3] @ OBB_LOCAL_CENTERS_M[OBB_LINK_NAMES.index("Link9")] + link_pose[:3, 3]
    for blocked in (False, True):
        scene["obstacles"] = ([{"name": "middle_obstacle", "center_m": center.tolist(), "radius_m": .06}]
                              if blocked else [])
        geometry = StaticGeometry(scene)
        for q in (first, last_si):
            assert all(row.distance.valid and row.distance.distance_m >= row.required_m
                       for row in geometry.query(q, gradients=False))
        if blocked:
            assert any(not row.distance.valid or row.distance.distance_m < row.required_m
                       for row in geometry.query(midpoint, gradients=False))
        scene_path.write_text(json.dumps(scene), encoding="utf-8")
        output = tmp_path / f"route-{blocked}.json"
        result = subprocess.run([
            sys.executable, str(ROOT / "scripts/joint_positioning.py"), "--preview", "--csv", str(table_path),
            "--start-readonly", str(recording), "--scene-config", str(scene_path), "--output", str(output),
        ], cwd=ROOT, capture_output=True, text=True)
        assert result.returncode == (2 if blocked else 0), result.stderr
        report = json.loads(output.read_bytes())
        plan = report["synchronized_plan"]
        collision = plan["rows"][0]["collision"]
        assert collision["accepted"] is (not blocked)
        assert collision["checked_intervals"] > 0
        assert collision["policy"] == {"minimum_interval_s": 1e-5, "maximum_intervals": 4096}
        assert len(collision["checker_sha256"]) == 64
        assert plan["scene_identity"]["physical_scene_verified"] is False
        assert plan["motion_commands_sent"] == 0
        assert report["hardware_executable"] is False
        if blocked:
            assert "middle_obstacle" in collision["pair"]
        else:
            assert collision["status"] == "proven"
            assert collision["minimum_proven_margin_m"] > 0


def test_preview_rejects_incomplete_original_capture_and_execution_option_misuse(tmp_path):
    with ZipFile(ROOT / "tests/data/motor_positioning_research_20261030.zip") as archive:
        source = archive.read("nineaxis-operating-readonly.json")
    recording = tmp_path / "incomplete.json"
    recording.write_bytes(source)
    output = tmp_path / "absent/output.json"
    result = subprocess.run([
        sys.executable, str(ROOT / "scripts/joint_positioning.py"), "--preview",
        "--start-readonly", str(recording), "--output", str(output),
    ], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode != 0
    assert "定位起点需要完整的原始只读记录" in result.stderr
    assert not output.parent.exists()
    result = subprocess.run([
        sys.executable, str(ROOT / "scripts/joint_positioning.py"), "--port", "/absent/device",
        "--start-readonly", str(recording), "--output", str(output),
    ], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 2
    assert "需要 --preview" in result.stderr
    assert not output.parent.exists()


def test_common_reference_reaches_target_and_holds_five_seconds_in_mujoco(model, tmp_path):
    import mujoco
    from scipy.interpolate import CubicHermiteSpline

    with ZipFile(ROOT / "tests/data/j1_bounded_counter_experiments_20261030.zip") as archive:
        source = archive.read("final-nineaxis-readonly.json")
    recording = tmp_path / "recording.json"
    recording.write_bytes(source)
    last = {row["joint"]: row["state"]["angle_deg"] for row in json.loads(source)["samples"][-9:]}
    start = [last[joint] / (72 if joint == "J1" else 1) for joint in DEFAULT_JOINT_NAMES]
    target = [start[0] + .1, start[1] + .4, *start[2:]]
    table_path = tmp_path / "target.csv"
    with table_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(TABLE_COLUMNS)
        writer.writerow(["mujoco_reference", *target])
    result = subprocess.run([
        sys.executable, str(ROOT / "scripts/joint_positioning.py"), "--preview", "--csv", str(table_path),
        "--start-readonly", str(recording),
    ], cwd=ROOT, capture_output=True, text=True, check=True)
    row = json.loads(result.stdout)["synchronized_plan"]["rows"][0]
    from work.robot_units import joints_mm_deg_to_si

    times = np.array([sample["time_s"] for sample in row["samples"]])
    positions = joints_mm_deg_to_si([sample["position_mm_deg"] for sample in row["samples"]])
    speeds = joints_mm_deg_to_si([sample["velocity_mm_deg_s"] for sample in row["samples"]])
    reference = CubicHermiteSpline(times, positions, speeds, axis=0, extrapolate=False)
    table = single_target_table(dict(zip(DEFAULT_JOINT_NAMES, target)), model)
    executor = JointTableExecutor(table, model, measurement_kind="positioning", task_id="common-reference",
                                  feedback_source="simulation_feedback", manual_positioning=True)
    plant = MuJoCoObservations(executor)
    plant.data.qpos[plant.qpos] = positions[0]
    plant.data.ctrl[plant.actuators] = positions[0]
    mujoco.mj_forward(plant.physics, plant.data)
    goal = joints_mm_deg_to_si(target)
    observations = []
    while plant.data.time <= row["duration_s"] + 8.1:
        target_time = min(plant.data.time, row["duration_s"])
        feedback = plant.step(target=reference(target_time))
        if plant.data.time >= row["duration_s"] + 3:
            actual = plant.data.qpos[plant.qpos].copy()
            assert np.all(np.abs(actual - goal) <= np.array([.001, *([np.deg2rad(.1)] * 8)]))
            assert set(feedback) == set(DEFAULT_JOINT_NAMES)
            observations.append(plant.data.time)
    assert observations[-1] - observations[0] >= 5


def test_full_stroke_reference_preview_keeps_both_model_endpoints(tmp_path):
    with ZipFile(ROOT / "tests/data/j1_bounded_counter_experiments_20261030.zip") as archive:
        source = archive.read("final-nineaxis-readonly.json")
    recording = tmp_path / "recording.json"
    recording.write_bytes(source)
    last = {row["joint"]: row["state"]["angle_deg"] for row in json.loads(source)["samples"][-9:]}
    rotary = [last[joint] for joint in DEFAULT_JOINT_NAMES[1:]]
    table_path = tmp_path / "full-stroke.csv"
    with table_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(TABLE_COLUMNS)
        writer.writerows([["full_outward", 500, *rotary], ["full_return", 0, *rotary]])
    result = subprocess.run([
        sys.executable, str(ROOT / "scripts/joint_positioning.py"), "--preview", "--csv", str(table_path),
        "--start-readonly", str(recording),
    ], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    plan = json.loads(result.stdout)["synchronized_plan"]
    assert plan["hardware_executable"] is False
    assert plan["duration_s"] > 4000
    assert plan["rows"][0]["samples"][-1]["position_mm_deg"][0] == pytest.approx(500, abs=1e-10)
    assert plan["rows"][1]["samples"][-1]["position_mm_deg"][0] == pytest.approx(0, abs=1e-10)
    for row in plan["rows"]:
        for sample in row["samples"]:
            assert 0 <= sample["position_mm_deg"][0] <= 500
