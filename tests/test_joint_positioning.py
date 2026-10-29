import csv
import io
import json
import os
import struct
import subprocess
import sys
from dataclasses import replace
from pathlib import Path

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
from robot_safecontrol_moveit.robot_spec import DEFAULT_JOINT_NAMES
from robot_safecontrol_moveit.single_joint_execution import MotionRejected

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
