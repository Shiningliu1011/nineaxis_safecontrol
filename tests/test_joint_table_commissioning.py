import csv
import json
import time
from dataclasses import replace
from hashlib import sha256
from pathlib import Path
from zipfile import ZipFile

import pytest
from work.nineaxis_model import NineaxisModel

from robot_safecontrol_moveit.drempower_can import CMD_PROPERTY_READ
from robot_safecontrol_moveit.drempower_joint_control import JointFeedback
from robot_safecontrol_moveit.drempower_serial import SERIAL_CAN_FRAME
from robot_safecontrol_moveit.joint_table import TABLE_COLUMNS, JointTable
from robot_safecontrol_moveit.joint_table_commissioning import SinglePoseCommissioning
from robot_safecontrol_moveit.joint_table_execution import JointTableExecutor
from robot_safecontrol_moveit.joint_table_runtime import run_joint_table
from robot_safecontrol_moveit.motor_motion_limits import (
    MotionBounds,
    validate_motion_parameters,
)
from robot_safecontrol_moveit.motor_stop_latch import MotorStopLatch
from robot_safecontrol_moveit.single_joint_execution import (
    ExecutionLimits,
    MotionRejected,
    ObservedFeedback,
)

ROOT = Path(__file__).resolve().parents[1]
ARCHIVE = ROOT / "tests/data/joint_commissioning_stationary_20261029.zip"
TARGET = [50, 10, -20, 20, 60, -20, 30, -30, 30]


@pytest.fixture(scope="module")
def model():
    return NineaxisModel.load(root=ROOT)


@pytest.fixture
def request_files(tmp_path, model):
    with ZipFile(ARCHIVE) as archive:
        content = archive.read("commissioning-stationary.json")
    source = tmp_path / "actual-settings.json"
    source.write_bytes(content)
    table_path = tmp_path / "target.csv"
    with table_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(TABLE_COLUMNS)
        writer.writerow(["authorized-first", *TARGET])
    table = JointTable.load(table_path, model)
    original = json.loads(content)
    request = {
        "purpose": "single_pose_commissioning", "operator_attended": True,
        "operator_emergency_stop_confirmed": True, "operator_j1_zero_confirmed": True,
        "settings_path": str(source), "settings_sha256": sha256(content).hexdigest(),
        "table_sha256": table.source_sha256, "model_hash": model.model_hash,
        "config_hashes": {name: sha256((ROOT / "config" / name).read_bytes()).hexdigest()
                          for name in ("drempower.yaml", "hardware_joint_zero.yaml", "j1_transmission.yaml")},
        "port": original["port"], "expires_unix_s": time.time() + 590,
        "stop_latch_sha256": sha256((ROOT / ".scratch/motor_control/stop.json").read_bytes()).hexdigest()
        if (ROOT / ".scratch/motor_control/stop.json").is_file() else "0" * 64,
        "expected_j1_serial_number_l": 1955, "expected_j1_zero_offset_counts": 7733, "gui_scale": .75,
        "noise_evidence_path": str(ROOT / "tests/data/first_group_gui_readiness_20261029.zip"),
        "noise_evidence_sha256": sha256((ROOT / "tests/data/first_group_gui_readiness_20261029.zip").read_bytes()).hexdigest(),
    }
    path = tmp_path / "request.json"
    path.write_text(json.dumps(request), encoding="utf-8")
    # 纯计算直接构造请求；此处没有真实 I/O 或恢复现场锁定。
    permit = SinglePoseCommissioning(
        path, sha256(path.read_bytes()).hexdigest(), source, request["settings_sha256"],
        table.source_sha256, model.model_hash, tuple(sorted(request["config_hashes"].items())),
        request["port"], request["expires_unix_s"], request["stop_latch_sha256"], 1955, 7733, .75,
        Path(request["noise_evidence_path"]), request["noise_evidence_sha256"])
    return permit, table, original


def observations(source):
    cycle = {}
    for row in source["samples"]:
        data = {key: value for key, value in row.items() if key != "joint"}
        data["state"] = JointFeedback(**data["state"])
        cycle[row["joint"]] = ObservedFeedback(**data)
        if len(cycle) == 9:
            yield cycle
            cycle = {}
    assert not cycle


def test_stationary_archive_preserves_all_original_bytes_and_read_only_frames():
    with ZipFile(ARCHIVE) as archive:
        index = json.loads(archive.read("provenance.json"))
        content = archive.read("commissioning-stationary.json")
    assert sha256(content).hexdigest() == index["sha256"]
    assert len(content) == index["bytes"]
    source = json.loads(content)
    assert source["status"] == "readonly_completed"
    assert len(source["samples"]) == 3870
    requests = []
    for row in source["frames"]:
        if row["direction"] == "tx":
            frame = SERIAL_CAN_FRAME.parse(bytes.fromhex(row["raw_hex"]))
            assert frame.frame_id & 31 == CMD_PROPERTY_READ
            requests.append(frame)
    assert len(requests) == index["property_read_requests"] == 3960


def test_scaled_actual_gui_parameters_reach_commands_without_changing_default_caps(request_files, model):
    permit, table, source = request_files
    limits = permit.limits(ROOT)
    for joint, limit in limits.items():
        assert limit.accel_rpm_s == 22.5
        assert limit.speed_rpm == (18 if joint in {"J2", "J3", "J4", "J5", "J6"} else 24.99)
        assert limit.stationary_window_s == 5
        assert limit.stopped_rpm == .02
        assert limit.position_noise_deg == .01
        assert limit.tolerance_deg == .1
        assert limit.bounds.source_sha256 == permit.settings_sha256
        with pytest.raises(ValueError):
            validate_motion_parameters(limit.speed_rpm, limit.accel_rpm_s, axis_type=limit.axis_type)
        with pytest.raises(ValueError):
            replace(limit, speed_rpm=limit.speed_rpm + .01)
    executor = JointTableExecutor(table, model, measurement_kind="positioning", task_id="offline-real-record",
                                  feedback_source="recorded_device_feedback", limits=limits)
    started = None
    for cycle in observations(source):
        now = max(row.received_s for row in cycle.values())
        started = now if started is None else started
        executor.observe(cycle, now_s=now)
        if executor.phase == "ready":
            break
    assert executor.phase == "ready"
    assert 5.5 <= now - started < 7
    commands = executor.begin(now_s=now)
    assert commands[0]["arguments"]["angle"] == -3600
    assert [row["arguments"]["speed"] for row in commands] == [24.99, 18, 18, 18, 18, 18, 24.99, 24.99, 24.99]
    assert all(row["arguments"]["param"] == 22.5 for row in commands)
    assert executor.report()["hardware_qualified"] is False
    assert not executor.records


@pytest.mark.parametrize("field,value", [
    ("operator_emergency_stop_confirmed", False), ("operator_attended", False),
    ("operator_j1_zero_confirmed", False), ("gui_scale", 1), ("expires_unix_s", 0),
    ("table_sha256", "0" * 64), ("model_hash", "0" * 64), ("port", "absent"),
])
def test_unqualified_single_pose_request_rejected_before_transport(request_files, model, field, value):
    permit, table, _ = request_files
    request = json.loads(permit.request_path.read_bytes())
    request[field] = value
    permit.request_path.write_text(json.dumps(request))
    with pytest.raises((ValueError, RuntimeError)):
        SinglePoseCommissioning.load(permit.request_path, table, model, port=permit.port)


def test_request_cannot_retry_after_consumption(request_files, tmp_path):
    permit, _, _ = request_files
    permit.consume(tmp_path / "first.jsonl")
    with pytest.raises(FileExistsError):
        permit.consume(tmp_path / "second.jsonl")


def test_changed_limits_and_feedback_source_rejected_before_creating_io(request_files, model, tmp_path):
    permit, table, _ = request_files
    executor = JointTableExecutor(table, model, measurement_kind="positioning", task_id="offline-no-io",
                                  feedback_source="recorded_device_feedback", limits=permit.limits(ROOT))
    with pytest.raises((RuntimeError, ValueError)):
        run_joint_table(executor, port="absent", log_path=tmp_path / "run.jsonl",
                        measurement_dir=tmp_path, commissioning=permit)
    assert not (tmp_path / "run.jsonl").exists()
    assert not permit.request_path.with_suffix(".used.json").exists()


def test_manual_latch_recovery_preserves_original_and_rejects_changed_identity(tmp_path):
    path = tmp_path / "stop.json"
    latch = MotorStopLatch(path)
    latch.trip(reason="original", log_path=tmp_path / "original.jsonl")
    original = path.read_bytes()
    with pytest.raises(RuntimeError, match="身份变化"):
        latch.acknowledge_stop(expected_sha256="0" * 64, archive_path=tmp_path / "wrong.json")
    assert path.read_bytes() == original
    latch.acknowledge_stop(expected_sha256=sha256(original).hexdigest(), archive_path=tmp_path / "saved.json")
    assert (tmp_path / "saved.json").read_bytes() == original
    assert not path.exists()
    latch.trip(reason="single_pose_authorization_completed", log_path=tmp_path / "new.jsonl")
    with pytest.raises(RuntimeError, match="single_pose_authorization_completed"):
        latch.require_clear()


def test_real_first_pose_replays_the_same_j2_rejection_and_preserves_observation(model, tmp_path):
    archive_path = ROOT / "tests/data/first_pose_commissioning_20261029.zip"
    with ZipFile(archive_path) as archive:
        index = json.loads(archive.read("provenance.json"))
        for entry in index["files"]:
            content = archive.read(entry["member"])
            assert sha256(content).hexdigest() == entry["sha256"]
            assert len(content) == entry["bytes"]
        original = json.loads(archive.read("first-pose-075-execution.report.json"))
        events = [json.loads(line) for line in archive.read("first-pose-075-execution.jsonl").splitlines()]
        frames = json.loads(archive.read("first-pose-075-execution.frames.json"))
        analysis = json.loads(archive.read("first-pose-075-analysis.json"))
    with ZipFile(ROOT / "tests/data/joint_motion_recovery_20261029.zip") as archive:
        source = archive.read("requested-target.csv")
    table_path = tmp_path / "original-target.csv"
    table_path.write_bytes(source)
    table = JointTable.load(table_path, model)
    assert table.source_sha256 == original["table_sha256"]
    limits = {joint: ExecutionLimits(**{**row, "bounds": MotionBounds(**row["bounds"])})
              for joint, row in original["limits"].items()}
    executor = JointTableExecutor(table, model, measurement_kind="positioning", task_id="actual-first-pose-replay",
                                  feedback_source="recorded_device_feedback", limits=limits)
    begun = False
    with pytest.raises(MotionRejected, match="J2:motion_outside_requested_interval"):
        for event in events:
            if event["event"] == "command_requested" and not begun:
                executor.begin(now_s=event["monotonic_s"])
                begun = True
            if event["event"] == "feedback":
                samples = {joint: ObservedFeedback(**{**row, "state": JointFeedback(**row["state"])})
                           for joint, row in event["samples"].items()}
                executor.observe(samples, now_s=event["monotonic_s"])
    assert begun
    assert executor.rejection_observation["joint"] == "J2"
    assert executor.rejection_observation["values"]["angle_deg"] == 10.686097145080566
    assert executor.rejection_observation["target_deg"] == 10
    assert executor.rejection_observation["trusted"] is False
    assert original["termination"]["stop_result"] == "external_stop_required"
    assert analysis["position_commands_sent"] == analysis["hold_commands_sent"] == 9
    assert analysis["programmatic_all_axis_stop_confirmed"] is False
    commands = [SERIAL_CAN_FRAME.parse(bytes.fromhex(row["raw_hex"])) for row in frames if row["direction"] == "tx"]
    assert sum(row.frame_id & 31 == 26 for row in commands) == 18
    assert all(row.frame_id & 31 in (26, CMD_PROPERTY_READ) for row in commands)
    assert not executor.records


def test_gravity_stop_blocker_cannot_be_acknowledged_or_removed(tmp_path):
    with ZipFile(ROOT / "tests/data/joint_motion_recovery_20261029.zip") as archive:
        original = archive.read("current-stop.json")
        operator = json.loads(archive.read("operator-stop-and-zero-request.json"))
    assert operator["operator_confirmed_j5_j6_moved_under_gravity_when_disabled"] is True
    assert operator["operator_confirmed_mechanical_support_or_brake_available"] is False
    assert operator["return_zero_executed"] is False
    path = tmp_path / "stop.json"
    path.write_bytes(original)
    saved = tmp_path / "previous-stop.json"
    with pytest.raises(RuntimeError, match="single_pose_hardware_blocked"):
        MotorStopLatch(path).acknowledge_stop(expected_sha256=sha256(original).hexdigest(), archive_path=saved)
    assert path.read_bytes() == original
    assert not saved.exists()
