import json
import shutil
from dataclasses import replace
from hashlib import sha256
from pathlib import Path
from zipfile import ZipFile

import pytest
from test_joint_table_commissioning import observations
from work.nineaxis_model import NineaxisModel

from robot_safecontrol_moveit.attended_positioning import AttendedPositioning
from robot_safecontrol_moveit.drempower_joint_control import JointFeedback
from robot_safecontrol_moveit.joint_positioning import (
    positioning_limits,
    single_target_table,
)
from robot_safecontrol_moveit.joint_table import require_hardware_qualification
from robot_safecontrol_moveit.joint_table_execution import JointTableExecutor
from robot_safecontrol_moveit.joint_table_runtime import run_joint_table
from robot_safecontrol_moveit.motor_motion_limits import MotionBounds
from robot_safecontrol_moveit.robot_spec import DEFAULT_JOINT_NAMES
from robot_safecontrol_moveit.single_joint_execution import (
    ExecutionLimits,
    MotionRejected,
    ObservedFeedback,
)
from robot_safecontrol_moveit.tracking_log_replay import _match_commands, _raw_tracking

ROOT = Path(__file__).resolve().parents[1]
ARCHIVE = ROOT / "tests/data/attended_positioning_readiness_20261029.zip"


@pytest.fixture(scope="module")
def actual_model():
    return NineaxisModel.load(root=ROOT)


@pytest.fixture
def experiment(tmp_path, actual_model):
    shutil.copytree(ROOT / "config", tmp_path / "config")
    for relative in ("models/ninezzhou/urdf/ninezzhou.urdf", "tests/data/first_group_gui_readiness_20261029.zip"):
        destination = tmp_path / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / relative, destination)
    directory = tmp_path / ".scratch/motor_control"
    directory.mkdir(parents=True)
    with ZipFile(ARCHIVE) as archive:
        for item in json.loads(archive.read("provenance.json")):
            content = archive.read(item["member"])
            assert sha256(content).hexdigest() == item["sha256"]
            assert len(content) == item["bytes"]
        for name in ("attended-positioning.json", "stop.json"):
            (directory / name).write_bytes(archive.read(name))
        source = json.loads(archive.read("hardware-readonly-supervised-01.json"))
    policy_path = directory / "attended-positioning.json"
    policy = json.loads(policy_path.read_bytes())
    policy["maximum_feedback_deg_s"] = 5.
    policy_path.write_text(json.dumps(policy), encoding="utf-8")
    model = replace(actual_model, root=tmp_path)
    table = single_target_table(dict(zip(DEFAULT_JOINT_NAMES, [50, 10, -20, 30, -30, 60, 10, 20, 30])), model)
    limits = positioning_limits(tmp_path, dict(zip(DEFAULT_JOINT_NAMES, [.25, *([2.46] * 8)])),
                                dict(zip(DEFAULT_JOINT_NAMES, [.05, *([.3] * 8)])))
    session = AttendedPositioning.load(tmp_path, directory / "attended-positioning.json", table, model,
                                       port=source["port"], limits=limits)
    return session, model, table, source, limits


def test_real_stationary_feedback_enters_ready_and_retains_motion_limits(experiment):
    session, model, table, source, original = experiment
    continuous = {**source, "samples": source["samples"][9:]}
    baseline = JointTableExecutor(table, model, measurement_kind="positioning", task_id="baseline",
                                   feedback_source="recorded_device_feedback", limits=original, manual_positioning=True)
    with pytest.raises(MotionRejected, match="initial_stationary_timeout"):
        for cycle in observations(continuous):
            baseline.observe(cycle, now_s=max(sample.received_s for sample in cycle.values()))
    executor = JointTableExecutor(table, model, measurement_kind="positioning", task_id="attended-replay",
                                  feedback_source="recorded_device_feedback", limits=session.limits, manual_positioning=True)
    for cycle in observations(continuous):
        now_s = max(sample.received_s for sample in cycle.values())
        executor.observe(cycle, now_s=now_s)
    assert executor.phase == "ready"
    assert not executor.attempts
    commands = executor.begin(now_s=now_s)
    assert len(commands) == 9 and executor.phase == "moving"
    for joint, limit in session.limits.items():
        previous = original[joint]
        assert limit.speed_rpm == previous.speed_rpm
        assert limit.accel_rpm_s == previous.accel_rpm_s
        assert limit.max_feedback_rpm == (previous.max_feedback_rpm if joint == "J1" else 5 / 6)
        assert limit.position_noise_deg == previous.position_noise_deg
        assert limit.tolerance_deg == previous.tolerance_deg
        assert limit.stopped_rpm == previous.stopped_rpm
        assert limit.settle_s == previous.settle_s
    assert session.report()["gravity_safe_stop_verified"] is False
    with pytest.raises(RuntimeError, match="锁定"):
        require_hardware_qualification(session.root)


def test_new_fault_prevents_restart_without_changing_reviewed_stop(experiment, tmp_path):
    session, model, table, source, limits = experiment
    original = (session.root / ".scratch/motor_control/stop.json").read_bytes()
    session.check_stop()
    session.stop_latch.trip(reason="J2:velocity_limit", log_path=tmp_path / "experiment.jsonl")
    with pytest.raises(RuntimeError, match="J2:velocity_limit"):
        session.check_stop()
    with pytest.raises(RuntimeError, match="J2:velocity_limit"):
        AttendedPositioning.load(session.root, session.policy_path, table, model, port=source["port"], limits=limits)
    assert (session.root / ".scratch/motor_control/stop.json").read_bytes() == original


@pytest.mark.parametrize("field", ["reviewed_stop_sha256", "model_hash", "port", "config_hashes",
                                    "noise_evidence_sha256", "conditions"])
def test_changed_scope_rejected_before_transport(experiment, field):
    session, model, table, source, limits = experiment
    policy = json.loads(session.policy_path.read_bytes())
    if field == "conditions":
        policy[field]["operator_attended"] = False
    elif field == "config_hashes":
        policy[field]["drempower.yaml"] = "0" * 64
    else:
        policy[field] = "0" * 64
    session.policy_path.write_text(json.dumps(policy), encoding="utf-8")
    with pytest.raises((ValueError, RuntimeError)):
        AttendedPositioning.load(session.root, session.policy_path, table, model, port=source["port"], limits=limits)


def test_running_session_detects_policy_and_stop_changes(experiment):
    session, _, _, _, _ = experiment
    session.policy_path.write_text("{}", encoding="utf-8")
    with pytest.raises(RuntimeError, match="现场记录在运行期间变化"):
        session.check_stop()
    stop = session.root / ".scratch/motor_control/stop.json"
    stop.write_text("{}", encoding="utf-8")
    with pytest.raises(RuntimeError, match="已审阅停止记录变化"):
        session.check_stop()


def test_attended_policy_cannot_enable_automatic_or_recorded_execution(experiment, tmp_path):
    session, model, table, source, _ = experiment
    executor = JointTableExecutor(table, model, measurement_kind="positioning", task_id="rejected",
                                  feedback_source="recorded_device_feedback", limits=session.limits, manual_positioning=True)
    with pytest.raises(ValueError, match="真实反馈"):
        session.require_scope(executor, port=source["port"])
    with pytest.raises(ValueError, match="独立的手动入口"):
        run_joint_table(executor, port=source["port"], log_path=tmp_path / "not-created.jsonl",
                        measurement_dir=tmp_path, attended=session)
    assert not (tmp_path / "not-created.jsonl").exists()


def test_actual_j7_motion_and_later_stationary_feedback_use_reviewed_experiment_limits(experiment):
    session, model, table, _, _ = experiment
    with ZipFile(ROOT / "tests/data/attended_positioning_recovery_20261029.zip") as archive:
        for item in json.loads(archive.read("provenance.json")):
            content = archive.read(item["member"])
            assert len(content) == item["bytes"]
            assert sha256(content).hexdigest() == item["sha256"]
        events = [json.loads(line) for line in archive.read("attended-motion-01.jsonl").splitlines()]
        original = json.loads(archive.read("attended-motion-01.report.json"))
        held = json.loads(archive.read("attended-held-readonly-01.json"))
    assert original["rejection_reason"] == "J7:feedback_position_jump"
    executor = JointTableExecutor(table, model, measurement_kind="positioning", task_id="actual-motion-replay",
                                  feedback_source="recorded_device_feedback", limits=session.limits, manual_positioning=True)
    for event in events:
        if event["event"] == "command_requested" and not executor.attempts:
            assert len(executor.begin(now_s=event["monotonic_s"])) == 9
        if event["event"] == "feedback":
            samples = {joint: ObservedFeedback(**{**row, "state": JointFeedback(**row["state"])})
                       for joint, row in event["samples"].items()}
            executor.observe(samples, now_s=event["monotonic_s"])
    assert executor.phase == "moving"
    assert executor.latest["J7"].state.angle_deg == original["rejection_observation"]["values"]["angle_deg"]
    holds = [event for event in events if event["event"] == "hold_feedback"]
    observed_s = holds[-1]["monotonic_s"] - holds[0]["monotonic_s"]
    assert original["termination"]["conditions"]["J1"]["position_stability"] is False
    assert observed_s < session.limits["J1"].reposition_hold_timeout_s
    stopped = JointTableExecutor(table, model, measurement_kind="positioning", task_id="actual-held-replay",
                                 feedback_source="recorded_device_feedback", limits=session.limits, manual_positioning=True)
    for cycle in observations({**held, "samples": held["samples"][9:]}):
        now_s = max(sample.received_s for sample in cycle.values())
        stopped.observe(cycle, now_s=now_s)
    assert stopped.phase == "ready"
    assert abs(stopped.latest["J7"].state.angle_deg) > 3
    assert len(stopped.begin(now_s=now_s)) == 9


def test_actual_nineaxis_native_positioning_replays_arrival_and_continuous_hold(actual_model):
    with ZipFile(ROOT / "tests/data/direct-position-02_20261030.zip") as archive:
        for item in json.loads(archive.read("provenance.json"))["files"]:
            source = archive.read(item["member"])
            assert len(source) == item["bytes"]
            assert sha256(source).hexdigest() == item["sha256"]
        events = [json.loads(line) for line in archive.read("direct-position-02.jsonl").splitlines()]
        report = json.loads(archive.read("direct-position-02.report.json"))
        frames = json.loads(archive.read("direct-position-02.frames.json"))
    target = dict(zip(DEFAULT_JOINT_NAMES, [2., 1., -1., 1., -1., 1., 1., -1., 1.]))
    table = single_target_table(target, actual_model)
    limits = {joint: ExecutionLimits(**{**values, "bounds": MotionBounds(**values["bounds"])
                                        if values["bounds"] else None})
              for joint, values in report["limits"].items()}
    executor = JointTableExecutor(table, actual_model, measurement_kind="positioning", task_id="native-positioning",
                                  feedback_source="recorded_device_feedback", limits=limits, manual_positioning=True)
    trace = _raw_tracking({"frames": frames, "status": report["phase"]}, executor.control)
    _match_commands([{**row, "event": "low_speed_seed_requested"} if row["event"] == "command_requested"
                     else row for row in events], trace, executor.control)
    assert len(trace["positions"]) == 9 and not trace["broadcasts"]
    assert set(trace["tx_commands"]) == {"0x1a", "0x1e"}
    arrived_s = None
    for event in events:
        if event["event"] == "command_requested" and not executor.attempts:
            assert len(executor.begin(now_s=event["monotonic_s"])) == 9
        if event["event"] == "feedback":
            samples = {joint: ObservedFeedback(**{**row, "state": JointFeedback(**row["state"])})
                       for joint, row in event["samples"].items()}
            executor.observe(samples, now_s=event["monotonic_s"])
        if event["event"] == "positioning_completed":
            assert executor.phase == "holding_position"
            arrived_s = event["monotonic_s"]
        if event["event"] == "completed":
            executor.finish_positioning(now_s=event["monotonic_s"])
    assert arrived_s is not None and events[-1]["monotonic_s"] - arrived_s > 50
    assert executor.phase == "completed"
    assert executor.report()["all_positioning_targets_completed"] is True
    assert executor.limits["J1"].tolerance_deg == 72.
    assert all(abs(row["error_mm_deg"]) < (.001 if joint == "J1" else .01)
               for joint, row in executor.feedback_report().items())


def test_actual_larger_motion_waits_for_all_joint_stationarity(actual_model):
    with ZipFile(ROOT / "tests/data/larger-position-01_20261030.zip") as archive:
        for item in json.loads(archive.read("provenance.json"))["files"]:
            source = archive.read(item["member"])
            assert len(source) == item["bytes"]
            assert sha256(source).hexdigest() == item["sha256"]
        events = [json.loads(line) for line in archive.read("larger-position-01.jsonl").splitlines()]
        report = json.loads(archive.read("larger-position-01.report.json"))
    table = single_target_table(dict(zip(DEFAULT_JOINT_NAMES, report["attempts"][0]["target_mm_deg"])), actual_model)
    limits = {joint: ExecutionLimits(**{**values, "bounds": MotionBounds(**values["bounds"])
                                        if values["bounds"] else None})
              for joint, values in report["limits"].items()}
    executor = JointTableExecutor(table, actual_model, measurement_kind="positioning", task_id="larger-native-motion",
                                  feedback_source="recorded_device_feedback", limits=limits, manual_positioning=True)
    assert report["termination"]["reason"] == "J7:target_left_settling_tolerance"
    for event in events:
        if event["event"] == "command_requested" and not executor.attempts:
            executor.begin(now_s=event["monotonic_s"])
        if event["event"] == "feedback":
            samples = {joint: ObservedFeedback(**{**row, "state": JointFeedback(**row["state"])})
                       for joint, row in event["samples"].items()}
            executor.observe(samples, now_s=event["monotonic_s"])
    sample = executor.latest["J7"]
    assert abs(sample.state.angle_deg - 7) < .003
    assert sample.stationary_position_span_deg == .010471343994140625
    assert executor.limits["J7"].position_noise_deg == .01
    assert executor.attempts[-1]["last_conditions"]["J7"]["position_stability"] is False
    assert executor.phase == "moving"
    assert executor.settled_since_s is None
    assert not executor.positioning_records
