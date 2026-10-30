import json
import os
from hashlib import sha256
from pathlib import Path
import subprocess
import sys
from zipfile import ZipFile

import pytest

from robot_safecontrol_moveit.drempower_can import decode_feedback
from robot_safecontrol_moveit.drempower_serial import DrEmpowerSerial, SERIAL_CAN_FRAME
from robot_safecontrol_moveit.joint_table_execution import JointTableControl
from robot_safecontrol_moveit.motor_motion_limits import TrackingProbeBounds
from robot_safecontrol_moveit.tracking_log_replay import replay_tracking_capture


ROOT = Path(__file__).resolve().parents[1]
FAILURE = ROOT / "tests/data/nineaxis_continuous_failure_20261030.zip"
PREVIOUS = ROOT / "tests/data/motor_sync_live_validation_20261030.zip"


def original_sources(archive_path, name):
    with ZipFile(archive_path) as archive:
        return archive.read(f"{name}.jsonl"), archive.read(f"{name}.capture.json")


def test_actual_nineaxis_abort_keeps_unlogged_automatic_feedback_and_source_failure():
    report = replay_tracking_capture(*original_sources(FAILURE, "nineaxis-roundtrip-02"), ROOT)
    assert report["capture_status"] == "aborted"
    assert report["recorded_terminal"]["stop_result"] == "external_stop_required"
    assert report["continuous_roundtrip_completed"] is False
    assert report["matched_samples"] == 5949
    assert report["raw_feedback_samples"] == 7542
    assert report["broadcast_batches"] == 524
    first = report["first_interval_violation"]
    assert first["joint"] == "J4"
    assert first["position"] == pytest.approx(-.15252381563186646)
    assert first["unit"] == "deg"
    assert first["raw_frame_index"] == 31015
    assert first["included_in_feedback_journal"] is False
    assert first["axis_error"] is False
    assert first["traj_done"] is True
    assert first["reply_source"] == "motion_acknowledgement"
    assert first["device_torque_nm"] == 4.5
    assert first["reference"] == pytest.approx(-.041648946702480316)
    assert first["reference_time_s"] == 23975.906094615
    assert first["tracking_error"] == pytest.approx(-.11087486892938614)
    assert first["reference_time_s"] < first["received_s"]
    assert report["maximum_broadcast_feedback_age_s"] == pytest.approx(.053323115997045534)
    assert report["broadcast_interval_s"]["median"] == pytest.approx(.050036868000461254)
    assert report["hardware_qualified"] is report["physical_stop_independently_verified"] is False


def test_original_response_preserves_large_position_step_without_a_target_step():
    report = replay_tracking_capture(*original_sources(FAILURE, "nineaxis-roundtrip-02"), ROOT)
    dynamics = report["response_dynamics"]
    assert dynamics["time_basis"] == "host_receive_intervals"
    assert dynamics["device_sample_time_known"] is dynamics["independent_motion_measurement"] is False
    j4 = dynamics["joints"]["J4"]
    pair = j4["largest_adjacent_position_change"]
    assert j4["unit"] == "deg"
    assert pair["position_change"] == pytest.approx(-.04579275846481323)
    assert pair["reference_change"] == 0.
    assert pair["host_receive_interval_s"] == pytest.approx(.03903597099997569)
    assert pair["position_rate_per_host_second"] == pytest.approx(-1.1730913127495086)
    assert pair["before"]["reply_source"] == "property_read"
    assert pair["after"]["reply_source"] == "motion_acknowledgement"
    assert pair["before"]["device_torque_nm"] == 6.18
    assert pair["after"]["device_torque_nm"] == 5.34
    assert j4["maximum_error_sample"]["tracking_error"] == pytest.approx(-.11091868206858635)
    assert dynamics["joints"]["J5"]["maximum_error_sample"]["tracking_error"] == pytest.approx(.059659577906131744)
    assert dynamics["joints"]["J6"]["maximum_error_sample"]["tracking_error"] == pytest.approx(-.07116097025573255)


def test_actual_pre_broadcast_rejection_keeps_nineaxis_hold_without_streaming_commands():
    report = replay_tracking_capture(*original_sources(FAILURE, "sequence"), ROOT)
    assert report["capture_status"] == "aborted"
    assert report["recorded_terminal"]["stop_result"] == "hold_observed"
    assert report["broadcast_batches"] == 0
    assert report["position_commands"] == 18
    assert report["first_interval_violation"] is None
    assert report["continuous_roundtrip_completed"] is False
    assert all(row["largest_adjacent_position_change"] is None
               and row["maximum_error_sample"] is None
               for row in report["response_dynamics"]["joints"].values())


def test_original_completed_four_axis_tracking_remains_separate_from_nineaxis_roundtrip():
    report = replay_tracking_capture(*original_sources(PREVIOUS, "tracking-forward"), ROOT)
    assert report["capture_status"] == "completed"
    assert report["matched_samples"] == 8946
    assert report["broadcast_batches"] == 722
    assert report["position_commands"] == 9
    assert report["first_interval_violation"] is None
    assert report["continuous_roundtrip_completed"] is False


@pytest.mark.parametrize("change,message", [("status", "终态"), ("feedback", "反馈"), ("target", "目标")])
def test_actual_tracking_replay_rejects_contradictory_completion_feedback_or_device_targets(change, message):
    source, capture_source = original_sources(FAILURE, "nineaxis-roundtrip-02")
    captured = json.loads(capture_source)
    if change == "status":
        captured["status"] = "completed"
    else:
        events = [json.loads(line) for line in source.splitlines()]
        if change == "feedback":
            next(row for row in events if row["event"] == "feedback")["samples"]["J4"]["state"]["angle_deg"] = 0.
        else:
            next(row for row in events if row["event"] == "tracking_broadcast")["targets_device_deg"]["1"] *= -1
        source = ("\n".join(json.dumps(row) for row in events) + "\n").encode()
        captured["log_sha256"] = sha256(source).hexdigest()
    with pytest.raises(ValueError, match=message):
        replay_tracking_capture(source, json.dumps(captured).encode(), ROOT)


def test_existing_motion_replay_cli_accepts_embedded_tracking_frames_without_a_port(tmp_path):
    source, captured = original_sources(FAILURE, "nineaxis-roundtrip-02")
    log_path, capture_path, output = tmp_path / "motion.jsonl", tmp_path / "capture.json", tmp_path / "replay.json"
    log_path.write_bytes(source)
    capture_path.write_bytes(captured)
    result = subprocess.run([
        sys.executable, "-m", "robot_safecontrol_moveit.motion_log_replay", "--root", str(ROOT),
        "--log", str(log_path), "--tracking-capture", str(capture_path), "--output", str(output)],
        cwd=ROOT, env=os.environ.copy(), text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
    report = json.loads(output.read_bytes())
    assert report["capture_status"] == "aborted"
    assert report["first_interval_violation"]["joint"] == "J4"


def test_encoder_target_conversion_matches_actual_nineaxis_broadcast_requests():
    source, _ = original_sources(FAILURE, "nineaxis-roundtrip-02")
    events = [json.loads(line) for line in source.splitlines()]
    control = JointTableControl(ROOT)
    references = [row for row in events if row["event"] == "tracking_reference"]
    broadcasts = [row for row in events if row["event"] == "tracking_broadcast"]
    assert len(references) == len(broadcasts) == 524
    for reference, broadcast in zip(references, broadcasts):
        actual = control.encoder_targets(reference["q_si"])
        expected = {int(node): angle for node, angle in broadcast["targets_device_deg"].items()}
        assert actual == pytest.approx(expected, abs=1e-12)
    for invalid in ([0.] * 8, [float("nan"), *([0.] * 8)]):
        with pytest.raises(ValueError, match="完整九轴有限 SI"):
            control.encoder_targets(invalid)


def test_bounded_tracking_feedback_rejects_actual_automatic_reply_before_next_broadcast():
    _, source = original_sources(FAILURE, "nineaxis-roundtrip-02")
    capture = json.loads(source)
    control = JointTableControl(ROOT)
    lower = control.encoder_targets(capture["review"]["lower_si"])
    upper = control.encoder_targets(capture["review"]["upper_si"])
    bounds = TrackingProbeBounds(tuple(min(lower[node], upper[node]) for node in range(1, 10)),
                                 tuple(max(lower[node], upper[node]) for node in range(1, 10)))
    accepted = 0
    for index, frame in enumerate(capture["frames"]):
        if frame["direction"] != "rx":
            continue
        parsed = SERIAL_CAN_FRAME.parse(bytes.fromhex(frame["raw_hex"]))
        if parsed.frame_id & 0x1f not in (1, 3, 5, 7):
            continue
        feedback = decode_feedback(parsed.frame_id, parsed.data)
        if index == 31015:
            with pytest.raises(RuntimeError, match="节点 4.*已检查范围"):
                bounds.require_position(feedback.node_id, feedback.pos_deg)
            assert accepted > 0
            break
        bounds.require_position(feedback.node_id, feedback.pos_deg)
        accepted += 1
    else:
        pytest.fail("原始自动回复没有触发已检查范围拒绝")


def test_tracking_scope_requires_checked_bounds_before_opening_serial(tmp_path):
    nodes = {1: "prismatic", **dict.fromkeys(range(2, 10), "revolute")}
    with pytest.raises(ValueError, match="已检查范围"):
        DrEmpowerSerial(str(tmp_path), position_nodes=nodes, tracking_probe=True)


def test_aborted_original_trace_prefix_retains_unbroadcast_partial_batch():
    source, capture_source = original_sources(FAILURE, "nineaxis-roundtrip-02")
    capture = json.loads(capture_source)
    capture["frames"] = capture["frames"][:31016]
    events = [json.loads(line) for line in source.splitlines()]
    cutoff = capture["frames"][-1]["monotonic_s"]
    prefix = [row for row in events if row["monotonic_s"] <= cutoff]
    prefix.append(events[-1])
    source = ("\n".join(json.dumps(row) for row in prefix) + "\n").encode()
    capture["log_sha256"] = sha256(source).hexdigest()
    report = replay_tracking_capture(source, json.dumps(capture).encode(), ROOT)
    assert report["capture_status"] == "aborted"
    assert report["broadcast_batches"] == 523
    assert [row["node_id"] for row in report["unbroadcast_presets"]] == [1, 2, 3, 4]
    assert report["first_interval_violation"]["raw_frame_index"] == 31015
    assert report["continuous_roundtrip_completed"] is False


@pytest.mark.parametrize("lower,upper", [
    ((-1.,) * 8, (1.,) * 9),
    ((-1.,) * 9, (-1.,) * 9),
    ((float("nan"),) + (-1.,) * 8, (1.,) * 9),
    ([-1.] * 9, (1.,) * 9),
])
def test_tracking_bounds_require_complete_finite_immutable_intervals(lower, upper):
    with pytest.raises(ValueError, match="完整九轴有限设备角度范围"):
        TrackingProbeBounds(lower, upper)
