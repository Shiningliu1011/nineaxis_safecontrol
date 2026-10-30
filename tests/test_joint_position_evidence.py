import json
import os
import subprocess
import sys
from hashlib import sha256
from pathlib import Path
from zipfile import ZipFile

import pytest

from robot_safecontrol_moveit.drempower_can import (
    CMD_PROPERTY_READ,
    CMD_PROPERTY_WRITE,
    CMD_SYSTEM,
    SYSTEM_ORDER_SAVE,
    encode_property_read,
    encode_property_write,
    encode_system,
)
from robot_safecontrol_moveit.drempower_serial import SERIAL_CAN_FRAME

ARCHIVE = Path(__file__).with_name("data") / "joint_position_readonly_20261029.zip"
MULTI_CIRCLE_ARCHIVE = ARCHIVE.with_name("j1_multi_circle_20261029.zip")
RETREAT_ARCHIVE = ARCHIVE.with_name("j1_retreat28_preflight_20261029.zip")
POSITIONING_ARCHIVE = ARCHIVE.with_name("motor_positioning_readonly_20261030.zip")
SAMPLING_ARCHIVE = ARCHIVE.with_name("j1_counter_sampling_20261030.zip")
OPERATING_CAPTURE = ARCHIVE.with_name("motor_power_cycle_operating_20261030.json")
SERVO_ARCHIVE = ARCHIVE.with_name("motor_servo_diagnosis_20261030.zip")


def test_replay_reconstructs_fault_properties_and_host_read_intervals():
    from robot_safecontrol_moveit.joint_table_readonly import replay_readonly_report

    with ZipFile(ARCHIVE.with_name("attended_positioning_j1_fault_20261029.zip")) as archive:
        source = archive.read("j1-fault-01.json")
    report = replay_readonly_report(source)
    assert report["validation_scope"] == "offline_original_readonly_replay"
    assert report["source_sha256"] == sha256(source).hexdigest()
    assert report["status"] == "replayed_completed"
    assert report["nodes"]["1"]["encoder_turn_abs"] == -2047
    assert report["nodes"]["1"]["encoder_error"] == 512
    assert report["nodes"]["1"]["zero_offset_counts"] == 7733
    error_read = next(item for item in report["property_observations"] if item["name"] == "axis_error")
    assert error_read["node_id"] == 1
    assert error_read["address"] == 30001
    assert error_read["value"] == 256
    assert error_read["request_s"] == 17528.290725312
    assert error_read["received_s"] == 17528.291363807
    assert report["device_sample_time_known"] is False
    assert report["hardware_qualified"] is False
    assert report["motion_commands_sent"] == report["parameter_writes_sent"] == 0


def test_readonly_cli_replays_an_original_archive_without_a_port(tmp_path):
    root = Path(__file__).resolve().parents[1]
    output = tmp_path / "fault-replay.json"
    command = [sys.executable, "-m", "robot_safecontrol_moveit.joint_table_readonly", "--root", str(root),
               "--replay", str(ARCHIVE.with_name("attended_positioning_j1_fault_20261029.zip")),
               "--archive-member", "j1-fault-01.json", "--output", str(output)]
    result = subprocess.run(command, cwd=root, env=os.environ.copy(), text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
    report = json.loads(output.read_text())
    assert report["nodes"]["1"]["axis_error"] == 256
    assert report["archive_member"] == "j1-fault-01.json"
    assert report["archive_member_verified"] is True
    repeated = subprocess.run(command, cwd=root, env=os.environ.copy(), text=True, capture_output=True)
    assert repeated.returncode != 0
    assert output.read_text() == json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n"


def test_readonly_cli_replays_actual_power_cycle_operating_modes(tmp_path):
    root = Path(__file__).resolve().parents[1]
    output = tmp_path / "operating-replay.json"
    result = subprocess.run([
        sys.executable, "-m", "robot_safecontrol_moveit.joint_table_readonly", "--root", str(root),
        "--replay", str(OPERATING_CAPTURE), "--output", str(output)],
        cwd=root, env=os.environ.copy(), text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
    report = json.loads(output.read_bytes())
    assert len(report["property_observations"]) == 63
    assert len(report["feedback_observations"]) == 9
    for node, values in report["nodes"].items():
        assert values["control_mode"] == 3
        assert values["input_mode"] == 5
        assert values["trajectory_done"] == 1
        assert values["input_filter_bandwidth"] == (66 if node in ("4", "5", "6") else 100)
    assert report["hardware_qualified"] is False


def test_operating_replay_rejects_reported_mode_that_disagrees_with_original_reply():
    from robot_safecontrol_moveit.joint_table_readonly import replay_readonly_report

    recorded = json.loads(OPERATING_CAPTURE.read_bytes())
    recorded["properties"]["J4"]["input_mode"] = 1
    with pytest.raises(ValueError, match="input_mode"):
        replay_readonly_report(json.dumps(recorded).encode())


@pytest.mark.parametrize("member,properties,feedback,status", [
    ("post-estop-operating.json", 225, 990, "replayed_completed"),
    ("post-estop-servo.json", 36, 9, "replayed_incomplete"),
    ("post-estop-servo-core.json", 324, 1611, "replayed_completed"),
    ("post-estop-servo-bandwidth.json", 333, 1629, "replayed_completed"),
])
def test_post_stop_servo_captures_preserve_actual_values_and_read_failures(member, properties, feedback, status):
    from robot_safecontrol_moveit.joint_table_readonly import replay_readonly_archive

    report = replay_readonly_archive(SERVO_ARCHIVE, member)
    assert report["status"] == status
    assert len(report["property_observations"]) == properties
    assert len(report["feedback_observations"]) == feedback
    assert report["motion_commands_sent"] == report["parameter_writes_sent"] == 0
    assert report["hardware_qualified"] is report["device_sample_time_known"] is False
    if status == "replayed_incomplete":
        assert report["incomplete_reads"][0]["address"] == 32101
        assert report["incomplete_reads"][0]["response_bytes"] == 0
    if member == "post-estop-servo-bandwidth.json":
        for node, values in report["nodes"].items():
            assert values["current_control_bandwidth"] == 200
            assert values["vel_integrator_gain"] == pytest.approx(1.604)
            assert values["pos_gain"] == (10 if node in ("2", "3", "4", "5", "6") else 12)
            assert values["output_input_pos_deg"] == values["output_pos_setpoint_deg"]
        assert report["nodes"]["4"]["vel_gain"] == pytest.approx(.2475)


@pytest.mark.parametrize("field", ["pos_gain", "current_control_bandwidth", "output_pos_setpoint_deg"])
def test_servo_replay_rejects_configuration_or_target_inconsistent_with_raw_reply(field):
    from robot_safecontrol_moveit.joint_table_readonly import replay_readonly_report

    with ZipFile(SERVO_ARCHIVE) as archive:
        recorded = json.loads(archive.read("post-estop-servo-bandwidth.json"))
    recorded["properties"]["J4"][field] += 1
    with pytest.raises(ValueError, match=field):
        replay_readonly_report(json.dumps(recorded).encode())


def test_readonly_cli_replays_original_servo_fields_without_a_port(tmp_path):
    root = Path(__file__).resolve().parents[1]
    output = tmp_path / "servo-replay.json"
    result = subprocess.run([
        sys.executable, "-m", "robot_safecontrol_moveit.joint_table_readonly", "--root", str(root),
        "--replay", str(SERVO_ARCHIVE), "--archive-member", "post-estop-servo-bandwidth.json",
        "--output", str(output)], cwd=root, env=os.environ.copy(), text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
    report = json.loads(output.read_bytes())
    assert report["nodes"]["4"]["pos_gain"] == 10
    assert report["nodes"]["4"]["current_control_bandwidth"] == 200


def test_replay_decodes_original_j1_encoder_configuration():
    from robot_safecontrol_moveit.joint_table_readonly import replay_readonly_report

    source = ARCHIVE.with_name("j1_encoder_configuration_20261029.json").read_bytes()
    report = replay_readonly_report(source)
    assert report["nodes"]["1"] == {"gear_ratio": 36, "cpr": 16384, "pole_pairs": 14}
    assert report["property_observations"][0]["kind"] == "f32"
    assert report["property_observations"][1]["kind"] == "s32"


def test_replay_rejects_reported_properties_that_disagree_with_original_frames():
    from robot_safecontrol_moveit.joint_table_readonly import replay_readonly_report

    with ZipFile(ARCHIVE.with_name("attended_positioning_j1_fault_20261029.zip")) as archive:
        report = json.loads(archive.read("j1-fault-01.json"))
    report["properties"]["encoder_error"] = 0
    with pytest.raises(ValueError, match="encoder_error"):
        replay_readonly_report(json.dumps(report).encode())


def test_replay_preserves_actual_missing_property_reply_and_partial_values():
    from robot_safecontrol_moveit.joint_table_readonly import replay_readonly_report

    with ZipFile(ARCHIVE) as archive:
        source = archive.read("j1-recovery-readonly.json")
    report = replay_readonly_report(source)
    assert report["status"] == "replayed_incomplete"
    assert report["source_status"] == "readonly_failed"
    assert report["nodes"]["1"]["hw_version"] == 220905
    assert report["nodes"]["1"]["fw_version"] == 16843008
    assert report["nodes"]["1"]["zero_offset_counts"] == 3357
    assert "stored_turns_counts" not in report["nodes"]["1"]
    assert report["incomplete_reads"][0]["name"] == "stored_turns_counts"
    assert report["incomplete_reads"][0]["address"] == 31206
    assert report["incomplete_reads"][0]["response_bytes"] == 0
    assert report["device_sample_time_known"] is report["hardware_qualified"] is False


@pytest.mark.parametrize("member,properties,feedback,status", [
    ("nineaxis-static-readonly.json", 261, 918, "replayed_completed"),
    ("nineaxis-encoder-readonly.json", 13, 9, "replayed_incomplete"),
    ("revolute-encoder-readonly.json", 104, 832, "replayed_completed"),
])
def test_actual_scoped_capture_matches_raw_values_and_host_time_intervals(member, properties, feedback, status):
    from robot_safecontrol_moveit.joint_table_readonly import replay_readonly_archive

    with ZipFile(POSITIONING_ARCHIVE) as archive:
        recorded = json.loads(archive.read(member))
    report = replay_readonly_archive(POSITIONING_ARCHIVE, member)
    assert report["status"] == status
    assert len(report["property_observations"]) == properties
    assert len(report["feedback_observations"]) == feedback
    for capture, raw in zip(recorded["property_observations"], report["property_observations"]):
        for name in ("node_id", "name", "address", "kind", "value"):
            assert capture[name] == raw[name]
        assert capture["request_s"] <= raw["request_s"] <= raw["received_s"] <= capture["received_s"]
    assert report["hardware_qualified"] is report["device_sample_time_known"] is False


def test_current_device_identity_and_configuration_remain_observable():
    from robot_safecontrol_moveit.joint_table_readonly import replay_readonly_archive

    report = replay_readonly_archive(POSITIONING_ARCHIVE, "nineaxis-static-readonly.json")
    assert set(report["nodes"]) == set(map(str, range(1, 10)))
    assert {value["serial_number_l"] for value in report["nodes"].values()} == {1955}
    assert {value["can_timeout_ms"] for value in report["nodes"].values()} == {0}
    assert report["nodes"]["1"]["circular_min_deg"] == -36000
    assert report["nodes"]["1"]["circular_max_deg"] == 1000
    assert report["nodes"]["1"]["zero_offset_counts"] == 11145
    assert report["nodes"]["5"]["circular_min_deg"] == -180
    assert report["nodes"]["5"]["circular_max_deg"] == 180


def test_replay_keeps_initial_snapshot_with_later_original_property_reads():
    from robot_safecontrol_moveit.joint_table_readonly import replay_readonly_report

    with ZipFile(SAMPLING_ARCHIVE) as archive:
        source = archive.read("j1-continuous-counters.json")
    report = replay_readonly_report(source)
    assert report["initial_nodes"]["1"]["output_pos_deg"] == -0.03582596778869629
    assert report["nodes"]["1"]["output_pos_deg"] == -0.03532052040100098
    assert report["hardware_qualified"] is False


def test_actual_counter_batches_match_raw_reads_and_unchanged_configuration():
    from robot_safecontrol_moveit.joint_table_readonly import replay_readonly_archive

    report = replay_readonly_archive(SAMPLING_ARCHIVE, "j1-continuous-counters.json")
    assert len(report["encoder_samples"]) == 89
    assert len(report["property_observations"]) == 1440
    assert len(report["feedback_observations"]) == 90
    for sample in report["encoder_samples"]:
        assert sample["configuration_matches"] is True
        assert len(sample["observations"]) == 16
        assert sample["values"]["zero_offset_counts"] == 11145
        assert sample["values"]["encoder_turn_abs"] == 0
        assert 11000 < sample["values"]["encoder_pos_abs"] < 11200
        assert all(sample["started_s"] <= row["request_s"] <= row["received_s"] <= sample["finished_s"]
                   for row in sample["observations"])
    assert report["initial_nodes"]["1"]["encoder_pos_abs"] == 11086
    assert report["nodes"]["1"]["encoder_pos_abs"] == 11087
    assert report["hardware_qualified"] is False


@pytest.mark.parametrize("change", ["value", "time", "configuration", "ledger"])
def test_counter_replay_rejects_batch_or_ledger_mismatch(change):
    from robot_safecontrol_moveit.joint_table_readonly import replay_readonly_report

    with ZipFile(SAMPLING_ARCHIVE) as archive:
        recorded = json.loads(archive.read("j1-continuous-counters.json"))
    if change == "value":
        recorded["encoder_samples"][0]["values"]["encoder_pos_abs"] += 1
    elif change == "time":
        recorded["encoder_samples"][0]["finished_s"] = recorded["encoder_samples"][0]["started_s"]
    elif change == "configuration":
        recorded["encoder_samples"][0]["configuration_matches"] = False
    else:
        recorded["property_observations"][20]["value"] += 1
    with pytest.raises(ValueError, match="连续采集|属性读取记录"):
        replay_readonly_report(json.dumps(recorded).encode())


def test_readonly_replay_rejects_an_actual_motion_record():
    from robot_safecontrol_moveit.joint_table_readonly import replay_readonly_report

    with ZipFile(ARCHIVE.with_name("attended_positioning_j1_fault_20261029.zip")) as archive:
        frames = json.loads(archive.read("attended-motion-02.frames.json"))
    with pytest.raises(ValueError, match="运动或参数写入"):
        replay_readonly_report(json.dumps({"frames": frames}).encode())


def test_quick_feedback_replay_requires_the_manufacturer_zero_request():
    from robot_safecontrol_moveit.joint_table_readonly import replay_readonly_report

    with ZipFile(POSITIONING_ARCHIVE) as archive:
        recorded = json.loads(archive.read("nineaxis-static-readonly.json"))
    parsed = SERIAL_CAN_FRAME.parse(bytes.fromhex(recorded["frames"][0]["raw_hex"]))
    parsed.data = encode_property_read(1, 0, "u32")
    recorded["frames"][0]["raw_hex"] = SERIAL_CAN_FRAME.build(parsed).hex()
    with pytest.raises(ValueError, match="快速读取"):
        replay_readonly_report(json.dumps(recorded).encode())


def test_archive_hash_mismatch_is_rejected_before_replay(tmp_path):
    from robot_safecontrol_moveit.joint_table_readonly import replay_readonly_archive

    damaged = tmp_path / "damaged.zip"
    with ZipFile(POSITIONING_ARCHIVE) as archive, ZipFile(damaged, "x") as destination:
        destination.writestr("provenance.json", archive.read("provenance.json"))
        destination.writestr("nineaxis-static-readonly.json", archive.read("nineaxis-static-readonly.json") + b" ")
    with pytest.raises(ValueError, match="校验失败"):
        replay_readonly_archive(damaged, "nineaxis-static-readonly.json")


@pytest.mark.parametrize("arguments,message", [
    (["--scope", "encoder", "--joints", "J1", "J1"], "不能重复"),
    (["--scope", "encoder", "--fields", "gear_ratio", "gear_ratio"], "不能重复"),
    (["--scope", "position", "--fields", "gear_ratio"], "仅适用于"),
    (["--scope", "encoder", "--fields", "encoder_ready"], "invalid choice"),
    (["--scope", "encoder", "--sample-fields", "encoder_pos_abs", "encoder_pos_abs"], "不能重复"),
    (["--scope", "position", "--sample-fields", "encoder_pos_abs"], "仅适用于"),
    (["--scope", "servo", "--joints", "J4", "J4"], "不能重复"),
    (["--scope", "servo", "--fields", "gear_ratio"], "仅适用于"),
    (["--scope", "servo", "--sample-fields", "encoder_pos_abs"], "仅适用于"),
])
def test_invalid_read_selection_is_rejected_before_device_creation(tmp_path, arguments, message):
    root = Path(__file__).resolve().parents[1]
    output = tmp_path / "rejected.json"
    result = subprocess.run([sys.executable, "-m", "robot_safecontrol_moveit.joint_table_readonly",
                             "--root", str(root), "--port", str(tmp_path / "absent-device"),
                             "--output", str(output), *arguments], cwd=root, env=os.environ.copy(),
                            text=True, capture_output=True)
    assert result.returncode == 2
    assert message in result.stderr
    assert not output.exists()


def test_current_original_evidence_only_contains_read_commands():
    with ZipFile(ARCHIVE) as archive:
        index = json.loads(archive.read("provenance.json"))
        assert index["motion_commands_sent"] == 0
        assert len(index["files"]) == 3
        for entry in index["files"]:
            content = archive.read(entry["member"])
            assert sha256(content).hexdigest() == entry["sha256"]
            assert len(content) == entry["bytes"]
            report = json.loads(content)
            assert report["motion_commands_sent"] == 0
            for frame in report["frames"]:
                if frame["direction"] == "tx":
                    assert SERIAL_CAN_FRAME.parse(bytes.fromhex(frame["raw_hex"])).frame_id & 0x1F == CMD_PROPERTY_READ


def test_actual_j1_recovery_configuration_and_remaining_gap():
    with ZipFile(ARCHIVE) as archive:
        report = json.loads(archive.read("j1-recovery-readonly.json"))
    properties = report["properties"]["J1"]
    assert properties["multi_circle_enabled"] == 0
    assert properties["circular_limit_enabled"] == 1
    assert properties["circular_min_deg"] == -36000
    assert properties["circular_max_deg"] == 0
    assert report["status"] == "readonly_failed"
    assert report["reading_property"] == "stored_turns_counts"
    assert "stored_turns_counts" not in properties
    assert abs(report["samples"][0]["state"]["angle_deg"]) < .01


def test_actual_device_position_settings_are_not_hardware_qualification():
    with ZipFile(ARCHIVE) as archive:
        report = json.loads(archive.read("readonly-position-settings.json"))
    assert report["status"] == "readonly_completed"
    assert len(report["samples"]) == 981
    assert report["hardware_qualified"] is False
    for joint, properties in report["properties"].items():
        assert properties["axis_state"] == 8
        assert properties["axis_error"] == 0
        assert properties["traj_mode"] == 1
        assert properties["trap_acceleration_rpm_s"] == 30
        expected = 24 if joint in {"J2", "J3", "J4", "J5", "J6"} else 33.32999801635742
        assert properties["trap_velocity_rpm"] == expected


def test_j1_authorized_multi_circle_frames_only_enable_and_save():
    writes = []
    with ZipFile(MULTI_CIRCLE_ARCHIVE) as archive:
        index = json.loads(archive.read("provenance.json"))
        assert len(index["files"]) == 6
        for entry in index["files"]:
            content = archive.read(entry["member"])
            assert sha256(content).hexdigest() == entry["sha256"]
            assert len(content) == entry["bytes"]
            report = json.loads(content)
            for frame in report.get("frames", []):
                if frame["direction"] != "tx":
                    continue
                parsed = SERIAL_CAN_FRAME.parse(bytes.fromhex(frame["raw_hex"]))
                assert parsed.frame_id >> 5 == 1
                if parsed.frame_id & 0x1F != CMD_PROPERTY_READ:
                    writes.append((parsed.frame_id & 0x1F, parsed.data))
    assert writes == [(CMD_PROPERTY_WRITE, encode_property_write(1, 31207, 1, "u32")),
                      (CMD_SYSTEM, encode_system(1, SYSTEM_ORDER_SAVE))]


def test_j1_enabled_flag_is_separate_from_zero_and_retention_verification():
    with ZipFile(MULTI_CIRCLE_ARCHIVE) as archive:
        enabled = json.loads(archive.read("j1-multi-circle-enable.json"))
        verified = json.loads(archive.read("j1-multi-circle-verification.json"))
        reference = json.loads(archive.read("j1-user-reference.json"))
    before, after = enabled["properties_before"], verified["properties"]
    assert before["multi_circle_enabled"] == 0
    assert after["multi_circle_enabled"] == 1
    assert enabled["save_call_returned"] is True
    assert verified["status"] == "readonly_completed"
    assert after["encoder_turn_abs"] == -1594
    assert after["battery_voltage"] == 3.9706287384033203
    assert after["axis_error"] == after["encoder_error"] == 0
    for name in ("zero_offset_counts", "circular_min_deg", "circular_max_deg", "circular_limit_enabled"):
        assert after[name] == before[name]
    assert len(verified["samples"]) >= 60
    assert all(abs(sample["pos_deg"]) < .01 for sample in verified["samples"])
    assert verified["power_cycle_retention_verified"] is False
    assert verified["zero_recovery_verified"] is False
    assert enabled["zero_writes_sent"] == enabled["motion_commands_sent"] == 0
    assert enabled["stop_latch_unchanged"] is True
    assert reference["position_new_mm"] == 500 - reference["distance_to_original_585mm_endpoint_mm"] == 28
    assert reference["zero_reference_approximate"] is True
    assert reference["device_to_model_coordinate_verified"] is False


def test_authorized_j1_retreat_keeps_device_limits_and_stationary_rejections():
    with ZipFile(RETREAT_ARCHIVE) as archive:
        index = json.loads(archive.read("provenance.json"))
        for entry in index["files"]:
            content = archive.read(entry["member"])
            assert sha256(content).hexdigest() == entry["sha256"]
            assert len(content) == entry["bytes"]
        original = archive.read("preflight_20261029T101049.json")
        raw = json.loads(original)
        review = json.loads(archive.read("j1-retreat28-review.json"))
        authorization = json.loads(archive.read("j1-retreat28-authorization.json"))
    assert authorization["requested_delta_mm"] == review["requested_delta_mm"] == -28
    assert authorization["other_joints_authorized"] is False
    assert authorization["device_zero_write_authorized"] is False
    assert review["source_sha256"] == sha256(original).hexdigest()
    assert raw["properties"]["limit_min_deg"] == -36000
    assert raw["properties"]["limit_max_deg"] == 0
    assert review["proposed_raw_target_deg"] > raw["properties"]["limit_max_deg"]
    assert review["longest_stationary_s"] < review["required_stationary_s"] == .5
    assert review["windows_above_stopped_rpm"] == 82
    assert review["full_velocity_windows"] == 95
    assert review["rejection_reasons"] == ["target_outside_current_device_limits",
                                          "initial_stationary_duration_not_met"]
    assert review["motion_commands_sent"] == review["parameter_writes_sent"] == 0
    for frame in raw["frames"]:
        if frame["direction"] == "tx":
            parsed = SERIAL_CAN_FRAME.parse(bytes.fromhex(frame["raw_hex"]))
            assert parsed.frame_id >> 5 == 1
            assert parsed.frame_id & 0x1F == CMD_PROPERTY_READ
