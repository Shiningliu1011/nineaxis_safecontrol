import json
from hashlib import sha256
from pathlib import Path
from zipfile import ZipFile

from robot_safecontrol_moveit.drempower_can import (
    CMD_PROPERTY_READ,
    CMD_PROPERTY_WRITE,
    CMD_SYSTEM,
    SYSTEM_ORDER_SAVE,
    encode_property_write,
    encode_system,
)
from robot_safecontrol_moveit.drempower_serial import SERIAL_CAN_FRAME

ARCHIVE = Path(__file__).with_name("data") / "joint_position_readonly_20261029.zip"
MULTI_CIRCLE_ARCHIVE = ARCHIVE.with_name("j1_multi_circle_20261029.zip")
RETREAT_ARCHIVE = ARCHIVE.with_name("j1_retreat28_preflight_20261029.zip")


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
