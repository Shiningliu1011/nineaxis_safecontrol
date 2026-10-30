import json
from hashlib import sha256
from pathlib import Path
from zipfile import ZipFile

import pytest

from robot_safecontrol_moveit.motion_log_replay import replay_encoder_probe, replay_motion_log

ARCHIVE = Path(__file__).with_name("data") / "joint_motion_logs_20261028.zip"
PROBE_ARCHIVE = ARCHIVE.with_name("j1_bounded_counter_experiments_20261030.zip")
LOGS = (
    ("forward50_20261028T193051", None, "aborted"),
    ("J2_direction_target4_20261028", "velocity_limit", "aborted"),
    ("J5_absolute90_speed246_20261028", None, "aborted"),
    ("J7_absolute60_speed246_20261028", "feedback_position_jump", "aborted"),
    ("J2_absolute10_2p5_20261028", None, "completed"),
    ("J3_absolute10_2p5_20261028", None, "completed"),
    ("J4_absolute10_2p5_20261028", None, "completed"),
    ("J6_absolute10_speed246_20261028", None, "completed"),
    ("J8_absolute10_speed246_20261028", None, "completed"),
    ("J9_absolute10_speed246_20261028", None, "completed"),
    ("J4_absolute9_gentle_20261021", None, "aborted"),
)


def test_archived_original_bytes_match_provenance():
    with ZipFile(ARCHIVE) as archive:
        provenance = json.loads(archive.read("provenance.json"))
        assert len(provenance["files"]) == 22
        for source in provenance["files"]:
            content = archive.read(source["member"])
            assert len(content) == source["bytes"]
            assert sha256(content).hexdigest() == source["sha256"]


@pytest.mark.parametrize("name,reason,terminal", LOGS)
def test_complete_real_records_and_current_monitor(name, reason, terminal):
    with ZipFile(ARCHIVE) as archive:
        result = replay_motion_log(archive.read(f"{name}.jsonl"), archive.read(f"{name}.frames.json"))
    assert result["matched_samples"] > 100
    assert result["recorded_terminal"]["event"] == terminal
    actual = result["current_replay_rejection"]
    assert (actual["reason"] if actual else None) == reason
    assert result["hardware_qualified"] is False
    assert result["device_sample_time_known"] is False
    assert result["physical_stop_independently_verified"] is False
    if name.startswith("forward50"):
        assert result["hold"]["condition_failures"] == {"speed": 268}
        assert result["hold"]["samples"] == 360
        assert result["hold"]["max_settled_s"] == pytest.approx(.330146752001383)
        assert result["recorded_terminal"]["stop_result"] == "external_stop_required"
    if name == "J4_absolute9_gentle_20261021":
        assert len(result["invalid_rx_frames"]) == 1
        assert result["invalid_rx_frames"][0]["raw_hex"] == ""
    else:
        assert not result["invalid_rx_frames"]


def test_bounded_probe_archive_keeps_original_bytes_and_operator_measurements():
    with ZipFile(PROBE_ARCHIVE) as archive:
        provenance = json.loads(archive.read("provenance.json"))
        for entry in provenance["files"]:
            source = archive.read(entry["member"])
            assert len(source) == entry["bytes"]
            assert sha256(source).hexdigest() == entry["sha256"]
        measurement = json.loads(archive.read("operator-measurements.json"))
    assert provenance["motion_commands_sent"] == 4
    assert provenance["other_joint_motion_commands_sent"] == provenance["parameter_writes_sent"] == 0
    assert [row["reading_mm"] for row in measurement["samples"]] == [500, 495, 500]
    assert measurement["resolution_mm"] == 1
    assert measurement["uncertainty_mm"] is None
    assert measurement["absolute_coordinate_accuracy_verified"] is False


@pytest.mark.parametrize("name,count,target", [
    ("j1-outward-1mm", 692, 72.), ("j1-return-0mm", 694, 0.),
    ("j1-outward-5mm", 2179, 360.), ("j1-return-from5mm", 2166, 0.),
])
def test_actual_bounded_J1_motion_matches_commands_counters_and_all_feedback(name, count, target):
    with ZipFile(PROBE_ARCHIVE) as archive:
        report = replay_encoder_probe(*(archive.read(name + suffix)
                                        for suffix in (".jsonl", ".frames.json", ".capture.json")))
    assert report["motion_commands_sent"] == 1
    assert report["other_joint_motion_commands_sent"] == report["parameter_writes_sent"] == 0
    assert report["capture_status"] == report["recorded_terminal"]["event"] == "completed"
    assert report["recorded_terminal"]["target_deg"] == target
    assert report["current_replay_rejection"] is None
    assert report["current_replay_phase"] == "completed"
    assert report["configuration_unchanged"] is True
    assert report["matched_samples"] == report["encoder_samples"] == count
    assert report["feedback_counts_by_node"] == dict.fromkeys(range(1, 10), count)
    relationship = report["counter_relationship"]
    assert 7.9 < relationship["turn_abs_per_reported_rotor_turn"] < 8.1
    assert 16380 < relationship["unwrapped_pos_abs_per_reported_rotor_turn"] < 16388
    assert .09999 < relationship["reported_rotor_turns_per_output_degree"] < .10001
    assert report["final_encoder"]["output_pos_deg"] == pytest.approx(-target, abs=.01)
    assert report["initial_encoder"]["zero_offset_counts"] == report["final_encoder"]["zero_offset_counts"] == 11145
    assert report["hardware_qualified"] is report["physical_stop_independently_verified"] is False


@pytest.fixture(scope="module")
def original_probe_sources():
    with ZipFile(PROBE_ARCHIVE) as archive:
        return tuple(archive.read("j1-outward-1mm" + suffix)
                     for suffix in (".jsonl", ".frames.json", ".capture.json"))


@pytest.mark.parametrize("change,message", [
    ("identity", "原始文件身份"), ("counter", "连续采集"),
    ("ledger", "属性读取"), ("command", "位置命令"),
])
def test_encoder_probe_rejects_modified_evidence(original_probe_sources, change, message):
    source, frames, capture_source = original_probe_sources
    capture = json.loads(capture_source)
    if change == "identity":
        capture["frames_sha256"] = "0" * 64
    elif change == "counter":
        capture["encoder_samples"][0]["values"]["encoder_pos_abs"] += 1
    elif change == "ledger":
        capture["property_observations"][0]["value"] += 1
    else:
        events = [json.loads(line) for line in source.decode().splitlines()]
        command = next(event for event in events if event["event"] == "command_requested")
        command["arguments"]["speed"] -= .01
        source = ("\n".join(json.dumps(event) for event in events) + "\n").encode()
        capture["log_sha256"] = sha256(source).hexdigest()
    with pytest.raises(ValueError, match=message):
        replay_encoder_probe(source, frames, json.dumps(capture).encode())
