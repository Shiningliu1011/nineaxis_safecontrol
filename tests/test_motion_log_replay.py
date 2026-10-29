import json
from hashlib import sha256
from pathlib import Path
from zipfile import ZipFile

import pytest

from robot_safecontrol_moveit.motion_log_replay import replay_motion_log

ARCHIVE = Path(__file__).with_name("data") / "joint_motion_logs_20261028.zip"
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
