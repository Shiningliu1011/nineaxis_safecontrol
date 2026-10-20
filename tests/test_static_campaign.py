from collections import Counter
from copy import deepcopy
from pathlib import Path
import multiprocessing
import os
import sqlite3
import time
import zipfile

import pytest

from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig, content_hash
from robot_safecontrol_moveit.static_experiment_provenance import (
    environment_identity, relocate_manifest, snapshot_sources, verify_snapshot,
    recording_hashes,
)
from robot_safecontrol_moveit.static_experiment_recording import RunRecorder, finalize_recording, read_stream
from robot_safecontrol_moveit.static_formal_experiments import experiment_rows
from robot_safecontrol_moveit.static_period_measurement import select_period, PERIODS


ROOT = Path(__file__).resolve().parents[1]


def test_formal_plan_has_every_required_paired_configuration():
    rows = experiment_rows()
    assert len(rows) == len({row["id"] for row in rows}) == 1080
    assert Counter(row["method"] for row in rows) == dict.fromkeys("ABCDE", 216)
    pairs = Counter((row["scene"], row["initial"], row["feed_factor"], row["repeat"]) for row in rows)
    assert len(pairs) == 216
    assert set(pairs.values()) == {5}
    assert len({row["configuration"] for row in rows}) == 72


def test_period_selection_requires_complete_measurements_and_all_five_methods():
    rows = [
        {"period_s": period, "method": method, "status": "measured", "samples": 1000,
         "p999_s": .04, "maximum_s": .06, "deadline_misses": 1 if period < .06 else 0}
        for period in PERIODS for method in "ABCDE"
    ]
    result = select_period(rows)
    assert result["status"] == "pass"
    assert result["selected_period_s"] == .05
    assert select_period(rows[:-1])["status"] == "insufficient_evidence"
    delayed = deepcopy(rows)
    for row in delayed:
        if row["method"] == "E":
            row.update(p999_s=.11, maximum_s=.12, deadline_misses=1000)
    assert select_period(delayed)["status"] == "fail"
    assert select_period(delayed)["selected_period_s"] is None
    with pytest.raises(ValueError):
        select_period([*rows, rows[0]])
    invalid = deepcopy(rows)
    invalid[0]["p999_s"] = float("nan")
    with pytest.raises(ValueError):
        select_period(invalid)


def test_source_archive_preserves_actual_files_and_detects_changed_content(tmp_path):
    source = ROOT / "src/robot_safecontrol_moveit/static_experiment.py"
    resource = ROOT / "config/static_experiment_s0.yaml"
    report = snapshot_sources(ROOT, [{
        "source_sha256": {str(source): content_hash(source)},
        "resources": {"configuration": {"path": str(resource), "sha256": content_hash(resource)}},
    }], tmp_path / "provenance")
    assert verify_snapshot(tmp_path / "provenance") == report
    with zipfile.ZipFile(tmp_path / "provenance/sources.zip") as archive:
        assert archive.read(source.relative_to(ROOT).as_posix()) == source.read_bytes()
        assert archive.read(resource.relative_to(ROOT).as_posix()) == resource.read_bytes()
    with zipfile.ZipFile(tmp_path / "provenance/sources.zip", "a") as archive:
        archive.writestr("unexpected.txt", "changed")
    with pytest.raises(ValueError, match="归档身份"):
        verify_snapshot(tmp_path / "provenance")


def test_recorder_commits_each_sample_and_checkpoints_after_sampling(tmp_path):
    database = tmp_path / "records.sqlite3"
    with RunRecorder(database) as recorder:
        for index in range(1500):
            recorder.append("event", monotonic_s=time.monotonic(), values={"sequence": index})
        rows = read_stream(database, "event")
        assert [row["values"]["sequence"] for row in rows] == list(range(1500))
        assert Path(str(database) + "-wal").stat().st_size > 4096 * 1000
    assert not Path(str(database) + "-wal").exists()
    with sqlite3.connect(database) as connection:
        assert connection.execute("PRAGMA integrity_check").fetchone()[0] == "ok"
        assert connection.execute("SELECT COUNT(*) FROM samples").fetchone()[0] == 1500


def test_relocation_preserves_resource_identity_and_original_manifest(tmp_path):
    source = ROOT / "config/static_experiment_s0.yaml"
    original = {
        "resource_root": str(ROOT),
        "resources": {"config": {"path": str(source), "sha256": content_hash(source)}},
        "source_sha256": {str(source): content_hash(source)},
        "outside_root": str(ROOT.parent / "other-resource"),
    }
    manifest = relocate_manifest(original, tmp_path)
    expected = str(tmp_path / source.relative_to(ROOT))
    assert manifest["resources"]["config"] == {"path": expected, "sha256": content_hash(source)}
    assert manifest["source_sha256"] == {expected: content_hash(source)}
    assert manifest["outside_root"] == original["outside_root"]
    assert original["resources"]["config"]["path"] == str(source)
    assert relocate_manifest({"legacy": True}, tmp_path) == {"legacy": True}


def test_legacy_scene_relocation_keeps_geometry_roles_and_hashes(tmp_path):
    config = ExperimentConfig.load(ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT)
    original = config.manifest()
    original.pop("resource_root")
    relocated = relocate_manifest(original, tmp_path)
    for first, second in zip(
        original["scene"]["self_collision"]["meshes"],
        relocated["scene"]["self_collision"]["meshes"],
    ):
        assert second == {
            **first, "path": str(tmp_path / Path(first["path"]).relative_to(ROOT)),
        }
    assert relocated["scene"]["obstacles"] == original["scene"]["obstacles"]
    assert relocated["execution_limits"] == original["execution_limits"]
    assert "resource_root" not in original
    changed = deepcopy(original)
    changed["config"]["collision_mesh_manifest"] = "different/collision_geometry_sources.json"
    with pytest.raises(ValueError, match="资源根目录"):
        relocate_manifest(changed, tmp_path)


def test_device_identity_is_repeatable_on_the_actual_environment():
    first, second = environment_identity(), environment_identity()
    assert first["sha256"] == second["sha256"]
    assert first["sqlite_version"] == sqlite3.sqlite_version
    assert first["packages"]
    assert first["system_packages"]


def _record_before_process_exit(path):
    recorder = RunRecorder(path)
    for index in range(10):
        recorder.append("event", monotonic_s=time.monotonic(), values={"index": index})
    os._exit(3)


def test_abrupt_process_exit_preserves_committed_wal_records(tmp_path):
    database = tmp_path / "plant.sqlite3"
    process = multiprocessing.get_context("spawn").Process(target=_record_before_process_exit, args=(database,))
    process.start()
    process.join(timeout=30)
    assert process.exitcode == 3
    assert len(read_stream(database, "event")) == 10
    with pytest.raises(ValueError, match="checkpoint"):
        recording_hashes(tmp_path)
    finalize_recording(database)
    assert len(read_stream(database, "event")) == 10
    assert recording_hashes(tmp_path) == {"plant": content_hash(database)}
