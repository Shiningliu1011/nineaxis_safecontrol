from pathlib import Path

import pytest

from robot_safecontrol_moveit.static_experiment_capacity import (
    CapacityBudget, assess_capacity, require_capacity, require_formal_capacity,
)
from robot_safecontrol_moveit.static_experiment_config import content_hash
from robot_safecontrol_moveit.static_experiment_recording import write_json
from robot_safecontrol_moveit.static_experiment_volumes import (
    allocate_volume_budgets, capacity_run_directory, filesystem_identity, require_volume_capacity,
)


def test_allocation_reserves_workspace_and_system_space_on_each_volume():
    budget = CapacityBudget(4, 100, 0, 50, 20, 10, 20)
    volumes = [
        {"root": "/first", "filesystem_uuid": "first", "available_bytes": 400},
        {"root": "/second", "filesystem_uuid": "second", "available_bytes": 300},
    ]
    result = allocate_volume_budgets(["a", "b", "c", "d"], budget, volumes)
    assert result[0]["run_ids"] == ["a", "b", "c"]
    assert result[0]["budget"]["planned_runs"] == 3
    assert result[0]["required_free_bytes"] == 400
    assert result[1]["run_ids"] == ["d"]
    assert result[1]["required_free_bytes"] == 200
    assert capacity_run_directory(Path("/campaign"), {"volumes": result}, "d") == Path("/second/d")


def test_allocation_rejects_double_counting_and_total_shortfall():
    budget = CapacityBudget(4, 100, 0, 50, 20, 10, 20)
    with pytest.raises(ValueError, match="文件系统"):
        allocate_volume_budgets(["a", "b", "c", "d"], budget, [
            {"root": "/first", "filesystem_uuid": "same", "available_bytes": 400},
            {"root": "/second", "filesystem_uuid": "same", "available_bytes": 400},
        ])
    with pytest.raises(ValueError, match="容量不足"):
        allocate_volume_budgets(["a", "b", "c", "d"], budget, [
            {"root": "/first", "filesystem_uuid": "first", "available_bytes": 400},
        ])


def test_filesystem_identity_describes_real_output_directory(tmp_path):
    identity = filesystem_identity(tmp_path)
    assert identity["root"] == str(tmp_path.resolve())
    assert identity["device"] == tmp_path.stat().st_dev
    assert identity["available_bytes"] > 0
    assert identity["filesystem_uuid"]


def test_engineering_budget_is_identified_separately_from_unconditional_bound():
    report = assess_capacity(
        CapacityBudget(4, 100, 0, 50, 20, 10, 20),
        available_bytes=600, engineering_validated=True,
    )
    assert report["status"] == "pass"
    assert report["is_validated_capacity_upper_bound"] is False
    assert report["is_validated_engineering_budget"] is True
    assert report["capacity_basis"] == "validated_engineering_envelope"
    require_capacity(report)


def test_formal_engineering_admission_requires_reproducible_evidence(tmp_path):
    source = Path(__file__).resolve()
    manifest = {
        "source_sha256": {str(source): content_hash(source)},
        "runs": [{"id": "first"}],
    }
    write_json(tmp_path / "formal_manifest.json", manifest)
    report = assess_capacity(
        CapacityBudget(1, 1000, 0, 1000, 1000, 1000, 1000),
        available_bytes=100_000, engineering_validated=True,
    )
    report.update(
        formal_manifest_sha256=content_hash(tmp_path / "formal_manifest.json"),
        source_sha256=manifest["source_sha256"],
        evidence_sha256={str(source): content_hash(source)},
    )
    write_json(tmp_path / "capacity_plan.json", report)
    with pytest.raises(ValueError, match="工程容量.*证据"):
        require_formal_capacity(tmp_path, manifest)


def test_record_envelope_counts_keyframes_and_serial_slow_writes():
    from robot_safecontrol_moveit.static_experiment_engineering_capacity import (
        bounded_stream_counts, encoded_stream_bytes,
    )
    from robot_safecontrol_moveit.static_experiment_capacity import RecordingLimits

    counts = bounded_stream_counts(RecordingLimits(420., 2., .1))
    assert counts["plant/feedback"] == 236500
    assert counts["plant/recording_delay"] == 23651
    assert counts["controller/candidate"] == 4732
    assert counts["controller/transition"] == 14197
    assert encoded_stream_bytes(129, keyframe=100, delta=10) == 1470


def test_engineering_model_rejects_missing_real_stream_evidence():
    from robot_safecontrol_moveit.static_experiment_engineering_capacity import engineering_model
    from robot_safecontrol_moveit.static_experiment_capacity import RecordingLimits

    with pytest.raises(ValueError, match="真实记录"):
        engineering_model([], RecordingLimits(420., 2., .1))


def test_volume_allocation_accounts_for_each_method_record_structure():
    budget = CapacityBudget(4, 100, 1, 50, 20, 10, 20)
    result = allocate_volume_budgets(
        ["qp1", "sns1", "qp2", "sns2"], budget,
        [
            {"root": "/first", "filesystem_uuid": "first", "available_bytes": 350},
            {"root": "/second", "filesystem_uuid": "second", "available_bytes": 250},
        ],
        run_retained_bytes={"qp1": 100, "sns1": 50, "qp2": 100, "sns2": 50},
    )
    assert result[0]["run_ids"] == ["qp1", "sns1"]
    assert result[0]["retained_bytes"] == 250
    assert result[1]["run_ids"] == ["qp2", "sns2"]
    assert result[1]["retained_bytes"] == 150
    assert sum(row["required_free_bytes"] for row in result) == 600


def test_volume_resume_preserves_completed_data_and_checks_destination(tmp_path):
    data = tmp_path / "data"
    data.mkdir()
    (tmp_path / "runs").mkdir()
    budget = CapacityBudget(2, 16384, 0, 1000, 1000, 1000, 1000)
    volumes = allocate_volume_budgets(
        ["first", "second"], budget, [filesystem_identity(data)],
        run_retained_bytes={"first": 8192, "second": 16384},
    )
    report = assess_capacity(
        CapacityBudget(2, 16384, 0, 1000, 1000, 1000, 1000, planned_retained_bytes=24576),
        available_bytes=100_000, upper_bound_validated=True,
    )
    report["volumes"] = volumes
    manifest = {"runs": [{"id": "first"}, {"id": "second"}]}
    first = data / "first"
    first.mkdir()
    write_json(first / "result.json", {"termination": "timeout"})
    (tmp_path / "runs/first").symlink_to(first, target_is_directory=True)
    current = require_volume_capacity(tmp_path, manifest, report)
    assert current["volumes"][0]["budget"]["planned_runs"] == 1
    assert current["volumes"][0]["retained_bytes"] == 16384
    assert (first / "result.json").is_file()
    (tmp_path / "runs/second").symlink_to(first, target_is_directory=True)
    with pytest.raises(ValueError, match="输出位置"):
        require_volume_capacity(tmp_path, manifest, report)
