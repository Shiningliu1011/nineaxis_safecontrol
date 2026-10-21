from pathlib import Path
import shutil
import time

import pytest

from robot_safecontrol_moveit.static_experiment_capacity import (
    CapacityBudget, RecordingBudget, RecordingBudgetExhausted, RecordingLimits,
    assess_capacity, lifecycle_outcome_valid, require_capacity, require_formal_capacity,
)
from robot_safecontrol_moveit.static_experiment_config import content_hash
from robot_safecontrol_moveit.static_experiment_recording import write_json


def test_recording_limits_include_timeout_settle_stop_and_one_cycle_per_execution_step():
    limits = RecordingLimits(timeout_s=420., settle_s=2., period_s=.1)
    report = limits.manifest()
    assert report["startup_order"] == ["controller", "plant"]
    assert report["maximum_recording_s"] == 473.
    assert report["maximum_feedback_records"] == 236500
    assert report["maximum_candidate_records"] == 4732
    assert report["maximum_trials_per_candidate"] == 9


def test_recording_budget_stops_new_tasks_at_count_or_local_deadline():
    limits = RecordingLimits(timeout_s=.001, settle_s=0., period_s=.1)
    budget = RecordingBudget(limits)
    for _ in range(limits.manifest()["maximum_candidate_records"]):
        budget.begin("calculation")
    with pytest.raises(RecordingBudgetExhausted, match="calculation"):
        budget.begin("calculation")
    assert budget.snapshot()["started_tasks"]["calculation"] == 513
    expired = RecordingBudget(limits)
    expired.start(time.monotonic() - limits.manifest()["maximum_recording_s"])
    with pytest.raises(RecordingBudgetExhausted, match="deadline"):
        expired.begin("feedback")
    assert expired.snapshot()["started_tasks"] == {}


def test_lifecycle_accepts_only_expected_termination_and_successful_workers():
    assert lifecycle_outcome_valid(
        {"termination": "completed", "worker_exit_codes": [0, 0]}, expected_termination="completed",
    )
    for result in (
        {"termination": "fault", "worker_exit_codes": [0, 0]},
        {"termination": "completed", "worker_exit_codes": [1, 0]},
        {"termination": "completed", "worker_exit_codes": [0]},
        {"termination": "interrupted", "worker_exit_codes": [0, 0]},
        {"termination": "budget_exhausted", "worker_exit_codes": [0, 0]},
        {"termination": "timeout", "worker_exit_codes": [0, 0]},
    ):
        assert not lifecycle_outcome_valid(result, expected_termination="completed")
    assert lifecycle_outcome_valid(
        {"termination": "timeout", "worker_exit_codes": [0, 0]}, expected_termination="timeout",
    )


def test_capacity_includes_retained_retries_and_one_serial_working_directory():
    budget = CapacityBudget(
        planned_runs=1080, retained_run_bytes=100_000_000,
        retry_runs=108, transient_run_bytes=2_000_000_000,
        derived_workspace_bytes=500_000_000, provenance_bytes=100_000_000,
        reserve_bytes=10_000_000_000,
    )
    report = assess_capacity(budget, available_bytes=130_000_000_000, upper_bound_validated=True)
    assert report["required_free_bytes"] == 131_400_000_000
    assert report["shortfall_bytes"] == 1_400_000_000
    assert report["status"] == "fail"
    with pytest.raises(ValueError, match="空间"):
        require_capacity(report)


def test_sample_projection_does_not_admit_collection_when_space_is_available():
    budget = CapacityBudget(1080, 49_317_246, 108, 1_130_258_432, 49_937_480,
                            100_000_000, 10_000_000_000)
    report = assess_capacity(budget, available_bytes=80_980_201_472)
    assert report["shortfall_bytes"] == 0
    assert report["status"] == "insufficient_evidence"
    with pytest.raises(ValueError, match="样本外推"):
        require_capacity(report)
    constrained = assess_capacity(budget, available_bytes=100)
    assert constrained["status"] == "insufficient_evidence"
    assert constrained["shortfall_bytes"] > 0


def test_formal_capacity_requires_matching_manifest_and_evidence(tmp_path):
    source = Path(__file__).resolve()
    manifest = {
        "source_sha256": {str(source): content_hash(source)},
        "runs": [{"id": "first"}, {"id": "second"}],
    }
    write_json(tmp_path / "formal_manifest.json", manifest)
    with pytest.raises(ValueError, match="缺少 capacity_plan"):
        require_formal_capacity(tmp_path, manifest)
    report = assess_capacity(CapacityBudget(2, 1000, 1, 1000, 1000, 1000, 1000),
                             available_bytes=100_000, upper_bound_validated=True)
    report.update(
        formal_manifest_sha256=content_hash(tmp_path / "formal_manifest.json"),
        source_sha256=manifest["source_sha256"],
        evidence_sha256={str(source): content_hash(source)},
    )
    write_json(tmp_path / "capacity_plan.json", report)
    assert require_formal_capacity(tmp_path, manifest)["status"] == "pass"
    write_json(tmp_path / "formal_manifest.json", {**manifest, "changed": True})
    with pytest.raises(ValueError, match="清单"):
        require_formal_capacity(tmp_path, manifest)


def test_capacity_resume_charges_only_remaining_runs_and_reserved_retries(tmp_path):
    source = Path(__file__).resolve()
    manifest = {
        "source_sha256": {str(source): content_hash(source)},
        "runs": [{"id": "first"}, {"id": "second"}],
    }
    write_json(tmp_path / "formal_manifest.json", manifest)
    report = assess_capacity(CapacityBudget(2, 10_000, 1, 1000, 1000, 1000, 1000),
                             available_bytes=100_000, upper_bound_validated=True)
    report.update(
        formal_manifest_sha256=content_hash(tmp_path / "formal_manifest.json"),
        source_sha256=manifest["source_sha256"],
        evidence_sha256={str(source): content_hash(source)},
    )
    write_json(tmp_path / "capacity_plan.json", report)
    run = tmp_path / "runs/first"
    run.mkdir(parents=True)
    write_json(run / "result.json", {"termination": "interrupted"})
    current = require_formal_capacity(tmp_path, manifest)
    assert current["budget"]["planned_runs"] == 1
    assert current["retained_bytes"] == 20_000
    write_json(run / "result.json", {"termination": "budget_exhausted"})
    with pytest.raises(ValueError, match="耗尽记录预算"):
        require_formal_capacity(tmp_path, manifest)


def test_formal_admission_rechecks_real_free_space(tmp_path):
    source = Path(__file__).resolve()
    manifest = {
        "source_sha256": {str(source): content_hash(source)},
        "runs": [{"id": "first"}],
    }
    write_json(tmp_path / "formal_manifest.json", manifest)
    total = shutil.disk_usage(tmp_path).total
    report = assess_capacity(CapacityBudget(1, total + 1, 0, 0, 0, 0, 0),
                             available_bytes=2 * total, upper_bound_validated=True)
    report.update(
        formal_manifest_sha256=content_hash(tmp_path / "formal_manifest.json"),
        source_sha256=manifest["source_sha256"],
        evidence_sha256={str(source): content_hash(source)},
    )
    write_json(tmp_path / "capacity_plan.json", report)
    with pytest.raises(ValueError, match="空间不足"):
        require_formal_capacity(tmp_path, manifest)
