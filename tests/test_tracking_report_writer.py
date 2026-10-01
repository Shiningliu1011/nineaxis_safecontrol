"""Background persistence state, failure visibility and sealed sample identity."""

import hashlib
import json
from pathlib import Path

import pytest

from robot_safecontrol_moveit.tracking_evaluator import TrackingEvaluator
from robot_safecontrol_moveit.tracking_report_writer import TrackingReportWriter, write_tracking_bundle


def test_real_background_submission_is_idempotent_and_drains_on_close(tmp_path):
    evaluator = TrackingEvaluator()
    for index in range(4000):
        evaluator.update({}, wall_time_s=index * .01)
    evaluator.finish("held")
    target = str(tmp_path / "tracking.md")
    writer = TrackingReportWriter()
    try:
        writer.submit(evaluator, target)
        assert writer.status() == dict(state="writing", path=target, error=None)
        writer.submit(evaluator, target)
        with pytest.raises(RuntimeError):
            evaluator.update({}, wall_time_s=1.)
    finally:
        writer.close()
    assert writer.status() == dict(state="saved", path=target, error=None)
    summary = json.loads(Path(target).with_suffix(".json").read_text())
    samples = Path(target).with_suffix(".samples.json").read_bytes()
    assert summary["sample_data_sha256"] == hashlib.sha256(samples).hexdigest()
    assert json.loads(samples)["termination"] == "held"
    assert len(json.loads(samples)["samples"]) == 4000
    identity = Path(target).stat().st_mtime_ns
    writer.submit(evaluator, target)
    assert Path(target).stat().st_mtime_ns == identity


def test_failure_is_observable_and_does_not_claim_saved(tmp_path):
    blocker = tmp_path / "not-a-directory"
    blocker.write_text("keep me")
    evaluator = TrackingEvaluator()
    evaluator.finish("interrupted")
    writer = TrackingReportWriter()
    writer.submit(evaluator, str(blocker / "tracking.md"))
    writer.close()
    assert writer.status()["state"] == "failed"
    assert writer.status()["error"]
    assert blocker.read_text() == "keep me"


def test_unfinished_sampling_is_not_handed_to_background_writer(tmp_path):
    writer = TrackingReportWriter()
    try:
        with pytest.raises(ValueError, match="finish sampling"):
            writer.submit(TrackingEvaluator(), str(tmp_path / "tracking.md"))
        assert writer.status()["state"] == "idle"
    finally:
        writer.close()


def test_write_failure_does_not_replace_existing_artifacts(tmp_path):
    evaluator = TrackingEvaluator()
    evaluator.finish("held")
    path = tmp_path / "tracking.md"
    write_tracking_bundle(evaluator, str(path))
    destinations = (path, path.with_suffix(".json"), path.with_suffix(".samples.json"))
    before = {p: p.read_bytes() for p in destinations}
    original_mode = tmp_path.stat().st_mode
    tmp_path.chmod(0o500)
    try:
        with pytest.raises(PermissionError):
            write_tracking_bundle(evaluator, str(path))
    finally:
        tmp_path.chmod(original_mode)
    assert {p: p.read_bytes() for p in destinations} == before
    assert not list(tmp_path.glob(".tracking-*"))
