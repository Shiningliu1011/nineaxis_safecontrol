"""Background persistence state, failure visibility and sealed sample identity."""

import hashlib
import json
from pathlib import Path
from threading import Event, get_ident

import pytest

from robot_safecontrol_moveit.tracking_evaluator import TrackingEvaluator
from robot_safecontrol_moveit.tracking_report_writer import TrackingReportWriter, write_tracking_bundle


def test_submission_returns_while_writer_is_blocked_and_joins_only_on_close(tmp_path):
    entered, release = Event(), Event()
    thread_ids = []
    evaluator = TrackingEvaluator()
    evaluator.update({}, wall_time_s=0.)
    evaluator.finish("held")
    target = str(tmp_path / "tracking.md")

    def blocked_write(ev, path):
        thread_ids.append(get_ident())
        entered.set()
        assert release.wait(5.)
        return write_tracking_bundle(ev, path)

    writer = TrackingReportWriter(write=blocked_write)
    try:
        writer.submit(evaluator, target)
        assert entered.wait(2.)
        assert thread_ids != [get_ident()]
        assert writer.status() == dict(state="writing", path=target, error=None)
        assert not Path(target).exists()
        writer.submit(evaluator, target)  # the same finish notification is idempotent
        with pytest.raises(RuntimeError):
            evaluator.update({}, wall_time_s=1.)
    finally:
        release.set()
        writer.close()
    assert writer.status() == dict(state="saved", path=target, error=None)
    summary = json.loads(Path(target).with_suffix(".json").read_text())
    samples = Path(target).with_suffix(".samples.json").read_bytes()
    assert summary["sample_data_sha256"] == hashlib.sha256(samples).hexdigest()
    assert json.loads(samples)["termination"] == "held"
    assert len(thread_ids) == 1


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


def test_write_failure_does_not_replace_existing_artifacts(tmp_path, monkeypatch):
    evaluator = TrackingEvaluator()
    evaluator.finish("held")
    path = tmp_path / "tracking.md"
    write_tracking_bundle(evaluator, str(path))
    destinations = (path, path.with_suffix(".json"), path.with_suffix(".samples.json"))
    before = {p: p.read_bytes() for p in destinations}
    original = Path.write_text

    def fail_staging(self, *args, **kwargs):
        if self.name == "1":
            raise OSError("disk write failed")
        return original(self, *args, **kwargs)

    monkeypatch.setattr(Path, "write_text", fail_staging)
    with pytest.raises(OSError, match="disk write failed"):
        write_tracking_bundle(evaluator, str(path))
    assert {p: p.read_bytes() for p in destinations} == before
    assert not list(tmp_path.glob(".tracking-*"))
