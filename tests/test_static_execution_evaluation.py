from copy import deepcopy
from dataclasses import asdict, fields
import json
from pathlib import Path
import shutil
import sqlite3
import time

import numpy as np
import pytest

from robot_safecontrol_moveit.oscbf_trajectory import bootstrap_portable
from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig, content_hash
from robot_safecontrol_moveit.static_experiment_evaluation import evaluate_run, evaluation_samples
from robot_safecontrol_moveit.static_experiment_nodes import _resources
from robot_safecontrol_moveit.static_experiment_recording import (
    RunRecorder, iter_records, read_stream, write_json,
)
from robot_safecontrol_moveit.static_experiment_storage import compact_run
from robot_safecontrol_moveit.static_experiment_provenance import validate_run_report
from robot_safecontrol_moveit.static_execution_evaluation import evaluate_execution, arc_statistics, ordered_coverage
from robot_safecontrol_moveit.static_cycle_timing import (
    CYCLE_TIMING_BOUNDARY, LEGACY_CYCLE_TIMING_BOUNDARY, CycleTimings,
    cycle_timing_summary, load_cycle_timing,
)

ROOT = Path(__file__).resolve().parents[1]
bootstrap_portable(ROOT / "portable_oscbf")


def test_actual_processed_motion_and_rejected_admission_have_reconstructable_progress(tmp_path):
    from work.static_control import StaticControl
    from work.static_execution import ExecutionSchedule, ScheduledMotion
    from work.static_motion import ActuatorState

    config = ExperimentConfig.load(ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT)
    path, geometry, planner = _resources(config)
    control = StaticControl(path, geometry, planner, config.values["control"])
    state = ActuatorState(config.initial_position, np.zeros(9), np.zeros(9))
    schedule = ExecutionSchedule(state, planner, .1)
    started = time.monotonic()
    complete_timing = CycleTimings(maximum_records=2)
    candidate = control.propose(state, 0., time_s=.1, executed_feed=0.)
    assert candidate is not None and candidate.certificate.accepted
    command = ScheduledMotion(1, 50, 0., candidate.solution.feed_m_s, candidate.phase, candidate.motion)
    with RunRecorder(tmp_path / "controller.sqlite3") as controller, RunRecorder(tmp_path / "plant.sqlite3") as plant:
        controller.append("candidate", monotonic_s=time.monotonic(), values={
            "trials": control.last_trials, "initial": state.to_record(),
            "reference_progress_m": 0.,
            "geometry": [row.to_record() for row in control.last_measurements],
        })
        for command_id in (1, 2):
            request = ScheduledMotion(
                command_id, 50, 0., candidate.solution.feed_m_s, candidate.phase, candidate.motion,
            )
            controller.append("command", monotonic_s=time.monotonic(), values={
                "command_id": command_id, "scheduled": request.to_record(),
            })
            reason = schedule.queue(request, candidate.certificate)
            plant.append("admission", monotonic_s=time.monotonic(), values={
                "command_id": command_id, "accepted": reason == "accepted", "reason": reason,
                "certificate": asdict(candidate.certificate),
            })
            base_completed = time.monotonic()
            controller.append("cycle", monotonic_s=base_completed, values={
                "command_id": command_id, "accepted": reason == "accepted", "reason": reason,
                "duration_s": base_completed - started, "period_s": .1,
            })
            complete_timing.record(controller.last_sequence, started)
            complete_timing.complete(controller.last_sequence)
            for _ in range(50 if command_id == 1 else 100):
                snapshot = schedule.advance()
                plant.append("feedback", monotonic_s=time.monotonic(),
                             simulation_s=snapshot["simulation_s"], values={
                    "q": snapshot["position"], **{
                        key: snapshot[key] for key in (
                            "velocity", "acceleration", "step", "reference_progress_m",
                            "phase", "active_command_id",
                        )
                    },
                })
        assert reason == "late_command"
    manifest = config.manifest()
    manifest["execution_limits"] = {
        name: getattr(planner, name).tolist()
        for name in ("lower", "upper", "max_velocity", "max_acceleration", "max_jerk")
    }
    feedback = read_stream(tmp_path / "plant.sqlite3", "feedback")
    commands = read_stream(tmp_path / "controller.sqlite3", "command")
    report = evaluate_execution(tmp_path, manifest, feedback, commands)
    assert report["reference_settlement"]["status"] == "pass"
    assert report["joint_limits"]["status"] == "pass"
    assert report["qp"]["status"] == "pass"
    assert report["timing"]["admission_rejections"] == 1
    assert report["timing"]["samples"] == 2
    assert report["timing"]["boundary"] == LEGACY_CYCLE_TIMING_BOUNDARY
    assert report["timing"]["status"] == "insufficient_evidence"
    assert not report["recovery"]["pauses"][-1]["resumed"]
    json.dumps(report, allow_nan=False)
    incomplete_manifest = deepcopy(manifest)
    del incomplete_manifest["execution_limits"]
    incomplete = evaluate_execution(tmp_path, incomplete_manifest, feedback, commands)
    assert incomplete["joint_limits"]["status"] == "insufficient_evidence"
    assert incomplete["terminal_constraints"]["status"] == "insufficient_evidence"
    assert feedback[-1]["values"]["reference_progress_m"] == command.feed_m_s * .1
    changed = deepcopy(feedback)
    changed[-1]["values"]["reference_progress_m"] += .001
    assert evaluate_execution(tmp_path, manifest, changed, commands)["reference_settlement"]["status"] == "fail"
    assert evaluate_execution(tmp_path, manifest, feedback, [])["reference_settlement"]["status"] == "insufficient_evidence"
    np.savez_compressed(tmp_path / "path.npz", **{field.name: getattr(path, field.name) for field in fields(path)})
    manifest.update(source_sha256={}, path_sha256=content_hash(tmp_path / "path.npz"))
    write_json(tmp_path / "manifest.json", manifest)
    write_json(tmp_path / "result.json", {
        "termination": "stopped", "reason": "processed_motion_boundary", "worker_exit_codes": [],
    })
    evaluated = evaluate_run(tmp_path, resource_root=ROOT)
    assert not evaluated["completed"]
    assert evaluated["functional_verdict"] == "fail"
    assert evaluated["runtime_verdict"] == report["timing"]["status"]
    assert validate_run_report(tmp_path)[1] == evaluated
    assert evaluated["derived_data_policy"] == "on_demand"
    assert not (tmp_path / "feedback_metrics.npz").exists()
    report_hash = content_hash(tmp_path / "feedback_report.json")
    with evaluation_samples(tmp_path, ROOT, evaluated) as (samples, recomputed):
        hashes = recomputed["derived_data_sha256"]
        assert set(hashes) == {
            "feedback_metrics.npz", "clearance_metrics.npz", "feedback_arc_metrics.npz",
        }
        assert samples != tmp_path
        assert all(content_hash(samples / name) == expected for name, expected in hashes.items())
        assert recomputed["evaluator_sha256"] == evaluated["evaluator_sha256"]
        assert recomputed["evaluation_sources_sha256"] == evaluated["evaluation_sources_sha256"]
    assert not samples.exists()
    assert content_hash(tmp_path / "feedback_report.json") == report_hash
    historical = deepcopy(evaluated)
    historical_timing = historical["execution"]["timing"]
    for name in ("schema_version", "boundary", "eligible_for_period", "sidecar_sha256"):
        del historical_timing[name]
    historical_timing["status"] = (
        "pass" if historical_timing["p999_s"] <= .8 * historical_timing["period_s"] else "fail"
    )
    historical["runtime_verdict"] = historical_timing["status"]
    write_json(tmp_path / "historical_report.json", historical)
    historical_hash = content_hash(tmp_path / "historical_report.json")
    with evaluation_samples(tmp_path, ROOT, historical) as (_, recomputed):
        assert recomputed["execution"]["timing"]["boundary"] == LEGACY_CYCLE_TIMING_BOUNDARY
        assert recomputed["runtime_verdict"] == "insufficient_evidence"
        assert recomputed["task_verdict"] == historical["task_verdict"]
    assert content_hash(tmp_path / "historical_report.json") == historical_hash
    manifest["cycle_timing_boundary"] = CYCLE_TIMING_BOUNDARY
    write_json(tmp_path / "manifest.json", manifest)
    write_json(tmp_path / "controller.recording.json", {"cycle_timing": complete_timing.snapshot()})
    result = json.loads((tmp_path / "result.json").read_text())
    result["recording_statistics_sha256"] = {"controller": content_hash(tmp_path / "controller.recording.json")}
    write_json(tmp_path / "result.json", result)
    completed_report = evaluate_run(tmp_path, resource_root=ROOT)
    completed_timing = load_cycle_timing(tmp_path)
    assert completed_timing["eligible_for_period"]
    assert all(completed_report["execution"]["timing"][key] == value
               for key, value in cycle_timing_summary(completed_timing, .1).items())
    assert validate_run_report(tmp_path)[1] == completed_report
    changed_report = deepcopy(completed_report)
    changed_report["execution"]["timing"]["maximum_s"] += .01
    write_json(tmp_path / "feedback_report.json", changed_report)
    with pytest.raises(ValueError, match="完整周期"):
        validate_run_report(tmp_path)
    write_json(tmp_path / "feedback_report.json", completed_report)
    legacy = tmp_path / "legacy"
    legacy.mkdir()
    for name in ("manifest.json", "result.json", "path.npz", "controller.recording.json"):
        shutil.copy2(tmp_path / name, legacy / name)
    for role in ("plant", "controller"):
        with sqlite3.connect(legacy / f"{role}.sqlite3") as connection:
            connection.execute(
                "CREATE TABLE samples (sequence INTEGER PRIMARY KEY, stream TEXT, "
                "monotonic_s REAL, simulation_s REAL, ros_time_ns INTEGER, payload TEXT)"
            )
            for row in iter_records(tmp_path / f"{role}.sqlite3"):
                connection.execute("INSERT INTO samples VALUES (?, ?, ?, ?, ?, ?)", (
                    row["sequence"], row["stream"], row["monotonic_s"], row["simulation_s"],
                    row["ros_time_ns"], json.dumps(row["values"], allow_nan=False),
                ))
    evaluate_run(legacy, resource_root=ROOT)
    compact = tmp_path / "compact"
    compact_run(legacy, compact)
    converted = evaluate_run(compact, resource_root=ROOT)
    assert load_cycle_timing(compact) == completed_timing
    assert converted["execution"]["timing"] == completed_report["execution"]["timing"]
    assert validate_run_report(compact)[1] == converted
    converted_report_hash = content_hash(compact / "feedback_report.json")
    for filename in ("manifest.json", "result.json", "path.npz"):
        target = compact / filename
        original = target.read_bytes()
        target.write_bytes(original + b"\n")
        with pytest.raises(ValueError, match="转换保留文件身份"):
            evaluate_run(compact, resource_root=ROOT)
        with pytest.raises(ValueError, match="转换保留文件身份"):
            validate_run_report(compact)
        target.write_bytes(original)
    with sqlite3.connect(compact / "plant.sqlite3") as connection:
        deleted = connection.execute("DELETE FROM samples WHERE stream='admission'").rowcount
        assert deleted > 0
        connection.commit()
        connection.execute("PRAGMA wal_checkpoint(TRUNCATE)")
    with pytest.raises(ValueError, match="转换数据库身份"):
        evaluate_run(compact, resource_root=ROOT)
    with pytest.raises(ValueError, match="转换数据库身份"):
        validate_run_report(compact)
    assert content_hash(compact / "feedback_report.json") == converted_report_hash
    write_json(tmp_path / "result.json", {"termination": "changed"})
    with pytest.raises(ValueError, match="原始运行身份"):
        validate_run_report(tmp_path)


def test_arc_statistics_preserve_unobserved_path_and_ignore_pause_duration(tmp_path):
    first = arc_statistics(
        tmp_path, 1., np.array([0., .1, .2, .2, .2, .3]),
        {"error": np.array([0., .1, .2, .2, .2, .3])},
    )
    second = arc_statistics(
        tmp_path, 1., np.array([0., .1, .2, .3]),
        {"error": np.array([0., .1, .2, .3])},
    )
    assert first == second
    assert first["observed_points"] < first["grid_points"]
    assert first["metrics"]["error"]["maximum"] == .3


def test_coverage_requires_executed_reference_and_measured_endpoint_tolerance():
    total = 1.8167597440585048
    progress = np.linspace(0., 1.816759034636806, 1001)
    reference = np.linspace(0., total, 1001)
    report = ordered_coverage(progress, reference, total, 0., 3.2626763789698347e-6)
    assert report["ordered_full_path"]
    assert report["unreached_projected_length_m"] > 0.
    assert not ordered_coverage(progress, reference * .99, total, 0., 0.)["ordered_full_path"]
    assert not ordered_coverage(progress * .99, reference, total, 0., 0.)["ordered_full_path"]
    assert not ordered_coverage(progress, reference, total, 0., .00051)["ordered_full_path"]
    assert not ordered_coverage(progress[::20], reference[::20], total, 0., 0.)["ordered_full_path"]
