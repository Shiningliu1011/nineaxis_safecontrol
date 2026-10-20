from copy import deepcopy
from dataclasses import asdict, fields
import json
from pathlib import Path
import time

import numpy as np
import pytest

from robot_safecontrol_moveit.oscbf_trajectory import bootstrap_portable
from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig, content_hash
from robot_safecontrol_moveit.static_experiment_evaluation import evaluate_run
from robot_safecontrol_moveit.static_experiment_nodes import _resources
from robot_safecontrol_moveit.static_experiment_recording import RunRecorder, read_stream, write_json
from robot_safecontrol_moveit.static_experiment_provenance import validate_run_report
from robot_safecontrol_moveit.static_execution_evaluation import evaluate_execution, arc_statistics, ordered_coverage

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
    candidate = control.propose(state, 0., time_s=.1, executed_feed=0.)
    assert candidate is not None and candidate.certificate.accepted
    command = ScheduledMotion(1, 50, 0., candidate.solution.feed_m_s, candidate.phase, candidate.motion)
    with RunRecorder(tmp_path / "controller.sqlite3") as controller, RunRecorder(tmp_path / "plant.sqlite3") as plant:
        controller.append("candidate", monotonic_s=time.monotonic(), values={"trials": control.last_trials})
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
            controller.append("cycle", monotonic_s=time.monotonic(), values={
                "command_id": command_id, "accepted": reason == "accepted", "reason": reason,
                "duration_s": time.monotonic() - started, "period_s": .1,
            })
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
    assert not report["recovery"]["pauses"][-1]["resumed"]
    json.dumps(report, allow_nan=False)
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
