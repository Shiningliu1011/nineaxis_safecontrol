from __future__ import annotations

from pathlib import Path

import numpy as np

from .static_experiment_recording import iter_records
from .static_cycle_timing import cycle_timing_summary, load_cycle_timing


def ordered_coverage(progress, reference, total_length, initial_error, endpoint_error) -> dict:
    progress, reference = np.asarray(progress), np.asarray(reference)
    roundoff = 64 * np.spacing(max(1., total_length))
    tolerance = .0005
    reference_complete = bool(reference[0] <= roundoff and reference[-1] >= total_length - roundoff)
    ordered = bool(np.max(np.abs(np.diff(progress))) <= .01)
    measured_endpoints = bool(
        progress[0] <= tolerance and progress[-1] >= total_length - tolerance
        and initial_error <= tolerance and endpoint_error <= tolerance
    )
    return {
        "ordered_full_path": reference_complete and ordered and measured_endpoints,
        "reference_full_path": reference_complete,
        "endpoint_tolerance_m": tolerance,
        "initial_progress_m": float(progress[0]), "final_progress_m": float(progress[-1]),
        "total_length_m": total_length,
        "unreached_projected_length_m": float(max(0., total_length - progress[-1])),
        "maximum_forward_step_m": float(np.max(np.diff(progress))),
    }


def _records(path: Path, stream: str):
    for row in iter_records(path, stream):
        yield row["values"]


def evaluate_execution(directory: Path, manifest: dict, feedback: list, commands: list) -> dict:
    directory = Path(directory)
    controller = directory / "controller.sqlite3"
    plant = directory / "plant.sqlite3"
    trials, failed, violations = 0, 0, 0
    maximum_violation, maximum_residual = 0., 0.
    sns_trials, sns_violations, sns_scales, sns_residuals = 0, 0, [], []
    positive_times = []
    last_candidate = None
    for candidate in _records(controller, "candidate"):
        last_candidate = candidate
        if "start_step" in candidate and any(
            trial["feed_m_s"] > 1e-8 and trial["certificate"]["accepted"] for trial in candidate["trials"]
        ):
            positive_times.append(candidate["start_step"] * .002)
        for trial in candidate["trials"]:
            if trial.get("solver", "qp") != "qp":
                sns_trials += 1
                problem = trial["problem"]
                velocity = np.asarray(trial["solution"])
                value = np.asarray(problem["G"]) @ velocity
                sns_violations += bool(np.any(value > np.asarray(problem["h"]) + 1e-9)
                                       or np.any(value < np.asarray(problem["lower"]) - 1e-9))
                sns_scales.append(trial["task_scale"])
                sns_residuals.append(trial["task_residual"])
                continue
            trials += 1
            problem = trial["problem"]
            G, h, solution = (np.asarray(value) for value in (
                problem["G"], problem["h"], trial["solution"],
            ))
            violation = max(0., float(np.max(G @ solution - h)))
            maximum_violation = max(maximum_violation, violation)
            maximum_residual = max(maximum_residual, trial["kkt_residual"])
            if violation > 1e-9:
                violations += 1
    for _ in _records(controller, "qp_failure"):
        failed += 1
    attempts = trials + failed
    qp_success = trials / attempts if attempts else None
    qp = {
        "status": ("not_applicable" if manifest["config"]["control"]["solver_mode"] == "sns" else
                   "insufficient_evidence" if not attempts else
                   "pass" if qp_success >= .999 and not violations else "fail"),
        "attempts": attempts, "successes": trials, "failures": failed,
        "success_fraction": qp_success, "constraint_violations": violations,
        "maximum_constraint_residual": maximum_violation,
        "maximum_kkt_residual": maximum_residual,
    }
    sns_failures = sum(1 for _ in _records(controller, "sns_failure"))
    sns = {
        "status": ("not_applicable" if manifest["config"]["control"]["solver_mode"] != "sns" else
                   "fail" if sns_violations or sns_failures else
                   "pass" if sns_trials else "insufficient_evidence"),
        "attempts": sns_trials + sns_failures, "failures": sns_failures,
        "constraint_violations": sns_violations,
        "minimum_task_scale": min(sns_scales) if sns_scales else None,
        "maximum_task_residual": max(sns_residuals) if sns_residuals else None,
    }
    admissions = {row["command_id"]: row for row in _records(plant, "admission")}
    period = manifest["config"]["control"]["period_s"]
    timing = cycle_timing_summary(load_cycle_timing(directory, manifest), period)
    timing["admission_rejections"] = sum(not row["accepted"] for row in admissions.values())
    scheduled = {row["values"]["command_id"]: row["values"].get("scheduled") for row in commands}
    missing, unauthorized, inconsistent = 0, 0, 0
    maximum_progress_residual = 0.
    for row in feedback:
        value = row["values"]
        command_id = value.get("active_command_id")
        if command_id is None or "reference_progress_m" not in value:
            missing += 1
            continue
        if command_id == 0:
            expected = 0.
        elif command_id not in scheduled or scheduled[command_id] is None:
            missing += 1
            continue
        else:
            admission = admissions.get(command_id)
            if admission is None or not admission["accepted"]:
                unauthorized += 1
            command = scheduled[command_id]
            elapsed = (value["step"] - command["start_step"]) * .002
            if elapsed < 0:
                unauthorized += 1
            expected = command["reference_start_m"] + command["feed_m_s"] * min(
                max(elapsed, 0.), command["motion"]["period_s"],
            )
            if command["phase"] != "RUNNING" and command["feed_m_s"] != 0.:
                inconsistent += 1
        residual = abs(expected - value["reference_progress_m"])
        maximum_progress_residual = max(maximum_progress_residual, residual)
        inconsistent += residual > 1e-12
    settlement = {
        "status": ("insufficient_evidence" if missing else
                   "fail" if unauthorized or inconsistent else "pass"),
        "missing_records": missing, "unauthorized_samples": unauthorized,
        "inconsistent_samples": int(inconsistent),
        "maximum_reference_residual_m": maximum_progress_residual,
    }
    limits = manifest.get("execution_limits")
    joint = {"status": "insufficient_evidence", "reason": "缺少执行器限制快照"}
    if limits is not None:
        q, velocity, acceleration = [
            np.asarray([row["values"][name] for row in feedback])
            for name in ("q", "velocity", "acceleration")
        ]
        lower, upper = np.asarray(limits["lower"]), np.asarray(limits["upper"])
        vmax, amax = np.asarray(limits["max_velocity"]), np.asarray(limits["max_acceleration"])
        position_margin = np.minimum(q - lower, upper - q)
        velocity_margin, acceleration_margin = vmax - abs(velocity), amax - abs(acceleration)
        joint = {
            "status": ("pass" if np.all(position_margin >= 0)
                       and np.all(velocity_margin >= -1e-12)
                       and np.all(acceleration_margin >= -1e-12) else "fail"),
            "minimum_position_margin": position_margin.min(axis=0).tolist(),
            "minimum_velocity_margin": velocity_margin.min(axis=0).tolist(),
            "minimum_acceleration_margin": acceleration_margin.min(axis=0).tolist(),
        }
    transitions = list(_records(controller, "transition"))
    planned_recoveries = [
        row for row in transitions if row["to"] == "RUNNING" and "recovery_delay_s" in row
    ]
    phases = np.array([row["values"].get("phase", "") for row in feedback])
    reference = np.array([row["values"].get("reference_progress_m", 0.) for row in feedback])
    q = np.array([row["values"]["q"] for row in feedback])
    pause_intervals = []
    recoveries = []
    active = phases == "RUNNING"
    boundaries = np.r_[0, np.flatnonzero(active[1:] != active[:-1]) + 1, len(active)]
    first_running = int(np.flatnonzero(active)[0]) if np.any(active) else len(active)
    for interval_index, (start, end) in enumerate(zip(boundaries[:-1], boundaries[1:])):
        if active[start] or phases[start] == "ENDPOINT_HOLD":
            continue
        pause_intervals.append({
            "start_simulation_s": feedback[start]["simulation_s"],
            "duration_s": (end - start) * .002,
            "reference_progress_m": float(reference[start]),
            "reference_range_m": float(np.ptp(reference[start:end])),
            "maximum_joint_change": np.max(np.abs(q[start:end] - q[start]), axis=0).tolist(),
            "resumed": bool(end < len(active) and active[end]),
        })
        if end < len(active) and active[end] and start > first_running:
            resumed_s = feedback[end]["simulation_s"]
            candidates = [value for value in positive_times
                          if feedback[start]["simulation_s"] <= value <= resumed_s]
            next_end = boundaries[interval_index + 2]
            recoveries.append({
                "time_s": resumed_s,
                "following_interval_end_s": (feedback[next_end]["simulation_s"]
                                             if next_end < len(feedback) else feedback[-1]["simulation_s"] + .002),
                "first_positive_candidate_s": min(candidates) if candidates else None,
                "recovery_delay_s": resumed_s - min(candidates) if candidates else None,
                "delay_status": "measured" if candidates else "insufficient_evidence",
            })
    limiting = {
        "status": "insufficient_evidence",
        "reason": "缺少执行器限制快照" if limits is None else "缺少末次候选几何记录",
    }
    if (limits is not None and last_candidate is not None
            and "initial" in last_candidate and "geometry" in last_candidate):
        position = np.asarray(last_candidate["initial"]["position"])
        limiting = {
            "status": "measured",
            "reference_progress_m": last_candidate["reference_progress_m"],
            "joint_lower_margin": (position - limits["lower"]).tolist(),
            "joint_upper_margin": (np.asarray(limits["upper"]) - position).tolist(),
            "geometry": [
                {"pair": row["pair"], "kind": row["kind"], "distance_m": row["distance_m"],
                 "required_m": row["required_m"], "margin_m": row["distance_m"] - row["required_m"]}
                for row in last_candidate["geometry"]
            ],
            "candidates": [
                {"feed_m_s": trial["feed_m_s"], "certificate": trial["certificate"]}
                for trial in last_candidate["trials"]
            ],
        }
    return {
        "qp": qp, "sns": sns, "timing": timing, "reference_settlement": settlement,
        "terminal_constraints": limiting,
        "joint_limits": joint, "recovery": {
            "events": recoveries, "planned_events": planned_recoveries, "pauses": pause_intervals,
            "restart_count": len(recoveries),
            "faults": [row for row in transitions if row["to"] == "FAULT"],
        },
    }


def arc_statistics(directory: Path, total_length: float, progress, series: dict,
                   *, save_samples: bool = True) -> dict:
    progress = np.asarray(progress)
    forward, indices = np.unique(np.maximum.accumulate(progress), return_index=True)
    grid = np.linspace(0., total_length, 2001)
    observed = grid[(grid >= forward[0]) & (grid <= forward[-1])]
    values = {name: np.interp(observed, forward, data[indices]) for name, data in series.items()}
    if save_samples:
        np.savez_compressed(directory / "feedback_arc_metrics.npz", progress_m=observed, **values)
    return {
        "grid_points": len(grid), "observed_points": len(observed),
        "spacing_m": total_length / (len(grid) - 1),
        "metrics": {
            name: {
                "rms": float(np.sqrt(np.mean(data ** 2))) if len(data) else None,
                "p95": float(np.quantile(data, .95)) if len(data) else None,
                "maximum": float(np.max(data)) if len(data) else None,
            }
            for name, data in values.items()
        },
    }
