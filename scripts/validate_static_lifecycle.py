from __future__ import annotations

import argparse
from contextlib import closing
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys
import time

import msgpack
import psutil

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from robot_safecontrol_moveit.static_experiment_config import content_hash
from robot_safecontrol_moveit.static_experiment_capacity import lifecycle_outcome_valid
from robot_safecontrol_moveit.static_experiment_provenance import validate_run_report
from robot_safecontrol_moveit.static_experiment_recording import iter_records, write_json


def inspect_run(directory: Path, *, expect_timeout: bool = False) -> dict:
    manifest, report = validate_run_report(directory)
    streams, starts, faults = {}, [], []
    identities = {name: content_hash(directory / name) for name in (
        "manifest.json", "result.json", "plant.sqlite3", "controller.sqlite3", "feedback_report.json",
    )}
    for role in ("plant", "controller"):
        database = directory / f"{role}.sqlite3"
        with closing(sqlite3.connect(database.resolve().as_uri() + "?mode=ro", uri=True)) as connection:
            encoded = {name: {"total": total, "maximum": maximum} for name, total, maximum in
                       connection.execute("SELECT stream,sum(length(payload)),max(length(payload)) "
                                          "FROM samples GROUP BY stream")}
        for row in iter_records(database):
            key = f"{role}/{row['stream']}"
            values = row["values"]
            item = streams.setdefault(key, {
                "count": 0, "first_monotonic_s": row["monotonic_s"],
                "last_monotonic_s": row["monotonic_s"], "maximum_raw_bytes": 0,
                "encoded": encoded[row["stream"]], "maximum_trials": 0, "maximum_pairs": 0,
                "trial_fields": [], "problem_shapes": {},
            })
            item["count"] += 1
            item["last_monotonic_s"] = row["monotonic_s"]
            item["maximum_raw_bytes"] = max(item["maximum_raw_bytes"], len(msgpack.packb(
                values, use_bin_type=True, use_single_float=False,
            )))
            item["maximum_trials"] = max(item["maximum_trials"], len(values.get("trials", [])))
            item["maximum_pairs"] = max(item["maximum_pairs"], len(values.get("geometry", [])))
            for trial in values.get("trials", []):
                item["trial_fields"] = sorted(set(item["trial_fields"]) | trial.keys())
                for name, value in trial["problem"].items():
                    shape = [len(value)]
                    if value and isinstance(value[0], list):
                        shape.append(len(value[0]))
                    if name not in item["problem_shapes"] or shape > item["problem_shapes"][name]:
                        item["problem_shapes"][name] = shape
            if role == "controller" and row["stream"] == "candidate":
                starts.append(values["start_step"])
            if row["stream"] in ("qp_failure", "sns_failure") or (
                row["stream"] == "event" and values["reason"] in (
                    "fault", "interrupted", "budget_exhausted",
                )
            ):
                faults.append({"role": role, **row})
    ready = {
        role: json.loads((directory / f"{role}.ready").read_text())
        for role in ("plant", "controller")
    }
    recording = {
        role: json.loads((directory / f"{role}.recording.json").read_text())
        for role in ("plant", "controller") if (directory / f"{role}.recording.json").exists()
    }
    result = json.loads((directory / "result.json").read_text())
    for role, expected in result.get("recording_statistics_sha256", {}).items():
        if content_hash(directory / f"{role}.recording.json") != expected:
            raise ValueError(f"记录容量统计身份变化：{role}")
    for role, expected in result.get("ready_sha256", {}).items():
        if content_hash(directory / f"{role}.ready") != expected:
            raise ValueError(f"worker ready 身份变化：{role}")
    bound_identities = all(
        set(result.get(name, {})) == {"plant", "controller"}
        for name in ("ready_sha256", "recording_statistics_sha256")
    ) and all(
        row.get("source_sha256") == manifest["source_sha256"]
        and row.get("manifest_sha256") == identities["manifest.json"]
        for row in ready.values()
    )
    ticks = round(manifest["config"]["control"]["period_s"] / .002)
    unique_steps = len(starts) == len(set(starts))
    increasing = all(second - first >= ticks for first, second in zip(starts, starts[1:]))
    startup_order = ready["controller"]["monotonic_s"] <= ready["plant"]["monotonic_s"]
    feedback = streams["plant/feedback"]
    cycle_bound = (feedback["count"] + ticks - 1) // ticks + 2
    limits = manifest.get("recording_limits")
    within_limits = limits is not None and "stream_record_limits" in limits and (
        set(recording) == {"plant", "controller"}
    ) and (
        feedback["count"] <= limits["maximum_feedback_records"]
        and len(starts) <= limits["maximum_candidate_records"]
        and all(
            name in limits["stream_record_limits"][role]
            and item["count"] <= limits["stream_record_limits"][role][name]
            for key, item in streams.items() for role, name in [key.split("/")]
        )
        and all(
            item["budget"] is not None
            and all(
                started < item["budget"]["deadline_monotonic_s"]
                for started in item["budget"]["last_started_monotonic_s"].values()
            )
            and all(
                count == streams[f"{role}/{name}"]["count"]
                for name, count in item["stream_counts"].items()
            )
            for role, item in recording.items()
        )
    )
    expected = "timeout" if expect_timeout else (
        "pause_timeout" if manifest["config"]["scenario"] == "S5" else "completed"
    )
    outcome_valid = lifecycle_outcome_valid(result, expected_termination=expected) and not faults
    full_timeout = not expect_timeout or (
        feedback["last_monotonic_s"] - feedback["first_monotonic_s"] >= limits["timeout_s"]
    )
    storage = manifest.get("storage_budget")
    within_storage = None if storage is None else all(
        item.get("storage_budget") is not None
        and item["storage_budget"]["limits"] == storage["writers"][role]
        and item["storage_budget"]["reason"] is None
        and item["storage_budget"]["completion_within_reserve"]
        for role, item in recording.items()
    )
    return {
        "status": "pass" if unique_steps and increasing and startup_order and within_limits
        and len(starts) <= cycle_bound and outcome_valid and bound_identities and full_timeout
        and within_storage is not False else "fail",
        "directory": str(directory), "identities_sha256": identities,
        "streams": streams, "recording": recording,
        "ready": ready, "controller_ready_before_plant": startup_order,
        "bound_source_ready_statistics": bound_identities,
        "expected_termination": expected, "outcome_valid": outcome_valid,
        "faults": faults, "full_timeout_recording_verified": full_timeout if expect_timeout else None,
        "candidate_start_steps": {
            "count": len(starts), "unique": len(set(starts)), "strict_period_increase": increasing,
            "maximum_count_from_feedback": cycle_bound, "period_ticks": ticks,
        },
        "recording_limits": limits, "within_recording_limits": within_limits,
        "within_storage_limits": within_storage,
        "result": result,
        "timing": report["execution"]["timing"], "verdict": report["verdict"],
        "directory_bytes": sum(path.stat().st_size for path in directory.rglob("*") if path.is_file()),
        "allocated_bytes": sum(path.stat().st_blocks * 512 for path in directory.rglob("*") if path.is_file()),
    }


def measure(command: list[str], directory: Path) -> dict:
    peaks = {"rss_bytes": 0}
    interval_s = .1
    with (directory / "supervisor.log").open("x") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        started = time.monotonic()
        while process.poll() is None:
            rows = list(psutil.process_iter(attrs=["pid", "ppid", "memory_info"], ad_value=None))
            descendants = {process.pid}
            for _ in range(4):
                descendants.update(row.info["pid"] for row in rows if row.info["ppid"] in descendants)
            memory = sum(row.info["memory_info"].rss for row in rows
                         if row.info["pid"] in descendants and row.info["memory_info"] is not None)
            peaks["rss_bytes"] = max(peaks["rss_bytes"], memory)
            time.sleep(interval_s)
        return {"returncode": process.returncode, "elapsed_s": time.monotonic() - started,
                "peaks": peaks, "command": command, "monitor_interval_s": interval_s}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--resource-root", type=Path)
    parser.add_argument("--scene-evidence", type=Path)
    parser.add_argument("--storage-budget", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--method", default="D")
    parser.add_argument("--feed-factor", type=float, default=1.)
    parser.add_argument("--domain", type=int, default=200)
    parser.add_argument("--cpus", default="0-15")
    parser.add_argument("--expect-timeout", action="store_true")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    report = {"script_sha256": content_hash(Path(__file__)), "command": sys.argv,
              "monitor_affinity": sorted(os.sched_getaffinity(0))}
    if args.source is None:
        run = output / "run"
        command = [
            "taskset", "-c", args.cpus, sys.executable, "-m", "robot_safecontrol_moveit.static_experiment",
            "run", "--config", str(args.config.resolve(strict=True)), "--method", args.method,
            "--feed-factor", str(args.feed_factor), "--resource-root",
            str(args.resource_root.resolve(strict=True)), "--output", str(run),
            "--domain", str(args.domain),
        ]
        if args.scene_evidence is not None:
            command.extend(["--scene-evidence", str(args.scene_evidence.resolve(strict=True))])
        if args.storage_budget is not None:
            command.extend(["--storage-budget", str(args.storage_budget.resolve(strict=True))])
        report["process"] = measure(command, output)
    else:
        run = args.source.resolve(strict=True)
    report["inspection"] = inspect_run(run, expect_timeout=args.expect_timeout)
    if "process" in report:
        report["process"]["peaks"]["wal_individual_maxima_sum_bytes"] = sum(
            row["wal_bytes_before_checkpoint"] for row in report["inspection"]["recording"].values()
        )
    report["status"] = report["inspection"]["status"]
    if "process" in report:
        expected_code = 0 if report["inspection"]["verdict"] == "pass" else 1
        if report["process"]["returncode"] != expected_code:
            report["status"] = "fail"
    write_json(output / "lifecycle-validation.json", report)
    print(json.dumps({
        "status": report["status"], "inspection_status": report["inspection"]["status"],
        **{key: report["inspection"][key] for key in (
            "controller_ready_before_plant", "candidate_start_steps", "verdict",
        )},
    }, ensure_ascii=False))
    return 0 if report["status"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
