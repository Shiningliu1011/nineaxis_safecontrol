import argparse
import csv
import gzip
import json
from pathlib import Path
import shutil

import numpy as np

from robot_safecontrol_moveit.static_experiment_config import content_hash
from robot_safecontrol_moveit.static_experiment_evaluation import evaluation_samples
from robot_safecontrol_moveit.static_experiment_provenance import validate_run_report
from robot_safecontrol_moveit.static_experiment_reporting import _plot_run


ROOT = Path(__file__).resolve().parents[1]
PACK = ROOT / "docs/proposal-static-obstacles"
NTFS = Path("/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020")
PREFLIGHT = NTFS / "complete-cycle-preflight-01"
RUNS = {
    "s0": PREFLIGHT / "d-s0/run",
    "s2": NTFS / "guarded-preflight-01/d-s2/run",
    "s4": PREFLIGHT / "d-s4-initial3-feed0.5/run",
    "s5-ext4": ROOT / "output/static-capacity-20261020/complete-cycle-preflight-01/d-s5/run",
    "s5-ntfs": PREFLIGHT / "d-s5/run",
    "s5-windows": Path("/media/lsn/Windows/robot_safecontrol_static_experiments_20261021/complete-cycle-preflight-01/d-s5/run"),
    "s4-A": ROOT / "output/static-scene-validation-20261020/s4-ac-pair-01/A/run",
    "s4-C": ROOT / "output/static-scene-validation-20261020/s4-ac-pair-01/C/run",
    "period-20-A": NTFS / "guarded-period-study-02/period-20-A",
    "period-20-B": NTFS / "guarded-period-study-02/period-20-B",
}


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def export(label, include_series):
    source = RUNS[label]
    manifest, report = validate_run_report(source)
    target = PACK / "data" / label
    target.mkdir(parents=True, exist_ok=False)
    originals = {}
    for filename in ("manifest.json", "feedback_report.json", "clearance_report.json", "result.json", "source_configuration.yaml", "path.npz"):
        path = source / filename
        shutil.copy2(path, target / filename)
        originals[filename] = content_hash(path)
    identity = {
        "original_directory": str(source),
        "files_sha256": originals,
        "database_sha256": report["records_sha256"],
        "runtime_sources_sha256": manifest["source_sha256"],
        "raw_databases_in_git": False,
        "series_exported": include_series,
    }
    if include_series:
        with evaluation_samples(source, ROOT, report) as (derived, recomputed):
            with np.load(derived / "feedback_metrics.npz", allow_pickle=False) as data:
                columns = ["simulation_s", "path_distance_m", "reference_position_error_m", "tool_axis_angle_rad", "tcp_workpiece_distance_m", "reference_progress_m", "projected_progress_m", "phase"]
                arrays = [data[key] for key in columns]
                with np.load(derived / "clearance_metrics.npz", allow_pickle=False) as clearance:
                    identity["clearance_array_keys"] = list(clearance.files)
                rows = zip(*arrays)
                with gzip.open(target / "feedback-full.csv.gz", "wt", encoding="utf-8", newline="") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(columns)
                    writer.writerows(rows)
                indices = np.unique(np.r_[np.arange(0, len(arrays[0]), 20), len(arrays[0]) - 1])
                with (target / "feedback-25hz.csv").open("w", encoding="utf-8", newline="") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(columns)
                    writer.writerows(zip(*(array[indices] for array in arrays)))
                identity["feedback_samples"] = len(arrays[0])
                identity["plot_samples"] = len(indices)
                identity["series_policy"] = "full: all evaluated feedback; 25hz: every twentieth plus final sample; all summary statistics use full report"
            _plot_run(source, ROOT, ROOT / "picture/proposal-static-obstacles" / label, manifest, report, derived, recomputed)
    for filename, expected in originals.items():
        if content_hash(source / filename) != expected:
            raise ValueError(f"原始文件身份发生变化：{source / filename}")
    write_json(target / "source.json", identity)
    print(label, "validated and exported", flush=True)


def summarize():
    rows = []
    for label in RUNS:
        directory = PACK / "data" / label
        report = json.loads((directory / "feedback_report.json").read_text())
        manifest = json.loads((directory / "manifest.json").read_text())
        timing = report["execution"]["timing"]
        coverage = report["coverage"]
        row = {
            "run": label, "scenario": manifest["config"]["scenario"],
            "solver_mode": manifest["config"]["control"]["solver_mode"],
            "recovery_mode": manifest["config"]["control"]["recovery"]["mode"],
            "nominal_feed_multiplier": manifest["config"]["control"]["nominal_feed_multiplier"],
            "task_verdict": report["task_verdict"], "runtime_verdict": report.get("runtime_verdict", "legacy"),
            "termination": report["termination"]["termination"], "completed": report["completed"],
            "feedback_samples": report["feedback"]["samples"],
            "completion_time_s": report["completion_time_s"],
            "coverage_fraction": coverage["final_progress_m"] / coverage["total_length_m"],
            "unreached_length_m": coverage["unreached_projected_length_m"],
            "path_rms_mm": report["metrics"]["path_distance_m"]["rms"] * 1000,
            "path_p95_mm": report["metrics"]["path_distance_m"]["p95"] * 1000,
            "path_max_mm": report["metrics"]["path_distance_m"]["maximum"] * 1000,
            "tool_axis_rms_deg": np.rad2deg(report["metrics"]["tool_axis_angle_rad"]["rms"]),
            "tool_axis_max_deg": np.rad2deg(report["metrics"]["tool_axis_angle_rad"]["maximum"]),
            "external_min_mm": report["external_clearance"]["minimum_m"] * 1000,
            "self_min_mm": report["self_clearance"]["minimum_m"] * 1000,
            "restarts": report["execution"]["recovery"]["restart_count"],
            "period_ms": timing["period_s"] * 1000,
            "cycle_p999_ms": timing["p999_s"] * 1000,
            "cycle_max_ms": timing["maximum_s"] * 1000,
            "deadline_misses": timing["deadline_misses"],
            "timing_boundary": timing.get("boundary", "legacy_incomplete_cycle"),
        }
        rows.append(row)
    with (PACK / "data/metrics.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    write_json(PACK / "data/metrics.json", rows)
    inventory = {}
    for base in (PACK, ROOT / "picture/proposal-static-obstacles"):
        for path in sorted(base.rglob("*")):
            if path.is_file() and path != PACK / "checksums.json":
                inventory[str(path.relative_to(ROOT))] = {"sha256": content_hash(path), "bytes": path.stat().st_size}
    for name in ("export_proposal_evidence.py", "plot_proposal_summary.py", "verify_proposal_evidence.py"):
        path = ROOT / "scripts" / name
        inventory[str(path.relative_to(ROOT))] = {"sha256": content_hash(path), "bytes": path.stat().st_size}
    write_json(PACK / "checksums.json", inventory)
    print("inventory", len(inventory), "files", sum(item["bytes"] for item in inventory.values()), "bytes", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("label", choices=[*RUNS, "summary"])
    parser.add_argument("--series", action="store_true")
    args = parser.parse_args()
    if args.label == "summary":
        summarize()
    else:
        export(args.label, args.series)
