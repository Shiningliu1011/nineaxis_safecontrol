from __future__ import annotations

from collections import Counter
from copy import deepcopy
import itertools
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import yaml

from .static_experiment_config import ExperimentConfig, METHODS, content_hash
from .static_experiment_capacity import require_formal_capacity
from .static_experiment_volumes import capacity_run_directory
from .static_experiment_provenance import (
    environment_identity, relocate_manifest, snapshot_sources, validate_run_report, verify_snapshot,
)
from .static_experiment_recording import write_json


SCENARIOS = ("s0", "s1", "s2", "s3-travel-100", "s3-travel-70", "s3-travel-40", "s4", "s5")


def experiment_rows() -> list[dict]:
    return [
        {
            "id": f"{scene}-initial-{initial}-feed-{feed:g}-{method}-repeat-{repeat}",
            "scene": scene, "initial": initial, "feed_factor": feed,
            "method": method, "repeat": repeat,
            "configuration": f"configs/{scene}-initial-{initial}-feed-{feed:g}.yaml",
        }
        for scene, initial, feed, method, repeat in itertools.product(
            SCENARIOS, (1, 2, 3), (.5, 1., 1.5), METHODS, (1, 2, 3),
        )
    ]


def _physical_identity(manifest, resource_root: Path):
    root = Path(resource_root).resolve(strict=True)
    manifest = relocate_manifest(manifest, root)
    scene = deepcopy(manifest["scene"])
    del scene["config_sha256"]
    collision = scene["self_collision"]
    for record in [collision["manifest"], *collision["auxiliary"], *collision["meshes"]]:
        if content_hash(Path(record["path"])) != record["sha256"]:
            raise ValueError(f"可行性碰撞资源身份变化：{record['path']}")
    for name in ("trajectory_mat", "kinematics_config"):
        path = root / manifest["config"][name]
        if content_hash(path) != manifest["resources"][name]["sha256"]:
            raise ValueError(f"可行性资源身份变化：{path}")
    return {
        "scene": scene, "initial_position": manifest["config"]["initial_position"],
        "execution_limits": manifest["execution_limits"],
        "trajectory_mat": manifest["resources"]["trajectory_mat"]["sha256"],
        "kinematics_config": manifest["resources"]["kinematics_config"]["sha256"],
    }


def freeze_formal(catalog_path: Path, period_directory: Path, evidence_index: Path,
                  directory: Path, resource_root: Path) -> dict:
    from .static_experiment import _sources
    from .static_scenario_evidence import verify_blockage, verify_reconfiguration
    from .static_period_measurement import validate_period_storage

    catalog = json.loads(catalog_path.read_text())
    period = json.loads((period_directory / "period_report.json").read_text())
    study = json.loads((period_directory / "study.json").read_text())
    evidence = json.loads(evidence_index.read_text())
    root = resource_root.resolve(strict=True)
    if (catalog["kind"] != "development" or len(catalog["initial_positions"]) != 3
            or catalog["feed_factors"] != [.5, 1., 1.5]
            or {row["id"] for row in catalog["scenarios"]} != set(SCENARIOS)):
        raise ValueError("开发清单必须包含完整场景、三个初态和三档进给")
    if (period["status"] != "pass" or period["selected_period_s"] is None
            or period["study_sha256"] != content_hash(period_directory / "study.json")):
        raise ValueError("共同周期尚未通过完整测量")
    selected_period = period["selected_period_s"]
    for group in period["groups"]:
        for row in group["methods"]:
            run = period_directory / row["run"]
            validate_run_report(run)
            if validate_period_storage(run) != row.get("storage_statistics_sha256"):
                raise ValueError("共同周期缺少匹配的容量守卫统计身份")
            if content_hash(run / "feedback_report.json") != row["report_sha256"]:
                raise ValueError("周期测量报告身份不一致")
            for role, expected in row["records_sha256"].items():
                if content_hash(run / f"{role}.sqlite3") != expected:
                    raise ValueError("周期测量原始记录身份不一致")
    rationale = Path(evidence["parameter_rationale"]).resolve(strict=True)
    if not rationale.read_text().strip():
        raise ValueError("缺少开发参数选择依据")
    feasible = []
    for filename in evidence["feasibility_runs"]:
        run = Path(filename).resolve(strict=True)
        manifest, report = validate_run_report(run)
        if not report["completed"] or report["task_verdict"] != "pass":
            raise ValueError(f"可行性来源尚未完成完整跟随：{run}")
        if any(report[name]["status"] != "pass" for name in ("external_clearance", "self_clearance")):
            raise ValueError(f"可行性来源的几何检查未通过：{run}")
        for role, expected in report["records_sha256"].items():
            if content_hash(run / f"{role}.sqlite3") != expected:
                raise ValueError(f"可行性来源原始记录变化：{run}")
        feasible.append({
            "physical_identity": _physical_identity(manifest, root), "run": str(run),
            "report_sha256": content_hash(run / "feedback_report.json"),
            "records_sha256": report["records_sha256"],
        })
    environment = environment_identity()
    first_config = ExperimentConfig.load(
        catalog_path.parent / catalog["scenarios"][0]["config"], resource_root=root,
    )
    if _sources(first_config) != study["source_sha256"]:
        raise ValueError("正式代码与周期测量代码不同")
    if environment["sha256"] != study["environment"]["sha256"]:
        raise ValueError("正式设备环境与周期测量不同")
    directory = directory.resolve()
    directory.mkdir(parents=True, exist_ok=False)
    (directory / "configs").mkdir()
    configurations, manifests, scene_evidence = {}, [], {}
    for row in catalog["scenarios"]:
        source = catalog_path.parent / row["config"]
        if content_hash(source) != row["config_sha256"]:
            raise ValueError("开发场景配置身份变化")
        base = yaml.safe_load(source.read_text())
        for initial, feed in itertools.product((1, 2, 3), (.5, 1., 1.5)):
            values = deepcopy(base)
            values["initial_position"] = catalog["initial_positions"][initial - 1]
            values["control"] = deepcopy(study["common_configuration"]["control"])
            values["control"]["solver_mode"] = "separated"
            values["control"]["recovery"]["mode"] = "basic"
            values["control"]["period_s"] = selected_period
            values["control"]["nominal_feed_multiplier"] *= feed
            filename = f"configs/{row['id']}-initial-{initial}-feed-{feed:g}.yaml"
            target = directory / filename
            target.write_text(yaml.safe_dump(values, sort_keys=False, allow_unicode=True), encoding="utf-8")
            config = ExperimentConfig.load(target, resource_root=root)
            manifest = {**config.manifest(), "source_sha256": _sources(config)}
            if manifest["source_sha256"] != study["source_sha256"]:
                raise ValueError("正式代码与周期测量代码不同")
            from work.static_collision import StaticGeometry
            measured = StaticGeometry(manifest["scene"]).query(config.initial_position)
            if not all(item.distance.valid and item.distance.distance_m >= item.required_m for item in measured):
                raise ValueError(f"正式初态未满足全部几何要求：{filename}")
            physical = _physical_identity(manifest, root)
            matches = [item for item in feasible if item["physical_identity"] == physical]
            configurations[filename] = {
                "sha256": content_hash(target), "physical_identity": physical,
                "feasibility": "proven" if matches else "insufficient_evidence",
                "feasibility_runs": matches,
            }
            manifests.append(manifest)
            if row["scenario"] == "S4" and row["id"] not in scene_evidence:
                scene_evidence[row["id"]] = verify_reconfiguration(config, Path(evidence["s4_witness"]))
            if row["scenario"] == "S5" and row["id"] not in scene_evidence:
                scene_evidence[row["id"]] = verify_blockage(config, Path(evidence["s5_blockage"]))
    rows = experiment_rows()
    for row in rows:
        config = ExperimentConfig.load(directory / row["configuration"], resource_root=root, method=row["method"])
        row["config_sha256"] = config.identity
    snapshot = snapshot_sources(root, manifests, directory / "provenance")
    manifest = {
        "schema_version": 1, "kind": "formal", "resource_root": str(root),
        "period_s": selected_period, "environment_identity": environment["sha256"],
        "source_sha256": study["source_sha256"],
        "development_catalog": {"path": str(catalog_path.resolve()), "sha256": content_hash(catalog_path)},
        "period_study": {"path": str(period_directory.resolve()),
                         "report_sha256": content_hash(period_directory / "period_report.json")},
        "parameter_rationale": {"path": str(rationale), "sha256": content_hash(rationale),
                                "text": rationale.read_text()},
        "evidence_index_sha256": content_hash(evidence_index),
        "configurations": configurations, "scene_evidence": scene_evidence,
        "provenance_sha256": content_hash(directory / "provenance/snapshot.json"),
        "source_archive_sha256": snapshot["archive_sha256"], "runs": rows,
    }
    write_json(directory / "formal_manifest.json", manifest)
    write_json(directory / "formal_identity.json", {"sha256": content_hash(directory / "formal_manifest.json")})
    return manifest


def load_formal(directory: Path) -> dict:
    manifest_path = directory / "formal_manifest.json"
    identity = json.loads((directory / "formal_identity.json").read_text())
    if content_hash(manifest_path) != identity["sha256"]:
        raise ValueError("正式清单身份不一致")
    manifest = json.loads(manifest_path.read_text())
    if manifest["kind"] != "formal" or len(manifest["runs"]) != 1080:
        raise ValueError("正式清单缺少规定组合")
    expected = experiment_rows()
    if any(any(actual[key] != value for key, value in row.items())
           for actual, row in zip(manifest["runs"], expected)):
        raise ValueError("正式运行编号或配对关系不一致")
    for filename, record in manifest["configurations"].items():
        if content_hash(directory / filename) != record["sha256"]:
            raise ValueError(f"正式配置身份变化：{filename}")
    if content_hash(directory / "provenance/snapshot.json") != manifest["provenance_sha256"]:
        raise ValueError("正式来源身份变化")
    verify_snapshot(directory / "provenance")
    return manifest


def run_formal(directory: Path, *, resource_root: Path, domain: int, limit: int | None = None) -> dict:
    from .static_experiment import _sources

    directory = directory.resolve(strict=True)
    manifest = load_formal(directory)
    if environment_identity()["sha256"] != manifest["environment_identity"]:
        raise ValueError("运行设备或软件环境与正式清单不一致")
    if limit is not None and limit <= 0:
        raise ValueError("运行数量必须为正整数")
    (directory / "runs").mkdir(exist_ok=True)
    (directory / "logs").mkdir(exist_ok=True)
    capacity = require_formal_capacity(directory, manifest)
    executed = 0
    recorded_count = sum(
        capacity_run_directory(directory, capacity, row["id"]).exists() for row in manifest["runs"]
    )
    for row in manifest["runs"]:
        config = ExperimentConfig.load(
            directory / row["configuration"], resource_root=resource_root, method=row["method"],
        )
        if config.identity != row["config_sha256"] or _sources(config) != manifest["source_sha256"]:
            raise ValueError("正式运行的代码或配置身份变化")
        run = capacity_run_directory(directory, capacity, row["id"])
        if run.exists():
            continue
        capacity = require_formal_capacity(directory, manifest)
        anchor = directory / "runs" / row["id"]
        if anchor != run and not anchor.is_symlink():
            anchor.symlink_to(run, target_is_directory=True)
        command = [
            sys.executable, "-m", "robot_safecontrol_moveit.static_experiment", "run",
            "--config", str(config.source), "--method", row["method"], "--output", str(run),
            "--resource-root", str(resource_root), "--domain", str(domain),
        ]
        from .static_experiment_engineering_capacity import run_capacity_group
        from .static_recording_capacity import runtime_storage_configuration

        if capacity.get("is_validated_engineering_budget") is not True:
            raise ValueError("正式运行需要逐 writer 空间预算证据")
        engineering = json.loads(Path(capacity["engineering_evidence"]).read_text())
        volume = next(item for item in capacity["volumes"] if row["id"] in item["run_ids"])
        storage = runtime_storage_configuration(
            engineering["models"][run_capacity_group(row, engineering)],
            engineering["wal_bytes_per_record"], volume["budget"],
        )
        storage.update(
            config_sha256=config.identity, source_sha256=manifest["source_sha256"],
            filesystem_uuid=volume["filesystem_uuid"], run_id=row["id"],
        )
        storage_path = directory / "logs" / f"{row['id']}.storage.json"
        write_json(storage_path, storage)
        command.extend(["--storage-budget", str(storage_path)])
        if row["scene"] in manifest["scene_evidence"]:
            command.extend(["--scene-evidence", manifest["scene_evidence"][row["scene"]]["source"]])
        with (directory / "logs" / f"{row['id']}.log").open("x", encoding="utf-8") as log:
            process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=False)
        write_json(directory / "logs" / f"{row['id']}.process.json", {
            "returncode": process.returncode, "command": command,
        })
        executed += 1
        recorded_count += int(run.exists())
        run_result = run / "result.json"
        termination = (
            json.loads(run_result.read_text())["termination"] if run_result.is_file() else None
        )
        write_json(directory / "batch_progress.json", {
            "recorded_runs": recorded_count, "planned_runs": 1080,
            "last_run": row["id"], "last_returncode": process.returncode,
            "stop_reason": "budget_exhausted" if termination == "budget_exhausted" else None,
        })
        print(f"正式运行 {row['id']}；已有记录 {recorded_count}/1080", flush=True)
        if termination == "budget_exhausted":
            break
        require_formal_capacity(directory, manifest)
        if limit is not None and executed >= limit:
            break
    return paired_report(directory, manifest=manifest)


def _run_metrics(report):
    recovery = report["execution"]["recovery"]
    feedback = report["feedback"]
    return {
        "completion_time_s": report.get("completion_time_s"),
        "observed_time_s": feedback["simulation_duration_s"],
        "final_progress_m": report["coverage"]["final_progress_m"],
        "uncompleted_length_m": report["coverage"]["unreached_projected_length_m"],
        "effective_feed_m_s": report["coverage"]["final_progress_m"] / feedback["simulation_duration_s"],
        "pause_time_s": sum(row["duration_s"] for row in recovery["pauses"]),
        "restart_count": recovery["restart_count"],
        "maximum_recovery_delay_s": max((
            row["recovery_delay_s"] for row in recovery["events"] if row["recovery_delay_s"] is not None
        ), default=None),
        "path_rms_m": report["metrics"]["path_distance_m"]["rms"],
        "path_maximum_m": report["metrics"]["path_distance_m"]["maximum"],
        "axis_rms_rad": report["metrics"]["tool_axis_angle_rad"]["rms"],
        "axis_maximum_rad": report["metrics"]["tool_axis_angle_rad"]["maximum"],
        "external_minimum_m": report["external_clearance"].get("minimum_m"),
        "self_minimum_m": report["self_clearance"].get("minimum_m"),
        "cycle_p999_s": report["execution"]["timing"]["p999_s"],
        "cycle_maximum_s": report["execution"]["timing"]["maximum_s"],
    }


def paired_report(directory: Path, *, manifest=None) -> dict:
    directory = directory.resolve(strict=True)
    manifest = load_formal(directory) if manifest is None else manifest
    rows, lookup = [], {}
    for entry in manifest["runs"]:
        run = directory / "runs" / entry["id"]
        record = {
            **entry, "status": "not_run", "functional_verdict": "not_run",
            "runtime_verdict": "not_run", "completed": False, "metrics": {},
        }
        if run.exists():
            record["status"] = "insufficient_evidence"
            record["functional_verdict"] = record["runtime_verdict"] = "insufficient_evidence"
            recorded = json.loads((run / "manifest.json").read_text()) if (run / "manifest.json").exists() else None
            if recorded is not None and (
                recorded["config_sha256"] != entry["config_sha256"]
                or recorded["source_sha256"] != manifest["source_sha256"]
            ):
                raise ValueError(f"正式原始记录身份不一致：{run}")
            report_path = run / "feedback_report.json"
            if report_path.exists():
                recorded, report = validate_run_report(run)
                record.update(status=report["verdict"], completed=report["completed"],
                              functional_verdict=report["functional_verdict"],
                              runtime_verdict=report["runtime_verdict"],
                              report_sha256=content_hash(report_path), termination=report["termination"])
                if "execution" in report:
                    record["metrics"] = _run_metrics(report)
        rows.append(record)
        lookup[(entry["scene"], entry["initial"], entry["feed_factor"], entry["repeat"], entry["method"])] = record
    paired = {}
    for first, second in (("A", "B"), ("C", "D"), ("A", "C"), ("B", "D")):
        pairs = []
        for scene, initial, feed, repeat in itertools.product(SCENARIOS, (1, 2, 3), (.5, 1., 1.5), (1, 2, 3)):
            key = scene, initial, feed, repeat
            a, b = lookup[(*key, first)], lookup[(*key, second)]
            differences = {
                metric: b["metrics"][metric] - value
                for metric, value in a["metrics"].items()
                if value is not None and b["metrics"].get(metric) is not None
            }
            pairs.append({
                "first": a["id"], "second": b["id"], "first_status": a["status"], "second_status": b["status"],
                "both_completed": a["completed"] and b["completed"],
                "difference_second_minus_first": differences,
            })
        names = {name for row in pairs for name in row["difference_second_minus_first"]}
        paired[f"{first}/{second}"] = {
            "planned_pairs": len(pairs),
            "both_completed": sum(row["both_completed"] for row in pairs),
            "metrics": {
                name: {
                    "paired_observations": len(values),
                    "mean_difference": float(np.mean(values)), "median_difference": float(np.median(values)),
                }
                for name in sorted(names)
                for values in [[row["difference_second_minus_first"][name] for row in pairs
                                if name in row["difference_second_minus_first"]]]
            },
            "pairs": pairs,
        }
    counts = Counter(row["status"] for row in rows)
    eligible = [
        row for row in rows if row["method"] == "D"
        and (row["scene"] == "s5" or manifest["configurations"][row["configuration"]]["feasibility"] == "proven")
    ]
    functional = ("fail" if any(row["functional_verdict"] == "fail" for row in eligible) else
                  "pass" if eligible and all(row["functional_verdict"] == "pass" for row in eligible)
                  else "insufficient_evidence")
    result = {
        "schema_version": 1, "planned_runs": len(rows),
        "recorded_runs": len(rows) - counts["not_run"], "status_counts": dict(counts),
        "collection_complete": counts["not_run"] == 0,
        "d_functional_verdict": functional, "d_eligible_runs": len(eligible),
        "d_excluded_unproven_runs": 216 - len(eligible),
        "d_eligible_scenes": dict(Counter(row["scene"] for row in eligible)),
        "formal_manifest_sha256": content_hash(directory / "formal_manifest.json"),
        "methods": {
            method: {
                "planned": 216, "completed": sum(row["completed"] for row in rows if row["method"] == method),
                "statuses": dict(Counter(row["status"] for row in rows if row["method"] == method)),
            } for method in METHODS
        },
        "comparisons": paired, "runs": rows,
    }
    write_json(directory / "paired_report.json", result)
    text = [
        "# 静态障碍物正式配对实验", "",
        f"已有记录：{result['recorded_runs']} / {len(rows)}；D 组功能：`{functional}`。",
        f"D 组功能评价包含 {len(eligible)} 项；{216 - len(eligible)} 项缺少预先可行性证据。",
        "", "| 方法 | 计划数量 | 完成数量 | 结果 |", "|---|---:|---:|---|",
        *[f"| {method} | 216 | {row['completed']} | {json.dumps(row['statuses'], ensure_ascii=False)} |"
          for method, row in result["methods"].items()],
        "", "配对差值采用后一个方法减前一个方法；各指标单独报告成对观测数量。",
        "未完成运行保留已观测指标，完成时间只用于完整完成的运行。所有缺失组合保留在清单中。",
    ]
    (directory / "paired_report.md").write_text("\n".join(text) + "\n", encoding="utf-8")
    return result
