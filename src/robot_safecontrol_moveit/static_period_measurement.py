from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
import subprocess
import sys

from .static_experiment_config import ExperimentConfig, METHODS, content_hash
from .static_experiment_provenance import environment_identity, validate_run_report
from .static_experiment_recording import write_json
from .static_cycle_timing import CYCLE_TIMING_BOUNDARY, cycle_timing_summary, load_cycle_timing


PERIODS = (.02, .04, .05, .1)


def method_identity(values: dict) -> str:
    for method, (solver, recovery) in METHODS.items():
        if (values["control"]["solver_mode"], values["control"]["recovery"]["mode"]) == (solver, recovery):
            return method
    raise ValueError("配置不属于 A–E 方法组合")


def common_configuration(values: dict) -> dict:
    common = deepcopy(values)
    del common["control"]["solver_mode"]
    del common["control"]["period_s"]
    del common["control"]["recovery"]["mode"]
    return common


def select_period(rows: list[dict]) -> dict:
    expected = {(period, method) for period in PERIODS for method in METHODS}
    keys = [(row["period_s"], row["method"]) for row in rows]
    if len(set(keys)) != len(keys) or not set(keys) <= expected:
        raise ValueError("周期测量包含重复或无效的方法组合")
    for row in rows:
        if row["status"] == "measured" and (
            not isinstance(row["samples"], int) or row["samples"] <= 0
            or not math.isfinite(row["p999_s"]) or not math.isfinite(row["maximum_s"])
            or not 0 <= row["p999_s"] <= row["maximum_s"]
            or not isinstance(row["deadline_misses"], int)
            or not 0 <= row["deadline_misses"] <= row["samples"]
        ):
            raise ValueError("周期测量包含无效的样本数或耗时")
    qualified = []
    groups = []
    for period in PERIODS:
        measured = [row for row in rows if row["period_s"] == period]
        missing = sorted(set(METHODS) - {row["method"] for row in measured})
        passed = not missing and all(
            row["status"] == "measured" and row["samples"] > 0
            and row.get("cycle_timing_boundary") == CYCLE_TIMING_BOUNDARY
            and row["p999_s"] <= .8 * period for row in measured
        )
        if passed:
            qualified.append(period)
        groups.append({"period_s": period, "qualified": passed, "missing_methods": missing,
                       "methods": measured})
    complete = set(keys) == expected and all(
        row["status"] == "measured" and row.get("cycle_timing_boundary") == CYCLE_TIMING_BOUNDARY
        for row in rows
    )
    return {
        "status": "insufficient_evidence" if not complete else "pass" if qualified else "fail",
        "reason": ("missing_measurements" if not complete else
                   "all_periods_exceed_budget" if not qualified else "shortest_qualified_period"),
        "selected_period_s": min(qualified) if complete and qualified else None,
        "cycle_timing_boundary": CYCLE_TIMING_BOUNDARY,
        "groups": groups,
    }


def load_period_storage_budgets(path: Path) -> dict:
    entries = json.loads(path.read_text())
    expected = {f"period-{round(period * 1000)}-{method}"
                for period in PERIODS for method in METHODS}
    if set(entries) != expected:
        raise ValueError("周期测量容量索引必须包含完整 20 项")
    result = {}
    for name, item in entries.items():
        filename = (path.parent / item["path"]).resolve(strict=True)
        if content_hash(filename) != item["sha256"]:
            raise ValueError(f"周期测量容量文件身份变化：{name}")
        result[name] = {**item, "path": str(filename)}
    return result


def validate_period_storage(run: Path) -> dict:
    manifest = json.loads((run / "manifest.json").read_text())
    storage = manifest.get("storage_budget")
    if storage is None:
        raise ValueError(f"周期测量缺少运行期间容量守卫：{run}")
    result = json.loads((run / "result.json").read_text())
    identities = {}
    for role in ("plant", "controller"):
        path = run / f"{role}.recording.json"
        identities[role] = content_hash(path)
        statistics = json.loads(path.read_text())
        guard = statistics["storage_budget"]
        if (identities[role] != result["recording_statistics_sha256"][role]
                or guard is None or guard["limits"] != storage["writers"][role]
                or guard["reason"] is not None or not guard["completion_within_reserve"]):
            raise ValueError(f"周期测量容量守卫统计未通过：{run}/{role}")
    return identities


def run_period_study(config: ExperimentConfig, directory: Path, domain: int, *,
                     storage_budget_index: Path | None = None) -> dict:
    if storage_budget_index is None:
        raise ValueError("周期测量需要完整容量预算索引")
    budgets = load_period_storage_budgets(storage_budget_index)
    directory = directory.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    study_path = directory / "study.json"
    manifest = config.manifest()
    environment = environment_identity()
    from .static_experiment import _sources

    identity = {
        "schema_version": 1, "kind": "development_period_study",
        "common_configuration": common_configuration(manifest["config"]),
        "source_sha256": _sources(config), "environment": environment,
        "periods_s": list(PERIODS), "methods": list(METHODS), "domain": domain,
        "storage_budget_index_sha256": content_hash(storage_budget_index),
        "storage_budgets": budgets,
        "cycle_timing_boundary": CYCLE_TIMING_BOUNDARY,
    }
    if study_path.exists():
        previous = json.loads(study_path.read_text())
        if (previous["common_configuration"] != identity["common_configuration"]
                or previous["source_sha256"] != identity["source_sha256"]
                or previous["environment"]["sha256"] != environment["sha256"]
                or previous.get("storage_budgets") != budgets
                or previous.get("storage_budget_index_sha256")
                != identity["storage_budget_index_sha256"]
                or previous.get("cycle_timing_boundary") != CYCLE_TIMING_BOUNDARY):
            raise ValueError("周期测量的配置、代码或设备环境已变化")
    else:
        if any(directory.iterdir()):
            raise ValueError("周期测量目录包含未识别内容")
        write_json(study_path, identity)
    rows = []
    for period in PERIODS:
        for method in METHODS:
            run = directory / f"period-{round(period * 1000)}-{method}"
            run_config = ExperimentConfig.load(
                config.source, resource_root=config.resource_root, method=method, period_s=period,
                **{key: value for key, value in config.overrides.items()
                   if key not in ("method", "solver_mode", "recovery_mode", "period_s")},
            )
            budget = budgets[run.name]
            storage = json.loads(Path(budget["path"]).read_text())
            if (content_hash(Path(budget["path"])) != budget["sha256"]
                    or storage["config_sha256"] != run_config.identity
                    or storage["source_sha256"] != identity["source_sha256"]):
                raise ValueError(f"周期测量容量配置或源码身份不一致：{run.name}")
            if not run.exists():
                with (directory / f"{run.name}.log").open("x", encoding="utf-8") as log:
                    process = subprocess.run(
                        [sys.executable, "-m", "robot_safecontrol_moveit.static_experiment",
                         "run", "--config", str(config.source), "--output", str(run),
                         "--resource-root", str(config.resource_root), "--domain", str(domain),
                         "--storage-budget", budget["path"],
                         *[value for key, setting in run_config.overrides.items()
                           for value in (f"--{key.replace('_', '-')}", str(setting))]],
                        stdout=log, stderr=subprocess.STDOUT, check=False,
                    )
                write_json(directory / f"{run.name}.process.json", {"returncode": process.returncode})
            row = {"method": method, "period_s": period, "run": run.name}
            if not (run / "manifest.json").is_file():
                row.update(status="insufficient_evidence", reason="missing_manifest")
            else:
                recorded = json.loads((run / "manifest.json").read_text())
                if (recorded["config_sha256"] != run_config.identity
                        or recorded["source_sha256"] != identity["source_sha256"]
                        or recorded.get("storage_budget") != storage):
                    raise ValueError(f"周期测量来源不一致：{run}")
                statistics = validate_period_storage(run)
                report_path = run / "feedback_report.json"
                report = validate_run_report(run)[1] if report_path.exists() else {}
                timing = report.get("execution", {}).get("timing", {})
                measured = load_cycle_timing(run, recorded)
                expected_timing = cycle_timing_summary(measured, period)
                if timing.get("samples", 0) and measured["eligible_for_period"]:
                    if any(timing.get(key) != value for key, value in expected_timing.items()):
                        raise ValueError(f"周期测量报告与完整计时原件不一致：{run}")
                    row.update(
                        status="measured", samples=timing["samples"], p999_s=timing["p999_s"],
                        maximum_s=timing["maximum_s"], deadline_misses=timing["deadline_misses"],
                        admission_rejections=timing["admission_rejections"],
                        functional_verdict=report["task_verdict"], run_verdict=report["verdict"],
                        report_sha256=content_hash(report_path),
                        records_sha256=report["records_sha256"],
                        storage_budget_sha256=budget["sha256"],
                        storage_statistics_sha256=statistics,
                        cycle_timing_boundary=CYCLE_TIMING_BOUNDARY,
                        cycle_timing_sha256=measured["sidecar_sha256"],
                    )
                else:
                    row.update(status="insufficient_evidence", reason="missing_complete_cycles")
            rows.append(row)
            report = select_period(rows)
            report["study_sha256"] = content_hash(study_path)
            write_json(directory / "period_report.json", report)
            print(f"周期测量 {period * 1000:g} ms / {method}: {row['status']}", flush=True)
    return report
