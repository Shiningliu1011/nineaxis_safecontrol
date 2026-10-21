import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time


def require(condition, message):
    if not condition:
        raise ValueError(message)


def load(path):
    return json.loads(Path(path).read_text())


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1048576), b""):
            value.update(chunk)
    return value.hexdigest()


def validate_capacity_documents(engineering_path, capacity_path, sources):
    engineering, capacity = load(engineering_path), load(capacity_path)
    require(engineering["source_sha256"] == sources and capacity["source_sha256"] == sources,
            "工程容量与当前生产源码身份不一致")
    require(engineering["status"] == "pass" and engineering["checks"]
            and all(engineering["checks"].values()) and capacity["status"] == "pass"
            and capacity["shortfall_bytes"] == 0 and not capacity["generation"]["formal_bound"],
            "当前工程容量尚未通过")
    require(Path(capacity["engineering_evidence"]).resolve() == Path(engineering_path).resolve()
            and capacity["evidence_sha256"][str(Path(engineering_path).resolve())] == digest(engineering_path),
            "工程容量证据来源不一致")
    for path, identity in {**engineering["evidence_sha256"], **capacity["evidence_sha256"]}.items():
        require(digest(path) == identity, f"工程容量输入身份变化：{path}")
    return engineering, capacity


def validate_study_documents(directory, boundary):
    directory = Path(directory)
    study, report = load(directory / "study.json"), load(directory / "period_report.json")
    require(study.get("cycle_timing_boundary") == boundary
            and report.get("cycle_timing_boundary") == boundary, "共同周期计时边界不符")
    require(report["study_sha256"] == digest(directory / "study.json"), "共同周期 study 身份不一致")
    expected = {(period, method) for period in (.02, .04, .05, .1) for method in "ABCDE"}
    rows = [row for group in report["groups"] for row in group["methods"]]
    measured = [(row["period_s"], row["method"]) for row in rows]
    require(len(measured) == 20 and set(measured) == expected, "共同周期缺少完整 20 项测量")
    require(all(row["status"] != "measured" or row.get("cycle_timing_boundary") == boundary for row in rows),
            "共同周期计时边界不符")
    return study, report, rows


def bind_revision(root, revision):
    resolved = subprocess.run(
        ["git", "rev-parse", "--verify", revision + "^{commit}"], cwd=root,
        check=True, capture_output=True, text=True,
    ).stdout.strip()
    differences = subprocess.run(
        ["git", "diff", "--exit-code", resolved, "--", "src", "portable_oscbf"],
        cwd=root, capture_output=True, text=True,
    )
    require(differences.returncode == 0, "生产源码与 runtime revision 不一致")
    untracked = subprocess.run(
        ["git", "ls-files", "--others", "--exclude-standard", "--", "src", "portable_oscbf"],
        cwd=root, check=True, capture_output=True, text=True,
    ).stdout.strip()
    require(not untracked, "生产源码目录包含未登记文件")
    return resolved


def study_plan(root, revision):
    capacity = root / "output/static-capacity-20261020"
    preflight = capacity / "complete-cycle-preflight-01"
    volume = Path("/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020")
    return {
        "runtime_revision": revision,
        "cases": [f"period-{period}-{method}" for period in (20, 40, 50, 100) for method in "ABCDE"],
        "periods_s": [.02, .04, .05, .1], "methods": list("ABCDE"),
        "domain": 200, "worker_cpus": "0-15", "monitor_cpus": "22-27",
        "preflight": str(preflight), "volume": str(volume), "filesystem_uuid": "083CBB753CBB5BFE",
        "engineering": str(preflight / "engineering-plan/engineering_evidence.json"),
        "capacity_plan": str(preflight / "engineering-plan/capacity_plan.json"),
        "config": str(root / "output/static-scene-validation-20261020/development-scenes-07/s4.yaml"),
        "budgets": str(capacity / "guarded-period-budgets-02"),
        "study": str(volume / "guarded-period-study-02"),
    }


def main():
    parser = argparse.ArgumentParser(description="完整周期 study02 的准备与串行执行入口")
    parser.add_argument("--resource-root", type=Path, required=True)
    parser.add_argument("--runtime-revision", required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check-only", action="store_true")
    mode.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    root = args.resource_root.resolve(strict=True)
    revision = bind_revision(root, args.runtime_revision)
    plan = study_plan(root, revision)
    required = [Path(plan["engineering"]), Path(plan["capacity_plan"]),
                Path(plan["preflight"]) / "plan.json", Path(plan["preflight"]) / "completed.json"]
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        print(json.dumps({"status": "pending_capacity", "missing": missing, **plan}, ensure_ascii=False))
        return 2
    sys.path.insert(0, str(root / "src"))
    from robot_safecontrol_moveit.static_cycle_timing import CYCLE_TIMING_BOUNDARY, load_cycle_timing
    from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig, METHODS
    from robot_safecontrol_moveit.static_experiment_provenance import source_identity
    from robot_safecontrol_moveit.static_experiment_recording import write_json
    from robot_safecontrol_moveit.static_period_measurement import PERIODS, load_period_storage_budgets

    require(list(PERIODS) == plan["periods_s"] and list(METHODS) == plan["methods"], "公共 benchmark 的测量矩阵变化")
    sources = source_identity(root)
    engineering, _ = validate_capacity_documents(plan["engineering"], plan["capacity_plan"], sources)
    preflight = Path(plan["preflight"])
    preflight_plan, completed = load(preflight / "plan.json"), load(preflight / "completed.json")
    require(completed["status"] == "completed" and preflight_plan["source_sha256"] == sources
            and preflight_plan["cycle_timing_boundary"] == CYCLE_TIMING_BOUNDARY,
            "完整周期 preflight 或来源身份尚未通过")
    expected_preflight = {
        "ext4.d-s5", "ntfs.d-s5", "windows.d-s5", "ntfs.d-s0",
        "ntfs.d-s4-initial3-feed0.5", "ntfs.d-timeout", "ntfs.d-wal-limit",
    }
    require({row["name"] for row in completed["runs"]} == expected_preflight
            and len(completed["runs"]) == 7, "完整周期 preflight 项目缺失")
    require(all(row["runtime_timing"]["boundary"] == CYCLE_TIMING_BOUNDARY
                and row["runtime_timing"]["eligible_for_period"]
                and (row["expected_failure"] or row["functional_verdict"] == "pass")
                for row in completed["runs"]), "preflight 完整计时或正常项目功能未通过")
    volume, budgets, study = (Path(plan[key]) for key in ("volume", "budgets", "study"))
    require(volume.resolve(strict=True) == volume, "共同周期输出卷路径包含重定向")
    filesystem = json.loads(subprocess.run(
        ["findmnt", "--json", "--target", str(volume), "--output", "SOURCE,TARGET,FSTYPE,OPTIONS,UUID"],
        cwd=root, check=True, capture_output=True, text=True,
    ).stdout)["filesystems"][0]
    require(filesystem["uuid"] == plan["filesystem_uuid"] and "rw" in filesystem["options"].split(","),
            "共同周期输出卷身份或读写状态不符")
    require(not study.exists(), "study02 输出已存在，保留现有测量后终止本次启动")
    identity_paths = [*required, Path(plan["config"]), Path(__file__).resolve(),
                      root / "scripts/prepare_static_storage_budget.py",
                      root / "scripts/static_obstacle_experiment.py", root / ".scratch/host-recovery/env.sh"]
    inputs = {str(path): digest(path) for path in identity_paths}
    plan.update(source_sha256=sources, cycle_timing_boundary=CYCLE_TIMING_BOUNDARY,
                input_sha256=inputs, filesystem=filesystem)
    index = budgets / "index.json"
    prepare_command = [
        "taskset", "-c", "16-21", sys.executable, str(root / "scripts/prepare_static_storage_budget.py"),
        "--engineering-evidence", plan["engineering"], "--config", plan["config"],
        "--resource-root", str(root), "--volume", str(volume), "--maximum-pairs", "76",
        "--output", str(index), "--study-index",
    ]
    benchmark_command = [
        "taskset", "-c", "0-15", sys.executable, str(root / "scripts/static_obstacle_experiment.py"),
        "benchmark", "--config", plan["config"], "--resource-root", str(root), "--domain", "200",
        "--storage-budget-index", str(index), "--output", str(study),
    ]
    plan.update(prepare_command=prepare_command, benchmark_command=benchmark_command)

    def require_current_inputs():
        bind_revision(root, revision)
        require(source_identity(root) == sources, "共同周期生产源码或 native 身份变化")
        require(all(digest(path) == expected for path, expected in inputs.items()), "共同周期输入身份变化")

    def inspect_budgets():
        entries = load_period_storage_budgets(index)
        require(set(entries) == set(plan["cases"]), "预算索引未完整覆盖 20 项")
        allocations = []
        for period in PERIODS:
            for method in METHODS:
                name = f"period-{round(period * 1000)}-{method}"
                item = entries[name]
                path = Path(item["path"])
                require(path.parent == budgets and path.name == name + ".storage.json", "预算目录身份不符")
                storage = load(path)
                configuration = ExperimentConfig.load(Path(plan["config"]), resource_root=root,
                                                      method=method, period_s=period)
                require(storage["source_sha256"] == sources and storage["config_sha256"] == configuration.identity
                        and storage["evidence_sha256"] == inputs[plan["engineering"]]
                        and storage["filesystem_uuid"] == plan["filesystem_uuid"]
                        and storage["run_id"] == name and storage["development_only"], "共同周期预算身份不一致")
                allocations.append(storage)
        required_bytes = (
            sum(item["retained_run_bytes"] for item in allocations)
            + max(sum(writer["wal_bytes"] for writer in item["writers"].values()) for item in allocations)
            + max(item["reserve_bytes"] for item in allocations)
            + engineering["per_volume_budget"]["derived_workspace_bytes"]
            + engineering["per_volume_budget"]["provenance_bytes"]
        )
        free = shutil.disk_usage(volume).free
        require(free >= required_bytes, f"完整 20 项保留数据与瞬时空间不足：{free} < {required_bytes}")
        return {"entries": entries, "required_free_bytes": required_bytes, "available_bytes": free}

    require_current_inputs()
    prepared_path = budgets / "prepared.json"
    if budgets.exists():
        prepared = load(prepared_path)
        require(prepared["plan"] == plan and prepared["index_sha256"] == digest(index), "已有 budgets02 身份不符")
        allocation = inspect_budgets()
    elif args.check_only:
        print(json.dumps({"status": "ready_for_preparation", **plan}, ensure_ascii=False, indent=2))
        return 0
    else:
        os.sched_setaffinity(0, set(range(22, 28)))
        budgets.mkdir()
        write_json(budgets / "plan.json", plan)
        with (budgets / "preparation.log").open("x") as log:
            subprocess.run(prepare_command, cwd=root, stdout=log, stderr=subprocess.STDOUT, check=True)
        require_current_inputs()
        allocation = inspect_budgets()
        write_json(prepared_path, {"plan": plan, "index_sha256": digest(index), "allocation": allocation})
    if not args.execute:
        print(json.dumps({"status": "prepared", "budgets": str(budgets), "study_started": False,
                          "allocation": allocation}, ensure_ascii=False))
        return 0

    require_current_inputs()
    os.sched_setaffinity(0, set(range(22, 28)))
    execution_path = budgets / "execution.json"
    require(not execution_path.exists(), "执行记录已存在，保留现有记录后终止本次启动")
    execution = {"status": "running", "plan_sha256": digest(budgets / "plan.json"),
                 "prepared_sha256": digest(prepared_path), "command": benchmark_command,
                 "started_monotonic_s": time.monotonic(), "monitor_affinity": sorted(os.sched_getaffinity(0)),
                 "runtime_revision": revision, "source_sha256": sources}
    write_json(execution_path, execution)
    try:
        with (budgets / "benchmark.log").open("x") as log:
            process = subprocess.run(benchmark_command, cwd=root, stdout=log, stderr=subprocess.STDOUT)
        execution.update(returncode=process.returncode, elapsed_s=time.monotonic() - execution["started_monotonic_s"])
        write_json(execution_path, execution)
        require_current_inputs()
        identity, report, rows = validate_study_documents(study, CYCLE_TIMING_BOUNDARY)
        require(identity["source_sha256"] == sources and identity["storage_budget_index_sha256"] == digest(index),
                "共同周期测量来源或预算索引不一致")
        records = []
        for row in rows:
            run = study / row["run"]
            measured = load_cycle_timing(run)
            require(measured["eligible_for_period"] and measured["boundary"] == CYCLE_TIMING_BOUNDARY,
                    "共同周期 run 缺少完整计时")
            records.append({
                **row, "process": load(study / f"{run.name}.process.json"),
                "sidecar_sha256": measured["sidecar_sha256"],
                "sha256": {path.name: digest(path) for path in sorted(run.iterdir()) if path.is_file()
                           and path.suffix in (".sqlite3", ".json", ".ready", ".yaml", ".log")},
            })
        require(process.returncode == (0 if report["status"] == "pass" else 1), "benchmark 退出码与报告不一致")
        execution.update(status="completed", period_status=report["status"],
                         selected_period_s=report["selected_period_s"], runs=records,
                         study_sha256=digest(study / "study.json"),
                         period_report_sha256=digest(study / "period_report.json"))
        print(json.dumps({"status": "completed", "period_status": report["status"],
                          "selected_period_s": report["selected_period_s"], "study": str(study)}, ensure_ascii=False))
        return process.returncode
    finally:
        if execution["status"] == "running":
            execution["status"] = "incomplete"
        write_json(execution_path, execution)
if __name__ == "__main__":
    raise SystemExit(main())
