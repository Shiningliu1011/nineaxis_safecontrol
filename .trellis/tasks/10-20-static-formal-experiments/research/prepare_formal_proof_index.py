import argparse
import json
from pathlib import Path
import sys


REGISTRATION_SHA256 = "0f27070df3e6bd707426db43b85ff61d442e93f89aa55af3fdf12a319e7f3c9e"
PLAN_SHA256 = "172575eb2694e2d57e7214fc3a7d1e30a8a38e6c4c324b3bb08d1392f4a60935"
NAMES = {"d-s2-initial2-feed1": 2, "d-s2-initial3-feed1": 3}


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def require(condition, message):
    if not condition:
        raise ValueError(message)


def main():
    parser = argparse.ArgumentParser(description="补充两个S2完整来源并生成正式证据索引")
    parser.add_argument("--resource-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--control-directory", type=Path)
    args = parser.parse_args()
    root = args.resource_root.resolve(strict=True)
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError(output)
    control = (args.control_directory or root / "output/static-capacity-20261020/guarded-s2-initials-01").resolve()

    # 两项结果齐备后才加载验证依赖与读取大型原件。
    completed = read_json(control / "completed.json")
    execution = read_json(control / "execution.json")
    status = read_json(control / "status.json")
    results = completed["results"]
    require(len(results) == 2 and {row["name"] for row in results} == set(NAMES), "两项S2完成结果不齐全")
    require(status["running"] is None and status["completed"] == results, "S2队列尚未完整结束")
    require(completed["source_sha256"] == execution["source_sha256"], "S2执行与完成源码身份不同")
    for row in results:
        run = Path(row["run"]).resolve(strict=True)
        for name in ("manifest.json", "result.json", "feedback_report.json", "controller.sqlite3", "plant.sqlite3"):
            require((run / name).is_file(), f"完整来源缺少原件：{run / name}")
        require((run.parent / "lifecycle-validation.json").is_file(), "缺少生命周期报告")
    require((control / "source/snapshot.json").is_file(), "缺少两项S2源码归档")

    sys.path.insert(0, str(root / "src"))
    from robot_safecontrol_moveit.static_experiment_config import content_hash
    from robot_safecontrol_moveit.static_experiment_provenance import validate_run_report, verify_snapshot
    from robot_safecontrol_moveit.static_experiment_recording import write_json
    from robot_safecontrol_moveit.static_formal_experiments import _physical_identity
    from robot_safecontrol_moveit.static_period_measurement import validate_period_storage

    registration_path = root / ".trellis/tasks/10-20-static-scene-witnesses/research/formal-scene-input.json"
    require(content_hash(registration_path) == REGISTRATION_SHA256, "正式物理输入登记身份变化")
    registration = read_json(registration_path)
    require(registration["kind"] == "formal_physical_scene_input", "正式物理输入类型不符")
    for key in ("development_catalog", "physical_configurations", "feasibility_snapshot", "s2_selection",
                "s4_witness", "s4_states", "s5_blockage"):
        entry = registration[key]
        require(content_hash(Path(entry["path"])) == entry["sha256"], f"登记资源身份变化：{key}")
    configurations = read_json(registration["physical_configurations"]["path"])
    previous_matrix = read_json(registration["feasibility_snapshot"]["path"])
    require(len(configurations) == len(previous_matrix) == 72, "物理配置数量不符")
    require(len({item["id"] for item in configurations}) == 72, "物理配置标识重复")
    require(sum(row["status"] == "proven" for row in previous_matrix) == 33, "基础可行性快照数量不符")
    require({row["id"] for row in previous_matrix} == {row["id"] for row in configurations}, "基础矩阵配置身份不齐全")

    plan_path = root / "output/static-scene-validation-20261020/guarded-scene-evidence-01/pending-s2-initials-plan.json"
    require(content_hash(plan_path) == PLAN_SHA256 == execution["plan_sha256"], "S2计划身份变化")
    plan = read_json(plan_path)
    plans = {entry["name"]: entry for entry in plan["entries"]}
    require(set(plans) == set(NAMES), "S2计划条目不符")
    for entry in plans.values():
        require(content_hash(Path(entry["configuration"])) == entry["configuration_sha256"], "S2配置文件身份变化")

    snapshot = verify_snapshot(control / "source")
    archived_sources = {item["original_path"]: item["sha256"] for item in snapshot["files"].values()}
    require(bool(execution["source_sha256"]), "S2源码身份为空")
    for path, expected in execution["source_sha256"].items():
        require(archived_sources.get(path) == expected, f"S2源码归档不含对应来源：{path}")

    declared = {}
    for row in previous_matrix:
        for source in row["complete_run_evidence"]:
            run = str(Path(source["run"]).resolve(strict=True))
            item = declared.setdefault(run, {"existing": [], "new": None})
            item["existing"].append(source)
    for result in results:
        run = Path(result["run"]).resolve(strict=True)
        entry = plans[result["name"]]
        command = entry["validate_command"]
        expected_run = (Path(command[command.index("--output") + 1]) / "run").resolve(strict=True)
        require(run == expected_run, "S2完成目录与计划不同")
        item = declared.setdefault(str(run), {"existing": [], "new": None})
        require(item["new"] is None, "S2完成目录重复")
        item["new"] = result

    physical_cache = {}
    physical_configurations = []
    for item in configurations:
        path = Path(item["configuration"])
        require(content_hash(path) == item["sha256"], f"物理配置文件身份变化：{path}")
        cache_key = json.dumps(item["physical_identity"], sort_keys=True)
        if cache_key not in physical_cache:
            physical_cache[cache_key] = _physical_identity(item["manifest"], root)
        physical_configurations.append({**item, "physical_identity": physical_cache[cache_key]})

    verified = []
    for run_name, expected in sorted(declared.items()):
        run = Path(run_name)
        manifest, report = validate_run_report(run)
        result = read_json(run / "result.json")
        require(report["completed"] and report["task_verdict"] == "pass", f"来源未完整通过：{run}")
        require(all(report[key]["status"] == "pass" for key in ("external_clearance", "self_clearance")),
                f"来源几何未通过：{run}")
        require(set(report["records_sha256"]) == {"controller", "plant"}, f"来源数据库不齐全：{run}")
        require(bool(manifest["source_sha256"]), f"来源缺少源码身份：{run}")
        identity = {
            "report_sha256": content_hash(run / "feedback_report.json"),
            "manifest_sha256": report["manifest_sha256"], "records_sha256": report["records_sha256"],
        }
        for old in expected["existing"]:
            require(all(old[key] == value for key, value in identity.items()), f"既有完整来源身份变化：{run}")
        physical = _physical_identity(manifest, root)
        matches = [item["id"] for item in physical_configurations if item["physical_identity"] == physical]
        require(bool(matches), f"完整来源与已登记配置不匹配：{run}")
        source_binding = {"kind": "registered_manifest_sha256", "manifest_sha256": identity["manifest_sha256"]}
        if expected["new"] is not None:
            current = expected["new"]
            entry = plans[current["name"]]
            require(manifest["source_sha256"] == completed["source_sha256"], "新增S2源码身份不同")
            require(manifest["config_sha256"] == entry["effective_config_sha256"], "新增S2有效配置身份不同")
            require(result["worker_exit_codes"] == [0, 0] and result["termination"] == "completed", "新增S2未正常完成")
            expected_hashes = {
                "manifest.json": identity["manifest_sha256"], "feedback_report.json": identity["report_sha256"],
                "result.json": report["result_sha256"],
                **{f"{role}.sqlite3": value for role, value in report["records_sha256"].items()},
            }
            require(current["sha256"] == expected_hashes, "新增S2完成记录身份不同")
            lifecycle_path = run.parent / "lifecycle-validation.json"
            require(content_hash(lifecycle_path) == current["validation_sha256"], "新增S2生命周期报告身份变化")
            lifecycle = read_json(lifecycle_path)
            require(lifecycle["status"] == "pass" and lifecycle["inspection"]["within_storage_limits"] is True,
                    "新增S2生命周期或容量守卫未通过")
            prepare = entry["prepare_budget"]
            budget_path = Path(prepare[prepare.index("--output") + 1])
            require(manifest.get("storage_budget") == read_json(budget_path), "新增S2容量预算身份不同")
            guard_statistics = validate_period_storage(run)
            expected_matches = {f"s2-initial-{NAMES[current['name']]}-feed-{feed}" for feed in ("0.5", "1", "1.5")}
            require(set(matches) == expected_matches, "新增S2物理身份与目标初态不同")
            source_binding = {"kind": "verified_snapshot", "snapshot_sha256": content_hash(control / "source/snapshot.json"),
                              "archive_sha256": snapshot["archive_sha256"], "validation_sha256": current["validation_sha256"],
                              "storage_budget_sha256": content_hash(budget_path), "guard_statistics_sha256": guard_statistics}
        verified.append({
            "run": str(run), **identity, "result_sha256": report["result_sha256"],
            "completed": True, "task_verdict": "pass", "geometry_pass": True, "identity_status": "pass",
            "physical_identity": physical, "matching_configurations": matches,
            "source_sha256": manifest["source_sha256"], "source_binding": source_binding,
        })
        print(json.dumps({"validated": str(run), "configurations": matches}), flush=True)

    matrix = []
    for item in configurations:
        sources = [row for row in verified if item["id"] in row["matching_configurations"]]
        matrix.append({key: item[key] for key in ("id", "scene_id", "initial", "feed_factor")} | {
            "status": "proven" if sources else "insufficient_evidence", "complete_run_evidence": sources,
            "scope": "同场景、初态、执行限制、路径与模型的完整物理路径；各进给功能分别验证",
        })
    require(sum(row["status"] == "proven" for row in matrix) == 39, "增补后可行性覆盖范围不符")
    witness = Path(registration["s4_witness"]["path"])
    witness_record = read_json(witness)
    require(witness_record["status"] == "proven", "S4见证状态不符")
    require(witness_record["states_sha256"] == registration["s4_states"]["sha256"], "S4见证状态数组身份不同")
    blockage = Path(registration["s5_blockage"]["path"])
    require(read_json(blockage)["status"] == "proven", "S5阻断证据状态不符")
    rationale = root / ".trellis/tasks/10-20-static-method-matrix/research/parameter-selection.md"
    require(bool(rationale.read_text(encoding="utf-8").strip()), "参数依据为空")
    index = {"parameter_rationale": str(rationale.resolve(strict=True)),
             "feasibility_runs": [row["run"] for row in verified],
             "s4_witness": str(witness.parent.resolve(strict=True)), "s5_blockage": str(blockage.resolve(strict=True))}
    require(len(index["feasibility_runs"]) == len(set(index["feasibility_runs"])), "正式索引存在重复目录")

    output.mkdir(parents=True, exist_ok=False)
    write_json(output / "feasibility_matrix.json", matrix)
    write_json(output / "verified_sources.json", verified)
    write_json(output / "evidence_index.json", index)
    write_json(output / "summary.json", {
        "scope": "formal_evidence_index_preparation", "formal_run_admission": "pending_period_capacity_and_final_review",
        "registration_sha256": REGISTRATION_SHA256, "registration_snapshot_proven_configurations": 33,
        "configuration_count": 72, "proven_configurations": 39, "unique_complete_sources": len(verified),
        "new_run_names": sorted(NAMES), "plan_sha256": PLAN_SHA256,
        "completed_sha256": content_hash(control / "completed.json"),
        "execution_sha256": content_hash(control / "execution.json"),
        "source_snapshot_sha256": content_hash(control / "source/snapshot.json"),
        "source_archive_sha256": snapshot["archive_sha256"],
        "parameter_rationale_sha256": content_hash(rationale), "script_sha256": content_hash(Path(__file__)),
        "resource_root": str(root), "outputs_sha256": {path.name: content_hash(path) for path in output.glob("*.json")},
    })
    print(json.dumps({"output": str(output), "summary_sha256": content_hash(output / "summary.json"),
                      "evidence_index_sha256": content_hash(output / "evidence_index.json")}), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
