import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys


REGISTRATION_SHA = "0f27070df3e6bd707426db43b85ff61d442e93f89aa55af3fdf12a319e7f3c9e"
INDEX_SHA = "69e724c3cc392dc4980739c586bef85c2b6029b0822de9dcb3e09de2cdc38f16"
MATRIX_SHA = "75494befed0744a911c8e5c55ad6e5f1af94c368ba731658110dadbcee42cf49"
SERVICE = "static-complete-cycle-period-study-02.service"
SCENES = ("s0", "s1", "s2", "s3-travel-100", "s3-travel-70", "s3-travel-40", "s4", "s5")


def helper_module():
    specification = importlib.util.spec_from_file_location(
        "complete_cycle_period_study", Path(__file__).with_name("complete-cycle-period-study.py"),
    )
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


HELPER = helper_module()
require, load, digest = HELPER.require, HELPER.load, HELPER.digest


def period_status(directory):
    from robot_safecontrol_moveit.static_cycle_timing import CYCLE_TIMING_BOUNDARY
    from robot_safecontrol_moveit.static_period_measurement import select_period

    directory = Path(directory)
    study = load(directory / "study.json")
    require(study.get("cycle_timing_boundary") == CYCLE_TIMING_BOUNDARY, "共同周期计时边界不符")
    path = directory / "period_report.json"
    if not path.exists():
        return {"status": "pending_study", "measured": 0}
    report = load(path)
    require(report.get("cycle_timing_boundary") == CYCLE_TIMING_BOUNDARY
            and report["study_sha256"] == digest(directory / "study.json"), "共同周期计时边界不符或study身份变化")
    rows = [row for group in report["groups"] for row in group["methods"]]
    selected = select_period(rows)
    require(all(report.get(key) == value for key, value in selected.items()), "共同周期报告与选择计算不同")
    if len(rows) < 20:
        return {"status": "pending_study", "measured": len(rows)}
    require(report["status"] == "pass", "完整20项共同周期未通过")
    require(report["selected_period_s"] == .1, "共同周期与100ms生命周期不同，需要重新验证")
    return {"status": "ready", "measured": len(rows), "selected_period_s": .1,
            "period_report_sha256": digest(path)}


def prepare(root, revision):
    revision = HELPER.bind_revision(root, revision)
    sys.path.insert(0, str(root / "src"))
    from robot_safecontrol_moveit.static_experiment_provenance import source_identity, environment_identity
    from robot_safecontrol_moveit.static_cycle_timing import CYCLE_TIMING_BOUNDARY

    base = HELPER.study_plan(root, revision)
    capacity = root / "output/static-capacity-20261020"
    scene = root / "output/static-scene-validation-20261020"
    study, budgets = Path(base["study"]), Path(base["budgets"])
    prepared = load(budgets / "prepared.json")
    sources = source_identity(root)
    require(len(sources) == 125 and sources == prepared["plan"]["source_sha256"], "固定的125项运行来源变化")
    engineering, capacity_plan = load(base["engineering"]), load(base["capacity_plan"])
    require(engineering["status"] == capacity_plan["status"] == "pass"
            and all(engineering["checks"].values()) and engineering["checks"]
            and engineering["source_sha256"] == capacity_plan["source_sha256"] == sources
            and not capacity_plan["generation"]["formal_bound"], "前置工程容量尚未通过")
    registration = root / ".trellis/tasks/10-20-static-scene-witnesses/research/formal-scene-input.json"
    proof = scene / "formal-feasibility-01"
    for path, expected in ((registration, REGISTRATION_SHA), (proof / "evidence_index.json", INDEX_SHA),
                           (proof / "feasibility_matrix.json", MATRIX_SHA)):
        require(digest(path) == expected, f"正式登记或证明身份变化：{path}")
    study_identity = load(study / "study.json")
    require(study_identity["source_sha256"] == sources
            and study_identity["cycle_timing_boundary"] == CYCLE_TIMING_BOUNDARY
            and study_identity["environment"]["cpu_affinity"] == list(range(16)), "study02来源或环境身份不同")
    representatives = [f"{scene_id}-initial-1-feed-1-D-repeat-1" for scene_id in SCENES]
    representatives.extend(f"s4-initial-1-feed-1-{method}-repeat-1" for method in "ABCE")
    paths = [registration, proof / "evidence_index.json", proof / "feasibility_matrix.json",
             Path(base["engineering"]), Path(base["capacity_plan"]), study / "study.json",
             budgets / "plan.json", budgets / "prepared.json", budgets / "index.json",
             Path(base["preflight"]) / "capacity-inputs.json", Path(base["preflight"]) / "completed.json",
             root / ".scratch/host-recovery/env.sh", Path(__file__).resolve(),
             Path(__file__).with_name("complete-cycle-period-study.py").resolve(),
             root / "scripts/static_obstacle_experiment.py", root / "scripts/plan_static_engineering_capacity.py",
             Path(load(proof / "evidence_index.json")["parameter_rationale"])]
    paths.extend(Path(value["path"]) for value in load(registration).values()
                 if isinstance(value, dict) and "path" in value)
    plan = {
        "resource_root": str(root), "runtime_revision": revision, "source_sha256": sources,
        "monitor_environment": environment_identity(), "study_environment": study_identity["environment"],
        "study": str(study), "budgets": str(budgets), "preflight": base["preflight"],
        "engineering": base["engineering"], "capacity_plan": base["capacity_plan"],
        "control": str(capacity / "formal-serial-driver-01"),
        "formal": str(root / "output/static-experiments/formal-01"),
        "bound_capacity": str(capacity / "formal-capacity-01"),
        "figures": str(root / "picture/static-obstacle-experiments/formal-01"),
        "proof_index": str(proof / "evidence_index.json"), "registration": str(registration),
        "proof_matrix": str(proof / "feasibility_matrix.json"),
        "catalog": load(registration)["development_catalog"]["path"],
        "report_python": str(root / ".scratch/host-recovery/report-venv/bin/python"),
        "service": SERVICE, "worker_cpus": "0-15", "monitor_cpus": "22-27", "domain": 200,
        "planned_runs": 1080, "proven_configurations": 39, "representatives": representatives,
        "input_sha256": {str(path): digest(path) for path in paths}, "executed": False,
    }
    for key in ("control", "formal", "bound_capacity", "figures"):
        require(not Path(plan[key]).exists(), f"输出已存在，保留全部原件：{plan[key]}")
    plan["period_gate"] = period_status(study)
    plan["status"] = plan["period_gate"]["status"]
    return plan


def main():
    parser = argparse.ArgumentParser(description="#182到#183的本地串行接续准备")
    parser.add_argument("--resource-root", type=Path, required=True)
    parser.add_argument("--runtime-revision", required=True)
    args = parser.parse_args()
    plan = prepare(args.resource_root.resolve(strict=True), args.runtime_revision)
    print(json.dumps(plan, ensure_ascii=False, indent=2))
    return 2 if plan["status"] == "pending_study" else 0


if __name__ == "__main__":
    raise SystemExit(main())
