from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from robot_safecontrol_moveit.static_experiment_capacity import (
    CapacityBudget, RecordingLimits, assess_capacity,
)
from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig, content_hash
from robot_safecontrol_moveit.static_experiment_engineering_capacity import (
    build_engineering_evidence, require_engineering_evidence, run_capacity_group,
)
from robot_safecontrol_moveit.static_experiment_provenance import source_identity
from robot_safecontrol_moveit.static_experiment_recording import write_json
from robot_safecontrol_moveit.static_experiment_volumes import (
    allocate_volume_budgets, filesystem_identity, require_volume_capacity,
)
from robot_safecontrol_moveit.static_formal_experiments import experiment_rows, load_formal


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--audit", action="append", type=Path, required=True)
    parser.add_argument("--lifecycle", action="append", type=Path, required=True)
    parser.add_argument("--filesystem", action="append", type=Path, required=True)
    parser.add_argument("--runtime-archive", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--resource-root", type=Path, required=True)
    parser.add_argument("--volume", action="append", type=Path, required=True)
    parser.add_argument("--formal", type=Path)
    parser.add_argument("--retry-runs", type=int, default=0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.retry_runs < 0:
        raise ValueError("额外重试预算必须非负")
    config = ExperimentConfig.load(args.config, resource_root=args.resource_root, method="D")
    sources = source_identity(args.resource_root)
    manifest = load_formal(args.formal) if args.formal else None
    if manifest is not None and manifest["source_sha256"] != sources:
        raise ValueError("正式清单与当前容量实现来源不一致")
    volumes = [filesystem_identity(path) for path in args.volume]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)

    def reports(paths):
        return {str(path.resolve(strict=True)): json.loads(path.read_text()) for path in paths}

    evidence = build_engineering_evidence(
        audits=reports(args.audit), lifecycles=reports(args.lifecycle), filesystems=reports(args.filesystem),
        sources=sources, archive=args.runtime_archive.resolve(strict=True),
        limits=RecordingLimits(
            config.values["timeout_s"], config.values["settle_s"], config.values["control"]["period_s"],
        ),
        common_control=config.manifest()["config"]["control"], output_volumes=volumes,
    )
    evidence_path = output / "engineering_evidence.json"
    write_json(evidence_path, evidence)
    rows = experiment_rows() if manifest is None else manifest["runs"]
    allowances = {
        row["id"]: evidence["models"][run_capacity_group(row, evidence)]["retained_run_bytes"]
        for row in rows
    }
    template = CapacityBudget(
        planned_runs=len(rows), retained_run_bytes=max(allowances.values()),
        retry_runs=args.retry_runs, **evidence["per_volume_budget"],
    )
    total = replace(
        template, planned_retained_bytes=sum(allowances.values()),
        **{name: sum(row[name] for row in evidence["volume_workspaces"].values())
           for name in evidence["per_volume_budget"]},
    )
    validated = evidence["status"] == "pass"
    plan = assess_capacity(
        total, available_bytes=sum(row["available_bytes"] for row in volumes),
        engineering_validated=validated,
    )
    plan.update(
        source_sha256=sources, engineering_evidence=str(evidence_path),
        evidence_sha256={
            **evidence["evidence_sha256"], str(evidence_path): content_hash(evidence_path),
        },
        formal_manifest_sha256=None if manifest is None else content_hash(args.formal / "formal_manifest.json"),
        generation={
            "command": sys.argv, "cpu_affinity": sorted(os.sched_getaffinity(0)),
            "script_sha256": content_hash(Path(__file__)), "formal_bound": manifest is not None,
        },
    )
    write_json(output / "capacity_plan.json", plan)
    if plan["shortfall_bytes"] == 0:
        allocation = allocate_volume_budgets(
            [row["id"] for row in rows], template, volumes, run_retained_bytes=allowances,
            volume_workspaces=evidence["volume_workspaces"],
        )
        for row in allocation:
            row.update(assess_capacity(
                CapacityBudget(**row["budget"]), available_bytes=row["available_bytes"],
                engineering_validated=validated,
            ))
        plan["volumes"] = allocation
        if any(sum(row["budget"][key] for row in allocation) != value for key, value in asdict(total).items()
               if key != "retained_run_bytes"):
            raise ValueError("分卷预算总和与完整矩阵预算不一致")
        write_json(output / "capacity_plan.json", plan)
        if manifest is not None and plan["status"] == "pass":
            require_engineering_evidence(plan, manifest, args.formal)
            require_volume_capacity(args.formal, manifest, plan)
    print(json.dumps({
        "status": plan["status"], "formal_bound": manifest is not None,
        "checks": evidence["checks"], "budget": plan["budget"],
        "required_free_bytes": plan["required_free_bytes"],
        "available_bytes": plan["available_bytes"], "shortfall_bytes": plan["shortfall_bytes"],
    }, ensure_ascii=False))
    return 0 if plan["status"] == "pass" else 2


if __name__ == "__main__":
    raise SystemExit(main())
