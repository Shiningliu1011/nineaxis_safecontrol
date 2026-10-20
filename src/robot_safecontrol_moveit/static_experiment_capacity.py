from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import json
from pathlib import Path
import shutil

from .static_experiment_config import content_hash


@dataclass(frozen=True)
class CapacityBudget:
    planned_runs: int
    retained_run_bytes: int
    retry_runs: int
    transient_run_bytes: int
    derived_workspace_bytes: int
    provenance_bytes: int
    reserve_bytes: int

    def __post_init__(self):
        for name, value in asdict(self).items():
            if type(value) is not int or value < 0:
                raise ValueError(f"{name} 必须为非负整数")
        if self.retained_run_bytes == 0:
            raise ValueError("单次记录容量必须为正整数")


def assess_capacity(budget: CapacityBudget, *, available_bytes: int,
                    upper_bound_validated: bool = False) -> dict:
    if type(available_bytes) is not int or available_bytes < 0:
        raise ValueError("available_bytes 必须为非负整数")
    retained = (budget.planned_runs + budget.retry_runs) * budget.retained_run_bytes
    required = (
        retained + budget.transient_run_bytes + budget.derived_workspace_bytes
        + budget.provenance_bytes + budget.reserve_bytes
    )
    shortfall = max(0, required - available_bytes)
    return {
        "schema_version": 1,
        "budget": asdict(budget),
        "retained_bytes": retained,
        "required_free_bytes": required,
        "available_bytes": available_bytes,
        "shortfall_bytes": shortfall,
        "is_validated_capacity_upper_bound": upper_bound_validated,
        "status": ("insufficient_evidence" if not upper_bound_validated
                   else "fail" if shortfall else "pass"),
    }


def require_capacity(report: dict) -> None:
    budget = CapacityBudget(**report["budget"])
    assessed = assess_capacity(
        budget, available_bytes=report["available_bytes"],
        upper_bound_validated=report["is_validated_capacity_upper_bound"] is True,
    )
    if any(report.get(key) != assessed[key] for key in (
            "required_free_bytes", "shortfall_bytes", "retained_bytes", "status")):
        raise ValueError("容量计划计算结果不一致")
    if assessed["status"] == "fail":
        raise ValueError(f"正式实验空间不足：缺少 {assessed['shortfall_bytes']} 字节")
    if assessed["status"] != "pass":
        raise ValueError("正式实验容量上界证据不足，样本外推不能用于准入")


def require_formal_capacity(directory: Path, manifest: dict) -> dict:
    path = directory / "capacity_plan.json"
    if not path.is_file():
        raise ValueError("正式实验缺少 capacity_plan.json 容量上界证据")
    report = json.loads(path.read_text())
    if (report.get("formal_manifest_sha256") != content_hash(directory / "formal_manifest.json")
            or report.get("source_sha256") != manifest["source_sha256"]
            or report["budget"]["planned_runs"] != len(manifest["runs"])):
        raise ValueError("容量计划与正式清单或代码身份不一致")
    evidence = report.get("evidence_sha256", {})
    if not evidence:
        raise ValueError("容量计划缺少原始验证证据")
    for filename, expected in evidence.items():
        if content_hash(Path(filename)) != expected:
            raise ValueError(f"容量证据身份变化：{filename}")
    require_capacity(report)
    budget = CapacityBudget(**report["budget"])
    existing = [directory / "runs" / row["id"] for row in manifest["runs"]
                if (directory / "runs" / row["id"]).exists()]
    for run in existing:
        allocated = sum(path.stat().st_blocks * 512 for path in run.rglob("*") if path.is_file())
        if allocated > budget.retained_run_bytes:
            raise ValueError(f"已有运行超过容量上界，需要重新验证：{run}")
    current = assess_capacity(
        replace(budget, planned_runs=budget.planned_runs - len(existing)),
        available_bytes=shutil.disk_usage(directory).free,
        upper_bound_validated=True,
    )
    require_capacity(current)
    return current
