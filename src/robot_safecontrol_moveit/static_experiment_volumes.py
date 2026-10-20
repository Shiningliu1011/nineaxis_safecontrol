from __future__ import annotations

from dataclasses import replace
import json
import os
from pathlib import Path
import shutil
import subprocess

from .static_experiment_capacity import CapacityBudget, assess_capacity, require_capacity


def filesystem_identity(directory: Path) -> dict:
    directory = directory.resolve(strict=True)
    filesystem = json.loads(subprocess.run(
        ["findmnt", "--json", "--target", str(directory), "--output", "SOURCE,TARGET,FSTYPE,OPTIONS,UUID"],
        check=True, text=True, capture_output=True,
    ).stdout)["filesystems"][0]
    if not filesystem["uuid"]:
        raise ValueError(f"输出文件系统缺少稳定 UUID：{directory}")
    if "rw" not in filesystem["options"].split(","):
        raise ValueError(f"输出文件系统未以读写模式挂载：{directory}")
    return {
        "root": str(directory), "filesystem_uuid": filesystem["uuid"],
        "filesystem_type": filesystem["fstype"], "mount_source": filesystem["source"],
        "device": directory.stat().st_dev, "available_bytes": shutil.disk_usage(directory).free,
        "allocation_unit_bytes": os.statvfs(directory).f_frsize,
    }


def allocate_volume_budgets(run_ids: list[str], budget: CapacityBudget, volumes: list[dict], *,
                            run_retained_bytes: dict[str, int] | None = None,
                            volume_workspaces: dict[str, dict] | None = None) -> list[dict]:
    if len(run_ids) != budget.planned_runs or len(set(run_ids)) != len(run_ids):
        raise ValueError("卷分配必须覆盖全部不同运行")
    if any(Path(name).name != name or name in (".", "..", "") for name in run_ids):
        raise ValueError("运行名称必须为单层目录名称")
    identities = [volume["filesystem_uuid"] for volume in volumes]
    if not all(identities) or len(set(identities)) != len(identities):
        raise ValueError("不同卷必须对应不同文件系统")
    if volume_workspaces is not None and (
        set(volume_workspaces) != set(identities) or any(set(row) != {
            "transient_run_bytes", "derived_workspace_bytes", "provenance_bytes", "reserve_bytes",
        } for row in volume_workspaces.values())
    ):
        raise ValueError("工作空间配置必须逐卷包含完整空间类别")
    allowances = run_retained_bytes or dict.fromkeys(run_ids, budget.retained_run_bytes)
    if (set(allowances) != set(run_ids)
            or any(type(value) is not int or value <= 0 or value > budget.retained_run_bytes
                   for value in allowances.values())):
        raise ValueError("逐次记录额度必须覆盖全部运行且不超过最大额度")
    remainder, result = list(run_ids), []
    for index, volume in enumerate(volumes):
        retries = budget.retry_runs if index == 0 else 0
        current = replace(budget, **(
            {} if volume_workspaces is None else volume_workspaces[volume["filesystem_uuid"]]
        ))
        working = (current.transient_run_bytes + current.derived_workspace_bytes
                   + current.provenance_bytes + current.reserve_bytes
                   + retries * budget.retained_run_bytes)
        assigned, total = [], 0
        while remainder and total + allowances[remainder[0]] + working <= volume["available_bytes"]:
            name, remainder = remainder[0], remainder[1:]
            assigned.append(name)
            total += allowances[name]
        current = replace(
            current, planned_runs=len(assigned), retry_runs=retries, planned_retained_bytes=total,
        )
        result.append({
            **volume, "run_ids": assigned,
            "run_retained_bytes": {name: allowances[name] for name in assigned},
            **assess_capacity(current, available_bytes=volume["available_bytes"]),
        })
    if remainder:
        raise ValueError(f"全部输出卷容量不足：尚有 {len(remainder)} 次运行没有空间")
    return result


def capacity_run_directory(directory: Path, report: dict, run_id: str) -> Path:
    if "volumes" not in report:
        return directory / "runs" / run_id
    destinations = [
        Path(volume["root"]) / run_id for volume in report["volumes"] if run_id in volume["run_ids"]
    ]
    if len(destinations) != 1:
        raise ValueError(f"运行必须具有唯一输出卷：{run_id}")
    return destinations[0]


def require_volume_capacity(directory: Path, manifest: dict, report: dict) -> dict:
    ids = [row["id"] for row in manifest["runs"]]
    assigned = [run_id for volume in report["volumes"] for run_id in volume["run_ids"]]
    if sorted(ids) != sorted(assigned) or len(set(assigned)) != len(assigned):
        raise ValueError("容量卷分配与正式运行清单不一致")
    identities, current = set(), []
    for volume in report["volumes"]:
        root = Path(volume["root"]).resolve(strict=True)
        actual = filesystem_identity(root)
        if (actual["filesystem_uuid"] != volume["filesystem_uuid"]
                or actual["filesystem_type"] != volume["filesystem_type"]
                or actual["allocation_unit_bytes"] != volume["allocation_unit_bytes"]
                or actual["filesystem_uuid"] in identities):
            raise ValueError(f"输出文件系统身份变化或重复：{root}")
        identities.add(actual["filesystem_uuid"])
        budget = CapacityBudget(**volume["budget"])
        if budget.planned_runs != len(volume["run_ids"]):
            raise ValueError("卷预算与分配运行数量不一致")
        allowances = volume["run_retained_bytes"]
        if (set(allowances) != set(volume["run_ids"])
                or any(type(value) is not int or value <= 0 or value > budget.retained_run_bytes
                       for value in allowances.values())
                or sum(allowances.values()) != budget.planned_retained_bytes):
            raise ValueError("逐次记录额度与卷总量不一致")
        existing = []
        for run_id in volume["run_ids"]:
            run = root / run_id
            anchor = directory / "runs" / run_id
            if (anchor.exists() or anchor.is_symlink()) and anchor.resolve() != run.resolve():
                raise ValueError(f"运行索引与已验证输出位置不一致：{run_id}")
            if not run.exists():
                continue
            if not anchor.exists():
                raise ValueError(f"已有运行缺少正式目录索引：{run_id}")
            existing.append(run)
            result_path = run / "result.json"
            if result_path.is_file() and json.loads(result_path.read_text())["termination"] == "budget_exhausted":
                raise ValueError(f"已有运行耗尽记录预算，正式批次停止：{run}")
            allocated = sum(path.stat().st_blocks * 512 for path in run.rglob("*") if path.is_file())
            if allocated > allowances[run_id]:
                raise ValueError(f"已有运行超过工程容量额度，需要重新验证：{run}")
        assessment = assess_capacity(
            replace(
                budget, planned_runs=budget.planned_runs - len(existing),
                planned_retained_bytes=sum(allowances.values()) - sum(allowances[run.name] for run in existing),
            ),
            available_bytes=actual["available_bytes"],
            upper_bound_validated=report["is_validated_capacity_upper_bound"] is True,
            engineering_validated=report.get("is_validated_engineering_budget") is True,
        )
        require_capacity(assessment)
        current.append({**volume, **actual, **assessment})
    total_budget = CapacityBudget(**report["budget"])
    for field in ("planned_runs", "retry_runs", "transient_run_bytes", "derived_workspace_bytes",
                  "provenance_bytes", "reserve_bytes", "planned_retained_bytes"):
        if sum(volume["budget"][field] for volume in report["volumes"]) != getattr(total_budget, field):
            raise ValueError(f"总容量与分卷预算不一致：{field}")
    if any(volume["budget"]["retained_run_bytes"] != total_budget.retained_run_bytes
           for volume in report["volumes"]):
        raise ValueError("分卷记录额度必须使用同一已验证上限")
    return {**report, "volumes": current, "status": "pass"}
