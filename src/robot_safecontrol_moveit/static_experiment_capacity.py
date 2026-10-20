from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import json
import math
from pathlib import Path
import shutil
import time

from .static_experiment_config import content_hash


WORKER_STARTUP_TIMEOUT_S = 180.
WORKER_STOP_TIMEOUT_S = 20.
WORKER_INTERRUPT_TIMEOUT_S = 5.
SUPERVISOR_POLL_S = .1


@dataclass(frozen=True)
class RecordingLimits:
    timeout_s: float
    settle_s: float
    period_s: float

    def __post_init__(self):
        if (not math.isfinite(self.timeout_s) or self.timeout_s <= 0
                or not math.isfinite(self.settle_s) or self.settle_s < 0
                or self.period_s not in (.02, .04, .05, .1)):
            raise ValueError("记录生命周期参数无效")

    def manifest(self) -> dict:
        maximum_s = self.timeout_s + self.settle_s + 2 * (
            WORKER_STOP_TIMEOUT_S + WORKER_INTERRUPT_TIMEOUT_S
        ) + 1.
        feedback = math.ceil(maximum_s / .002)
        ticks = round(self.period_s / .002)
        calculations = math.ceil(feedback / ticks) + 2
        return {
            **asdict(self), "startup_order": ["controller", "plant"],
            "worker_startup_timeout_s": WORKER_STARTUP_TIMEOUT_S,
            "worker_stop_timeout_s": WORKER_STOP_TIMEOUT_S,
            "worker_interrupt_timeout_s": WORKER_INTERRUPT_TIMEOUT_S,
            "supervisor_poll_s": SUPERVISOR_POLL_S, "polling_allowance_s": 1.,
            "maximum_recording_s": maximum_s, "maximum_feedback_records": feedback,
            "maximum_candidate_records": calculations,
            "maximum_trials_per_candidate": 9,
            "stream_record_limits": {
                "plant": {
                    "feedback": feedback, "recording_delay": feedback,
                    "received_command": calculations, "admission": calculations, "event": 1,
                },
                "controller": {
                    "candidate": calculations, "cycle": calculations,
                    "command": calculations, "admission": calculations, "rejection": calculations,
                    "transition": 3 * calculations + 1,
                    "qp_failure": 1, "sns_failure": 1, "event": 1,
                },
            },
        }


class RecordingBudgetExhausted(RuntimeError):
    pass


class RecordingBudget:
    def __init__(self, limits: RecordingLimits):
        self.limits = limits.manifest()
        self.started_monotonic_s = None
        self.started_tasks: dict[str, int] = {}
        self.last_started_monotonic_s: dict[str, float] = {}

    def start(self, monotonic_s: float) -> None:
        if not math.isfinite(monotonic_s):
            raise ValueError("记录开始时间必须有限")
        if self.started_monotonic_s is None:
            self.started_monotonic_s = monotonic_s

    def check(self) -> None:
        if (self.started_monotonic_s is not None
                and time.monotonic() >= self.started_monotonic_s + self.limits["maximum_recording_s"]):
            raise RecordingBudgetExhausted("budget_exhausted: deadline")

    def begin(self, task: str) -> None:
        maximum = {
            "feedback": self.limits["maximum_feedback_records"],
            "calculation": self.limits["maximum_candidate_records"],
            "received_command": self.limits["maximum_candidate_records"],
        }[task]
        now = time.monotonic()
        self.start(now)
        self.check()
        count = self.started_tasks.get(task, 0)
        if count >= maximum:
            raise RecordingBudgetExhausted(f"budget_exhausted: {task}")
        self.started_tasks[task] = count + 1
        self.last_started_monotonic_s[task] = now

    def snapshot(self) -> dict:
        return {
            "started_monotonic_s": self.started_monotonic_s,
            "deadline_monotonic_s": (
                None if self.started_monotonic_s is None else
                self.started_monotonic_s + self.limits["maximum_recording_s"]
            ),
            "started_tasks": dict(self.started_tasks),
            "last_started_monotonic_s": dict(self.last_started_monotonic_s),
        }


def lifecycle_outcome_valid(result: dict, *, expected_termination: str) -> bool:
    if expected_termination not in ("completed", "pause_timeout", "timeout"):
        raise ValueError("生命周期预期终止原因无效")
    return (
        result.get("termination") == expected_termination
        and result.get("worker_exit_codes") == [0, 0]
    )


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
        result_path = run / "result.json"
        if result_path.is_file() and json.loads(result_path.read_text())["termination"] == "budget_exhausted":
            raise ValueError(f"已有运行耗尽记录预算，正式批次停止：{run}")
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
