from __future__ import annotations

import json
import math
from pathlib import Path
import time

import numpy as np

from .static_experiment_config import content_hash
from .static_experiment_recording import iter_records


CYCLE_TIMING_SCHEMA_VERSION = 1
CYCLE_TIMING_BOUNDARY = "controller_after_synchronous_records_and_recovery_v1"
LEGACY_CYCLE_TIMING_BOUNDARY = "before_cycle_record_and_admission_recovery"


class CycleTimings:
    def __init__(self, *, maximum_records: int):
        if type(maximum_records) is not int or maximum_records <= 0:
            raise ValueError("完整周期记录上限必须为正整数")
        self.maximum_records = maximum_records
        self._cycles = {}

    def record(self, sequence: int, started: float) -> None:
        if type(sequence) is not int or not 0 < sequence < 2 ** 63 or sequence in self._cycles:
            raise ValueError("完整周期 sequence 必须为唯一 SQLite 正整数")
        if not math.isfinite(started) or len(self._cycles) >= self.maximum_records:
            raise ValueError("完整周期起点无效或记录数量超过上限")
        self._cycles[sequence] = {"sequence": sequence, "started_monotonic_s": started}

    def complete(self, sequence: int) -> None:
        row = self._cycles[sequence]
        if "completed_monotonic_s" in row:
            raise ValueError("完整周期不能重复完成")
        completed = time.monotonic()
        if completed < row["started_monotonic_s"]:
            raise ValueError("完整周期终点早于起点")
        row.update(completed_monotonic_s=completed,
                   duration_s=completed - row["started_monotonic_s"])

    def complete_pending(self) -> None:
        for sequence, row in self._cycles.items():
            if "completed_monotonic_s" not in row:
                self.complete(sequence)

    def snapshot(self) -> dict:
        if any("completed_monotonic_s" not in row for row in self._cycles.values()):
            raise ValueError("完整周期仍有未完成记录")
        return {
            "schema_version": CYCLE_TIMING_SCHEMA_VERSION, "boundary": CYCLE_TIMING_BOUNDARY,
            "maximum_records": self.maximum_records,
            "cycles": [dict(row) for row in self._cycles.values()],
        }


def load_cycle_timing(directory: Path, manifest: dict | None = None) -> dict:
    directory = Path(directory)
    if manifest is None:
        manifest = json.loads((directory / "manifest.json").read_text())
    database = directory / "controller.sqlite3"
    cycles = list(iter_records(database, "cycle")) if database.exists() else []
    declaration = manifest.get("cycle_timing_boundary")
    path = directory / "controller.recording.json"
    statistics = json.loads(path.read_text()) if path.exists() else {}
    timing = statistics.get("cycle_timing")
    if declaration is None and timing is None:
        return {
            "schema_version": 0, "boundary": LEGACY_CYCLE_TIMING_BOUNDARY,
            "eligible_for_period": False, "sidecar_sha256": None,
            "cycles": [{**row["values"], "sequence": row["sequence"]} for row in cycles],
        }
    if declaration != CYCLE_TIMING_BOUNDARY or not isinstance(timing, dict):
        raise ValueError("完整周期边界声明或 sidecar 缺失")
    result = json.loads((directory / "result.json").read_text())
    identity = content_hash(path)
    if result.get("recording_statistics_sha256", {}).get("controller") != identity:
        raise ValueError("完整周期 sidecar SHA-256 不一致")
    if (set(timing) != {"schema_version", "boundary", "maximum_records", "cycles"}
            or type(timing["schema_version"]) is not int
            or timing["schema_version"] != CYCLE_TIMING_SCHEMA_VERSION
            or timing["boundary"] != CYCLE_TIMING_BOUNDARY
            or type(timing["maximum_records"]) is not int or timing["maximum_records"] <= 0
            or not isinstance(timing["cycles"], list)
            or len(timing["cycles"]) != len(cycles)
            or len(cycles) > timing["maximum_records"]):
        raise ValueError("完整周期版本、边界或记录数量无效")
    if ("recording_limits" in manifest
            and timing["maximum_records"] != manifest["recording_limits"]["maximum_candidate_records"]):
        raise ValueError("完整周期数量上限与运行生命周期不一致")
    measured = {}
    for row in timing["cycles"]:
        if not isinstance(row, dict) or set(row) != {
                "sequence", "started_monotonic_s", "completed_monotonic_s", "duration_s"}:
            raise ValueError("完整周期字段不完整")
        sequence = row["sequence"]
        if type(sequence) is not int or not 0 < sequence < 2 ** 63 or sequence in measured:
            raise ValueError("完整周期 sequence 无效或重复")
        if any(type(row[name]) not in (float, int) or not math.isfinite(row[name])
               for name in ("started_monotonic_s", "completed_monotonic_s", "duration_s")):
            raise ValueError("完整周期时间必须有限")
        if (row["duration_s"] < 0
                or row["completed_monotonic_s"] - row["started_monotonic_s"] != row["duration_s"]):
            raise ValueError("完整周期起点、终点和 duration 不一致")
        measured[sequence] = row
    if set(measured) != {row["sequence"] for row in cycles}:
        raise ValueError("完整周期必须与 SQLite cycle 一一对应")
    output = []
    for base in cycles:
        row = measured[base["sequence"]]
        duration = base["values"]["duration_s"]
        if (not math.isfinite(duration) or duration < 0
                or row["completed_monotonic_s"] < base["monotonic_s"]
                or row["duration_s"] < duration
                or base["monotonic_s"] - row["started_monotonic_s"] != duration):
            raise ValueError("完整周期与基础计时不一致")
        output.append({**base["values"], **row, "base_duration_s": duration})
    return {
        "schema_version": CYCLE_TIMING_SCHEMA_VERSION, "boundary": CYCLE_TIMING_BOUNDARY,
        "eligible_for_period": True, "sidecar_sha256": identity, "cycles": output,
    }


def cycle_timing_summary(measured: dict, period_s: float) -> dict:
    durations = np.array([row["duration_s"] for row in measured["cycles"]])
    return {
        "schema_version": measured["schema_version"], "boundary": measured["boundary"],
        "eligible_for_period": measured["eligible_for_period"],
        "sidecar_sha256": measured["sidecar_sha256"],
        "status": ("insufficient_evidence" if not len(durations) or not measured["eligible_for_period"] else
                   "pass" if np.quantile(durations, .999) <= .8 * period_s else "fail"),
        "samples": len(durations), "period_s": period_s,
        "p999_s": float(np.quantile(durations, .999)) if len(durations) else None,
        "maximum_s": float(durations.max()) if len(durations) else None,
        "deadline_misses": int(np.count_nonzero(durations > period_s)),
        "rejected_cycles": sum(not row["accepted"] for row in measured["cycles"]),
    }
