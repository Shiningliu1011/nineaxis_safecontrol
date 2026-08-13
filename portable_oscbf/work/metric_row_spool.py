"""Disk-backed per-cycle metrics for long headless simulations.

The control loop must not retain one large Python dictionary per cycle.  This
module spools rows to JSONL with a bounded write buffer, then reconstructs the
legacy CSV and exact percentile summary after the run has stopped.
"""

from __future__ import annotations

import csv
import json
import os
from pathlib import Path
from typing import Any, Callable, Iterable, Iterator, Mapping, Sequence

import numpy as np


def _json_default(value: Any) -> Any:
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"metric value is not JSON serializable: {type(value)!r}")


def ordered_fieldnames(keys: Iterable[str]) -> list[str]:
    """Keep the established step/t-first CSV layout."""
    ordered = sorted(set(keys))
    preferred = [key for key in ("step", "t") if key in ordered]
    return preferred + [key for key in ordered if key not in preferred]


def summarize_numeric_columns(
        keys: Sequence[str], values_for: Callable[[str], np.ndarray]) -> dict[str, float]:
    """Match the legacy ``PerfSummary`` statistics from one numeric column at a time."""
    stats: dict[str, float] = {}

    for key in keys:
        values = np.asarray(values_for(key), dtype=float)
        if values.size == 0:
            continue
        stats[f"{key}_p50"] = float(np.percentile(values, 50, method="nearest"))
        stats[f"{key}_p95"] = float(np.percentile(values, 95, method="nearest"))
        stats[f"{key}_p99"] = float(np.percentile(values, 99, method="nearest"))
        stats[f"{key}_max"] = float(np.max(values))

    if "period_overrun" in keys:
        values = np.asarray(values_for("period_overrun"), dtype=float)
        if values.size:
            count = float(np.count_nonzero(values > 0.5))
            stats["period_overrun_count"] = count
            stats["period_overrun_ratio"] = count / float(len(values))

    if "ee_err_mm" in keys:
        values = np.asarray(values_for("ee_err_mm"), dtype=float)
        if values.size:
            stats["max_ee_err_mm"] = float(np.max(values))
            stats["p95_ee_err_mm"] = float(np.percentile(values, 95, method="nearest"))
            stats["final_ee_err_mm"] = float(values[-1])
    if "orient_err_deg" in keys:
        values = np.asarray(values_for("orient_err_deg"), dtype=float)
        if values.size:
            stats["max_oe_deg"] = float(np.max(values))
            stats["p95_oe_deg"] = float(np.percentile(values, 95, method="nearest"))
            stats["final_oe_deg"] = float(values[-1])
    if "dyn_min" in keys:
        values = np.asarray(values_for("dyn_min"), dtype=float)
        if values.size:
            stats["min_dyn_min_mm"] = float(np.min(values)) * 1000.0
    if "h_min" in keys:
        values = np.asarray(values_for("h_min"), dtype=float)
        if values.size:
            stats["min_h_min_mm"] = float(np.min(values)) * 1000.0
    if "qp_ok" in keys:
        values = np.asarray(values_for("qp_ok"), dtype=float)
        if values.size:
            stats["qp_fail_count"] = float(np.count_nonzero(values < 0.5))

    return stats


def summarize_metric_rows(rows: Sequence[Mapping[str, float]]) -> dict[str, float]:
    """Summarize in-memory rows while preserving the old public behavior."""
    if not rows:
        return {}
    keys = sorted({key for row in rows for key in row})

    def values_for(key: str) -> np.ndarray:
        return np.asarray([float(row[key]) for row in rows if key in row], dtype=float)

    return summarize_numeric_columns(keys, values_for)


class MetricRowSpool:
    """Bounded-memory JSONL spool with exact final CSV/statistic reconstruction."""

    def __init__(self, path: str | os.PathLike[str], *, flush_rows: int = 128) -> None:
        if int(flush_rows) < 1:
            raise ValueError("flush_rows must be >= 1")
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self.path.open("w", encoding="utf-8", buffering=1024 * 1024)
        self._flush_rows = int(flush_rows)
        self._buffer: list[str] = []
        self._row_count = 0
        self._finalized = False
        self._summary_cache: dict[str, float] | None = None

    @property
    def row_count(self) -> int:
        return self._row_count

    def append(self, row: Mapping[str, Any]) -> None:
        if self._finalized:
            raise RuntimeError("cannot append after the metric spool is finalized")
        self._buffer.append(json.dumps(
            dict(row), default=_json_default, ensure_ascii=True,
            separators=(",", ":"), allow_nan=True))
        self._row_count += 1
        if len(self._buffer) >= self._flush_rows:
            self.flush()

    def flush(self) -> None:
        if not self._buffer:
            return
        self._stream.write("\n".join(self._buffer))
        self._stream.write("\n")
        self._stream.flush()
        self._buffer.clear()

    def finalize(self) -> None:
        if self._finalized:
            return
        self.flush()
        self._stream.close()
        self._finalized = True

    def iter_rows(self) -> Iterator[dict[str, Any]]:
        self.finalize()
        with self.path.open(encoding="utf-8") as stream:
            for line in stream:
                if line.strip():
                    payload = json.loads(line)
                    if not isinstance(payload, dict):
                        raise ValueError(f"invalid metric spool row in {self.path}")
                    yield payload

    def _fieldnames(self) -> list[str]:
        fields: set[str] = set()
        for row in self.iter_rows():
            fields.update(row)
        return ordered_fieldnames(fields)

    def summarize(self) -> dict[str, float]:
        """Calculate exact legacy statistics using a temporary disk matrix."""
        if self._summary_cache is not None:
            return dict(self._summary_cache)
        fields = self._fieldnames()
        if not fields or self._row_count == 0:
            self._summary_cache = {}
            return {}

        field_indices = {key: index for index, key in enumerate(fields)}
        matrix_path = self.path.with_name(
            f".{self.path.name}.{os.getpid()}.summary.mmap")
        matrix = None
        try:
            matrix = np.memmap(
                matrix_path, dtype=np.float64, mode="w+",
                shape=(self._row_count, len(fields)))
            matrix.fill(np.nan)
            for row_index, row in enumerate(self.iter_rows()):
                for key, value in row.items():
                    try:
                        matrix[row_index, field_indices[key]] = float(value)
                    except (KeyError, TypeError, ValueError):
                        continue
            matrix.flush()

            def values_for(key: str) -> np.ndarray:
                values = matrix[:, field_indices[key]]
                return values[~np.isnan(values)]

            self._summary_cache = summarize_numeric_columns(fields, values_for)
            return dict(self._summary_cache)
        finally:
            if matrix is not None:
                del matrix
            matrix_path.unlink(missing_ok=True)

    def write_csv(self, path: str | os.PathLike[str], *, cleanup: bool = True) -> Path:
        """Write the normal union-schema CSV after a clean run."""
        fields = self._fieldnames()
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for row in self.iter_rows():
                writer.writerow(row)
        if cleanup:
            self.path.unlink(missing_ok=True)
        return output
