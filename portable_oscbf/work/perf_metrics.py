#!/usr/bin/env python3
"""Small performance-metric helpers for the Python OSCBF runner."""

from __future__ import annotations

import csv
import os
import time
from contextlib import contextmanager
from typing import Dict, Iterable, List

import numpy as np

try:
    from metric_row_spool import MetricRowSpool, summarize_metric_rows
except ModuleNotFoundError:  # pragma: no cover - package import compatibility
    from work.metric_row_spool import MetricRowSpool, summarize_metric_rows


def compute_phase_lag_metrics(ee_minus_des, desired_velocity, v_min: float = 1.0e-3) -> Dict[str, float]:
    """Decompose position error into tangent lag and normal error.

    ee_minus_des is p_ee - p_des. Positive phase lag means the robot is behind
    the desired motion along desired_velocity.
    """
    err = np.asarray(ee_minus_des, dtype=float).reshape(3)
    vel = np.asarray(desired_velocity, dtype=float).reshape(3)
    speed = float(np.linalg.norm(vel))
    err_des_minus_ee = -err

    if speed < float(v_min):
        return {
            "phase_lag_ms": 0.0,
            "phase_lag_valid": 0.0,
            "tangent_error_mm": 0.0,
            "normal_error_mm": float(np.linalg.norm(err_des_minus_ee) * 1000.0),
        }

    tangent_unit = vel / speed
    tangent_error_m = float(np.dot(err_des_minus_ee, tangent_unit))
    normal_vec = err_des_minus_ee - tangent_error_m * tangent_unit
    phase_lag_s = tangent_error_m / speed
    return {
        "phase_lag_ms": float(phase_lag_s * 1000.0),
        "phase_lag_valid": 1.0,
        "tangent_error_mm": float(tangent_error_m * 1000.0),
        "normal_error_mm": float(np.linalg.norm(normal_vec) * 1000.0),
    }


def compute_velocity_tracking_metrics(actual_velocity, desired_velocity,
                                      v_min: float = 1.0e-3) -> Dict[str, float]:
    """Compare actual EE velocity against the desired path tangent velocity."""
    actual = np.asarray(actual_velocity, dtype=float).reshape(3)
    desired = np.asarray(desired_velocity, dtype=float).reshape(3)
    desired_speed = float(np.linalg.norm(desired))

    if desired_speed < float(v_min):
        return {
            "desired_speed_mm_s": float(desired_speed * 1000.0),
            "actual_speed_mm_s": float(np.linalg.norm(actual) * 1000.0),
            "actual_tangent_speed_mm_s": 0.0,
            "actual_normal_speed_mm_s": float(np.linalg.norm(actual) * 1000.0),
            "tangent_speed_error_mm_s": 0.0,
            "speed_ratio": 0.0,
            "speed_tracking_valid": 0.0,
        }

    tangent_unit = desired / desired_speed
    actual_tangent_speed = float(np.dot(actual, tangent_unit))
    actual_normal_vec = actual - actual_tangent_speed * tangent_unit
    return {
        "desired_speed_mm_s": float(desired_speed * 1000.0),
        "actual_speed_mm_s": float(np.linalg.norm(actual) * 1000.0),
        "actual_tangent_speed_mm_s": float(actual_tangent_speed * 1000.0),
        "actual_normal_speed_mm_s": float(np.linalg.norm(actual_normal_vec) * 1000.0),
        "tangent_speed_error_mm_s": float((actual_tangent_speed - desired_speed) * 1000.0),
        "speed_ratio": float(actual_tangent_speed / desired_speed),
        "speed_tracking_valid": 1.0,
    }


def _clipped_load(metrics: Dict[str, float], *keys: str) -> float:
    for key in keys:
        if key in metrics:
            return float(np.clip(float(metrics[key]), 0.0, 1.0))
    return 0.0


def limit_phase_preview_by_mat_schedule(phase_preview_s: float, dt: float,
                                        schedule_metrics: Dict[str, float] | None = None) -> Dict[str, float]:
    """Clip phase preview, reducing forward look-ahead near hard .mat segments.

    With no schedule loads this preserves the original fixed limit:
    backward <= 1dt and forward <= 2dt.  High boundary, tangent-jerk, and
    deceleration loads shrink the forward limit so timing catch-up is less
    aggressive where the planned curve is already dynamically difficult.
    """
    dt = max(float(dt), 1.0e-9)
    metrics = schedule_metrics or {}
    boundary_load = _clipped_load(metrics, "schedule_boundary_load", "boundary_load")
    tangent_jerk_load = _clipped_load(metrics, "schedule_tangent_jerk_load", "tangent_jerk_load")
    decel_load = _clipped_load(metrics, "schedule_decel_load", "decel_load")

    forward_scale = 1.0 - 0.45 * boundary_load - 0.30 * tangent_jerk_load - 0.20 * decel_load
    backward_scale = 1.0 - 0.25 * boundary_load - 0.10 * tangent_jerk_load
    forward_limit = max(0.5 * dt, 2.0 * dt * max(forward_scale, 0.0))
    backward_limit = max(0.5 * dt, dt * max(backward_scale, 0.0))
    clipped = float(np.clip(float(phase_preview_s), -backward_limit, forward_limit))

    return {
        "phase_preview_s": clipped,
        "phase_preview_raw_s": float(phase_preview_s),
        "phase_preview_forward_limit_s": float(forward_limit),
        "phase_preview_backward_limit_s": float(backward_limit),
    }


def should_finish_time_tracking(time_source: str, step_count: int, total_steps: int,
                                wall_elapsed_s: float, tracking_time_limit_s: float) -> bool:
    """Return whether the current tracking run should stop recording steps."""
    if int(step_count) >= int(total_steps):
        return True
    if str(time_source) == "wall_clock":
        return float(wall_elapsed_s) >= float(tracking_time_limit_s)
    return False


class StepTimer:
    """Accumulate named section durations for one control step."""

    def __init__(self) -> None:
        self._seconds: Dict[str, float] = {}

    @contextmanager
    def section(self, name: str):
        start = time.perf_counter()
        try:
            yield
        finally:
            self.record(name, time.perf_counter() - start)

    def record(self, name: str, seconds: float) -> None:
        if seconds < 0.0:
            raise ValueError("seconds must be non-negative")
        self._seconds[name] = self._seconds.get(name, 0.0) + float(seconds)

    def snapshot(self) -> Dict[str, float]:
        return {f"{name}_ms": seconds * 1000.0 for name, seconds in self._seconds.items()}

    def reset(self) -> None:
        self._seconds.clear()


class PerfSummary:
    """Collect per-step metrics and summarize them with robust percentiles."""

    def __init__(self, *, stream_path: str | os.PathLike | None = None,
                 stream_flush_rows: int = 128) -> None:
        self._rows: List[Dict[str, float]] = []
        self._spool = (
            MetricRowSpool(stream_path, flush_rows=stream_flush_rows)
            if stream_path else None)
        self._pending_stream_row: Dict[str, float] | None = None

    @property
    def row_count(self) -> int:
        if self._spool is None:
            return len(self._rows)
        return self._spool.row_count + int(self._pending_stream_row is not None)

    def _finalize_stream(self) -> None:
        if self._spool is None:
            return
        if self._pending_stream_row is not None:
            self._spool.append(self._pending_stream_row)
            self._pending_stream_row = None
        self._spool.finalize()

    def record_step(self, fields: Dict[str, float]) -> None:
        row = dict(fields)
        if self._spool is None:
            self._rows.append(row)
            return
        if self._pending_stream_row is not None:
            self._spool.append(self._pending_stream_row)
        self._pending_stream_row = row

    def update_last(self, fields: Dict[str, float]) -> None:
        """Annotate the final recorded step with an end-of-run state."""
        if self._spool is not None:
            if self._pending_stream_row is not None:
                self._pending_stream_row.update(dict(fields))
            return
        if self._rows:
            self._rows[-1].update(dict(fields))

    def summarize(self) -> Dict[str, float]:
        if self._spool is not None:
            self._finalize_stream()
            return self._spool.summarize()
        return summarize_metric_rows(self._rows)

    def write_csv(self, path: str) -> None:
        if self._spool is not None:
            self._finalize_stream()
            self._spool.write_csv(path)
            return
        parent = os.path.dirname(os.path.abspath(path))
        if parent:
            os.makedirs(parent, exist_ok=True)

        keys = sorted({key for row in self._rows for key in row})
        preferred = [key for key in ("step", "t") if key in keys]
        fieldnames = preferred + [key for key in keys if key not in preferred]
        with open(path, "w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=fieldnames)
            writer.writeheader()
            for row in self._rows:
                writer.writerow(row)

    @property
    def rows(self) -> Iterable[Dict[str, float]]:
        if self._spool is not None:
            self._finalize_stream()
            return self._spool.iter_rows()
        return tuple(self._rows)


# Strict validation thresholds for main-path acceptance.
STRICT_MAX_EE_ERR_MM = 0.5
STRICT_MAX_OE_DEG = 0.5


def validate_strict(stats: Dict[str, float],
                    *, require_dynamic_clearance: bool = True) -> Dict[str, object]:
    """Check strict max-error acceptance criteria.

    Returns a dict with 'pass' (bool), 'failures' (list of strings),
    and the key metric values for reporting.

    When *require_dynamic_clearance* is False the ``min_dyn_min_mm <= 0``
    check is skipped.  Use this for smoke runs that do not exercise dynamic
    obstacles (e.g. ``--no-obstacles``).
    """
    failures = []

    max_ee = stats.get("max_ee_err_mm", float("inf"))
    max_oe = stats.get("max_oe_deg", float("inf"))
    qp_fail = stats.get("qp_fail_count", float("inf"))
    min_dyn = stats.get("min_dyn_min_mm", 0.0)

    if max_ee > STRICT_MAX_EE_ERR_MM:
        failures.append(f"max_ee_err_mm={max_ee:.4f} > {STRICT_MAX_EE_ERR_MM}")
    if max_oe > STRICT_MAX_OE_DEG:
        failures.append(f"max_oe_deg={max_oe:.4f} > {STRICT_MAX_OE_DEG}")
    if np.isfinite(qp_fail) and qp_fail > 0:
        failures.append(f"qp_fail_count={int(qp_fail)} > 0")
    if require_dynamic_clearance and min_dyn <= 0:
        failures.append(f"min_dyn_min_mm={min_dyn:.4f} <= 0")

    return {
        "pass": len(failures) == 0,
        "failures": failures,
        "max_ee_err_mm": max_ee,
        "max_oe_deg": max_oe,
        "qp_fail_count": int(qp_fail) if np.isfinite(qp_fail) else 0,
        "min_dyn_min_mm": min_dyn,
    }
