"""Pure OFF-15 evaluation; no ROS, I/O, control gates or total score.

Declare scope/evidence before sampling, and finish explicitly on termination.
Statistics are per sample (RMS and linear p95), not time-weighted. Missing and
nonfinite values retain coverage counts and cannot pass a required criterion.
"""
from __future__ import annotations

from collections import Counter
from collections.abc import Mapping
from copy import deepcopy
from dataclasses import asdict, dataclass, field, replace
import json
import hashlib
import math
from typing import Any

import numpy as np

from .tracking_contract import (
    BOUNDARIES, EvidenceContext, EvaluationScope, Threshold, VERDICT_LABELS,
    baseline_thresholds, combine_verdicts,
)


def _number(value: Any) -> float | None:
    if value is None:
        return None
    try:
        if isinstance(value, (bool, np.bool_)):
            return math.nan
        return float(value)
    except (TypeError, ValueError, OverflowError):
        return math.nan


def _boolean(value: Any) -> bool | None:
    return bool(value) if isinstance(value, (bool, np.bool_)) else None


def _vector(value: Any, shape: tuple[int, ...]) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError("invalid geometry shape or nonfinite values")
    return array


def _unit_vector(value: np.ndarray) -> np.ndarray:
    scale = float(np.max(np.abs(value)))
    if scale == 0:
        raise ValueError("direction must be nonzero")
    scaled = value / scale
    return scaled / np.linalg.norm(scaled)


def tool_axis_angle(current_rotation: Any, reference_rotation: Any) -> float:
    """0..pi angle of tool X axes: roll is free, antiparallel axes give pi."""
    current = _vector(current_rotation, (3, 3))[:, 0]
    desired = _vector(reference_rotation, (3, 3))[:, 0]
    current, desired = _unit_vector(current), _unit_vector(desired)
    return float(np.arctan2(np.linalg.norm(np.cross(desired, current)),
                           np.dot(desired, current)))


@dataclass(frozen=True)
class TrackingStepData:
    """One boundary's measurements; missing values never default to success."""

    values: dict[str, float | None] = field(default_factory=dict)
    qp_ok: bool | None = None
    admission_ok: bool | None = None
    overlap: bool | None = None
    projected_progress_m: float | None = None
    projection_before_m: float | None = None
    reference_progress_m: float | None = None
    at_endpoint: bool | None = None
    limiting_reason_code: int | None = None
    constraint_metrics: dict[str, dict] = field(default_factory=dict)


def step_from_result(result: Any) -> TrackingStepData:
    """Read explicit measurements or derive geometry at one state/reference.

    Legacy err_6d orientation and scheduler cross-track are diagnostic only.
    min_obs_dist is a margin, not a physical obstacle clearance.
    """
    if isinstance(result, TrackingStepData):
        values = {name: _number(value) for name, value in result.values.items()}
        _validate_values(values)
        return replace(result, values=values, qp_ok=_boolean(result.qp_ok),
                       admission_ok=_boolean(result.admission_ok), overlap=_boolean(result.overlap),
                       projected_progress_m=_number(result.projected_progress_m),
                       projection_before_m=_number(result.projection_before_m),
                       reference_progress_m=_number(result.reference_progress_m))
    get = result.get if isinstance(result, Mapping) else lambda k, default=None: getattr(result, k, default)
    aliases = {
        "pos_error_m": "pos_error_m",
        "tool_axis_error_rad": "tool_axis_error_rad",
        "cross_track_m": "measured_cross_track_error_m",
        "online_cross_track_m": "cross_track_error_m",
        "feedrate_m_s": "feedrate_m_s",
        "actual_tangent_speed_m_s": "actual_tangent_speed_m_s",
        "obstacle_margin_m": "min_obs_dist",
        "obstacle_clearance_m": "obstacle_clearance_m",
        "delta_slack": "delta_slack",
        "qp_primal_residual": "qp_primal_residual",
        "step_latency_ms": "step_latency_ms",
        "source_time_s": "reference_source_time_s",
    }
    values = {name: _number(get(key)) for name, key in aliases.items()}
    values["legacy_orientation_error"] = None
    if get("err_6d") is not None:
        try:
            error = _vector(get("err_6d"), (6,))
            values["pos_error_m"] = float(np.linalg.norm(error[:3]))
            values["legacy_orientation_error"] = float(np.linalg.norm(error[3:]))
        except (TypeError, ValueError):
            values["pos_error_m"] = values["legacy_orientation_error"] = math.nan
    position, reference = get("ee_pos"), get("reference_position_m")
    if position is not None and reference is not None:
        try:
            difference = _vector(position, (3,)) - _vector(reference, (3,))
            values["pos_error_m"] = float(np.linalg.norm(difference))
            if _boolean(get("reference_at_endpoint")) is True:
                values["cross_track_m"] = values["pos_error_m"]
            elif get("reference_tangent") is not None:
                tangent = _vector(get("reference_tangent"), (3,))
                tangent = _unit_vector(tangent)
                values["cross_track_m"] = float(np.linalg.norm(
                    difference - tangent * np.dot(tangent, difference)))
        except (TypeError, ValueError):
            values["pos_error_m"] = values["cross_track_m"] = math.nan
    if get("ee_rot") is not None and get("reference_rotation") is not None:
        try:
            values["tool_axis_error_rad"] = tool_axis_angle(get("ee_rot"), get("reference_rotation"))
        except (TypeError, ValueError):
            values["tool_axis_error_rad"] = math.nan
    _validate_values(values)
    projected = _number(get("projected_progress_m"))
    reference_progress = _number(get("reference_progress_m", get("path_progress_m")))
    if get("path_state") is not None:
        try:
            state = _vector(get("path_state"), (5,))
            reference_progress, projected = float(state[0]), float(state[1])
        except (TypeError, ValueError):
            reference_progress = projected = math.nan
    code = _number(get("limiting_reason_code"))
    return TrackingStepData(
        values, _boolean(get("qp_ok")), _boolean(get("admission_ok")),
        _boolean(get("overlap")), projected, _number(get("projection_before_m")),
        reference_progress, _boolean(get("reference_at_endpoint")),
        int(code) if code is not None and math.isfinite(code) and code.is_integer() else None,
        get("constraint_metrics", {}) or {},
    )


def _validate_values(values: dict) -> None:
    for name in ("pos_error_m", "cross_track_m", "tool_axis_error_rad", "step_latency_ms"):
        if values.get(name) is not None and values[name] < 0:
            values[name] = math.nan
    if values.get("tool_axis_error_rad") is not None and values["tool_axis_error_rad"] > math.pi:
        values["tool_axis_error_rad"] = math.nan


@dataclass(frozen=True)
class MetricStatistics:
    count: int
    missing: int
    invalid: int
    mean: float | None = None
    rms: float | None = None
    p95: float | None = None
    minimum: float | None = None
    maximum: float | None = None

    @property
    def complete(self) -> bool:
        return self.count > 0 and self.missing == 0 and self.invalid == 0


def _statistics(values: list[float | None]) -> MetricStatistics:
    missing = sum(v is None for v in values)
    finite = [v for v in values if v is not None and math.isfinite(v)]
    invalid = len(values) - missing - len(finite)
    if not finite:
        return MetricStatistics(0, missing, invalid)
    data = np.asarray(finite)
    # Scale to avoid overflow from squaring otherwise finite inputs.
    scale = float(np.max(np.abs(data)))
    rms = scale * float(np.sqrt(np.mean((data / scale) ** 2))) if scale else 0.0
    mean = scale * float(np.mean(data / scale)) if scale else 0.0
    p95 = scale * float(np.percentile(data / scale, 95)) if scale else 0.0
    return MetricStatistics(len(finite), missing, invalid, mean, rms,
                            p95, float(np.min(data)), float(np.max(data)))


@dataclass(frozen=True)
class CriterionResult:
    threshold: Threshold
    value: float | None
    verdict: str
    reason: str


@dataclass(frozen=True)
class TrackingReport:
    total_steps: int
    sample_data_sha256: str
    evidence: EvidenceContext
    scope: EvaluationScope | None
    metrics: dict[str, MetricStatistics]
    criteria: tuple[CriterionResult, ...]
    completed: bool
    completion_fraction: float | None
    scope_completion_fraction: float | None
    full_path_verified: bool
    full_path_covered: bool
    initial_progress_m: float | None
    final_progress_m: float | None
    termination: str | None
    task_verdict: str
    online_verdict: str
    verdict: str
    qp_success_rate: float | None
    qp_fail_count: int
    qp_missing_count: int
    admission_counts: dict[str, int]
    overlap_counts: dict[str, int]
    events: dict[str, int]
    limiting_reason_counts: dict[int, int]
    constraint_metrics: dict[str, dict]
    wall_time_s: float | None
    trajectory_duration_s: float | None
    deadline_ms: float | None
    deadline_miss_rate: float | None
    issues: tuple[str, ...]

    def summary(self) -> str:
        return (
            f"task={self.task_verdict} online={self.online_verdict} "
            f"evidence={self.evidence.kind}/{self.evidence.boundary} "
            f"scope_done={_format(self.scope_completion_fraction)} "
            f"arc={_format(self.completion_fraction)} completed={self.completed} "
            f"full_path_verified={self.full_path_verified} qp={_format(self.qp_success_rate)}"
        )

    def to_dict(self) -> dict:
        return asdict(self)

    def json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False, indent=2, allow_nan=False)

    def markdown(self) -> str:
        lines = [
            "# 路径跟踪评价报告", "",
            f"- 任务质量：{VERDICT_LABELS[self.task_verdict]}；在线门控：{VERDICT_LABELS[self.online_verdict]}",
            f"- 全部所需判据：{VERDICT_LABELS[self.verdict]}（仅限下述证据范围，不授予实机准入）",
            f"- 证据：{self.evidence.kind}；测量边界：{self.evidence.boundary}",
            f"- 运行：{self.evidence.run_id or '未提供'}；工况：{self.evidence.scenario or '未提供'}",
            f"- 测量定义：{self.evidence.measurement or '未提供'}；时间基准：{self.evidence.time_basis or '未提供'}",
            f"- 模型 / 配置 / 路径 / 数据身份：{self.evidence.model_id or '未提供'} / {self.evidence.config_id or '未提供'} / {self.evidence.trajectory_id or '未提供'} / {self.evidence.data_id or '未提供'}",
            f"- 总样本：{self.total_steps}；终止原因：{self.termination or '未终止/未声明'}",
            f"- 样本记录 SHA-256：{self.sample_data_sha256}",
            f"- 声明范围：{self.scope}；初始/最终投影弧长：{_format(self.initial_progress_m)} / {_format(self.final_progress_m)} m",
            f"- 绝对弧长比：{_format(self.completion_fraction)}；本次区间覆盖比：{_format(self.scope_completion_fraction)}",
            f"- 本次区间完成：{self.completed}；整条路径覆盖：{self.full_path_covered}；全部所需判据下整条路径验证：{self.full_path_verified}",
            f"- 采样耗时：{_format(self.wall_time_s)} s；计算 deadline：{_format(self.deadline_ms)} ms；超期率：{_format(self.deadline_miss_rate)}",
            "- 未采集边界：" + ", ".join(b for b in BOUNDARIES[1:] if b != self.evidence.boundary),
            "", "## 误差及诊断统计", "",
            "统计按样本等权；部分有效样本仍可供诊断，缺失/无效样本阻止对应指标通过。", "",
            "| 指标（单位见定义） | 有效/缺失/无效 | 均值 | RMS | p95 | 最小 | 最大 |",
            "|---|---|---|---|---|---|---|",
        ]
        for name, stat in self.metrics.items():
            lines.append(f"| {name} | {stat.count}/{stat.missing}/{stat.invalid} | " + " | ".join(
                _format(getattr(stat, key)) for key in ("mean", "rms", "p95", "minimum", "maximum")) + " |")
        lines += ["", "## 逐项判据与来源", "", "| 指标 | 类别 | 阈值/单位 | 状态 | 测量值 | 结论 | 来源/原因 |", "|---|---|---|---|---|---|---|"]
        for c in self.criteria:
            t = c.threshold
            lines.append(f"| {t.metric} | {t.category} | {t.comparison} {_format(t.limit)} {t.unit} | {t.status} | {_format(c.value)} | {VERDICT_LABELS[c.verdict]} | {t.source}; {c.reason} |")
        lines += ["", "## 在线门控与约束", "",
                  f"- QP：success={_format(self.qp_success_rate)}，失败={self.qp_fail_count}，缺失/无效={self.qp_missing_count}",
                  f"- 准入：{self.admission_counts}；重叠：{self.overlap_counts}",
                  f"- 状态事件：{self.events}；限制因素：{self.limiting_reason_counts}",
                  "- 几何裕度、原始约束残差和松弛分别报告；不同量纲不相加。", "",
                  "- legacy_orientation_error 是旧控制误差诊断；delta_slack/qp_primal_residual 是跨类求解诊断，不能当作米解释，物理量请使用逐类数据。", "",
                  "```json", json.dumps(self.constraint_metrics, ensure_ascii=False, indent=2, allow_nan=False), "```", ""]
        if self.issues:
            lines += ["## 数据与范围限制", ""] + [f"- {issue}" for issue in self.issues]
        return "\n".join(lines) + "\n"


def _format(value: float | None) -> str:
    return "未测" if value is None else f"{value:.6g}"


class TrackingEvaluator:
    """Append-only evaluation of one declared scope and one measurement boundary."""

    def __init__(self, trajectory_duration_s: float | None = None, *,
                 scope: EvaluationScope | None = None,
                 evidence: EvidenceContext | None = None,
                 thresholds: tuple[Threshold, ...] | None = None,
                 deadline_ms: float | None = None) -> None:
        for value in (trajectory_duration_s, deadline_ms):
            if value is not None and (not math.isfinite(value) or value <= 0):
                raise ValueError("duration/deadline must be positive and finite")
        self._scope = scope
        self._evidence = evidence or EvidenceContext()
        overrides = tuple(thresholds or ())
        if len({t.metric for t in overrides}) != len(overrides):
            raise ValueError("duplicate threshold metric")
        merged = {t.metric: t for t in baseline_thresholds()}
        merged.update({t.metric: t for t in overrides})
        self._thresholds = tuple(merged.values())
        self._trajectory_duration_s = trajectory_duration_s
        self._deadline_ms = deadline_ms
        self._steps: list[TrackingStepData] = []
        self._times: list[float | None] = []
        self._events: Counter = Counter()
        self._termination: str | None = None

    @property
    def scope(self) -> EvaluationScope | None:
        return self._scope

    @property
    def step_count(self) -> int:
        return len(self._steps)

    @property
    def termination(self) -> str | None:
        return self._termination

    @property
    def trajectory_duration_s(self) -> float | None:
        return self._trajectory_duration_s

    def update(self, step: Any, *, wall_time_s: float | None = None) -> None:
        if self._termination is not None:
            raise RuntimeError("cannot append after finish; start a new evaluator")
        if isinstance(step, Mapping):
            for key, expected in (("measurement_boundary", self._evidence.boundary), ("evidence_kind", self._evidence.kind)):
                if key in step and step[key] != expected:
                    self._events["evidence_invalid"] += 1
                    raise ValueError("sample evidence differs from this evaluator; use a separate report")
        self._steps.append(deepcopy(step_from_result(step)))
        self._times.append(_number(wall_time_s))

    def update_from_controller_result(self, result: dict, *, wall_time_s: float | None = None) -> None:
        self.update(result, wall_time_s=wall_time_s)

    def record_event(self, event: str) -> None:
        if self._termination is not None:
            raise RuntimeError("cannot append events after finish")
        if not event:
            raise ValueError("event must be named")
        self._events[event] += 1

    def finish(self, reason: str) -> None:
        if reason not in ("completed", "held", "task_conflict", "fault", "cancelled", "interrupted"):
            raise ValueError("explicit terminal reason required")
        if self._termination is not None and reason != self._termination:
            raise RuntimeError("cannot rewrite termination")
        self._termination = reason

    def trace_json(self) -> str:
        """Canonical trace for reproduction of the summary, including invalids.

        Nonfinite samples use explicit JSON markers rather than silently
        dropping the sample or emitting nonstandard JSON NaN literals.
        """
        def encode(value):
            if isinstance(value, np.generic):
                value = value.item()
            if isinstance(value, float) and not math.isfinite(value):
                return {"nonfinite": str(value)}
            if isinstance(value, Mapping):
                return {str(k): encode(v) for k, v in value.items()}
            if isinstance(value, (list, tuple, np.ndarray)):
                return [encode(v) for v in value]
            return value
        trace = {
            "schema_version": 1, "scope": asdict(self._scope) if self._scope else None,
            "evidence": asdict(self._evidence), "termination": self._termination,
            "thresholds": [asdict(t) for t in self._thresholds],
            "deadline_ms": self._deadline_ms, "trajectory_duration_s": self.trajectory_duration_s,
            "statistics": "sample-weighted RMS; linear percentile",
            "events": dict(self._events),
            "samples": [dict(asdict(step), wall_time_s=stamp) for step, stamp in zip(self._steps, self._times)],
        }
        return json.dumps(encode(trace), ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False)

    def report(self) -> TrackingReport:
        steps, n = self._steps, self.step_count
        names = set(step_from_result({}).values)
        names.update(name for s in steps for name in s.values)
        metrics = {name: _statistics([_number(s.values.get(name)) for s in steps]) for name in sorted(names)}
        qp_known = sum(s.qp_ok is not None for s in steps)
        qp_fail = sum(s.qp_ok is False for s in steps)
        qp_rate = (qp_known - qp_fail) / qp_known if qp_known else None
        admission = Counter("accepted" if s.admission_ok is True else "rejected" if s.admission_ok is False else "unmeasured" for s in steps)
        overlap = Counter("overlap" if s.overlap is True else "clear" if s.overlap is False else "unmeasured" for s in steps)
        issues: list[str] = []
        times_valid = n > 0 and all(t is not None and math.isfinite(t) for t in self._times)
        times_ordered = times_valid and all(b > a for a, b in zip(self._times, self._times[1:]))
        wall = self._times[-1] - self._times[0] if times_ordered else None
        if not times_ordered:
            issues.append("sample timestamps missing, nonfinite or not strictly increasing")
        if not self._evidence.complete:
            issues.append("evidence identity or measurement boundary incomplete")
        initial = final = fraction = scope_fraction = None
        coverage = False
        if self._scope is not None and n:
            scope = self._scope
            progress = [s.projected_progress_m for s in steps]
            initial = steps[0].projection_before_m
            if initial is None:
                initial = progress[0]
            sequence = [initial] + progress
            # Every supplied observed boundary belongs to the declared interval,
            # including pre-step positions after the first sample. References
            # may lead the observed position and are not scope measurements.
            observed = sequence + [s.projection_before_m for s in steps[1:]
                                   if s.projection_before_m is not None]
            valid = all(v is not None and math.isfinite(v) and -scope.roundoff_m <= v <= scope.total_length_m + scope.roundoff_m for v in observed)
            if valid:
                final = progress[-1]
                fraction = min(max(final / scope.total_length_m, 0.0), 1.0)
                monotonic = all(b + scope.roundoff_m >= a for a, b in zip(sequence, sequence[1:]))
                scope_fraction = min(max((min(final, scope.end_m) - max(initial, scope.start_m)) / (scope.end_m - scope.start_m), 0.0), 1.0)
                within_scope = all(scope.start_m - scope.roundoff_m <= v <= scope.end_m + scope.roundoff_m for v in observed)
                advanced = final - initial > scope.roundoff_m
                coverage = within_scope and advanced and monotonic and abs(initial - scope.start_m) <= scope.roundoff_m and final + scope.roundoff_m >= scope.end_m
                if not within_scope:
                    issues.append("observed samples outside predeclared scope; all statistics retained for diagnosis only")
                if not advanced:
                    issues.append("no positive progress beyond floating-point comparison resolution")
                if not monotonic:
                    issues.append("projected progress regressed; interval coverage invalid")
                if abs(initial - scope.start_m) > scope.roundoff_m:
                    issues.append("observed start differs from predeclared scope start")
            else:
                initial = None
                issues.append("projected progress missing, nonfinite or outside path")
        else:
            issues.append("no samples or no predeclared positive-length scope")
        primary_valid = all(metrics[name].complete for name in ("cross_track_m", "tool_axis_error_rad"))
        completed = coverage and primary_valid and times_ordered and self._termination == "completed"
        terminal_verdict = "pass" if completed else "fail" if self._termination is not None and n else "insufficient_evidence"
        if not completed:
            issues.append("declared interval has not been validly completed")
        latency = metrics["step_latency_ms"]
        deadline_rate = None
        if self._deadline_ms is not None and latency.count:
            samples = [s.values.get("step_latency_ms") for s in steps]
            deadline_rate = sum(v is not None and math.isfinite(v) and v > self._deadline_ms for v in samples) / latency.count
        criteria: list[CriterionResult] = []
        constraints = self._constraint_report(issues)
        units = {
            "cross_track_m": "m", "pos_error_m": "m", "tool_axis_error_rad": "rad",
            "obstacle_clearance_m": "m", "obstacle_margin_m": "m",
            "step_latency_ms": "ms", "feedrate_m_s": "m/s",
            "actual_tangent_speed_m_s": "m/s", "source_time_s": "s",
        }
        for threshold in self._thresholds:
            key = threshold.metric
            value, valid, unit = None, False, None
            if key == "qp_success_rate":
                value, valid = qp_rate, n > 0 and qp_known == n
                unit = "1"
            elif key == "obstacle_clearance_nonnegative":
                clearance = metrics["obstacle_clearance_m"]
                value, valid, unit = clearance.minimum, clearance.complete, "m"
            elif key == "deadline_miss_rate":
                value, valid = deadline_rate, latency.complete and self._deadline_ms is not None
                unit = "1"
            elif "." in key:
                name, statistic = key.rsplit(".", 1)
                stat = metrics.get(name)
                if stat is not None and statistic in ("mean", "rms", "p95", "minimum", "maximum"):
                    value, valid = getattr(stat, statistic), stat.complete
                    unit = units.get(name)
                elif name in constraints and statistic in ("mean", "rms", "p95", "minimum", "maximum"):
                    series = constraints[name]
                    stats = series["statistics"]
                    value = stats[statistic]
                    valid = bool(stats["count"] and not stats["missing"] and not stats["invalid"])
                    unit = series["unit"]
            if threshold.status != "accepted" or threshold.limit is None:
                verdict, reason = "insufficient_evidence", "threshold not accepted for this scope"
            elif unit != threshold.unit:
                verdict, reason = "insufficient_evidence", "criterion unit does not match measured quantity"
            elif not valid or value is None:
                verdict, reason = "insufficient_evidence", "required samples missing or invalid"
            else:
                passed = value <= threshold.limit if threshold.comparison == "<=" else value >= threshold.limit
                verdict, reason = ("pass" if passed else "fail"), "complete samples compared with sourced threshold"
            criteria.append(CriterionResult(threshold, value, verdict, reason))
        fatal = any(self._events[event] for event in ("fault", "task_conflict", "command_rejected", "admission_rejected", "evidence_invalid")) or self._termination in ("fault", "task_conflict")
        negative_clearance = metrics["obstacle_clearance_m"].minimum is not None and metrics["obstacle_clearance_m"].minimum < 0.0
        online = "fail" if fatal or negative_clearance or admission["rejected"] or overlap["overlap"] else "pass" if n and admission["accepted"] == n and overlap["clear"] == n else "insufficient_evidence"
        task_criteria = [c.verdict for c in criteria if c.threshold.category == "task"]
        task = combine_verdicts(task_criteria + [terminal_verdict,
            "pass" if times_ordered and self._evidence.complete and primary_valid else "insufficient_evidence",
            "fail" if fatal or negative_clearance or admission["rejected"] or overlap["overlap"] else "pass"])
        if not task_criteria:
            task = combine_verdicts([task, "insufficient_evidence"])
        verdict = combine_verdicts([task, online] + [c.verdict for c in criteria if c.threshold.category != "task"])
        return TrackingReport(
            total_steps=n, sample_data_sha256=hashlib.sha256(self.trace_json().encode()).hexdigest(),
            evidence=self._evidence, scope=self._scope, metrics=metrics,
            criteria=tuple(criteria), completed=completed, completion_fraction=fraction,
            scope_completion_fraction=scope_fraction,
            full_path_verified=bool(completed and verdict == "pass" and self._scope.is_full_path),
            full_path_covered=bool(coverage and self._scope.is_full_path),
            initial_progress_m=initial, final_progress_m=final, termination=self._termination,
            task_verdict=task, online_verdict=online, verdict=verdict, qp_success_rate=qp_rate,
            qp_fail_count=qp_fail, qp_missing_count=n - qp_known,
            admission_counts=dict(admission), overlap_counts=dict(overlap), events=dict(self._events),
            limiting_reason_counts=dict(Counter(s.limiting_reason_code for s in steps if s.limiting_reason_code is not None)),
            constraint_metrics=constraints, wall_time_s=wall, trajectory_duration_s=self.trajectory_duration_s,
            deadline_ms=self._deadline_ms, deadline_miss_rate=deadline_rate, issues=tuple(issues),
        )

    def _constraint_report(self, issues: list[str]) -> dict[str, dict]:
        """Each class/quantity/unit stays distinct, including gaps across ticks."""
        names = sorted({name for s in self._steps for name in s.constraint_metrics})
        result = {}
        for name in names:
            entries = [s.constraint_metrics.get(name, {}) for s in self._steps]
            identities = {(e.get("quantity"), e.get("unit"), e.get("source")) for e in entries if e}
            valid_identity = len(identities) == 1 and all(next(iter(identities)))
            if not valid_identity:
                issues.append(f"constraint metric {name}: missing/changing quantity, unit or source")
            identity = next(iter(identities)) if len(identities) == 1 else (None, None, None)
            result[name] = dict(zip(("quantity", "unit", "source"), identity))
            result[name]["statistics"] = asdict(_statistics([_number(e.get("value")) if valid_identity else math.nan for e in entries]))
            result[name]["inactive_count"] = sum(e.get("active") is False for e in entries)
        return result
