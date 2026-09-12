"""Immutable OFF-15 scope, evidence identity and sourced thresholds."""

from dataclasses import dataclass
import math


BOUNDARIES = ("unspecified", "kernel_candidate", "filtered_command", "simulated_state", "real_feedback")
EVIDENCE_KINDS = ("unspecified", "analytic", "model", "simulation", "hardware_recording")
VERDICT_LABELS = {"pass": "通过", "fail": "不通过", "insufficient_evidence": "证据不足"}


@dataclass(frozen=True)
class EvaluationScope:
    """Arc interval fixed before sampling; zero-length tasks are invalid."""

    total_length_m: float
    start_m: float = 0.0
    end_m: float | None = None

    def __post_init__(self) -> None:
        if self.end_m is None:
            object.__setattr__(self, "end_m", self.total_length_m)
        if not all(math.isfinite(v) for v in (self.total_length_m, self.start_m, self.end_m)) or not 0 <= self.start_m < self.end_m <= self.total_length_m:
            raise ValueError("scope requires 0 <= start < end <= finite total length")

    @property
    def is_full_path(self) -> bool:
        return self.start_m == 0.0 and self.end_m == self.total_length_m

    @property
    def roundoff_m(self) -> float:
        # Floating-point comparison only, never a task/safety allowance.
        return 64 * math.ulp(max(1.0, self.total_length_m))


@dataclass(frozen=True)
class EvidenceContext:
    """Source and measurement boundary are independent explicit dimensions.

    Identities may reference an immutable runtime snapshot. A hardware label
    describes supplied recordings; it never authorizes hardware execution.
    """

    run_id: str = ""
    kind: str = "unspecified"
    boundary: str = "unspecified"
    model_id: str = ""
    config_id: str = ""
    trajectory_id: str = ""
    data_id: str = ""
    scenario: str = ""
    measurement: str = ""
    time_basis: str = ""

    def __post_init__(self) -> None:
        if self.boundary not in BOUNDARIES or self.kind not in EVIDENCE_KINDS:
            raise ValueError("unknown evidence kind or measurement boundary")
        if self.boundary == "real_feedback" and self.kind != "hardware_recording":
            raise ValueError("real_feedback requires hardware_recording evidence")

    @property
    def complete(self) -> bool:
        return self.boundary != "unspecified" and self.kind != "unspecified" and all(
            isinstance(v, str) and v.strip() for v in (
                self.run_id, self.model_id, self.config_id, self.trajectory_id,
                self.data_id, self.scenario, self.measurement, self.time_basis,
            ))


@dataclass(frozen=True)
class Threshold:
    """Units, purpose, approval state and provenance travel with each criterion."""

    metric: str
    limit: float | None
    comparison: str
    unit: str
    category: str
    status: str
    source: str

    def __post_init__(self) -> None:
        if self.comparison not in ("<=", ">="):
            raise ValueError("threshold comparison must be <= or >=")
        if self.category not in ("task", "numerical", "safety", "timing"):
            raise ValueError("unknown threshold category")
        if self.status not in ("accepted", "provisional", "unknown"):
            raise ValueError("unknown threshold status")
        if not self.metric or not self.unit or not self.source:
            raise ValueError("threshold requires metric, unit and source")
        if self.limit is not None and not math.isfinite(self.limit):
            raise ValueError("threshold limit must be finite or None")
        if self.status == "accepted" and self.limit is None:
            raise ValueError("accepted threshold requires a limit")


def baseline_thresholds() -> tuple[Threshold, ...]:
    """Preserve #14 task criteria; provisional safety numbers stay provisional."""
    source = "GitHub #14 resolution 2026-09-04"
    angle = source + "; OFF-15 user decision: true axis angle 2026-09-12"
    return (
        Threshold("cross_track_m.rms", 0.00015, "<=", "m", "task", "accepted", source),
        Threshold("cross_track_m.p95", 0.0005, "<=", "m", "task", "accepted", source),
        Threshold("cross_track_m.maximum", 0.002, "<=", "m", "task", "accepted", source),
        Threshold("tool_axis_error_rad.rms", math.radians(0.05), "<=", "rad", "task", "accepted", angle),
        Threshold("tool_axis_error_rad.maximum", math.radians(0.5), "<=", "rad", "task", "accepted", angle),
        Threshold("qp_success_rate", 0.999, ">=", "1", "task", "accepted", source),
        Threshold("obstacle_clearance_nonnegative", 0.0, ">=", "m", "safety", "accepted", source + "; nonnegative clearance alone does not prove non-overlap"),
        Threshold("obstacle_clearance_m.minimum", 0.03, ">=", "m", "safety", "provisional", source + "; geometry/scope pending #8/#18"),
        Threshold("deadline_miss_rate", 0.01, "<=", "1", "timing", "provisional", source + "; deadline must match current measured boundary/config"),
    )


def combine_verdicts(verdicts: list[str]) -> str:
    if "fail" in verdicts:
        return "fail"
    if not verdicts or "insufficient_evidence" in verdicts:
        return "insufficient_evidence"
    return "pass"
