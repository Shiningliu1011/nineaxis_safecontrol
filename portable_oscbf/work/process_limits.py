from __future__ import annotations

import json
from collections.abc import Iterable, Mapping
from dataclasses import asdict, dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import TYPE_CHECKING, Any

import jsonschema
import numpy as np
import yaml

from work import joint_limit_contract
from work.actuator_limits import JOINT_NAMES
from work.joint_limit_contract import JOINT_LIMIT_CBF_MARGIN, START_IK_MIN_JOINT_MARGIN
from work.nineaxis_model import (
    NineaxisModel,
    ToolMotion,
    content_identity,
    immutable_array,
    require_sha256,
    source_digest,
)
from work.robot_units import joints_mm_deg_to_si

if TYPE_CHECKING:
    from work.static_motion import CandidateMotion

PHASES = ("initialization", "entry", "startup", "tracking", "deceleration", "stopping", "paused",
          "recovery", "endpoint_hold", "exit", "fault")
_FIXED_PHASES = {
    **dict.fromkeys(("joint_max_velocity", "joint_max_acceleration", "joint_max_jerk", "joint_position",
                     "cbf_position_margin", "start_ik_position_margin", "total_timeout"), frozenset(PHASES)),
    "recovery_position": frozenset(("entry", "recovery")),
    "recovery_axis": frozenset(("entry", "recovery")),
    "recovery_confirmation": frozenset(("recovery",)), "pause_timeout": frozenset(("paused", "recovery")),
    "endpoint_position": frozenset(("endpoint_hold",)), "endpoint_axis": frozenset(("endpoint_hold",)),
    "endpoint_hold": frozenset(("endpoint_hold",)), "entry_observation": frozenset(("entry",)),
    "source_chord_error": frozenset(("source_interpolation",)),
    "source_interpolation_period": frozenset(("source_interpolation",)),
    "source_nominal_feed": frozenset(("source_nominal",)),
    "source_nominal_normal_acceleration": frozenset(("source_nominal",)),
    "source_geometry_scan_feed": frozenset(("source_geometry_scan",)),
    "source_numeric_position_tolerance": frozenset(("source_numeric_check",)),
    **dict.fromkeys(("s0_path_radius", "s0_workpiece_radius", "s0_nominal_work_distance"),
                    frozenset(("scene_construction",))),
    **dict.fromkeys(("existing_position_manipulability_min", "existing_position_manipulability_activation"),
                    frozenset(("source_metric",))),
}
_CONFIG = Path(__file__).resolve().parents[1] / "config/nineaxis_research_limits.yaml"
_UNITS = {
    "joint_max_velocity": "mm_deg/s", "joint_max_acceleration": "mm_deg/s^2", "joint_max_jerk": "mm_deg/s^3",
    "joint_position": "m_rad", "cbf_position_margin": "m_rad", "start_ik_position_margin": "m_rad",
    "feed_max": "mm/s", "feed_acceleration_max": "mm/s^2", "normal_acceleration_max": "mm/s^2",
    "feed_jerk_max": "mm/s^3", "normal_jerk_max": "mm/s^3", "source_chord_error": "mm",
    "source_interpolation_period": "s", "source_nominal_feed": "mm/s",
    "source_nominal_normal_acceleration": "mm/s^2", "source_geometry_scan_feed": "mm/s",
    "source_numeric_position_tolerance": "mm", "cross_track_rms": "mm", "cross_track_p95": "mm",
    "cross_track_max": "mm", "axis_rms": "deg", "axis_max": "deg", "pause_position": "mm",
    "pause_axis": "deg", "recovery_position": "mm", "recovery_axis": "deg", "recovery_confirmation": "s",
    "pause_timeout": "s", "endpoint_position": "mm", "endpoint_axis": "deg", "endpoint_hold": "s",
    "entry_observation": "s", "total_timeout": "s", "external_clearance": "mm", "self_clearance": "mm",
    "s0_path_radius": "mm", "s0_workpiece_radius": "mm", "s0_nominal_work_distance": "mm",
    "minimum_processing_feed": "mm/s", "tcp_acceleration_norm": "mm/s^2", "tcp_jerk_norm": "mm/s^3",
    "workpiece_normal_speed": "mm/s", "tool_axis_speed": "1/s", "tool_axis_acceleration": "1/s^2",
    "tool_axis_jerk": "1/s^3", "angular_speed": "deg/s", "angular_acceleration": "deg/s^2",
    "angular_jerk": "deg/s^3", "work_distance_tolerance": "mm", "feedback_stopped": "criterion",
    "timing_qualification": "s", "singularity_qualification": "criterion",
    "existing_position_manipulability_min": "legacy_mixed_SI_metric",
    "existing_position_manipulability_activation": "legacy_mixed_SI_metric",
}
_DERIVED = {"joint_position", "cbf_position_margin", "start_ik_position_margin"}
_UNAVAILABLE_CHECKS = {"work_distance_tolerance", "feedback_stopped", "timing_qualification", "singularity_qualification"}
_TEXT = {"type": "string", "minLength": 1}
_PARAMETER_PROPERTIES = {
    **{name: _TEXT for name in ("quantity", "object", "frame", "unit", "source")},
    "phases": {"type": "array", "minItems": 1, "uniqueItems": True, "items": {"enum": [
        *PHASES, "source_interpolation", "source_nominal", "source_geometry_scan", "source_numeric_check", "scene_construction", "source_metric"]}},
    "value_or_bounds": {"oneOf": [{"type": "number"}, {"type": "null"}, {
        "type": "array", "minItems": 9, "maxItems": 9, "items": {"type": "number"}}]},
    "assumptions": {"type": "array", "minItems": 1, "items": _TEXT},
    "verification_state": {"enum": ["confirmed_research", "source_only", "model_derived", "unconfigured"]},
    "acquisition": {"type": "string"},
}
_PARAMETER_SCHEMA = {"type": "object", "properties": _PARAMETER_PROPERTIES,
                     "required": list(_PARAMETER_PROPERTIES), "additionalProperties": False}
_CONFIG_PROPERTIES = {
    "schema_version": {"type": "integer", "const": 1}, "revision": _TEXT, "model_id": _TEXT,
    "joint_names": {"const": list(JOINT_NAMES)}, "evidence_scope": {"const": "research_simulation"},
    "parameters": {"type": "object", "properties": {name: _PARAMETER_SCHEMA for name in _UNITS},
                   "required": list(_UNITS), "additionalProperties": False},
}
_SCHEMA = {"type": "object", "properties": _CONFIG_PROPERTIES,
           "required": list(_CONFIG_PROPERTIES), "additionalProperties": False}


@dataclass(frozen=True)
class LimitParameter:
    quantity: str
    object: str
    frame: str
    value_or_bounds: float | tuple[float, ...] | None
    unit: str
    phases: tuple[str, ...]
    source: str
    assumptions: tuple[str, ...]
    verification_state: str
    acquisition: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "phases", tuple(self.phases))
        object.__setattr__(self, "assumptions", tuple(self.assumptions))
        if isinstance(self.value_or_bounds, (list, tuple)):
            object.__setattr__(self, "value_or_bounds", tuple(float(value) for value in self.value_or_bounds))
        elif self.value_or_bounds is not None:
            object.__setattr__(self, "value_or_bounds", float(self.value_or_bounds))
        if self.value_or_bounds is not None and (
                not np.all(np.isfinite(self.value_or_bounds)) or np.any(np.asarray(self.value_or_bounds) <= 0)):
            raise ValueError(f"{self.quantity} 需要有限正数")
        if self.verification_state == "unconfigured":
            if self.value_or_bounds is not None or not self.acquisition:
                raise ValueError(f"{self.quantity} 未配置项需要空值与取得方式")
        elif self.verification_state == "model_derived":
            if self.value_or_bounds is not None:
                raise ValueError(f"{self.quantity} 从共同模型读取")
        elif self.value_or_bounds is None:
            raise ValueError(f"{self.quantity} 缺少数值")


@dataclass(frozen=True)
class ProcessLimits:
    revision: str
    model_id: str
    parameters: Mapping[str, LimitParameter]
    config_hash: str = field(init=False)

    def __post_init__(self) -> None:
        parameters = dict(self.parameters)
        rows = json.loads(json.dumps({name: asdict(value) for name, value in parameters.items()}, allow_nan=False))
        record = {"schema_version": 1, "revision": self.revision, "model_id": self.model_id,
                  "joint_names": list(JOINT_NAMES), "evidence_scope": "research_simulation", "parameters": rows}
        jsonschema.validate(record, _SCHEMA)
        for name, parameter in parameters.items():
            if parameter.unit != _UNITS[name]:
                raise ValueError(f"{name} unit 需要 {_UNITS[name]}")
            phases = frozenset(parameter.phases)
            if name in _FIXED_PHASES:
                if phases != _FIXED_PHASES[name]:
                    raise ValueError(f"{name} phases 需要对应已实现的阶段职责")
            elif not phases.issubset(PHASES):
                raise ValueError(f"{name} phases 需要运行阶段")
            if name.startswith("joint_max_") != isinstance(parameter.value_or_bounds, tuple):
                raise ValueError(f"{name} 需要对应标量或九轴数组")
            if (parameter.verification_state == "model_derived") != (name in _DERIVED):
                raise ValueError(f"{name} model_derived 来源不符")
            if name in _UNAVAILABLE_CHECKS and parameter.verification_state != "unconfigured":
                raise ValueError(f"{name} 需要对应任务提供完整检查定义，当前保持 unconfigured")
        object.__setattr__(self, "parameters", MappingProxyType(parameters))
        object.__setattr__(self, "config_hash", content_identity(record))

    @classmethod
    def load(cls, path: Path = _CONFIG, *, expected_hash: str | None = None) -> ProcessLimits:
        values = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        jsonschema.validate(values, _SCHEMA)
        parameters = {name: LimitParameter(**row) for name, row in values["parameters"].items()}
        result = cls(values["revision"], values["model_id"], parameters)
        if expected_hash is not None and expected_hash != result.config_hash:
            raise ValueError("expected_hash 与当前限制配置不符")
        return result

    def value(self, name: str) -> float:
        value = self.parameters[name].value_or_bounds
        if value is None or isinstance(value, tuple):
            raise ValueError(f"{name} 没有已配置的标量")
        return float(value)

    def bind(self, model: NineaxisModel, *, path_hash: str, workpiece_hash: str, scene_hash: str, path_length_m: float,
             phase: str, evidence_scope: str) -> BoundProcessLimits:
        if model.model_id != self.model_id:
            raise ValueError("model_id 与限制配置不符")
        if evidence_scope != "research_simulation":
            raise RuntimeError("共同研究配置仅具有 research_simulation 证据，hardware 需要设备与标定验收")
        return BoundProcessLimits(self, model, path_hash, workpiece_hash, scene_hash, path_length_m, phase, evidence_scope)


@dataclass(frozen=True)
class ReferenceFeed:
    length_m: float
    nu_s_inv: float
    nu_dot_s_inv2: float
    nu_ddot_s_inv3: float
    curvature_m_inv: float
    source_identity: str
    path_hash: str
    time_s: float
    clock_id: str

    def __post_init__(self) -> None:
        require_sha256(self.source_identity, "reference source_identity")
        require_sha256(self.path_hash, "reference path_hash")
        if not isinstance(self.clock_id, str) or not self.clock_id:
            raise ValueError("reference clock_id 需要明确时间基准")
        values = [self.length_m, self.nu_s_inv, self.nu_dot_s_inv2, self.nu_ddot_s_inv3, self.curvature_m_inv, self.time_s]
        if not np.all(np.isfinite(values)) or self.length_m <= 0 or self.nu_s_inv < 0 or self.curvature_m_inv < 0:
            raise ValueError("reference feed 需要有限数值、正弧长和非负进给及曲率")
        if self.time_s < 0:
            raise ValueError("reference time_s 需要有限非负值")


@dataclass(frozen=True)
class ProcessObservation:
    motion: ToolMotion
    layer: str
    source_identity: str
    path_hash: str
    workpiece_hash: str
    scene_hash: str
    tangent_derivatives: np.ndarray
    workpiece_normal: np.ndarray
    reference_position_m: np.ndarray
    reference_axis: np.ndarray
    projected_position_m: np.ndarray
    projection_identity: str
    projection_time_s: float
    projection_clock_id: str
    projection_valid: bool
    work_distances_m: np.ndarray | None = None
    external_clearance_m: float | None = None
    self_clearance_m: float | None = None
    reference_feed: ReferenceFeed | None = None

    def __post_init__(self) -> None:
        for name in ("source_identity", "path_hash", "workpiece_hash", "scene_hash", "projection_identity"):
            require_sha256(getattr(self, name), name)
        if (not np.isfinite(self.projection_time_s) or self.projection_time_s != self.motion.time_s
                or self.projection_clock_id != self.motion.clock_id or self.projection_valid is not True):
            raise ValueError("projection 需要同一有效时刻、时间基准和有序局部分支证据")
        if self.layer not in ("actual", "execution_reference", "source_nominal"):
            raise ValueError("layer 需要 actual、execution_reference 或 source_nominal")
        if self.layer == "actual" and self.motion.sample_source == "generated_motion":
            raise ValueError("actual 工具运动需要反馈来源")
        if self.layer != "actual" and self.motion.sample_source != "generated_motion":
            raise ValueError("参考工具运动需要 generated_motion 来源")
        for name, shape in (("tangent_derivatives", (3, 3)), ("workpiece_normal", (3,)),
                            ("reference_position_m", (3,)), ("reference_axis", (3,)), ("projected_position_m", (3,))):
            object.__setattr__(self, name, immutable_array(getattr(self, name), shape, name))
        tangent, first, second = self.tangent_derivatives
        if (abs(tangent @ tangent - 1) > 1e-9 or abs(tangent @ first) > 1e-9
                or abs(tangent @ second + first @ first) > 1e-9):
            raise ValueError("tangent 及时间导数需要满足单位轴约束")
        for name in ("workpiece_normal", "reference_axis"):
            if abs(np.linalg.norm(getattr(self, name)) - 1) > 1e-9:
                raise ValueError(f"{name} 需要单位向量")
        if self.work_distances_m is not None:
            object.__setattr__(self, "work_distances_m", immutable_array(self.work_distances_m, (3,), "work_distances_m"))
            if self.work_distances_m[0] < 0 or self.work_distances_m[2] < 0:
                raise ValueError("最近工作距离与切向分量需要非负值")
        for name in ("external_clearance_m", "self_clearance_m"):
            value = getattr(self, name)
            if value is not None and not np.isfinite(value):
                raise ValueError(f"{name} 需要有限数值或明确缺失")


@dataclass(frozen=True)
class LimitReport:
    binding_identity: str
    layer: str
    source_identity: str
    metrics: Mapping[str, float]
    violations: tuple[str, ...]
    missing_required: tuple[str, ...]
    unconfigured: tuple[str, ...]
    pause_required: bool
    evidence_scope: str
    provenance: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "metrics", MappingProxyType(dict(self.metrics)))
        object.__setattr__(self, "provenance", MappingProxyType(dict(self.provenance)))

    @property
    def accepted(self) -> bool:
        return not self.violations and not self.missing_required

    def to_record(self) -> dict[str, Any]:
        return {"schema_version": 1, "binding_identity": self.binding_identity, "layer": self.layer,
                "source_identity": self.source_identity, "metrics": dict(self.metrics),
                "violations": list(self.violations), "missing_required": list(self.missing_required),
                "unconfigured": list(self.unconfigured), "pause_required": self.pause_required,
                "evidence_scope": self.evidence_scope, "accepted": self.accepted,
                "provenance": dict(self.provenance)}


@dataclass(frozen=True)
class PhaseEvidence:
    total_elapsed_s: float
    phase_elapsed_s: float
    confirmation_elapsed_s: float
    stopped: bool | None
    ordered_coverage_complete: bool
    positive_candidate: bool = False
    waiting_elapsed_s: float | None = None

    def __post_init__(self) -> None:
        values = [self.total_elapsed_s, self.phase_elapsed_s, self.confirmation_elapsed_s]
        if not np.all(np.isfinite(values)) or min(values) < 0:
            raise ValueError("phase 时间需要有限非负值")
        if self.confirmation_elapsed_s > self.phase_elapsed_s or self.phase_elapsed_s > self.total_elapsed_s:
            raise ValueError("phase 时间区间包含关系不符")
        if self.waiting_elapsed_s is not None and (not np.isfinite(self.waiting_elapsed_s)
                or self.waiting_elapsed_s < 0 or self.waiting_elapsed_s > self.total_elapsed_s):
            raise ValueError("waiting_elapsed_s 需要累计等待时间并包含在总时间内")
        if (self.stopped is not None and type(self.stopped) is not bool
                or type(self.ordered_coverage_complete) is not bool or type(self.positive_candidate) is not bool):
            raise ValueError("phase 条件需要明确布尔证据")


@dataclass(frozen=True)
class PhaseStatus:
    can_continue: bool
    ready_to_transition: bool
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class BoundProcessLimits:
    limits: ProcessLimits
    model: NineaxisModel
    path_hash: str
    workpiece_hash: str
    scene_hash: str
    path_length_m: float
    phase: str
    evidence_scope: str
    identity: str = field(init=False)
    max_velocity: np.ndarray = field(init=False, repr=False)
    max_acceleration: np.ndarray = field(init=False, repr=False)
    max_jerk: np.ndarray = field(init=False, repr=False)

    def __post_init__(self) -> None:
        for name in ("path_hash", "workpiece_hash", "scene_hash"):
            require_sha256(getattr(self, name), name)
        if self.evidence_scope != "research_simulation" or self.model.model_id != self.limits.model_id:
            raise ValueError("binding model_id 或 evidence_scope 不符合共同研究配置")
        if self.phase not in PHASES:
            raise ValueError("phase 不属于共同阶段")
        if not np.isfinite(self.path_length_m) or self.path_length_m <= 0:
            raise ValueError("path_length_m 需要最终路径的有限正弧长")
        record = {name: getattr(self, name) for name in ("path_hash", "workpiece_hash", "scene_hash", "phase", "evidence_scope")}
        record.update(config_hash=self.limits.config_hash, model_hash=self.model.model_hash,
                      geometry_hash=self.model.geometry_hash,
                      path_length_m=self.path_length_m,
                      joint_margin_source_hash=source_digest(Path(joint_limit_contract.__file__)),
                      cbf_margin=JOINT_LIMIT_CBF_MARGIN, start_ik_margin=START_IK_MIN_JOINT_MARGIN)
        object.__setattr__(self, "identity", content_identity(record))
        for name in ("max_velocity", "max_acceleration", "max_jerk"):
            parameter = self.limits.parameters[f"joint_{name}"].value_or_bounds
            if not isinstance(parameter, tuple):
                raise TypeError(f"joint_{name} 需要九轴数组")
            value = joints_mm_deg_to_si(parameter)
            object.__setattr__(self, name, immutable_array(value, (9,), name))

    @property
    def position_lower(self) -> np.ndarray:
        return self.model.position_lower

    @property
    def position_upper(self) -> np.ndarray:
        return self.model.position_upper

    @property
    def cbf_margin(self) -> float:
        return JOINT_LIMIT_CBF_MARGIN

    @property
    def start_ik_margin(self) -> float:
        return START_IK_MIN_JOINT_MARGIN

    def _applies(self, quantity: str) -> bool:
        return self.phase in self.limits.parameters[quantity].phases

    def _validate(self, observation: ProcessObservation) -> None:
        for name in ("path_hash", "workpiece_hash", "scene_hash"):
            if getattr(observation, name) != getattr(self, name):
                raise ValueError(f"{name} 与当前绑定不符")
        if observation.motion.model_hash != self.model.model_hash:
            raise ValueError("model_hash 与当前绑定不符")
        if observation.reference_feed is not None:
            feed = observation.reference_feed
            if (feed.path_hash != self.path_hash or feed.length_m != self.path_length_m
                    or feed.time_s != observation.motion.time_s or feed.clock_id != observation.motion.clock_id):
                raise ValueError("reference_feed 与绑定路径、最终弧长或观测时刻不符")

    def measure(self, observation: ProcessObservation) -> dict[str, float]:
        self._validate(observation)
        motion = observation.motion
        tangent, first, second = observation.tangent_derivatives
        velocity, acceleration, jerk = motion.velocity_m_s, motion.acceleration_m_s2, motion.jerk_m_s3
        normal_projector = np.eye(3) - np.outer(tangent, tangent)
        axis = motion.axis_derivatives[0]
        axis_error = np.arctan2(np.linalg.norm(np.cross(axis, observation.reference_axis)), axis @ observation.reference_axis)
        metrics = {
            "tcp_speed_mm_s": np.linalg.norm(velocity) * 1000,
            "tcp_acceleration_mm_s2": np.linalg.norm(acceleration) * 1000,
            "tcp_jerk_mm_s3": np.linalg.norm(jerk) * 1000,
            "feed_mm_s": tangent @ velocity * 1000,
            "feed_acceleration_mm_s2": (first @ velocity + tangent @ acceleration) * 1000,
            "feed_jerk_mm_s3": (second @ velocity + 2 * first @ acceleration + tangent @ jerk) * 1000,
            "cartesian_tangent_acceleration_mm_s2": tangent @ acceleration * 1000,
            "cartesian_tangent_jerk_mm_s3": tangent @ jerk * 1000,
            "normal_plane_speed_mm_s": np.linalg.norm(normal_projector @ velocity) * 1000,
            "normal_plane_acceleration_mm_s2": np.linalg.norm(normal_projector @ acceleration) * 1000,
            "normal_plane_jerk_mm_s3": np.linalg.norm(normal_projector @ jerk) * 1000,
            "workpiece_normal_speed_mm_s": observation.workpiece_normal @ velocity * 1000,
            "reference_position_error_mm": np.linalg.norm(motion.position_m - observation.reference_position_m) * 1000,
            "cross_track_mm": np.linalg.norm(motion.position_m - observation.projected_position_m) * 1000,
            "axis_error_deg": np.rad2deg(axis_error),
            "existing_position_manipulability_raw": np.prod(np.linalg.svd(self.model.jacobian(
                motion.joint_derivatives[0])[:3], compute_uv=False)),
        }
        for order, name in enumerate(("speed", "acceleration", "jerk"), start=1):
            metrics[f"tool_axis_{name}_s_inv{order}"] = np.linalg.norm(motion.axis_derivatives[order])
            metrics[f"angular_{name}_deg_s{order}"] = np.rad2deg(np.linalg.norm(motion.angular_derivatives[order - 1]))
        if observation.work_distances_m is not None:
            for name, value in zip(("work_distance_nearest_mm", "work_distance_normal_mm", "work_distance_tangent_mm"), observation.work_distances_m):
                metrics[name] = value * 1000
        if observation.reference_feed is not None:
            feed = observation.reference_feed
            speed = feed.length_m * feed.nu_s_inv
            metrics.update({
                "reference_feed_mm_s": speed * 1000,
                "reference_feed_acceleration_mm_s2": feed.length_m * feed.nu_dot_s_inv2 * 1000,
                "reference_feed_jerk_mm_s3": feed.length_m * feed.nu_ddot_s_inv3 * 1000,
                "reference_normal_acceleration_mm_s2": feed.curvature_m_inv * speed**2 * 1000,
                "reference_curvature_jerk_mm_s3": feed.curvature_m_inv**2 * speed**3 * 1000,
            })
        if not np.all(np.isfinite(list(metrics.values()))):
            raise ValueError("工具工艺计算产生非有限值")
        return {name: float(value) for name, value in metrics.items()}

    def check(self, observation: ProcessObservation, *, required_quantities: Iterable[str] = ()) -> LimitReport:
        metrics = self.measure(observation)
        violations, missing = [], []
        for name in required_quantities:
            parameter = self.limits.parameters[name]
            if parameter.verification_state == "unconfigured":
                missing.append(name)
        positions = observation.motion.joint_derivatives[0]
        if np.any(positions < self.position_lower) or np.any(positions > self.position_upper):
            violations.append("joint_position")
        for index, name in enumerate(("velocity", "acceleration", "jerk"), start=1):
            ratio = np.max(np.abs(observation.motion.joint_derivatives[index]) / getattr(self, f"max_{name}"))
            metrics[f"joint_{name}_max_ratio"] = float(ratio)
            if ratio > 1:
                violations.append(f"joint_max_{name}")
        checks = {
            "feed_max": [metrics["tcp_speed_mm_s"]],
            "feed_acceleration_max": [abs(metrics["feed_acceleration_mm_s2"])],
            "normal_acceleration_max": [metrics["normal_plane_acceleration_mm_s2"]],
            "feed_jerk_max": [abs(metrics["feed_jerk_mm_s3"])],
            "normal_jerk_max": [metrics["normal_plane_jerk_mm_s3"]],
        }
        if observation.reference_feed is not None:
            checks["feed_max"].append(metrics["reference_feed_mm_s"])
            checks["feed_acceleration_max"].append(abs(metrics["reference_feed_acceleration_mm_s2"]))
            checks["feed_jerk_max"].append(abs(metrics["reference_feed_jerk_mm_s3"]))
            checks["normal_acceleration_max"].append(metrics["reference_normal_acceleration_mm_s2"])
            checks["normal_jerk_max"].append(metrics["reference_curvature_jerk_mm_s3"])
        elif observation.layer != "actual":
            missing.append("reference_feed")
        for name, values in checks.items():
            if self._applies(name) and max(values) > self.limits.value(name):
                violations.append(name)
        optional_metrics = {
            "minimum_processing_feed": ("feed_mm_s", True),
            "tcp_acceleration_norm": ("tcp_acceleration_mm_s2", False),
            "tcp_jerk_norm": ("tcp_jerk_mm_s3", False),
            "workpiece_normal_speed": ("workpiece_normal_speed_mm_s", False),
            "tool_axis_speed": ("tool_axis_speed_s_inv1", False),
            "tool_axis_acceleration": ("tool_axis_acceleration_s_inv2", False),
            "tool_axis_jerk": ("tool_axis_jerk_s_inv3", False),
            "angular_speed": ("angular_speed_deg_s1", False),
            "angular_acceleration": ("angular_acceleration_deg_s2", False),
            "angular_jerk": ("angular_jerk_deg_s3", False),
        }
        for name, (metric, minimum) in optional_metrics.items():
            parameter = self.limits.parameters[name]
            if parameter.value_or_bounds is not None and self.phase in parameter.phases:
                value = metrics[metric] if minimum else abs(metrics[metric])
                if (minimum and value < self.limits.value(name)) or (not minimum and value > self.limits.value(name)):
                    violations.append(name)
        for name, metric in (("cross_track_max", "cross_track_mm"), ("axis_max", "axis_error_deg")):
            if self._applies(name) and metrics[metric] > self.limits.value(name):
                violations.append(name)
        for name in ("external_clearance", "self_clearance"):
            if not self._applies(name):
                continue
            value = getattr(observation, f"{name}_m")
            if value is None:
                missing.append(name)
            elif value * 1000 < self.limits.value(name):
                violations.append(name)
            else:
                metrics[f"{name}_mm"] = value * 1000
        pause = any(self._applies(name) and metrics[metric] > self.limits.value(name) for name, metric in (
            ("pause_position", "reference_position_error_mm"), ("pause_axis", "axis_error_deg")))
        return LimitReport(self.identity, observation.layer, observation.source_identity, metrics,
                           tuple(violations), tuple(missing), tuple(name for name, parameter in self.limits.parameters.items()
                           if parameter.verification_state == "unconfigured"), pause, self.evidence_scope,
                           {"model_hash": self.model.model_hash, "config_hash": self.limits.config_hash,
                            "path_hash": self.path_hash, "workpiece_hash": self.workpiece_hash, "scene_hash": self.scene_hash,
                            "path_length_m": self.path_length_m,
                            "phase": self.phase, "time_s": observation.motion.time_s, "clock_id": observation.motion.clock_id,
                            "sample_source": observation.motion.sample_source, "projection_identity": observation.projection_identity,
                            "reference_feed_identity": None if observation.reference_feed is None else observation.reference_feed.source_identity})

    def phase_status(self, observation: ProcessObservation, evidence: PhaseEvidence) -> PhaseStatus:
        if observation.layer != "actual":
            raise ValueError("phase 检查需要 actual 运动")
        report = self.check(observation)
        reasons = list(report.violations + report.missing_required)
        if report.pause_required:
            reasons.append("pause_required")
        if evidence.total_elapsed_s > self.limits.value("total_timeout"):
            reasons.append("total_timeout")
        if self.phase in ("paused", "recovery"):
            if evidence.waiting_elapsed_s is None:
                reasons.append("waiting_elapsed_s")
            elif evidence.waiting_elapsed_s > self.limits.value("pause_timeout"):
                reasons.append("pause_timeout")
        ready = False
        if self.phase == "recovery":
            ready = (report.metrics["reference_position_error_mm"] <= self.limits.value("recovery_position")
                     and report.metrics["axis_error_deg"] <= self.limits.value("recovery_axis")
                     and evidence.confirmation_elapsed_s >= self.limits.value("recovery_confirmation")
                     and evidence.positive_candidate)
        elif self.phase == "endpoint_hold":
            stopped_consistent = evidence.stopped is True and not np.any(observation.motion.joint_derivatives[1:])
            ready = (evidence.ordered_coverage_complete and stopped_consistent
                     and report.metrics["reference_position_error_mm"] <= self.limits.value("endpoint_position")
                     and report.metrics["axis_error_deg"] <= self.limits.value("endpoint_axis")
                     and evidence.confirmation_elapsed_s >= self.limits.value("endpoint_hold"))
        elif self.phase == "entry":
            ready = (evidence.phase_elapsed_s >= self.limits.value("entry_observation")
                     and report.metrics["reference_position_error_mm"] <= self.limits.value("recovery_position")
                     and report.metrics["axis_error_deg"] <= self.limits.value("recovery_axis"))
        return PhaseStatus(not reasons, bool(ready and not reasons), tuple(reasons))

    def check_joint_motion(self, motion: CandidateMotion) -> LimitReport:
        if motion.limits_identity != self.identity:
            raise ValueError("limits_identity 与完整关节运动检查不符")
        bounds = motion.bounds
        violations = []
        metrics = {"complete_motion_duration_s": float(motion.duration_s)}
        if np.any(bounds.lower < self.position_lower) or np.any(bounds.upper > self.position_upper):
            violations.append("joint_position")
        for name in ("velocity", "acceleration", "jerk"):
            ratio = float(np.max(getattr(bounds, f"max_{name}") / getattr(self, f"max_{name}")))
            if not np.isfinite(ratio):
                raise ValueError(f"完整关节运动 {name} 产生非有限值")
            metrics[f"joint_{name}_max_ratio"] = ratio
            if ratio > 1 + 1e-12:
                violations.append(f"joint_max_{name}")
        for index, name in enumerate(self.model.joint_names):
            unit = "m" if index == 0 else "rad"
            metrics[f"{name}_position_lower_{unit}"] = float(bounds.lower[index])
            metrics[f"{name}_position_upper_{unit}"] = float(bounds.upper[index])
        return LimitReport(self.identity, "joint_motion", content_identity(motion.to_record()), metrics,
            tuple(violations), (), (), False, self.evidence_scope,
            {"check_scope": "complete_joint_motion_and_stop", "model_hash": self.model.model_hash,
             "config_hash": self.limits.config_hash, "phase": self.phase})

    def tracking_quality(self, observations: Iterable[ProcessObservation]) -> LimitReport:
        samples = list(observations)
        if not samples:
            raise ValueError("tracking quality empty 样本")
        if any(sample.layer != "actual" for sample in samples):
            raise ValueError("tracking quality 需要 actual 样本")
        metrics = [self.measure(sample) for sample in samples]
        cross_track = np.array([row["cross_track_mm"] for row in metrics])
        axes = np.array([row["axis_error_deg"] for row in metrics])
        values = {
            "cross_track_rms_mm": float(np.sqrt(np.mean(cross_track**2))),
            "cross_track_p95_mm": float(np.percentile(cross_track, 95)),
            "cross_track_max_mm": float(np.max(cross_track)),
            "axis_rms_deg": float(np.sqrt(np.mean(axes**2))), "axis_max_deg": float(np.max(axes)),
        }
        violations = tuple(name for name, key in (("cross_track_rms", "cross_track_rms_mm"),
            ("cross_track_p95", "cross_track_p95_mm"), ("cross_track_max", "cross_track_max_mm"),
            ("axis_rms", "axis_rms_deg"), ("axis_max", "axis_max_deg"))
            if self._applies(name) and values[key] > self.limits.value(name))
        identity = content_identity([sample.source_identity for sample in samples])
        return LimitReport(self.identity, "actual", identity, values, violations, (), (), False, self.evidence_scope)
