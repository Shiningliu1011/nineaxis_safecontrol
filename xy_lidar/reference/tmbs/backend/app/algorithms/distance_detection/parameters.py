"""巷道距离探测纯算法参数模型。

所有模型均使用 snake_case 字段名，不依赖 FastAPI、ORM、EventHub、
ArtifactStore 或真实雷达。字段默认值和取值范围严格对齐探测 PRD §9。
"""

from __future__ import annotations

from typing import Any, Mapping

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    ValidationError,
    field_validator,
    model_validator,
)


def _validate_finite_nested(value: Any) -> None:
    """递归检查嵌套结构中的非有限浮点数。"""

    import math

    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("all numeric values must be finite")
    if isinstance(value, Mapping):
        for item in value.values():
            _validate_finite_nested(item)
    elif isinstance(value, (list, tuple)):
        for item in value:
            _validate_finite_nested(item)


class _StrictModel(BaseModel):
    """算法参数模型基类：禁止额外字段，禁止非有限浮点数。"""

    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)

    @model_validator(mode="after")
    def _require_finite_nested(self) -> "_StrictModel":
        _validate_finite_nested(self.model_dump())
        return self


# ── 采集 ──────────────────────────────────────────────────────────────


class CaptureParameters(_StrictModel):
    """采集参数 (PRD §9.1)。"""

    duration_seconds: float = Field(0.5, ge=0.2, le=2.0)
    # These are application execution policies rather than Ranger/geometry
    # quality gates, but the pure pipeline accepts the complete Schema 5
    # snapshot used by replay and parameter-test callers.
    max_consecutive_fault_cycles: int = Field(3, ge=1, le=100)
    minimum_cycle_interval_seconds: float = Field(1.0, ge=0.1, le=60.0)
    failure_backoff_initial_seconds: float = Field(1.0, ge=0.1, le=60.0)
    failure_backoff_max_seconds: float = Field(8.0, ge=0.1, le=300.0)

    @model_validator(mode="after")
    def _require_ordered_backoff(self) -> "CaptureParameters":
        if self.failure_backoff_initial_seconds > self.failure_backoff_max_seconds:
            raise ValueError(
                "failure_backoff_initial_seconds must be <= failure_backoff_max_seconds"
            )
        return self


# ── 车辆 AABB ─────────────────────────────────────────────────────────


class VehicleAabb(_StrictModel):
    """车辆轴对齐包围盒 (PRD §9.2)。

    所有边界均为部署必填项，不提供通用默认值。"""

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

    @model_validator(mode="after")
    def _require_ordered_bounds(self) -> "VehicleAabb":
        if not (
            self.x_min < self.x_max
            and self.y_min < self.y_max
            and self.z_min < self.z_max
        ):
            raise ValueError("vehicle AABB requires min < max on every axis")
        return self


class PointCloudRange(_StrictModel):
    """clean 后、voxel 前保留的点云范围；默认关闭且边界为空。"""

    enabled: bool = False
    x_min: float | None = None
    x_max: float | None = None
    y_min: float | None = None
    y_max: float | None = None
    z_min: float | None = None
    z_max: float | None = None

    @model_validator(mode="after")
    def _require_ordered_bounds(self) -> "PointCloudRange":
        if not self.enabled:
            return self
        bounds = (
            self.x_min,
            self.x_max,
            self.y_min,
            self.y_max,
            self.z_min,
            self.z_max,
        )
        if any(value is None for value in bounds):
            raise ValueError("point cloud range requires all six bounds when enabled")
        if not (
            self.x_min < self.x_max
            and self.y_min < self.y_max
            and self.z_min < self.z_max
        ):
            raise ValueError("point cloud range requires min < max on every axis")
        return self


# ── 几何 ──────────────────────────────────────────────────────────────


class GeometryParameters(_StrictModel):
    """几何参数 (PRD §9.2)。

    heading_vector 是车辆 XY 平面中表示车头方向的非零向量。"""

    heading_vector: tuple[float, float, float] = Field(
        default_factory=lambda: (1.0, 0.0, 0.0),
        min_length=3,
        max_length=3,
    )
    vehicle_aabb: VehicleAabb

    @field_validator("heading_vector")
    @classmethod
    def _require_nonzero_xy(cls, value: tuple[float, float, float]) -> tuple[float, float, float]:
        if value[0] == 0.0 and value[1] == 0.0:
            raise ValueError("heading_vector must have a non-zero XY component")
        return value


# ── 预处理 ────────────────────────────────────────────────────────────


class StatisticalFilterParameters(_StrictModel):
    """统计离群值过滤参数 (PRD §9.3)。"""

    enabled: bool = True
    neighbors: int = Field(20, ge=5, le=100)
    std_ratio: float = Field(2.0, ge=0.5, le=5.0)


class PreprocessParameters(_StrictModel):
    """预处理参数 (PRD §9.3)。"""

    point_cloud_range: PointCloudRange = Field(default_factory=PointCloudRange)
    voxel_size_m: float = Field(0.05, ge=0.01, le=0.20)
    statistical_filter: StatisticalFilterParameters = Field(
        default_factory=StatisticalFilterParameters,
    )


class BoundaryPathParameters(_StrictModel):
    """密度无关的二维占用边界路径参数。"""

    grid_resolution_m: float = Field(0.10, ge=0.03, le=0.50)
    max_gap_m: float = Field(0.60, ge=0.10, le=3.0)
    max_step_m: float = Field(0.30, ge=0.05, le=1.0)
    min_coverage_ratio: float = Field(0.55, ge=0.20, le=1.0)
    min_path_span_m: float = Field(1.0, ge=0.30, le=10.0)
    min_vertical_span_m: float = Field(0.20, ge=0.05, le=2.0)
    max_vertical_gap_m: float = Field(0.40, ge=0.10, le=2.0)


# ── 三墙拟合与质量 ────────────────────────────────────────────────────


class FitParameters(_StrictModel):
    """三墙拟合与质量参数 (PRD §9.4)。"""

    distance_threshold_m: float = Field(0.08, ge=0.01, le=0.30)
    min_inlier_ratio: float = Field(0.60, ge=0.0, le=1.0)
    max_rmse_m: float = Field(0.05, ge=0.005, le=0.30)
    max_parallel_angle_deg: float = Field(5.0, ge=0.5, le=20.0)
    max_orthogonal_angle_deg: float = Field(5.0, ge=0.5, le=20.0)


class ProbeParameters(_StrictModel):
    """AABB-derived fixed directional probe parameters."""

    scale_ratio: float = Field(0.90, ge=0.50, le=0.95)


# ── 调试 ──────────────────────────────────────────────────────────────


class DebugParameters(_StrictModel):
    """调试参数 (PRD §9.5)。"""

    save_debug_stages: bool = False
    max_debug_cycles: int = Field(100, ge=1, le=1000)


class ResultFilterParameters(_StrictModel):
    """应用跨轮次结果滤波策略；纯单轮算法只校验并携带该快照。"""

    enabled: bool = True
    median_window: int = Field(3, ge=1, le=9)
    ema_alpha: float = Field(0.60, gt=0.0, le=1.0)

    @field_validator("median_window")
    @classmethod
    def _require_odd_window(cls, value: int) -> int:
        if value % 2 == 0:
            raise ValueError("median_window must be odd")
        return value


# ── 顶层参数树 ────────────────────────────────────────────────────────


class DetectionParameters(_StrictModel):
    """巷道距离探测完整参数树 (PRD §9)。

    使用方式：

        params = DetectionParameters(
            geometry=GeometryParameters(
                vehicle_aabb=VehicleAabb(
                    x_min=-2.0, x_max=5.0,
                    y_min=-2.5, y_max=2.5,
                    z_min=-1.5, z_max=3.0,
                ),
            ),
        )
        # 使用默认值的字段自动填充
        print(params.model_dump())

    验证失败会抛出 pydantic.ValidationError。"""

    capture: CaptureParameters = Field(default_factory=CaptureParameters)
    geometry: GeometryParameters
    preprocess: PreprocessParameters = Field(default_factory=PreprocessParameters)
    probe: ProbeParameters = Field(default_factory=ProbeParameters)
    result_filter: ResultFilterParameters = Field(default_factory=ResultFilterParameters)
    debug: DebugParameters = Field(default_factory=DebugParameters)

    @classmethod
    def build_defaults(
        cls,
        vehicle_aabb: VehicleAabb,
    ) -> "DetectionParameters":
        """以给定车辆 AABB 构建全部使用默认值的参数实例。"""

        return cls(
            geometry=GeometryParameters(vehicle_aabb=vehicle_aabb),
        )

    @classmethod
    def validate_dict(cls, data: Mapping[str, Any]) -> "DetectionParameters":
        """从字典严格校验并返回参数实例。

        比 model_validate 提供更清晰的错误信息。"""

        return cls.model_validate(dict(data))
