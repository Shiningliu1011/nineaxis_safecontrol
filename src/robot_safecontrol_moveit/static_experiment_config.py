from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping

import jsonschema
import numpy as np
import yaml

from .oscbf_trajectory import bootstrap_portable


METHODS = MappingProxyType({
    "A": ("separated", "basic"), "B": ("joint", "basic"),
    "C": ("separated", "improved"), "D": ("joint", "improved"),
    "E": ("sns", "improved"),
})

_POSITIVE = {"type": "number", "exclusiveMinimum": 0}
_RECOVERY_PROPERTIES = {
    "mode": {"enum": ["basic", "improved"]},
    **{name: _POSITIVE for name in (
        "pause_position_m", "resume_position_m", "pause_angle_rad", "resume_angle_rad",
        "confirmation_s", "timeout_s", "feed_acceleration_m_s2",
    )},
}
_CONTROL_PROPERTIES = {
    "period_s": {"enum": [.02, .04, .05, .1]},
    "solver_mode": {"enum": ["separated", "joint", "sns"]},
    "clearance_reserve_m": {"type": "number", "minimum": 0},
    "posture_joint_margin_fraction": {"type": "number", "exclusiveMinimum": 0, "maximum": 1},
    **{name: _POSITIVE for name in (
        "nominal_feed_multiplier", "cbf_alpha", "posture_weight", "smooth_weight",
        "feed_weight", "posture_speed_limit", "posture_influence_m",
    )},
    **{name: {"type": "array", "minItems": 5, "maxItems": 5, "items": _POSITIVE}
       for name in ("task_gains", "task_weights")},
    "recovery": {
        "type": "object", "properties": _RECOVERY_PROPERTIES,
        "required": list(_RECOVERY_PROPERTIES), "additionalProperties": False,
    },
}
_RESOURCES = (
    "controller_profile", "plant_profile", "trajectory_mat",
    "kinematics_config", "urdf", "mesh_directory", "collision_mesh_manifest",
)
_PROPERTIES = {
    "schema_version": {"const": 1},
    "mode": {"const": "sim"},
    "scenario": {"enum": ["S0", "S1", "S2", "S3", "S4", "S5"]},
    "control": {
        "type": "object", "properties": _CONTROL_PROPERTIES,
        "required": [name for name in _CONTROL_PROPERTIES
                     if name not in ("clearance_reserve_m", "posture_joint_margin_fraction")],
        "additionalProperties": False,
    },
    "obstacles": {
        "type": "array",
        "items": {
            "type": "object", "additionalProperties": False,
            "required": ["name", "center_m", "radius_m"],
            "properties": {
                "name": {"type": "string", "minLength": 1},
                "center_m": {
                    "type": "array", "minItems": 3, "maxItems": 3,
                    "items": {"type": "number"},
                },
                "radius_m": {"const": 0.06},
            },
        },
    },
    "path_radius_m": _POSITIVE,
    "workpiece_radius_m": _POSITIVE,
    "external_clearance_m": {"const": .03},
    "self_clearance_m": {"const": .01},
    "workpiece_axial_margin_m": _POSITIVE,
    "table_half_extents_m": {
        "type": "array", "minItems": 3, "maxItems": 3, "items": _POSITIVE,
    },
    "initial_position": {
        "type": "array", "minItems": 9, "maxItems": 9, "items": {"type": "number"},
    },
    "timeout_s": _POSITIVE,
    "endpoint_hold_s": {"type": "number", "minimum": 0.5},
    "settle_s": {"type": "number", "minimum": 0.5},
    **{name: {"type": "string", "minLength": 1} for name in _RESOURCES},
}
_SCHEMA = {
    "type": "object", "properties": {
        **_PROPERTIES, "joint_travel_scale": {"enum": [1., .7, .4]},
    },
    "required": list(_PROPERTIES), "additionalProperties": False,
}


def content_hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _freeze(value):
    if isinstance(value, dict):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, list):
        return tuple(_freeze(item) for item in value)
    return value


def _plain(value):
    if isinstance(value, Mapping):
        return {key: _plain(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_plain(item) for item in value]
    return value


@dataclass(frozen=True)
class ExperimentConfig:
    source: Path
    resource_root: Path
    values: Mapping[str, Any]
    identity: str
    overrides: Mapping[str, Any] = field(default_factory=dict)

    @classmethod
    def load(cls, path: Path, *, resource_root: Path,
             solver_mode: str | None = None, recovery_mode: str | None = None,
             method: str | None = None, period_s: float | None = None,
             feed_factor: float | None = None,
             joint_travel_scale: float | None = None,
             clearance_reserve_m: float | None = None) -> ExperimentConfig:
        source = Path(path).resolve(strict=True)
        values = yaml.safe_load(source.read_text(encoding="utf-8"))
        errors = list(jsonschema.Draft202012Validator(_SCHEMA).iter_errors(values))
        if errors:
            raise ValueError(f"{source}: {errors[0].json_path}: {errors[0].message}")
        values["control"].setdefault("posture_joint_margin_fraction", .05)
        overrides = {}
        if method is not None:
            if method not in METHODS:
                raise ValueError("method 必须为 A、B、C、D 或 E")
            expected_solver, expected_recovery = METHODS[method]
            if (solver_mode not in (None, expected_solver)
                    or recovery_mode not in (None, expected_recovery)):
                raise ValueError("method 与显式 solver_mode 或 recovery_mode 冲突")
            values["control"]["solver_mode"] = expected_solver
            values["control"]["recovery"]["mode"] = expected_recovery
            overrides["method"] = method
        if solver_mode is not None:
            values["control"]["solver_mode"] = solver_mode
            overrides["solver_mode"] = solver_mode
        if recovery_mode is not None:
            values["control"]["recovery"]["mode"] = recovery_mode
            overrides["recovery_mode"] = recovery_mode
        if period_s is not None:
            values["control"]["period_s"] = period_s
            overrides["period_s"] = period_s
        if feed_factor is not None:
            if not np.isfinite(feed_factor) or feed_factor <= 0:
                raise ValueError("feed_factor 必须为有限正数")
            values["control"]["nominal_feed_multiplier"] *= feed_factor
            overrides["feed_factor"] = feed_factor
        if joint_travel_scale is not None:
            values["joint_travel_scale"] = joint_travel_scale
            overrides["joint_travel_scale"] = joint_travel_scale
        if clearance_reserve_m is not None:
            if not np.isfinite(clearance_reserve_m) or clearance_reserve_m < 0:
                raise ValueError("clearance_reserve_m 必须为有限非负数")
            values["control"]["clearance_reserve_m"] = clearance_reserve_m
            overrides["clearance_reserve_m"] = clearance_reserve_m
        errors = list(jsonschema.Draft202012Validator(_SCHEMA).iter_errors(values))
        if errors:
            raise ValueError(f"{source}: {errors[0].json_path}: {errors[0].message}")
        encoded = json.dumps(values, sort_keys=True, allow_nan=False).encode()
        names = [item["name"] for item in values["obstacles"]]
        if len(names) != len(set(names)) or set(names) & {
            "workpiece", "table", "base_link", *[f"Link{index}" for index in range(1, 10)],
        }:
            raise ValueError("障碍物名称必须唯一且与机器人、工件、台面区分")
        if values["scenario"] == "S0" and values["obstacles"]:
            raise ValueError("S0 不包含附加障碍物")
        if values["scenario"] != "S3" and values.get("joint_travel_scale", 1.) != 1.:
            raise ValueError("只有 S3 允许收紧 J1、J4 行程")
        rules = values["control"]["recovery"]
        if (rules["resume_position_m"] >= rules["pause_position_m"]
                or rules["resume_angle_rad"] >= rules["pause_angle_rad"]
                or rules["confirmation_s"] < .2 or rules["timeout_s"] > 30):
            raise ValueError("恢复阈值与持续时间不满足实验要求")
        if not np.isclose(
            values["path_radius_m"] - values["workpiece_radius_m"], 0.05,
            rtol=0.0, atol=1e-12,
        ):
            raise ValueError("path_radius_m 与 workpiece_radius_m 必须相差 0.05 m")
        root = Path(resource_root).resolve(strict=True)
        for name in _RESOURCES:
            resource = (root / values[name]).resolve(strict=True)
            if name == "mesh_directory":
                if not resource.is_dir():
                    raise ValueError(f"{name} 必须为目录")
            elif not resource.is_file():
                raise ValueError(f"{name} 必须为文件")
        # 序列冻结后，配置身份在运行期间保持不变。
        return cls(source, root, _freeze(values), hashlib.sha256(encoded).hexdigest(), _freeze(overrides))

    @property
    def mode(self) -> str:
        return self.values["mode"]

    @property
    def path_radius_m(self) -> float:
        return float(self.values["path_radius_m"])

    @property
    def workpiece_radius_m(self) -> float:
        return float(self.values["workpiece_radius_m"])

    @property
    def initial_position(self) -> np.ndarray:
        return np.asarray(self.values["initial_position"], dtype=float)

    def resource(self, name: str) -> Path:
        return (self.resource_root / self.values[name]).resolve(strict=True)

    def controller_parameters(self) -> dict:
        return {
            "production_config_yaml": str(self.resource("controller_profile")),
            "portable_oscbf_root": str(self.resource_root / "portable_oscbf"),
            "trajectory_mat": str(self.resource("trajectory_mat")),
            "portable_config_yaml": str(self.resource("kinematics_config")),
            "path_radius_m": self.path_radius_m,
            "shared_path_reference": True,
        }

    def plant_parameters(self) -> dict:
        values = yaml.safe_load(self.resource("plant_profile").read_text())["/**"]["ros__parameters"]
        if float(values["publish_frequency_hz"]) != 500.0:
            raise ValueError("静态实验要求 publish_frequency_hz=500")
        if values["randomize_start"]:
            raise ValueError("静态实验要求固定 initial_position")
        return {
            **values, "portable_oscbf_root": str(self.resource_root / "portable_oscbf"),
            "start_position": self.initial_position.tolist(),
        }

    def execution_limits(self) -> dict:
        bootstrap_portable(self.resource_root / "portable_oscbf")
        from work.actuator_limits import load_actuator_limit_profile
        from work.nineaxis_kinematics import NineaxisKinematics

        limits = NineaxisKinematics().joint_limits
        profile = load_actuator_limit_profile()
        lower, upper = limits.q_min.copy(), limits.q_max.copy()
        if np.any(self.initial_position < lower) or np.any(self.initial_position > upper):
            raise ValueError("实验初态超出机器人关节位置限制")
        for joint in (0, 3):
            scale = self.values.get("joint_travel_scale", 1.)
            lower[joint] = self.initial_position[joint] + scale * (lower[joint] - self.initial_position[joint])
            upper[joint] = self.initial_position[joint] + scale * (upper[joint] - self.initial_position[joint])
        return {
            "lower": lower.tolist(), "upper": upper.tolist(),
            "max_velocity": profile.velocity_limits.tolist(),
            "max_acceleration": profile.acceleration_limits.tolist(),
            "max_jerk": (profile.acceleration_limits / self.plant_parameters()["jerk_time"]).tolist(),
        }

    def load_path(self):
        bootstrap_portable(self.resource_root / "portable_oscbf")
        from work.ik_data_loader import load_repository_trajectory

        profile = yaml.safe_load(self.resource("controller_profile").read_text())["/**"]["ros__parameters"]
        trajectory = load_repository_trajectory(
            str(self.resource("trajectory_mat")),
            config_yaml_path=str(self.resource("kinematics_config")),
            feedrate_scale=float(profile["reference_feedrate_scale"]) * self.values["control"]["nominal_feed_multiplier"],
        )
        trajectory.set_surface_normal_orientation(
            profile["cylinder_axis_direction"],
            axis_point=profile.get("cylinder_center") or None,
            radius_m=self.path_radius_m,
        )
        geometry = trajectory.path_geometry(require_continuous=True)
        return geometry, {
            "center_m": trajectory.surface_centre.tolist(),
            "axis": trajectory.surface_axis.tolist(),
            "path_radius_m": self.path_radius_m,
            "workpiece_radius_m": self.workpiece_radius_m,
        }

    def scene(self) -> dict:
        geometry, cylinder = self.load_path()
        from work.static_collision_meshes import mesh_sources

        axis = np.asarray(cylinder["axis"])
        if not np.allclose(axis, [0, 1, 0], atol=1e-12):
            raise ValueError("静态实验场景要求 base_link 的 +Y 圆柱轴")
        coordinates = geometry.positions_m @ axis
        margin = self.values["workpiece_axial_margin_m"]
        bottom, top = float(coordinates.min() - margin), float(coordinates.max() + margin)
        center = np.asarray(cylinder["center_m"])
        center += (0.5 * (bottom + top) - center @ axis) * axis
        table_center = center + (bottom - self.values["table_half_extents_m"][1] - center @ axis) * axis
        return {
            "frame_id": "base_link", "config_sha256": self.identity,
            "workpiece": {
                "center_m": center.tolist(), "axis": axis.tolist(),
                "radius_m": self.workpiece_radius_m, "height_m": top - bottom,
            },
            "table": {
                "center_m": table_center.tolist(),
                "half_extents_m": list(self.values["table_half_extents_m"]),
            },
            "obstacles": _plain(self.values["obstacles"]),
            "clearances": {
                "external_m": self.values["external_clearance_m"],
                "self_m": self.values["self_clearance_m"],
            },
            "self_collision": mesh_sources(self.resource("collision_mesh_manifest")),
        }

    def manifest(self) -> dict:
        scene = self.scene()
        files = {name: self.resource(name) for name in _RESOURCES if name != "mesh_directory"}
        files["actuator_profile"] = self.resource_root / "portable_oscbf/config/actuator_modules.yaml"
        files["experiment_configuration"] = self.source
        files.update({
            f"mesh/{path.name}": path
            for path in sorted(self.resource("mesh_directory").glob("*.STL"))
        })
        mesh_source = scene["self_collision"]
        files.update({
            f"self_collision/{index}": Path(record["path"])
            for index, record in enumerate([mesh_source["manifest"], *mesh_source["auxiliary"], *mesh_source["meshes"]])
        })
        return {
            "config": _plain(self.values), "config_sha256": self.identity,
            "resource_root": str(self.resource_root),
            "overrides": _plain(self.overrides),
            "execution_limits": self.execution_limits(),
            "resources": {
                name: {"path": str(path), "sha256": content_hash(path)}
                for name, path in files.items()
            },
            "scene": scene,
        }
