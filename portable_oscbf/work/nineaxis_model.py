from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping
from dataclasses import dataclass, field, replace
from pathlib import Path
from types import MappingProxyType
from typing import Any, NamedTuple
from urllib.parse import urlparse

import jax
import jax.numpy as jnp
import jsonschema
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from srdfdom.srdf import SRDF
from urdf_parser_py.urdf import URDF, Mesh

from work.actuator_limits import JOINT_NAMES
from work.kinematics_data import JOINT_CHAIN, JOINT_POSITION_LOWER, JOINT_POSITION_UPPER
from work.nineaxis_manipulator_jax import (
    NineaxisManipulatorJAX,
    ee_transform_fn,
    full_jacobian_fn,
)

_ROOT = Path(__file__).resolve().parents[2]
_SOURCE_SPEC = {
    "type": "object", "additionalProperties": False,
    "required": ["path", "sha256"],
    "properties": {
        "path": {"type": "string", "minLength": 1},
        "sha256": {"type": "string", "pattern": "^[0-9a-f]{64}$"},
    },
}
_TEXT = {"type": "string", "minLength": 1}
_MODEL_PROPERTIES = {
    "schema_version": {"type": "integer", "const": 1},
    "numeric_dtype": {"const": "float64"},
    **{name: _TEXT for name in ("revision", "model_id", "base_frame", "base_convention", "tcp_frame")},
    **{name: {"type": "array", "minItems": 9, "maxItems": 9, "items": _TEXT}
       for name in ("joint_names", "joint_types", "joint_units")},
    "tool_axis_local": {"type": "array", "minItems": 3, "maxItems": 3, "items": {"type": "number"}},
    "sources": {
        "type": "object", "additionalProperties": False,
        "required": ["urdf", "srdf", "collision_mesh_manifest", "generated_kinematics"],
        "properties": {name: _SOURCE_SPEC for name in (
            "urdf", "srdf", "collision_mesh_manifest", "generated_kinematics")},
    },
    "nominal_tcp": {
        "type": "object", "additionalProperties": False,
        "required": ["revision", "source", "verification_state", "matrix", "translation_unit"],
        "properties": {
            **{name: _TEXT for name in ("revision", "source", "verification_state")},
            "translation_unit": {"const": "m"},
            "matrix": {"type": "array", "minItems": 4, "maxItems": 4, "items": {
                "type": "array", "minItems": 4, "maxItems": 4, "items": {"type": "number"}}},
        },
    },
    "missing": {
        "type": "object", "additionalProperties": False,
        "required": ["workpiece", "joint_mapping", "physical_tcp", "tool_collision_geometry"],
        "properties": {name: _TEXT for name in (
            "workpiece", "joint_mapping", "physical_tcp", "tool_collision_geometry")},
    },
}


def immutable_array(value: Any, shape: tuple[int, ...], name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != shape or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} 需要形状 {shape} 的有限数值")
    # bytes 保存底层数据，调用者不能重新开启 writeable。
    return np.frombuffer(array.tobytes(), dtype=np.float64).reshape(shape)


def content_identity(value: Any) -> str:
    return hashlib.sha256(json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False,
    ).encode("utf-8")).hexdigest()


def _asset_model_identity(revision: str, model_id: str, config_hash: str,
                          asset_hashes: tuple[tuple[str, str], ...]) -> str:
    return content_identity({"revision": revision, "model_id": model_id,
                             "config_hash": config_hash, "assets": asset_hashes})


def require_sha256(value: str, name: str) -> None:
    if (not isinstance(value, str) or len(value) != 64
            or any(character not in "0123456789abcdef" for character in value)):
        raise ValueError(f"{name} 需要 SHA-256 身份")


def source_digest(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _se3(value: Any, name: str) -> np.ndarray:
    matrix = immutable_array(value, (4, 4), name)
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-12, rtol=0)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-12, rtol=0)
            or abs(np.linalg.det(rotation) - 1) > 1e-12):
        raise ValueError(f"{name} 必须属于 SE(3)，长度单位为 m")
    return matrix


class ModelKinematicParameters(NamedTuple):
    screw_axes: jax.Array
    tcp_zero: jax.Array


@jax.jit
def tool_motion_jax(q, q_dot, q_ddot, q_jerk, screw_axes, tcp_zero):
    def transform(position):
        return ee_transform_fn(position, screw_axes, tcp_zero)

    def first(position, velocity):
        return jax.jvp(transform, (position,), (velocity,))[1]

    def second(position, velocity, acceleration):
        return jax.jvp(first, (position, velocity), (velocity, acceleration))[1]

    value, velocity = jax.jvp(transform, (q,), (q_dot,))
    acceleration = second(q, q_dot, q_ddot)
    jerk = jax.jvp(second, (q, q_dot, q_ddot), (q_dot, q_ddot, q_jerk))[1]
    matrices = jnp.stack((value, velocity, acceleration, jerk))
    precision = jax.config.read("jax_enable_x64") and all(value.dtype == jnp.dtype("float64")
        for value in (q, q_dot, q_ddot, q_jerk, screw_axes, tcp_zero))
    return matrices, jnp.all(jnp.isfinite(matrices)) & precision


@dataclass(frozen=True)
class ToolMotion:
    model_hash: str
    sample_source: str
    time_s: float
    clock_id: str
    joint_derivatives: np.ndarray
    transform_derivatives: np.ndarray
    angular_derivatives: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        require_sha256(self.model_hash, "model_hash")
        if self.sample_source not in ("simulation_feedback", "measured_feedback", "generated_motion"):
            raise ValueError("sample_source 需要 simulation_feedback、measured_feedback 或 generated_motion")
        if not np.isfinite(self.time_s) or self.time_s < 0:
            raise ValueError("time_s 需要有限非负值")
        if not isinstance(self.clock_id, str) or not self.clock_id:
            raise ValueError("clock_id 需要明确时间基准")
        object.__setattr__(self, "joint_derivatives", immutable_array(self.joint_derivatives, (4, 9), "joint_derivatives"))
        object.__setattr__(self, "transform_derivatives", immutable_array(self.transform_derivatives, (4, 4, 4), "transform_derivatives"))
        rotation, first, second, third = self.transform_derivatives[:, :3, :3]
        matrices = (first @ rotation.T, second @ rotation.T + first @ first.T,
                    third @ rotation.T + 2 * second @ first.T + first @ second.T)
        angular = [[(matrix[2, 1] - matrix[1, 2]) / 2, (matrix[0, 2] - matrix[2, 0]) / 2,
                    (matrix[1, 0] - matrix[0, 1]) / 2] for matrix in matrices]
        object.__setattr__(self, "angular_derivatives", immutable_array(angular, (3, 3), "angular_derivatives"))

    @property
    def position_m(self) -> np.ndarray:
        return self.transform_derivatives[0, :3, 3]

    @property
    def velocity_m_s(self) -> np.ndarray:
        return self.transform_derivatives[1, :3, 3]

    @property
    def acceleration_m_s2(self) -> np.ndarray:
        return self.transform_derivatives[2, :3, 3]

    @property
    def jerk_m_s3(self) -> np.ndarray:
        return self.transform_derivatives[3, :3, 3]

    @property
    def axis_derivatives(self) -> np.ndarray:
        return self.transform_derivatives[:, :3, 0]


@dataclass(frozen=True)
class RigidTransform:
    source_frame: str
    target_frame: str
    matrix: np.ndarray
    revision: str
    source: str
    verification_state: str
    applicable_model_hash: str = field(kw_only=True)
    position_residual_mm: float | None = None
    angle_residual_deg: float | None = None
    translation_unit: str = "m"

    def __post_init__(self) -> None:
        require_sha256(self.applicable_model_hash, "applicable_model_hash")
        if self.translation_unit != "m":
            raise ValueError("translation_unit 需要 m")
        for name in ("source_frame", "target_frame", "revision", "source"):
            if not isinstance(getattr(self, name), str) or not getattr(self, name):
                raise ValueError(f"{name} 需要明确内容")
        if self.verification_state not in ("simulation", "measured_verified"):
            raise ValueError("verification_state 需要 simulation 或 measured_verified")
        object.__setattr__(self, "matrix", _se3(self.matrix, "matrix"))
        for name in ("position_residual_mm", "angle_residual_deg"):
            value = getattr(self, name)
            if value is not None and (not np.isfinite(value) or value < 0):
                raise ValueError(f"{name} 需要有限非负值")
            if self.verification_state == "measured_verified" and value is None:
                raise ValueError(f"measured_verified 缺少 {name}")

    def to_record(self) -> dict[str, Any]:
        return {
            "source_frame": self.source_frame, "target_frame": self.target_frame,
            "matrix_m": self.matrix.tolist(), "revision": self.revision, "source": self.source,
            "verification_state": self.verification_state,
            "position_residual_mm": self.position_residual_mm,
            "angle_residual_deg": self.angle_residual_deg,
            "translation_unit": self.translation_unit,
            "applicable_model_hash": self.applicable_model_hash,
        }

    @property
    def identity(self) -> str:
        return content_identity(self.to_record())


@dataclass(frozen=True)
class ModelCalibration:
    revision: str
    tcp_correction: RigidTransform
    workpiece: RigidTransform | None
    joint_mapping_evidence: str | None
    applicable_model_hash: str = field(kw_only=True)

    def __post_init__(self) -> None:
        require_sha256(self.applicable_model_hash, "applicable_model_hash")
        if not isinstance(self.revision, str) or not self.revision:
            raise ValueError("calibration revision 需要明确内容")
        if (self.tcp_correction.source_frame, self.tcp_correction.target_frame) != ("tcp", "tool0"):
            raise ValueError("tcp_correction 需要 T_tool0_tcp")
        if self.workpiece is not None and (
                self.workpiece.source_frame, self.workpiece.target_frame) != ("workpiece", "base_link"):
            raise ValueError("workpiece 需要 T_base_workpiece")
        if self.joint_mapping_evidence is not None and (
                not isinstance(self.joint_mapping_evidence, str) or not self.joint_mapping_evidence):
            raise ValueError("joint_mapping_evidence 需要明确来源")
        if self.tcp_correction.applicable_model_hash != self.applicable_model_hash:
            raise ValueError("tcp_correction applicable_model_hash 与标定记录不符")
        if self.workpiece is not None and self.workpiece.applicable_model_hash != self.applicable_model_hash:
            raise ValueError("workpiece applicable_model_hash 与标定记录不符")

    def to_record(self) -> dict[str, Any]:
        return {
            "revision": self.revision, "tcp_correction": self.tcp_correction.to_record(),
            "workpiece": None if self.workpiece is None else self.workpiece.to_record(),
            "joint_mapping_evidence": self.joint_mapping_evidence,
            "applicable_model_hash": self.applicable_model_hash,
        }


@dataclass(frozen=True)
class MeshSource:
    link: str
    original_path: Path
    closed_path: Path
    original_hash: str
    closed_hash: str
    local_transform: np.ndarray

    def __post_init__(self) -> None:
        require_sha256(self.original_hash, "original_hash")
        require_sha256(self.closed_hash, "closed_hash")
        object.__setattr__(self, "local_transform", _se3(self.local_transform, "mesh local_transform"))


@dataclass(frozen=True)
class NineaxisModel:
    root: Path
    revision: str
    model_id: str
    config_hash: str
    asset_hashes: tuple[tuple[str, str], ...]
    mesh_sources: tuple[MeshSource, ...]
    self_collision_exclusions: tuple[tuple[str, str], ...]
    calibration: ModelCalibration
    missing_sources: Mapping[str, str]
    _robot: NineaxisManipulatorJAX = field(repr=False, compare=False)
    asset_model_hash: str = field(init=False)
    model_hash: str = field(init=False)
    geometry_hash: str = field(init=False)
    position_lower: np.ndarray = field(init=False, repr=False)
    position_upper: np.ndarray = field(init=False, repr=False)
    tool_axis_local: np.ndarray = field(init=False, repr=False)
    _m_tcp: jax.Array = field(init=False, repr=False, compare=False)

    joint_names = JOINT_NAMES
    joint_types = ("prismatic",) + ("revolute",) * 8
    joint_units = ("m",) + ("rad",) * 8
    base_frame = "base_link"
    tcp_frame = "tool0"

    def __post_init__(self) -> None:
        if not jax.config.read("jax_enable_x64"):
            raise RuntimeError("共同模型需要启用 JAX float64")
        object.__setattr__(self, "asset_hashes", tuple(tuple(row) for row in self.asset_hashes))
        object.__setattr__(self, "missing_sources", MappingProxyType(dict(self.missing_sources)))
        object.__setattr__(self, "position_lower", immutable_array(JOINT_POSITION_LOWER, (9,), "position_lower"))
        object.__setattr__(self, "position_upper", immutable_array(JOINT_POSITION_UPPER, (9,), "position_upper"))
        object.__setattr__(self, "tool_axis_local", immutable_array([1, 0, 0], (3,), "tool_axis_local"))
        asset_identity = _asset_model_identity(self.revision, self.model_id, self.config_hash, self.asset_hashes)
        if self.calibration.applicable_model_hash != asset_identity:
            raise ValueError("calibration applicable_model_hash 与当前模型资产身份不符")
        object.__setattr__(self, "asset_model_hash", asset_identity)
        object.__setattr__(self, "model_hash", content_identity({
            "asset_model_hash": asset_identity, "calibration": self.calibration.to_record(),
        }))
        object.__setattr__(self, "geometry_hash", content_identity({
            "model_hash": self.model_hash, "exclusions": self.self_collision_exclusions,
            "tool_collision_geometry": self.missing_sources["tool_collision_geometry"],
        }))
        object.__setattr__(self, "_m_tcp", self._robot.M_zero @ jnp.asarray(self.calibration.tcp_correction.matrix))

    @classmethod
    def load(cls, *, root: Path = _ROOT, config_path: Path | None = None,
             calibration: ModelCalibration | None = None, expected_hash: str | None = None) -> NineaxisModel:
        jax.config.update("jax_enable_x64", True)
        root = Path(root).resolve(strict=True)
        path = root / "portable_oscbf/config/nineaxis_model.yaml" if config_path is None else Path(config_path)
        values = yaml.safe_load(path.read_text(encoding="utf-8"))
        jsonschema.validate(values, {
            "type": "object", "properties": _MODEL_PROPERTIES,
            "required": list(_MODEL_PROPERTIES), "additionalProperties": False,
        })
        if (tuple(values["joint_names"]) != cls.joint_names
                or tuple(values["joint_types"]) != cls.joint_types
                or tuple(values["joint_units"]) != cls.joint_units
                or values["base_frame"] != cls.base_frame or values["tcp_frame"] != cls.tcp_frame
                or values["base_convention"] != "Y_up_J1_positive_Z"
                or values["tool_axis_local"] != [1, 0, 0]):
            raise ValueError("九轴 joint_names/types/units/base_frame/TCP/tool_axis 定义不符")
        assets, sources = [], {}
        for name, source in values["sources"].items():
            source_path = (root / source["path"]).resolve(strict=True)
            source_path.relative_to(root)
            digest = source_digest(source_path)
            if digest != source["sha256"]:
                raise ValueError(f"{name} source hash 不符：{source_path}")
            assets.append((source["path"], digest))
            sources[name] = source_path
        description = URDF.from_xml_string(sources["urdf"].read_bytes())
        cls._verify_chain(description)
        semantic = SRDF.from_xml_string(sources["srdf"].read_bytes())
        exclusions = tuple((row.link1, row.link2) for row in semantic.disable_collisionss)
        manifest_path = sources["collision_mesh_manifest"]
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest["schema_version"] != 1 or manifest["source_urdf_sha256"] != values["sources"]["urdf"]["sha256"]:
            raise ValueError("collision mesh manifest 来源或 schema_version 不符")
        meshes = []
        for row in manifest["links"]:
            original = (manifest_path.parent / row["source_mesh"]).resolve(strict=True)
            closed = (manifest_path.parent / row["closed_mesh"]).resolve(strict=True)
            for mesh_path, hash_key in ((original, "source_mesh_sha256"), (closed, "closed_mesh_sha256")):
                relative = mesh_path.relative_to(root).as_posix()
                digest = source_digest(mesh_path)
                if digest != row[hash_key]:
                    raise ValueError(f"mesh source hash 不符：{mesh_path}")
                assets.append((relative, digest))
            link = description.link_map[row["link"]]
            if len(link.collisions) != 1 or not isinstance(link.collisions[0].geometry, Mesh):
                raise ValueError(f"{link.name} 需要现有 URDF mesh collision")
            collision = link.collisions[0]
            uri = urlparse(collision.geometry.filename)
            mesh_file = (root / "models" / uri.netloc / uri.path.lstrip("/")).resolve(strict=True)
            if uri.scheme != "package" or mesh_file != original or collision.geometry.scale not in (None, [1, 1, 1]):
                raise ValueError(f"{link.name} mesh identity/scale 不符")
            local = np.eye(4)
            if collision.origin is not None:
                local[:3, :3] = Rotation.from_euler("xyz", collision.origin.rpy).as_matrix()
                local[:3, 3] = collision.origin.xyz
            meshes.append(MeshSource(row["link"], original, closed, row["source_mesh_sha256"], row["closed_mesh_sha256"], local))
        if tuple(mesh.link for mesh in meshes) != ("base_link",) + tuple(f"Link{index}" for index in range(1, 10)):
            raise ValueError("mesh manifest 需要覆盖全部具名连杆")
        config_hash, asset_hashes = content_identity(values), tuple(sorted(assets))
        asset_identity = _asset_model_identity(values["revision"], values["model_id"], config_hash, asset_hashes)
        if calibration is None:
            nominal = values["nominal_tcp"]
            calibration = ModelCalibration("nominal-simulation-v1", RigidTransform(
                "tcp", "tool0", nominal["matrix"], nominal["revision"], nominal["source"], nominal["verification_state"],
                applicable_model_hash=asset_identity,
            ), None, None, applicable_model_hash=asset_identity)
        result = cls(root, values["revision"], values["model_id"], config_hash,
                     asset_hashes, tuple(meshes), exclusions, calibration,
                     values["missing"], NineaxisManipulatorJAX())
        if expected_hash is not None and expected_hash != result.model_hash:
            raise ValueError("expected_hash 与当前模型不符")
        return result

    @classmethod
    def _verify_chain(cls, description: URDF) -> None:
        names = description.get_chain("base_link", "tool0", joints=True, links=False, fixed=True)
        if len(names) != len(JOINT_CHAIN) - 1:
            raise ValueError("URDF 与生成关节链长度不符")
        active_names: list[str] = []
        for name, row in zip(names, JOINT_CHAIN[1:]):
            joint = description.joint_map[name]
            parent, child, kind = row[:3]
            expected_child = "tool0" if child == "ee_link" else child
            if (joint.parent, joint.child, joint.joint_type) != (parent, expected_child, kind):
                raise ValueError(f"{name} URDF 与生成链身份不符")
            if (not np.allclose(joint.origin.xyz, row[3:6], atol=1e-15, rtol=0)
                    or not np.allclose(joint.origin.rpy, row[6:9], atol=1e-15, rtol=0)):
                raise ValueError(f"{name} URDF 与生成链 origin 不符")
            if kind != "fixed":
                index = len(active_names)
                if (not np.allclose(joint.axis, row[9], atol=1e-15, rtol=0)
                        or joint.limit.lower != JOINT_POSITION_LOWER[index]
                        or joint.limit.upper != JOINT_POSITION_UPPER[index]):
                    raise ValueError(f"{name} URDF 与生成链 axis/position 不符")
                active_names.append(name)
        if tuple(active_names) != cls.joint_names:
            raise ValueError("URDF joint_names 顺序不符")

    @property
    def missing_calibrations(self) -> tuple[str, ...]:
        missing = []
        if self.calibration.workpiece is None or self.calibration.workpiece.verification_state != "measured_verified":
            missing.append("workpiece")
        if self.calibration.joint_mapping_evidence is None:
            missing.append("joint_mapping")
        if self.calibration.tcp_correction.verification_state != "measured_verified":
            missing.append("physical_tcp")
        return tuple(missing)

    @property
    def hardware_validated(self) -> bool:
        return not self.missing_hardware_sources

    @property
    def missing_hardware_sources(self) -> tuple[str, ...]:
        return self.missing_calibrations + ("tool_collision_geometry",)

    def require_hardware_validated(self) -> None:
        if self.missing_hardware_sources:
            raise RuntimeError("模型现场验证缺项：" + ", ".join(self.missing_hardware_sources))

    def with_calibration(self, calibration: ModelCalibration) -> NineaxisModel:
        return replace(self, calibration=calibration)

    @property
    def jax_parameters(self) -> ModelKinematicParameters:
        return ModelKinematicParameters(self._robot.S_axes, self._m_tcp)

    def _position(self, q: Any, *, units: str = "m_rad", model_hash: str | None = None,
                  joint_names: tuple[str, ...] | None = None) -> np.ndarray:
        if (not jax.config.read("jax_enable_x64") or self._robot.S_axes.dtype != jnp.dtype("float64")
                or self._m_tcp.dtype != jnp.dtype("float64")):
            raise RuntimeError("共同模型求值需要保持 JAX float64")
        if units != "m_rad":
            raise ValueError("units 必须为 m_rad")
        if model_hash is not None and model_hash != self.model_hash:
            raise ValueError("model_hash 与当前模型不符")
        if joint_names is not None and tuple(joint_names) != self.joint_names:
            raise ValueError("joint_names 与当前模型顺序不符")
        return immutable_array(q, (9,), "q")

    def fk(self, q: Any, **identity: Any) -> np.ndarray:
        position = self._position(q, **identity)
        return immutable_array(ee_transform_fn(jnp.asarray(position), self._robot.S_axes, self._m_tcp), (4, 4), "TCP transform")

    def jacobian(self, q: Any, **identity: Any) -> np.ndarray:
        position = self._position(q, **identity)
        return immutable_array(full_jacobian_fn(jnp.asarray(position), self._robot.S_axes, self._m_tcp), (6, 9), "Jacobian")

    def tool_axis_jacobian(self, q: Any, **identity: Any) -> np.ndarray:
        position = self._position(q, **identity)
        axis, angular = self.fk(position)[:3, 0], self.jacobian(position)[3:]
        return immutable_array(np.cross(angular.T, axis).T, (3, 9), "tool axis Jacobian")

    def link_poses(self, q: Any, **identity: Any) -> Mapping[str, np.ndarray]:
        position = self._position(q, **identity)
        transforms = np.asarray(self._robot._compute_all_link_transforms(jnp.asarray(position)))
        result = {("tool0" if row[1] == "ee_link" else row[1]): immutable_array(transform, (4, 4), row[1])
                  for row, transform in zip(JOINT_CHAIN, transforms)}
        result["tcp"] = self.fk(position)
        return MappingProxyType(result)

    def source_pose_to_base(self, position_m: Any, source_rotation: Any) -> np.ndarray:
        if self.calibration.workpiece is None:
            raise RuntimeError("workpiece transform 未配置")
        source = np.eye(4)
        source[:3, :3] = immutable_array(source_rotation, (3, 3), "source_rotation")
        source[:3, 3] = immutable_array(position_m, (3,), "source_position_m")
        source = _se3(source, "source pose")
        conversion = np.array([[0., 0., -1.], [0., 1., 0.], [1., 0., 0.]])
        result = self.calibration.workpiece.matrix @ source
        result[:3, :3] = result[:3, :3] @ conversion
        return immutable_array(result, (4, 4), "source pose base")

    def tool_motion(self, q: Any, q_dot: Any, q_ddot: Any, q_jerk: Any, *, sample_source: str,
                    time_s: float, clock_id: str, **identity: Any) -> ToolMotion:
        derivatives = np.stack((self._position(q, **identity), immutable_array(q_dot, (9,), "q_dot"),
                                immutable_array(q_ddot, (9,), "q_ddot"), immutable_array(q_jerk, (9,), "q_jerk")))
        matrices, finite = tool_motion_jax(*map(jnp.asarray, derivatives), self._robot.S_axes, self._m_tcp)
        if not bool(finite):
            raise ValueError("工具运动产生非有限值")
        return ToolMotion(self.model_hash, sample_source, time_s, clock_id, derivatives, matrices)
