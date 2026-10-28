from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
import trimesh
from jsonschema import Draft202012Validator

from work.collision_parameters import CollisionParameterArtifact, CollisionPolicy
from work.collision_safety import CollisionIdentities, CollisionSafety, CollisionSafetyConfig
from work.fixed_environment import FixedEnvironmentModel
from work.lidar_occupancy import OccupancyGrid
from work.lidar_self_filter import ClockMapping, GeometryValidation, MeshRaySelfFilter, RobotRayGeometry, SelfFilterBounds
from work.shared_collision_scene import SharedSceneReader, SharedSceneWriter

from robot_safecontrol_moveit.calibration_record import load_calibration_record
from robot_safecontrol_moveit.collision_scene_pipeline import CollisionScenePipeline
from robot_safecontrol_moveit.lidar_self_filter_adapter import calibration_for_self_filter


SOURCE_SCHEMA = {
    "type": "object", "additionalProperties": False,
    "required": ["schema_version", "parameter_artifact", "device", "scenario", "geometry_artifact",
                 "kernel_version", "ray_geometry", "calibration_record", "lidar_clock_mapping",
                 "joint_clock_mapping", "grid", "history", "max_range_m", "fixed_environment"],
    "properties": {
        "schema_version": {"const": 1, "type": "integer"},
        **{name: {"type": "string", "minLength": 1} for name in (
            "parameter_artifact", "device", "scenario", "geometry_artifact", "calibration_record")},
        "kernel_version": {"type": "string", "pattern": "^[0-9a-f]{64}$"},
        "ray_geometry": {
            "type": "object", "additionalProperties": False, "required": ["validation"],
            "properties": {
                "urdf": {"type": "string", "minLength": 1},
                "package_roots": {"type": "object", "additionalProperties": {"type": "string", "minLength": 1}},
                "source_id": {"type": "string", "minLength": 1},
                "link_meshes": {"type": "array", "minItems": 1, "items": {
                    "type": "object", "required": ["link", "path"], "additionalProperties": False,
                    "properties": {name: {"type": "string", "minLength": 1} for name in ("link", "path")}}},
                "validation": {"type": "object"},
            },
            "oneOf": [
                {"required": ["urdf", "package_roots"], "not": {"required": ["link_meshes"]}},
                {"required": ["link_meshes", "source_id"], "not": {"required": ["urdf"]}},
            ],
        },
        "lidar_clock_mapping": {"type": "object"}, "joint_clock_mapping": {"type": "object"},
        "grid": {"type": "object", "additionalProperties": False,
                 "required": ["origin_m", "shape", "required_mask"], "properties": {
                     "origin_m": {"type": "array", "minItems": 3, "maxItems": 3, "items": {"type": "number"}},
                     "shape": {"type": "array", "minItems": 3, "maxItems": 3,
                               "items": {"type": "integer", "minimum": 1}},
                     "required_mask": {"type": ["string", "null"], "minLength": 1}}},
        "history": {"type": "object", "additionalProperties": False,
                    "required": ["feedback_source", "zero_calibration_id", "capacity", "joint_speed_bound_si",
                                 "maximum_gap_ns", "evidence_id"], "properties": {
                        "feedback_source": {"enum": ["simulation", "encoder"]},
                        "zero_calibration_id": {"type": "string", "minLength": 1},
                        "capacity": {"type": "integer", "minimum": 2},
                        "joint_speed_bound_si": {"type": "array", "minItems": 9, "maxItems": 9,
                                                 "items": {"type": "number", "minimum": 0}},
                        "maximum_gap_ns": {"type": "integer", "minimum": 1},
                        "evidence_id": {"type": "string", "minLength": 1}}},
        "max_range_m": {"type": "number", "exclusiveMinimum": 0},
        "fixed_environment": {"type": ["object", "null"], "additionalProperties": False,
                              "required": ["mesh_path", "mesh_sha256", "model_revision", "validation_evidence", "error_mm"],
                              "properties": {
                                  "mesh_path": {"type": "string", "minLength": 1},
                                  "mesh_sha256": {"type": "string", "pattern": "^[0-9a-f]{64}$"},
                                  "model_revision": {"type": "integer", "minimum": 1},
                                  "validation_evidence": {"type": "string", "minLength": 1},
                                  "error_mm": {"type": "number", "minimum": 0}}},
    },
}


@dataclass(frozen=True)
class CollisionSceneSources:
    manifest_path: Path
    manifest_hash: str
    safety: CollisionSafety
    filter_module: MeshRaySelfFilter
    grid: OccupancyGrid
    lidar_mapping: ClockMapping
    joint_mapping: ClockMapping
    feedback_source: str
    zero_calibration_id: str
    history_capacity: int
    max_range_m: float
    fixed_environment: FixedEnvironmentModel | None

    @property
    def source_identity(self):
        config = self.safety.config
        geometry, calibration = self.filter_module.geometry, self.filter_module.calibration
        document = {
            "geometry_hash": config.geometry_hash.hex(), "kernel_version": config.kernel_version.hex(),
            "collision_policy_hash": config.collision_policy_hash.hex(),
            "device": config.policy.artifact.device, "scenario": config.policy.artifact.scenario,
            "ray_geometry_hash": geometry.geometry_hash, "ray_validation": asdict(geometry.validation),
            "calibration_id": calibration.calibration_id, "T_base_lidar": calibration.T_base_lidar.tolist(),
            "frame_from": calibration.frame_from, "frame_to": calibration.frame_to,
            "calibration_admission_ready": calibration.admission_ready,
            "self_filter_bounds": self.filter_module.bounds.identity,
            "lidar_clock_mapping": self.lidar_mapping.identity, "joint_clock_mapping": self.joint_mapping.identity,
            "required_space_hash": self.grid.required_space_hash.hex(),
            "feedback_source": self.feedback_source, "zero_calibration_id": self.zero_calibration_id,
            "history_capacity": self.history_capacity, "max_range_m": self.max_range_m,
            "fixed_environment_model_hash": None if self.fixed_environment is None else self.fixed_environment.model_hash.hex(),
        }
        return hashlib.sha256(json.dumps(document, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).digest()

    def create_reader(self, shared_path, *, slot_timeout_ns):
        config = self.safety.config
        identities = CollisionIdentities(
            np.zeros(16, "u1"), np.int64(0), np.frombuffer(config.geometry_hash, "u1"),
            np.frombuffer(config.kernel_version, "u1"), np.frombuffer(config.collision_policy_hash, "u1"),
            np.int64(0 if self.fixed_environment is None else self.fixed_environment.model_revision),
            np.frombuffer(self.grid.required_space_hash, "u1"))
        return SharedSceneReader(
            shared_path, self.safety, identities, slot_timeout_ns=slot_timeout_ns,
            source_identity=self.source_identity,
            fixed_environment_model_hash=None if self.fixed_environment is None else self.fixed_environment.model_hash)

    def create_pipeline(self, shared_path):
        writer = SharedSceneWriter(shared_path, self.safety, source_identity=self.source_identity)
        try:
            pipeline = CollisionScenePipeline(
                writer, self.filter_module, self.grid, self.lidar_mapping, joint_mapping=self.joint_mapping,
                feedback_source=self.feedback_source, zero_calibration_id=self.zero_calibration_id,
                history_capacity=self.history_capacity, max_range_m=self.max_range_m)
            if self.fixed_environment is not None:
                pipeline.set_fixed_environment(self.fixed_environment)
            return pipeline
        except BaseException:
            writer.close()
            raise


def load_scene_sources(path):
    source = Path(path).resolve(strict=True)
    raw = source.read_bytes()
    document = json.loads(raw)
    Draft202012Validator(SOURCE_SCHEMA).validate(document)

    def resource(value):
        return (source.parent / value).resolve(strict=True)

    artifact = CollisionParameterArtifact.load(resource(document["parameter_artifact"]),
                                               device=document["device"], scenario=document["scenario"])
    geometry_document = json.loads(resource(document["geometry_artifact"]).read_bytes())
    policy = CollisionPolicy(artifact, bytes.fromhex(geometry_document["geometry_hash"]),
                             bytes.fromhex(document["kernel_version"]),
                             tuple(geometry_document["self_collision"]["allowed_contacts"]))
    safety = CollisionSafety(CollisionSafetyConfig.from_policy(policy), geometry_artifact=geometry_document)
    ray = document["ray_geometry"]
    if "urdf" in ray:
        geometry = RobotRayGeometry.from_urdf(resource(ray["urdf"]),
                                              package_roots={name: resource(value) for name, value in ray["package_roots"].items()})
    else:
        geometry = RobotRayGeometry.from_link_meshes(
            [(entry["link"], trimesh.load_mesh(resource(entry["path"]), process=False)) for entry in ray["link_meshes"]],
            source_id=ray["source_id"])
    geometry = geometry.with_validation(GeometryValidation(**ray["validation"]))
    calibration = calibration_for_self_filter(load_calibration_record(explicit=resource(document["calibration_record"])))
    if not geometry.admission_ready or not calibration.admission_ready:
        raise ValueError("场景服务需要可准入的原始 mesh 几何与 CalibrationRecord")
    history = document["history"]
    bounds = SelfFilterBounds.from_collision_policy(
        policy, joint_speed_bound_si=history["joint_speed_bound_si"], maximum_gap_ns=history["maximum_gap_ns"],
        history_evidence_id=history["evidence_id"])
    filter_module = MeshRaySelfFilter(geometry, calibration, bounds)
    grid = document["grid"]
    required = None if grid["required_mask"] is None else np.load(resource(grid["required_mask"]), allow_pickle=False)
    grid = OccupancyGrid(np.asarray(grid["origin_m"]), tuple(grid["shape"]),
                         artifact.parameters["voxel_size_mm"]["value"], required)
    lidar_mapping = ClockMapping(**document["lidar_clock_mapping"])
    joint_mapping = ClockMapping(**document["joint_clock_mapping"])
    if lidar_mapping.run_id != joint_mapping.run_id or lidar_mapping.target_clock_id != joint_mapping.target_clock_id:
        raise ValueError("LiDAR 与反馈映射必须使用共同 run 和目标时钟")
    fixed = document["fixed_environment"]
    fixed_model = None
    if fixed is not None:
        mesh_path = resource(fixed["mesh_path"])
        mesh_hash = hashlib.sha256(mesh_path.read_bytes()).hexdigest()
        if mesh_hash != fixed["mesh_sha256"]:
            raise ValueError("固定环境 mesh_sha256 与来源文件不一致")
        fixed_model = FixedEnvironmentModel.from_mesh(
            trimesh.load_mesh(mesh_path, process=False), policy=policy, model_revision=fixed["model_revision"],
            source_hash=bytes.fromhex(mesh_hash), validation_evidence=fixed["validation_evidence"], error_mm=fixed["error_mm"])
    return CollisionSceneSources(source, hashlib.sha256(raw).hexdigest(), safety, filter_module, grid,
                                  lidar_mapping, joint_mapping, history["feedback_source"], history["zero_calibration_id"],
                                  history["capacity"], document["max_range_m"], fixed_model)
