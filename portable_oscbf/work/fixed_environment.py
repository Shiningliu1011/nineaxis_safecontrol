from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass

import jax
import numpy as np
import trimesh

from work.collision_parameters import CollisionPolicy


@jax.tree_util.register_static
@dataclass(frozen=True, eq=False)
class FixedEnvironmentModel:
    model_revision: int
    source_hash: bytes
    validation_evidence: str
    policy_hash: bytes
    support_points_m: np.ndarray
    support_radii_mm: np.ndarray
    support_demand: int
    geometry_hash: bytes
    model_hash: bytes

    def __post_init__(self):
        if (type(self.model_revision) is not int or self.model_revision < 1
                or type(self.support_demand) is not int or self.support_demand < 1
                or any(not isinstance(value, bytes) or len(value) != 32 or not any(value)
                       for value in (self.source_hash, self.policy_hash, self.geometry_hash, self.model_hash))
                or not isinstance(self.validation_evidence, str) or not self.validation_evidence.strip()):
            raise ValueError("固定模型的版本和来源证据无效")
        points = np.array(self.support_points_m, dtype=np.float64, copy=True)
        radii = np.array(self.support_radii_mm, dtype=np.float64, copy=True)
        if (points.ndim != 2 or points.shape[1] != 3 or radii.shape != (len(points),)
                or not np.all(np.isfinite(points)) or not np.all(np.isfinite(radii))
                or np.any(radii <= 0) or len(points) not in (self.support_demand, 0)):
            raise ValueError("固定模型的 support 形状、数值或需求数量无效")
        points.setflags(write=False)
        radii.setflags(write=False)
        object.__setattr__(self, "support_points_m", points)
        object.__setattr__(self, "support_radii_mm", radii)
        self.verify_identity()

    def __hash__(self):
        return hash(self.model_hash)

    def __eq__(self, other):
        if not isinstance(other, FixedEnvironmentModel):
            return NotImplemented
        return self.model_hash == other.model_hash

    def __reduce__(self):
        return type(self), tuple(getattr(self, name) for name in self.__dataclass_fields__)

    def verify_identity(self):
        expected = self._identity(
            self.model_revision, self.source_hash, self.validation_evidence, self.policy_hash,
            self.support_points_m, self.support_radii_mm, self.support_demand, self.geometry_hash,
        )
        if self.model_hash != expected:
            raise ValueError("固定模型 model_hash 与内容、来源或 policy 不一致")

    def support_history_hashes(self):
        return np.asarray([np.frombuffer(hashlib.sha256(self.model_hash + np.int64(slot).tobytes()).digest(),
                                        dtype=np.uint8) for slot in range(len(self.support_points_m))],
                          dtype=np.uint8).reshape((-1, 32))

    @staticmethod
    def _identity(revision, source, evidence, policy, points, radii, demand, geometry):
        return hashlib.sha256(json.dumps({
            "model_revision": revision, "source_hash": source.hex(), "validation_evidence": evidence,
            "policy_hash": policy.hex(), "support_points_m": points.tolist(),
            "support_radii_mm": radii.tolist(), "support_demand": demand, "geometry_hash": geometry.hex(),
        }, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).digest()

    @classmethod
    def from_mesh(
            cls, mesh: trimesh.Trimesh, *, policy: CollisionPolicy, model_revision: int,
            source_hash: bytes, validation_evidence: str, error_mm: float,
            frame_id: str = "base_link", length_unit: str = "m"):
        if not isinstance(mesh, trimesh.Trimesh) or not mesh.is_volume:
            raise ValueError("固定 mesh 必须是经过检查的封闭正体积几何")
        if (type(model_revision) is not int or model_revision < 1
                or not isinstance(source_hash, bytes) or len(source_hash) != 32 or not any(source_hash)
                or not isinstance(validation_evidence, str) or not validation_evidence.strip()
                or frame_id != "base_link" or length_unit != "m"
                or not np.isfinite(error_mm) or error_mm < 0):
            raise ValueError("固定模型的 revision、来源、单位或误差证据无效")
        bounds = np.array(mesh.bounds, dtype=np.float64)
        if not np.all(np.isfinite(bounds)):
            raise ValueError("固定 mesh 的坐标必须有限")
        parameters = policy.artifact.parameters
        limit = parameters["support_radius_limit_mm"]["value"]
        if error_mm >= limit:
            raise ValueError("固定模型误差超出 support_radius_limit_mm")
        edge_m = 2 * np.nextafter(limit - error_mm, 0.) / np.sqrt(3) * 1e-3
        counts = np.maximum(1, np.ceil((bounds[1] - bounds[0]) / edge_m).astype(np.int64))
        demand = int(np.prod(counts, dtype=object))
        possible = parameters["max_support_points"]["value"] * parameters["support_merge_max_points"]["value"]
        if demand > possible:
            points, radii = np.empty((0, 3)), np.empty(0)
        else:
            indices = np.indices(tuple(counts)).reshape(3, -1).T
            widths = (bounds[1] - bounds[0]) / counts
            points = bounds[0] + (indices + .5) * widths
            radii = np.full(demand, np.nextafter(np.linalg.norm(widths) * 500 + error_mm, np.inf))
            if np.any(radii > limit):
                raise ValueError("固定模型包围范围无法满足 support_radius_limit_mm")
        identity = hashlib.sha256()
        for value in (np.asarray(mesh.vertices), np.asarray(mesh.faces), np.asarray([error_mm])):
            identity.update(value.tobytes())
        geometry_hash = identity.digest()
        model_hash = cls._identity(model_revision, source_hash, validation_evidence, policy.collision_policy_hash,
                                   points, radii, demand, geometry_hash)
        return cls(model_revision, source_hash, validation_evidence, policy.collision_policy_hash,
                   points, radii, demand, geometry_hash, model_hash)
