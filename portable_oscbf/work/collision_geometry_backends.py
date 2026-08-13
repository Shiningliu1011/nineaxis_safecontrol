#!/usr/bin/env python3
"""Collision geometry backend interface and adapters."""

from __future__ import annotations

import importlib.util
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

import numpy as np

from fcl_collision import FclCollisionPair, FclSelfCollisionChecker
from point_cloud_obstacles import FCLPointCloudCollision


_PROJECT_ROOT = Path(__file__).resolve().parents[1]
_COLLISION_BACKEND_CLI_CHOICES = ("primitive", "dpax", "mesh-validate", "convex")


def _ensure_project_root_on_path() -> None:
    root = str(_PROJECT_ROOT)
    if root not in sys.path:
        sys.path.insert(0, root)


def _module_available(module_name: str) -> bool:
    try:
        return importlib.util.find_spec(module_name) is not None
    except (ModuleNotFoundError, ValueError):
        return False


def normalize_collision_backend_name(name: str) -> str:
    """Convert a CLI backend name into the internal support-map key."""
    return name.replace("-", "_")


def collision_backend_cli_choices() -> List[str]:
    """Return the supported CLI names in default/recommended order."""
    return list(_COLLISION_BACKEND_CLI_CHOICES)


class CollisionGeometryBackend:
    """Minimal interface shared by runtime and benchmark collision backends."""

    name = "base"

    def update_poses(self, T_all: Dict[str, np.ndarray]) -> None:
        raise NotImplementedError

    def distance_to_points(self, points: np.ndarray, max_dist: float):
        raise NotImplementedError

    def self_collision_pairs(self, activation_dist: float) -> List[FclCollisionPair]:
        raise NotImplementedError


@dataclass
class PrimitiveCollisionBackend(CollisionGeometryBackend):
    """Current high-rate Box/Sphere/Capsule FCL backend."""

    mesh_dir: str

    name = "primitive"

    def __post_init__(self) -> None:
        self.point_collision = FCLPointCloudCollision()
        self.self_checker = FclSelfCollisionChecker(self.mesh_dir)
        self._T_all = None

    def update_poses(self, T_all: Dict[str, np.ndarray]) -> None:
        self._T_all = T_all
        self.point_collision.update_poses(T_all)

    def distance_to_points(self, points: np.ndarray, max_dist: float):
        return self.point_collision.compute_distances_to_points(points, max_dist=max_dist)

    def self_collision_pairs(self, activation_dist: float) -> List[FclCollisionPair]:
        if self._T_all is None:
            raise RuntimeError("update_poses must be called before self_collision_pairs")
        return self.self_checker.check(self._T_all, activation_dist=activation_dist)


@dataclass
class MeshValidationCollisionBackend(CollisionGeometryBackend):
    """FCL BVHModel backend used for validation/benchmarking, not default runtime."""

    mesh_dir: str
    max_faces: int = 300

    name = "mesh_validate"

    def __post_init__(self) -> None:
        from fcl_collision_mesh import FclMeshSelfCollisionChecker
        from point_cloud_obstacles_mesh import FCLMeshPointCloudCollision

        self.point_collision = FCLMeshPointCloudCollision(self.mesh_dir, self.max_faces)
        self.self_checker = FclMeshSelfCollisionChecker(self.mesh_dir, self.max_faces)
        self._T_all = None

    def update_poses(self, T_all: Dict[str, np.ndarray]) -> None:
        self._T_all = T_all
        self.point_collision.update_poses(T_all)

    def distance_to_points(self, points: np.ndarray, max_dist: float):
        return self.point_collision.compute_distances_to_points(points, max_dist=max_dist)

    def self_collision_pairs(self, activation_dist: float) -> List[FclCollisionPair]:
        if self._T_all is None:
            raise RuntimeError("update_poses must be called before self_collision_pairs")
        return self.self_checker.check(self._T_all, activation_dist=activation_dist)


def _mesh_files_present(mesh_dir: str) -> bool:
    required = ["base_link.STL", "Link1.STL", "Link2.STL", "Link9.STL"]
    return bool(mesh_dir) and all(os.path.exists(os.path.join(mesh_dir, name)) for name in required)


def detect_collision_backend_support(mesh_dir: str = None) -> dict:
    """Return availability of collision backends without silently substituting."""
    mesh_ok = False
    if mesh_dir is not None and _mesh_files_present(mesh_dir):
        mesh_ok = (
            _module_available("fcl_collision_mesh")
            and _module_available("point_cloud_obstacles_mesh")
        )
    convex_ok = (
        _module_available("coal")
        or _module_available("hppfcl")
    )
    if _module_available("dpax"):
        _ensure_project_root_on_path()
        dpax_ok = _module_available("DCOLuse.dpax_collision")
    else:
        dpax_ok = False
    return {
        "primitive": True,
        "mesh_validate": bool(mesh_ok),
        "convex": bool(convex_ok),
        "dpax": bool(dpax_ok),
    }


def make_collision_backend(name: str, mesh_dir: str, max_faces: int = 300) -> CollisionGeometryBackend:
    """Construct a collision backend or raise for unavailable explicit requests."""
    normalized = normalize_collision_backend_name(name)
    support = detect_collision_backend_support(mesh_dir)
    if normalized == "primitive":
        return PrimitiveCollisionBackend(mesh_dir)
    if normalized == "mesh_validate":
        if not support["mesh_validate"]:
            raise RuntimeError("mesh-validate collision backend is unavailable")
        return MeshValidationCollisionBackend(mesh_dir, max_faces=max_faces)
    if normalized == "convex":
        if not support["convex"]:
            raise RuntimeError("convex collision backend is unavailable: install Coal/hpp-fcl bindings")
        raise NotImplementedError("convex backend support is detected but not integrated yet")
    if normalized == "dpax":
        if not support["dpax"]:
            raise RuntimeError("dpax collision backend is unavailable: install dpax and check DCOLuse/")
        _ensure_project_root_on_path()
        from DCOLuse.dpax_backend import DpaxCollisionBackend
        return DpaxCollisionBackend(mesh_dir)
    raise ValueError(f"unknown collision backend: {name}")
