from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

from work.obb_collision_model import OBB_LINK_NAMES


def _resource(path: Path, expected: str | None = None) -> dict:
    resolved = path.resolve(strict=True)
    digest = hashlib.sha256(resolved.read_bytes()).hexdigest()
    if expected is not None and digest != expected:
        raise ValueError(f"{resolved}: SHA256 与来源记录不一致")
    return {"path": str(resolved), "sha256": digest}


def mesh_sources(manifest_path: Path) -> dict:
    path = Path(manifest_path).resolve(strict=True)
    manifest = json.loads(path.read_text())
    if manifest["schema_version"] != 1 or manifest["allowed_contacts"]:
        raise ValueError("自碰撞来源要求版本 1 且没有额外接触豁免")
    entries = manifest["links"]
    if [entry["link"] for entry in entries] != list(OBB_LINK_NAMES):
        raise ValueError("自碰撞来源必须按顺序包含十个连杆")
    provenance_path = (path.parent / manifest["source_provenance"]).resolve(strict=True)
    provenance = json.loads(provenance_path.read_text())
    if provenance["schema_version"] != 1 or len(provenance["links"]) != 10:
        raise ValueError("闭合 mesh 来源记录需要十个连杆")
    urdf_path = (path.parent / manifest["urdf"]).resolve(strict=True)
    urdf = ET.parse(urdf_path).getroot()
    if urdf.tag != "robot":
        raise ValueError("自碰撞来源需要有效 URDF")
    auxiliary = [
        _resource(provenance_path), _resource(urdf_path),
        _resource(provenance_path.parent / provenance["source_urdf"], provenance["source_urdf_sha256"]),
        _resource(path.parent.parent / "scripts/generate_closed_collision_meshes.py",
                  provenance["generator_sha256"]),
    ]
    meshes = []
    for entry, record in zip(entries, provenance["links"]):
        name = entry["link"]
        mesh_path = (path.parent / entry["mesh"]).resolve(strict=True)
        if (entry["unit"] != "m" or record["link"] != name
                or mesh_path != (provenance_path.parent / record["closed_mesh"]).resolve(strict=True)):
            raise ValueError(f"{name}: mesh 来源、单位或顺序不一致")
        links = [link for link in urdf.findall("link") if link.get("name") == name]
        if len(links) != 1 or len(links[0].findall("collision")) != 1:
            raise ValueError(f"{name}: URDF 需要一个 collision")
        collision = links[0].find("collision")
        origin, mesh = collision.find("origin"), collision.find("geometry/mesh")
        if (origin is None or mesh is None
                or mesh.get("filename") != f"package://ninezzhou/collision_meshes/{name}.stl"
                or any([float(value) for value in origin.get(field, "0 0 0").split()] != [0, 0, 0]
                       for field in ("xyz", "rpy"))
                or [float(value) for value in mesh.get("scale", "1 1 1").split()] != [1, 1, 1]):
            raise ValueError(f"{name}: collision 坐标与闭合 mesh 来源不一致")
        meshes.append({"link": name, **_resource(mesh_path, record["closed_mesh_sha256"])})
        auxiliary.append(_resource(
            provenance_path.parent / record["source_mesh"], record["source_mesh_sha256"],
        ))
    return {
        "manifest": _resource(path), "auxiliary": auxiliary, "meshes": meshes,
        "pairs": [[first, second] for first in range(10) for second in range(first + 2, 10)],
    }


@dataclass(frozen=True)
class ClosedMesh:
    link: str
    vertices: np.ndarray
    faces: np.ndarray
    radius_m: float


def load_meshes(source: dict) -> tuple[ClosedMesh, ...]:
    expected_pairs = [[first, second] for first in range(10) for second in range(first + 2, 10)]
    if (source["pairs"] != expected_pairs
            or [item["link"] for item in source["meshes"]] != list(OBB_LINK_NAMES)):
        raise ValueError("自碰撞需要十个连杆及全部 36 个非相邻组合")
    for record in [source["manifest"], *source["auxiliary"], *source["meshes"]]:
        _resource(Path(record["path"]), record["sha256"])
    meshes = []
    for record in source["meshes"]:
        mesh = trimesh.load_mesh(record["path"])
        if (not isinstance(mesh, trimesh.Trimesh) or not mesh.is_volume
                or not mesh.is_watertight or not mesh.is_winding_consistent or not mesh.is_convex
                or not np.all(np.isfinite(mesh.vertices))):
            raise ValueError(f"{record['link']}: mesh 必须为有限、闭合、方向一致的凸多面体")
        vertices, faces = np.asarray(mesh.vertices).copy(), np.asarray(mesh.faces).copy()
        vertices.setflags(write=False)
        faces.setflags(write=False)
        meshes.append(ClosedMesh(
            record["link"], vertices, faces, float(np.linalg.norm(vertices, axis=1).max()),
        ))
    return tuple(meshes)
