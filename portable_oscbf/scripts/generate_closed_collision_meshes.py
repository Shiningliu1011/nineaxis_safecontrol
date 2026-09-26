from __future__ import annotations

import argparse
import hashlib
import io
import json
from pathlib import Path
from urllib.parse import urlparse
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
import trimesh


REPO_ROOT = Path(__file__).resolve().parents[2]
SOURCE_URDF = REPO_ROOT / "models/ninezzhou/urdf/ninezzhou.urdf"
OUTPUT_URDF = REPO_ROOT / "models/ninezzhou/urdf/ninezzhou_collision.urdf"
OUTPUT_DIR = REPO_ROOT / "models/ninezzhou/collision_meshes"
OUTPUT_PROVENANCE = OUTPUT_DIR / "provenance.json"


def _source_mesh_path(filename: str, link: str) -> Path:
    parsed = urlparse(filename)
    expected = f"/meshes/{link}.STL"
    if parsed.scheme != "package" or parsed.netloc != "ninezzhou" or parsed.path != expected:
        raise ValueError(f"{link}: unexpected source mesh URI {filename!r}")
    path = REPO_ROOT / "models/ninezzhou" / parsed.path.lstrip("/")
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def _closed_hull(path: Path) -> tuple[bytes, float]:
    source = trimesh.load_mesh(io.BytesIO(path.read_bytes()), file_type="stl", process=False)
    if not isinstance(source, trimesh.Trimesh):
        raise ValueError(f"{path}: expected one triangle mesh")
    if not np.all(np.isfinite(source.vertices)) or len(source.vertices) < 4:
        raise ValueError(f"{path}: invalid source vertices")
    hull = source.convex_hull
    if not hull.is_volume or not hull.is_watertight or not hull.is_winding_consistent:
        raise ValueError(f"{path}: generated hull is not a closed volume")

    hull_bytes = hull.export(file_type="stl")
    saved = trimesh.load_mesh(io.BytesIO(hull_bytes), file_type="stl")
    if not saved.is_volume:
        raise ValueError(f"{path}: exported hull is not a closed volume")
    planes = ConvexHull(saved.vertices).equations
    scale = float(np.ptp(source.vertices, axis=0).max())
    tolerance = scale * 1e-8
    maximum_excess = -float("inf")
    for start in range(0, len(source.vertices), 4096):
        vertices = source.vertices[start:start + 4096]
        signed = vertices @ planes[:, :3].T + planes[:, 3]
        maximum_excess = max(maximum_excess, float(signed.max()))
        if maximum_excess > tolerance:
            raise ValueError(f"{path}: exported hull does not contain all source vertices")
    return hull_bytes, maximum_excess


def _outputs() -> dict[Path, bytes]:
    root = ET.parse(SOURCE_URDF).getroot()
    if root.tag != "robot":
        raise ValueError(f"{SOURCE_URDF}: expected a URDF robot")
    outputs: dict[Path, bytes] = {}
    provenance = {
        "schema_version": 1,
        "generator_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "source_urdf": "../urdf/ninezzhou.urdf",
        "source_urdf_sha256": hashlib.sha256(SOURCE_URDF.read_bytes()).hexdigest(),
        "links": [],
    }
    for link in root.findall("link"):
        name = link.get("name")
        collisions = link.findall("collision")
        if not collisions:
            continue
        if not name or len(collisions) != 1:
            raise ValueError(f"{SOURCE_URDF}: invalid collision link {name!r}")
        mesh = collisions[0].find("geometry/mesh")
        if mesh is None or mesh.get("filename") is None:
            raise ValueError(f"{SOURCE_URDF}: {name} requires one collision mesh")
        source = _source_mesh_path(mesh.get("filename"), name)
        hull_bytes, maximum_excess = _closed_hull(source)
        outputs[OUTPUT_DIR / f"{name}.stl"] = hull_bytes
        provenance["links"].append({
            "link": name,
            "source_mesh": f"../meshes/{name}.STL",
            "source_mesh_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            "closed_mesh": f"{name}.stl",
            "closed_mesh_sha256": hashlib.sha256(hull_bytes).hexdigest(),
            "maximum_source_vertex_excess_m": maximum_excess,
        })
        mesh.set("filename", f"package://ninezzhou/collision_meshes/{name}.stl")
    if len(outputs) != 10:
        raise ValueError(f"{SOURCE_URDF}: expected ten collision meshes, found {len(outputs)}")
    ET.indent(root, space="  ")
    outputs[OUTPUT_URDF] = ET.tostring(root, encoding="utf-8", xml_declaration=True)
    outputs[OUTPUT_PROVENANCE] = (
        json.dumps(provenance, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    ).encode("utf-8")
    return outputs


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = _outputs()
    for path, expected in outputs.items():
        if args.check:
            if not path.is_file() or path.read_bytes() != expected:
                raise ValueError(f"{path}: differs from generated source")
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(expected)
        print(f"{path.relative_to(REPO_ROOT)} {hashlib.sha256(expected).hexdigest()}")


if __name__ == "__main__":
    main()
