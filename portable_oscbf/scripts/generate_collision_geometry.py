from __future__ import annotations

import argparse
import hashlib
import io
import itertools
import json
import math
from pathlib import Path
import time
from urllib.parse import urlparse
import xml.etree.ElementTree as ET

import numpy as np
import scipy
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import tetgen
import trimesh

from generate_closed_collision_meshes import _closed_hull


GENERATOR_VERSION = 1
UNIT_SCALE_M = {"m": 1.0, "mm": 0.001}


def _json_bytes(value: object) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False
    ).encode("utf-8")


def _sha256(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _require_keys(
    value: dict, required: set[str], name: str, optional: set[str] | None = None
) -> None:
    missing = required - value.keys()
    unknown = value.keys() - required - (optional or set())
    if missing or unknown:
        raise ValueError(f"{name}: missing={sorted(missing)}, unknown={sorted(unknown)}")


def _source_path(manifest_path: Path, value: str) -> Path:
    if not isinstance(value, str) or not value:
        raise ValueError("source path must be a nonempty string")
    path = (manifest_path.parent / value).resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def _urdf_vector(value: str | None, default: tuple[float, float, float], name: str) -> list[float]:
    numbers = list(default) if value is None else [float(part) for part in value.split()]
    if len(numbers) != 3 or not np.all(np.isfinite(numbers)):
        raise ValueError(f"{name} must contain three finite numbers")
    return numbers


def _urdf_mesh_path(urdf_path: Path, filename: str) -> Path:
    parsed = urlparse(filename)
    if parsed.scheme == "package":
        if not parsed.netloc or not parsed.path or parsed.query or parsed.fragment:
            raise ValueError(f"{urdf_path}: invalid package mesh URI {filename!r}")
        path = Path(__file__).resolve().parents[2] / "models" / parsed.netloc / parsed.path.lstrip("/")
    elif parsed.scheme == "":
        path = urdf_path.parent / filename
    else:
        raise ValueError(f"{urdf_path}: unsupported mesh URI {filename!r}")
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def _urdf_links(path: Path) -> tuple[set[str], dict[str, dict]]:
    root = ET.parse(path).getroot()
    if root.tag != "robot":
        raise ValueError(f"{path}: expected a URDF robot element")
    all_links: set[str] = set()
    collision_links: dict[str, dict] = {}
    for element in root.findall("link"):
        name = element.get("name")
        if not name or name in all_links:
            raise ValueError(f"{path}: invalid or duplicate URDF link {name!r}")
        all_links.add(name)
        collisions = element.findall("collision")
        if collisions:
            if len(collisions) != 1 or collisions[0].find("geometry/mesh") is None:
                raise ValueError(f"{path}: {name} requires one collision mesh")
            mesh = collisions[0].find("geometry/mesh")
            filename = mesh.get("filename")
            if not filename:
                raise ValueError(f"{path}: {name} collision mesh filename is missing")
            scale = _urdf_vector(mesh.get("scale"), (1.0, 1.0, 1.0), f"{name} mesh scale")
            if any(value <= 0.0 for value in scale):
                raise ValueError(f"{path}: {name} mesh scale must be positive")
            origin = collisions[0].find("origin")
            xyz = _urdf_vector(
                None if origin is None else origin.get("xyz"), (0.0, 0.0, 0.0), f"{name} collision xyz"
            )
            rpy = _urdf_vector(
                None if origin is None else origin.get("rpy"), (0.0, 0.0, 0.0), f"{name} collision rpy"
            )
            collision_links[name] = {
                "path": _urdf_mesh_path(path, filename),
                "scale": scale,
                "origin_xyz_m": xyz,
                "origin_rpy_rad": rpy,
            }
    return all_links, collision_links


def _load_manifest(path: Path) -> tuple[dict, list[dict], str | None]:
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict):
        raise ValueError("manifest must be an object")
    _require_keys(
        manifest,
        {"schema_version", "urdf", "tool_identity", "tool_link", "slot_capacity", "links", "allowed_contacts"},
        "manifest",
        {"source_provenance"},
    )
    if manifest["schema_version"] != 1:
        raise ValueError("manifest.schema_version must be 1")
    if not isinstance(manifest["tool_identity"], str) or not manifest["tool_identity"]:
        raise ValueError("manifest.tool_identity must be a nonempty string")
    capacity = manifest["slot_capacity"]
    if type(capacity) is not int or capacity <= 0:
        raise ValueError("manifest.slot_capacity must be a positive integer")
    if not isinstance(manifest["links"], list) or not manifest["links"]:
        raise ValueError("manifest.links must be a nonempty list")
    if not isinstance(manifest["allowed_contacts"], list):
        raise ValueError("manifest.allowed_contacts must be a list")
    if manifest["allowed_contacts"]:
        raise ValueError("allowed_contacts requires a verified full-joint-range certificate")

    all_links, collision_links = _urdf_links(_source_path(path, manifest["urdf"]))
    tool_link = manifest["tool_link"]
    if tool_link not in all_links:
        raise ValueError(f"manifest.tool_link {tool_link!r} is absent from the URDF")
    entries: list[dict] = []
    seen: set[str] = set()
    for index, entry in enumerate(manifest["links"]):
        if not isinstance(entry, dict):
            raise ValueError(f"manifest.links[{index}] must be an object")
        _require_keys(entry, {"link", "mesh", "unit"}, f"manifest.links[{index}]")
        link = entry["link"]
        if not isinstance(link, str) or link not in all_links or link in seen:
            raise ValueError(f"manifest.links[{index}].link is invalid or duplicated: {link!r}")
        if link not in collision_links and link != tool_link:
            raise ValueError(f"manifest.links[{index}].link has no URDF collision mesh: {link}")
        seen.add(link)
        if entry["unit"] not in UNIT_SCALE_M:
            raise ValueError(f"manifest.links[{index}].unit must be m or mm")
        mesh_path = _source_path(path, entry["mesh"])
        collision = collision_links.get(link)
        if collision is not None and mesh_path != collision["path"]:
            raise ValueError(f"manifest.links[{index}].mesh differs from URDF collision mesh: {link}")
        if collision is not None and entry["unit"] == "mm" and not np.allclose(
            collision["scale"], [0.001, 0.001, 0.001], rtol=0.0, atol=1e-15
        ):
            raise ValueError(f"manifest.links[{index}].unit mm requires URDF mesh scale 0.001")
        entries.append({**entry, "path": mesh_path, "collision": collision})
    expected = collision_links.keys() | {tool_link}
    if seen != expected:
        raise ValueError(f"manifest.links mismatch: missing={sorted(expected - seen)}, extra={sorted(seen - expected)}")
    provenance_hash = None
    if "source_provenance" in manifest:
        provenance_path = _source_path(path, manifest["source_provenance"])
        raw = provenance_path.read_bytes()
        provenance = json.loads(raw)
        if not isinstance(provenance, dict):
            raise ValueError(f"{provenance_path}: expected a provenance object")
        _require_keys(
            provenance,
            {"schema_version", "generator_sha256", "source_urdf", "source_urdf_sha256", "links"},
            "source_provenance",
        )
        if provenance["schema_version"] != 1 or not isinstance(provenance["links"], list):
            raise ValueError(f"{provenance_path}: invalid provenance schema")
        hull_generator = Path(__file__).with_name("generate_closed_collision_meshes.py")
        if provenance["generator_sha256"] != _sha256(hull_generator.read_bytes()):
            raise ValueError(f"{provenance_path}: closed mesh generator hash differs")
        source_urdf = _source_path(provenance_path, provenance["source_urdf"])
        if _sha256(source_urdf.read_bytes()) != provenance["source_urdf_sha256"]:
            raise ValueError(f"{provenance_path}: source URDF hash differs")
        _, source_links = _urdf_links(source_urdf)
        if len(provenance["links"]) != len(entries):
            raise ValueError(f"{provenance_path}: source link count differs")
        source_root = ET.parse(source_urdf).getroot()
        derived_root = ET.parse(_source_path(path, manifest["urdf"])).getroot()
        for entry, record in zip(entries, provenance["links"]):
            if not isinstance(record, dict):
                raise ValueError(f"{provenance_path}: invalid source link record")
            _require_keys(
                record,
                {"link", "source_mesh", "source_mesh_sha256", "closed_mesh", "closed_mesh_sha256",
                 "maximum_source_vertex_excess_m"},
                "source_provenance.links",
            )
            source_mesh = _source_path(provenance_path, record["source_mesh"])
            closed_mesh = _source_path(provenance_path, record["closed_mesh"])
            if record["link"] != entry["link"] or source_mesh != source_links[entry["link"]]["path"]:
                raise ValueError(f"{provenance_path}: source link differs from source URDF")
            if closed_mesh != entry["path"]:
                raise ValueError(f"{provenance_path}: closed mesh differs from manifest")
            source_element = source_root.find(f"./link[@name='{entry['link']}']/collision/geometry/mesh")
            derived_element = derived_root.find(f"./link[@name='{entry['link']}']/collision/geometry/mesh")
            if source_element is None or derived_element is None:
                raise ValueError(f"{provenance_path}: derived URDF link is missing")
            source_element.set("filename", derived_element.get("filename"))
            if _sha256(source_mesh.read_bytes()) != record["source_mesh_sha256"]:
                raise ValueError(f"{provenance_path}: source mesh hash differs")
            if _sha256(closed_mesh.read_bytes()) != record["closed_mesh_sha256"]:
                raise ValueError(f"{provenance_path}: closed mesh hash differs")
            excess = record["maximum_source_vertex_excess_m"]
            if not isinstance(excess, (int, float)) or not math.isfinite(excess):
                raise ValueError(f"{provenance_path}: source enclosure record is invalid")
            expected_hull, measured_excess = _closed_hull(source_mesh)
            if closed_mesh.read_bytes() != expected_hull or excess != measured_excess:
                raise ValueError(f"{provenance_path}: closed mesh enclosure differs from source")
        ET.indent(source_root, space="  ")
        ET.indent(derived_root, space="  ")
        if ET.tostring(source_root) != ET.tostring(derived_root):
            raise ValueError(f"{provenance_path}: derived URDF differs from source structure")
        provenance_hash = _sha256(raw)
    return manifest, entries, provenance_hash


def _load_closed_mesh(path: Path, unit: str, collision: dict | None) -> tuple[trimesh.Trimesh, str]:
    raw = path.read_bytes()
    mesh = trimesh.load_mesh(io.BytesIO(raw), file_type=path.suffix.lstrip("."), process=False)
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"{path}: expected one triangle mesh")
    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    if not np.all(np.isfinite(vertices)) or len(vertices) < 4:
        raise ValueError(f"{path}: invalid mesh vertices")
    if not np.all(mesh.nondegenerate_faces()):
        raise ValueError(f"{path}: degenerate triangle")
    raw_face_count = len(mesh.faces)
    mesh.process(validate=False)
    if len(mesh.faces) != raw_face_count:
        raise ValueError(f"{path}: mesh processing removed invalid faces")
    if not mesh.is_watertight or not mesh.is_winding_consistent or not mesh.is_volume:
        raise ValueError(f"{path}: collision mesh must be closed with valid topology and outward faces")
    if collision is None:
        mesh.vertices = np.asarray(mesh.vertices) * UNIT_SCALE_M[unit]
    else:
        scaled = np.asarray(mesh.vertices) * np.asarray(collision["scale"])
        rotation = Rotation.from_euler("xyz", collision["origin_rpy_rad"]).as_matrix()
        mesh.vertices = scaled @ rotation.T + np.asarray(collision["origin_xyz_m"])
    if not np.isfinite(mesh.volume) or mesh.volume <= 0.0:
        raise ValueError(f"{path}: collision mesh volume must be positive")
    return mesh, _sha256(raw)


def _tetrahedralize(mesh: trimesh.Trimesh, source: Path) -> tuple[np.ndarray, np.ndarray, str]:
    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    surface_faces = np.asarray(mesh.faces, dtype=np.int32)
    mesher = tetgen.TetGen(vertices, surface_faces)
    nodes, tetrahedra, _, _ = mesher.tetrahedralize(
        plc=True, quality=False, nobisect=True, order=1, quiet=True
    )
    nodes = np.asarray(nodes, dtype=np.float64)
    tetrahedra = np.asarray(tetrahedra, dtype=np.int32)
    if nodes.ndim != 2 or nodes.shape[1] != 3 or tetrahedra.ndim != 2 or tetrahedra.shape[1] != 4:
        raise ValueError(f"{source}: invalid tetrahedralization shape")
    if not np.all(np.isfinite(nodes)) or len(tetrahedra) == 0:
        raise ValueError(f"{source}: empty or nonfinite tetrahedralization")
    if np.any(tetrahedra < 0) or np.any(tetrahedra >= len(nodes)):
        raise ValueError(f"{source}: tetrahedralization index outside nodes")

    a, b, c, d = (nodes[tetrahedra[:, i]] for i in range(4))
    volumes = np.abs(np.einsum("ij,ij->i", np.cross(b - a, c - a), d - a)) / 6.0
    if np.any(volumes <= 0.0):
        raise ValueError(f"{source}: zero-volume tetrahedron")
    if not math.isclose(float(volumes.sum()), float(mesh.volume), rel_tol=1e-7, abs_tol=0.0):
        raise ValueError(f"{source}: tetrahedralization volume differs from source mesh")

    distance, source_to_nodes = cKDTree(nodes).query(vertices)
    scale = float(np.ptp(vertices, axis=0).max())
    coordinate_magnitude = float(np.abs(vertices).max())
    tolerance = scale * 1e-10 + np.finfo(np.float64).eps * coordinate_magnitude * 16.0
    if np.any(distance > tolerance) or len(np.unique(source_to_nodes)) != len(vertices):
        raise ValueError(f"{source}: tetrahedralization changed source surface vertices")
    expected_faces = {
        tuple(sorted(face)) for face in source_to_nodes[surface_faces].tolist()
    }
    all_faces = np.concatenate(
        [tetrahedra[:, [0, 1, 2]], tetrahedra[:, [0, 1, 3]],
         tetrahedra[:, [0, 2, 3]], tetrahedra[:, [1, 2, 3]]],
        axis=0,
    )
    unique_faces, counts = np.unique(np.sort(all_faces, axis=1), axis=0, return_counts=True)
    if np.any(counts > 2):
        raise ValueError(f"{source}: tetrahedralization has nonmanifold faces")
    boundary_faces = {tuple(face) for face in unique_faces[counts == 1].tolist()}
    if boundary_faces != expected_faces:
        raise ValueError(f"{source}: tetrahedralization boundary differs from source mesh")

    tetra_hash = _sha256(nodes.astype("<f8").tobytes() + tetrahedra.astype("<i4").tobytes())
    return nodes, tetrahedra, tetra_hash


def _fit_ellipsoid(nodes: np.ndarray, tetrahedra: np.ndarray, group: np.ndarray) -> dict:
    points = nodes[np.unique(tetrahedra[group])]
    lower = points.min(axis=0)
    upper = points.max(axis=0)
    center = (lower + upper) / 2.0
    radii = (upper - lower) * (math.sqrt(3.0) / 2.0) * (1.0 + 1e-9)
    if np.any(radii <= 0.0):
        raise ValueError("tetrahedron group has no three-dimensional extent")
    return {"center_m": center.tolist(), "radii_m": radii.tolist()}


def _ellipsoid_volume(ellipsoid: dict) -> float:
    return 4.0 * math.pi * float(np.prod(ellipsoid["radii_m"])) / 3.0


def _fit_groups(nodes: np.ndarray, tetrahedra: np.ndarray, capacity: int) -> list[dict]:
    groups = [np.arange(len(tetrahedra), dtype=np.int32)]
    while len(groups) < capacity:
        best: tuple[float, int, np.ndarray, np.ndarray] | None = None
        for index, group in enumerate(groups):
            if len(group) < 2:
                continue
            centroids = nodes[tetrahedra[group]].mean(axis=1)
            axis = int(np.argmax(np.ptp(centroids, axis=0)))
            order = np.argsort(centroids[:, axis], kind="stable")
            split = len(order) // 2
            left, right = group[order[:split]], group[order[split:]]
            current = _ellipsoid_volume(_fit_ellipsoid(nodes, tetrahedra, group))
            children = sum(
                _ellipsoid_volume(_fit_ellipsoid(nodes, tetrahedra, child))
                for child in (left, right)
            )
            gain = current - children
            if gain > 0.0 and (best is None or gain > best[0]):
                best = gain, index, left, right
        if best is None:
            break
        _, index, left, right = best
        groups[index:index + 1] = [left, right]
    return [_fit_ellipsoid(nodes, tetrahedra, group) for group in groups]


def _verify_coverage(nodes: np.ndarray, tetrahedra: np.ndarray, slots: list[dict]) -> dict:
    active = [slot for slot in slots if slot["active_mask"]]
    if not active:
        raise ValueError("coverage requires an active ellipsoid")
    centers = np.asarray([slot["center_m"] for slot in active], dtype=np.float64)
    radii = np.asarray([slot["radii_m"] for slot in active], dtype=np.float64)
    if np.any(~np.isfinite(centers)) or np.any(~np.isfinite(radii)) or np.any(radii <= 0.0):
        raise ValueError("ellipsoid center or radii are invalid")
    witnesses: list[np.ndarray] = []
    maximum_scale_sq = 0.0
    for start in range(0, len(tetrahedra), 4096):
        points = nodes[tetrahedra[start:start + 4096]]
        normalized = (points[:, None, :, :] - centers[None, :, None, :]) / radii[None, :, None, :]
        scale_sq = np.sum(normalized * normalized, axis=-1).max(axis=-1)
        witness = np.argmin(scale_sq, axis=1)
        selected = scale_sq[np.arange(len(witness)), witness]
        if np.any(selected > 1.0 + 1e-12):
            first = start + int(np.flatnonzero(selected > 1.0 + 1e-12)[0])
            raise ValueError(f"tetrahedron {first} is not covered by one outer ellipsoid")
        maximum_scale_sq = max(maximum_scale_sq, float(selected.max()))
        witnesses.append(witness.astype(np.int32))
    assignment = np.concatenate(witnesses)
    return {
        "tetrahedron_count": int(len(tetrahedra)),
        "coverage_count_by_slot": np.bincount(assignment, minlength=len(active)).tolist(),
        "maximum_vertex_scale_squared": maximum_scale_sq,
        "witness_sha256": _sha256(assignment.astype("<i4").tobytes()),
    }


def _build_link(entry: dict, capacity: int) -> tuple[dict, int]:
    start = time.perf_counter_ns()
    mesh, mesh_hash = _load_closed_mesh(entry["path"], entry["unit"], entry["collision"])
    nodes, tetrahedra, tetra_hash = _tetrahedralize(mesh, entry["path"])
    ellipsoids = _fit_groups(nodes, tetrahedra, capacity)
    slots = [
        {"active_mask": True, **ellipsoid} for ellipsoid in ellipsoids
    ]
    slots += [
        {"active_mask": False, "center_m": [0.0, 0.0, 0.0], "radii_m": [1.0, 1.0, 1.0]}
        for _ in range(capacity - len(slots))
    ]
    proof = _verify_coverage(nodes, tetrahedra, slots)
    active = slots[:len(ellipsoids)]
    envelope_min = np.min([np.asarray(slot["center_m"]) - slot["radii_m"] for slot in active], axis=0)
    envelope_max = np.max([np.asarray(slot["center_m"]) + slot["radii_m"] for slot in active], axis=0)
    return {
        "link": entry["link"],
        "source_mesh": entry["mesh"],
        "source_unit": entry["unit"],
        "collision_origin_xyz_m": (
            [0.0, 0.0, 0.0] if entry["collision"] is None else entry["collision"]["origin_xyz_m"]
        ),
        "collision_origin_rpy_rad": (
            [0.0, 0.0, 0.0] if entry["collision"] is None else entry["collision"]["origin_rpy_rad"]
        ),
        "urdf_mesh_scale": (
            [UNIT_SCALE_M[entry["unit"]]] * 3 if entry["collision"] is None else entry["collision"]["scale"]
        ),
        "mesh_sha256": mesh_hash,
        "tetra_sha256": tetra_hash,
        "mesh_volume_m3": float(mesh.volume),
        "ellipsoid_volume_sum_m3": sum(_ellipsoid_volume(item) for item in ellipsoids),
        "envelope_bounds_m": [envelope_min.tolist(), envelope_max.tolist()],
        "slots": slots,
        "coverage_proof": proof,
    }, time.perf_counter_ns() - start


def generate(manifest_path: Path) -> dict:
    manifest, entries, provenance_hash = _load_manifest(manifest_path)
    urdf_path = _source_path(manifest_path, manifest["urdf"])
    started = time.perf_counter_ns()
    links = []
    link_runtime_ns = {}
    for entry in entries:
        artifact_link, runtime_ns = _build_link(entry, manifest["slot_capacity"])
        links.append(artifact_link)
        link_runtime_ns[entry["link"]] = runtime_ns
    link_names = [entry["link"] for entry in entries]
    all_pairs = [list(pair) for pair in itertools.combinations(link_names, 2)]
    geometry = {
        "schema_version": 1,
        "generator_version": GENERATOR_VERSION,
        "generator_source_sha256": _sha256(Path(__file__).read_bytes()),
        "tetgen_version": tetgen.__version__,
        "trimesh_version": trimesh.__version__,
        "numpy_version": np.__version__,
        "scipy_version": scipy.__version__,
        "urdf_sha256": _sha256(urdf_path.read_bytes()),
        "source_provenance_sha256": provenance_hash,
        "tool_identity": manifest["tool_identity"],
        "tool_link": manifest["tool_link"],
        "slot_capacity": manifest["slot_capacity"],
        "links": links,
    }
    geometry_hash = _sha256(_json_bytes(geometry))
    return {
        "geometry_hash": geometry_hash,
        "geometry": geometry,
        "self_collision": {
            "candidate_pairs": all_pairs,
            "allowed_contacts": [],
        },
        "runtime_evidence": {
            "kind": "offline_generation",
            "qualified_for_query_deadline": False,
            "generation_runtime_ns": time.perf_counter_ns() - started,
            "link_generation_runtime_ns": link_runtime_ns,
        },
    }


def _verify_runtime_evidence(artifact: dict, entries: list[dict]) -> None:
    runtime = artifact["runtime_evidence"]
    _require_keys(
        runtime,
        {"kind", "qualified_for_query_deadline", "generation_runtime_ns", "link_generation_runtime_ns"},
        "runtime_evidence",
    )
    if runtime["kind"] != "offline_generation" or runtime["qualified_for_query_deadline"] is not False:
        raise ValueError("runtime_evidence is not an offline generation record")
    total = runtime["generation_runtime_ns"]
    links = runtime["link_generation_runtime_ns"]
    if type(total) is not int or total <= 0 or not isinstance(links, dict):
        raise ValueError("runtime_evidence generation duration is invalid")
    if set(links) != {entry["link"] for entry in entries}:
        raise ValueError("runtime_evidence link identities differ from manifest")
    if any(type(value) is not int or value <= 0 for value in links.values()):
        raise ValueError("runtime_evidence link duration is invalid")
    if sum(links.values()) > total:
        raise ValueError("runtime_evidence link durations exceed total duration")


def verify_artifact(manifest_path: Path, artifact: dict) -> None:
    manifest, entries, _ = _load_manifest(manifest_path)
    _verify_runtime_evidence(artifact, entries)
    geometry = artifact["geometry"]
    if geometry["urdf_sha256"] != _sha256(_source_path(manifest_path, manifest["urdf"]).read_bytes()):
        raise ValueError("artifact URDF hash differs from source")
    if geometry["tool_identity"] != manifest["tool_identity"] or geometry["tool_link"] != manifest["tool_link"]:
        raise ValueError("artifact tool identity differs from manifest")
    if geometry["slot_capacity"] != manifest["slot_capacity"] or len(geometry["links"]) != len(entries):
        raise ValueError("artifact link count or slot capacity differs from manifest")
    for entry, link in zip(entries, geometry["links"]):
        if link["link"] != entry["link"] or link["source_mesh"] != entry["mesh"]:
            raise ValueError(f"artifact link source differs from manifest: {entry['link']}")
        mesh, mesh_hash = _load_closed_mesh(entry["path"], entry["unit"], entry["collision"])
        if link["mesh_sha256"] != mesh_hash or link["source_unit"] != entry["unit"]:
            raise ValueError(f"artifact mesh identity differs from source: {entry['link']}")
        nodes, tetrahedra, tetra_hash = _tetrahedralize(mesh, entry["path"])
        if link["tetra_sha256"] != tetra_hash:
            raise ValueError(f"artifact tetrahedralization differs from source: {entry['link']}")
        if len(link["slots"]) != manifest["slot_capacity"]:
            raise ValueError(f"artifact slot count differs from manifest: {entry['link']}")
        proof = _verify_coverage(nodes, tetrahedra, link["slots"])
        if proof != link["coverage_proof"]:
            raise ValueError(f"artifact coverage proof differs from source: {entry['link']}")
    if artifact["geometry_hash"] != _sha256(_json_bytes(geometry)):
        raise ValueError("artifact geometry hash differs from geometry")
    generated = generate(manifest_path)
    if geometry != generated["geometry"]:
        raise ValueError("artifact geometry differs from generated geometry")
    if artifact["self_collision"] != generated["self_collision"]:
        raise ValueError("artifact self collision candidates differ from manifest")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--verify", action="store_true")
    args = parser.parse_args()
    if args.check and args.verify:
        parser.error("--check and --verify cannot be combined")
    if args.verify:
        artifact = json.loads(args.output.read_text(encoding="utf-8"))
        verify_artifact(args.manifest, artifact)
        print(artifact["geometry_hash"])
        return
    artifact = generate(args.manifest)
    if args.check:
        saved = json.loads(args.output.read_text(encoding="utf-8"))
        verify_artifact(args.manifest, saved)
        for key in ("geometry_hash", "geometry", "self_collision"):
            if saved[key] != artifact[key]:
                raise ValueError(f"{args.output}: {key} differs from generated geometry")
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(artifact, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    print(artifact["geometry_hash"])


if __name__ == "__main__":
    main()
