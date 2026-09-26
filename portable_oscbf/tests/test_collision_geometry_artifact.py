from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh


REPO_ROOT = Path(__file__).resolve().parents[2]
GENERATOR = REPO_ROOT / "portable_oscbf" / "scripts" / "generate_collision_geometry.py"
CLOSED_MESH_GENERATOR = REPO_ROOT / "portable_oscbf" / "scripts" / "generate_closed_collision_meshes.py"
PRODUCTION_MANIFEST = REPO_ROOT / "portable_oscbf" / "config" / "collision_geometry_sources.json"
PRODUCTION_ARTIFACT = REPO_ROOT / "portable_oscbf" / "config" / "collision_geometry_artifact.json"


def _write_sources(directory: Path) -> tuple[Path, Path]:
    robot = directory / "robot.urdf"
    robot.write_text(
        '<robot name="geometry_test">'
        '<link name="base_link"><collision><geometry><mesh filename="base_link.stl"/>'
        '</geometry></collision></link>'
        '<link name="Link1"><collision><geometry><mesh filename="Link1.stl"/>'
        '</geometry></collision></link>'
        '</robot>',
        encoding="utf-8",
    )
    trimesh.creation.box(extents=[0.2, 0.1, 0.3]).export(directory / "base_link.stl")
    trimesh.creation.box(extents=[0.1, 0.2, 0.2]).export(directory / "Link1.stl")
    manifest = directory / "sources.json"
    manifest.write_text(
        json.dumps({
            "schema_version": 1,
            "urdf": "robot.urdf",
            "tool_identity": "tool-box-1",
            "tool_link": "Link1",
            "slot_capacity": 3,
            "links": [
                {"link": "base_link", "mesh": "base_link.stl", "unit": "m"},
                {"link": "Link1", "mesh": "Link1.stl", "unit": "m"},
            ],
            "allowed_contacts": [],
        }),
        encoding="utf-8",
    )
    return manifest, directory / "artifact.json"


def _run(manifest: Path, output: Path, *options: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(GENERATOR), "--manifest", str(manifest), "--output", str(output), *options],
        cwd=REPO_ROOT,
        env=os.environ.copy(),
        capture_output=True,
        text=True,
        check=False,
    )


def test_closed_mesh_generates_reproducible_artifact(tmp_path: Path) -> None:
    manifest, output = _write_sources(tmp_path)
    generated = _run(manifest, output)
    assert generated.returncode == 0, generated.stderr
    first = json.loads(output.read_text(encoding="utf-8"))
    assert len(first["geometry_hash"]) == 64
    assert first["geometry"]["tool_identity"] == "tool-box-1"
    assert len(first["geometry"]["links"]) == 2
    assert first["self_collision"]["candidate_pairs"] == [["base_link", "Link1"]]
    for link in first["geometry"]["links"]:
        assert len(link["slots"]) == 3
        assert any(slot["active_mask"] for slot in link["slots"])
        assert sum(link["coverage_proof"]["coverage_count_by_slot"]) == link["coverage_proof"]["tetrahedron_count"]
        assert link["coverage_proof"]["maximum_vertex_scale_squared"] <= 1.0
        assert len(link["mesh_sha256"]) == 64
    checked = _run(manifest, output, "--check")
    assert checked.returncode == 0, checked.stderr
    verified = _run(manifest, output, "--verify")
    assert verified.returncode == 0, verified.stderr
    second_output = tmp_path / "artifact-again.json"
    again = _run(manifest, second_output)
    assert again.returncode == 0, again.stderr
    second = json.loads(second_output.read_text(encoding="utf-8"))
    assert first["geometry_hash"] == second["geometry_hash"]
    assert first["geometry"] == second["geometry"]
    assert first["self_collision"] == second["self_collision"]
    assert first["runtime_evidence"]["generation_runtime_ns"] > 0


def test_nonclosed_and_nonmanifold_meshes_are_rejected(tmp_path: Path) -> None:
    manifest, output = _write_sources(tmp_path)
    mesh = trimesh.creation.box()
    mesh.update_faces(range(len(mesh.faces) - 1))
    mesh.export(tmp_path / "Link1.stl")
    opened = _run(manifest, output)
    assert opened.returncode != 0
    assert "closed" in opened.stderr

    mesh = trimesh.creation.box()
    mesh.faces = list(mesh.faces) + [mesh.faces[0]]
    mesh.export(tmp_path / "Link1.stl")
    nonmanifold = _run(manifest, output)
    assert nonmanifold.returncode != 0
    assert "closed" in nonmanifold.stderr or "topology" in nonmanifold.stderr


def test_unit_and_link_coverage_are_required(tmp_path: Path) -> None:
    manifest, output = _write_sources(tmp_path)
    source = json.loads(manifest.read_text(encoding="utf-8"))
    del source["links"][0]["unit"]
    manifest.write_text(json.dumps(source), encoding="utf-8")
    missing_unit = _run(manifest, output)
    assert missing_unit.returncode != 0
    assert "unit" in missing_unit.stderr

    source["links"][0]["unit"] = "m"
    source["links"].pop()
    manifest.write_text(json.dumps(source), encoding="utf-8")
    missing_link = _run(manifest, output)
    assert missing_link.returncode != 0
    assert "missing" in missing_link.stderr


def test_urdf_mesh_identity_and_collision_transform(tmp_path: Path) -> None:
    manifest, output = _write_sources(tmp_path)
    urdf = tmp_path / "robot.urdf"
    tree = ET.parse(urdf)
    link = tree.getroot().find("./link[@name='Link1']")
    collision = link.find("collision")
    ET.SubElement(collision, "origin", xyz="1 2 3", rpy="0 0 1.5707963267948966")
    collision.find("geometry/mesh").set("scale", "2 1 1")
    tree.write(urdf, encoding="unicode")
    transformed = _run(manifest, output)
    assert transformed.returncode == 0, transformed.stderr
    artifact = json.loads(output.read_text(encoding="utf-8"))
    second_link = artifact["geometry"]["links"][1]
    assert np.isclose(second_link["mesh_volume_m3"], 0.008, rtol=1e-6)
    assert second_link["collision_origin_xyz_m"] == [1.0, 2.0, 3.0]
    assert second_link["urdf_mesh_scale"] == [2.0, 1.0, 1.0]
    assert np.allclose(second_link["slots"][0]["center_m"], [1.0, 2.0, 3.0])

    collision.find("geometry/mesh").set("filename", "base_link.stl")
    tree.write(urdf, encoding="unicode")
    mismatch = _run(manifest, output)
    assert mismatch.returncode != 0
    assert "differs from URDF collision mesh" in mismatch.stderr


def test_nonfinite_source_triangle_is_rejected_before_mesh_processing(tmp_path: Path) -> None:
    manifest, output = _write_sources(tmp_path)
    mesh = trimesh.creation.box()
    mesh.vertices = np.vstack([mesh.vertices, [[np.nan, 0, 0], [0, 0, 0], [1, 0, 0]]])
    mesh.faces = np.vstack([mesh.faces, [[8, 9, 10]]])
    mesh.export(tmp_path / "Link1.stl")
    invalid = _run(manifest, output)
    assert invalid.returncode != 0
    assert "invalid mesh vertices" in invalid.stderr


def test_small_closed_mesh_keeps_relative_volume_check(tmp_path: Path) -> None:
    manifest, output = _write_sources(tmp_path)
    trimesh.creation.box(extents=[1e-4, 2e-4, 3e-4]).export(tmp_path / "Link1.stl")
    generated = _run(manifest, output)
    assert generated.returncode == 0, generated.stderr
    artifact = json.loads(output.read_text(encoding="utf-8"))
    volume = artifact["geometry"]["links"][1]["mesh_volume_m3"]
    assert np.isclose(volume, 6e-12, rtol=1e-6)
    assert _run(manifest, output, "--verify").returncode == 0


def test_uncovered_tetrahedron_is_rejected(tmp_path: Path) -> None:
    manifest, output = _write_sources(tmp_path)
    generated = _run(manifest, output)
    assert generated.returncode == 0, generated.stderr
    artifact = json.loads(output.read_text(encoding="utf-8"))
    for link in artifact["geometry"]["links"]:
        for slot in link["slots"]:
            if slot["active_mask"]:
                slot["radii_m"] = [radius / 100.0 for radius in slot["radii_m"]]
    output.write_text(json.dumps(artifact), encoding="utf-8")
    verified = _run(manifest, output, "--verify")
    assert verified.returncode != 0
    assert "not covered" in verified.stderr


def test_runtime_record_is_marked_unqualified_and_validated(tmp_path: Path) -> None:
    manifest, output = _write_sources(tmp_path)
    generated = _run(manifest, output)
    assert generated.returncode == 0, generated.stderr
    artifact = json.loads(output.read_text(encoding="utf-8"))
    assert artifact["runtime_evidence"]["qualified_for_query_deadline"] is False
    artifact["runtime_evidence"]["generation_runtime_ns"] = -1
    output.write_text(json.dumps(artifact), encoding="utf-8")
    invalid = _run(manifest, output, "--verify")
    assert invalid.returncode != 0
    assert "runtime_evidence" in invalid.stderr


def test_mesh_tool_and_allowed_contact_identity(tmp_path: Path) -> None:
    manifest, output = _write_sources(tmp_path)
    generated = _run(manifest, output)
    assert generated.returncode == 0, generated.stderr
    original = json.loads(output.read_text(encoding="utf-8"))
    source = json.loads(manifest.read_text(encoding="utf-8"))
    source["allowed_contacts"] = [{
        "links": ["base_link", "Link1"],
        "reason": "verified contact",
        "evidence": "evidence/report.txt",
        "geometry_hash": original["geometry_hash"],
    }]
    manifest.write_text(json.dumps(source), encoding="utf-8")
    unverified = _run(manifest, output)
    assert unverified.returncode != 0
    assert "full-joint-range certificate" in unverified.stderr

    source["tool_identity"] = "tool-box-2"
    manifest.write_text(json.dumps(source), encoding="utf-8")
    source["allowed_contacts"] = []
    manifest.write_text(json.dumps(source), encoding="utf-8")
    new_tool = _run(manifest, output)
    assert new_tool.returncode == 0, new_tool.stderr
    changed = json.loads(output.read_text(encoding="utf-8"))
    assert changed["geometry_hash"] != original["geometry_hash"]

    trimesh.creation.box(extents=[0.11, 0.2, 0.2]).export(tmp_path / "Link1.stl")
    changed_mesh = _run(manifest, output)
    assert changed_mesh.returncode == 0, changed_mesh.stderr
    mesh_artifact = json.loads(output.read_text(encoding="utf-8"))
    assert mesh_artifact["geometry_hash"] != changed["geometry_hash"]


def test_production_closed_meshes_and_artifact() -> None:
    checked_meshes = subprocess.run(
        [sys.executable, str(CLOSED_MESH_GENERATOR), "--check"],
        cwd=REPO_ROOT,
        env=os.environ.copy(),
        capture_output=True,
        text=True,
        check=False,
    )
    assert checked_meshes.returncode == 0, checked_meshes.stderr
    verified = _run(PRODUCTION_MANIFEST, PRODUCTION_ARTIFACT, "--verify")
    assert verified.returncode == 0, verified.stderr
    artifact = json.loads(PRODUCTION_ARTIFACT.read_text(encoding="utf-8"))
    assert len(artifact["geometry"]["links"]) == 10
    assert artifact["geometry"]["tool_link"] == "Link9"
    assert len(artifact["self_collision"]["candidate_pairs"]) == 45
    assert artifact["runtime_evidence"]["qualified_for_query_deadline"] is False
