#!/usr/bin/env python3
"""M2 acceptance: OBB envelope calibration and self-collision topology."""

from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path

import _path_setup  # noqa: F401
import numpy as np
import pytest
import trimesh

from work.fcl_collision_mesh import FclMeshSelfCollisionChecker
from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
from work.obb_collision_model import (
    OBB_COLLISION_PAIRS,
    OBB_HALF_EXTENTS_M,
    OBB_LINK_INDICES,
    OBB_LINK_NAMES,
    OBB_LOCAL_CENTERS_M,
    OBB_LOCAL_ROTATIONS,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
MESH_DIR = REPO_ROOT / "models" / "ninezzhou" / "meshes"
GENERATOR = REPO_ROOT / "portable_oscbf" / "scripts" / "generate_obb_calibration.py"


def _load_stl_vertices() -> dict[str, np.ndarray]:
    vertices = {}
    for link_name in OBB_LINK_NAMES:
        mesh = trimesh.load_mesh(str(MESH_DIR / f"{link_name}.STL"))
        vertices[link_name] = np.asarray(mesh.vertices, dtype=np.float64)
    return vertices


def _world_obb(link_index: int, transform: np.ndarray):
    """World OBB (R_w, c_w, h) from a link transform."""

    rotation = np.asarray(OBB_LOCAL_ROTATIONS[link_index])
    center = np.asarray(OBB_LOCAL_CENTERS_M[link_index])
    half = np.asarray(OBB_HALF_EXTENTS_M[link_index])
    R_w = transform[:3, :3] @ rotation
    c_w = transform[:3, :3] @ center + transform[:3, 3]
    return R_w, c_w, half


def _obb_sat_separated(obb_a, obb_b) -> bool:
    """True when the two OBBs are disjoint (a separating axis exists)."""

    (R_a, c_a, h_a), (R_b, c_b, h_b) = obb_a, obb_b
    axes = [R_a[:, 0], R_a[:, 1], R_a[:, 2],
            R_b[:, 0], R_b[:, 1], R_b[:, 2]]
    for i in range(3):
        for j in range(3):
            cross = np.cross(R_a[:, i], R_b[:, j])
            norm = np.linalg.norm(cross)
            if norm > 1e-12:
                axes.append(cross / norm)
    delta = c_b - c_a
    for axis in axes:
        radius_sum = (
            h_a @ np.abs(R_a.T @ axis) + h_b @ np.abs(R_b.T @ axis))
        if abs(delta @ axis) > radius_sum:
            return True
    return False


def test_obb_encloses_all_stl_vertices():
    vertices = _load_stl_vertices()
    tolerance = 1e-6
    for index, link_name in enumerate(OBB_LINK_NAMES):
        points = vertices[link_name]
        rotation = np.asarray(OBB_LOCAL_ROTATIONS[index])
        center = np.asarray(OBB_LOCAL_CENTERS_M[index])
        half = np.asarray(OBB_HALF_EXTENTS_M[index])
        local = (points - center) @ rotation  # OBB coordinates
        assert np.all(local <= half + tolerance), (
            f"{link_name}: vertex exceeds OBB upper bound")
        assert np.all(local >= -half - tolerance), (
            f"{link_name}: vertex exceeds OBB lower bound")


def test_obb_volume_ratio_above_threshold():
    vertices = _load_stl_vertices()
    ratios = {}
    for index, link_name in enumerate(OBB_LINK_NAMES):
        points = vertices[link_name]
        aabb_volume = float(np.prod(points.max(axis=0) - points.min(axis=0)))
        obb_volume = float(np.prod(2.0 * np.asarray(OBB_HALF_EXTENTS_M[index])))
        ratios[link_name] = obb_volume / aabb_volume
    for link_name, ratio in ratios.items():
        print(f"OBB volume ratio {link_name}: {ratio:.4f}")
        assert ratio > 0.7, f"{link_name} OBB volume ratio {ratio:.4f} <= 0.7"


def test_collision_pairs_are_non_adjacent_and_exemptions_preserved():
    pairs = np.asarray(OBB_COLLISION_PAIRS)
    assert pairs.shape == (14, 2)
    assert np.all((pairs >= 0) & (pairs <= 9))
    for i, j in pairs:
        assert abs(int(i) - int(j)) >= 2, (
            f"adjacent pair in topology: {(int(i), int(j))}")
    exempted = frozenset((3, 5))
    for i, j in pairs:
        assert exempted != frozenset((int(i), int(j))), (
            "Link3-Link5 exemption was not preserved")


def test_zero_configuration_has_no_obb_overlap_and_fcl_clearance():
    robot = NineaxisManipulatorJAX()
    transforms = np.asarray(robot._compute_all_link_transforms(
        np.zeros(robot.num_joints)))
    transforms_by_name = {
        OBB_LINK_NAMES[index]: transforms[index] for index in OBB_LINK_INDICES
    }

    # SAT: every OBB pair must be disjoint at the zero configuration.
    for i, j in np.asarray(OBB_COLLISION_PAIRS):
        obb_a = _world_obb(int(i), transforms[int(i)])
        obb_b = _world_obb(int(j), transforms[int(j)])
        assert _obb_sat_separated(obb_a, obb_b), (
            f"OBB overlap at zero config: {(OBB_LINK_NAMES[i], OBB_LINK_NAMES[j])}")

    # FCL: mesh clearance must be positive for every topology pair.
    checker = FclMeshSelfCollisionChecker(str(MESH_DIR), max_faces=300)
    results = checker.check(
        {name: np.asarray(T) for name, T in transforms_by_name.items()},
        activation_dist=100.0)
    distance_by_pair = {
        frozenset((result.name_i, result.name_j)): result.distance
        for result in results
    }
    for i, j in np.asarray(OBB_COLLISION_PAIRS):
        key = frozenset((OBB_LINK_NAMES[i], OBB_LINK_NAMES[j]))
        assert key in distance_by_pair, (
            f"FCL pair missing from results: {tuple(key)}")
        assert distance_by_pair[key] > 0.0, (
            f"FCL distance <= 0 at zero config: {tuple(key)} "
            f"({distance_by_pair[key]:.6f} m)")


def test_generator_is_deterministic():
    with tempfile.TemporaryDirectory() as tmp:
        tmp_dir = Path(tmp)
        outputs = []
        for _ in range(2):
            output_py = tmp_dir / "obb_collision_model.py"
            output_yaml = tmp_dir / "obb_model.yaml"
            subprocess.run(
                [sys.executable, str(GENERATOR),
                 "--mesh-dir", str(MESH_DIR),
                 "--output-py", str(output_py),
                 "--output-yaml", str(output_yaml)],
                check=True, capture_output=True, text=True)
            outputs.append((output_py.read_bytes(), output_yaml.read_bytes()))
        assert outputs[0] == outputs[1], (
            "OBB generator output is not deterministic")


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
