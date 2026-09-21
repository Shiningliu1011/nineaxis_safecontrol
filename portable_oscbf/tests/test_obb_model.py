#!/usr/bin/env python3
"""M2 acceptance: OBB envelope calibration and self-collision topology."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import _path_setup  # noqa: F401
import numpy as np
import pytest
import trimesh
import yaml

from work.fcl_collision_mesh import FclMeshSelfCollisionChecker
from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
from work.obb_geometry_admission import (
    ALL_NONADJACENT_PAIRS,
    OMITTED_NONADJACENT_PAIRS,
)
from work.obb_collision_model import (
    OBB_COLLISION_PAIRS,
    OBB_HALF_EXTENTS_M,
    OBB_LINK_INDICES,
    OBB_LINK_NAMES,
    OBB_LOCAL_CENTERS_M,
    OBB_LOCAL_ROTATIONS,
    OBB_SAMPLE_GRID_SHAPES,
    OBB_SAMPLE_SPHERE_LINK_INDICES,
    OBB_SAMPLE_SPHERE_LOCAL_CENTERS_M,
    OBB_SAMPLE_SPHERE_PADDING_M,
    OBB_SAMPLE_SPHERE_RADII_M,
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


def test_obb_sample_spheres_follow_registered_grids():
    expected_link_indices = []
    expected_centers = []
    expected_radii = []
    for link_index, grid_shape in enumerate(OBB_SAMPLE_GRID_SHAPES):
        half_extents = np.asarray(OBB_HALF_EXTENTS_M[link_index])
        cell_half_extents = half_extents / grid_shape
        for cell_index in np.ndindex(*grid_shape):
            center_obb = (-half_extents +
                          (np.asarray(cell_index) + 0.5) *
                          (2.0 * cell_half_extents))
            expected_link_indices.append(link_index)
            expected_centers.append(
                OBB_LOCAL_CENTERS_M[link_index]
                + OBB_LOCAL_ROTATIONS[link_index] @ center_obb)
            expected_radii.append(
                np.linalg.norm(cell_half_extents)
                + OBB_SAMPLE_SPHERE_PADDING_M)

    assert len(expected_link_indices) == 32
    np.testing.assert_array_equal(
        OBB_SAMPLE_SPHERE_LINK_INDICES, expected_link_indices)
    np.testing.assert_allclose(
        OBB_SAMPLE_SPHERE_LOCAL_CENTERS_M, expected_centers,
        rtol=0.0, atol=5.0e-10)
    np.testing.assert_allclose(
        OBB_SAMPLE_SPHERE_RADII_M, expected_radii,
        rtol=0.0, atol=5.0e-10)


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


def test_online_pairs_are_non_adjacent_and_omissions_are_not_exemptions():
    pairs = np.asarray(OBB_COLLISION_PAIRS)
    assert pairs.shape == (14, 2)
    assert np.all((pairs >= 0) & (pairs <= 9))
    for i, j in pairs:
        assert abs(int(i) - int(j)) >= 2, (
            f"adjacent pair in topology: {(int(i), int(j))}")
    assert len(ALL_NONADJACENT_PAIRS) == 36
    assert len(OMITTED_NONADJACENT_PAIRS) == 22
    assert (3, 5) in OMITTED_NONADJACENT_PAIRS
    assert any(6 in pair for pair in OMITTED_NONADJACENT_PAIRS)


def test_mesh_checker_covers_every_non_adjacent_pair_including_link6():
    checker = FclMeshSelfCollisionChecker(str(MESH_DIR), max_faces=300)
    actual = {
        tuple(sorted((OBB_LINK_NAMES.index(left), OBB_LINK_NAMES.index(right))))
        for left, right in checker._check_pairs
    }
    assert actual == set(ALL_NONADJACENT_PAIRS)
    assert "Link6" in checker._mesh_objs
    assert (3, 5) in actual


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


def test_generated_obb_files_match_meshes_and_sampling_rules():
    result = subprocess.run(
        [sys.executable, str(GENERATOR), "--check"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_generated_yaml_marks_14_pairs_as_online_subset():
    document = yaml.safe_load(
        (REPO_ROOT / "portable_oscbf" / "config" / "obb_model.yaml")
        .read_text(encoding="utf-8")
    )
    assert document["sample_sphere_padding_m"] == 0.002
    assert document["sample_sphere_count"] == 32
    assert [entry["sample_grid_shape"] for entry in document["links"]] == (
        np.asarray(OBB_SAMPLE_GRID_SHAPES).tolist())
    assert document["collision_pairs_role"] == "online_cbf_subset"
    assert document["exclusions"] == []
    assert document["pair_policy"] == {
        "adjacent_pairs": "excluded",
        "all_nonadjacent_pair_count": 36,
        "online_pair_count": 14,
        "omitted_nonadjacent_pair_count": 22,
        "omitted_pair_evidence": "OFF-02 bounded-region certificate",
    }


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
