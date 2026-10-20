from pathlib import Path

import numpy as np
import pytest
from scipy.optimize import minimize
from scipy.spatial import ConvexHull
import trimesh
from robot_static_fcl import Body

from robot_safecontrol_moveit.oscbf_trajectory import bootstrap_portable
from robot_safecontrol_moveit.static_experiment_config import ExperimentConfig

ROOT = Path(__file__).resolve().parents[1]
bootstrap_portable(ROOT / "portable_oscbf")

from work.static_collision import StaticGeometry, distance_between
from work.static_collision_meshes import load_meshes, mesh_sources


def test_closed_mesh_sources_and_exact_vertices_are_preserved():
    source = mesh_sources(ROOT / "portable_oscbf/config/collision_geometry_sources.json")
    meshes = load_meshes(source)
    assert len(meshes) == 10
    assert len(source["pairs"]) == 36
    expected = {(first, second) for first in range(10) for second in range(first + 2, 10)}
    assert {tuple(pair) for pair in source["pairs"]} == expected
    for mesh, record in zip(meshes, source["meshes"]):
        independent = trimesh.load_mesh(record["path"])
        np.testing.assert_array_equal(mesh.vertices, independent.vertices)
        np.testing.assert_array_equal(mesh.faces, independent.faces)
        assert mesh.radius_m >= np.linalg.norm(mesh.vertices, axis=1).max()


def test_changed_mesh_identity_is_rejected():
    source = mesh_sources(ROOT / "portable_oscbf/config/collision_geometry_sources.json")
    source["meshes"][3]["sha256"] = "0" * 64
    with pytest.raises(ValueError, match="SHA256"):
        load_meshes(source)


@pytest.mark.parametrize("translation,extent,expected", [
    ([.7, 0, 0], .4, "separated"),
    ([.4, 0, 0], .4, "collision"),
    ([.3, 0, 0], .4, "collision"),
    ([0, 0, 0], .04, "collision"),
])
def test_closed_mesh_contact_overlap_and_containment(translation, extent, expected):
    first = trimesh.creation.box(extents=[.4] * 3)
    second = trimesh.creation.box(extents=[extent] * 3)
    first_body = Body.convex(first.vertices, first.faces)
    second_body = Body.convex(second.vertices, second.faces)
    second_body.set_transform(np.eye(3), np.asarray(translation, dtype=float))
    distance = distance_between(first_body, second_body)
    assert distance.status == expected
    assert distance.valid == (expected == "separated")
    if distance.valid:
        assert distance.distance_m == pytest.approx(.3, abs=1e-9)
    else:
        assert distance.first_point is None and distance.second_point is None


def _independent_distance(vertices_first, vertices_second):
    planes = [ConvexHull(vertices).equations for vertices in (vertices_first, vertices_second)]
    matrix = np.zeros((sum(len(value) for value in planes), 6))
    matrix[:len(planes[0]), :3] = planes[0][:, :3]
    matrix[len(planes[0]):, 3:] = planes[1][:, :3]
    upper = -np.concatenate([plane[:, 3] for plane in planes])

    def objective(points):
        difference = points[3:] - points[:3]
        return .5 * difference @ difference, np.r_[-difference, difference]

    result = minimize(
        objective, np.r_[vertices_first.mean(axis=0), vertices_second.mean(axis=0)],
        jac=True, method="SLSQP",
        constraints={"type": "ineq", "fun": lambda points: upper - matrix @ points,
                     "jac": lambda points: -matrix},
        options={"ftol": 1e-14, "maxiter": 1000},
    )
    assert result.success, result.message
    assert np.max(matrix @ result.x - upper) <= 1e-10
    return np.linalg.norm(result.x[3:] - result.x[:3])


def test_all_self_pairs_and_gradients_match_independent_queries():
    config = ExperimentConfig.load(ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT)
    scene = config.scene()
    geometry = StaticGeometry(scene)
    q = config.initial_position
    rows = [row for row in geometry.query(q) if row.kind == "self"]
    assert len(rows) == 36
    assert all(row.required_m == .01 and row.distance.valid for row in rows)
    assert min(row.distance.distance_m for row in rows) >= .01
    target = min(rows, key=lambda row: row.distance.distance_m)
    meshes = load_meshes(scene["self_collision"])
    transforms = geometry.kinematics.forward_kinematics(q)
    vertices = {
        mesh.link: mesh.vertices @ transforms[mesh.link][:3, :3].T + transforms[mesh.link][:3, 3]
        for mesh in meshes
    }
    assert target.distance.distance_m == pytest.approx(
        _independent_distance(*(vertices[name] for name in target.pair)), abs=1e-8,
    )
    epsilon = 2e-6
    finite = []
    for step in epsilon * np.eye(9):
        plus = {row.pair: row.distance.distance_m for row in geometry.query(q + step)}
        minus = {row.pair: row.distance.distance_m for row in geometry.query(q - step)}
        finite.append((plus[target.pair] - minus[target.pair]) / (2 * epsilon))
    np.testing.assert_allclose(target.gradient, finite, atol=2e-5)
    np.testing.assert_allclose(target.gradient[:3], 0, atol=1e-12)


def test_every_self_gradient_matches_finite_differences():
    config = ExperimentConfig.load(ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT)
    geometry = StaticGeometry(config.scene())
    q = config.initial_position + np.array([.001, .01, -.02, .015, -.01, .02, -.01, .01, -.02])
    rows = [row for row in geometry.query(q) if row.kind == "self"]
    assert len(rows) == 36 and all(row.distance.valid for row in rows)
    finite = []
    for step in np.eye(9) * 2e-6:
        plus = [row.distance.distance_m for row in geometry.query(q + step) if row.kind == "self"]
        minus = [row.distance.distance_m for row in geometry.query(q - step) if row.kind == "self"]
        finite.append((np.array(plus) - minus) / 4e-6)
    np.testing.assert_allclose(np.array([row.gradient for row in rows]), np.array(finite).T, atol=3e-5)


def test_closest_self_pair_changes_and_intersection_has_no_gradient():
    config = ExperimentConfig.load(ROOT / "config/static_experiment_s0.yaml", resource_root=ROOT)
    geometry = StaticGeometry(config.scene())
    first = [row for row in geometry.query(config.initial_position) if row.kind == "self"]
    q = np.array([
        .5610871585363391, 1.109274689614743, -.11432832268201332,
        -.8213535687386871, -2.751676740221647, -1.3727219851907795,
        1.4564652580335808, 1.3963494994681076, .39895462501701884,
    ])
    second = [row for row in geometry.query(q) if row.kind == "self"]
    assert min(first, key=lambda row: row.distance.distance_m).pair != min(
        second, key=lambda row: row.distance.distance_m,
    ).pair
    assert all(row.distance.valid for row in second)
    intersecting_q = np.array([
        .010020933078072601, -.18142433735842833, -1.3532870341361734,
        -1.4186785575682663, 2.5216669723618232, .7238508762788514,
        .5082563556425679, 1.3252169416531787, -.45214738627998274,
    ])
    intersections = [
        row for row in geometry.query(intersecting_q)
        if row.kind == "self" and row.distance.status == "collision"
    ]
    assert intersections
    assert all(not row.distance.valid and row.gradient is None for row in intersections)
