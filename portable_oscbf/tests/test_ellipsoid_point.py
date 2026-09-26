from __future__ import annotations

import jax
import jax.numpy as jnp
import mpmath as mp
import numpy as np
import pytest

from test_ellipsoid_dcol import _geometry, _module, _prepared, _sign_geometry, _world
from collision_scene_inputs import scene_input, signed_scene
from work.collision_safety import (
    CollisionScene, CollisionStatus, PrimitiveKind, QueryBatch, QueryMode, SupportTrackStatus,
)


def _environment(radii=(0.05, 0.05, 0.05), link="Link1", capacity=1, **limits):
    doc = _geometry(["base_link", link], [[5.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                    [[0.1, 0.1, 0.1], list(radii)], capacity=capacity)
    return doc, _module(doc, **{
        "max_primitive_rows": 8, "max_environment_pairs_per_ellipsoid": 3,
        "distance_max_iterations": 100, "distance_tolerance_mm": 1e-7, **limits,
    })


def _scene(module, points, rho=10.0, tracks=None, velocity=None):
    points = np.asarray(points, dtype=float).reshape((-1, 3))
    count = len(points)
    prepared = _prepared(module)
    scene = scene_input(module.config, prepared.identities,
                        points=np.pad(points, ((0, 3-count), (0, 0))),
                        radii=[rho] * count + [0.0] * (3-count), mask=np.arange(3) < count,
                        tracks=tracks, velocity=velocity)
    scene = signed_scene(scene._replace(support_ids=np.array([101, 202, 303], dtype=np.int32)))
    return module.prepare_scene(scene, prepared.identities)


def _run(module, scene, mode=QueryMode.DISTANCE_MM, q=None, active=(True, False)):
    q = np.zeros((2, 9)) if q is None else np.asarray(q)
    if q.shape == (9,):
        q = np.array([q, q])
    batch = QueryBatch(jnp.asarray(q), jnp.asarray(active))
    module.query(batch, scene, mode)
    return module.query(batch, scene, mode)


def _assert_distance_only(result):
    assert int(result.header.status) == CollisionStatus.OK
    assert np.array_equal(result.distance_valid_mask, [True, False])
    assert not np.any(result.valid_mask)
    assert not np.any(result.state_valid_mask)
    assert not np.any(result.state_valid)
    assert not np.any(result.solver_healthy)
    assert not np.any(result.barrier)
    assert not np.any(result.grad_h_q)
    assert not np.any(jax.jit(lambda value: value)(result).valid_mask)


@pytest.mark.parametrize("x", [0.2, 0.06, 0.060000001, 0.059999999, 0.055, 0.02, 0.0, -0.08])
def test_sphere_distance_signed_witness_clearance_and_barrier(x):
    _, module = _environment()
    scene = _scene(module, [[x, 0, 0]])
    distance = _run(module, scene)
    _assert_distance_only(distance)
    expected = (abs(x) - 0.05 - 0.01) * 1000
    assert distance.distance_mm[0] == pytest.approx(expected, abs=2e-6, rel=0)
    assert distance.required_clearance_mm[0] == 30
    assert distance.remaining_margin_mm[0] == pytest.approx(expected-30, abs=2e-6, rel=0)
    sign = -1 if x < 0 else 1
    np.testing.assert_allclose(distance.nearest_robot_point_m[0], [sign*0.05, 0, 0], atol=2e-9, rtol=0)
    np.testing.assert_allclose(distance.nearest_support_point_m[0], [x-sign*0.01, 0, 0], atol=2e-9, rtol=0)
    assert distance.nearest_link_index[0] == 1
    assert distance.nearest_ellipsoid_slot[0] == 0
    assert np.array_equal(distance.nearest_pair_id[0], [1, 101])
    assert int(distance.track_status[0]) == SupportTrackStatus.UNTRACKED
    assert distance.header.scene_revision == scene.identities.scene_revision
    assert distance.header.source_stamp_ns == scene.source_stamp_ns
    assert np.array_equal(distance.header.scene_epoch, scene.identities.scene_epoch)

    barrier = _run(module, scene, QueryMode.OSCBF_BARRIER)
    assert int(barrier.header.status) == CollisionStatus.OK
    rows = np.flatnonzero(np.asarray(barrier.valid_mask[0]) &
                          (np.asarray(barrier.primitive_kind[0]) == PrimitiveKind.ENVIRONMENT))
    assert len(rows) == 2
    row = rows[1]
    assert barrier.barrier[0, row] == pytest.approx((x/0.05)**2 - 1.8**2, abs=2e-9, rel=0)
    assert barrier.proximity_scale[0, row] == pytest.approx(abs(x)/0.05, abs=2e-9, rel=0)
    assert not np.any(barrier.distance_valid_mask)


def _reference(point, radii):
    # 直接求四维 KKT 的 Newton 根，并核对全局最近点的 multiplier 条件。
    with mp.workdps(80):
        p = list(map(mp.mpf, point))
        a = list(map(mp.mpf, radii))
        length = max(a)
        p, a = [value/length for value in p], [value/length for value in a]
        norm = mp.sqrt(sum((p[i]/a[i])**2 for i in range(3)))
        initial = [v/norm for v in p]
        if norm < 1:
            axis = a.index(min(a))
            initial = list(p)
            initial[axis] = mp.sign(p[axis]) * a[axis] * mp.sqrt(
                1-sum((p[i]/a[i])**2 for i in range(3) if i != axis))
        multiplier = min(a)*mp.sqrt(sum(value*value for value in p)) if norm > 1 else -min(a)**2/2

        def equations(x, y, z, t):
            w = [x, y, z]
            return (*(w[i] - p[i] + t*w[i]/a[i]**2 for i in range(3)),
                    sum((w[i]/a[i])**2 for i in range(3))-1)

        root = mp.findroot(equations, (*initial, multiplier), tol=mp.mpf("1e-65"), maxsteps=100)
        assert root[3] > -min(a)**2
        signed = mp.sqrt(sum((p[i]-root[i])**2 for i in range(3))) * (1 if norm > 1 else -1)
        return float(signed*length), np.array([float(v*length) for v in root[:3]])


@pytest.mark.parametrize("point", [[0.2, 0.1, 0.04], [0.018, 0.01, 0.005], [-0.08, 0.01, -0.03]])
def test_rotated_ellipsoid_distance_and_nine_axis_barrier_gradient(point):
    radii = [0.04, 0.03, 0.02]
    doc, module = _environment(radii, link="Link9")
    q = np.array([0.15, 0.1, -0.2, 0.3, 0.15, -0.25, 0.2, 0.05, -0.1])
    center, factor = _world(doc, q)[1]
    rotation = factor / np.asarray(radii)[None, :]
    world_point = center + rotation @ point
    scene = _scene(module, [world_point], rho=7.0)
    result = _run(module, scene, q=q)
    _assert_distance_only(result)
    reference, witness = _reference(point, radii)
    assert result.distance_mm[0] == pytest.approx((reference-0.007)*1000, abs=2e-6, rel=0)
    expected_robot = center + rotation @ witness
    normal = rotation @ (witness / np.asarray(radii)**2)
    normal /= np.linalg.norm(normal)
    np.testing.assert_allclose(result.nearest_robot_point_m[0], expected_robot, atol=2e-9, rtol=0)
    np.testing.assert_allclose(result.nearest_support_point_m[0], world_point-0.007*normal, atol=2e-9, rtol=0)
    np.testing.assert_allclose(result.nearest_support_point_m[0]-result.nearest_robot_point_m[0],
                               float(result.distance_mm[0])*1e-3*normal, atol=2e-9, rtol=0)
    barrier = _run(module, scene, QueryMode.OSCBF_BARRIER, q=q)
    row = 2
    assert barrier.valid_mask[0, row]
    assert barrier.barrier[0, row] == pytest.approx(
        np.sum((np.array(point)/radii)**2) - (1+0.037/min(radii))**2, abs=2e-9, rel=0)

    def high_precision_barrier(state):
        c, f = _world(doc, state)[1]
        with mp.workdps(80):
            u = mp.matrix(f.tolist())**-1 * mp.matrix((world_point-c).tolist())
            return float(sum(v*v for v in u))

    for joint in range(9):
        step = np.zeros(9)
        step[joint] = 1e-6
        reference_gradient = (high_precision_barrier(q+step)-high_precision_barrier(q-step))/2e-6
        assert barrier.grad_h_q[0, row, joint] == pytest.approx(reference_gradient, abs=2e-5, rel=0)


@pytest.mark.parametrize("radii,point,witness", [
    ([0.08, 0.04, 0.02], [0, 0, 0], [0, 0, 0.02]),
    ([0.08, 0.02, 0.02], [0, 0, 0], [0, 0.02, 0]),
    ([0.08, 0.04, 0.02], [0.075, 0, 0], [0.08, 0, 0]),
    ([0.08, 0.04, 0.02], [0.04, 0, 0], [0.08**2*0.04/(0.08**2-0.02**2), 0,
        0.02*np.sqrt(1-(0.08*0.04/(0.08**2-0.02**2))**2)]),
    ([0.08, 0.04, 0.02], [0, 0, 0.02], [0, 0, 0.02]),
    ([0.08, 0.04, 0.02], [0.08, 0, 0], [0.08, 0, 0]),
])
def test_axis_aligned_interior_singular_and_surface_points(radii, point, witness):
    _, module = _environment(radii)
    result = _run(module, _scene(module, [point], rho=0.0))
    _assert_distance_only(result)
    expected = -np.linalg.norm(np.array(point)-witness)*1000
    assert result.distance_mm[0] == pytest.approx(expected, abs=2e-6, rel=0)
    np.testing.assert_allclose(result.nearest_robot_point_m[0], witness, atol=2e-9, rtol=0)


def test_multiple_slots_supports_and_batch_choose_nearest_identity():
    doc, _ = _environment(capacity=2)
    doc["geometry"]["links"][1]["slots"][1] = {
        "center_m": [0.0, 0.0, 0.2], "radii_m": [0.03]*3, "active_mask": True,
    }
    _sign_geometry(doc)
    module = _module(doc, distance_max_iterations=100, distance_tolerance_mm=1e-7,
                     max_primitive_rows=12, max_environment_pairs_per_ellipsoid=3)
    scene = _scene(module, [[0, 0, 0.5], [0, 0, 0.25]], rho=10.0)
    q = np.zeros((2, 9))
    q[1, 0] = 0.3
    result = _run(module, scene, q=q, active=(True, True))
    assert int(result.header.status) == CollisionStatus.OK
    assert np.all(result.distance_valid_mask)
    assert np.array_equal(result.nearest_pair_id, [[3, 202], [3, 101]])
    assert np.array_equal(result.nearest_link_index, [1, 1])
    assert np.array_equal(result.nearest_ellipsoid_slot, [1, 1])
    assert np.array_equal(result.track_status, [SupportTrackStatus.TRACKED, SupportTrackStatus.UNTRACKED])
    np.testing.assert_allclose(result.distance_mm, [10, -40], atol=2e-6, rtol=0)
    assert not np.any(result.valid_mask)


@pytest.mark.parametrize("limits", [{"max_primitive_rows": 2}, {"max_environment_pairs_per_ellipsoid": 1}])
def test_environment_capacity_rejects_incomplete_rows(limits):
    _, module = _environment(**limits)
    scene = _scene(module, [[0.2, 0, 0], [0, 0.2, 0]])
    result = _run(module, scene, QueryMode.OSCBF_BARRIER)
    assert int(result.header.status) == CollisionStatus.CONSTRAINT_OVERFLOW
    assert not np.any(result.valid_mask)
    assert not np.any(result.state_valid_mask)
    _assert_distance_only(_run(module, scene))


def test_distance_iteration_limit_and_deadline_clear_all_admission_masks():
    _, module = _environment((0.04, 0.03, 0.02), distance_max_iterations=1)
    result = _run(module, _scene(module, [[0.11, 0.07, 0.03]]))
    assert int(result.header.status) == CollisionStatus.SOLVER_UNHEALTHY
    assert not np.any(result.distance_valid_mask)
    assert not np.any(result.valid_mask)
    _, late = _environment(query_deadline_ns=1)
    late_scene = _scene(late, [[0.2, 0, 0]])
    result = _run(late, late_scene)
    assert int(result.header.status) == CollisionStatus.DEADLINE_MISSED
    assert result.header.source_stamp_ns == late_scene.source_stamp_ns
    assert not np.any(result.distance_valid_mask)
    assert not np.any(result.state_valid_mask)
    assert not np.any(result.valid_mask)


def test_scene_status_identity_and_track_validation_reject_distance():
    _, module = _environment()
    scene = _scene(module, [[0.2, 0, 0], [0, 0.2, 0]])
    invalid_scenes = [scene._replace(status=jnp.int32(status)) for status in CollisionStatus if status != CollisionStatus.OK]
    invalid_scenes.extend([
        scene._replace(support_track_status=jnp.array([9, 0, 0], dtype=jnp.int32)),
        scene._replace(support_ids=jnp.array([101, 101, 0], dtype=jnp.int32)),
        scene._replace(support_ids=jnp.array([-1, 101, 0], dtype=jnp.int32)),
        scene._replace(identities=scene.identities._replace(kernel_version=jnp.zeros(32, dtype=jnp.uint8))),
    ])
    for invalid in invalid_scenes:
        result = _run(module, invalid)
        assert int(result.header.status) != CollisionStatus.OK
        assert result.header.source_stamp_ns == invalid.source_stamp_ns
        assert not np.any(result.distance_valid_mask)
        assert not np.any(result.valid_mask)
        assert not np.any(result.state_valid_mask)


def test_symmetric_environment_rows_and_support_time_derivative():
    _, module = _environment()
    scene = _scene(module, [[0, 0, 0.08], [0, 0, -0.08]], tracks=[1, 1, 0],
                   velocity=[[0., 0., -0.01], [0., 0., 0.01], [0., 0., 0.]])
    result = _run(module, scene, QueryMode.OSCBF_BARRIER)
    assert int(result.header.status) == CollisionStatus.OK
    rows = [3, 4]
    assert np.all(result.valid_mask[0, rows])
    np.testing.assert_allclose(result.grad_h_q[0, rows, 0], [-64, 64], atol=2e-9, rtol=0)
    np.testing.assert_allclose(result.partial_h_partial_t[0, rows], [-0.64, -0.64], atol=2e-9, rtol=0)
    assert np.all(result.barrier[0, rows] < 0)
    assert not result.state_valid[0]


@pytest.mark.parametrize("radii,point", [
    ([0.00001, 0.00002, 0.00003], [0.00004, 0.00003, 0.00002]),
    ([0.1, 0.0001, 0.03], [0.08, 0.0003, 0.02]),
    ([0.08, 0.04, 0.02], [0.04, 0.0, 1e-10]),
    ([0.08, 0.02, 0.02], [0.1, -0.03, 0.04]),
])
def test_extreme_aspect_ratio_and_near_singular_distance(radii, point):
    _, module = _environment(radii)
    result = _run(module, _scene(module, [point], rho=0.0))
    _assert_distance_only(result)
    reference, witness = _reference(point, radii)
    assert result.distance_mm[0] == pytest.approx(reference*1000, abs=2e-6, rel=0)
    np.testing.assert_allclose(result.nearest_robot_point_m[0], witness, atol=2e-9, rtol=0)


def test_seeded_distance_corpus_and_nonfinite_computation():
    radii = [0.04, 0.03, 0.02]
    _, module = _environment(radii)
    rng = np.random.default_rng(109)
    for _ in range(12):
        point = rng.uniform(0.02, 0.1, size=3) * rng.choice([-1, 1], size=3)
        result = _run(module, _scene(module, [point], rho=5.0))
        _assert_distance_only(result)
        reference, witness = _reference(point, radii)
        assert result.distance_mm[0] == pytest.approx(reference*1000-5, abs=2e-6, rel=0)
        np.testing.assert_allclose(result.nearest_robot_point_m[0], witness, atol=2e-9, rtol=0)
    for mode in QueryMode:
        result = _run(module, _scene(module, [[1e308, 0, 0]]), mode)
        assert int(result.header.status) in (CollisionStatus.SOLVER_UNHEALTHY, CollisionStatus.DEADLINE_MISSED)
        assert not np.any(result.valid_mask)
        assert not np.any(result.state_valid_mask)
        assert not np.any(result.distance_valid_mask)


def test_clearance_changes_barrier_and_margin_without_changing_distance():
    _, module = _environment()
    _, other = _environment(environment_clearance_mm=50.0)
    scene = _scene(module, [[0.12, 0, 0]], rho=8.0)
    other_scene = _scene(other, [[0.12, 0, 0]], rho=8.0)
    first, second = _run(module, scene), _run(other, other_scene)
    _assert_distance_only(first)
    _assert_distance_only(second)
    np.testing.assert_allclose(first.distance_mm, second.distance_mm, atol=0, rtol=0)
    assert second.remaining_margin_mm[0] == pytest.approx(float(first.remaining_margin_mm[0])-20, abs=2e-6)
    first_barrier = _run(module, scene, QueryMode.OSCBF_BARRIER)
    second_barrier = _run(other, other_scene, QueryMode.OSCBF_BARRIER)
    assert first_barrier.barrier[0, 2] == pytest.approx((0.12/0.05)**2-(1+0.038/0.05)**2, abs=2e-9)
    assert second_barrier.barrier[0, 2] == pytest.approx((0.12/0.05)**2-(1+0.058/0.05)**2, abs=2e-9)
    result = _run(other, scene)
    assert int(result.header.status) != CollisionStatus.OK
    assert not np.any(result.distance_valid_mask)
