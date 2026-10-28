import hashlib
import time
from dataclasses import replace

import numpy as np
import pytest
import trimesh
from test_collision_safety import _config, _identities
from work.collision_safety import CollisionSafety, CollisionStatus
from work.lidar_occupancy import (
    FixedEnvironmentModel,
    LidarOccupancyScene,
    LidarRayBatch,
    OccupancyGrid,
    RayReturn,
    VoxelState,
)


def _scene(shape=(4, 1, 1), required=None, **parameters):
    values = dict(
        lidar_range_error_mm=0.0, lidar_incidence_error_mm=0.0,
        extrinsic_translation_error_mm=0.0, lidar_angular_error_rad=0.0,
        extrinsic_rotation_error_rad=0.0,
    )
    values.update(parameters)
    config = _config(**values)
    grid = OccupancyGrid(np.zeros(3), shape, 1.0, required_mask=required)
    identities = _identities(config)._replace(
        required_space_hash=np.frombuffer(grid.required_space_hash, dtype=np.uint8),
    )
    producer = LidarOccupancyScene(config, identities, grid)
    return producer, CollisionSafety(config)


def _ray(stamp_ns, endpoint_x_m, *, scan_id=1, kind=RayReturn.HIT):
    return LidarRayBatch(
        origins_m=np.array([[-.001, .0005, .0005]]),
        endpoints_m=np.array([[endpoint_x_m, .0005, .0005]]),
        point_stamp_ns=np.array([stamp_ns], dtype=np.int64),
        return_kind=np.array([kind], dtype=np.int32),
        scan_id=scan_id,
    )


def test_hit_updates_ray_cells_and_preserves_unknown_required_space():
    producer, safety = _scene()
    now = time.monotonic_ns()
    producer.update(_ray(now, .0025), now_ns=now)
    snapshot = producer.snapshot(now_ns=now)
    np.testing.assert_array_equal(snapshot.cells[:, 0, 0], [
        VoxelState.FREE, VoxelState.FREE, VoxelState.OCCUPIED, VoxelState.UNKNOWN,
    ])
    assert not snapshot.scene.metadata.required_space_covered
    assert snapshot.scene.metadata.support_count == 1
    point = snapshot.scene.support_points_m[snapshot.scene.support_mask][0]
    np.testing.assert_allclose(point, [.0025, .0005, .0005], atol=1e-15)
    assert snapshot.scene.support_radii_mm[snapshot.scene.support_mask][0] >= np.sqrt(3) / 2
    prepared = safety.prepare_scene(snapshot.scene, snapshot.scene.identities)
    assert int(prepared.status) == CollisionStatus.UNKNOWN_REQUIRED_SPACE
    assert not np.any(prepared.support_valid_mask)


def test_only_three_independent_continuous_free_scans_clear_occupied():
    producer, safety = _scene(required=np.array([[[True]], [[False]], [[False]], [[False]]]))
    base = time.monotonic_ns() - 100_000_000
    producer.update(_ray(base, .0005), now_ns=base)
    for offset in range(1, 4):
        stamp = base + offset * 20_000_000
        producer.update(_ray(stamp, .006, scan_id=offset + 1), now_ns=stamp)
        snapshot = producer.snapshot(now_ns=stamp)
        expected = VoxelState.FREE if offset == 3 else VoxelState.OCCUPIED
        assert snapshot.cells[0, 0, 0] == expected
        prepared = safety.prepare_scene(snapshot.scene, snapshot.scene.identities)
        assert int(prepared.status) == CollisionStatus.OK


@pytest.mark.parametrize("kind", [RayReturn.NO_RETURN, RayReturn.OCCLUDED, RayReturn.AMBIGUOUS])
def test_missing_or_blocked_returns_cannot_refresh_old_occupied(kind):
    producer, safety = _scene()
    now = time.monotonic_ns()
    hit_stamp = now - 2_100_000_000
    producer.update(_ray(hit_stamp, .0025), now_ns=hit_stamp)
    producer.update(_ray(now, .006, scan_id=2, kind=kind), now_ns=now)
    snapshot = producer.snapshot(now_ns=now)
    assert snapshot.cells[2, 0, 0] == VoxelState.UNKNOWN
    assert snapshot.last_observed_ns[2, 0, 0] == hit_stamp
    assert not np.any(snapshot.scene.support_mask)
    prepared = safety.prepare_scene(snapshot.scene, snapshot.scene.identities)
    assert int(prepared.status) == CollisionStatus.UNKNOWN_REQUIRED_SPACE
    assert not np.any(prepared.support_valid_mask)


def test_support_radius_preserves_measurement_error_and_actual_observation_age():
    producer, safety = _scene(
        shape=(1, 1, 1), lidar_range_error_mm=1., lidar_incidence_error_mm=1.,
        extrinsic_translation_error_mm=1., lidar_angular_error_rad=.001,
        extrinsic_rotation_error_rad=.002, untracked_speed_mm_s=2.,
        untracked_acceleration_mm_s2=4.,
    )
    now = time.monotonic_ns()
    stamp = now - 1_000_000_000
    producer.update(_ray(stamp, .0005), now_ns=stamp)
    first = producer.snapshot(now_ns=stamp)
    aged = producer.snapshot(now_ns=now)
    minimum = np.sqrt(3) / 2 + 3. + 1.5 * .003 + 2. + 2.
    assert aged.scene.support_radii_mm[0] >= minimum
    assert aged.scene.support_radii_mm[0] > first.scene.support_radii_mm[0]
    assert int(aged.scene.source_stamp_ns) == stamp
    assert int(aged.scene.identities.scene_revision) > int(first.scene.identities.scene_revision)
    prepared = safety.prepare_scene(aged.scene, aged.scene.identities)
    assert int(prepared.status) == CollisionStatus.OK


def test_verified_fixed_mesh_survives_lidar_free_and_versions_its_geometry():
    producer, safety = _scene(shape=(1, 1, 1))
    mesh = trimesh.creation.box(extents=[.001, .001, .001])
    mesh.apply_translation([.0005, .0005, .0005])
    model = FixedEnvironmentModel.from_mesh(
        mesh, policy=producer.config.policy, model_revision=1,
        source_hash=hashlib.sha256(mesh.export(file_type="stl")).digest(),
        validation_evidence="解析立方体：边长 1 mm，体积 1 mm³，八个角点独立核验",
        error_mm=.1,
    )
    producer.set_fixed_environment(model)
    base = time.monotonic_ns() - 100_000_000
    for offset in range(3):
        stamp = base + offset * 20_000_000
        producer.update(_ray(stamp, .006, scan_id=offset + 1), now_ns=stamp)
    snapshot = producer.snapshot(now_ns=stamp)
    assert snapshot.cells[0, 0, 0] == VoxelState.OCCUPIED
    assert snapshot.scene.metadata.support_count == 1
    assert int(snapshot.scene.identities.fixed_environment_revision) == 1
    center = snapshot.scene.support_points_m[snapshot.scene.support_mask][0]
    radius_m = snapshot.scene.support_radii_mm[snapshot.scene.support_mask][0] * .001
    assert np.all(np.linalg.norm(mesh.vertices - center, axis=1) <= radius_m)
    assert int(safety.prepare_scene(snapshot.scene, snapshot.scene.identities).status) == CollisionStatus.OK
    moved = mesh.copy()
    moved.apply_translation([.001, 0, 0])
    replacement = FixedEnvironmentModel.from_mesh(
        moved, policy=producer.config.policy, model_revision=2,
        source_hash=hashlib.sha256(moved.export(file_type="stl")).digest(),
        validation_evidence="同一解析立方体沿 +X 平移 1 mm",
        error_mm=.1,
    )
    producer.set_fixed_environment(replacement)
    newer = producer.snapshot(now_ns=stamp)
    assert int(newer.scene.identities.fixed_environment_revision) == 2
    assert int(newer.scene.identities.scene_revision) > int(snapshot.scene.identities.scene_revision)
    assert int(safety.prepare_scene(snapshot.scene, newer.scene.identities).status) == CollisionStatus.INVALID_SCENE


def test_capacity_merge_contains_every_original_ball_and_preserves_oldest_time():
    producer, safety = _scene(shape=(4, 1, 1), max_support_points=2)
    now = time.monotonic_ns()
    origins = np.tile([-.001, .0005, .0005], (4, 1))
    endpoints = np.column_stack([np.arange(4) * .001 + .0005, np.full((4, 2), .0005)])
    stamps = np.array([now - 40_000_000, now - 30_000_000, now - 20_000_000, now - 10_000_000], dtype=np.int64)
    producer.update(LidarRayBatch(origins, endpoints, stamps, np.zeros(4, dtype=np.int32), 1), now_ns=now)
    snapshot = producer.snapshot(now_ns=now)
    assert snapshot.scene.metadata.support_count <= 2
    assert int(snapshot.scene.source_stamp_ns) == int(stamps.min())
    original_radii_mm = np.sqrt(3) / 2 + (now - stamps + 100) * 1e-9 + .5 * ((now - stamps + 100) * 1e-9) ** 2
    centers = snapshot.scene.support_points_m[snapshot.scene.support_mask]
    radii = snapshot.scene.support_radii_mm[snapshot.scene.support_mask]
    for point, radius in zip(endpoints, original_radii_mm):
        assert np.any(np.linalg.norm(centers - point, axis=1) * 1e3 + radius <= radii)
    assert int(safety.prepare_scene(snapshot.scene, snapshot.scene.identities).status) == CollisionStatus.OK


def test_unprovable_capacity_returns_overflow_through_real_prepare_scene():
    producer, safety = _scene(shape=(4, 1, 1), max_support_points=1, support_merge_max_points=2)
    now = time.monotonic_ns()
    endpoints = np.column_stack([np.arange(4) * .001 + .0005, np.full((4, 2), .0005)])
    producer.update(LidarRayBatch(
        np.tile([-.001, .0005, .0005], (4, 1)), endpoints,
        np.full(4, now, dtype=np.int64), np.zeros(4, dtype=np.int32), 1,
    ), now_ns=now)
    snapshot = producer.snapshot(now_ns=now)
    assert int(snapshot.scene.status) == CollisionStatus.CAPACITY_OVERFLOW
    prepared = safety.prepare_scene(snapshot.scene, snapshot.scene.identities)
    assert int(prepared.status) == CollisionStatus.CAPACITY_OVERFLOW
    assert not np.any(prepared.support_valid_mask)


@pytest.mark.parametrize("field,value", [
    ("point_stamp_ns", np.array([1.5])),
    ("return_kind", np.array([.5])),
    ("return_kind", np.array([2**32], dtype=np.int64)),
    ("point_stamp_ns", np.array([2**63], dtype=np.uint64)),
])
def test_ray_integer_fields_reject_fractional_and_overflow_values(field, value):
    values = {
        "origins_m": np.array([[-.001, .0005, .0005]]),
        "endpoints_m": np.array([[.0025, .0005, .0005]]),
        "point_stamp_ns": np.array([time.monotonic_ns()], dtype=np.int64),
        "return_kind": np.array([RayReturn.HIT], dtype=np.int32),
        "scan_id": 1,
    }
    values[field] = value
    with pytest.raises(ValueError):
        LidarRayBatch(**values)


def test_a_same_cell_large_error_hit_is_preserved_within_one_scan():
    producer, safety = _scene(shape=(1, 1, 1), lidar_angular_error_rad=.1)
    now = time.monotonic_ns()
    batch = LidarRayBatch(
        origins_m=np.array([[-.010, .0005, .0005], [-.001, .0005, .0005]]),
        endpoints_m=np.full((2, 3), .0005),
        point_stamp_ns=np.array([now - 10_000_000, now], dtype=np.int64),
        return_kind=np.zeros(2, dtype=np.int32), scan_id=1,
    )
    producer.update(batch, now_ns=now)
    snapshot = producer.snapshot(now_ns=now)
    assert snapshot.scene.support_radii_mm[0] >= np.sqrt(3) / 2 + 10.5 * .1
    assert int(safety.prepare_scene(snapshot.scene, snapshot.scene.identities).status) == CollisionStatus.OK


def test_duplicate_scan_and_multiple_rays_cannot_supply_independent_free_evidence():
    producer, safety = _scene(shape=(1, 1, 1))
    base = time.monotonic_ns() - 100_000_000
    producer.update(_ray(base, .0005), now_ns=base)
    stamp = base + 20_000_000
    producer.update(LidarRayBatch(
        np.tile([-.001, .0005, .0005], (6, 1)), np.tile([.006, .0005, .0005], (6, 1)),
        np.full(6, stamp, dtype=np.int64), np.zeros(6, dtype=np.int32), 2,
    ), now_ns=stamp)
    with pytest.raises(ValueError):
        producer.update(_ray(stamp, .006, scan_id=2), now_ns=stamp)
    producer.update(_ray(stamp, .006, scan_id=3), now_ns=stamp)
    snapshot = producer.snapshot(now_ns=stamp)
    assert snapshot.cells[0, 0, 0] == VoxelState.OCCUPIED
    assert snapshot.free_evidence_count[0, 0, 0] == 1
    assert int(safety.prepare_scene(snapshot.scene, snapshot.scene.identities).status) == CollisionStatus.OK


@pytest.mark.parametrize("interruption", ["no_return", "long_gap", "new_hit"])
def test_interrupted_free_sequence_requires_new_continuous_confirmation(interruption):
    producer, safety = _scene(shape=(1, 1, 1))
    base = time.monotonic_ns() - 400_000_000
    producer.update(_ray(base, .0005), now_ns=base)
    producer.update(_ray(base + 20_000_000, .006, scan_id=2), now_ns=base + 20_000_000)
    producer.update(_ray(base + 40_000_000, .006, scan_id=3), now_ns=base + 40_000_000)
    if interruption == "long_gap":
        last = base + 200_000_000
        producer.update(_ray(last, .006, scan_id=4), now_ns=last)
    else:
        stamp = base + 60_000_000
        producer.update(_ray(stamp, .0005, scan_id=4,
                             kind=RayReturn.NO_RETURN if interruption == "no_return" else RayReturn.HIT), now_ns=stamp)
        last = base + 80_000_000
        producer.update(_ray(last, .006, scan_id=5), now_ns=last)
    snapshot = producer.snapshot(now_ns=last)
    assert snapshot.cells[0, 0, 0] == VoxelState.OCCUPIED
    assert snapshot.free_evidence_count[0, 0, 0] == 1
    assert int(safety.prepare_scene(snapshot.scene, snapshot.scene.identities).status) == CollisionStatus.OK


def test_a_nearer_hit_preserves_the_occluded_occupied_voxel_and_its_time():
    producer, safety = _scene()
    base = time.monotonic_ns() - 100_000_000
    producer.update(_ray(base, .0025), now_ns=base)
    for offset in range(1, 4):
        stamp = base + offset * 20_000_000
        producer.update(_ray(stamp, .0005, scan_id=offset + 1), now_ns=stamp)
    snapshot = producer.snapshot(now_ns=stamp)
    assert snapshot.cells[2, 0, 0] == VoxelState.OCCUPIED
    assert snapshot.last_hit_ns[2, 0, 0] == base
    assert int(snapshot.scene.source_stamp_ns) == base
    assert int(safety.prepare_scene(snapshot.scene, snapshot.scene.identities).status) == CollisionStatus.UNKNOWN_REQUIRED_SPACE


def test_new_frame_does_not_refresh_unobserved_free_space():
    producer, safety = _scene()
    now = time.monotonic_ns()
    old = now - 2_100_000_000
    producer.update(_ray(old, .006), now_ns=old)
    producer.update(_ray(now, .0005, scan_id=2), now_ns=now)
    snapshot = producer.snapshot(now_ns=now)
    assert snapshot.cells[1, 0, 0] == VoxelState.UNKNOWN
    assert snapshot.last_observed_ns[1, 0, 0] == old
    assert int(safety.prepare_scene(snapshot.scene, snapshot.scene.identities).status) == CollisionStatus.UNKNOWN_REQUIRED_SPACE


def test_fully_observed_free_space_passes_prepare_without_obstacle_supports():
    producer, safety = _scene()
    now = time.monotonic_ns()
    producer.update(_ray(now, .006), now_ns=now)
    snapshot = producer.snapshot(now_ns=now)
    assert np.all(snapshot.cells == VoxelState.FREE)
    assert not np.any(snapshot.scene.support_mask)
    assert int(safety.prepare_scene(snapshot.scene, snapshot.scene.identities).status) == CollisionStatus.OK


def test_merge_range_limit_preserves_capacity_error_with_unknown_space():
    producer, safety = _scene(shape=(400, 1, 1), max_support_points=2)
    now = time.monotonic_ns()
    endpoints = np.array([[.0005, .0005, .0005], [.1505, .0005, .0005], [.3005, .0005, .0005]])
    producer.update(LidarRayBatch(np.tile([-.001, .0005, .0005], (3, 1)), endpoints,
                                 np.full(3, now, dtype=np.int64), np.zeros(3, dtype=np.int32), 1), now_ns=now)
    snapshot = producer.snapshot(now_ns=now)
    assert not snapshot.scene.metadata.required_space_covered
    assert snapshot.support_demand == 3
    prepared = safety.prepare_scene(snapshot.scene, snapshot.scene.identities)
    assert int(prepared.status) == CollisionStatus.CAPACITY_OVERFLOW
    assert not np.any(prepared.support_valid_mask)


def test_fixed_model_does_not_grant_coverage_outside_its_verified_envelope():
    producer, safety = _scene(shape=(2, 1, 1))
    mesh = trimesh.creation.box(extents=[.001, .001, .001])
    mesh.apply_translation([.0005, .0005, .0005])
    model = FixedEnvironmentModel.from_mesh(
        mesh, policy=producer.config.policy, model_revision=1,
        source_hash=hashlib.sha256(mesh.export(file_type="stl")).digest(),
        validation_evidence="解析立方体八个角点和正体积", error_mm=.1,
    )
    producer.set_fixed_environment(model)
    snapshot = producer.snapshot(now_ns=time.monotonic_ns())
    np.testing.assert_array_equal(snapshot.cells[:, 0, 0], [VoxelState.OCCUPIED, VoxelState.UNKNOWN])
    assert int(safety.prepare_scene(snapshot.scene, snapshot.scene.identities).status) == CollisionStatus.UNKNOWN_REQUIRED_SPACE


def test_large_fixed_model_keeps_capacity_demand_and_cannot_become_empty_ok_scene():
    producer, safety = _scene(shape=(1, 1, 1), max_support_points=1, support_merge_max_points=2)
    mesh = trimesh.creation.box(extents=[1., 1., 1.])
    model = FixedEnvironmentModel.from_mesh(
        mesh, policy=producer.config.policy, model_revision=1,
        source_hash=hashlib.sha256(mesh.export(file_type="stl")).digest(),
        validation_evidence="解析 1000 mm 立方体与封闭体积", error_mm=1.,
    )
    producer.set_fixed_environment(model)
    snapshot = producer.snapshot(now_ns=time.monotonic_ns())
    assert snapshot.support_demand > 2
    prepared = safety.prepare_scene(snapshot.scene, snapshot.scene.identities)
    assert int(prepared.status) == CollisionStatus.CAPACITY_OVERFLOW
    assert not np.any(prepared.support_valid_mask)


@pytest.mark.parametrize("parameter,value", [
    ("free_evidence_required_count", 1),
    ("free_evidence_min_interval_ns", 200_000_000),
    ("free_evidence_max_gap_ns", 3_000_000_000),
])
def test_evidence_thresholds_are_validated_by_the_versioned_artifact(parameter, value):
    with pytest.raises(ValueError):
        _scene(**{parameter: value})


def test_single_oversized_observation_reports_capacity_failure():
    producer, safety = _scene(shape=(1, 1, 1), lidar_range_error_mm=101.)
    now = time.monotonic_ns()
    producer.update(_ray(now, .0005), now_ns=now)
    snapshot = producer.snapshot(now_ns=now)
    prepared = safety.prepare_scene(snapshot.scene, snapshot.scene.identities)
    assert int(prepared.status) == CollisionStatus.CAPACITY_OVERFLOW
    assert not np.any(prepared.support_valid_mask)


@pytest.mark.parametrize("field", [
    "support_points_m", "support_radii_mm", "source_hash", "policy_hash",
    "geometry_hash", "validation_evidence", "model_revision",
])
def test_fixed_model_rejects_changed_content_with_unchanged_identity(field):
    producer, _ = _scene(shape=(1, 1, 1))
    mesh = trimesh.creation.box(extents=[.001, .001, .001])
    model = FixedEnvironmentModel.from_mesh(
        mesh, policy=producer.config.policy, model_revision=1,
        source_hash=hashlib.sha256(mesh.export(file_type="stl")).digest(),
        validation_evidence="解析立方体的封闭体积和八个角点", error_mm=.1,
    )
    value = getattr(model, field)
    if isinstance(value, bytes):
        changed = b"x" * 32
    elif isinstance(value, str):
        changed = value + "，独立内容检查"
    elif isinstance(value, int):
        changed = value + 1
    else:
        changed = np.array(value, copy=True)
        changed.flat[0] += .1
    with pytest.raises(ValueError):
        replace(model, **{field: changed})


@pytest.mark.parametrize("endpoint_x", [-.0002, .0012])
def test_outside_hit_uncertainty_entering_required_space_is_preserved(endpoint_x):
    producer, safety = _scene(shape=(1, 1, 1), lidar_range_error_mm=1.)
    base = time.monotonic_ns() - 50_000_000
    producer.update(_ray(base, .006), now_ns=base)
    stamp = base + 10_000_000
    producer.update(_ray(stamp, endpoint_x, scan_id=2), now_ns=stamp)
    snapshot = producer.snapshot(now_ns=stamp)
    assert snapshot.cells[0, 0, 0] == VoxelState.OCCUPIED
    points = snapshot.scene.support_points_m[snapshot.scene.support_mask]
    radii = snapshot.scene.support_radii_mm[snapshot.scene.support_mask]
    endpoint = np.array([endpoint_x, .0005, .0005])
    assert np.any(np.linalg.norm(points - endpoint, axis=1) * 1000 + 1. <= radii)
    assert int(safety.prepare_scene(snapshot.scene, snapshot.scene.identities).status) == CollisionStatus.OK


@pytest.mark.parametrize("previous", ["free", "occupied"])
def test_ambiguous_observation_revokes_required_coverage_until_valid_evidence(previous):
    producer, safety = _scene(shape=(1, 1, 1))
    base = time.monotonic_ns() - 50_000_000
    producer.update(_ray(base, .006 if previous == "free" else .0005), now_ns=base)
    stamp = base + 10_000_000
    producer.update(_ray(stamp, .0005, scan_id=2, kind=RayReturn.AMBIGUOUS), now_ns=stamp)
    snapshot = producer.snapshot(now_ns=stamp)
    assert not snapshot.scene.metadata.required_space_covered
    if previous == "free":
        assert snapshot.cells[0, 0, 0] == VoxelState.UNKNOWN
    else:
        assert snapshot.cells[0, 0, 0] == VoxelState.OCCUPIED
        assert np.any(snapshot.scene.support_mask)
    prepared = safety.prepare_scene(snapshot.scene, snapshot.scene.identities)
    assert int(prepared.status) == CollisionStatus.UNKNOWN_REQUIRED_SPACE
    assert not np.any(prepared.support_valid_mask)
    later = stamp + 10_000_000
    producer.update(_ray(later, .0005, scan_id=3), now_ns=later)
    resolved = producer.snapshot(now_ns=later)
    assert int(safety.prepare_scene(resolved.scene, resolved.scene.identities).status) == CollisionStatus.OK


def test_expired_free_observations_cannot_complete_a_current_confirmation_sequence():
    producer, safety = _scene(
        shape=(1, 1, 1), occupied_max_age_ns=10_000_000_000,
        free_max_age_ns=1_500_000_000, free_evidence_min_interval_ns=1_000_000_000,
        free_evidence_max_gap_ns=1_000_000_000,
    )
    base = time.monotonic_ns() - 3_000_000_000
    producer.update(_ray(base, .0005), now_ns=base)
    for scan_id in range(2, 5):
        stamp = base + (scan_id - 1) * 1_000_000_000
        producer.update(_ray(stamp, .006, scan_id=scan_id), now_ns=stamp)
    snapshot = producer.snapshot(now_ns=stamp)
    assert snapshot.cells[0, 0, 0] == VoxelState.OCCUPIED
    assert snapshot.free_evidence_count[0, 0, 0] == 2
    assert snapshot.free_evidence_stamp_ns[(0, 0, 0)] == (
        base + 2_000_000_000, base + 3_000_000_000,
    )
    assert np.any(snapshot.scene.support_mask)
    assert int(safety.prepare_scene(snapshot.scene, snapshot.scene.identities).status) == CollisionStatus.OK


def test_fixed_environment_owns_a_copy_and_rechecks_mutated_incoming_content():
    producer, safety = _scene(shape=(1, 1, 1))
    mesh = trimesh.creation.box(extents=[.001, .001, .001])
    mesh.apply_translation([.0005, .0005, .0005])
    model = FixedEnvironmentModel.from_mesh(
        mesh, policy=producer.config.policy, model_revision=1,
        source_hash=hashlib.sha256(mesh.export(file_type="stl")).digest(),
        validation_evidence="解析立方体的封闭体积和八个角点", error_mm=.1,
    )
    producer.set_fixed_environment(model)
    original_points = np.array(model.support_points_m, copy=True)
    model.support_points_m.setflags(write=True)
    model.support_points_m[0, 0] += 1.
    with pytest.raises(ValueError, match="model_hash"):
        producer.set_fixed_environment(model)
    snapshot = producer.snapshot(now_ns=time.monotonic_ns())
    np.testing.assert_allclose(snapshot.scene.support_points_m[snapshot.scene.support_mask], original_points)
    assert int(safety.prepare_scene(snapshot.scene, snapshot.scene.identities).status) == CollisionStatus.OK


def test_ambiguous_uncertainty_revokes_every_intersected_free_voxel():
    producer, safety = _scene(shape=(4, 1, 1), lidar_range_error_mm=1.)
    base = time.monotonic_ns() - 50_000_000
    producer.update(_ray(base, .010), now_ns=base)
    stamp = base + 10_000_000
    producer.update(_ray(stamp, .0015, scan_id=2, kind=RayReturn.AMBIGUOUS), now_ns=stamp)
    snapshot = producer.snapshot(now_ns=stamp)
    np.testing.assert_array_equal(snapshot.cells[:, 0, 0], [
        VoxelState.UNKNOWN, VoxelState.UNKNOWN, VoxelState.UNKNOWN, VoxelState.FREE,
    ])
    np.testing.assert_array_equal(snapshot.ambiguous_mask[:, 0, 0], [True, True, True, False])
    assert int(safety.prepare_scene(snapshot.scene, snapshot.scene.identities).status) == CollisionStatus.UNKNOWN_REQUIRED_SPACE


def test_ambiguous_return_prevents_same_scan_free_evidence_from_clearing_occupied():
    producer, safety = _scene(shape=(1, 1, 1))
    base = time.monotonic_ns() - 100_000_000
    producer.update(_ray(base, .0005), now_ns=base)
    for number in (1, 2):
        stamp = base + number * 20_000_000
        producer.update(_ray(stamp, .006, scan_id=number + 1), now_ns=stamp)
    stamp = base + 60_000_000
    producer.update(LidarRayBatch(
        np.tile([-.001, .0005, .0005], (2, 1)),
        np.array([[.006, .0005, .0005], [.0005, .0005, .0005]]),
        np.full(2, stamp, dtype=np.int64),
        np.array([RayReturn.HIT, RayReturn.AMBIGUOUS], dtype=np.int32), 4,
    ), now_ns=stamp)
    snapshot = producer.snapshot(now_ns=stamp)
    assert snapshot.cells[0, 0, 0] == VoxelState.OCCUPIED
    assert snapshot.free_evidence_count[0, 0, 0] == 0
    assert int(safety.prepare_scene(snapshot.scene, snapshot.scene.identities).status) == CollisionStatus.UNKNOWN_REQUIRED_SPACE
