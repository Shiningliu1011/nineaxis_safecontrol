import hashlib
import json
import pickle
import time
from dataclasses import replace

import jax.numpy as jnp
import numpy as np
import pytest
from test_collision_safety import _config, _identities
from work.collision_safety import CollisionSafety, CollisionStatus
from work.lidar_occupancy import (
    LidarOccupancyScene,
    LidarRayBatch,
    OccupancyGrid,
    RayReturn,
)


def _producer(shape=(1, 1, 1), origin=(0., 0., 0.), **changes):
    parameters = {"lidar_range_error_mm": 0., "lidar_incidence_error_mm": 0.,
                  "extrinsic_translation_error_mm": 0., "lidar_angular_error_rad": 0.,
                  "extrinsic_rotation_error_rad": 0.,
                  "untracked_speed_mm_s": 50., "untracked_acceleration_mm_s2": 4.}
    parameters.update(changes)
    config = _config(**parameters)
    grid = OccupancyGrid(np.array(origin), shape, 1., required_mask=np.zeros(shape, dtype=bool))
    identities = _identities(config)._replace(
        required_space_hash=np.frombuffer(grid.required_space_hash, dtype=np.uint8))
    return LidarOccupancyScene(config, identities, grid), CollisionSafety(config)


def _hit(producer, stamp, *, scan=1, point=None):
    endpoint = (producer.grid.origin_m + .0005 if point is None else np.asarray(point))
    producer.update(LidarRayBatch(
        np.array([endpoint - [.001, 0., 0.]]), np.array([endpoint]),
        np.array([stamp], dtype=np.int64), np.array([RayReturn.HIT], dtype=np.int32), scan), now_ns=stamp)


def test_untracked_future_tube_preserves_original_observation_age_once():
    producer, _ = _producer(untracked_speed_mm_s=2.)
    now = time.monotonic_ns()
    stamp = now - 500_000_000
    _hit(producer, stamp)
    snapshot = producer.snapshot(now_ns=now)
    assert snapshot.scene.support_time_model is not None
    from work.dynamic_support import reachable_support_tube

    finish = now + 500_000_000
    tube = reachable_support_tube(snapshot.scene, now, finish, policy=producer.config.policy)
    age = (finish - stamp + 100) * 1e-9
    expected = np.sqrt(3) / 2 + 2. * age + 2. * age ** 2
    assert tube.valid_mask[0]
    assert tube.end_radii_mm[0] == pytest.approx(expected, abs=3e-14)
    assert tube.radius_rate_end_mm_s[0] == pytest.approx(2. + 4. * age)
    assert tube.enclosing_radii_mm[0] >= expected
    assert int(snapshot.scene.source_stamp_ns) == stamp
    observations = producer.observations()
    assert observations[0].point_stamp_ns == stamp
    assert observations[0].measurement_error_mm == 0.
    assert observations[0].geometry_radius_mm == pytest.approx(np.sqrt(3) / 2)
    assert not observations[0].point_m.flags.writeable


def _tracking(producer, stamp, velocity=(-.01, 0., 0.), until=None, ids=None, track_id=7):
    from work.dynamic_support import TrackingFrame, observation_hash
    records = producer.observations()
    ids = sorted({item.support_id for item in records}) if ids is None else ids
    document = {
        "schema_version": 1, "scene_epoch": np.asarray(producer.identities.scene_epoch).tobytes().hex(),
        "required_space_hash": producer.grid.required_space_hash.hex(),
        "policy_hash": producer.config.collision_policy_hash.hex(), "clock_id": "monotonic",
        "tracks": [{
            "track_id": track_id, "support_ids": ids,
            "observation_hashes": [observation_hash(tuple(item for item in records if item.support_id == sid)).hex()
                                   for sid in ids],
            "velocity_m_s": velocity, "position_error_mm": .2, "velocity_error_mm_s": .3,
            "acceleration_bound_m_s2": .002, "timestamp_error_ns": 200,
            "updated_stamp_ns": stamp, "valid_until_ns": stamp + 1_000_000_000 if until is None else until,
            "valid": True,
            "evidence": {
                "source_hash": hashlib.sha256(b"analytic source: velocity and interval acceleration").hexdigest(),
                "validation_hash": hashlib.sha256(b"independent: quadratic envelope and finite difference").hexdigest(),
                "method": "解析位置与速度误差，二次可达范围的独立公式检查",
                "device": producer.config.policy.artifact.device,
                "scenario": producer.config.policy.artifact.scenario,
            },
        }],
    }
    return TrackingFrame.from_dict(json.loads(json.dumps(document)))


@pytest.mark.parametrize("velocity", [(-.01, 0., 0.), (.01, 0., 0.), (0., .01, 0.)])
def test_tracking_replay_changes_motion_without_changing_occupancy(velocity):
    from work.dynamic_support import reachable_support_tube
    producer, _ = _producer()
    stamp = time.monotonic_ns()
    _hit(producer, stamp)
    frame = _tracking(producer, stamp, velocity)
    snapshot = producer.snapshot(now_ns=stamp, tracking=frame)
    tube = reachable_support_tube(snapshot.scene, stamp, stamp + 500_000_000, policy=producer.config.policy)
    np.testing.assert_allclose(tube.end_points_m[0], [.0005, .0005, .0005] + np.array(velocity) * .5,
                               atol=1e-16, rtol=0)
    assert snapshot.cells[0, 0, 0] == 2
    assert snapshot.scene.support_track_status[0] == 1
    assert snapshot.scene.support_track_ids[0] == 7
    tau = 200e-9
    history_age = 100e-9 + tau
    history = 50. * history_age + 2. * history_age ** 2
    expected = np.sqrt(3) / 2 + history + .2 + np.linalg.norm(velocity) * 1000 * tau
    expected += .3 * (.5 + tau) + (.5 + tau) ** 2
    assert tube.end_radii_mm[0] == pytest.approx(expected, abs=2e-14)
    assert tube.radius_rate_end_mm_s[0] == pytest.approx(.3 + 2. * (.5 + tau), abs=1e-14)


def _geometry_producer():
    from test_ellipsoid_dcol import _prepared
    from test_ellipsoid_point import _environment
    _, module = _environment(lidar_range_error_mm=0., lidar_incidence_error_mm=0.,
                            extrinsic_translation_error_mm=0., lidar_angular_error_rad=0.,
                            extrinsic_rotation_error_rad=0., untracked_speed_mm_s=50.,
                            untracked_acceleration_mm_s2=4.)
    grid = OccupancyGrid(np.array([.1995, -.0005, -.0005]), (1, 1, 1), 1.,
                         required_mask=np.zeros((1, 1, 1), dtype=bool))
    identities = _prepared(module).identities._replace(
        required_space_hash=np.frombuffer(grid.required_space_hash, dtype=np.uint8))
    return LidarOccupancyScene(module.config, identities, grid), module


def _tracked_scene(producer, module, velocity=(-.01, 0., 0.), scan=1):
    stamp = time.monotonic_ns()
    _hit(producer, stamp, scan=scan)
    snapshot = producer.snapshot(now_ns=stamp, tracking=_tracking(producer, stamp, velocity))
    return stamp, module.prepare_scene(snapshot.scene, snapshot.scene.identities)


def test_query_rows_share_predicted_support_for_distance_barrier_and_radius_derivative():
    from work.collision_safety import PrimitiveKind, QueryBatch, QueryMode
    producer, module = _geometry_producer()
    stamp, prepared = _tracked_scene(producer, module)
    def run(mode, times):
        return module.query(QueryBatch(jnp.zeros((2, 9)), jnp.ones(2, dtype=bool),
                                       query_time_ns=jnp.asarray(times, dtype=jnp.int64)), prepared, mode)
    run(QueryMode.OSCBF_BARRIER, [stamp + 100_000_000, stamp + 500_000_000])
    stamp, prepared = _tracked_scene(producer, module, scan=2)
    run(QueryMode.DISTANCE_MM, [stamp + 100_000_000, stamp + 500_000_000])
    stamp, prepared = _tracked_scene(producer, module, scan=3)
    times = [stamp + 100_000_000, stamp + 500_000_000]
    barrier = run(QueryMode.OSCBF_BARRIER, times)
    distance = run(QueryMode.DISTANCE_MM, times)
    assert int(barrier.header.status) == CollisionStatus.OK
    assert int(distance.header.status) == CollisionStatus.OK
    rows = np.flatnonzero((np.asarray(barrier.primitive_kind[0]) == PrimitiveKind.ENVIRONMENT)
                          & (np.asarray(barrier.primitive_pair_id[0])[:, 0] == 1))
    assert len(rows) == 1
    row = rows[0]
    from work.dynamic_support import evaluate_support_time
    points, radii, rates = evaluate_support_time(prepared, np.asarray(times, dtype=np.int64), np)
    for index in range(2):
        x, rho, rate = points[index, 0, 0], radii[index, 0], rates[index, 0]
        beta = 1. + (rho + 30.) / 50.
        expected = (x / .05) ** 2 - beta ** 2
        derivative = 2. * x * -.01 / .05 ** 2 - 2. * beta * rate / 50.
        assert barrier.barrier[index, row] == pytest.approx(expected, abs=2e-12)
        assert barrier.partial_h_partial_t[index, row] == pytest.approx(derivative, abs=2e-12)
        assert distance.distance_mm[index] == pytest.approx((x - .05) * 1000 - rho, abs=2e-6)
    middle, delta = stamp + 300_000_000, 100_000
    difference = run(QueryMode.OSCBF_BARRIER, [middle - delta, middle + delta])
    numeric = (difference.barrier[1, row] - difference.barrier[0, row]) / (2. * delta * 1e-9)
    middle_result = run(QueryMode.OSCBF_BARRIER, [middle, middle])
    assert numeric == pytest.approx(float(middle_result.partial_h_partial_t[0, row]), abs=2e-8)


def test_track_interruption_expiry_and_reassociation_keep_occupied_and_full_history():
    from work.dynamic_support import reachable_support_tube
    producer, _ = _producer()
    stamp = time.monotonic_ns()
    _hit(producer, stamp)
    frame = _tracking(producer, stamp, until=stamp + 200_000_000)
    tracked = producer.snapshot(now_ns=stamp, tracking=frame)
    interrupted = producer.snapshot(now_ns=stamp + 100_000_000)
    expired = producer.snapshot(now_ns=stamp + 300_000_000, tracking=frame)
    restored_frame = _tracking(producer, stamp + 400_000_000)
    restored = producer.snapshot(now_ns=stamp + 400_000_000, tracking=restored_frame)
    for snapshot in (tracked, interrupted, expired, restored):
        assert snapshot.cells[0, 0, 0] == 2
        assert snapshot.scene.support_mask[0]
        assert int(snapshot.scene.source_stamp_ns) == stamp
    assert interrupted.scene.support_track_status[0] == expired.scene.support_track_status[0] == 0
    assert not np.any(expired.scene.tracks.active_mask)
    tube = reachable_support_tube(expired.scene, stamp + 300_000_000, stamp + 600_000_000,
                                  policy=producer.config.policy)
    age = .6 + 100e-9
    assert tube.end_radii_mm[0] == pytest.approx(np.sqrt(3) / 2 + 50 * age + 2 * age ** 2, abs=2e-14)
    assert restored.scene.support_track_status[0] == 1
    assert int(restored.scene.support_time_model.source_stamp_ns[0, 0]) == stamp


def test_new_observation_revokes_old_association_without_removing_occupancy():
    producer, _ = _producer()
    stamp = time.monotonic_ns()
    _hit(producer, stamp)
    frame = _tracking(producer, stamp)
    _hit(producer, stamp + 10_000_000, scan=2)
    snapshot = producer.snapshot(now_ns=stamp + 10_000_000, tracking=frame)
    assert snapshot.cells[0, 0, 0] == 2
    assert snapshot.scene.support_track_status[0] == 0
    assert len(producer.observations()) == 2
    assert not np.any(snapshot.scene.tracks.active_mask)


def test_mixed_track_merge_encloses_every_member_for_entire_future_interval():
    from work.dynamic_support import evaluate_support_time, reachable_support_tube
    producer, safety = _producer(shape=(2, 1, 1), max_support_points=1)
    stamp = time.monotonic_ns()
    producer.update(LidarRayBatch(
        np.array([[-.001, .0005, .0005], [-.001, .0005, .0005]]),
        np.array([[.0005, .0005, .0005], [.0015, .0005, .0005]]),
        np.array([stamp, stamp], dtype=np.int64), np.array([0, 0], dtype=np.int32), 1), now_ns=stamp)
    first = _tracking(producer, stamp, velocity=(-.01, 0, 0), ids=[0], track_id=7)
    second = _tracking(producer, stamp, velocity=(.02, .01, 0), ids=[1], track_id=8)
    frame = replace(first, tracks=first.tracks + second.tracks)
    snapshot = producer.snapshot(now_ns=stamp, tracking=frame)
    scene = snapshot.scene
    assert scene.support_track_status[0] == 2
    assert scene.support_track_ids[0] == -1
    assert int(safety.prepare_scene(scene, scene.identities).status) == CollisionStatus.OK
    model = scene.support_time_model
    source_mask = model.source_mask[0]
    source_points = model.source_points_m[0, source_mask]
    source_radii = model.source_radii_mm[0, source_mask]
    source_velocity = model.source_velocity_m_s[0, source_mask]
    source_rates = model.source_radius_rate_mm_s[0, source_mask]
    source_acceleration = model.source_acceleration_mm_s2[0, source_mask]
    initial_bound = np.linalg.norm(source_points - scene.support_points_m[0], axis=1) * 1000 + source_radii
    derivative_bound = np.linalg.norm(source_velocity - scene.support_velocity_m_s[0], axis=1) * 1000 + source_rates
    assert np.all(scene.support_radii_mm[0] >= initial_bound)
    assert np.all(model.radius_rate_mm_s[0] >= derivative_bound)
    assert np.all(model.radius_acceleration_mm_s2[0] >= source_acceleration)
    times = stamp + np.arange(1001, dtype=np.int64) * 1_000_000
    centers, radii, _ = evaluate_support_time(scene, times, np)
    elapsed = ((times - stamp) * 1e-9)[:, None]
    member_points = source_points[None, :, :] + elapsed[:, :, None] * source_velocity
    member_radii = source_radii + elapsed * source_rates + .5 * elapsed ** 2 * source_acceleration
    required = np.linalg.norm(member_points - centers[:, :1, :], axis=-1) * 1000 + member_radii
    assert np.all(required <= radii[:, :1] + 2e-14)
    tube = reachable_support_tube(scene, stamp, stamp + 1_000_000_000, policy=producer.config.policy)
    required_interval = np.linalg.norm(member_points - tube.enclosing_points_m[0], axis=-1) * 1000 + member_radii
    assert np.all(required_interval <= tube.enclosing_radii_mm[0] + 2e-14)


def test_public_tube_rejects_changed_content_and_resigned_invalid_envelope():
    from collision_scene_inputs import signed_scene
    from work.dynamic_support import reachable_support_tube
    producer, safety = _producer()
    stamp = time.monotonic_ns()
    _hit(producer, stamp)
    scene = producer.snapshot(now_ns=stamp).scene
    radii = scene.support_radii_mm.copy()
    radii[0] *= .5
    changed = scene._replace(support_radii_mm=radii)
    assert not np.any(reachable_support_tube(changed, stamp, stamp + 100_000_000,
                                            policy=producer.config.policy).valid_mask)
    resigned = signed_scene(changed)
    assert not np.any(reachable_support_tube(resigned, stamp, stamp + 100_000_000,
                                            policy=producer.config.policy).valid_mask)
    assert int(safety.prepare_scene(resigned, resigned.identities).status) == CollisionStatus.INVALID_SCENE


@pytest.mark.parametrize("field", [
    "reference_stamp_ns", "scene_epoch", "run_clock_hash", "policy_hash", "radius_limit_mm",
    "radius_rate_mm_s", "radius_acceleration_mm_s2", "valid_until_ns", "source_mask", "source_ids",
    "source_points_m", "source_radii_mm", "source_velocity_m_s", "source_radius_rate_mm_s",
    "source_acceleration_mm_s2", "source_stamp_ns", "source_valid_until_ns", "source_track_ids",
    "source_history_hash",
])
def test_checksum_binds_every_time_model_field(field):
    producer, safety = _producer()
    stamp = time.monotonic_ns()
    _hit(producer, stamp)
    scene = producer.snapshot(now_ns=stamp).scene
    model = scene.support_time_model
    value = np.array(getattr(model, field), copy=True)
    value.flat[-1] = not value.flat[-1] if value.dtype.kind == "b" else value.flat[-1] + 1
    changed = scene._replace(support_time_model=model._replace(**{field: value}))
    assert int(safety.prepare_scene(changed, changed.identities).status) == CollisionStatus.INVALID_SCENE


def test_time_model_shape_and_dtype_validation_covers_all_nested_fields():
    producer, safety = _producer()
    stamp = time.monotonic_ns()
    _hit(producer, stamp)
    scene = producer.snapshot(now_ns=stamp).scene
    model = scene.support_time_model
    for field, original in model._asdict().items():
        for value, error in ((np.zeros((5, 2), dtype=np.asarray(original).dtype), ValueError),
                             (np.asarray(original).astype(np.complex128), TypeError)):
            changed = scene._replace(support_time_model=model._replace(**{field: value}))
            with pytest.raises(error, match=f"support_time_model.{field}"):
                safety.prepare_scene(changed, changed.identities)


@pytest.mark.parametrize("field,value", [
    ("radius_rate_mm_s", 0.), ("radius_acceleration_mm_s2", 0.), ("source_radii_mm", -1.),
    ("source_velocity_m_s", .01), ("source_track_ids", 999), ("source_mask", False),
    ("source_history_hash", 0), ("scene_epoch", 3), ("run_clock_hash", 0),
    ("policy_hash", 0), ("radius_limit_mm", 101.),
])
def test_resigned_invalid_time_data_remains_rejected(field, value):
    from collision_scene_inputs import signed_scene
    producer, safety = _producer()
    stamp = time.monotonic_ns()
    _hit(producer, stamp)
    scene = producer.snapshot(now_ns=stamp).scene
    model = scene.support_time_model
    array = np.array(getattr(model, field), copy=True)
    if array.ndim == 0:
        array[...] = value
    else:
        array[0] = value
    scene = signed_scene(scene._replace(support_time_model=model._replace(**{field: array})))
    prepared = safety.prepare_scene(scene, scene.identities)
    assert int(prepared.status) == CollisionStatus.INVALID_SCENE
    assert not np.any(prepared.support_valid_mask)


def test_future_validity_uses_exact_ns_boundaries_and_rejects_missing_time_data():
    from work.collision_safety import QueryBatch, QueryMode
    from work.dynamic_support import reachable_support_tube
    producer, safety = _producer()
    stamp = time.monotonic_ns()
    _hit(producer, stamp)
    until = stamp + 1_000_000_000
    scene = producer.snapshot(now_ns=stamp, tracking=_tracking(producer, stamp, until=until)).scene
    assert reachable_support_tube(scene, stamp, until, policy=producer.config.policy).valid_mask[0]
    assert not np.any(reachable_support_tube(scene, stamp, until + 1, policy=producer.config.policy).valid_mask)
    assert not np.any(reachable_support_tube(scene, stamp - 1, until, policy=producer.config.policy).valid_mask)
    prepared = safety.prepare_scene(scene, scene.identities)
    q, mask = jnp.zeros((2, 9)), jnp.ones(2, dtype=bool)
    batch = QueryBatch(q, mask, jnp.array([until, until + 1], dtype=jnp.int64))
    safety.query(batch, prepared, QueryMode.DISTANCE_MM)
    result = safety.query(batch, prepared, QueryMode.DISTANCE_MM)
    assert int(result.header.status) == CollisionStatus.UNKNOWN_REQUIRED_SPACE
    assert not np.any(result.distance_valid_mask)
    from collision_scene_inputs import signed_scene
    legacy = signed_scene(scene._replace(support_time_model=None))
    prepared_legacy = safety.prepare_scene(legacy, legacy.identities)
    batch = QueryBatch(q, mask, jnp.array([until, until], dtype=jnp.int64))
    safety.query(batch, prepared_legacy, QueryMode.DISTANCE_MM)
    result = safety.query(batch, prepared_legacy, QueryMode.DISTANCE_MM)
    assert int(result.header.status) == CollisionStatus.UNKNOWN_REQUIRED_SPACE


def test_ns_difference_preserves_one_nanosecond_at_large_reference_time():
    from work.dynamic_support import evaluate_support_time
    producer, _ = _producer(untracked_speed_mm_s=2000.)
    stamp = 2 ** 62 + 131
    _hit(producer, stamp)
    scene = producer.snapshot(now_ns=stamp, tracking=_tracking(producer, stamp, velocity=(1., 0., 0.))).scene
    times = np.array([stamp, stamp + 1], dtype=np.int64)
    points, radii, rates = evaluate_support_time(scene, times, np)
    assert points[1, 0, 0] - points[0, 0, 0] == pytest.approx(1e-9, abs=1e-19)
    assert rates[1, 0] - rates[0, 0] == pytest.approx(2e-9, abs=1e-16)
    device_points, device_radii, device_rates = evaluate_support_time(scene, jnp.asarray(times))
    np.testing.assert_allclose(device_points, points, atol=1e-19, rtol=0)
    np.testing.assert_allclose(device_radii, radii, atol=5e-16, rtol=0)
    np.testing.assert_allclose(device_rates, rates, atol=1e-16, rtol=0)


def test_future_radius_capacity_is_explicit_and_certify_stays_unproved():
    from test_collision_safety import _segment_batch
    from work.collision_safety import QueryBatch, QueryMode
    from work.dynamic_support import reachable_support_tube
    producer, safety = _producer(untracked_speed_mm_s=100.)
    stamp = time.monotonic_ns()
    _hit(producer, stamp)
    scene = producer.snapshot(now_ns=stamp).scene
    assert not reachable_support_tube(scene, stamp, stamp + 1_000_000_000,
                                      policy=producer.config.policy).valid_mask[0]
    prepared = safety.prepare_scene(scene, scene.identities)
    batch = QueryBatch(jnp.zeros((2, 9)), jnp.ones(2, dtype=bool),
                       jnp.array([stamp + 1_000_000_000] * 2, dtype=jnp.int64))
    safety.query(batch, prepared, QueryMode.DISTANCE_MM)
    result = safety.query(batch, prepared, QueryMode.DISTANCE_MM)
    assert int(result.header.status) == CollisionStatus.CAPACITY_OVERFLOW
    certificate = safety.certify(_segment_batch(), prepared)
    assert int(certificate.header.status) == CollisionStatus.CERTIFICATE_FAILED
    assert not np.any(certificate.certified_mask)


def test_fixed_environment_has_zero_future_motion_and_preserves_source_identity():
    import trimesh
    from work.dynamic_support import evaluate_support_time
    from work.lidar_occupancy import FixedEnvironmentModel
    producer, safety = _producer()
    mesh = trimesh.creation.box(extents=[.001] * 3)
    mesh.apply_translation([.0005] * 3)
    model = FixedEnvironmentModel.from_mesh(
        mesh, policy=producer.config.policy, model_revision=1,
        source_hash=hashlib.sha256(mesh.export(file_type="stl")).digest(),
        validation_evidence="解析立方体的完整体积与角点范围", error_mm=0.)
    producer.set_fixed_environment(model)
    stamp = time.monotonic_ns()
    snapshot = producer.snapshot(now_ns=stamp)
    scene = snapshot.scene
    assert snapshot.fixed_model_hash == model.model_hash
    assert int(safety.prepare_scene(scene, scene.identities).status) == CollisionStatus.OK
    points, radii, rates = evaluate_support_time(scene, np.array([stamp, stamp + 1_000_000_000]), np)
    np.testing.assert_array_equal(points[0], points[1])
    np.testing.assert_array_equal(radii[0], radii[1])
    np.testing.assert_array_equal(rates, np.zeros_like(rates))
    assert scene.support_time_model.source_stamp_ns[0, 0] == 0


@pytest.mark.parametrize("change", ["epoch", "policy", "space", "scope", "clock", "source", "future", "unknown"])
def test_tracking_replay_rejects_identity_source_time_and_occupancy_mismatch(change):
    from work.dynamic_support import TrackingFrame
    producer, _ = _producer()
    stamp = time.monotonic_ns()
    _hit(producer, stamp)
    document = _tracking(producer, stamp).to_dict()
    if change in ("epoch", "policy", "space"):
        name = {"epoch": "scene_epoch", "policy": "policy_hash", "space": "required_space_hash"}[change]
        document[name] = "ab" * (16 if change == "epoch" else 32)
    elif change == "scope":
        document["tracks"][0]["evidence"]["device"] = "unrelated-device"
    elif change == "clock":
        document["clock_id"] = "simulation"
    elif change == "source":
        document["tracks"][0]["evidence"]["validation_hash"] = document["tracks"][0]["evidence"]["source_hash"]
    elif change == "future":
        document["tracks"][0]["updated_stamp_ns"] += 1
    else:
        document["tracks"][0]["support_ids"] = [999]
    with pytest.raises(ValueError):
        frame = TrackingFrame.from_dict(document)
        producer.snapshot(now_ns=stamp, tracking=frame)
    assert producer.observations()[0].point_stamp_ns == stamp


def test_tracking_velocity_outside_verified_untracked_physical_range_is_rejected():
    producer, _ = _producer(untracked_speed_mm_s=2.)
    stamp = time.monotonic_ns()
    _hit(producer, stamp)
    frame = _tracking(producer, stamp, velocity=(1., 0., 0.))
    with pytest.raises(ValueError, match="速度.*上界"):
        producer.snapshot(now_ns=stamp, tracking=frame)
    assert producer.observations()[0].point_stamp_ns == stamp


def test_three_members_of_one_track_preserve_the_declared_velocity_exactly():
    producer, safety = _producer(shape=(3, 1, 1), max_support_points=1, untracked_speed_mm_s=200.)
    stamp = time.monotonic_ns()
    endpoints = np.array([[.0005, .0005, .0005], [.0015, .0005, .0005], [.0025, .0005, .0005]])
    producer.update(LidarRayBatch(np.tile([-.001, .0005, .0005], (3, 1)), endpoints,
                                 np.full(3, stamp, dtype=np.int64), np.zeros(3, dtype=np.int32), 1), now_ns=stamp)
    frame = _tracking(producer, stamp, velocity=(.1, 0., 0.))
    scene = producer.snapshot(now_ns=stamp, tracking=frame).scene
    assert int(safety.prepare_scene(scene, scene.identities).status) == CollisionStatus.OK
    np.testing.assert_array_equal(scene.support_velocity_m_s[0], frame.tracks[0].velocity_m_s)


@pytest.mark.parametrize("elapsed_ns", [100_000_000, 300_000_000])
def test_update_time_error_covers_pre_update_motion_at_current_and_future_times(elapsed_ns):
    from work.dynamic_support import reachable_support_tube
    producer, safety = _producer(untracked_speed_mm_s=1000., untracked_acceleration_mm_s2=10000.,
                                 support_radius_limit_mm=1000.)
    now = time.monotonic_ns()
    stamp = now - elapsed_ns
    _hit(producer, stamp)
    frame = _tracking(producer, stamp, velocity=(0., 0., 0.))
    track = replace(frame.tracks[0], timestamp_error_ns=200_000_000, position_error_mm=0.,
                    velocity_error_mm_s=0., acceleration_bound_m_s2=0.)
    frame = replace(frame, tracks=(track,))
    snapshot = producer.snapshot(now_ns=now, tracking=frame)
    duration = min(elapsed_ns * 1e-9, .2)
    displacement_mm = (duration - 2.5 * duration ** 2) * 1000
    assert snapshot.scene.support_radii_mm[0] >= displacement_mm
    prepared = safety.prepare_scene(snapshot.scene, snapshot.scene.identities)
    assert int(prepared.status) == CollisionStatus.OK
    tube = reachable_support_tube(prepared, now, now + 1_000_000, policy=producer.config.policy)
    assert tube.valid_mask[0]
    assert tube.enclosing_radii_mm[0] >= displacement_mm


@pytest.mark.parametrize("field", ["growth", "acceleration", "radius", "validity"])
def test_untracked_sources_and_groups_remain_bound_to_actual_policy_after_serialization(field):
    from collision_scene_inputs import signed_scene
    from work.collision_safety import QueryBatch, QueryMode
    from work.dynamic_support import reachable_support_tube
    producer, safety = _producer()
    now = time.monotonic_ns()
    stamp = now - 200_000_000
    _hit(producer, stamp)
    original = producer.snapshot(now_ns=now).scene
    scene = pickle.loads(pickle.dumps(original))
    model = scene.support_time_model
    if field == "growth":
        model = model._replace(radius_rate_mm_s=np.zeros_like(model.radius_rate_mm_s),
                               source_radius_rate_mm_s=np.zeros_like(model.source_radius_rate_mm_s))
    elif field == "acceleration":
        model = model._replace(radius_acceleration_mm_s2=np.zeros_like(model.radius_acceleration_mm_s2),
                               source_acceleration_mm_s2=np.zeros_like(model.source_acceleration_mm_s2))
    elif field == "radius":
        radii = np.array(scene.support_radii_mm)
        source_radii = np.array(model.source_radii_mm)
        radii[0] = source_radii[0, 0] = np.nextafter(np.sqrt(3) / 2, np.inf)
        scene = scene._replace(support_radii_mm=radii)
        model = model._replace(source_radii_mm=source_radii)
    else:
        source_until, group_until = np.array(model.source_valid_until_ns), np.array(model.valid_until_ns)
        source_until[0, 0] = group_until[0] = stamp + 5_000_000_000
        model = model._replace(source_valid_until_ns=source_until, valid_until_ns=group_until)
        scene = scene._replace(coverage_valid_until_ns=np.int64(stamp + 5_000_000_000))
    changed = signed_scene(scene._replace(support_time_model=model))
    prepared = safety.prepare_scene(changed, changed.identities)
    assert int(prepared.status) == CollisionStatus.INVALID_SCENE
    assert not np.any(reachable_support_tube(changed, now, now + 100_000_000,
                                            policy=producer.config.policy).valid_mask)
    batch = QueryBatch(jnp.zeros((2, 9)), jnp.ones(2, dtype=bool), jnp.array([now] * 2, dtype=jnp.int64))
    for mode in (QueryMode.OSCBF_BARRIER, QueryMode.DISTANCE_MM):
        safety.query(batch, prepared, mode)
        result = safety.query(batch, prepared, mode)
        assert int(result.header.status) == CollisionStatus.INVALID_SCENE
        assert not np.any(result.valid_mask)
        assert not np.any(result.distance_valid_mask)


def test_direct_tube_requires_real_policy_and_preserves_json_and_object_roundtrips():
    from work.dynamic_support import TrackingFrame, reachable_support_tube
    producer, safety = _producer()
    _, other_safety = _producer(untracked_speed_mm_s=0., untracked_acceleration_mm_s2=0.)
    stamp = time.monotonic_ns()
    _hit(producer, stamp)
    frame = TrackingFrame.from_dict(json.loads(json.dumps(_tracking(producer, stamp).to_dict())))
    scene = pickle.loads(pickle.dumps(producer.snapshot(now_ns=stamp, tracking=frame).scene))
    prepared = pickle.loads(pickle.dumps(safety.prepare_scene(scene, scene.identities)))
    for source in (scene, prepared):
        assert not np.any(reachable_support_tube(source, stamp, stamp + 100_000_000).valid_mask)
        assert not np.any(reachable_support_tube(source, stamp, stamp + 100_000_000,
                                                policy=other_safety.config.policy).valid_mask)
        assert reachable_support_tube(source, stamp, stamp + 100_000_000, policy=producer.config.policy).valid_mask[0]


@pytest.mark.parametrize("field", ["growth", "acceleration", "policy"])
def test_prepared_consumers_recheck_source_bounds_against_module_policy(field):
    from work.collision_safety import QueryBatch, QueryMode
    from work.dynamic_support import reachable_support_tube
    producer, safety = _producer()
    stamp = time.monotonic_ns()
    _hit(producer, stamp)
    scene = producer.snapshot(now_ns=stamp).scene
    prepared = pickle.loads(pickle.dumps(safety.prepare_scene(scene, scene.identities)))
    assert int(prepared.status) == CollisionStatus.OK
    model = prepared.support_time_model
    if field == "growth":
        model = model._replace(radius_rate_mm_s=jnp.zeros_like(model.radius_rate_mm_s),
                               source_radius_rate_mm_s=jnp.zeros_like(model.source_radius_rate_mm_s))
    elif field == "acceleration":
        model = model._replace(radius_acceleration_mm_s2=jnp.zeros_like(model.radius_acceleration_mm_s2),
                               source_acceleration_mm_s2=jnp.zeros_like(model.source_acceleration_mm_s2))
    else:
        _, replacement = _producer(untracked_speed_mm_s=0., untracked_acceleration_mm_s2=0.)
        replacement_hash = jnp.asarray(np.frombuffer(replacement.config.collision_policy_hash, dtype=np.uint8))
        model = model._replace(policy_hash=replacement_hash)
        prepared = prepared._replace(identities=prepared.identities._replace(collision_policy_hash=replacement_hash))
    changed = prepared._replace(support_time_model=model)
    assert not np.any(reachable_support_tube(changed, stamp, stamp + 100_000_000,
                                            policy=producer.config.policy).valid_mask)
    batch = QueryBatch(jnp.zeros((2, 9)), jnp.ones(2, dtype=bool), jnp.array([stamp] * 2, dtype=jnp.int64))
    for mode in (QueryMode.OSCBF_BARRIER, QueryMode.DISTANCE_MM):
        safety.query(batch, changed, mode)
        result = safety.query(batch, changed, mode)
        assert int(result.header.status) == CollisionStatus.INVALID_SCENE
        assert not np.any(result.valid_mask)
        assert not np.any(result.distance_valid_mask)


@pytest.mark.parametrize("delay_ns", [100_000_000, 3_000_000_000])
@pytest.mark.parametrize("invalid_input", ["type", "policy"])
def test_rejected_tracking_preserves_time_revision_and_original_observations(delay_ns, invalid_input):
    from work.dynamic_support import observation_hash
    producer, _ = _producer()
    stamp = time.monotonic_ns()
    _hit(producer, stamp)
    before = producer.snapshot(now_ns=stamp)
    original_hash = observation_hash(producer.observations())
    tracking = (object() if invalid_input == "type"
                else replace(_tracking(producer, stamp), policy_hash=bytes([171]) * 32))
    with pytest.raises(TypeError if invalid_input == "type" else ValueError):
        producer.snapshot(now_ns=stamp + delay_ns, tracking=tracking)
    after = producer.snapshot(now_ns=stamp)
    assert after.scene.metadata.checksum == before.scene.metadata.checksum
    assert observation_hash(producer.observations()) == original_hash
    np.testing.assert_array_equal(after.cells, before.cells)
    np.testing.assert_array_equal(after.last_observed_ns, before.last_observed_ns)
    np.testing.assert_array_equal(after.last_hit_ns, before.last_hit_ns)
    assert int(after.scene.identities.scene_revision) == int(before.scene.identities.scene_revision)


def _fixed_model(producer, *, center=(.0035, .0005, .0005), revision=1):
    import trimesh
    from work.lidar_occupancy import FixedEnvironmentModel
    mesh = trimesh.creation.box(extents=[.001] * 3)
    mesh.apply_translation(center)
    return FixedEnvironmentModel.from_mesh(
        mesh, policy=producer.config.policy, model_revision=revision,
        source_hash=hashlib.sha256(mesh.export(file_type="stl")).digest(),
        validation_evidence="解析立方体的完整体积与角点范围", error_mm=0.)


@pytest.mark.parametrize("with_fixed_model", [False, True])
def test_zero_observation_time_cannot_change_lidar_source_to_fixed_model(with_fixed_model):
    from collision_scene_inputs import signed_scene
    from work.collision_safety import QueryBatch, QueryMode
    from work.dynamic_support import reachable_support_tube
    producer, safety = _producer(max_support_points=2)
    if with_fixed_model:
        producer.set_fixed_environment(_fixed_model(producer))
    stamp = time.monotonic_ns()
    _hit(producer, stamp)
    scene = producer.snapshot(now_ns=stamp).scene
    original = safety.prepare_scene(scene, scene.identities)
    assert int(original.status) == CollisionStatus.OK
    model = scene.support_time_model
    changed_stamps = np.array(model.source_stamp_ns)
    changed_stamps[model.source_mask] = 0
    changed_model = model._replace(
        source_stamp_ns=changed_stamps,
        radius_rate_mm_s=np.zeros_like(model.radius_rate_mm_s),
        radius_acceleration_mm_s2=np.zeros_like(model.radius_acceleration_mm_s2),
        source_radius_rate_mm_s=np.zeros_like(model.source_radius_rate_mm_s),
        source_acceleration_mm_s2=np.zeros_like(model.source_acceleration_mm_s2))
    changed = signed_scene(scene._replace(support_time_model=changed_model))
    assert int(safety.prepare_scene(changed, changed.identities).status) == CollisionStatus.INVALID_SCENE
    assert not np.any(reachable_support_tube(changed, stamp, stamp + 100_000_000,
                                            policy=producer.config.policy).valid_mask)
    prepared = original._replace(support_time_model=type(model)(*(jnp.asarray(value) for value in changed_model)))
    assert not np.any(reachable_support_tube(prepared, stamp, stamp + 100_000_000,
                                            policy=producer.config.policy).valid_mask)
    batch = QueryBatch(jnp.zeros((2, 9)), jnp.ones(2, dtype=bool), jnp.array([stamp] * 2, dtype=jnp.int64))
    for mode in (QueryMode.OSCBF_BARRIER, QueryMode.DISTANCE_MM):
        safety.query(batch, prepared, mode)
        result = safety.query(batch, prepared, mode)
        assert int(result.header.status) == CollisionStatus.INVALID_SCENE
        assert not np.any(result.valid_mask)
        assert not np.any(result.distance_valid_mask)


@pytest.mark.parametrize("change", ["missing", "revision", "geometry", "history"])
def test_fixed_source_requires_actual_model_identity_geometry_and_member_history(change):
    from collision_scene_inputs import signed_scene
    from work.collision_safety import QueryBatch, QueryMode
    from work.dynamic_support import reachable_support_tube
    producer, safety = _producer()
    producer.set_fixed_environment(_fixed_model(producer))
    stamp = time.monotonic_ns()
    scene = producer.snapshot(now_ns=stamp).scene
    original = pickle.loads(pickle.dumps(safety.prepare_scene(scene, scene.identities)))
    assert int(original.status) == CollisionStatus.OK
    assert original.fixed_environment_model.model_hash == scene.fixed_environment_model.model_hash
    assert not original.fixed_environment_model.support_points_m.flags.writeable
    assert not original.fixed_environment_model.support_radii_mm.flags.writeable
    assert reachable_support_tube(original, stamp, stamp + 100_000_000,
                                 policy=producer.config.policy).valid_mask[0]
    if change == "history":
        hashes = np.array(scene.support_time_model.source_history_hash)
        hashes[0, 0] = 171
        model = scene.support_time_model._replace(source_history_hash=hashes)
        scene = scene._replace(support_time_model=model)
        prepared = original._replace(support_time_model=type(model)(*(jnp.asarray(value) for value in model)))
    else:
        fixed = (None if change == "missing" else _fixed_model(producer, revision=2) if change == "revision"
                 else _fixed_model(producer, center=(.0036, .0005, .0005)))
        scene = scene._replace(fixed_environment_model=fixed)
        prepared = original._replace(fixed_environment_model=fixed)
    scene = signed_scene(scene)
    assert int(safety.prepare_scene(scene, scene.identities).status) == CollisionStatus.INVALID_SCENE
    for source in (scene, prepared):
        assert not np.any(reachable_support_tube(source, stamp, stamp + 100_000_000,
                                                policy=producer.config.policy).valid_mask)
    batch = QueryBatch(jnp.zeros((2, 9)), jnp.ones(2, dtype=bool), jnp.array([stamp] * 2, dtype=jnp.int64))
    for mode in (QueryMode.OSCBF_BARRIER, QueryMode.DISTANCE_MM):
        safety.query(batch, prepared, mode)
        result = safety.query(batch, prepared, mode)
        assert int(result.header.status) == CollisionStatus.INVALID_SCENE
        assert not np.any(result.valid_mask)
        assert not np.any(result.distance_valid_mask)


def _merged_fixed_sources():
    import trimesh
    from work.fixed_environment import FixedEnvironmentModel
    producer, safety = _producer(max_support_points=2, support_radius_limit_mm=1.2,
                                 support_merge_radius_limit_mm=1.2, support_merge_max_points=4)
    mesh = trimesh.creation.box(extents=[.003, .0001, .0001])
    mesh.apply_translation([.0015, .0005, .0005])
    model = FixedEnvironmentModel.from_mesh(
        mesh, policy=producer.config.policy, model_revision=1,
        source_hash=hashlib.sha256(mesh.export(file_type="stl")).digest(),
        validation_evidence="三个解析几何成员的完整长方体与分组包围", error_mm=0.)
    producer.set_fixed_environment(model)
    stamp = time.monotonic_ns()
    _hit(producer, stamp)
    scene = producer.snapshot(now_ns=stamp).scene
    assert len(model.support_points_m) == 3
    assert int(np.sum(scene.support_mask)) == 2
    assert int(np.sum(scene.support_time_model.source_mask)) == 4
    return producer, safety, stamp, scene


@pytest.mark.parametrize("removed", ["all", "one", "group", "legacy-group"])
def test_every_fixed_model_member_requires_active_sources_and_enclosing_groups(removed):
    from collision_scene_inputs import signed_scene
    from work.collision_safety import QueryBatch, QueryMode
    from work.dynamic_support import reachable_support_tube
    producer, safety, stamp, scene = _merged_fixed_sources()
    original = safety.prepare_scene(scene, scene.identities)
    assert int(original.status) == CollisionStatus.OK
    model = scene.support_time_model
    kept = np.array(model.source_mask)
    fixed = kept & (model.source_stamp_ns == 0)
    if removed == "all":
        kept[fixed] = False
    elif removed == "one":
        kept[tuple(np.argwhere(fixed)[0])] = False
    else:
        fixed_only = np.flatnonzero(scene.support_mask & ~np.any(kept & (model.source_stamp_ns > 0), axis=1))[0]
        kept[fixed_only] = False
    supports = np.any(kept, axis=1)
    changed_model = model._replace(source_mask=kept)
    changed = signed_scene(scene._replace(
        support_mask=supports, support_valid_mask=supports.copy(),
        support_time_model=None if removed == "legacy-group" else changed_model,
        metadata=replace(scene.metadata, support_count=int(np.sum(supports)))))
    assert int(safety.prepare_scene(changed, changed.identities).status) == CollisionStatus.INVALID_SCENE
    assert not np.any(reachable_support_tube(changed, stamp, stamp + 1_000_000,
                                            policy=producer.config.policy).valid_mask)
    prepared = pickle.loads(pickle.dumps(original._replace(
        support_mask=jnp.asarray(supports), support_valid_mask=jnp.asarray(supports),
        support_time_model=(None if removed == "legacy-group" else
                            type(model)(*(jnp.asarray(value) for value in changed_model))))))
    assert not np.any(reachable_support_tube(prepared, stamp, stamp + 1_000_000,
                                            policy=producer.config.policy).valid_mask)
    batch = QueryBatch(jnp.zeros((2, 9)), jnp.ones(2, dtype=bool), jnp.array([stamp] * 2, dtype=jnp.int64))
    for mode in (QueryMode.OSCBF_BARRIER, QueryMode.DISTANCE_MM):
        safety.query(batch, prepared, mode)
        result = safety.query(batch, prepared, mode)
        assert int(result.header.status) == CollisionStatus.INVALID_SCENE
        assert not np.any(result.valid_mask)
        assert not np.any(result.distance_valid_mask)


@pytest.mark.parametrize("legacy", [False, True])
def test_complete_fixed_members_remain_enclosed_after_legal_merging(legacy):
    from collision_scene_inputs import signed_scene
    from work.dynamic_support import evaluate_support_time, reachable_support_tube
    producer, safety, stamp, scene = _merged_fixed_sources()
    if legacy:
        scene = signed_scene(scene._replace(support_time_model=None))
    prepared = safety.prepare_scene(scene, scene.identities)
    assert int(prepared.status) == CollisionStatus.OK
    times = stamp + (np.zeros(101, dtype=np.int64) if legacy else np.arange(101, dtype=np.int64) * 10_000)
    points, radii, _ = evaluate_support_time(prepared, times, np)
    actual = scene.fixed_environment_model
    required = (np.linalg.norm(points[:, :, None, :] - actual.support_points_m[None, None, :, :], axis=-1) * 1000
                + actual.support_radii_mm[None, None, :])
    assert np.all(np.any((required <= radii[:, :, None] + 1e-13)
                         & np.asarray(prepared.support_mask)[None, :, None], axis=1))
    tube = reachable_support_tube(prepared, stamp, stamp + 1_000_000, policy=producer.config.policy)
    if legacy:
        assert not np.any(tube.valid_mask)
    else:
        assert np.all(tube.valid_mask[:2])
        enclosing = (np.linalg.norm(tube.enclosing_points_m[:, None, :] - actual.support_points_m[None, :, :], axis=-1)
                     * 1000 + actual.support_radii_mm[None, :])
        assert np.all(np.any((enclosing <= tube.enclosing_radii_mm[:, None] + 1e-13)
                             & tube.valid_mask[:, None], axis=0))
