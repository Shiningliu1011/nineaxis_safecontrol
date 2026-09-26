from dataclasses import FrozenInstanceError, replace
import time

import jax
import jax.numpy as jnp
import numpy as np
import pytest

from collision_scene_inputs import scene_input, signed_scene
from test_collision_safety import _config, _identities, _query_batch, _segment_batch
from test_ellipsoid_point import _environment
from work.collision_safety import CollisionIdentities, CollisionSafety, CollisionStatus, QueryMode


@pytest.fixture(scope="module")
def module():
    return CollisionSafety(_config())


def _input(module):
    return scene_input(module.config, _identities(module.config))


def _prepare(module, scene):
    return module.prepare_scene(signed_scene(scene), scene.identities)


def _assert_rejected(module, prepared, expected):
    assert int(prepared.status) == expected
    assert not np.any(prepared.support_valid_mask)
    assert not np.any(prepared.tracks.valid_mask)
    query = module.query(_query_batch(), prepared, QueryMode.DISTANCE_MM)
    certificate = module.certify(_segment_batch(), prepared)
    assert int(query.header.status) != CollisionStatus.OK
    assert int(certificate.header.status) != CollisionStatus.OK
    for mask in (query.valid_mask, query.state_valid_mask, query.distance_valid_mask, certificate.valid_mask):
        assert not np.any(mask)


@pytest.mark.parametrize("field,value", [
    ("layout_version", 2), ("frame_id", "lidar_link"), ("point_unit", "mm"),
    ("radius_unit", "m"), ("velocity_unit", "mm/s"), ("time_unit", "s"),
    ("clock_id", "wall"), ("support_count", -1), ("track_count", -1),
    ("support_count", 1), ("track_count", 0),
])
def test_metadata_admission_rejects_resigned_invalid_fields(module, field, value):
    scene = _input(module)
    scene = scene._replace(metadata=replace(scene.metadata, **{field: value}))
    _assert_rejected(module, _prepare(module, scene), CollisionStatus.INVALID_SCENE)


@pytest.mark.parametrize("field", list(CollisionIdentities._fields))
def test_reader_expected_identity_must_match_source(module, field):
    scene = _input(module)
    array = np.array(getattr(scene.identities, field))
    array.flat[0] += 1
    expected = scene.identities._replace(**{field: array})
    _assert_rejected(module, module.prepare_scene(scene, expected), CollisionStatus.INVALID_SCENE)


@pytest.mark.parametrize("field", list(CollisionIdentities._fields))
def test_invalid_source_identity_rejected_even_when_reader_agrees(module, field):
    scene = _input(module)
    array = np.zeros_like(getattr(scene.identities, field))
    if array.ndim == 0:
        array[...] = -1
    scene = scene._replace(identities=scene.identities._replace(**{field: array}))
    _assert_rejected(module, _prepare(module, scene), CollisionStatus.INVALID_SCENE)


@pytest.mark.parametrize("field", [
    "support_points_m", "support_radii_mm", "required_clearance_mm", "support_velocity_m_s",
])
@pytest.mark.parametrize("value", [np.nan, np.inf, -np.inf])
def test_nonfinite_values_cannot_be_hidden_in_inactive_slots(module, field, value):
    scene = _input(module)
    array = np.array(getattr(scene, field))
    array[-1] = value
    _assert_rejected(module, _prepare(module, scene._replace(**{field: array})), CollisionStatus.INVALID_SCENE)


@pytest.mark.parametrize("field,value", [
    ("support_radii_mm", [-1., 20., 0.]), ("support_radii_mm", [101., 20., 0.]),
    ("required_clearance_mm", [0., 30., 0.]), ("support_ids", [11, 11, 0]),
    ("support_ids", [-1, 12, 0]), ("support_valid_mask", [False, True, False]),
    ("support_track_status", [0, 3, 0]), ("support_track_ids", [-1, 999, -1]),
])
def test_invalid_support_values_fail_with_valid_checksum(module, field, value):
    scene = _input(module)
    value = np.asarray(value, dtype=getattr(scene, field).dtype)
    _assert_rejected(module, _prepare(module, scene._replace(**{field: value})), CollisionStatus.INVALID_SCENE)


@pytest.mark.parametrize("case", ["zero", "future_source", "future_prepared", "reversed", "stale", "preparation_delay"])
def test_collection_and_preparation_time_admission(module, case):
    scene = _input(module)
    now = time.monotonic_ns()
    changes = {
        "zero": {"source_stamp_ns": 0},
        "future_source": {"source_stamp_ns": now + 10_000_000_000},
        "future_prepared": {"prepared_stamp_ns": now + 10_000_000_000},
        "reversed": {"prepared_stamp_ns": int(scene.source_stamp_ns) - 1},
        "stale": {"source_stamp_ns": now - 181_000_000_000, "prepared_stamp_ns": now - 181_000_000_000},
        "preparation_delay": {"source_stamp_ns": now - 121_000_000_000},
    }[case]
    scene = scene._replace(**{name: np.int64(value) for name, value in changes.items()})
    _assert_rejected(module, _prepare(module, scene), CollisionStatus.INVALID_SCENE)


@pytest.mark.parametrize("case", ["unknown", "wrong_space", "expired"])
def test_required_space_coverage_is_independent_of_support_count(module, case):
    scene = scene_input(module.config, _identities(module.config), mask=[False, False, False])
    if case == "unknown":
        scene = scene._replace(metadata=replace(scene.metadata, required_space_covered=False))
    elif case == "wrong_space":
        scene = scene._replace(coverage_space_hash=np.zeros(32, dtype=np.uint8))
    else:
        scene = scene._replace(coverage_valid_until_ns=np.int64(time.monotonic_ns() - 1))
    _assert_rejected(module, _prepare(module, scene), CollisionStatus.UNKNOWN_REQUIRED_SPACE)


@pytest.mark.parametrize("field", ["support_count", "track_count"])
def test_declared_demand_over_capacity_is_never_truncated_into_ok(module, field):
    scene = _input(module)
    scene = scene._replace(metadata=replace(scene.metadata, **{field: 4}))
    _assert_rejected(module, _prepare(module, scene), CollisionStatus.CAPACITY_OVERFLOW)


@pytest.mark.parametrize("field,kind", [
    ("ids", "negative"), ("valid_mask", "false"), ("active_mask", "false"),
    ("updated_stamp_ns", "zero"), ("updated_stamp_ns", "future"),
    ("valid_until_ns", "expired"), ("velocity_m_s", "different"),
    ("acceleration_bound_m_s2", "negative"), ("error_bound_mm", "negative"),
    ("error_bound_mm", "uncovered"), ("velocity_m_s", "nan"),
    ("acceleration_bound_m_s2", "nan"), ("error_bound_mm", "nan"),
])
def test_tracking_requires_valid_association_time_and_bounds(module, field, kind):
    scene = _input(module)
    values = np.array(getattr(scene.tracks, field))
    value = {"negative": -1, "false": False, "zero": 0, "future": time.monotonic_ns() + 10_000_000_000,
             "expired": time.monotonic_ns() - 1, "different": 1., "nan": np.nan, "uncovered": 21.}[kind]
    values[0] = value
    scene = scene._replace(tracks=scene.tracks._replace(**{field: values}))
    _assert_rejected(module, _prepare(module, scene), CollisionStatus.INVALID_SCENE)


def test_untracked_occupied_support_retains_occupancy_and_rejects_nominal_motion(module):
    scene = scene_input(module.config, _identities(module.config), tracks=[0, 0, 0])
    prepared = module.prepare_scene(scene, scene.identities)
    assert int(prepared.status) == CollisionStatus.OK
    np.testing.assert_array_equal(prepared.support_mask, [True, True, False])
    assert not np.any(prepared.tracks.active_mask)
    velocity = np.array(scene.support_velocity_m_s)
    velocity[0, 0] = .01
    _assert_rejected(module, _prepare(module, scene._replace(support_velocity_m_s=velocity)), CollisionStatus.INVALID_SCENE)


def test_duplicate_track_ids_are_rejected(module):
    scene = scene_input(module.config, _identities(module.config), tracks=[1, 1, 0])
    scene = scene._replace(tracks=scene.tracks._replace(ids=np.array([1000, 1000, -1], dtype=np.int32)))
    _assert_rejected(module, _prepare(module, scene), CollisionStatus.INVALID_SCENE)


def test_unknown_source_status_has_no_admission(module):
    scene = _input(module)._replace(status=np.int32(99))
    _assert_rejected(module, _prepare(module, scene), CollisionStatus.INVALID_SCENE)


@pytest.mark.parametrize("field", [
    "support_points_m", "support_radii_mm", "required_clearance_mm", "support_velocity_m_s", "support_ids",
    "support_mask", "source_stamp_ns", "prepared_stamp_ns", "status", "support_track_status",
    "support_valid_mask", "support_track_ids", "coverage_space_hash", "coverage_valid_until_ns",
])
def test_checksum_detects_corruption_in_each_payload_field(module, field):
    scene = _input(module)
    values = np.array(getattr(scene, field))
    values.flat[-1] = not values.flat[-1] if values.dtype.kind == "b" else values.flat[-1] + 1
    _assert_rejected(module, module.prepare_scene(scene._replace(**{field: values}), scene.identities),
                     CollisionStatus.INVALID_SCENE)


@pytest.mark.parametrize("group", ["identities", "tracks"])
def test_checksum_detects_all_nested_field_changes(module, group):
    scene = _input(module)
    record = getattr(scene, group)
    for field, original in record._asdict().items():
        values = np.array(original)
        values.flat[-1] = not values.flat[-1] if values.dtype.kind == "b" else values.flat[-1] + 1
        changed = scene._replace(**{group: record._replace(**{field: values})})
        _assert_rejected(module, module.prepare_scene(changed, changed.identities), CollisionStatus.INVALID_SCENE)


def test_checksum_covers_metadata_and_is_stable_for_equivalent_array_layouts(module):
    scene = _input(module)
    for field, value in [("frame_id", "world"), ("support_count", 3), ("required_space_covered", False)]:
        changed = scene._replace(metadata=replace(scene.metadata, **{field: value}))
        assert changed.compute_checksum() != scene.metadata.checksum
        _assert_rejected(module, module.prepare_scene(changed, scene.identities), CollisionStatus.INVALID_SCENE)
    same = scene._replace(support_points_m=np.asfortranarray(scene.support_points_m))
    assert same.compute_checksum() == scene.metadata.checksum
    changed = scene._replace(support_points_m=scene.support_points_m.astype(np.float64))
    assert changed.compute_checksum() != scene.metadata.checksum
    for checksum in (b"", b"x" * 32):
        changed = scene._replace(metadata=replace(scene.metadata, checksum=checksum))
        _assert_rejected(module, module.prepare_scene(changed, scene.identities), CollisionStatus.INVALID_SCENE)


@pytest.mark.parametrize("group", [None, "tracks", "identities"])
def test_wrong_shapes_and_dtypes_raise_before_scene_preparation(module, group):
    scene = _input(module)
    record = scene if group is None else getattr(scene, group)
    arrays = record._asdict()
    for field, original in arrays.items():
        if field in ("tracks", "identities", "metadata"):
            continue
        for value, error in [(np.zeros((5, 2), dtype=np.asarray(original).dtype), ValueError),
                             (np.asarray(original).astype(np.complex128), TypeError)]:
            changed = record._replace(**{field: value})
            bad = changed if group is None else scene._replace(**{group: changed})
            with pytest.raises(error, match=field):
                module.prepare_scene(bad, scene.identities)


def test_float32_conversion_and_deep_immutability(module):
    scene = _input(module)
    changes = {field: value.astype(np.float32) for field, value in scene._asdict().items()
               if isinstance(value, np.ndarray) and value.dtype.kind == "f"}
    tracks = scene.tracks._replace(**{field: value.astype(np.float32) for field, value in scene.tracks._asdict().items()
                                      if value.dtype.kind == "f"})
    scene = signed_scene(scene._replace(**changes, tracks=tracks))
    prepared = module.prepare_scene(scene, scene.identities)
    assert int(prepared.status) == CollisionStatus.OK
    before = [np.array(value) for value in jax.tree.leaves(prepared)]
    for value in jax.tree.leaves(prepared):
        assert isinstance(value, jax.Array)
        if value.dtype.kind == "f":
            assert value.dtype == jnp.float64
    for value in scene._asdict().values():
        if isinstance(value, np.ndarray):
            value[...] = 0
    scene.tracks.velocity_m_s[...] = 1
    for actual, expected in zip(jax.tree.leaves(prepared), before):
        np.testing.assert_array_equal(actual, expected)
    with pytest.raises(AttributeError):
        prepared.status = jnp.int32(2)
    with pytest.raises(TypeError):
        prepared.support_points_m[0, 0] = 1.
    with pytest.raises(FrozenInstanceError):
        scene.metadata.frame_id = "world"
    compiled = jax.jit(lambda value: value)(prepared)
    assert int(compiled.status) == CollisionStatus.OK


def test_float32_clearance_is_checked_after_conversion_to_computation_precision():
    module = CollisionSafety(_config(environment_clearance_mm=0.1))
    scene = _input(module)
    precise = module.prepare_scene(scene, scene.identities)
    assert int(precise.status) == CollisionStatus.OK
    rounded = scene._replace(required_clearance_mm=scene.required_clearance_mm.astype(np.float32))
    _assert_rejected(module, _prepare(module, rounded), CollisionStatus.INVALID_SCENE)


@pytest.mark.parametrize("parameter", ["scene_preparation_deadline_ns", "device_transfer_deadline_ns"])
def test_preparation_and_device_copy_deadlines(parameter):
    module = CollisionSafety(_config(**{parameter: 1}))
    scene = _input(module)
    _assert_rejected(module, module.prepare_scene(scene, scene.identities), CollisionStatus.DEADLINE_MISSED)


def test_prepared_expiration_is_rechecked_by_query_and_certify(module):
    scene = _input(module)
    prepared = module.prepare_scene(scene, scene.identities)
    for changed, expected in [
        (prepared._replace(valid_until_ns=jnp.int64(time.monotonic_ns() - 1)), CollisionStatus.INVALID_SCENE),
        (prepared._replace(coverage_valid_until_ns=jnp.int64(time.monotonic_ns() - 1)), CollisionStatus.UNKNOWN_REQUIRED_SPACE),
        (prepared._replace(tracks=prepared.tracks._replace(valid_until_ns=jnp.zeros(3, dtype=jnp.int64))),
         CollisionStatus.INVALID_SCENE),
    ]:
        result = module.query(_query_batch(), changed, QueryMode.DISTANCE_MM)
        assert int(result.header.status) in (expected, CollisionStatus.DEADLINE_MISSED)
        assert int(module.certify(_segment_batch(), changed).header.status) == expected
        assert not np.any(result.distance_valid_mask)


def test_unchanged_prepared_scene_expires_with_the_monotonic_clock(module):
    scene = _input(module)
    expires = time.monotonic_ns() + 300_000_000
    scene = signed_scene(scene._replace(coverage_valid_until_ns=np.int64(expires)))
    prepared = module.prepare_scene(scene, scene.identities)
    assert int(prepared.status) == CollisionStatus.OK
    remaining = (expires - time.monotonic_ns()) / 1e9
    if remaining > 0:
        time.sleep(remaining)
    certificate = module.certify(_segment_batch(), prepared)
    assert int(certificate.header.status) == CollisionStatus.UNKNOWN_REQUIRED_SPACE
    assert not np.any(certificate.valid_mask)
    assert certificate.header.fixed_environment_revision == scene.identities.fixed_environment_revision
    np.testing.assert_array_equal(certificate.header.required_space_hash, scene.identities.required_space_hash)


@pytest.mark.parametrize("field", ["valid_until_ns", "coverage_valid_until_ns", "tracking_valid_until_ns"])
def test_query_completion_cannot_outlive_any_admission_deadline(field):
    _, module = _environment()
    identities = _identities(module.config)._replace(
        geometry_hash=jnp.asarray(np.frombuffer(module.config.geometry_hash, dtype=np.uint8)),
        kernel_version=jnp.asarray(np.frombuffer(module.config.kernel_version, dtype=np.uint8)),
    )
    scene = scene_input(module.config, identities)
    prepared = module.prepare_scene(scene, identities)
    module.query(_query_batch(), prepared, QueryMode.DISTANCE_MM)
    accepted = 0
    rejected = 0
    for duration in np.linspace(1_000, 8_000_000, 80, dtype=np.int64):
        expires = time.monotonic_ns() + int(duration)
        timed = prepared._replace(**{field: jnp.int64(expires)})
        result = module.query(_query_batch(), timed, QueryMode.DISTANCE_MM)
        if int(result.header.status) == CollisionStatus.OK:
            accepted += 1
            assert bool(result.distance_valid_mask[0])
            assert int(result.header.completed_ns) <= expires
        else:
            rejected += 1
            assert not np.any(result.distance_valid_mask)
    assert accepted > 0
    assert rejected > 0


def test_support_and_track_activity_preserve_prepared_and_query_shapes():
    _, module = _environment(capacity=2)
    identities = _identities(module.config)._replace(
        geometry_hash=jnp.asarray(np.frombuffer(module.config.geometry_hash, dtype=np.uint8)),
        kernel_version=jnp.asarray(np.frombuffer(module.config.kernel_version, dtype=np.uint8)),
    )
    prepared_shapes, query_shapes = [], []
    for count in (0, 1, 2, 3):
        scene = scene_input(module.config, identities, mask=np.arange(3) < count, tracks=[1, 1, 1],
                            points=np.array([[0., 0., .3], [0., .3, 0.], [.3, 0., 0.]]))
        prepared = module.prepare_scene(scene, identities)
        assert int(prepared.status) == CollisionStatus.OK
        assert int(np.sum(prepared.support_valid_mask)) == count
        assert int(np.sum(prepared.tracks.valid_mask)) == count
        prepared_shapes.append([(value.shape, value.dtype) for value in jax.tree.leaves(prepared)])
        module.query(_query_batch(), prepared, QueryMode.OSCBF_BARRIER)
        result = module.query(_query_batch(), prepared, QueryMode.OSCBF_BARRIER)
        assert int(result.header.status) == CollisionStatus.OK
        assert int(np.sum(result.valid_mask)) == 1 + 2 * count
        query_shapes.append([(value.shape, value.dtype) for value in jax.tree.leaves(result)])
    assert all(value == prepared_shapes[0] for value in prepared_shapes)
    assert all(value == query_shapes[0] for value in query_shapes)
