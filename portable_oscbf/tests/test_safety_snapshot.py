#!/usr/bin/env python3
"""Safety-snapshot contract tests for the point-cloud-to-JAX boundary."""

import numpy as np
import pytest
import time
from dataclasses import replace
from types import SimpleNamespace


def test_preprocess_points_transforms_crops_downsamples_and_excludes_robot():
    from work.safety_snapshot import SafetyGridSpec, preprocess_points

    spec = SafetyGridSpec(
        workspace_min=np.array([0.0, 0.0, 0.0]),
        workspace_max=np.array([1.0, 1.0, 1.0]),
        voxel_size=0.1,
    )
    points = np.array([
        [-1.0, 0.0, 0.0],  # translated into the workspace
        [-0.96, 0.02, 0.01],  # same voxel as the first point
        [0.5, 0.5, 0.5],  # excluded as a robot point
        [3.0, 0.0, 0.0],  # outside after transformation
        [np.nan, 0.0, 0.0],
    ])
    transform = np.eye(4)
    transform[0, 3] = 1.0

    processed = preprocess_points(
        points,
        transform,
        spec,
        robot_spheres=[(np.array([0.5, 0.5, 0.5]), 0.08)],
    )

    assert processed.shape == (1, 3)
    assert np.allclose(processed[0], [0.0, 0.0, 0.0])


def test_distance_field_and_jax_sampling_are_conservative_at_boundaries():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp

    from work.safety_snapshot import (
        SafetyGridSpec,
        build_distance_field,
        sample_distance_field_jax,
    )

    spec = SafetyGridSpec(
        workspace_min=np.zeros(3),
        workspace_max=np.ones(3),
        voxel_size=0.1,
    )
    field = build_distance_field(np.array([[0.5, 0.5, 0.5]]), spec)
    at_obstacle = sample_distance_field_jax(
        jnp.asarray(field), jnp.asarray([0.5, 0.5, 0.5]),
        jnp.asarray(spec.workspace_min), spec.voxel_size)
    nearby = sample_distance_field_jax(
        jnp.asarray(field), jnp.asarray([0.7, 0.5, 0.5]),
        jnp.asarray(spec.workspace_min), spec.voxel_size)
    outside = sample_distance_field_jax(
        jnp.asarray(field), jnp.asarray([1.1, 0.5, 0.5]),
        jnp.asarray(spec.workspace_min), spec.voxel_size)

    assert float(at_obstacle) <= 1.0e-6
    assert float(nearby) > 0.15
    assert float(outside) < 0.0
    assert jax.jit(sample_distance_field_jax)(
        jnp.asarray(field), jnp.asarray([0.7, 0.5, 0.5]),
        jnp.asarray(spec.workspace_min), spec.voxel_size).shape == ()


def test_numpy_distance_field_sampling_matches_jax_interior_value():
    from work.safety_snapshot import (
        SafetyGridSpec, build_distance_field, sample_distance_field_numpy)

    spec = SafetyGridSpec(np.zeros(3), np.ones(3), 0.1)
    field = build_distance_field(np.array([[0.5, 0.5, 0.5]]), spec)

    assert sample_distance_field_numpy(field, [0.7, 0.5, 0.5],
                                       spec.workspace_min, spec.voxel_size) > 0.15
    assert sample_distance_field_numpy(field, [1.1, 0.5, 0.5],
                                       spec.workspace_min, spec.voxel_size) < 0.0


def test_snapshot_store_has_fixed_track_slots_and_rejects_stale_data():
    from work.safety_snapshot import (
        MAX_DYNAMIC_TRACKS,
        SafetyGridSpec,
        SafetySnapshot,
        SafetySnapshotStore,
    )

    spec = SafetyGridSpec(np.zeros(3), np.ones(3), 0.25)
    snapshot = SafetySnapshot.empty(spec, stamp_s=10.0)
    store = SafetySnapshotStore(max_age_s=0.10)
    store.publish(snapshot)

    assert snapshot.track_positions.shape == (MAX_DYNAMIC_TRACKS, 3)
    assert snapshot.track_velocities.shape == (MAX_DYNAMIC_TRACKS, 3)
    assert snapshot.track_enabled.shape == (MAX_DYNAMIC_TRACKS,)
    assert store.latest(now_s=10.05) is snapshot
    assert store.latest(now_s=10.11) is None


def test_snapshot_dynamic_margin_accounts_for_age_latency_and_braking():
    from work.safety_snapshot import SafetyGridSpec, SafetySnapshot

    spec = SafetyGridSpec(np.zeros(3), np.ones(3), 0.25)
    base = SafetySnapshot.empty(spec, stamp_s=1.0)
    snapshot = SafetySnapshot(
        **{**base.__dict__, 'source_latency_s': 0.05,
           'geometric_error_m': 0.01, 'calibration_error_m': 0.02,
           'max_obstacle_speed_m_s': 1.0,
           'max_obstacle_accel_m_s2': 2.0, 'braking_time_s': 0.10})

    assert snapshot.dynamic_margin(1.05) == pytest.approx(0.27)


def test_ros_pointcloud_header_age_is_included_in_source_latency():
    pytest.importorskip("newaxis")
    from newaxis.pointcloud_safety_adapter import _source_latency_from_header

    class _Clock:
        @staticmethod
        def now():
            return SimpleNamespace(nanoseconds=2_300_000_000)

    class _Node:
        @staticmethod
        def get_clock():
            return _Clock()

    stamp = SimpleNamespace(sec=2, nanosec=100_000_000)
    latency = _source_latency_from_header(
        _Node(), stamp, received_monotonic_s=10.0, now_monotonic_s=10.025)

    assert latency == pytest.approx(0.225)


def test_invalid_or_future_ros_stamp_does_not_create_negative_latency():
    pytest.importorskip("newaxis")
    from newaxis.pointcloud_safety_adapter import _source_latency_from_header

    class _Clock:
        @staticmethod
        def now():
            return SimpleNamespace(nanoseconds=1_000_000_000)

    class _Node:
        @staticmethod
        def get_clock():
            return _Clock()

    future_stamp = SimpleNamespace(sec=2, nanosec=0)
    assert _source_latency_from_header(
        _Node(), future_stamp, received_monotonic_s=10.0,
        now_monotonic_s=10.025) == pytest.approx(0.025)


def test_zero_ros_header_stamp_is_not_accepted_as_a_live_ordering_timestamp():
    pytest.importorskip("newaxis")
    from newaxis.pointcloud_safety_adapter import _header_source_stamp_ns

    assert _header_source_stamp_ns(SimpleNamespace(sec=0, nanosec=0)) is None
    assert _header_source_stamp_ns(SimpleNamespace(sec=1, nanosec=1_000_000_000)) is None


def test_adapter_reject_input_latches_an_explicit_live_perception_fault():
    pytest.importorskip("newaxis")
    from newaxis.pointcloud_safety_adapter import PointCloudSafetyAdapter
    from work.safety_snapshot import SafetyGridSpec, SafetySnapshotStore

    spec = SafetyGridSpec(np.zeros(3), np.ones(3), 0.2)
    adapter = PointCloudSafetyAdapter(spec, SafetySnapshotStore(max_age_s=1.0))
    rejected = adapter.reject_input('PointCloud2 header timestamp is missing', stamp_s=1.0)
    later = adapter.ingest_points(
        np.array([[0.4, 0.5, 0.5]]), np.eye(4), stamp_s=1.1)

    assert rejected.valid is False
    assert later.valid is False
    assert adapter.input_fault_reason == 'PointCloud2 header timestamp is missing'


def test_adapter_publishes_fixed_shape_grid_and_track_slots():
    pytest.importorskip("newaxis")
    from newaxis.pointcloud_safety_adapter import PointCloudSafetyAdapter
    from work.safety_snapshot import SafetyGridSpec, SafetySnapshotStore

    spec = SafetyGridSpec(np.zeros(3), np.ones(3), 0.2)
    store = SafetySnapshotStore(max_age_s=1.0)
    adapter = PointCloudSafetyAdapter(spec, store)
    adapter.set_tracks([
        {"position": [0.7, 0.5, 0.5], "radius": 0.1,
         "velocity": [-0.2, 0.0, 0.0]},
    ])
    snapshot = adapter.ingest_points(
        np.array([[0.4, 0.5, 0.5]]), np.eye(4), stamp_s=4.0)

    assert snapshot.distance_field.shape == spec.shape
    assert snapshot.track_enabled[0] == 1.0
    # A first sighting has no trusted finite-difference velocity yet.
    assert np.allclose(snapshot.track_velocities[0], 0.0)
    assert snapshot.track_ids[0] == 1
    assert np.count_nonzero(snapshot.track_enabled) == 1


def test_adapter_expands_a_large_same_id_measurement_jump_instead_of_stepping_barrier_geometry():
    pytest.importorskip("newaxis")
    from newaxis.hri_perception import SafetyMarginModel
    from newaxis.pointcloud_safety_adapter import PointCloudSafetyAdapter
    from work.safety_snapshot import SafetyGridSpec, SafetySnapshotStore

    spec = SafetyGridSpec(np.zeros(3), np.ones(3), 0.2)
    adapter = PointCloudSafetyAdapter(
        spec, SafetySnapshotStore(max_age_s=1.0),
        safety_margin_model=SafetyMarginModel(max_obstacle_speed_m_s=0.5))
    adapter.set_tracks([{
        'id': 42, 'position': [0.1, 0.5, 0.5], 'radius': 0.05,
    }])
    first = adapter.ingest_points(
        np.array([[0.4, 0.5, 0.5]]), np.eye(4), stamp_s=1.0)
    adapter.set_tracks([{
        'id': 42, 'position': [0.9, 0.5, 0.5], 'radius': 0.05,
    }])
    second = adapter.ingest_points(
        np.array([[0.4, 0.5, 0.5]]), np.eye(4), stamp_s=1.1)

    assert second.track_ids[0] == 42
    assert np.linalg.norm(second.track_positions[0] - first.track_positions[0]) <= 0.050001
    assert second.track_radii[0] >= (
        np.linalg.norm(second.track_positions[0] - first.track_positions[0])
        + first.track_radii[0] - 1.0e-6)
    assert second.track_radii[0] >= (
        np.linalg.norm(second.track_positions[0] - np.array([0.9, 0.5, 0.5]))
        + 0.05 - 1.0e-6)
    assert second.handoff_max_inflation_m > 0.0


def test_adapter_auto_tracks_point_cloud_clusters_and_marks_empty_input_invalid():
    pytest.importorskip("newaxis")
    from newaxis.pointcloud_safety_adapter import PointCloudSafetyAdapter
    from work.safety_snapshot import SafetyGridSpec, SafetySnapshotStore

    spec = SafetyGridSpec(np.zeros(3), np.ones(3), 0.02)
    adapter = PointCloudSafetyAdapter(spec, SafetySnapshotStore(max_age_s=1.0))
    rng = np.random.default_rng(7)
    cloud = np.array([0.5, 0.5, 0.5]) + rng.normal(0.0, 0.005, (12, 3))

    snapshot = adapter.ingest_points(cloud, np.eye(4), stamp_s=4.0)
    empty = adapter.ingest_points(np.empty((0, 3)), np.eye(4), stamp_s=4.1)

    assert np.count_nonzero(snapshot.track_enabled) == 1
    assert empty.valid is False


def test_adapter_inflates_retained_track_radius_during_short_occlusion():
    pytest.importorskip("newaxis")
    from newaxis.hri_perception import DynamicClusterTracker, SafetyMarginModel
    from newaxis.pointcloud_safety_adapter import PointCloudSafetyAdapter
    from work.safety_snapshot import SafetyGridSpec, SafetySnapshotStore

    spec = SafetyGridSpec(np.zeros(3), np.ones(3), 0.02)
    adapter = PointCloudSafetyAdapter(
        spec, SafetySnapshotStore(max_age_s=1.0),
        cluster_tracker=DynamicClusterTracker(min_cluster_points=4),
        safety_margin_model=SafetyMarginModel(max_obstacle_speed_m_s=1.0))
    rng = np.random.default_rng(8)
    first = np.array([0.20, 0.20, 0.20]) + rng.normal(0.0, 0.005, (12, 3))
    other = np.array([0.80, 0.20, 0.20]) + rng.normal(0.0, 0.005, (12, 3))

    initial = adapter.ingest_points(first, np.eye(4), stamp_s=0.0)
    occluded = adapter.ingest_points(other, np.eye(4), stamp_s=0.1)

    assert np.count_nonzero(occluded.track_enabled) == 2
    assert occluded.track_radii[0] > initial.track_radii[0]


def test_adapter_reuses_calibrated_static_esdf_and_tracks_new_dynamic_cluster():
    pytest.importorskip("newaxis")
    from newaxis.hri_perception import DynamicClusterTracker
    from newaxis.pointcloud_safety_adapter import PointCloudSafetyAdapter
    from work.safety_snapshot import SafetyGridSpec, SafetySnapshotStore, build_distance_field

    spec = SafetyGridSpec(np.zeros(3), np.ones(3), 0.02)
    static_field = build_distance_field(np.array([[0.15, 0.15, 0.15]]), spec)
    adapter = PointCloudSafetyAdapter(
        spec, SafetySnapshotStore(max_age_s=1.0),
        cluster_tracker=DynamicClusterTracker(min_cluster_points=4),
        static_distance_field=static_field,
        static_match_distance_m=0.04)
    rng = np.random.default_rng(9)
    dynamic = np.array([0.75, 0.40, 0.40]) + rng.normal(0.0, 0.005, (12, 3))

    snapshot = adapter.ingest_points(dynamic, np.eye(4), stamp_s=1.0)

    np.testing.assert_allclose(snapshot.distance_field, static_field)
    assert np.count_nonzero(snapshot.track_enabled) == 1
    assert snapshot.untracked_dynamic_point_count == 0


def test_static_only_observation_retains_track_then_marks_occlusion_timeout_invalid():
    pytest.importorskip("newaxis")
    from newaxis.hri_perception import DynamicClusterTracker, SafetyMarginModel
    from newaxis.pointcloud_safety_adapter import PointCloudSafetyAdapter
    from work.safety_snapshot import SafetyGridSpec, SafetySnapshotStore, build_distance_field

    spec = SafetyGridSpec(np.zeros(3), np.ones(3), 0.02)
    static_point = np.array([[0.15, 0.15, 0.15]], dtype=np.float32)
    adapter = PointCloudSafetyAdapter(
        spec, SafetySnapshotStore(max_age_s=1.0),
        cluster_tracker=DynamicClusterTracker(min_cluster_points=4, track_timeout_s=0.2),
        safety_margin_model=SafetyMarginModel(max_obstacle_speed_m_s=1.0),
        static_distance_field=build_distance_field(static_point, spec),
        static_match_distance_m=0.04)
    rng = np.random.default_rng(10)
    dynamic = np.array([0.75, 0.40, 0.40]) + rng.normal(0.0, 0.005, (12, 3))

    first = adapter.ingest_points(dynamic, np.eye(4), stamp_s=1.0)
    occluded = adapter.ingest_points(static_point, np.eye(4), stamp_s=1.1)
    timed_out = adapter.ingest_points(static_point, np.eye(4), stamp_s=1.21)

    assert np.count_nonzero(first.track_enabled) == 1
    assert occluded.valid is True
    assert np.count_nonzero(occluded.track_enabled) == 1
    assert occluded.track_radii[0] > first.track_radii[0]
    assert timed_out.valid is False
    assert 'occlusion timeout' in adapter.input_fault_reason


def test_static_esdf_mode_still_marks_an_empty_raw_cloud_invalid():
    pytest.importorskip("newaxis")
    from newaxis.hri_perception import DynamicClusterTracker
    from newaxis.pointcloud_safety_adapter import PointCloudSafetyAdapter
    from work.safety_snapshot import SafetyGridSpec, SafetySnapshotStore, build_distance_field

    spec = SafetyGridSpec(np.zeros(3), np.ones(3), 0.02)
    static_point = np.array([[0.15, 0.15, 0.15]], dtype=np.float32)
    adapter = PointCloudSafetyAdapter(
        spec, SafetySnapshotStore(max_age_s=1.0),
        cluster_tracker=DynamicClusterTracker(min_cluster_points=4),
        static_distance_field=build_distance_field(static_point, spec),
        static_match_distance_m=0.04)

    empty = adapter.ingest_points(np.empty((0, 3)), np.eye(4), stamp_s=1.0)
    later = adapter.ingest_points(static_point, np.eye(4), stamp_s=1.1)

    assert empty.valid is False
    assert later.valid is False
    assert 'empty after filtering' in adapter.input_fault_reason


def test_adapter_latches_out_of_order_source_timestamps_until_reset():
    pytest.importorskip("newaxis")
    from newaxis.pointcloud_safety_adapter import PointCloudSafetyAdapter
    from work.safety_snapshot import SafetyGridSpec, SafetySnapshotStore

    spec = SafetyGridSpec(np.zeros(3), np.ones(3), 0.2)
    adapter = PointCloudSafetyAdapter(spec, SafetySnapshotStore(max_age_s=1.0))
    points = np.array([[0.4, 0.5, 0.5]], dtype=np.float32)

    first = adapter.ingest_points(
        points, np.eye(4), stamp_s=1.0, source_stamp_ns=100)
    rejected = adapter.ingest_points(
        points, np.eye(4), stamp_s=1.1, source_stamp_ns=99)
    still_latched = adapter.ingest_points(
        points, np.eye(4), stamp_s=1.2, source_stamp_ns=101)

    assert first.valid is True
    assert rejected.valid is False
    assert still_latched.valid is False
    assert 'non-monotonic source timestamp' in adapter.input_fault_reason
    adapter.reset_tracking()
    after_reset = adapter.ingest_points(
        points, np.eye(4), stamp_s=2.0, source_stamp_ns=1)

    assert after_reset.valid is True
    assert adapter.input_fault_reason is None


def test_adapter_latches_a_source_timestamp_gap_larger_than_snapshot_age():
    pytest.importorskip("newaxis")
    from newaxis.pointcloud_safety_adapter import PointCloudSafetyAdapter
    from work.safety_snapshot import SafetyGridSpec, SafetySnapshotStore

    spec = SafetyGridSpec(np.zeros(3), np.ones(3), 0.2)
    adapter = PointCloudSafetyAdapter(spec, SafetySnapshotStore(max_age_s=0.10))
    points = np.array([[0.4, 0.5, 0.5]], dtype=np.float32)

    first = adapter.ingest_points(
        points, np.eye(4), stamp_s=1.0, source_stamp_ns=0)
    at_limit = adapter.ingest_points(
        points, np.eye(4), stamp_s=1.1, source_stamp_ns=100_000_000)
    gap = adapter.ingest_points(
        points, np.eye(4), stamp_s=1.2, source_stamp_ns=200_000_001)
    later = adapter.ingest_points(
        points, np.eye(4), stamp_s=1.3, source_stamp_ns=233_333_334)

    assert first.valid is True
    assert at_limit.valid is True
    assert gap.valid is False
    assert later.valid is False
    assert 'source timestamp gap exceeds' in adapter.input_fault_reason


def test_perception_manager_reports_latched_source_timestamp_fault():
    pytest.importorskip("newaxis")
    from newaxis.perception_safety_manager import PerceptionSafetyManager
    from newaxis.pointcloud_safety_adapter import PointCloudSafetyAdapter
    from work.safety_snapshot import SafetyGridSpec, SafetySnapshotStore

    spec = SafetyGridSpec(np.zeros(3), np.ones(3), 0.2)
    store = SafetySnapshotStore(max_age_s=1.0)
    adapter = PointCloudSafetyAdapter(spec, store)
    points = np.array([[0.4, 0.5, 0.5]], dtype=np.float32)
    adapter.ingest_points(points, np.eye(4), stamp_s=time.monotonic(), source_stamp_ns=2)
    adapter.ingest_points(points, np.eye(4), stamp_s=time.monotonic(), source_stamp_ns=1)
    runner = SimpleNamespace(
        _perception_enabled=True,
        _safety_snapshot_store=store,
        _pointcloud_safety_adapter=adapter,
        _latest_safety_snapshot=None,
        _perception_stop_reason=None,
        _last_safety_snapshot_age_ms=float('inf'),
        _last_perception_margin_m=0.0,
        _perception_fault_reported=False,
        DYN_D_SAFE=0.08,
    )

    inputs = PerceptionSafetyManager(runner).safety_snapshot_inputs()

    assert inputs is None
    assert runner._perception_stop_reason == adapter.input_fault_reason


def test_replay_manager_forwards_source_timestamps_to_the_safety_adapter():
    pytest.importorskip("newaxis")
    from newaxis.perception_safety_manager import PerceptionSafetyManager
    from newaxis.pointcloud_replay import PointCloudReplay, RecordedPointCloudFrame
    from newaxis.pointcloud_safety_adapter import PointCloudSafetyAdapter
    from work.safety_snapshot import SafetyGridSpec, SafetySnapshotStore

    spec = SafetyGridSpec(np.zeros(3), np.ones(3), 0.2)
    adapter = PointCloudSafetyAdapter(spec, SafetySnapshotStore(max_age_s=1.0))
    points = np.array([[0.4, 0.5, 0.5]], dtype=np.float32)
    replay = PointCloudReplay((
        RecordedPointCloudFrame(points, source_stamp_ns=2),
        RecordedPointCloudFrame(points, source_stamp_ns=1),
    ), rate_hz=30.0)
    runner = SimpleNamespace(
        _pointcloud_safety_adapter=adapter,
        _replay_source=replay,
        _replay_snapshot=None,
    )
    manager = PerceptionSafetyManager(runner)

    manager.publish_replay_snapshot()
    manager.publish_replay_snapshot()

    assert adapter.input_fault_reason is not None
    assert 'non-monotonic source timestamp' in adapter.input_fault_reason


def test_replay_processing_failure_latches_an_invalid_snapshot_before_next_frame():
    pytest.importorskip("newaxis")
    from newaxis.perception_safety_manager import PerceptionSafetyManager
    from newaxis.pointcloud_safety_adapter import PointCloudSafetyAdapter
    from work.safety_snapshot import SafetyGridSpec, SafetySnapshotStore

    spec = SafetyGridSpec(np.zeros(3), np.ones(3), 0.2)
    store = SafetySnapshotStore(max_age_s=1.0)
    adapter = PointCloudSafetyAdapter(spec, store)
    runner = SimpleNamespace(_pointcloud_safety_adapter=adapter)

    snapshot = PerceptionSafetyManager(runner)._latch_replay_failure(
        ValueError('malformed replay frame'), stamp_s=1.0)
    later = adapter.ingest_points(
        np.array([[0.4, 0.5, 0.5]]), np.eye(4), stamp_s=1.1)

    assert snapshot.valid is False
    assert later.valid is False
    assert adapter.input_fault_reason == 'Point-cloud replay processing failure: ValueError'


def test_adapter_marks_unclustered_dynamic_points_for_a_safety_stop_with_static_esdf():
    pytest.importorskip("newaxis")
    from newaxis.hri_perception import DynamicClusterTracker
    from newaxis.pointcloud_safety_adapter import PointCloudSafetyAdapter
    from work.safety_snapshot import SafetyGridSpec, SafetySnapshotStore, build_distance_field

    spec = SafetyGridSpec(np.zeros(3), np.ones(3), 0.02)
    static_field = build_distance_field(np.array([[0.15, 0.15, 0.15]]), spec)
    adapter = PointCloudSafetyAdapter(
        spec, SafetySnapshotStore(max_age_s=1.0),
        cluster_tracker=DynamicClusterTracker(min_cluster_points=20),
        static_distance_field=static_field,
        static_match_distance_m=0.04)
    dynamic = np.array([
        [0.75, 0.40, 0.40], [0.76, 0.40, 0.40], [0.75, 0.41, 0.40],
        [0.76, 0.41, 0.40], [0.75, 0.40, 0.41], [0.76, 0.40, 0.41],
    ])

    snapshot = adapter.ingest_points(dynamic, np.eye(4), stamp_s=1.0)

    assert snapshot.valid is True
    assert snapshot.untracked_dynamic_point_count > 0
    assert 'untracked dynamic' in adapter.input_fault_reason


def test_adapter_latches_track_slot_overflow_until_explicit_reset():
    pytest.importorskip("newaxis")
    from newaxis.hri_perception import DynamicClusterTracker
    from newaxis.pointcloud_safety_adapter import PointCloudSafetyAdapter
    from work.safety_snapshot import SafetyGridSpec, SafetySnapshotStore

    spec = SafetyGridSpec(np.zeros(3), np.ones(3), 0.02)
    adapter = PointCloudSafetyAdapter(
        spec, SafetySnapshotStore(max_age_s=1.0),
        cluster_tracker=DynamicClusterTracker(
            cluster_radius_m=0.04, min_cluster_points=4, max_tracks=1))
    rng = np.random.default_rng(13)
    points = np.vstack((
        np.array([0.20, 0.30, 0.50]) + rng.normal(0.0, 0.002, (6, 3)),
        np.array([0.80, 0.30, 0.50]) + rng.normal(0.0, 0.002, (6, 3)),
    ))

    overflow = adapter.ingest_points(points, np.eye(4), stamp_s=1.0)
    later = adapter.ingest_points(points[:6], np.eye(4), stamp_s=1.1)

    assert overflow.track_overflow is True
    assert later.valid is False
    assert 'track-slot overflow' in adapter.input_fault_reason


def test_perception_manager_refuses_untracked_dynamic_points_in_static_esdf_mode():
    pytest.importorskip("newaxis")
    from newaxis.perception_safety_manager import PerceptionSafetyManager
    from work.safety_snapshot import SafetyGridSpec, SafetySnapshot, SafetySnapshotStore

    spec = SafetyGridSpec(np.zeros(3), np.ones(3), 0.25)
    store = SafetySnapshotStore(max_age_s=1.0)
    store.publish(replace(
        SafetySnapshot.empty(spec, stamp_s=time.monotonic()),
        untracked_dynamic_point_count=3))
    runner = SimpleNamespace(
        _perception_enabled=True,
        _safety_snapshot_store=store,
        _latest_safety_snapshot=None,
        _perception_stop_reason=None,
        _last_safety_snapshot_age_ms=float('inf'),
        _last_perception_margin_m=0.0,
        _perception_fault_reported=False,
        DYN_D_SAFE=0.08,
    )

    inputs = PerceptionSafetyManager(runner).safety_snapshot_inputs()

    assert inputs is None
    assert 'unexplained dynamic point' in runner._perception_stop_reason
