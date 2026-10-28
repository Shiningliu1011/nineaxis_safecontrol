import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
import trimesh
from test_collision_safety import _config, _identities
from work.collision_safety import CollisionSafety, CollisionStatus
from work.lidar_occupancy import (
    LidarOccupancyScene,
    LidarRayBatch,
    OccupancyGrid,
    RayReturn,
    VoxelState,
)
from work.lidar_self_filter import (
    ClockMapping,
    FilterCalibration,
    GeometryValidation,
    JointStateHistory,
    MeshRaySelfFilter,
    RobotRayGeometry,
    SelfFilterBounds,
    TimedLidarScan,
)

ROOT = Path(__file__).resolve().parents[2]


def box_at(x, link="base_link"):
    mesh = trimesh.creation.box(extents=[1., 1., 1.])
    mesh.apply_translation([x, 0., 0.])
    return link, mesh


def test_first_hit_uses_nearest_original_triangles_across_links():
    geometry = RobotRayGeometry.from_link_meshes(
        [box_at(3.), box_at(5., "Link1")], source_id="analytic_boxes_1m_v1",
    )
    result = geometry.first_hits(np.array([[0., 0., 0.], [0., 2., 0.]]),
                                 np.array([[1., 0., 0.], [1., 0., 0.]]), np.zeros((2, 9)))
    np.testing.assert_array_equal(result.measured, [True, False])
    assert result.distance_m[0] == pytest.approx(2.5)
    assert result.link_name[0] == "base_link"
    assert result.triangle_index[0] >= 0


def test_gap_between_disconnected_mesh_parts_preserves_environment_ray():
    left = trimesh.creation.box(extents=[1., .2, 1.])
    left.apply_translation([3., -.5, 0.])
    right = left.copy()
    right.apply_translation([0., 1., 0.])
    mesh = trimesh.util.concatenate([left, right])
    geometry = RobotRayGeometry.from_link_meshes([("base_link", mesh)], source_id="analytic_gap_v1")
    result = geometry.first_hits(np.array([[0., 0., 0.]]), np.array([[1., 0., 0.]]), np.zeros((1, 9)))
    assert not result.measured[0]


def test_first_hit_identifies_collision_part_within_same_link():
    geometry = RobotRayGeometry.from_link_meshes([box_at(5.), box_at(3.)], source_id="analytic_same_link_parts_v1")
    result = geometry.first_hits(np.zeros((1, 3)), np.array([[1., 0., 0.]]), np.zeros((1, 9)))
    assert result.distance_m[0] == pytest.approx(2.5)
    assert result.part_index[0] == 1


def test_original_nineaxis_collision_mesh_and_shared_fk_sources():
    geometry = RobotRayGeometry.from_urdf(
        ROOT / "models/ninezzhou/urdf/ninezzhou.urdf",
        package_roots={"ninezzhou": ROOT / "models/ninezzhou"},
    )
    assert geometry.triangle_count == 696330
    assert len(geometry.parts) == 10
    assert geometry.urdf_sha256 == "766ea5870c41f110d7d57b46e6b31153335f45358faf15e641d465b3b31587a3"
    assert not geometry.admission_ready
    assert geometry.validation_reason == "geometry_evidence_missing"
    original = trimesh.load_mesh(ROOT / "models/ninezzhou/meshes/Link9.STL", process=False)
    np.testing.assert_array_equal(geometry.parts[-1].triangles_m, original.triangles)
    q = np.zeros((2, 9))
    q[1, 0] = .05
    origins = np.array([[0., 0., 0.], [0., 0., .05]])
    # 固定基座仍有自己的来源和几何，不按活动连杆删除。
    assert geometry.parts[0].link_name == "base_link"
    assert geometry.first_hits(origins, np.array([[1., 0., 0.]] * 2), q).measured.shape == (2,)


def analytic_filter(link="base_link", errors_mm=0.):
    geometry = RobotRayGeometry.from_link_meshes([box_at(3., link)], source_id="analytic_box_1m_v1")
    geometry = geometry.with_validation(GeometryValidation(geometry.geometry_hash, 0., "analytic_closed_box", True))
    calibration = FilterCalibration(np.eye(4), "livox_frame", "base_link", "analytic_identity", True)
    bounds = SelfFilterBounds(
        range_error_mm=errors_mm, incidence_error_mm=0., angular_error_rad=0.,
        translation_error_mm=0., rotation_error_rad=0., interpolation_error_si=np.zeros(9),
        joint_speed_bound_si=np.ones(9), max_history_gap_ns=1_000_000_000,
        timestamp_error_bound_ns=100, source_id="analytic_exact_coordinates_v1",
    )
    history = JointStateHistory(
        stamp_ns=np.array([1_000_000_000, 2_000_000_000]), q_si=np.zeros((2, 9)),
        timestamp_error_ns=np.zeros(2, dtype=np.int64), feedback_source="simulation",
        model_hash=geometry.geometry_hash, zero_calibration_id="analytic_zero", clock_mapping_id="joint_clock",
        run_id="analytic_run", target_clock_id="host_monotonic",
    )
    return MeshRaySelfFilter(geometry, calibration, bounds), history


def scan_for(points, stamp_ns=None, scan_id=1, errors_ns=None):
    points = np.asarray(points, dtype=float)
    count = len(points)
    stamps = np.full(count, 1_500_000_000, dtype=np.int64) if stamp_ns is None else np.array(stamp_ns, dtype=np.int64)
    return TimedLidarScan(
        points_lidar_m=points, point_stamp_ns=stamps,
        point_time_error_ns=np.zeros(count, dtype=np.int64) if errors_ns is None else np.array(errors_ns, dtype=np.int64),
        original_stamp_ns=stamps, row_indices=np.arange(count, dtype=np.int64), scan_id=scan_id,
        source_clock_id="lidar_clock", clock_mapping_id="lidar_mapping", frame_id="livox_frame",
        received_monotonic_ns=2_000_000_000, max_range_m=8.,
        run_id="analytic_run", target_clock_id="host_monotonic",
    )


def test_classification_before_at_after_surface_and_gap():
    module, history = analytic_filter()
    result = module.classify(scan_for([[2., 0., 0.], [2.5, 0., 0.], [4., 0., 0.], [4., 2., 0.]]), history)
    assert result.input_valid
    assert result.classification == ("environment", "self", "occluded", "environment")
    np.testing.assert_allclose(result.visible_free_until_m[:3], [2., 2.5, 2.5])
    np.testing.assert_allclose(result.unknown_start_m[1:3], [2.5, 2.5])
    np.testing.assert_array_equal(result.row_indices, np.arange(4))


def test_uncertainty_keeps_close_obstacle_ambiguous():
    module, history = analytic_filter(errors_mm=2.)
    result = module.classify(scan_for([[2.49, 0., 0.], [2.499, 0., 0.], [2.5, 0., 0.], [4., 0., 0.]]), history)
    assert result.classification == ("environment", "ambiguous", "ambiguous", "occluded")
    assert result.visible_free_until_m[1] <= 2.497


def test_scanning_motion_uses_each_point_history_and_shared_fk():
    module, history = analytic_filter(link="Link1")
    q = np.zeros((2, 9))
    q[1, 0] = .5
    history = JointStateHistory(
        history.stamp_ns, q, history.timestamp_error_ns, history.feedback_source,
        history.model_hash, history.zero_calibration_id, history.clock_mapping_id,
        history.run_id, history.target_clock_id,
    )
    # Link1 沿 +Z 移动，第二个点沿同样方向移动，表面距离保持 2500 mm。
    result = module.classify(scan_for([[2.5, 0., 0.], [2.5, 0., .5]], [1_000_000_000, 2_000_000_000]), history)
    assert result.input_valid
    assert result.first_hit_measured.all()
    np.testing.assert_allclose(result.first_hit_distance_m, [2.5, np.linalg.norm([2.5, .5])])


@pytest.mark.parametrize("failure", ["history_missing", "history_gap", "clock_error", "future_point", "model_identity", "uncalibrated"])
def test_invalid_sources_return_explicit_non_admissible_result(failure):
    from dataclasses import replace

    module, history = analytic_filter()
    scan = scan_for([[2., 0., 0.]])
    if failure == "history_missing":
        history = replace(history, stamp_ns=np.array([1_600_000_000, 2_000_000_000]))
    elif failure == "history_gap":
        module = MeshRaySelfFilter(module.geometry, module.calibration, replace(module.bounds, max_history_gap_ns=100))
    elif failure == "clock_error":
        scan = replace(scan, point_time_error_ns=np.array([101], dtype=np.int64))
    elif failure == "future_point":
        scan = replace(scan, received_monotonic_ns=1_499_999_999)
    elif failure == "model_identity":
        history = replace(history, model_hash="b" * 64)
    else:
        module = MeshRaySelfFilter(module.geometry, replace(module.calibration, admission_ready=False), module.bounds)
    result = module.classify(scan, history)
    assert not result.input_valid
    assert result.reason != "ok"
    assert result.classification == ("ambiguous",)
    assert result.visible_free_until_m[0] == 0.
    assert not result.to_ray_batch().input_valid


def test_clock_mapping_preserves_integer_epochs_and_float_quantization():
    mapping = ClockMapping(
        source_clock_id="device_epoch", mapping_id="clock_sync_1", source_anchor_ns=1_800_000_000_000_000_000,
        monotonic_anchor_ns=1_000_000_000, valid_source_start_ns=1_799_999_999_000_000_000,
        valid_source_end_ns=1_800_000_001_000_000_000, offset_error_ns=10, rate_error_ppb=0., evidence_id="exact_anchors",
        run_id="analytic_run", target_clock_id="host_monotonic",
    )
    stamps, errors = mapping.map(np.array([1_800_000_000_000_000_001, 1_800_000_000_000_000_002], dtype=np.int64))
    np.testing.assert_array_equal(stamps, [1_000_000_001, 1_000_000_002])
    np.testing.assert_array_equal(errors, [10, 10])
    _, floating_errors = mapping.map(np.array([1_800_000_000_000_000_000.]))
    assert floating_errors[0] >= 138
    with pytest.raises(ValueError, match="有效范围"):
        mapping.map(np.array([1_800_000_002_000_000_000], dtype=np.int64))


def occupancy_at(required_index):
    config = _config(
        voxel_size_mm=100., lidar_range_error_mm=0., lidar_incidence_error_mm=0.,
        lidar_angular_error_rad=0., extrinsic_translation_error_mm=0., extrinsic_rotation_error_rad=0.,
        untracked_speed_mm_s=0., untracked_acceleration_mm_s2=0.,
    )
    required = np.zeros((50, 1, 1), dtype=bool)
    required[required_index, 0, 0] = True
    grid = OccupancyGrid(np.array([0., -.05, -.05]), (50, 1, 1), 100., required)
    identities = _identities(config)._replace(required_space_hash=np.frombuffer(grid.required_space_hash, dtype=np.uint8))
    return LidarOccupancyScene(config, identities, grid), CollisionSafety(config)


def raw_ray(stamp, distance=4.5, scan_id=0):
    return LidarRayBatch(np.zeros((1, 3)), np.array([[distance, 0., 0.]]), np.array([stamp], dtype=np.int64),
                         np.array([RayReturn.HIT], dtype=np.int32), scan_id)


def real_time_history(history, stamp):
    return replace(history, stamp_ns=np.array([stamp - 500_000_000, stamp + 500_000_000], dtype=np.int64))


def test_occlusion_revokes_old_free_keeps_occupied_and_rejects_real_prepare_scene():
    producer, safety = occupancy_at(30)
    module, history = analytic_filter()
    stamp = time.monotonic_ns() - 200_000_000
    producer.update(raw_ray(stamp), now_ns=stamp)
    before = producer.snapshot(now_ns=stamp)
    assert before.cells[30, 0, 0] == VoxelState.FREE
    assert int(safety.prepare_scene(before.scene, before.scene.identities).status) == CollisionStatus.OK
    point_stamp = stamp + 20_000_000
    scan = replace(scan_for([[4., 0., 0.]], [point_stamp]), received_monotonic_ns=point_stamp)
    filtered = module.classify(scan, real_time_history(history, stamp))
    assert filtered.classification == ("occluded",)
    producer.update(filtered.to_ray_batch(), now_ns=point_stamp)
    after = producer.snapshot(now_ns=point_stamp)
    assert after.cells[20, 0, 0] == VoxelState.FREE
    assert after.cells[30, 0, 0] == VoxelState.UNKNOWN
    assert after.cells[45, 0, 0] == VoxelState.OCCUPIED
    assert after.scene.metadata.support_count == 1
    prepared = safety.prepare_scene(after.scene, after.scene.identities)
    assert int(prepared.status) == CollisionStatus.UNKNOWN_REQUIRED_SPACE
    assert not np.any(prepared.support_valid_mask)


def test_environment_before_body_keeps_real_occupied_support():
    producer, _ = occupancy_at(20)
    module, history = analytic_filter()
    stamp = time.monotonic_ns()
    scan = replace(scan_for([[2.05, 0., 0.]], [stamp]), received_monotonic_ns=stamp)
    result = module.classify(scan, real_time_history(history, stamp))
    assert result.classification == ("environment",)
    producer.update(result.to_ray_batch(), now_ns=stamp)
    snapshot = producer.snapshot(now_ns=stamp)
    assert snapshot.cells[20, 0, 0] == VoxelState.OCCUPIED
    assert snapshot.scene.metadata.support_count == 1
    assert snapshot.scene.support_points_m[snapshot.scene.support_mask][0, 0] == pytest.approx(2.05)


def test_invalid_history_revokes_old_free_and_valid_evidence_can_restore_front_coverage():
    producer, safety = occupancy_at(10)
    module, history = analytic_filter()
    stamp = time.monotonic_ns() - 200_000_000
    producer.update(raw_ray(stamp), now_ns=stamp)
    invalid_stamp = stamp + 20_000_000
    scan = replace(scan_for([[2., 0., 0.]], [invalid_stamp]), received_monotonic_ns=invalid_stamp)
    result = module.classify(scan, history)
    assert not result.input_valid
    producer.update(result.to_ray_batch(), now_ns=invalid_stamp)
    snapshot = producer.snapshot(now_ns=invalid_stamp)
    assert snapshot.cells[10, 0, 0] == VoxelState.UNKNOWN
    assert snapshot.cells[45, 0, 0] == VoxelState.OCCUPIED
    assert int(safety.prepare_scene(snapshot.scene, snapshot.scene.identities).status) == CollisionStatus.INVALID_SCENE
    valid_stamp = stamp + 40_000_000
    valid_scan = replace(scan, point_stamp_ns=np.array([valid_stamp]), original_stamp_ns=np.array([valid_stamp]),
                         received_monotonic_ns=valid_stamp, scan_id=2)
    producer.update(module.classify(valid_scan, real_time_history(history, stamp)).to_ray_batch(), now_ns=valid_stamp)
    recovered = producer.snapshot(now_ns=valid_stamp)
    assert recovered.cells[10, 0, 0] == VoxelState.FREE
    assert int(safety.prepare_scene(recovered.scene, recovered.scene.identities).status) == CollisionStatus.OK


def test_same_batch_occlusion_dominates_later_clear_ray():
    producer, _ = occupancy_at(30)
    module, history = analytic_filter(link="Link1")
    stamp = time.monotonic_ns() - 1_200_000_000
    q = np.zeros((2, 9))
    q[1, 0] = .585
    history = replace(history, stamp_ns=np.array([stamp, stamp + 1_000_000_000]), q_si=q)
    scan = replace(scan_for([[4., 0., 0.], [4.5, 0., 0.]], [stamp, stamp + 1_000_000_000]),
                   received_monotonic_ns=stamp + 1_000_000_000)
    result = module.classify(scan, history)
    assert result.classification == ("occluded", "environment")
    producer.update(result.to_ray_batch(), now_ns=scan.received_monotonic_ns)
    snapshot = producer.snapshot(now_ns=scan.received_monotonic_ns)
    assert snapshot.cells[30, 0, 0] == VoxelState.UNKNOWN
    assert snapshot.ambiguous_mask[30, 0, 0]


def test_filter_source_change_needs_new_scene_and_cannot_reuse_old_free():
    producer, safety = occupancy_at(10)
    module, history = analytic_filter()
    stamp = time.monotonic_ns() - 100_000_000
    history = real_time_history(history, stamp)
    scan = replace(scan_for([[2., 0., 0.]], [stamp]), received_monotonic_ns=stamp)
    producer.update(module.classify(scan, history).to_ray_batch(), now_ns=stamp)
    changed = MeshRaySelfFilter(module.geometry, module.calibration, replace(module.bounds, range_error_mm=.1))
    scan = replace(scan, scan_id=2, point_stamp_ns=np.array([stamp + 20_000_000]),
                   original_stamp_ns=np.array([stamp + 20_000_000]), received_monotonic_ns=stamp + 20_000_000)
    producer.update(changed.classify(scan, history).to_ray_batch(), now_ns=scan.received_monotonic_ns)
    snapshot = producer.snapshot(now_ns=scan.received_monotonic_ns)
    assert snapshot.cells[10, 0, 0] == VoxelState.UNKNOWN
    assert producer.invalid_reason == "filter_identity_changed_new_scene_required"
    assert int(safety.prepare_scene(snapshot.scene, snapshot.scene.identities).status) == CollisionStatus.INVALID_SCENE
    previous = replace(scan, scan_id=3, point_stamp_ns=scan.point_stamp_ns + 20_000_000,
                       original_stamp_ns=scan.original_stamp_ns + 20_000_000,
                       received_monotonic_ns=scan.received_monotonic_ns + 20_000_000)
    producer.update(module.classify(previous, history).to_ray_batch(), now_ns=previous.received_monotonic_ns)
    current = producer.snapshot(now_ns=previous.received_monotonic_ns)
    assert int(safety.prepare_scene(current.scene, current.scene.identities).status) == CollisionStatus.INVALID_SCENE


def test_collision_origin_scale_and_resource_mutation_are_checked(tmp_path):
    from scipy.spatial.transform import Rotation
    from urdf_parser_py.urdf import URDF, Pose

    robot = URDF.from_xml_string((ROOT / "models/ninezzhou/urdf/ninezzhou.urdf").read_bytes())
    part = robot.link_map["Link9"].collisions[0]
    part.geometry.scale = [2., 3., 4.]
    part.origin = Pose(xyz=[.1, .2, .3], rpy=[.2, .3, .4])
    source = tmp_path / "origin_scale.urdf"
    source.write_text(robot.to_xml_string())
    geometry = RobotRayGeometry.from_urdf(source, package_roots={"ninezzhou": ROOT / "models/ninezzhou"})
    raw = trimesh.load_mesh(ROOT / "models/ninezzhou/meshes/Link9.STL", process=False)
    expected = (raw.triangles * [2., 3., 4.]) @ Rotation.from_euler("xyz", [.2, .3, .4]).as_matrix().T + [.1, .2, .3]
    np.testing.assert_allclose(geometry.parts[-1].triangles_m, expected, rtol=0, atol=1e-15)
    source.write_text(robot.to_xml_string() + "\n")
    with pytest.raises(ValueError, match="URDF 内容改变"):
        geometry.verify_identity()


def test_source_joint_change_cannot_use_existing_shared_fk(tmp_path):
    from urdf_parser_py.urdf import URDF

    robot = URDF.from_xml_string((ROOT / "models/ninezzhou/urdf/ninezzhou.urdf").read_bytes())
    robot.joint_map["J3"].origin.xyz[0] += .01
    source = tmp_path / "different_chain.urdf"
    source.write_text(robot.to_xml_string())
    with pytest.raises(ValueError, match="共享 FK"):
        RobotRayGeometry.from_urdf(source, package_roots={"ninezzhou": ROOT / "models/ninezzhou"})


def test_clock_run_identity_and_whole_point_time_interval_are_required():
    module, history = analytic_filter()
    scan = scan_for([[2., 0., 0.]])
    result = module.classify(replace(scan, run_id="another_run"), history)
    assert result.reason == "clock_run_identity_mismatch"
    scan = scan_for([[2., 0., 0.]], [1_000_000_000], errors_ns=[1])
    result = module.classify(scan, history)
    assert result.reason == "history_missing"


def test_nonzero_geometry_error_uses_conservative_entry_and_interior_witness():
    module, history = analytic_filter()
    geometry = module.geometry.with_validation(GeometryValidation(module.geometry.geometry_hash, 1., "analytic_1mm_bound", True))
    module = MeshRaySelfFilter(geometry, module.calibration, module.bounds)
    result = module.classify(scan_for([[2., 0., 0.], [2.5, 0., 0.], [4., 0., 0.]]), history)
    assert result.classification == ("environment", "ambiguous", "occluded")
    assert result.first_hit_lower_m[1] <= 2.499
    assert result.first_hit_upper_m[1] >= 2.5
    assert result.first_hit_upper_valid[1]


def test_sensor_origin_inside_robot_cannot_create_free():
    module, history = analytic_filter()
    transform = np.eye(4)
    transform[0, 3] = 3.
    module = MeshRaySelfFilter(module.geometry, replace(module.calibration, T_base_lidar=transform), module.bounds)
    result = module.classify(scan_for([[1., 0., 0.]]), history)
    assert result.classification == ("ambiguous",)
    assert result.visible_free_until_m[0] == 0.


def test_geometry_identity_is_readonly_and_result_preserves_input_content_ids():
    module, history = analytic_filter()
    with pytest.raises(AttributeError):
        module.geometry.geometry_hash = "b" * 64
    scan = scan_for([[2., 0., 0.]])
    result = module.classify(scan, history)
    assert history.identity in result.error_sources
    assert scan.identity in result.error_sources
    changed_q = np.array(history.q_si)
    changed_q[:, 0] = .1
    assert replace(history, q_si=changed_q).identity != history.identity


def test_environment_return_before_body_still_revokes_old_free_behind_body():
    producer, safety = occupancy_at(30)
    module, history = analytic_filter(link="Link1")
    now = time.monotonic_ns()
    base = now - 900_000_000
    q = np.zeros((3, 9))
    q[:2, 0] = .585
    history = replace(history, stamp_ns=np.array([base - 100_000_000, base, base + 700_000_000]),
                      q_si=q, timestamp_error_ns=np.zeros(3, dtype=np.int64))
    first = replace(scan_for([[4.5, 0., 0.]], [base]), received_monotonic_ns=now)
    result = module.classify(first, history)
    assert result.classification == ("environment",)
    producer.update(result.to_ray_batch(), now_ns=now)
    clear = producer.snapshot(now_ns=now)
    assert clear.cells[30, 0, 0] == VoxelState.FREE
    assert int(safety.prepare_scene(clear.scene, clear.scene.identities).status) == CollisionStatus.OK
    second = replace(scan_for([[2., 0., 0.]], [base + 700_000_000], scan_id=2), received_monotonic_ns=now)
    result = module.classify(second, history)
    assert result.classification == ("environment",)
    producer.update(result.to_ray_batch(), now_ns=now)
    blocked = producer.snapshot(now_ns=now)
    assert blocked.cells[20, 0, 0] == VoxelState.OCCUPIED
    assert blocked.cells[30, 0, 0] == VoxelState.UNKNOWN
    assert int(safety.prepare_scene(blocked.scene, blocked.scene.identities).status) == CollisionStatus.UNKNOWN_REQUIRED_SPACE
