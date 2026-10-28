import json
import sys
import time
from hashlib import sha256
from pathlib import Path

import numpy as np
import pytest
import trimesh
from sensor_msgs.msg import JointState, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from work.collision_safety import CollisionSafety, CollisionStatus
from work.lidar_occupancy import LidarOccupancyScene, OccupancyGrid, VoxelState
from work.lidar_self_filter import (
    ClockMapping,
    FilterCalibration,
    GeometryValidation,
    MeshRaySelfFilter,
    RobotRayGeometry,
    SelfFilterBounds,
)

from robot_safecontrol_moveit.calibration_record import load_calibration_record
from robot_safecontrol_moveit.lidar_self_filter_adapter import (
    calibration_for_self_filter,
    history_from_joint_states,
    scan_from_pointcloud,
    update_scene_from_pointcloud,
)
from robot_safecontrol_moveit.robot_spec import DEFAULT_JOINT_NAMES

ROOT = Path(__file__).resolve().parents[1]


def clock_mapping():
    return ClockMapping(
        source_clock_id="analytic_ros", mapping_id="sync_v1", source_anchor_ns=1_000_000_000,
        monotonic_anchor_ns=2_000_000_000, valid_source_start_ns=1_000_000_000,
        valid_source_end_ns=3_000_000_000, offset_error_ns=0, rate_error_ppb=0.,
        evidence_id="analytic_one_second_offset", run_id="analytic_run", target_clock_id="host_monotonic",
    )


def cloud_message():
    fields = [PointField(name=name, offset=offset, datatype=datatype, count=1)
              for name, offset, datatype in [("x", 0, PointField.FLOAT32), ("y", 4, PointField.FLOAT32),
                                              ("z", 8, PointField.FLOAT32), ("intensity", 12, PointField.FLOAT32),
                                              ("tag", 16, PointField.UINT8), ("line", 17, PointField.UINT8),
                                              ("timestamp", 18, PointField.FLOAT64)]]
    return point_cloud2.create_cloud(
        Header(frame_id="livox_frame"), fields,
        [(2., 0., 0., 1., 0, 0, 1_000_000_000.), (3., .1, .2, 2., 0, 1, 1_500_000_000.)],
    )


def test_real_pointcloud2_preserves_unaligned_float64_point_time_and_rows():
    cloud = cloud_message()
    original = bytes(cloud.data)
    mapping = clock_mapping()
    scan = scan_from_pointcloud(cloud, mapping=mapping, scan_id=7, received_monotonic_ns=3_000_000_000, max_range_m=8.)
    np.testing.assert_allclose(scan.points_lidar_m, [[2., 0., 0.], [3., .1, .2]])
    np.testing.assert_array_equal(scan.point_stamp_ns, [2_000_000_000, 2_500_000_000])
    np.testing.assert_array_equal(scan.row_indices, [0, 1])
    np.testing.assert_array_equal(scan.original_stamp_ns, [1_000_000_000., 1_500_000_000.])
    assert scan.point_time_error_ns.min() >= 1
    assert scan.clock_mapping_id == mapping.identity
    assert scan.source_clock_id == "analytic_ros"
    assert scan.run_id == mapping.run_id
    assert bytes(cloud.data) == original


def test_real_jointstate_adapter_uses_names_and_feedback_identity():
    mapping = clock_mapping()
    samples = []
    for seconds, j1 in [(1, .1), (2, .2)]:
        message = JointState()
        message.header.stamp.sec = seconds
        message.name = list(reversed(DEFAULT_JOINT_NAMES))
        message.position = [0.] * 8 + [j1]
        samples.append(message)
    history = history_from_joint_states(
        samples, mapping=mapping, feedback_source="simulation", model_hash="a" * 64,
        zero_calibration_id="analytic_zero",
    )
    np.testing.assert_allclose(history.q_si[:, 0], [.1, .2])
    np.testing.assert_array_equal(history.q_si[:, 1:], 0.)
    np.testing.assert_array_equal(history.stamp_ns, [2_000_000_000, 3_000_000_000])
    assert history.clock_mapping_id == mapping.identity
    samples[0].name[-1] = "J2"
    with pytest.raises(ValueError, match="关节"):
        history_from_joint_states(samples, mapping=mapping, feedback_source="simulation", model_hash="a" * 64,
                                  zero_calibration_id="analytic_zero")


def test_unverified_current_calibration_remains_non_admissible():
    record = load_calibration_record(explicit=ROOT / "config/sensor_extrinsics.yaml")
    calibration = calibration_for_self_filter(record)
    assert not calibration.admission_ready
    assert calibration.frame_from == "livox_frame"
    assert calibration.frame_to == "base_link"
    assert record.file_sha256 in calibration.calibration_id


def test_pointcloud_without_timestamp_is_rejected():
    cloud = point_cloud2.create_cloud_xyz32(Header(frame_id="livox_frame"), [[1., 0., 0.]])
    with pytest.raises(ValueError, match="timestamp"):
        scan_from_pointcloud(cloud, mapping=clock_mapping(), scan_id=1, received_monotonic_ns=2_000_000_000, max_range_m=8.)


def test_analytic_file_replay_uses_messages_filter_occupancy_and_real_prepare_scene():
    portable_tests = str(ROOT / "portable_oscbf/tests")
    if portable_tests not in sys.path:
        sys.path.insert(0, portable_tests)
    from test_collision_safety import _config, _identities

    raw = (ROOT / "tests/data/lidar_self_filter/analytic_scan_v1.json").read_bytes()
    record = json.loads(raw)
    assert record["evidence_kind"] == "analytic_interface_replay"
    identity = sha256(raw).hexdigest()
    source = record["geometry"]
    mesh = trimesh.creation.box(extents=source["box_extents_m"])
    mesh.apply_translation(source["translation_m"])
    geometry = RobotRayGeometry.from_link_meshes([(source["link"], mesh)], source_id=identity)
    geometry = geometry.with_validation(GeometryValidation(geometry.geometry_hash, 0., record["validation"], True))
    calibration = FilterCalibration(np.array(record["calibration"]["matrix"]), record["frame_id"], "base_link",
                                    record["calibration"]["id"], True)
    config = _config(
        voxel_size_mm=100., lidar_range_error_mm=0., lidar_incidence_error_mm=0.,
        lidar_angular_error_rad=0., extrinsic_translation_error_mm=0., extrinsic_rotation_error_rad=0.,
        joint_interpolation_prismatic_error_mm=0., joint_interpolation_revolute_error_rad=0.,
        untracked_speed_mm_s=0., untracked_acceleration_mm_s2=0.,
    )
    bounds = SelfFilterBounds.from_collision_policy(
        config.policy, joint_speed_bound_si=record["limits"]["joint_speed_bound_si"],
        maximum_gap_ns=record["limits"]["history_max_gap_ns"], history_evidence_id=identity,
    )
    anchor = time.monotonic_ns() - 1_100_000_000
    mapping = ClockMapping(record["raw_clock_id"], identity, 1_000_000_000, anchor, 1_000_000_000, 2_000_000_000,
                           0, 0., record["validation"], identity, "host_monotonic_analytic_replay")
    feedback = []
    for sample in record["joint_history"]:
        message = JointState()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(sample["stamp_ns"], 1_000_000_000)
        message.name = list(DEFAULT_JOINT_NAMES)
        message.position = sample["q_si"]
        feedback.append(message)
    history = history_from_joint_states(feedback, mapping=mapping, feedback_source="simulation",
                                        model_hash=geometry.geometry_hash, zero_calibration_id=identity)
    fields = [PointField(name=name, offset=offset, datatype=datatype, count=1)
              for name, offset, datatype in [("x", 0, PointField.FLOAT32), ("y", 4, PointField.FLOAT32),
                                              ("z", 8, PointField.FLOAT32), ("timestamp", 12, PointField.FLOAT64)]]
    cloud = point_cloud2.create_cloud(Header(frame_id=record["frame_id"]), fields,
                                      [(*row["point_m"], float(row["stamp_ns"])) for row in record["returns"]])
    now = time.monotonic_ns()
    scan = scan_from_pointcloud(cloud, mapping=mapping, scan_id=1, received_monotonic_ns=now,
                                max_range_m=record["max_range_m"])
    module = MeshRaySelfFilter(geometry, calibration, bounds)
    result = module.classify(scan, history)
    assert result.classification == tuple(row["expected"] for row in record["returns"])
    required = np.zeros((50, 1, 1), dtype=bool)
    required[30, 0, 0] = True
    grid = OccupancyGrid(np.array([0., -.05, -.05]), (50, 1, 1), 100., required)
    identities = _identities(config)._replace(required_space_hash=np.frombuffer(grid.required_space_hash, dtype=np.uint8))
    producer = LidarOccupancyScene(config, identities, grid)
    updated = update_scene_from_pointcloud(
        cloud, mapping=mapping, scan_id=1, received_monotonic_ns=now, now_ns=now,
        max_range_m=record["max_range_m"], filter_module=module, history=history, producer=producer,
    )
    assert updated.classification == result.classification
    snapshot = producer.snapshot(now_ns=now)
    assert snapshot.cells[20, 0, 0] == VoxelState.OCCUPIED
    assert snapshot.cells[30, 0, 0] == VoxelState.UNKNOWN
    assert snapshot.scene.metadata.support_count == 1
    prepared = CollisionSafety(config).prepare_scene(snapshot.scene, snapshot.scene.identities)
    assert int(prepared.status) == CollisionStatus.UNKNOWN_REQUIRED_SPACE
    assert not prepared.support_valid_mask.any()


@pytest.mark.parametrize("failure", ["missing_timestamp", "missing_history"])
def test_invalid_message_revokes_existing_scene_before_propagating_error(failure):
    portable_tests = str(ROOT / "portable_oscbf/tests")
    if portable_tests not in sys.path:
        sys.path.insert(0, portable_tests)
    from test_lidar_self_filter import analytic_filter, occupancy_at, raw_ray

    producer, safety = occupancy_at(10)
    module, history = analytic_filter()
    stamp = time.monotonic_ns()
    producer.update(raw_ray(stamp), now_ns=stamp)
    cloud = point_cloud2.create_cloud_xyz32(Header(frame_id="livox_frame"), [[1., 0., 0.]])
    error_type = ValueError
    match = "timestamp"
    if failure == "missing_history":
        cloud, history, error_type, match = cloud_message(), None, TypeError, "JointStateHistory"
    with pytest.raises(error_type, match=match):
        update_scene_from_pointcloud(
            cloud, mapping=clock_mapping(), scan_id=1, received_monotonic_ns=stamp, now_ns=stamp,
            max_range_m=8., filter_module=module, history=history, producer=producer,
        )
    snapshot = producer.snapshot(now_ns=stamp)
    assert snapshot.cells[10, 0, 0] == VoxelState.UNKNOWN
    assert snapshot.cells[45, 0, 0] == VoxelState.OCCUPIED
    assert int(safety.prepare_scene(snapshot.scene, snapshot.scene.identities).status) == CollisionStatus.INVALID_SCENE


@pytest.mark.parametrize("failure,error_type", [
    ("zero_extra_count", AssertionError),
    ("unknown_extra_datatype", KeyError),
    ("duplicate_extra_name", AssertionError),
])
def test_decoder_errors_revoke_existing_coverage_and_preserve_original_exception(failure, error_type):
    portable_tests = str(ROOT / "portable_oscbf/tests")
    if portable_tests not in sys.path:
        sys.path.insert(0, portable_tests)
    from test_lidar_self_filter import analytic_filter, occupancy_at, raw_ray

    producer, safety = occupancy_at(10)
    module, history = analytic_filter()
    stamp = time.monotonic_ns()
    producer.update(raw_ray(stamp), now_ns=stamp)
    before = producer.snapshot(now_ns=stamp)
    assert before.cells[10, 0, 0] == VoxelState.FREE
    assert int(safety.prepare_scene(before.scene, before.scene.identities).status) == CollisionStatus.OK
    cloud = cloud_message()
    if failure == "zero_extra_count":
        cloud.fields[3].count = 0
    elif failure == "unknown_extra_datatype":
        cloud.fields[3].datatype = 99
    else:
        cloud.fields[4].name = cloud.fields[3].name
    mapping = clock_mapping()
    with pytest.raises(error_type) as original:
        scan_from_pointcloud(cloud, mapping=mapping, scan_id=1,
                             received_monotonic_ns=stamp, max_range_m=8.)
    with pytest.raises(error_type) as propagated:
        update_scene_from_pointcloud(
            cloud, mapping=mapping, scan_id=1, received_monotonic_ns=stamp, now_ns=stamp,
            max_range_m=8., filter_module=module, history=history, producer=producer,
        )
    assert propagated.value.args == original.value.args
    assert propagated.traceback[-1].name == "dtype_from_fields"
    assert str(propagated.traceback[-1].path).endswith("sensor_msgs_py/point_cloud2.py")
    after = producer.snapshot(now_ns=stamp)
    assert after.cells[10, 0, 0] == VoxelState.UNKNOWN
    assert after.cells[45, 0, 0] == VoxelState.OCCUPIED
    assert not after.scene.metadata.required_space_covered
    assert producer.invalid_reason == "self_filter_source_error:" + error_type.__name__
    prepared = safety.prepare_scene(after.scene, after.scene.identities)
    assert int(prepared.status) == CollisionStatus.INVALID_SCENE
    assert not np.any(prepared.support_valid_mask)
