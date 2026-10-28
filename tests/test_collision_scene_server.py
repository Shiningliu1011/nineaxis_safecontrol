import json
import os
import signal
import subprocess
import sys
import time
import uuid
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np
import pytest
import trimesh
import yaml
from sensor_msgs.msg import JointState, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "portable_oscbf/tests"))

from test_collision_safety import _config, _identities
from work.collision_safety import CollisionSafety, CollisionStatus
from work.fixed_environment import FixedEnvironmentModel
from work.lidar_occupancy import OccupancyGrid, VoxelState
from work.lidar_self_filter import ClockMapping, FilterCalibration, GeometryValidation, MeshRaySelfFilter, RobotRayGeometry, SelfFilterBounds
from work.shared_collision_scene import SharedSceneReader, SharedSceneWriter
from robot_safecontrol_moveit.collision_scene_pipeline import CollisionScenePipeline
from robot_safecontrol_moveit.robot_spec import DEFAULT_JOINT_NAMES


@pytest.fixture
def shared_path():
    path = Path("/dev/shm") / ("robot-scene-pipeline-" + uuid.uuid4().hex)
    yield path
    path.unlink(missing_ok=True)
    Path(str(path) + ".writer.lock").unlink(missing_ok=True)


def analytic_inputs():
    record = json.loads((ROOT / "tests/data/lidar_self_filter/analytic_scan_v1.json").read_text())
    mesh = trimesh.creation.box(extents=record["geometry"]["box_extents_m"])
    mesh.apply_translation(record["geometry"]["translation_m"])
    geometry = RobotRayGeometry.from_link_meshes([("base_link", mesh)], source_id=record["source_id"])
    geometry = geometry.with_validation(GeometryValidation(geometry.geometry_hash, 0., record["validation"], True))
    config = _config(
        voxel_size_mm=100., lidar_range_error_mm=0., lidar_incidence_error_mm=0., lidar_angular_error_rad=0.,
        extrinsic_translation_error_mm=0., extrinsic_rotation_error_rad=0.,
        joint_interpolation_prismatic_error_mm=0., joint_interpolation_revolute_error_rad=0.,
        untracked_speed_mm_s=0., untracked_acceleration_mm_s2=0.)
    safety = CollisionSafety(config)
    bounds = SelfFilterBounds.from_collision_policy(config.policy, joint_speed_bound_si=np.zeros(9),
                                                   maximum_gap_ns=1_000_000_000, history_evidence_id=record["source_id"])
    filter_module = MeshRaySelfFilter(geometry, FilterCalibration(np.eye(4), "livox_frame", "base_link", record["calibration"]["id"], True), bounds)
    anchor = time.monotonic_ns() - 1_100_000_000
    mapping = ClockMapping(record["raw_clock_id"], "analytic_map", 1_000_000_000, anchor,
                           1_000_000_000, 2_000_000_000, 0, 0., record["validation"], "analytic_run", "host_monotonic")
    required = np.zeros((50, 1, 1), dtype=bool)
    required[0, 0, 0] = True
    grid = OccupancyGrid(np.array([0., -.05, -.05]), (50, 1, 1), 100., required)
    feedback = []
    for sample in record["joint_history"]:
        message = JointState()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(sample["stamp_ns"], 1_000_000_000)
        message.name = list(DEFAULT_JOINT_NAMES)
        message.position = sample["q_si"]
        feedback.append(message)
    fields = [PointField(name=name, offset=offset, datatype=datatype, count=1) for name, offset, datatype in (
        ("x", 0, PointField.FLOAT32), ("y", 4, PointField.FLOAT32), ("z", 8, PointField.FLOAT32), ("timestamp", 12, PointField.FLOAT64))]
    cloud = point_cloud2.create_cloud(Header(frame_id="livox_frame"), fields,
                                      [(*row["point_m"], float(row["stamp_ns"])) for row in record["returns"]])
    return safety, filter_module, mapping, grid, feedback, cloud


def test_real_pipeline_owns_feedback_filter_occupancy_fixed_model_and_publish(shared_path):
    safety, filter_module, mapping, grid, feedback, cloud = analytic_inputs()
    with SharedSceneWriter(shared_path, safety) as writer:
        pipeline = CollisionScenePipeline(writer, filter_module, grid, mapping, feedback_source="simulation",
                                          zero_calibration_id="analytic_zero", history_capacity=8, max_range_m=8.)
        fixed_mesh = trimesh.creation.box(extents=[.02, .02, .02])
        fixed_mesh.apply_translation([4.5, 0., 0.])
        fixed = FixedEnvironmentModel.from_mesh(fixed_mesh, policy=safety.config.policy, model_revision=1,
                                               source_hash=b"f" * 32, validation_evidence="analytic 20 mm cube", error_mm=0.)
        pipeline.set_fixed_environment(fixed)
        for sample in feedback:
            pipeline.add_joint_state(sample)
        result = pipeline.ingest_pointcloud(cloud)
        assert result.filter_result.classification == ("environment", "self", "occluded", "environment")
        assert result.snapshot.cells[0, 0, 0] == VoxelState.FREE
        assert result.snapshot.cells[20, 0, 0] == VoxelState.OCCUPIED
        assert result.snapshot.cells[30, 0, 0] == VoxelState.UNKNOWN
        assert result.revision.status == CollisionStatus.OK
        with SharedSceneReader(shared_path, safety, result.snapshot.scene.identities,
                               slot_timeout_ns=10_000_000_000) as reader:
            prepared = reader.refresh().prepared_scene
            assert prepared is not None
            assert prepared.fixed_environment_model.model_hash == fixed.model_hash
            assert prepared.support_time_model is not None
            assert int(np.sum(prepared.support_mask)) == 2
            malformed = point_cloud2.create_cloud_xyz32(Header(frame_id="livox_frame"), [[2., 0., 0.]])
            with pytest.raises(ValueError, match="timestamp"):
                pipeline.ingest_pointcloud(malformed)
            assert reader.refresh().status == CollisionStatus.INVALID_SCENE
            assert reader.prepared_scene is None


def test_source_manifest_requires_all_provenance_before_creating_writer(tmp_path, shared_path):
    from jsonschema import ValidationError
    from robot_safecontrol_moveit.collision_scene_sources import load_scene_sources

    manifest = tmp_path / "scene.json"
    manifest.write_text(json.dumps({"schema_version": 1}), encoding="utf-8")
    with pytest.raises(ValidationError):
        load_scene_sources(manifest)
    assert not shared_path.exists()


def test_pipeline_preserves_occupied_when_verified_tracking_is_interrupted(shared_path):
    from work.dynamic_support import MotionEvidence, MotionTrack, TrackingFrame, observation_hash

    safety, filter_module, mapping, grid, feedback, cloud = analytic_inputs()
    with SharedSceneWriter(shared_path, safety) as writer:
        pipeline = CollisionScenePipeline(writer, filter_module, grid, mapping, feedback_source="simulation",
                                          zero_calibration_id="analytic_zero", history_capacity=8, max_range_m=8.)
        for sample in feedback:
            pipeline.add_joint_state(sample)
        initial = pipeline.ingest_pointcloud(cloud)
        observations = pipeline.observations()
        assert len(observations) == 1
        now = time.monotonic_ns()
        track = MotionTrack(
            track_id=9, support_ids=(20,), observation_hashes=(observation_hash(observations),),
            velocity_m_s=np.zeros(3), position_error_mm=1., velocity_error_mm_s=0., acceleration_bound_m_s2=0.,
            timestamp_error_ns=0, updated_stamp_ns=now, valid_until_ns=now + 1_000_000_000, valid=True,
            evidence=MotionEvidence(b"s" * 32, b"v" * 32, "analytic stationary box", safety.config.policy.artifact.device,
                                    safety.config.policy.artifact.scenario))
        frame = TrackingFrame(writer.epoch, grid.required_space_hash, safety.config.collision_policy_hash, "monotonic", (track,))
        pipeline.update_tracking(frame)
        tracked = pipeline.publish()
        with SharedSceneReader(shared_path, safety, initial.snapshot.scene.identities, slot_timeout_ns=10_000_000_000) as reader:
            prepared = reader.refresh().prepared_scene
            assert prepared is not None
            assert int(prepared.support_track_status[0]) == 1
            assert int(prepared.support_track_ids[0]) == 9
            assert int(prepared.tracks.ids[0]) == 9
            assert int(prepared.support_time_model.source_track_ids[0, 0]) == 9
            pipeline.update_tracking(replace(frame, tracks=(replace(track, valid=False),)))
            interrupted = pipeline.publish()
            restored = reader.refresh().prepared_scene
            assert restored is not None
            assert interrupted.revision.revision == tracked.revision.revision + 1
            assert int(restored.support_ids[0]) == 20
            assert bool(restored.support_mask[0])
            assert int(restored.support_track_status[0]) == 0
            assert int(restored.support_track_ids[0]) == -1
            assert not np.any(restored.tracks.active_mask)
            assert interrupted.snapshot.cells[20, 0, 0] == VoxelState.OCCUPIED


@pytest.mark.parametrize("manifest", ["", "/definitely-missing-collision-scene.json"])
def test_failed_node_constructor_releases_the_actual_ros_node(manifest):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from robot_safecontrol_moveit.collision_scene_server import CollisionSceneServer

    context = Context()
    rclpy.init(context=context, domain_id=185)
    observer = rclpy.create_node("scene_constructor_observer", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(observer)
    try:
        with pytest.raises((ValueError, FileNotFoundError)):
            CollisionSceneServer(context=context, parameter_overrides=[
                Parameter("source_manifest", value=manifest),
                Parameter("portable_oscbf_root", value=str(ROOT / "portable_oscbf"))])
        deadline = time.monotonic() + 2.
        while "collision_scene_server" in observer.get_node_names() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        assert "collision_scene_server" not in observer.get_node_names()
    finally:
        observer.destroy_node()
        executor.shutdown()
        context.shutdown()


def write_analytic_manifest(directory):
    from test_calibration_record import _calibrated_entry
    from test_ellipsoid_dcol import _geometry, _module

    safety, filter_module, mapping, grid, feedback, cloud = analytic_inputs()
    geometry = _geometry(["base_link", "Link1"], [[0., 0., 0.], [0., 0., 0.]], [[.1]*3, [.1]*3], capacity=4)
    parameters = {name: record["value"] for name, record in safety.config.policy.artifact.parameters.items()}
    parameters.update(occupied_max_age_ns=20_000_000_000, free_max_age_ns=20_000_000_000)
    module = _module(geometry, **parameters)
    (directory / "parameters.json").write_text(json.dumps(module.config.policy.artifact.to_document()), encoding="utf-8")
    (directory / "geometry.json").write_text(json.dumps(geometry), encoding="utf-8")
    box = trimesh.creation.box(extents=[1., 1., 1.])
    box.apply_translation([3., 0., 0.])
    mesh_path = directory / "analytic_robot.ply"
    box.export(mesh_path)
    ray = RobotRayGeometry.from_link_meshes([("base_link", trimesh.load_mesh(mesh_path, process=False))], source_id="analytic_box_file_v1")
    calibration = {"schema_version": 1, "lidar": _calibrated_entry(
        frame_from="livox_frame", frame_to="base_link", matrix=np.eye(4).ravel().tolist(),
        operator="analytic-interface-validation", sensor_serial="analytic-closed-box",
        residual={"translation_mm": 0., "rotation_deg": 0.}, error_estimate={"translation_mm": 0., "rotation_deg": 0.})}
    calibration_path = directory / "calibration.yaml"
    calibration_path.write_text(yaml.safe_dump(calibration), encoding="utf-8")
    np.save(directory / "required.npy", grid.required_mask, allow_pickle=False)
    manifest = {
        "schema_version": 1, "parameter_artifact": "parameters.json", "device": module.config.policy.artifact.device,
        "scenario": module.config.policy.artifact.scenario, "geometry_artifact": "geometry.json",
        "kernel_version": module.config.kernel_version.hex(),
        "ray_geometry": {"link_meshes": [{"link": "base_link", "path": "analytic_robot.ply"}],
                         "source_id": "analytic_box_file_v1", "validation": {
                             "geometry_hash": ray.geometry_hash, "surface_error_mm": 0.,
                             "evidence_id": "analytic closed box x=[2.5,3.5] m", "model_complete": True}},
        "calibration_record": "calibration.yaml", "lidar_clock_mapping": asdict(mapping), "joint_clock_mapping": asdict(mapping),
        "grid": {"origin_m": grid.origin_m.tolist(), "shape": list(grid.shape), "required_mask": "required.npy"},
        "history": {"feedback_source": "simulation", "zero_calibration_id": "analytic_zero", "capacity": 8,
                    "joint_speed_bound_si": [0.]*9, "maximum_gap_ns": 1_000_000_000, "evidence_id": "analytic stationary feedback"},
        "max_range_m": 8., "fixed_environment": None,
    }
    manifest_path = directory / "scene.json"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    return manifest_path, calibration_path, feedback, cloud


@pytest.mark.parametrize("change", [
    "calibration_same_path", "lidar_clock", "joint_clock", "run", "bounds", "ray_geometry", "ray_validation",
])
def test_reader_requires_the_actual_loaded_source_identity(tmp_path, shared_path, change):
    from robot_safecontrol_moveit.collision_scene_sources import load_scene_sources

    manifest, calibration, feedback, cloud = write_analytic_manifest(tmp_path)
    first_sources = load_scene_sources(manifest)
    document = json.loads(manifest.read_bytes())
    if change == "calibration_same_path":
        record = yaml.safe_load(calibration.read_text())
        record["lidar"]["operator"] = "independent-analytic-calibration"
        calibration.write_text(yaml.safe_dump(record), encoding="utf-8")
    elif change in ("lidar_clock", "joint_clock"):
        field = "lidar_clock_mapping" if change == "lidar_clock" else "joint_clock_mapping"
        document[field]["mapping_id"] += "_independent"
        manifest.write_text(json.dumps(document), encoding="utf-8")
    elif change == "run":
        for field in ("lidar_clock_mapping", "joint_clock_mapping"):
            document[field]["run_id"] += "_independent"
        manifest.write_text(json.dumps(document), encoding="utf-8")
    elif change == "ray_geometry":
        ray = document["ray_geometry"]
        ray["source_id"] += "_independent"
        geometry = RobotRayGeometry.from_link_meshes(
            [("base_link", trimesh.load_mesh(tmp_path / "analytic_robot.ply", process=False))], source_id=ray["source_id"])
        ray["validation"]["geometry_hash"] = geometry.geometry_hash
        manifest.write_text(json.dumps(document), encoding="utf-8")
    elif change == "ray_validation":
        document["ray_geometry"]["validation"]["evidence_id"] += "; independently validated"
        manifest.write_text(json.dumps(document), encoding="utf-8")
    else:
        document["history"]["maximum_gap_ns"] = 2_000_000_000
        manifest.write_text(json.dumps(document), encoding="utf-8")
    other_sources = load_scene_sources(manifest)
    if change == "calibration_same_path":
        assert first_sources.manifest_hash == other_sources.manifest_hash
    pipeline = first_sources.create_pipeline(shared_path)
    try:
        for sample in feedback:
            pipeline.add_joint_state(sample)
        pipeline.ingest_pointcloud(cloud)
        with first_sources.create_reader(shared_path, slot_timeout_ns=15_000_000_000) as matching:
            assert matching.refresh().status == CollisionStatus.OK
        with other_sources.create_reader(shared_path, slot_timeout_ns=15_000_000_000) as different:
            result = different.refresh()
            assert result.status == CollisionStatus.INVALID_SCENE
            assert result.prepared_scene is None
            assert different.device_transfer_count == 0
    finally:
        pipeline.writer.close()


def test_source_identity_uses_loaded_contents_across_manifest_paths_and_formatting(tmp_path, shared_path):
    from robot_safecontrol_moveit.collision_scene_sources import load_scene_sources

    manifest, calibration, feedback, cloud = write_analytic_manifest(tmp_path)
    first_sources = load_scene_sources(manifest)
    equivalent = tmp_path / "equivalent-scene.json"
    equivalent.write_text(json.dumps(json.loads(manifest.read_bytes()), indent=2, sort_keys=True), encoding="utf-8")
    same_sources = load_scene_sources(equivalent)
    assert same_sources.manifest_hash != first_sources.manifest_hash
    assert same_sources.source_identity == first_sources.source_identity
    pipeline = first_sources.create_pipeline(shared_path)
    try:
        for sample in feedback:
            pipeline.add_joint_state(sample)
        pipeline.ingest_pointcloud(cloud)
        with same_sources.create_reader(shared_path, slot_timeout_ns=15_000_000_000) as reader:
            assert reader.refresh().status == CollisionStatus.OK
            assert reader.result.revision.source_identity == first_sources.source_identity
            expected = reader.expected
        with SharedSceneReader(shared_path, same_sources.safety, expected, slot_timeout_ns=15_000_000_000) as unbound:
            assert unbound.refresh().status == CollisionStatus.INVALID_SCENE
            assert unbound.prepared_scene is None
            assert unbound.device_transfer_count == 0
    finally:
        pipeline.writer.close()


def test_real_bridge_and_server_processes_publish_one_shared_scene(tmp_path, shared_path, runtime_prefix=None):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.serialization import serialize_message
    from std_msgs.msg import String, UInt8MultiArray
    from work.shared_collision_scene import SceneRevision
    from robot_safecontrol_moveit.collision_scene_sources import load_scene_sources
    from robot_safecontrol_moveit.ros_conventions import COLLISION_SCENE_REVISION_TOPIC, JOINT_STATE_TOPIC, TIMED_LIDAR_TOPIC, state_stream_qos
    from robot_safecontrol_moveit.timed_lidar_transport import decode_timed_lidar

    manifest, calibration, feedback, cloud = write_analytic_manifest(tmp_path)
    sources = load_scene_sources(manifest)
    domain = 184
    environment = {**os.environ, "ROS_DOMAIN_ID": str(domain), "ROS_LOCALHOST_ONLY": "1",
                   "PYTHONPATH": str(ROOT / "src") + ":" + str(ROOT / "portable_oscbf") + ":" + os.environ.get("PYTHONPATH", "")}
    portable_root = ROOT / "portable_oscbf"
    if runtime_prefix is not None:
        prefix = Path(runtime_prefix).resolve(strict=True)
        portable_root = prefix / "share/robot_safecontrol_moveit/portable_oscbf"
        remaining = [entry for entry in os.environ.get("PYTHONPATH", "").split(":")
                     if entry and entry not in (str(ROOT / "src"), str(ROOT / "portable_oscbf"))]
        environment["PYTHONPATH"] = ":".join([str(prefix / "lib/python3.10/site-packages"), str(portable_root), *remaining])
        environment["AMENT_PREFIX_PATH"] = str(prefix) + ":" + os.environ.get("AMENT_PREFIX_PATH", "")
    processes, logs = [], []
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    client = rclpy.create_node("scene_process_acceptance_" + uuid.uuid4().hex[:8], context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(client)
    reader = None
    try:
        for name, arguments in (
            ("perception_bridge", ["-p", "scene_server_mode:=true", "-p", 'source_topic:=""', "-p", "source_topic_lidar:=/analytic_lidar_raw",
                                   "-p", "input_frame_lidar:=livox_frame", "-p", "source_time_domain_lidar:=" + sources.lidar_mapping.source_clock_id,
                                   "-p", "calibration_record_path:=" + str(calibration)]),
            ("collision_scene_server", ["-p", "source_manifest:=" + str(manifest), "-p", "shared_path:=" + str(shared_path),
                                         "-p", "health_period_s:=10.0"]),
        ):
            output = (tmp_path / (name + ".log")).open("w+", encoding="utf-8")
            logs.append(output)
            command = ([sys.executable, "-m", "robot_safecontrol_moveit." + name] if runtime_prefix is None
                       else [str(prefix / "lib/robot_safecontrol_moveit" / name)])
            processes.append(subprocess.Popen([*command, "--ros-args", *arguments],
                                               env=environment, cwd=ROOT, stdout=output, stderr=subprocess.STDOUT))
        from sensor_msgs.msg import PointCloud2

        raw = client.create_publisher(PointCloud2, "/analytic_lidar_raw", qos_profile_sensor_data)
        joints = client.create_publisher(JointState, JOINT_STATE_TOPIC, state_stream_qos())
        notices, envelopes = [], []
        client.create_subscription(String, COLLISION_SCENE_REVISION_TOPIC,
                                   lambda message: notices.append(SceneRevision.from_dict(json.loads(message.data))),
                                   QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        client.create_subscription(UInt8MultiArray, TIMED_LIDAR_TOPIC, envelopes.append, qos_profile_sensor_data)

        def until(condition, seconds=20.):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                if condition():
                    return
                assert all(process.poll() is None for process in processes), "场景进程提前退出"
                executor.spin_once(timeout_sec=.03)
            raise AssertionError("真实 ROS 场景进程等待超时")

        until(lambda: raw.get_subscription_count() == 1 and joints.get_subscription_count() == 1 and notices)
        joints.publish(feedback[0])
        time.sleep(.1)
        joints.publish(feedback[1])
        time.sleep(.1)
        raw.publish(cloud)
        until(lambda: any(notice.status == CollisionStatus.OK for notice in notices) and envelopes)
        restored, instance = decode_timed_lidar(envelopes[0], expected_clock_id=sources.lidar_mapping.source_clock_id,
                                               expected_calibration_id=sources.filter_module.calibration.calibration_id,
                                               expected_frame_id="livox_frame")
        assert serialize_message(restored) == serialize_message(cloud)
        assert instance.startswith("perception_bridge#")
        reader = sources.create_reader(shared_path, slot_timeout_ns=15_000_000_000)
        result = reader.refresh()
        assert result.status == CollisionStatus.OK
        np.testing.assert_allclose(np.asarray(result.prepared_scene.support_points_m)[0], [2.05, 0., 0.])
        assert reader.refresh().prepared_scene is result.prepared_scene
        assert reader.device_transfer_count == 1
        assert not client.get_publishers_info_by_topic("/perception/tracks")
        assert not client.get_publishers_info_by_topic("/perception/cloud_world")
        malformed = point_cloud2.create_cloud_xyz32(Header(frame_id="livox_frame"), [[2., 0., 0.]])
        raw.publish(malformed)
        until(lambda: notices[-1].status == CollisionStatus.INVALID_SCENE)
        assert reader.refresh().status == CollisionStatus.INVALID_SCENE
        assert reader.prepared_scene is None
        assert all(notice.source_identity == sources.source_identity for notice in notices)
        assert all(len(json.dumps(notice.to_dict())) < 600 for notice in notices)
    finally:
        if reader is not None:
            reader.close()
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(10)
        for output in logs:
            output.flush()
            output.seek(0)
            print(output.read())
            output.close()
        client.destroy_node()
        executor.shutdown()
        context.shutdown()
