import numpy as np
import pytest
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from robot_safecontrol_moveit.timed_lidar_transport import decode_timed_lidar, encode_timed_lidar


def timed_cloud():
    fields = [PointField(name=name, offset=offset, datatype=datatype, count=1) for name, offset, datatype in (
        ("x", 0, PointField.FLOAT32), ("y", 4, PointField.FLOAT32), ("z", 8, PointField.FLOAT32),
        ("tag", 12, PointField.UINT8), ("line", 13, PointField.UINT8), ("timestamp", 14, PointField.FLOAT64))]
    return point_cloud2.create_cloud(Header(frame_id="livox_frame"), fields,
                                     [(2., 0., 0., 1, 3, 1_234_567_891.), (2.5, 0., 0., 2, 4, 1_234_567_999.)])


def test_original_unaligned_pointcloud_is_bound_to_clock_calibration_and_instance():
    cloud = timed_cloud()
    envelope = encode_timed_lidar(cloud, source_clock_id="analytic_raw_ns", calibration_id="record:lidar",
                                  source_instance="bridge-instance-1", frame_id="livox_frame")
    restored, instance = decode_timed_lidar(envelope, expected_clock_id="analytic_raw_ns",
                                           expected_calibration_id="record:lidar", expected_frame_id="livox_frame")
    assert instance == "bridge-instance-1"
    assert serialize_message(restored) == serialize_message(cloud)
    assert restored.fields[-1].offset == 14
    assert bytes(restored.data) == bytes(cloud.data)
    with pytest.raises(ValueError, match="source identity"):
        decode_timed_lidar(envelope, expected_clock_id="different_clock",
                          expected_calibration_id="record:lidar", expected_frame_id="livox_frame")


def test_invalid_raw_cloud_transmits_explicit_rejection():
    cloud = point_cloud2.create_cloud_xyz32(Header(frame_id="livox_frame"), np.array([[2., 0., 0.]]))
    envelope = encode_timed_lidar(cloud, source_clock_id="analytic_raw_ns", calibration_id="record:lidar",
                                  source_instance="bridge-instance-1", frame_id="livox_frame")
    with pytest.raises(ValueError, match="timestamp"):
        decode_timed_lidar(envelope, expected_clock_id="analytic_raw_ns",
                          expected_calibration_id="record:lidar", expected_frame_id="livox_frame")
