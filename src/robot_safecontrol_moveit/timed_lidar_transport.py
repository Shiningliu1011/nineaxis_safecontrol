from __future__ import annotations

from array import array

import msgpack
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import UInt8MultiArray

from robot_safecontrol_moveit.lidar_self_filter_adapter import pointcloud_samples


def encode_timed_lidar(message: PointCloud2, *, source_clock_id: str, calibration_id: str,
                       source_instance: str, frame_id: str) -> UInt8MultiArray:
    if any(not isinstance(value, str) or not value.strip() for value in (
            source_clock_id, calibration_id, source_instance, frame_id)):
        raise ValueError("timed LiDAR 需要完整来源、frame、时钟与标定身份")
    valid, reason = True, "ok"
    try:
        if message.header.frame_id != frame_id:
            raise ValueError("LiDAR frame 与已配置来源不一致")
        pointcloud_samples(message)
    except Exception as error:
        valid, reason = False, type(error).__name__ + ":" + str(error)
    payload = msgpack.packb({
        "schema_version": 1, "source_clock_id": source_clock_id, "calibration_id": calibration_id,
        "source_instance": source_instance, "valid": valid, "reason": reason,
        "cloud": serialize_message(message),
    }, use_bin_type=True, strict_types=True)
    return UInt8MultiArray(data=array("B", payload))


def decode_timed_lidar(message: UInt8MultiArray, *, expected_clock_id: str,
                       expected_calibration_id: str, expected_frame_id: str) -> tuple[PointCloud2, str]:
    if not isinstance(message, UInt8MultiArray):
        raise TypeError("timed LiDAR 需要 UInt8MultiArray envelope")
    document = msgpack.unpackb(bytes(message.data), raw=False, strict_map_key=True)
    if (not isinstance(document, dict) or set(document) != {
            "schema_version", "source_clock_id", "calibration_id", "source_instance", "valid", "reason", "cloud"}
            or type(document["schema_version"]) is not int or document["schema_version"] != 1
            or type(document["valid"]) is not bool or not isinstance(document["cloud"], bytes)
            or any(not isinstance(document[name], str) or not document[name].strip()
                   for name in ("source_clock_id", "calibration_id", "source_instance", "reason"))):
        raise ValueError("timed LiDAR envelope 字段无效")
    if (document["source_clock_id"] != expected_clock_id or document["calibration_id"] != expected_calibration_id):
        raise ValueError("timed LiDAR source identity 与场景 manifest 不一致")
    if not document["valid"]:
        raise ValueError("timed LiDAR source rejected: " + document["reason"])
    cloud = deserialize_message(document["cloud"], PointCloud2)
    if cloud.header.frame_id != expected_frame_id:
        raise ValueError("timed LiDAR source frame 与场景 manifest 不一致")
    pointcloud_samples(cloud)
    return cloud, document["source_instance"]
