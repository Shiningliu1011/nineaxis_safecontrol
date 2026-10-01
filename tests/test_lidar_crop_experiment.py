import json
from time import monotonic

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker

from robot_safecontrol_moveit.lidar_crop_experiment import LidarCropExperiment
from robot_safecontrol_moveit.point_cloud_crop import cloud_xyz
from robot_safecontrol_moveit.ros_conventions import state_stream_qos
from test_point_cloud_crop import make_cloud


def test_real_ros_selection_resize_reset_and_stale_input():
    context = Context()
    rclpy.init(context=context, domain_id=96)
    executor = SingleThreadedExecutor(context=context)
    crop = LidarCropExperiment(
        context=context, namespace="crop_test",
        parameter_overrides=[Parameter("input_topic", value="raw")],
    )
    probe = Node("crop_probe", namespace="crop_test", context=context)
    executor.add_node(crop)
    executor.add_node(probe)
    qos = state_stream_qos()
    raw_pub = probe.create_publisher(PointCloud2, "raw", qos)
    center_pub = probe.create_publisher(PointStamped, "lidar_crop/center", 10)
    cropped, preview, markers, statuses = [], [], [], []
    probe.create_subscription(PointCloud2, "lidar_crop/points", cropped.append, qos)
    probe.create_subscription(PointCloud2, "lidar_crop/preview", preview.append, qos)
    probe.create_subscription(Marker, "lidar_crop/region", markers.append, 10)
    probe.create_subscription(String, "lidar_crop/status", lambda m: statuses.append(json.loads(m.data)), 10)

    def until(predicate, timeout=5.0):
        deadline = monotonic() + timeout
        while not predicate() and monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), "ROS 消息未在期限内到达"

    def deliver(cloud):
        preview.clear()
        raw_pub.publish(cloud)
        until(lambda: bool(preview))

    cloud = make_cloud([[0.5, 0, 0], [5, 0, 0], [5.9, 0, 0], [6.1, 0, 0]])
    try:
        until(lambda: raw_pub.get_subscription_count() == 1
              and center_pub.get_subscription_count() == 1
              and crop.count_subscribers("lidar_crop/preview") == 1
              and crop.count_subscribers("lidar_crop/points") == 1)
        deliver(cloud)
        assert preview[-1] == cloud
        assert cropped == []
        center_pub.publish(PointStamped(header=Header(frame_id="wrong_frame"), point=Point(x=5.0)))
        until(lambda: len(statuses) > 0)
        assert not crop.get_parameter("enabled").value
        center_pub.publish(PointStamped(header=Header(frame_id="livox_frame"), point=Point(x=5.0)))
        until(lambda: crop.get_parameter("enabled").value)
        deliver(cloud)
        until(lambda: len(cropped) == 1 and any(m.action == Marker.ADD for m in markers))
        assert cropped[-1].data == cloud.data[26:78]
        assert len([m for m in markers if m.action == Marker.ADD][-1].points) == 24
        result = crop.set_parameters_atomically([Parameter("roi_size", value=[0.2, 1.0, 1.0])])
        assert result.successful
        deliver(cloud)
        until(lambda: len(cropped) == 2)
        np.testing.assert_array_equal(cloud_xyz(cropped[-1]), [[5, 0, 0]])
        result = crop.set_parameters_atomically([Parameter("roi_size", value=[-1.0, 1.0, 1.0])])
        assert not result.successful
        assert crop.get_parameter("roi_size").value == [0.2, 1.0, 1.0]
        statuses.clear()
        until(lambda: any(s["state"] == "NO_RECENT_INPUT" for s in statuses))
        until(lambda: preview[-1].width == 0)
        assert preview[-1].header == cloud.header
        assert preview[-1].fields == cloud.fields
        assert len(cropped) == 2
        # 恢复输入时仅发布当前帧，历史位置不进入输出。
        moved = make_cloud([[5.05, 0.25, 0], [0.5, 0, 0]])
        moved.header.stamp.sec += 1
        deliver(moved)
        until(lambda: len(cropped) == 3)
        assert cropped[-1].data == moved.data[:26]
        assert preview[-1] == cropped[-1]
        assert cropped[-1].header == moved.header
        assert crop.set_parameters_atomically([Parameter("enabled", value=False)]).successful
        deliver(cloud)
        assert preview[-1] == cloud
        assert len(cropped) == 3
    finally:
        executor.shutdown()
        probe.destroy_node()
        crop.destroy_node()
        context.shutdown()


def test_topic_alias_and_invalid_startup_region_are_rejected():
    context = Context()
    rclpy.init(context=context, domain_id=96)
    try:
        with pytest.raises(ValueError, match="topic"):
            LidarCropExperiment(context=context, parameter_overrides=[
                Parameter("input_topic", value="/lidar_crop/points")])
        with pytest.raises(ValueError, match="size"):
            LidarCropExperiment(context=context, parameter_overrides=[
                Parameter("roi_size", value=[0.0, 2.0, 2.0])])
        with pytest.raises(ValueError, match="topic"):
            LidarCropExperiment(context=context, cli_args=[
                "--ros-args", "-r", "lidar_crop/preview:=/lidar_crop/points"])
    finally:
        context.shutdown()
