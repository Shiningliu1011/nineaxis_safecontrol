from __future__ import annotations

import itertools
import json
from time import monotonic, perf_counter

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from robot_safecontrol_moveit.point_cloud_crop import CropRegion, crop_cloud
from robot_safecontrol_moveit.ros_conventions import state_stream_qos


class LidarCropExperiment(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("lidar_crop_experiment", **kwargs)
        try:
            self._configure()
        except BaseException:
            self.destroy_node()
            raise

    def _configure(self) -> None:
        self.declare_parameter("input_topic", "/livox/lidar", ParameterDescriptor(read_only=True))
        self.declare_parameter("frame_id", "livox_frame", ParameterDescriptor(read_only=True))
        self.declare_parameter("roi_center", [0.0, 0.0, 0.0])
        self.declare_parameter("roi_size", [2.0, 2.0, 2.0])
        self.declare_parameter("enabled", False)
        self._frame_id = self.get_parameter("frame_id").value
        if not self._frame_id:
            raise ValueError("frame_id 不能为空")
        self._region()
        self.add_on_set_parameters_callback(self._validate_parameters)
        input_topic = self.resolve_topic_name(self.get_parameter("input_topic").value)
        cloud_topics = [input_topic, self.resolve_topic_name("lidar_crop/points"),
                        self.resolve_topic_name("lidar_crop/preview")]
        if len(set(cloud_topics)) != 3:
            raise ValueError("输入与输出 topic 必须分别命名")
        qos = state_stream_qos()
        # 点云预览只处理最新消息，不排队显示历史帧。
        qos.depth = 1
        self._crop_pub = self.create_publisher(PointCloud2, "lidar_crop/points", qos)
        self._preview_pub = self.create_publisher(PointCloud2, "lidar_crop/preview", qos)
        self._marker_pub = self.create_publisher(Marker, "lidar_crop/region", 10)
        self._status_pub = self.create_publisher(String, "lidar_crop/status", 10)
        self._last_input = None
        self._last_preview = None
        self._preview_cleared = False
        self._latest = {}
        # 默认互斥 callback group 让选区更新和逐帧处理串行执行。
        self.create_subscription(PointCloud2, input_topic, self._on_cloud, qos)
        self.create_subscription(PointStamped, "lidar_crop/center", self._on_center, 10)
        self.create_timer(0.1, self._publish_status)
        self.get_logger().info("RViz 选区实验：使用 Publish Point 选择中心，roi_size 为完整尺寸（米）")

    def _region(self, changes: dict | None = None) -> CropRegion:
        changes = changes or {}
        values = {
            name: changes.get(name, self.get_parameter(name).value)
            for name in ("roi_center", "roi_size")
        }
        return CropRegion(values["roi_center"], values["roi_size"])

    def _validate_parameters(self, parameters: list[Parameter]) -> SetParametersResult:
        try:
            self._region({p.name: p.value for p in parameters})
        except (TypeError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def _on_center(self, message: PointStamped) -> None:
        if message.header.frame_id != self._frame_id:
            self.get_logger().warning("选区坐标系不匹配，必须在 " + self._frame_id + " 中点击")
            return
        result = self.set_parameters_atomically([
            Parameter("roi_center", value=[message.point.x, message.point.y, message.point.z]),
            Parameter("enabled", value=True),
        ])
        if not result.successful:
            self.get_logger().warning("选区被拒绝：" + result.reason)
            return
        self.get_logger().info(f"裁剪中心={self._region().center}，尺寸={self._region().size} m")

    def _on_cloud(self, message: PointCloud2) -> None:
        if message.header.frame_id != self._frame_id:
            raise ValueError(f"输入 frame_id={message.header.frame_id}，预期 {self._frame_id}")
        region = self._region()
        enabled = self.get_parameter("enabled").value
        start = perf_counter()
        if enabled:
            cropped = crop_cloud(message, region)
            self._crop_pub.publish(cropped)
            preview = cropped
            output_count = cropped.width
        else:
            # preview 在等待点击时提供完整画面，points 仅发布已选区域。
            preview = message
            output_count = None
        self._preview_pub.publish(preview)
        self._last_preview = preview
        self._preview_cleared = False
        self._last_input = monotonic()
        self._latest = {
            "input_points": message.width * message.height,
            "output_points": output_count,
            "processing_ms": (perf_counter() - start) * 1000,
            "stamp_sec": message.header.stamp.sec,
            "stamp_nanosec": message.header.stamp.nanosec,
            "roi_center": list(region.center),
            "roi_size": list(region.size),
            "enabled": enabled,
        }
        self._publish_marker(region, enabled)

    def _publish_marker(self, region: CropRegion, enabled: bool) -> None:
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "crop_region"
        marker.id = 0
        marker.action = Marker.ADD if enabled else Marker.DELETE
        marker.type = Marker.LINE_LIST
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.015
        marker.color.g = 1.0
        marker.color.a = 1.0
        marker.lifetime.nanosec = 500_000_000
        lower, upper = region.bounds
        for corner in itertools.product((0, 1), repeat=3):
            for axis in range(3):
                if corner[axis] == 0:
                    end = list(corner)
                    end[axis] = 1
                    for vertex in (corner, end):
                        xyz = [float((lower, upper)[vertex[i]][i]) for i in range(3)]
                        marker.points.append(Point(x=xyz[0], y=xyz[1], z=xyz[2]))
        self._marker_pub.publish(marker)

    def _publish_status(self) -> None:
        age = None if self._last_input is None else monotonic() - self._last_input
        if age is None or age > 0.5:
            state = "NO_RECENT_INPUT"
            self._clear_preview()
        elif self.get_parameter("enabled").value:
            state = "CROPPING"
        else:
            state = "WAITING_SELECTION"
        self._status_pub.publish(String(data=json.dumps({
            "state": state,
            "input_age_s": age,
            "frame_id": self._frame_id,
            "last_frame": self._latest,
        }, allow_nan=False)))

    def _clear_preview(self) -> None:
        if self._last_preview is None or self._preview_cleared:
            return
        previous = self._last_preview
        empty = PointCloud2(
            header=previous.header,
            height=1,
            width=0,
            fields=previous.fields,
            is_bigendian=previous.is_bigendian,
            point_step=previous.point_step,
            row_step=0,
            is_dense=False,
        )
        # 空消息仅清除显示；points 停止输出，status 明确表示没有新数据。
        self._preview_pub.publish(empty)
        self._publish_marker(self._region(), False)
        self._preview_cleared = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = LidarCropExperiment()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
