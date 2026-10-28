from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String, UInt8MultiArray

from robot_safecontrol_moveit.oscbf_trajectory import bootstrap_portable, package_portable_root
from robot_safecontrol_moveit.ros_conventions import (
    COLLISION_SCENE_REVISION_TOPIC, JOINT_STATE_TOPIC, TIMED_LIDAR_TOPIC, state_stream_qos,
)


class CollisionSceneServer(Node):
    def __init__(self, **kwargs):
        super().__init__("collision_scene_server", **kwargs)
        self.pipeline = None
        try:
            self._configure()
        except BaseException:
            self.destroy_node()
            raise

    def _configure(self):
        self._source_instance = None
        descriptor = ParameterDescriptor(read_only=True)
        values = {}
        for name, default in {
            "source_manifest": "", "shared_path": "/dev/shm/robot_safecontrol_collision_scene",
            "portable_oscbf_root": str(package_portable_root()), "joint_state_topic": JOINT_STATE_TOPIC,
            "timed_lidar_topic": TIMED_LIDAR_TOPIC, "revision_topic": COLLISION_SCENE_REVISION_TOPIC,
            "health_period_s": .1,
        }.items():
            values[name] = self.declare_parameter(name, default, descriptor).value
        if (not str(values["source_manifest"]).strip() or not str(values["shared_path"]).strip()
                or not np.isfinite(values["health_period_s"]) or values["health_period_s"] <= 0):
            raise ValueError("场景服务需要 source_manifest、shared_path 与正数 health_period_s")
        topics = [self.resolve_topic_name(str(values[name])) for name in (
            "joint_state_topic", "timed_lidar_topic", "revision_topic")]
        if any(not str(values[name]).strip() for name in ("joint_state_topic", "timed_lidar_topic", "revision_topic")) or len(set(topics)) != 3:
            raise ValueError("场景输入与 revision topic 必须有效且互异")
        bootstrap_portable(Path(str(values["portable_oscbf_root"])))
        from robot_safecontrol_moveit.collision_scene_sources import load_scene_sources

        sources = load_scene_sources(str(values["source_manifest"]))
        try:
            self.pipeline = sources.create_pipeline(str(values["shared_path"]))
            self.revision_publisher = self.create_publisher(String, str(values["revision_topic"]), QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.joint_subscription = self.create_subscription(JointState, str(values["joint_state_topic"]), self._on_joint, state_stream_qos())
            self.scan_subscription = self.create_subscription(UInt8MultiArray, str(values["timed_lidar_topic"]), self._on_scan, qos_profile_sensor_data)
            self.health_timer = self.create_timer(float(values["health_period_s"]), self._on_health)
            self._publish_revision()
            self.get_logger().info(
                f"COLLISION_SCENE_SERVER_READY manifest={sources.manifest_path} sha256={sources.manifest_hash} "
                f"shared_path={values['shared_path']} joint_topic={topics[0]} lidar_topic={topics[1]} revision_topic={topics[2]}")
        except BaseException:
            if self.pipeline is not None:
                self.pipeline.writer.close()
            raise

    def _publish_revision(self):
        self.revision_publisher.publish(String(data=json.dumps(self.pipeline.writer.revision.to_dict(),
                                                               sort_keys=True, separators=(",", ":"), allow_nan=False)))

    def _on_joint(self, message):
        try:
            self.pipeline.add_joint_state(message)
        except Exception as error:
            self.get_logger().warning(f"COLLISION_SCENE_FEEDBACK_REJECTED {type(error).__name__}: {error}")
            self._publish_revision()

    def _on_scan(self, message):
        from robot_safecontrol_moveit.timed_lidar_transport import decode_timed_lidar
        from work.shared_collision_scene import ScenePublicationError

        try:
            cloud, instance = decode_timed_lidar(
                message, expected_clock_id=self.pipeline.mapping.source_clock_id,
                expected_calibration_id=self.pipeline.filter_module.calibration.calibration_id,
                expected_frame_id=self.pipeline.filter_module.calibration.frame_from)
            if self._source_instance is not None and self._source_instance != instance:
                raise ValueError("timed LiDAR source instance changed; new scene epoch required")
            self._source_instance = instance
            self.pipeline.ingest_pointcloud(cloud)
        except ScenePublicationError as error:
            self.get_logger().warning(f"COLLISION_SCENE_SCAN_REJECTED {type(error).__name__}: {error}")
        except Exception as error:
            self.pipeline.invalidate("timed_lidar_source_invalid")
            self.get_logger().warning(f"COLLISION_SCENE_SCAN_REJECTED {type(error).__name__}: {error}")
        self._publish_revision()

    def _on_health(self):
        try:
            self.pipeline.publish()
        except Exception as error:
            self.get_logger().warning(f"COLLISION_SCENE_HEALTH_REJECTED {type(error).__name__}: {error}", throttle_duration_sec=5.)
        self._publish_revision()

    def destroy_node(self):
        if self.pipeline is not None:
            self.pipeline.writer.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CollisionSceneServer()
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
