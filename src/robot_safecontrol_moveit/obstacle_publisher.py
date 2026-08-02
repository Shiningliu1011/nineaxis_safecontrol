"""Publish static obstacles from YAML as CollisionObject messages.

This persistent node reads ``obstacles.yaml`` once at startup and publishes
each entry as a ``moveit_msgs/CollisionObject`` ADD message on
``/static_collision_object`` with ``RELIABLE + TRANSIENT_LOCAL`` QoS.
Late-joining subscribers (the MuJoCo Viewer and transition-planning server)
receive the objects automatically. The planning server then installs those
registered static objects through MoveIt's ``/apply_planning_scene`` service.

Dynamic obstacles deliberately use the separate ``/collision_object`` topic
with volatile durability, so a static publisher can never negotiate an
incompatible durability policy with MoveIt's dynamic PlanningScene monitor.

Usage::

    ros2 run robot_safecontrol_moveit static_obstacle_publisher \
      --ros-args -p obstacles_file:=/path/to/obstacles.yaml
"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive

from .plan_transition import load_collision_objects


STATIC_COLLISION_OBJECT_TOPIC = "/static_collision_object"


def static_collision_qos(depth: int) -> QoSProfile:
    """Return the one QoS contract used by every static-object endpoint."""
    return QoSProfile(
        depth=max(1, int(depth)),
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


def _collision_object_msg(
    object_id: str,
    shape: str,
    dimensions: Sequence[float],
    position: tuple[float, float, float],
    quaternion_xyzw: tuple[float, float, float, float],
    frame_id: str,
) -> CollisionObject:
    """Build a CollisionObject ADD message from a YAML spec."""
    msg = CollisionObject()
    msg.id = object_id
    msg.operation = CollisionObject.ADD
    msg.header.frame_id = frame_id

    pose = Pose()
    pose.position.x = float(position[0])
    pose.position.y = float(position[1])
    pose.position.z = float(position[2])
    pose.orientation.x = float(quaternion_xyzw[0])
    pose.orientation.y = float(quaternion_xyzw[1])
    pose.orientation.z = float(quaternion_xyzw[2])
    pose.orientation.w = float(quaternion_xyzw[3])
    msg.pose = pose

    primitive = SolidPrimitive()
    if shape == "box":
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(d) for d in dimensions]
    elif shape == "sphere":
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [float(dimensions[0])]
    elif shape == "cylinder":
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [float(d) for d in dimensions]
    else:
        raise ValueError(f"Unsupported shape: {shape}")
    msg.primitives = [primitive]
    return msg


class StaticObstaclePublisher(Node):
    """Persistent node that publishes static obstacles once at startup."""

    def __init__(self) -> None:
        super().__init__("static_obstacle_publisher")
        self.declare_parameter("obstacles_file", "")
        self.declare_parameter(
            "static_collision_object_topic", STATIC_COLLISION_OBJECT_TOPIC
        )
        self.declare_parameter("default_frame_id", "base_link")

    def publish_all(self) -> None:
        obstacles_file = str(self.get_parameter("obstacles_file").value).strip()
        if not obstacles_file:
            from ament_index_python.packages import get_package_share_directory

            obstacles_file = str(
                Path(get_package_share_directory("robot_safecontrol_moveit"))
                / "config"
                / "obstacles.yaml"
            )
        path = Path(obstacles_file).expanduser()
        if not path.is_file():
            self.get_logger().error(f"Obstacles file not found: {path}")
            return

        default_frame = str(self.get_parameter("default_frame_id").value)
        specs = load_collision_objects(path, default_frame)
        topic = str(self.get_parameter("static_collision_object_topic").value)

        # Retained samples are available to late joiners; do not sleep to race
        # discovery. The server verifies its static registry before planning.
        self._publisher = self.create_publisher(
            CollisionObject, topic, static_collision_qos(max(100, len(specs)))
        )

        for spec in specs:
            msg = _collision_object_msg(
                object_id=spec.object_id,
                shape=spec.shape,
                dimensions=spec.dimensions,
                position=spec.position,
                quaternion_xyzw=spec.quaternion_xyzw,
                frame_id=spec.frame_id,
            )
            msg.header.stamp = self.get_clock().now().to_msg()
            self._publisher.publish(msg)
            self.get_logger().info(
                f"Published static obstacle {spec.object_id} ({spec.shape}) "
                f"on {topic}"
            )

        self.get_logger().info(
            f"All {len(specs)} static obstacle(s) published on {topic} "
            f"(TRANSIENT_LOCAL). Node stays alive."
        )


def main(args: Sequence[str] | None = None) -> int:
    rclpy.init(args=args)
    node = StaticObstaclePublisher()
    try:
        node.publish_all()
        rclpy.spin(node)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        node.get_logger().error(f"Static obstacle publisher failed: {exc}")
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
