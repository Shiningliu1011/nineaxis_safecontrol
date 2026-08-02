"""Verify the live dynamic-obstacle path against MoveIt's PlanningScene.

Run this after the final launch is up::

    ros2 run robot_safecontrol_moveit dynamic_obstacle_probe

The probe intentionally publishes on ``/collision_object`` rather than
calling ``ApplyPlanningScene``.  It therefore verifies the same volatile ROS
transport used by an external perception / obstacle-tracking system.
"""

from __future__ import annotations

import argparse
from time import monotonic
from typing import Callable

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents, RobotState
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive


_DYNAMIC_TOPIC = "/collision_object"
_SCENE_SERVICE = "/get_planning_scene"
_VALIDITY_SERVICE = "/check_state_validity"
_JOINT_NAMES = tuple(f"J{index}" for index in range(1, 10))
_PROBE_ID = "dynamic_obstacle_probe_box"
_BLOCKER_ID = "dynamic_obstacle_probe_blocker"


def _dynamic_qos() -> QoSProfile:
    """Match MoveIt's normal dynamic CollisionObject QoS contract."""
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _box(
    object_id: str,
    operation: int,
    position: tuple[float, float, float],
    *,
    dimensions: tuple[float, float, float] | None = None,
) -> CollisionObject:
    """Create one valid CollisionObject transport message.

    A Move operation deliberately carries no primitive geometry: that is the
    MoveIt message contract for retaining an existing object's shape.
    """
    message = CollisionObject()
    message.header.frame_id = "base_link"
    message.id = object_id
    message.operation = operation
    message.pose = Pose()
    (
        message.pose.position.x,
        message.pose.position.y,
        message.pose.position.z,
    ) = position
    message.pose.orientation.w = 1.0
    if dimensions is not None:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(dimensions)
        message.primitives = [primitive]
    return message


class DynamicObstacleProbe(Node):
    """A bounded, self-cleaning live PlanningScene validation node."""

    def __init__(self, timeout_s: float) -> None:
        super().__init__("dynamic_obstacle_probe")
        if timeout_s <= 0.0:
            raise ValueError("timeout must be positive")
        self._timeout_s = float(timeout_s)
        self._publisher = self.create_publisher(
            CollisionObject, _DYNAMIC_TOPIC, _dynamic_qos()
        )
        self._scene_client = self.create_client(GetPlanningScene, _SCENE_SERVICE)
        self._validity_client = self.create_client(
            GetStateValidity, _VALIDITY_SERVICE
        )
        self._active_ids: set[str] = set()

    def run(self) -> None:
        """Run ADD -> MOVE -> REMOVE and a FCL collision-effect check."""
        self._wait_for_service(self._scene_client, _SCENE_SERVICE)
        self._wait_for_service(self._validity_client, _VALIDITY_SERVICE)

        try:
            if not self._state_is_valid():
                raise RuntimeError(
                    "DYNAMIC_PROBE_BASELINE_INVALID: zero joint state is already "
                    "invalid; clear the existing scene before probing"
                )

            add = _box(
                _PROBE_ID,
                CollisionObject.ADD,
                (0.75, 0.20, 1.20),
                dimensions=(0.18, 0.18, 0.18),
            )
            self._active_ids.add(_PROBE_ID)
            added = self._publish_until(
                add,
                lambda objects: _PROBE_ID in objects,
                "ADD",
            )
            self._assert_box_dimensions(added[_PROBE_ID], (0.18, 0.18, 0.18))

            moved_position = (10.0, 10.0, 10.0)
            moved = self._publish_until(
                _box(_PROBE_ID, CollisionObject.MOVE, moved_position),
                lambda objects: self._at_position(
                    objects.get(_PROBE_ID), moved_position
                ),
                "MOVE",
            )
            # A correct MOVE carries no geometry but must preserve the ADDed box.
            self._assert_box_dimensions(moved[_PROBE_ID], (0.18, 0.18, 0.18))

            self._remove_and_confirm(_PROBE_ID)

            # This large object intentionally overlaps the zero-position arm.
            # It proves FCL collision checking changed, not merely topic delivery.
            blocker = _box(
                _BLOCKER_ID,
                CollisionObject.ADD,
                (0.0, 0.0, 0.0),
                dimensions=(1.0, 1.0, 1.0),
            )
            self._active_ids.add(_BLOCKER_ID)
            self._publish_until(
                blocker,
                lambda objects: _BLOCKER_ID in objects,
                "blocker ADD",
            )
            if self._state_is_valid():
                raise RuntimeError(
                    "DYNAMIC_PROBE_BLOCKER_INEFFECTIVE: the live blocker did not "
                    "make the zero joint state collide"
                )

            self._remove_and_confirm(_BLOCKER_ID)
            if not self._state_is_valid():
                raise RuntimeError(
                    "DYNAMIC_PROBE_RECOVERY_FAILED: zero joint state stayed invalid "
                    "after removing the live blocker"
                )

            self.get_logger().info(
                "DYNAMIC_OBSTACLE_PROBE_PASS: ADD/MOVE/REMOVE reached "
                "PlanningScene and FCL changed valid -> invalid -> valid"
            )
        finally:
            self._cleanup_active_objects()

    def _wait_for_service(self, client, service_name: str) -> None:
        if not client.wait_for_service(timeout_sec=self._timeout_s):
            raise RuntimeError(
                f"DYNAMIC_PROBE_SERVICE_UNAVAILABLE: {service_name}"
            )

    def _call(self, client, request):
        future = client.call_async(request)
        deadline = monotonic() + self._timeout_s
        while not future.done() and monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise RuntimeError("DYNAMIC_PROBE_SERVICE_TIMEOUT")
        response = future.result()
        if response is None:
            raise RuntimeError("DYNAMIC_PROBE_SERVICE_NO_RESPONSE")
        return response

    def _scene_objects(self) -> dict[str, CollisionObject]:
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        response = self._call(self._scene_client, request)
        return {
            object_.id: object_
            for object_ in response.scene.world.collision_objects
        }

    def _publish_until(
        self,
        message: CollisionObject,
        predicate: Callable[[dict[str, CollisionObject]], bool],
        action: str,
    ) -> dict[str, CollisionObject]:
        """Republish a volatile message until MoveIt confirms its effect."""
        deadline = monotonic() + self._timeout_s
        latest: dict[str, CollisionObject] = {}
        while monotonic() < deadline:
            self._publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)
            latest = self._scene_objects()
            if predicate(latest):
                return latest
        raise RuntimeError(
            f"DYNAMIC_PROBE_{action}_NOT_CONFIRMED: "
            f"PlanningScene IDs={sorted(latest)}"
        )

    def _remove_and_confirm(self, object_id: str) -> None:
        self._publish_until(
            _box(object_id, CollisionObject.REMOVE, (0.0, 0.0, 0.0)),
            lambda objects: object_id not in objects,
            "REMOVE",
        )
        self._active_ids.discard(object_id)

    def _state_is_valid(self) -> bool:
        request = GetStateValidity.Request()
        request.robot_state = RobotState()
        request.robot_state.joint_state = JointState()
        request.robot_state.joint_state.name = list(_JOINT_NAMES)
        request.robot_state.joint_state.position = [0.0] * len(_JOINT_NAMES)
        request.group_name = "arm"
        return bool(self._call(self._validity_client, request).valid)

    def _cleanup_active_objects(self) -> None:
        for object_id in tuple(self._active_ids):
            try:
                self._remove_and_confirm(object_id)
            except Exception as error:  # best effort during error handling
                self.get_logger().warning(
                    f"Could not clean dynamic probe object {object_id!r}: {error}"
                )

    @staticmethod
    def _at_position(
        object_: CollisionObject | None,
        expected: tuple[float, float, float],
    ) -> bool:
        if object_ is None:
            return False
        actual = object_.pose.position
        return all(
            abs(value - wanted) < 1e-6
            for value, wanted in zip((actual.x, actual.y, actual.z), expected)
        )

    @staticmethod
    def _assert_box_dimensions(
        object_: CollisionObject,
        expected: tuple[float, float, float],
    ) -> None:
        if not object_.primitives:
            raise RuntimeError("DYNAMIC_PROBE_GEOMETRY_LOST: no primitive remains")
        actual = tuple(float(value) for value in object_.primitives[0].dimensions)
        if actual != expected:
            raise RuntimeError(
                "DYNAMIC_PROBE_GEOMETRY_LOST: "
                f"expected dimensions={expected}, got={actual}"
            )


def main(args: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--timeout",
        type=float,
        default=12.0,
        help="per-action timeout in seconds (default: 12)",
    )
    parsed = parser.parse_args(args)

    rclpy.init(args=None)
    node: DynamicObstacleProbe | None = None
    try:
        node = DynamicObstacleProbe(parsed.timeout)
        node.run()
        return 0
    except Exception as error:
        if node is not None:
            node.get_logger().error(f"DYNAMIC_OBSTACLE_PROBE_FAIL: {error}")
        else:
            print(f"DYNAMIC_OBSTACLE_PROBE_FAIL: {error}")
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
