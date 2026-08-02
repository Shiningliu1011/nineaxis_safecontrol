"""Behaviour tests for collision-object transport and registry state."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path


_SRC = Path(__file__).resolve().parents[1] / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from geometry_msgs.msg import Pose  # noqa: E402
from moveit_msgs.msg import CollisionObject  # noqa: E402
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy  # noqa: E402
from shape_msgs.msg import SolidPrimitive  # noqa: E402

from robot_safecontrol_moveit.obstacle_publisher import (  # noqa: E402
    STATIC_COLLISION_OBJECT_TOPIC,
    _collision_object_msg,
    static_collision_qos,
)
from robot_safecontrol_moveit.transition_planning_server import (  # noqa: E402
    DYNAMIC_COLLISION_OBJECT_TOPIC,
    ObstacleRegistry,
    dynamic_collision_qos,
)


def _collision_message(
    object_id: str,
    operation,
    *,
    primitive_type: int | None = None,
    dimensions: tuple[float, ...] = (),
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
    frame_id: str = "base_link",
) -> CollisionObject:
    """Build a real ROS collision-object message for registry input."""
    message = CollisionObject()
    message.id = object_id
    message.operation = operation
    message.header.frame_id = frame_id
    message.pose = Pose()
    (
        message.pose.position.x,
        message.pose.position.y,
        message.pose.position.z,
    ) = position
    message.pose.orientation.w = 1.0
    if primitive_type is not None:
        primitive = SolidPrimitive()
        primitive.type = primitive_type
        primitive.dimensions = list(dimensions)
        message.primitives = [primitive]
    return message


class TestObstacleRegistry(unittest.TestCase):
    """The registry must preserve the state represented by ROS messages."""

    def setUp(self) -> None:
        self.registry = ObstacleRegistry()

    def test_move_without_primitive_keeps_registered_geometry(self) -> None:
        self.registry.apply(
            _collision_message(
                "dynamic_box",
                CollisionObject.ADD,
                primitive_type=SolidPrimitive.BOX,
                dimensions=(0.2, 0.3, 0.4),
                position=(0.1, 0.2, 0.3),
            )
        )
        initial_revision = self.registry.revision

        self.registry.apply(
            _collision_message(
                "dynamic_box",
                CollisionObject.MOVE,
                position=(1.0, -0.5, 0.75),
            )
        )

        entries = self.registry.active_objects()
        self.assertEqual(len(entries), 1)
        entry = entries[0]
        self.assertEqual(entry.object_id, "dynamic_box")
        self.assertEqual(entry.shape, "box")
        self.assertEqual(entry.dimensions, (0.2, 0.3, 0.4))
        self.assertEqual(entry.position, (1.0, -0.5, 0.75))
        self.assertGreater(entry.revision, initial_revision)

    def test_shape_change_replaces_registered_geometry(self) -> None:
        self.registry.apply(
            _collision_message(
                "reconfigured",
                CollisionObject.ADD,
                primitive_type=SolidPrimitive.SPHERE,
                dimensions=(0.15,),
            )
        )
        self.registry.apply(
            _collision_message(
                "reconfigured",
                CollisionObject.ADD,
                primitive_type=SolidPrimitive.CYLINDER,
                dimensions=(0.8, 0.1),
                position=(0.4, 0.5, 0.6),
            )
        )

        entries = self.registry.active_objects()
        self.assertEqual(len(entries), 1)
        self.assertEqual(entries[0].shape, "cylinder")
        self.assertEqual(entries[0].dimensions, (0.8, 0.1))
        self.assertEqual(entries[0].position, (0.4, 0.5, 0.6))

    def test_readd_reactivates_and_clears_pending_remove(self) -> None:
        self.registry.apply(
            _collision_message(
                "temporary",
                CollisionObject.ADD,
                primitive_type=SolidPrimitive.BOX,
                dimensions=(0.1, 0.1, 0.1),
            )
        )
        self.registry.apply(
            _collision_message("temporary", CollisionObject.REMOVE)
        )

        self.assertEqual(self.registry.active_objects(), [])
        self.assertEqual(
            self.registry.pending_removals,
            frozenset({"temporary"}),
        )

        self.registry.apply(
            _collision_message(
                "temporary",
                CollisionObject.ADD,
                primitive_type=SolidPrimitive.SPHERE,
                dimensions=(0.25,),
            )
        )

        entries = self.registry.active_objects()
        self.assertEqual([entry.object_id for entry in entries], ["temporary"])
        self.assertEqual(entries[0].shape, "sphere")
        self.assertEqual(self.registry.pending_removals, frozenset())


class TestCollisionObjectPublisherContract(unittest.TestCase):
    """Static publisher output must be consumable as real MoveIt messages."""

    def test_message_helper_populates_each_supported_shape(self) -> None:
        cases = (
            ("box", (0.2, 0.3, 0.4), SolidPrimitive.BOX),
            ("sphere", (0.25,), SolidPrimitive.SPHERE),
            ("cylinder", (0.8, 0.1), SolidPrimitive.CYLINDER),
        )
        for shape, dimensions, primitive_type in cases:
            with self.subTest(shape=shape):
                message = _collision_object_msg(
                    object_id=f"static_{shape}",
                    shape=shape,
                    dimensions=dimensions,
                    position=(0.1, 0.2, 0.3),
                    quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                    frame_id="base_link",
                )

                self.assertIsInstance(message, CollisionObject)
                self.assertEqual(message.operation, CollisionObject.ADD)
                self.assertEqual(message.id, f"static_{shape}")
                self.assertEqual(message.header.frame_id, "base_link")
                self.assertEqual(message.pose.position.z, 0.3)
                self.assertEqual(message.pose.orientation.w, 1.0)
                self.assertEqual(len(message.primitives), 1)
                self.assertEqual(message.primitives[0].type, primitive_type)
                self.assertEqual(
                    tuple(message.primitives[0].dimensions), dimensions
                )

    def test_static_and_dynamic_qos_have_different_durability(self) -> None:
        static_qos = static_collision_qos(7)
        dynamic_qos = dynamic_collision_qos(7)

        self.assertEqual(
            STATIC_COLLISION_OBJECT_TOPIC,
            "/static_collision_object",
        )
        self.assertEqual(DYNAMIC_COLLISION_OBJECT_TOPIC, "/collision_object")
        self.assertEqual(static_qos.depth, 7)
        self.assertEqual(
            static_qos.reliability,
            ReliabilityPolicy.RELIABLE,
        )
        self.assertEqual(
            static_qos.durability,
            DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.assertEqual(dynamic_qos.depth, 7)
        self.assertEqual(
            dynamic_qos.reliability,
            ReliabilityPolicy.RELIABLE,
        )
        self.assertEqual(dynamic_qos.durability, DurabilityPolicy.VOLATILE)


if __name__ == "__main__":
    unittest.main()
