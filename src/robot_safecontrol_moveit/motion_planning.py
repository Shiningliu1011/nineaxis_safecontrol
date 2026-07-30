"""Collision-aware transition planning through MoveIt's planning pipeline.

No OMPL planner, collision checker, or path interpolator is implemented here.
``pymoveit2`` forwards the request to MoveIt's standard ROS 2
``/plan_kinematic_path`` service, where the configured OMPL pipeline and
PlanningScene perform those jobs.
"""

from __future__ import annotations

from dataclasses import dataclass
from time import monotonic
from typing import Iterable, Sequence

import rclpy
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from pymoveit2 import MoveIt2
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class PlanningError(RuntimeError):
    """MoveIt could not create or validate a transition trajectory."""


@dataclass(frozen=True)
class PlanningOptions:
    pipeline_id: str = "ompl"
    planner_id: str = "RRTConnectkConfigDefault"
    planning_time_s: float = 10.0
    planning_attempts: int = 5
    velocity_scale: float = 0.2
    acceleration_scale: float = 0.2
    goal_joint_tolerance: float = 1e-3


@dataclass(frozen=True)
class CollisionObjectSpec:
    """A primitive collision object expressed in a MoveIt planning frame.

    Sizes are full dimensions: ``box=(x, y, z)``, ``sphere=(radius,)`` and
    ``cylinder=(height, radius)``.
    """

    object_id: str
    shape: str
    position: tuple[float, float, float]
    dimensions: tuple[float, ...]
    frame_id: str
    quaternion_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)


class MotionPlanner:
    """Owns MoveIt planning parameters and PlanningScene obstacle updates."""

    def __init__(
        self,
        node,
        moveit: MoveIt2,
        joint_names: Sequence[str],
        options: PlanningOptions,
    ):
        self._node = node
        self._moveit = moveit
        self._joint_names = tuple(joint_names)
        self._options = options
        # ``pymoveit2.update_planning_scene()`` uses the synchronous
        # ``Client.call()`` API.  That API requires a separate executor thread
        # and otherwise blocks a single-threaded pipeline before it can spin.
        # Keep the MoveIt client for planning/execution, but query the standard
        # MoveIt PlanningScene service asynchronously so this module owns its
        # timeout and remains usable from this Python entry point.
        self._planning_scene_client = node.create_client(
            GetPlanningScene, "get_planning_scene"
        )
        self._validate_options()
        self._configure_moveit()

    def install_collision_objects(
        self,
        objects: Iterable[CollisionObjectSpec],
        sync_timeout_s: float = 5.0,
    ) -> None:
        """Publish configured obstacles and verify that MoveIt received them."""

        objects = tuple(objects)
        if not objects:
            return
        if sync_timeout_s <= 0.0:
            raise ValueError("sync_timeout_s must be positive")

        expected_ids: set[str] = set()
        for item in objects:
            self._validate_collision_object(item)
            expected_ids.add(item.object_id)

        deadline = monotonic() + sync_timeout_s
        next_publish_time = 0.0
        scene_request = GetPlanningScene.Request()
        scene_request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        while monotonic() < deadline:
            now = monotonic()
            # CollisionObject uses a normal ROS topic.  A publisher created
            # just before this call may not yet have discovered move_group's
            # subscription, so a one-shot publish can be lost on first
            # startup.  Re-publish the same ADD objects at a modest rate until
            # MoveIt confirms that all IDs are present.
            if now >= next_publish_time:
                for item in objects:
                    self._publish_collision_object(item)
                next_publish_time = now + 0.2

            remaining_s = deadline - monotonic()
            if self._planning_scene_client.wait_for_service(
                timeout_sec=min(0.2, remaining_s)
            ):
                future = self._planning_scene_client.call_async(scene_request)
                response_wait_s = deadline - monotonic()
                if response_wait_s <= 0.0:
                    break
                rclpy.spin_until_future_complete(
                    self._node,
                    future,
                    timeout_sec=min(0.2, response_wait_s),
                )
                if not future.done():
                    continue
                response = future.result()
                if response is None:
                    continue
                scene = response.scene
                received_ids = {
                    collision_object.id
                    for collision_object in scene.world.collision_objects
                }
                if expected_ids.issubset(received_ids):
                    self._node.get_logger().info(
                        f"MoveIt PlanningScene contains {len(expected_ids)} configured obstacle(s)."
                    )
                    return

        raise PlanningError(
            "Timed out waiting for MoveIt PlanningScene to receive collision objects: "
            f"{sorted(expected_ids)}"
        )

    def _publish_collision_object(self, item: CollisionObjectSpec) -> None:
        """Forward one reviewed primitive to MoveIt's PlanningScene topic."""

        if item.shape == "box":
            self._moveit.add_collision_box(
                id=item.object_id,
                size=item.dimensions,
                position=item.position,
                quat_xyzw=item.quaternion_xyzw,
                frame_id=item.frame_id,
            )
        elif item.shape == "sphere":
            self._moveit.add_collision_sphere(
                id=item.object_id,
                radius=item.dimensions[0],
                position=item.position,
                quat_xyzw=item.quaternion_xyzw,
                frame_id=item.frame_id,
            )
        elif item.shape == "cylinder":
            self._moveit.add_collision_cylinder(
                id=item.object_id,
                height=item.dimensions[0],
                radius=item.dimensions[1],
                position=item.position,
                quat_xyzw=item.quaternion_xyzw,
                frame_id=item.frame_id,
            )
        else:  # Covered by _validate_collision_object; retained for type safety.
            raise PlanningError(f"Unsupported collision shape: {item.shape}")

    def plan_transition(
        self,
        start_state: JointState,
        goal_state: JointState,
    ) -> JointTrajectory:
        """Ask MoveIt/OMPL for a collision-free start-to-first-IK trajectory."""

        start = self._ordered_state(start_state)
        goal = self._ordered_state(goal_state)
        trajectory = self._moveit.plan(
            joint_positions=list(goal.position),
            joint_names=list(self._joint_names),
            tolerance_joint_position=self._options.goal_joint_tolerance,
            start_joint_state=start,
        )
        if trajectory is None or not trajectory.points:
            raise PlanningError(
                "MoveIt returned no transition trajectory. Check the PlanningScene, "
                "joint state, collision objects, and planner configuration."
            )
        if tuple(trajectory.joint_names) != self._joint_names:
            raise PlanningError(
                "MoveIt returned a trajectory with unexpected joint order: "
                f"{list(trajectory.joint_names)}"
            )
        return trajectory

    def _configure_moveit(self) -> None:
        self._moveit.pipeline_id = self._options.pipeline_id
        self._moveit.planner_id = self._options.planner_id
        self._moveit.allowed_planning_time = self._options.planning_time_s
        self._moveit.num_planning_attempts = self._options.planning_attempts
        self._moveit.max_velocity = self._options.velocity_scale
        self._moveit.max_acceleration = self._options.acceleration_scale

    def _validate_options(self) -> None:
        if not self._options.pipeline_id or not self._options.planner_id:
            raise ValueError("pipeline_id and planner_id must be non-empty")
        if self._options.planning_time_s <= 0.0:
            raise ValueError("planning_time_s must be positive")
        if self._options.planning_attempts < 1:
            raise ValueError("planning_attempts must be at least one")
        for field_name, value in (
            ("velocity_scale", self._options.velocity_scale),
            ("acceleration_scale", self._options.acceleration_scale),
        ):
            if not 0.0 < value <= 1.0:
                raise ValueError(f"{field_name} must be in (0, 1]")
        if self._options.goal_joint_tolerance <= 0.0:
            raise ValueError("goal_joint_tolerance must be positive")

    def _ordered_state(self, state: JointState) -> JointState:
        positions = dict(zip(state.name, state.position))
        missing = [name for name in self._joint_names if name not in positions]
        if missing:
            raise PlanningError(f"Joint state is missing required joints: {missing}")

        result = JointState()
        result.header = state.header
        result.name = list(self._joint_names)
        result.position = [float(positions[name]) for name in self._joint_names]
        return result

    @staticmethod
    def _validate_collision_object(item: CollisionObjectSpec) -> None:
        expected_dimensions = {"box": 3, "sphere": 1, "cylinder": 2}
        if item.shape not in expected_dimensions:
            raise ValueError(
                f"Collision object {item.object_id!r} has unsupported shape {item.shape!r}"
            )
        if not item.object_id or not item.frame_id:
            raise ValueError("Collision objects require non-empty id and frame_id")
        if len(item.position) != 3 or len(item.quaternion_xyzw) != 4:
            raise ValueError(f"Collision object {item.object_id!r} has invalid pose")
        if len(item.dimensions) != expected_dimensions[item.shape] or any(
            value <= 0.0 for value in item.dimensions
        ):
            raise ValueError(
                f"Collision object {item.object_id!r} has invalid dimensions "
                f"for {item.shape}: {item.dimensions}"
            )
