"""Real MoveIt runtime tests for AEB-RRT* and live dynamic obstacles.

This is intentionally an integration test, not a registry-only unit test.  It
starts the production final launch headlessly, asks ``move_group`` which
planner it actually loaded, then proves a volatile ``/collision_object`` ADD,
MOVE and REMOVE changes both the PlanningScene *and* collision validity.
"""

from __future__ import annotations

import os
from pathlib import Path
import time
import unittest

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from geometry_msgs.msg import Pose
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import launch_testing.markers
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningSceneComponents,
    RobotState,
)
from moveit_msgs.srv import (
    GetMotionPlan,
    GetPlanningScene,
    GetStateValidity,
    QueryPlannerInterfaces,
)
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive


_PROJECT = Path(__file__).resolve().parents[1]
_PACKAGE_NAME = "robot_safecontrol_moveit"
_LAUNCH_FILENAME = "mujoco_transition_final.launch.py"
_AEB_PLANNER_ID = "AEBRRTstarFaithfulConfigDefault"
_STARTUP_TIMEOUT_S = 45.0
_ACTION_TIMEOUT_S = 12.0
_POLL_PERIOD_S = 0.05
_TEST_DOMAIN_ID = 142 + (os.getpid() % 20)
_JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9")
_STATIC_OBJECT_IDS = frozenset({"obs_box1", "obs_sphere1", "obs_cyl1", "obs_box2"})
# A collision-free configuration whose distance from the zero start is larger
# than one AEB extension.  It exercises the bidirectional tree connection
# rather than the trivial direct-start-to-goal case.
_AEB_ENDPOINT_GOAL = (
    0.423262,
    -0.427864,
    -0.778006,
    -0.890106,
    -2.250302,
    0.702432,
    -1.102613,
    -0.964117,
    -1.179514,
)


def _final_launch_path() -> Path:
    """Prefer the installed launch so package installation is tested too."""
    try:
        installed = (
            Path(get_package_share_directory(_PACKAGE_NAME))
            / "launch"
            / _LAUNCH_FILENAME
        )
        if installed.is_file():
            return installed
    except PackageNotFoundError:
        pass

    source = _PROJECT / "launch" / _LAUNCH_FILENAME
    if source.is_file():
        return source
    raise FileNotFoundError(f"Could not locate {_LAUNCH_FILENAME}")


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description() -> LaunchDescription:
    # Keep this real graph separate from interactive sessions and other launch
    # tests.  The launch inherits this value before child processes start.
    os.environ["ROS_DOMAIN_ID"] = str(_TEST_DOMAIN_ID)
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(_final_launch_path())),
                launch_arguments={"start_viewer": "false"}.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


def _dynamic_qos() -> QoSProfile:
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _collision_box(
    object_id: str,
    operation: int,
    position: tuple[float, float, float],
    *,
    dimensions: tuple[float, float, float] | None = None,
) -> CollisionObject:
    """Build a real dynamic CollisionObject transport message."""
    message = CollisionObject()
    message.id = object_id
    message.operation = operation
    message.header.frame_id = "base_link"
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


class TestAEBAndDynamicObstacleRuntime(unittest.TestCase):
    """Test the production scene monitor and planner plugin over ROS 2."""

    def setUp(self) -> None:
        self._context = Context()
        rclpy.init(context=self._context, domain_id=_TEST_DOMAIN_ID)
        self._node = rclpy.create_node(
            "aeb_dynamic_obstacle_runtime_probe", context=self._context
        )
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._clients = []
        self._probe_active = False
        self._publisher = self._node.create_publisher(
            CollisionObject, "/collision_object", _dynamic_qos()
        )

    def tearDown(self) -> None:
        # Leave the test scene clean even if an assertion in ADD/MOVE fails.
        if self._context.ok() and self._probe_active:
            self._publisher.publish(
                _collision_box(
                    "aeb_dynamic_obstacle_runtime_probe",
                    CollisionObject.REMOVE,
                    (0.0, 0.0, 0.0),
                )
            )
            self._spin_for(0.25)
        for client in self._clients:
            self._node.destroy_client(client)
        self._node.destroy_publisher(self._publisher)
        self._executor.remove_node(self._node)
        self._executor.shutdown()
        self._node.destroy_node()
        if self._context.ok():
            rclpy.shutdown(context=self._context)

    def _spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self._executor.spin_once(timeout_sec=_POLL_PERIOD_S)

    def _client(self, service_type, service_name: str):
        client = self._node.create_client(service_type, service_name)
        self._clients.append(client)
        self.assertTrue(
            client.wait_for_service(timeout_sec=_STARTUP_TIMEOUT_S),
            f"{service_name} did not become available",
        )
        return client

    def _call(self, client, request, *, timeout_s: float = _ACTION_TIMEOUT_S):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            self._executor.spin_once(timeout_sec=_POLL_PERIOD_S)
        self.assertTrue(future.done(), "Timed out waiting for MoveIt service result")
        result = future.result()
        self.assertIsNotNone(result, "MoveIt service returned no response")
        return result

    def _scene_objects(self, scene_client) -> dict[str, CollisionObject]:
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        response = self._call(scene_client, request)
        return {
            object_.id: object_
            for object_ in response.scene.world.collision_objects
        }

    def _wait_for_static_scene(self, scene_client) -> None:
        """Do not plan against a scene snapshot that predates static objects."""
        deadline = time.monotonic() + _ACTION_TIMEOUT_S
        observed: set[str] = set()
        while time.monotonic() < deadline:
            observed = set(self._scene_objects(scene_client))
            if _STATIC_OBJECT_IDS.issubset(observed):
                return
            self._spin_for(_POLL_PERIOD_S)
        self.fail(
            "Static collision objects did not reach MoveIt's PlanningScene "
            f"before endpoint planning; observed IDs={sorted(observed)}"
        )

    def _wait_for_scene(
        self,
        scene_client,
        message: CollisionObject,
        predicate,
        description: str,
    ) -> dict[str, CollisionObject]:
        deadline = time.monotonic() + _ACTION_TIMEOUT_S
        last_scene: dict[str, CollisionObject] = {}
        while time.monotonic() < deadline:
            self._publisher.publish(message)
            self._spin_for(0.08)
            last_scene = self._scene_objects(scene_client)
            if predicate(last_scene):
                return last_scene
        self.fail(
            f"Dynamic collision object did not {description} in the real "
            f"MoveIt PlanningScene; observed IDs={sorted(last_scene)}"
        )

    @staticmethod
    def _state_validity_request(
        positions: tuple[float, ...] = (0.0,) * len(_JOINT_NAMES),
    ) -> GetStateValidity.Request:
        request = GetStateValidity.Request()
        request.robot_state = RobotState()
        request.robot_state.joint_state = JointState()
        request.robot_state.joint_state.name = list(_JOINT_NAMES)
        request.robot_state.joint_state.position = list(positions)
        request.group_name = "arm"
        return request

    @classmethod
    def _joint_goal_request(cls) -> GetMotionPlan.Request:
        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = "arm"
        motion_request.pipeline_id = "ompl"
        motion_request.planner_id = _AEB_PLANNER_ID
        motion_request.num_planning_attempts = 1
        motion_request.allowed_planning_time = 8.0
        motion_request.max_velocity_scaling_factor = 0.2
        motion_request.max_acceleration_scaling_factor = 0.2
        motion_request.start_state = RobotState()
        motion_request.start_state.joint_state = JointState()
        motion_request.start_state.joint_state.name = list(_JOINT_NAMES)
        motion_request.start_state.joint_state.position = [0.0] * len(_JOINT_NAMES)

        goal_constraints = Constraints()
        for name, position in zip(_JOINT_NAMES, _AEB_ENDPOINT_GOAL):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = 0.001
            constraint.tolerance_below = 0.001
            constraint.weight = 1.0
            goal_constraints.joint_constraints.append(constraint)
        motion_request.goal_constraints = [goal_constraints]
        return request

    def test_move_group_really_advertises_aeb_faithful(self) -> None:
        client = self._client(QueryPlannerInterfaces, "/query_planner_interface")
        response = self._call(client, QueryPlannerInterfaces.Request())

        aeb_interfaces = [
            interface
            for interface in response.planner_interfaces
            if interface.pipeline_id == "ompl"
            and any(_AEB_PLANNER_ID in planner_id for planner_id in interface.planner_ids)
        ]
        self.assertTrue(
            aeb_interfaces,
            "move_group did not advertise AEBRRTstarFaithfulConfigDefault; "
            f"advertised={response.planner_interfaces}",
        )

    def test_aeb_plan_reaches_the_requested_joint_goal(self) -> None:
        """The installed AEB plugin must return an exact, complete path."""
        scene_client = self._client(GetPlanningScene, "/get_planning_scene")
        self._wait_for_static_scene(scene_client)
        validity_client = self._client(GetStateValidity, "/check_state_validity")
        self.assertTrue(
            self._call(
                validity_client,
                self._state_validity_request(_AEB_ENDPOINT_GOAL),
            ).valid,
            "The endpoint regression target must be collision-free",
        )

        plan_client = self._client(GetMotionPlan, "/plan_kinematic_path")
        # AEB is randomized, so verify several independent requests.  A
        # single successful query would not detect an invalid path introduced
        # by a goal-tree rewire or a post-planning shortcut.
        for attempt in range(5):
            response = self._call(
                plan_client,
                self._joint_goal_request(),
                timeout_s=12.0,
            ).motion_plan_response
            self.assertEqual(
                response.error_code.val,
                MoveItErrorCodes.SUCCESS,
                f"AEB planner returned error code {response.error_code.val} "
                f"on repeat {attempt + 1}",
            )

            trajectory = response.trajectory.joint_trajectory
            self.assertEqual(tuple(trajectory.joint_names), _JOINT_NAMES)
            self.assertTrue(trajectory.points, "AEB returned an empty trajectory")
            final_positions = trajectory.points[-1].positions
            errors = [
                abs(actual - expected)
                for actual, expected in zip(final_positions, _AEB_ENDPOINT_GOAL)
            ]
            self.assertLessEqual(
                max(errors),
                0.001,
                "AEB path stops short of its requested goal on repeat "
                f"{attempt + 1}; errors={errors}",
            )

    def test_dynamic_add_move_remove_changes_scene_and_collision_validity(self) -> None:
        scene_client = self._client(GetPlanningScene, "/get_planning_scene")
        validity_client = self._client(GetStateValidity, "/check_state_validity")
        probe_id = "aeb_dynamic_obstacle_runtime_probe"

        # A valid baseline makes a following invalid result attributable to the
        # live object rather than a pre-existing static obstacle or self-collision.
        self.assertTrue(
            self._call(validity_client, self._state_validity_request()).valid,
            "The zero state must be valid before adding the dynamic blocker",
        )

        # A large base-frame box overlaps the base mesh at the zero state.
        # This proves planning safety changes, not just topic delivery.
        add = _collision_box(
            probe_id,
            CollisionObject.ADD,
            (0.0, 0.0, 0.0),
            dimensions=(1.0, 1.0, 1.0),
        )
        scene = self._wait_for_scene(
            scene_client,
            add,
            lambda objects: probe_id in objects,
            "appear after ADD",
        )
        self._probe_active = True
        self.assertEqual(tuple(scene[probe_id].primitives[0].dimensions), (1.0, 1.0, 1.0))
        self.assertFalse(
            self._call(validity_client, self._state_validity_request()).valid,
            "The ADDed dynamic blocker did not affect MoveIt's collision validity",
        )

        moved_position = (10.0, 10.0, 10.0)
        move = _collision_box(probe_id, CollisionObject.MOVE, moved_position)
        scene = self._wait_for_scene(
            scene_client,
            move,
            lambda objects: (
                probe_id in objects
                and abs(objects[probe_id].pose.position.x - moved_position[0]) < 1e-6
                and abs(objects[probe_id].pose.position.y - moved_position[1]) < 1e-6
                and abs(objects[probe_id].pose.position.z - moved_position[2]) < 1e-6
            ),
            "move after MOVE",
        )
        self.assertEqual(scene[probe_id].operation, CollisionObject.ADD)
        self.assertEqual(
            tuple(scene[probe_id].primitives[0].dimensions),
            (1.0, 1.0, 1.0),
            "MOVE must preserve the geometry installed by ADD",
        )
        self.assertTrue(
            self._call(validity_client, self._state_validity_request()).valid,
            "Moving the dynamic blocker away did not restore collision validity",
        )

        remove = _collision_box(probe_id, CollisionObject.REMOVE, moved_position)
        self._wait_for_scene(
            scene_client,
            remove,
            lambda objects: probe_id not in objects,
            "disappear after REMOVE",
        )
        self._probe_active = False
        self.assertTrue(
            self._call(validity_client, self._state_validity_request()).valid,
            "REMOVE did not restore the baseline collision validity",
        )


if __name__ == "__main__":
    unittest.main()
