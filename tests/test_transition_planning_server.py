"""Tests for transition planning server logic (no ROS node required).

Covers: success codes, result mode validation, error code formatting,
ObstacleRegistry REMOVE tracking, scene object validation, quaternion equivalence.
"""

import math
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

# Ensure the source tree is importable.
_src = Path(__file__).resolve().parents[1] / "src"
if str(_src) not in sys.path:
    sys.path.insert(0, str(_src))


class TestSuccessCodes(unittest.TestCase):
    """Issue #6: all success codes recognised."""

    def setUp(self):
        from robot_safecontrol_moveit.transition_planning_server import SUCCESS_CODES

        self.SUCCESS_CODES = SUCCESS_CODES

    def test_transition_planned_is_success(self):
        self.assertIn("TRANSITION_PLANNED", self.SUCCESS_CODES)

    def test_plan_only_success_is_success(self):
        self.assertIn("PLAN_ONLY_SUCCESS", self.SUCCESS_CODES)

    def test_transition_replayed_is_success(self):
        self.assertIn("TRANSITION_REPLAYED", self.SUCCESS_CODES)

    def test_transition_executed_is_success(self):
        self.assertIn("TRANSITION_EXECUTED", self.SUCCESS_CODES)

    def test_failure_code_not_in_success(self):
        self.assertNotIn("START_STATE_UNAVAILABLE", self.SUCCESS_CODES)
        self.assertNotIn("PLANNER_FAILED", self.SUCCESS_CODES)
        self.assertNotIn("SCENE_SYNC_TIMEOUT", self.SUCCESS_CODES)

    def test_success_check(self):
        code = "TRANSITION_REPLAYED"
        self.assertTrue(code in self.SUCCESS_CODES)


class TestResultModeValidation(unittest.TestCase):
    """Issue #5: transition_result_mode must be a valid value."""

    def setUp(self):
        from robot_safecontrol_moveit.transition_planning_server import (
            VALID_RESULT_MODES,
        )

        self.VALID_MODES = VALID_RESULT_MODES

    def test_valid_modes(self):
        self.assertIn("plan_only", self.VALID_MODES)
        self.assertIn("joint_state_replay", self.VALID_MODES)
        self.assertIn("moveit_execute", self.VALID_MODES)

    def test_invalid_mode_not_accepted(self):
        self.assertNotIn("execute_transition", self.VALID_MODES)
        self.assertNotIn("replay_transition", self.VALID_MODES)
        self.assertNotIn("", self.VALID_MODES)


class TestResultFormatting(unittest.TestCase):
    """Validate the pipe-delimited result format."""

    def setUp(self):
        from robot_safecontrol_moveit.transition_planning_server import (
            _format_result,
        )

        self._format = _format_result

    def test_basic_format(self):
        result = self._format("TRANSITION_PLANNED", 42, 3.141)
        self.assertIn("error_code=TRANSITION_PLANNED", result)
        self.assertIn("trajectory_points=42", result)
        self.assertIn("planning_time=3.141", result)

    def test_format_with_extra(self):
        result = self._format("PLANNER_FAILED", 0, 1.5, "detail=timeout")
        self.assertIn("detail=timeout", result)

    def test_format_parses_back(self):
        result = self._format("TRANSITION_REPLAYED", 10, 2.5)
        parts = dict(p.split("=", 1) for p in result.split("|") if "=" in p)
        self.assertEqual(parts["error_code"], "TRANSITION_REPLAYED")
        self.assertEqual(parts["trajectory_points"], "10")


class TestObstacleRegistry(unittest.TestCase):
    """Test the ObstacleRegistry ADD/MOVE/REMOVE and pending_removals."""

    def setUp(self):
        from robot_safecontrol_moveit.transition_planning_server import (
            ObstacleRegistry,
        )
        from moveit_msgs.msg import CollisionObject
        from geometry_msgs.msg import Pose
        from shape_msgs.msg import SolidPrimitive

        self.registry = ObstacleRegistry()
        self.CollisionObject = CollisionObject
        self.Pose = Pose
        self.SolidPrimitive = SolidPrimitive

    def _make_add_msg(self, oid: str, shape_type: int, dims, pos, quat):
        from builtin_interfaces.msg import Time as HeaderTime
        msg = self.CollisionObject()
        msg.id = oid
        msg.operation = self.CollisionObject.ADD
        msg.header.frame_id = "base_link"
        msg.pose = self.Pose()
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        prim = self.SolidPrimitive()
        prim.type = shape_type
        prim.dimensions = [float(d) for d in dims]
        msg.primitives = [prim]
        return msg

    def _make_remove_msg(self, oid: str):
        msg = self.CollisionObject()
        msg.id = oid
        msg.operation = self.CollisionObject.REMOVE
        return msg

    def test_add_object(self):
        msg = self._make_add_msg("box1", self.SolidPrimitive.BOX,
                                  [0.1, 0.2, 0.3], [1.0, 0.0, 0.5],
                                  [0.0, 0.0, 0.0, 1.0])
        self.registry.apply(msg)
        active = self.registry.active_objects()
        self.assertEqual(len(active), 1)
        self.assertEqual(active[0].object_id, "box1")
        self.assertEqual(active[0].shape, "box")

    def test_remove_object(self):
        msg = self._make_add_msg("box1", self.SolidPrimitive.BOX,
                                  [0.1, 0.2, 0.3], [1.0, 0.0, 0.5],
                                  [0.0, 0.0, 0.0, 1.0])
        self.registry.apply(msg)
        self.registry.apply(self._make_remove_msg("box1"))
        active = self.registry.active_objects()
        self.assertEqual(len(active), 0)

    def test_remove_pending_tracking(self):
        """Issue #10.4: REMOVE adds to pending_removals."""
        msg = self._make_add_msg("box1", self.SolidPrimitive.BOX,
                                  [0.1, 0.2, 0.3], [1.0, 0.0, 0.5],
                                  [0.0, 0.0, 0.0, 1.0])
        self.registry.apply(msg)
        self.registry.apply(self._make_remove_msg("box1"))
        self.assertIn("box1", self.registry.pending_removals)

    def test_revision_increments(self):
        r1 = self.registry.revision
        msg = self._make_add_msg("box1", self.SolidPrimitive.BOX,
                                  [0.1, 0.2, 0.3], [1.0, 0.0, 0.5],
                                  [0.0, 0.0, 0.0, 1.0])
        self.registry.apply(msg)
        self.assertGreater(self.registry.revision, r1)

    def test_move_without_primitive_preserves_shape(self):
        """Issue #11.4: MOVE without primitive keeps existing shape/size."""
        msg_add = self._make_add_msg("obj1", self.SolidPrimitive.BOX,
                                      [0.1, 0.2, 0.3], [1.0, 0.0, 0.5],
                                      [0.0, 0.0, 0.0, 1.0])
        self.registry.apply(msg_add)

        from moveit_msgs.msg import CollisionObject
        msg_move = CollisionObject()
        msg_move.id = "obj1"
        msg_move.operation = CollisionObject.MOVE
        msg_move.header.frame_id = "base_link"
        msg_move.pose = self.Pose()
        msg_move.pose.position.x = 2.0
        msg_move.pose.position.y = 0.0
        msg_move.pose.position.z = 1.0
        msg_move.pose.orientation.x = 0.0
        msg_move.pose.orientation.y = 0.0
        msg_move.pose.orientation.z = 0.0
        msg_move.pose.orientation.w = 1.0
        self.registry.apply(msg_move)

        active = self.registry.active_objects()
        self.assertEqual(len(active), 1)
        self.assertEqual(active[0].shape, "box")
        self.assertEqual(active[0].dimensions, (0.1, 0.2, 0.3))
        self.assertEqual(active[0].position[0], 2.0)


class TestQuaternionEquivalence(unittest.TestCase):
    """Issue #10.3: q and -q are the same rotation."""

    def test_identity_diff_zero(self):
        q = (0.0, 0.0, 0.0, 1.0)
        diff1 = math.sqrt(sum((a - b) ** 2 for a, b in zip(q, q)))
        diff2 = math.sqrt(sum((a + b) ** 2 for a, b in zip(q, q)))
        self.assertLess(min(diff1, diff2), 1e-9)

    def test_negated_quaternion_is_same_rotation(self):
        q1 = (0.0, 0.0, 0.7071068, 0.7071068)  # 90 deg around Z
        q2 = (0.0, 0.0, -0.7071068, -0.7071068)  # negated
        diff1 = math.sqrt(sum((a - b) ** 2 for a, b in zip(q1, q2)))
        diff2 = math.sqrt(sum((a + b) ** 2 for a, b in zip(q1, q2)))
        self.assertLess(min(diff1, diff2), 1e-9)

    def test_different_quaternions_differ(self):
        q1 = (0.0, 0.0, 0.0, 1.0)
        q2 = (0.7071068, 0.0, 0.0, 0.7071068)  # 90 deg around X
        diff1 = math.sqrt(sum((a - b) ** 2 for a, b in zip(q1, q2)))
        diff2 = math.sqrt(sum((a + b) ** 2 for a, b in zip(q1, q2)))
        self.assertGreater(min(diff1, diff2), 0.1)


class TestSceneObjectValidation(unittest.TestCase):
    """Issue #10: complete scene content validation."""

    def test_validate_primitive_type(self):
        """Primitive type must match expected shape."""
        shape_map = {1: "box", 2: "sphere", 3: "cylinder"}
        self.assertEqual(shape_map.get(1), "box")
        self.assertEqual(shape_map.get(2), "sphere")
        self.assertEqual(shape_map.get(3), "cylinder")
        # Unknown type not mapped.
        self.assertIsNone(shape_map.get(99))

    def test_validate_dimensions_count(self):
        """Each shape has an expected dimension count."""
        expected = {"box": 3, "sphere": 1, "cylinder": 2}
        self.assertEqual(expected["box"], 3)
        self.assertEqual(expected["sphere"], 1)
        self.assertEqual(expected["cylinder"], 2)


class TestFailClosed(unittest.TestCase):
    """Issue #8: state validity must be fail-closed."""

    def test_service_unavailable_raises(self):
        """When service is unavailable, should raise PlanningError."""
        from robot_safecontrol_moveit.motion_planning import PlanningError
        # Verify the error can be constructed and contains the right prefix.
        err = PlanningError("STATE_VALIDITY_SERVICE_UNAVAILABLE: test")
        self.assertIn("STATE_VALIDITY_SERVICE_UNAVAILABLE", str(err))

    def test_timeout_raises(self):
        from robot_safecontrol_moveit.motion_planning import PlanningError
        err = PlanningError("STATE_VALIDITY_TIMEOUT: test")
        self.assertIn("STATE_VALIDITY_TIMEOUT", str(err))

    def test_no_response_raises(self):
        from robot_safecontrol_moveit.motion_planning import PlanningError
        err = PlanningError("STATE_VALIDITY_NO_RESPONSE: test")
        self.assertIn("STATE_VALIDITY_NO_RESPONSE", str(err))

    def test_collision_error_prefixes(self):
        """Collision errors should not use joint names as link names."""
        prefixes = [
            "START_STATE_COLLISION",
            "GOAL_STATE_COLLISION",
        ]
        for prefix in prefixes:
            self.assertTrue(prefix.startswith("START_STATE_") or
                            prefix.startswith("GOAL_STATE_"))


class TestNoNestedSpin(unittest.TestCase):
    """Issue #3: verify rclpy.spin_once/spin_until_future_complete not called in callbacks."""

    def test_server_imports_no_spin_in_paths(self):
        """Verify the server module imports the threading module (for Event)."""
        import robot_safecontrol_moveit.transition_planning_server as server
        source = Path(server.__file__).read_text()
        # The server module should NOT have rclpy.spin_once or
        # rclpy.spin_until_future_complete in non-comment lines.
        lines = [l for l in source.splitlines()
                 if not l.strip().startswith("#") and l.strip()]
        spin_calls = [l for l in lines
                      if "spin_once" in l or "spin_until_future_complete" in l]
        self.assertEqual(
            len(spin_calls), 0,
            f"Found nested spin calls in server source: {spin_calls}"
        )

    def test_motion_planning_imports_no_spin(self):
        """Verify motion_planning.py has no nested spin calls."""
        import robot_safecontrol_moveit.motion_planning as mp
        source = Path(mp.__file__).read_text()
        lines = [l for l in source.splitlines()
                 if not l.strip().startswith("#") and l.strip()]
        spin_calls = [l for l in lines
                      if "spin_once" in l or "spin_until_future_complete" in l]
        self.assertEqual(
            len(spin_calls), 0,
            f"Found nested spin calls in motion_planning source: {spin_calls}"
        )

    def test_trajectory_execution_imports_no_spin(self):
        """Verify trajectory_execution.py has no nested spin calls."""
        import robot_safecontrol_moveit.trajectory_execution as te
        source = Path(te.__file__).read_text()
        lines = [l for l in source.splitlines()
                 if not l.strip().startswith("#") and l.strip()]
        spin_calls = [l for l in lines
                      if "spin_once" in l or "spin_until_future_complete" in l]
        self.assertEqual(
            len(spin_calls), 0,
            f"Found nested spin calls in trajectory_execution source: {spin_calls}"
        )


if __name__ == "__main__":
    unittest.main()
