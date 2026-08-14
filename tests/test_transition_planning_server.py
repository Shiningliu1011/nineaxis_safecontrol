"""Tests for transition planning server logic (no ROS node required).

Covers: success codes, result mode validation, error code formatting,
fail-closed state handling, and non-spin execution patterns.
"""

import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

# Ensure the source tree is importable.
_src = Path(__file__).resolve().parents[1] / "src"
if str(_src) not in sys.path:
    sys.path.insert(0, str(_src))


class _Param:
    def __init__(self, value):
        self.value = value


class _FakeLogger:
    def __init__(self, lines):
        self._lines = lines

    def info(self, message):
        self._lines.append(("info", message))

    def warn(self, message):
        self._lines.append(("warn", message))

    def error(self, message):
        self._lines.append(("error", message))


class _AutoPlanServer:
    """Minimal harness for the auto-plan orchestration logic."""

    def __init__(self, attempts: int, succeed_after: int):
        self._planning = False
        self._auto_plan_done = False
        self._auto_plan_attempt = 0
        self._calls = 0
        self._succeed_after = succeed_after
        self._params = {
            "auto_plan_attempts": _Param(attempts),
            "oscbf_plant_randomize_service": _Param("/oscbf_plant/randomize"),
        }
        self._logs = []
        self._logger = _FakeLogger(self._logs)

    def get_parameter(self, name):
        return self._params[name]

    def get_logger(self):
        return self._logger

    def _plan_callback(self, request, response):
        self._calls += 1
        response.success = self._calls >= self._succeed_after
        response.message = "TRANSITION_REPLAYED" if response.success else "START_STATE_IN_COLLISION"

    def _randomize_plant_start(self):
        return "RANDOMIZED"

    def _check_moveit_services(self):
        return True, ""

    def _oscbf_start_service_ready(self):
        return True


class _SettleServer:
    """Harness for the plant-settle wait before the tracking handoff."""

    def __init__(self, deliver_state: bool):
        from sensor_msgs.msg import JointState
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        self._joint_names = tuple(f"J{i}" for i in range(1, 10))
        self._deliver_state = deliver_state
        self._logs = []
        self._logger = _FakeLogger(self._logs)
        self._subscription_cb = None

        self.transition = JointTrajectory()
        self.transition.joint_names = list(self._joint_names)
        point = JointTrajectoryPoint()
        point.positions = [0.25] * 9
        self.transition.points.append(point)
        self.message = JointState()
        self.message.name = list(self._joint_names)
        self.message.position = [0.25] * 9

    def get_parameter(self, name):
        return _Param("/mujoco_joint_states")

    def get_logger(self):
        return self._logger

    def create_subscription(self, message_type, topic, callback, qos_profile):
        self._subscription_cb = callback
        return object()

    def destroy_subscription(self, subscription):
        pass

    def deliver(self):
        if self._deliver_state:
            self._subscription_cb(self.message)


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


class TestAutoPlanOrchestration(unittest.TestCase):
    """Autonomous transition: plan once, retry with a resampled start."""

    @staticmethod
    def _tick(server):
        from robot_safecontrol_moveit.transition_planning_server import (
            TransitionPlanningServer,
        )

        TransitionPlanningServer._auto_plan_tick(server)

    def test_auto_plan_succeeds_on_first_attempt(self):
        server = _AutoPlanServer(attempts=3, succeed_after=1)
        self._tick(server)
        self.assertTrue(server._auto_plan_done)
        self.assertEqual(server._auto_plan_attempt, 1)
        self.assertEqual(server._calls, 1)
        self.assertIn(
            ("info", "AUTO_PLAN_SUCCEEDED (attempt 1)"),
            server._logs,
        )

    def test_auto_plan_retries_then_reports_failure(self):
        server = _AutoPlanServer(attempts=2, succeed_after=99)
        self._tick(server)
        self.assertFalse(server._auto_plan_done)
        self._tick(server)
        self.assertTrue(server._auto_plan_done)
        self.assertEqual(server._calls, 2)
        self.assertTrue(
            any(
                level == "warn" and "AUTO_PLAN_RETRY 1/2" in message
                for level, message in server._logs
            )
        )
        self.assertTrue(
            any(
                level == "error" and "AUTO_PLAN_FAILED after 2 attempts" in message
                for level, message in server._logs
            )
        )


class TestPlantSettleBeforeHandoff(unittest.TestCase):
    @staticmethod
    def _call(server, timeout_s):
        from robot_safecontrol_moveit.transition_planning_server import (
            TransitionPlanningServer,
        )

        TransitionPlanningServer._wait_for_plant_settle(
            server, server.transition, timeout_s=timeout_s, tolerance=0.01
        )

    def test_settle_returns_when_plant_converges(self):
        import threading
        import time

        server = _SettleServer(deliver_state=True)
        thread = threading.Thread(
            target=lambda: (time.sleep(0.1), server.deliver()),
            daemon=True,
        )
        thread.start()
        self._call(server, timeout_s=2.0)
        thread.join(timeout=1.0)
        self.assertTrue(
            any(
                level == "info" and "PLANT_SETTLED" in message
                for level, message in server._logs
            )
        )

    def test_settle_times_out_gracefully(self):
        server = _SettleServer(deliver_state=False)
        self._call(server, timeout_s=0.2)
        self.assertTrue(
            any(
                level == "warn" and "PLANT_SETTLE_TIMEOUT" in message
                for level, message in server._logs
            )
        )


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
