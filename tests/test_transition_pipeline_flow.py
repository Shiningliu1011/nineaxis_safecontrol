"""Behavioral tests for the transition-planning and replay pipelines.

These tests deliberately exercise the production methods rather than inspect
source text.  ROS transports and MoveIt are represented by tiny fakes so the
ordering and fail-closed behavior can be verified without a running graph.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


# Keep this test independently runnable as well as pytest-collectable.
_SRC = Path(__file__).resolve().parents[1] / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from builtin_interfaces.msg import Duration, Time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from robot_safecontrol_moveit.continuous_ik import IKServiceUnavailable
from robot_safecontrol_moveit.motion_planning import StateValidityError
from robot_safecontrol_moveit.trajectory_execution import (
    ExecutionError,
    TrajectoryExecutor,
)
from robot_safecontrol_moveit.transition_planning_server import (
    TransitionPlanningServer,
)
import robot_safecontrol_moveit.transition_planning_server as planning_server
import robot_safecontrol_moveit.trajectory_execution as trajectory_execution


def _result_code(result: str) -> str:
    """Read the stable top-level status field returned by the server."""
    fields = dict(
        item.split("=", 1) for item in result.split("|") if "=" in item
    )
    return fields["error_code"]


def _joint_state(names=("J1", "J2"), positions=(0.1, -0.2)) -> JointState:
    state = JointState()
    state.name = list(names)
    state.position = [float(value) for value in positions]
    return state


class _Logger:
    def __init__(self) -> None:
        self.messages: list[tuple[str, str]] = []

    def info(self, message: object, *args, **kwargs) -> None:
        self.messages.append(("info", str(message)))

    def warning(self, message: object, *args, **kwargs) -> None:
        self.messages.append(("warning", str(message)))

    def error(self, message: object, *args, **kwargs) -> None:
        self.messages.append(("error", str(message)))


class _PipelinePlanner:
    """Small planner fake that records calls made by ``_execute_plan``."""

    def __init__(self, events: list[str]) -> None:
        self.events = events
        self._options = SimpleNamespace(planner_id="RRTConnectkConfigDefault")
        self.start_validation_error: Exception | None = None
        self.plan_error: Exception | None = None

    def validate_state(self, state: JointState, *, label: str) -> None:
        self.events.append(f"validate:{label}")
        if label == "START_STATE" and self.start_validation_error is not None:
            raise self.start_validation_error

    def plan_transition(
        self, start_state: JointState, goal_state: JointState
    ) -> SimpleNamespace:
        self.events.append("plan")
        if self.plan_error is not None:
            raise self.plan_error
        return SimpleNamespace(points=[object(), object(), object()])


class _PipelineServer:
    """The minimum server-shaped object accepted by the unbound method."""

    _DEFAULT_PARAMETERS = {
        "joint_state_topic": "/mujoco_joint_states",
        "joint_state_timeout_s": 1.0,
        "max_joint_state_age_s": 0.5,
        "allow_joint_state_fallback": False,
        "scene_sync_timeout_s": 1.0,
        "scene_position_tolerance_m": 0.001,
        "scene_dimension_tolerance_m": 0.001,
        "scene_orientation_tolerance": 0.001,
        "trajectory_mat": "",
        "trajectory_offset_m": (0.0, 0.0, 0.0),
        "max_points": 1,
        "point_stride": 1,
        "align_tool_x_to_surface_normal": False,
        "cylinder_axis_direction": (0.0, 0.0, 1.0),
        "orientation_xyzw": (0.0, 0.0, 0.0, 1.0),
        "max_joint_delta": 0.25,
        "ik_service_timeout_s": 1.0,
        "transition_result_mode": "plan_only",
        "replay_joint_state_topic": "/mujoco_joint_states",
        "replay_rate_hz": 30.0,
    }

    def __init__(self, events: list[str], *, scene_result=(True, "")) -> None:
        self.events = events
        self._parameters = dict(self._DEFAULT_PARAMETERS)
        self._logger = _Logger()
        self._joint_names = ("J1", "J2")
        self._tool_link = "tool0"
        self._base_frame = "base_link"
        self._planning_group = "arm"
        self._moveit = object()
        self._planner = _PipelinePlanner(events)
        self._executor = object()
        self._start_state = _joint_state()
        self._scene_result = scene_result

    def get_parameter(self, name: str) -> SimpleNamespace:
        return SimpleNamespace(value=self._parameters[name])

    def get_logger(self) -> _Logger:
        return self._logger

    def _check_moveit_services(self) -> tuple[bool, str]:
        self.events.append("services")
        return True, ""

    def _wait_for_joint_state(self, *args, **kwargs) -> JointState:
        self.events.append("fresh_state")
        return self._start_state

    def _synchronize_scene(self, *args, **kwargs) -> tuple[bool, str]:
        self.events.append("scene")
        return self._scene_result


class TestTransitionPlanningPipeline(unittest.TestCase):
    def _patched_task_helpers(self, events: list[str], solve):
        """Patch task inputs only; the server method remains the real code."""
        return (
            patch.object(
                planning_server,
                "_resolve_path",
                return_value=Path("/tmp/ik_input.mat"),
            ),
            patch.object(
                planning_server,
                "load_first_task_target",
                side_effect=lambda *args, **kwargs: (
                    events.append("target") or ([(0.4, 0.2, 0.8)], [0.0])
                ),
            ),
            patch.object(
                planning_server,
                "compute_first_task_orientation",
                side_effect=lambda *args, **kwargs: (
                    events.append("orientation") or ((0.0, 0.0, 0.0, 1.0), None)
                ),
            ),
            patch.object(
                planning_server,
                "solve_first_task_state",
                side_effect=solve,
            ),
        )

    def test_execute_plan_runs_the_fail_closed_stages_in_order(self):
        events: list[str] = []
        fake_server = _PipelineServer(events)
        goal_state = _joint_state(positions=(0.3, -0.1))

        def solve(**kwargs):
            events.append("ik")
            self.assertIs(kwargs["start_state"], fake_server._start_state)
            return goal_state

        patches = self._patched_task_helpers(events, solve)
        with patches[0], patches[1], patches[2], patches[3]:
            result = TransitionPlanningServer._execute_plan(fake_server)

        self.assertEqual(_result_code(result), "TRANSITION_PLANNED")
        self.assertEqual(
            events,
            [
                "services",
                "fresh_state",
                "scene",
                "target",
                "orientation",
                "ik",
                "validate:START_STATE",
                "validate:GOAL_STATE",
                "plan",
            ],
        )

    def test_scene_sync_failure_returns_before_target_or_ik(self):
        events: list[str] = []
        fake_server = _PipelineServer(
            events, scene_result=(False, "SCENE_OBJECT_POSE_MISMATCH")
        )

        def forbidden_ik(**kwargs):
            events.append("ik")
            raise AssertionError("IK must not run after scene synchronization fails")

        patches = self._patched_task_helpers(events, forbidden_ik)
        with patches[0], patches[1], patches[2], patches[3]:
            result = TransitionPlanningServer._execute_plan(fake_server)

        self.assertEqual(_result_code(result), "SCENE_OBJECT_POSE_MISMATCH")
        self.assertEqual(events, ["services", "fresh_state", "scene"])

    def test_ik_service_exception_is_returned_as_its_structured_status(self):
        events: list[str] = []
        fake_server = _PipelineServer(events)

        def unavailable_ik(**kwargs):
            events.append("ik")
            raise IKServiceUnavailable()

        patches = self._patched_task_helpers(events, unavailable_ik)
        with patches[0], patches[1], patches[2], patches[3]:
            result = TransitionPlanningServer._execute_plan(fake_server)

        self.assertEqual(_result_code(result), "IK_SERVICE_UNAVAILABLE")
        self.assertEqual(events[-1], "ik")
        self.assertNotIn("plan", events)

    def test_state_validity_exception_is_returned_before_planning(self):
        events: list[str] = []
        fake_server = _PipelineServer(events)
        fake_server._planner.start_validation_error = StateValidityError(
            "START_STATE_COLLISION: fixture contact"
        )

        def solve(**kwargs):
            events.append("ik")
            return _joint_state(positions=(0.3, -0.1))

        patches = self._patched_task_helpers(events, solve)
        with patches[0], patches[1], patches[2], patches[3]:
            result = TransitionPlanningServer._execute_plan(fake_server)

        self.assertEqual(_result_code(result), "START_STATE_COLLISION")
        self.assertEqual(events[-1], "validate:START_STATE")
        self.assertNotIn("plan", events)


class _UnavailableService:
    def wait_for_service(self, *, timeout_sec: float) -> bool:
        return False


class _ReplayPublisher:
    def __init__(self) -> None:
        self.messages: list[JointState] = []

    def publish(self, message: JointState) -> None:
        self.messages.append(message)


class _ReplayClock:
    def now(self) -> "_ReplayClock":
        return self

    def to_msg(self) -> Time:
        return Time(sec=123, nanosec=456)


class _ReplayNode:
    def __init__(self) -> None:
        self.events: list[str] = []
        self.publisher: _ReplayPublisher | None = None
        self.destroyed_publishers: list[_ReplayPublisher] = []
        self._logger = _Logger()
        self._clock = _ReplayClock()

    def create_client(self, service_type, service_name: str):
        self.events.append(f"client:{service_name}")
        # The replay success test deliberately leaves the broadcaster service
        # unavailable; that is a recoverable production path.
        return _UnavailableService()

    def create_publisher(self, message_type, topic: str, qos):
        self.events.append(f"publisher:{topic}")
        self.publisher = _ReplayPublisher()
        return self.publisher

    def destroy_publisher(self, publisher: _ReplayPublisher) -> None:
        self.events.append("destroy_publisher")
        self.destroyed_publishers.append(publisher)

    def destroy_client(self, client) -> None:
        self.events.append("destroy_client")

    def get_logger(self) -> _Logger:
        return self._logger

    def get_clock(self) -> _ReplayClock:
        return self._clock


class _ReplayMoveIt:
    """Replay does not need MoveIt calls, but receives this explicit fake."""


def _trajectory(joint_names=("J2", "J1")) -> JointTrajectory:
    trajectory = JointTrajectory()
    trajectory.joint_names = list(joint_names)
    for seconds, positions in ((0.1, (20.0, 10.0)), (0.2, (21.0, 11.0))):
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.time_from_start = Duration(
            sec=int(seconds),
            nanosec=int((seconds % 1.0) * 1_000_000_000),
        )
        trajectory.points.append(point)
    return trajectory


class TestTrajectoryReplay(unittest.TestCase):
    def test_tracking_switch_failure_happens_before_publisher_creation(self):
        node = _ReplayNode()
        executor = TrajectoryExecutor(node, _ReplayMoveIt(), ("J2", "J1"))

        with self.assertRaises(ExecutionError) as raised:
            executor.replay(_trajectory(), switch_viewer_to_tracking=True)

        self.assertIn("VIEWER_TRACKING_SWITCH_FAILED", str(raised.exception))
        self.assertEqual(node.events[0], "client:set_mujoco_manual_mode")
        self.assertFalse(
            any(event.startswith("publisher:") for event in node.events)
        )
        self.assertIsNone(node.publisher)

    def test_replay_publishes_points_in_the_trajectory_joint_order(self):
        node = _ReplayNode()
        executor = TrajectoryExecutor(node, _ReplayMoveIt(), ("J2", "J1"))

        with patch.object(trajectory_execution, "sleep", return_value=None):
            result = executor.replay(
                _trajectory(), topic="/mujoco_joint_states", rate_hz=100.0
            )

        self.assertTrue(result.submitted)
        self.assertTrue(result.completed)
        self.assertTrue(result.succeeded)
        self.assertIsNotNone(node.publisher)
        assert node.publisher is not None
        self.assertEqual(
            [list(message.name) for message in node.publisher.messages],
            [["J2", "J1"], ["J2", "J1"]],
        )
        self.assertEqual(
            [list(message.position) for message in node.publisher.messages],
            [[20.0, 10.0], [21.0, 11.0]],
        )
        self.assertEqual(node.destroyed_publishers, [node.publisher])


if __name__ == "__main__":
    unittest.main()
