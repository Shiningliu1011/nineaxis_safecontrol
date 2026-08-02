"""Production-path tests for MoveIt state-validity requests."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


_SRC = Path(__file__).resolve().parents[1] / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from robot_safecontrol_moveit.motion_planning import (
    MotionPlanner,
    PlanningError,
    PlanningOptions,
)


class _Logger:
    def __init__(self) -> None:
        self.messages: list[str] = []

    def info(self, message: object) -> None:
        self.messages.append(str(message))


class _Client:
    def __init__(self) -> None:
        self.request = None

    def wait_for_service(self, *, timeout_sec: float) -> bool:
        return True

    def call_async(self, request):
        self.request = request
        return SimpleNamespace(
            done=lambda: True,
            result=lambda: SimpleNamespace(valid=True, contacts=[]),
        )


class TestStateValidityRequest(unittest.TestCase):
    def test_uses_configured_group_not_a_pymoveit_private_attribute(self):
        """The real method must work with the installed pymoveit2 API."""
        client = _Client()
        logger = _Logger()
        state = JointState(name=["J1"], position=[0.1])
        planner = SimpleNamespace(
            _state_validity_client=client,
            _planning_group="arm",
            _ordered_state=lambda value: value,
            _node=SimpleNamespace(get_logger=lambda: logger),
            # Deliberately no ``_moveit.group_name`` exists in this fake.
        )

        MotionPlanner.validate_state(planner, state, label="START_STATE")

        self.assertEqual(client.request.group_name, "arm")
        self.assertEqual(client.request.robot_state.joint_state.name, ["J1"])
        self.assertIn("START_STATE_VALID", logger.messages)

    def test_planning_uses_async_moveit_request_without_nested_spin(self):
        """The production method must not call pymoveit2's blocking ``plan``."""
        state = JointState(name=["J1"], position=[0.1])
        trajectory = JointTrajectory()
        trajectory.joint_names = ["J1"]
        trajectory.points = [JointTrajectoryPoint(positions=[0.1])]
        calls: list[dict] = []
        future = SimpleNamespace(done=lambda: True)
        moveit = SimpleNamespace(
            plan_async=lambda **kwargs: calls.append(kwargs) or future,
            get_trajectory=lambda returned_future: trajectory,
            # Deliberately no blocking ``plan`` member exists.
        )
        planner = SimpleNamespace(
            _ordered_state=lambda value: value,
            validate_state=lambda value, *, label: None,
            _joint_names=("J1",),
            _options=PlanningOptions(planning_time_s=0.1),
            _moveit=moveit,
        )

        actual = MotionPlanner.plan_transition(planner, state, state)

        self.assertIs(actual, trajectory)
        self.assertEqual(calls[0]["joint_positions"], [0.1])
        self.assertIs(calls[0]["start_joint_state"], state)

    def test_planning_rejects_a_trajectory_that_stops_before_the_goal(self):
        state = JointState(name=["J1"], position=[0.1])
        truncated = JointTrajectory()
        truncated.joint_names = ["J1"]
        truncated.points = [JointTrajectoryPoint(positions=[0.05])]
        future = SimpleNamespace(done=lambda: True)
        moveit = SimpleNamespace(
            plan_async=lambda **kwargs: future,
            get_trajectory=lambda returned_future: truncated,
        )
        planner = SimpleNamespace(
            _ordered_state=lambda value: value,
            validate_state=lambda value, *, label: None,
            _joint_names=("J1",),
            _options=PlanningOptions(planning_time_s=0.1),
            _moveit=moveit,
        )

        with self.assertRaisesRegex(PlanningError, "PLANNER_GOAL_MISMATCH"):
            MotionPlanner.plan_transition(planner, state, state)


if __name__ == "__main__":
    unittest.main()
