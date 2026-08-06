"""Collision-aware transition planning through MoveIt's planning pipeline.

No OMPL planner, collision checker, or path interpolator is implemented here.
``pymoveit2`` forwards the request to MoveIt's standard ROS 2
``/plan_kinematic_path`` service, where the configured OMPL pipeline and
PlanningScene perform those jobs.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from time import monotonic, sleep
from typing import Sequence

from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from pymoveit2 import MoveIt2
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class PlanningError(RuntimeError):
    """MoveIt could not create or validate a transition trajectory."""


class StateValidityError(PlanningError):
    """The start or goal state failed MoveIt's collision/limit checks."""


@dataclass(frozen=True)
class PlanningOptions:
    pipeline_id: str = "ompl"
    planner_id: str = "AEBRRTstarFaithfulConfigDefault"
    planning_time_s: float = 10.0
    # Use one AEB-RRT* instance per request.  More than one causes MoveIt's
    # ParallelPlan/PathHybridization layer to combine concurrent candidates,
    # which is neither needed by the faithful AEB mode nor desirable for a
    # deterministic transition replay.
    planning_attempts: int = 1
    velocity_scale: float = 0.2
    acceleration_scale: float = 0.2
    goal_joint_tolerance: float = 1e-3


class MotionPlanner:
    """Owns MoveIt planning parameters and collision-aware state validation."""

    def __init__(
        self,
        node,
        moveit: MoveIt2,
        joint_names: Sequence[str],
        options: PlanningOptions,
        planning_group: str = "arm",
    ):
        self._node = node
        self._moveit = moveit
        self._joint_names = tuple(joint_names)
        self._options = options
        self._planning_group = str(planning_group)
        self._state_validity_client = node.create_client(
            GetStateValidity, "check_state_validity"
        )
        self._validate_options()
        self._configure_moveit()

    def validate_state(
        self,
        state: JointState,
        *,
        label: str = "state",
    ) -> None:
        """Check a joint state against MoveIt's collision and limit validity.

        Calls ``/check_state_validity`` and raises ``StateValidityError``
        with a specific prefix if invalid.  Uses non-spin polling (Issue #3).
        """
        ordered = self._ordered_state(state)
        if not self._state_validity_client.wait_for_service(timeout_sec=2.0):
            raise PlanningError(
                f"STATE_VALIDITY_SERVICE_UNAVAILABLE: cannot validate {label}"
            )

        request = GetStateValidity.Request()
        request.robot_state = RobotState()
        request.robot_state.joint_state = ordered
        # ``pymoveit2.MoveIt2`` keeps its group name private. Retain the
        # configured value ourselves rather than depending on a non-existent
        # public attribute, so state validation always reaches MoveIt.
        request.group_name = self._planning_group

        future = self._state_validity_client.call_async(request)
        deadline = monotonic() + 3.0
        while not future.done() and monotonic() < deadline:
            sleep(0.01)
        if not future.done():
            raise PlanningError(
                f"STATE_VALIDITY_TIMEOUT: {label} validation timed out"
            )

        response = future.result()
        if response is None:
            raise PlanningError(
                f"STATE_VALIDITY_NO_RESPONSE: {label} validation returned null"
            )

        if not response.valid:
            contacts = []
            for contact in response.contacts:
                contacts.append(
                    f"{contact.contact_body_1}-{contact.contact_body_2}"
                )
            contact_info = (
                f" contacts=[{', '.join(contacts)}]" if contacts else ""
            )
            # Issue #8: use safe classification — don't use joint names as link names.
            # Report START_STATE_COLLISION or GOAL_STATE_COLLISION with contact bodies.
            detail = f"{label.upper()}_INVALID"
            raise StateValidityError(
                f"{label.upper()}_COLLISION: {label} is invalid"
                f"{contact_info}"
            )

        self._node.get_logger().info(f"{label.upper()}_VALID")

    def plan_transition(
        self,
        start_state: JointState,
        goal_state: JointState,
    ) -> JointTrajectory:
        """Ask MoveIt/OMPL for a collision-free start-to-first-IK trajectory."""
        start = self._ordered_state(start_state)
        goal = self._ordered_state(goal_state)

        self.validate_state(start, label="START_STATE")
        self.validate_state(goal, label="GOAL_STATE")

        # ``MoveIt2.plan()`` spins its node internally.  This planner runs
        # inside the persistent server's MultiThreadedExecutor, so use the
        # asynchronous request and let that executor service the future.
        future = self._moveit.plan_async(
            joint_positions=list(goal.position),
            joint_names=list(self._joint_names),
            tolerance_joint_position=self._options.goal_joint_tolerance,
            start_joint_state=start,
        )
        if future is None:
            raise PlanningError(
                "PLANNER_SERVICE_UNAVAILABLE: MoveIt did not accept the planning request"
            )

        deadline = monotonic() + self._options.planning_time_s + 3.0
        while not future.done() and monotonic() < deadline:
            sleep(0.01)
        if not future.done():
            raise PlanningError(
                "PLANNER_TIMEOUT: /plan_kinematic_path did not respond before the deadline"
            )

        try:
            trajectory = self._moveit.get_trajectory(future)
        except Exception as exc:
            raise PlanningError(f"PLANNER_FAILED: {exc}") from exc
        if trajectory is None or not trajectory.points:
            raise PlanningError(
                "PLANNER_FAILED: MoveIt returned no transition trajectory. "
                "Check the PlanningScene, joint state, collision objects, "
                "and planner configuration."
            )
        if tuple(trajectory.joint_names) != self._joint_names:
            raise PlanningError(
                "MoveIt returned a trajectory with unexpected joint order: "
                f"{list(trajectory.joint_names)}"
            )
        MotionPlanner._validate_trajectory_goal(self, trajectory, goal)
        return trajectory

    def _validate_trajectory_goal(
        self,
        trajectory: JointTrajectory,
        goal: JointState,
    ) -> None:
        """Reject a nominally-successful plan that does not end at its goal.

        MoveIt normally enforces this contract, but treating a partial or
        approximate path as executable would make the MuJoCo replay stop
        short of the first task waypoint.  The C++ AEB planner makes the same
        check before returning an exact solution; this is an independent
        client-side safety boundary.
        """
        final_positions = tuple(float(value) for value in trajectory.points[-1].positions)
        expected_positions = tuple(float(value) for value in goal.position)
        expected_count = len(self._joint_names)
        if (
            len(final_positions) != expected_count
            or len(expected_positions) != expected_count
        ):
            raise PlanningError(
                "PLANNER_GOAL_MISMATCH: trajectory endpoint has an unexpected "
                f"joint count (got {len(final_positions)}, expected {expected_count})"
            )

        errors = [
            abs(actual - expected)
            for actual, expected in zip(final_positions, expected_positions)
        ]
        if not all(isfinite(error) for error in errors):
            raise PlanningError(
                "PLANNER_GOAL_MISMATCH: trajectory endpoint contains a non-finite "
                "joint position"
            )

        tolerance = self._options.goal_joint_tolerance
        if max(errors, default=0.0) > tolerance:
            detail = ", ".join(
                f"{name}:error={error:.6f}"
                for name, error in zip(self._joint_names, errors)
                if error > tolerance
            )
            raise PlanningError(
                "PLANNER_GOAL_MISMATCH: trajectory endpoint does not reach the "
                f"requested IK goal within {tolerance:.6f} rad ({detail})"
            )

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
