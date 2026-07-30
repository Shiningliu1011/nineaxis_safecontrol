"""Continuous inverse kinematics backed by MoveIt's ``/compute_ik`` service.

This module deliberately contains no numerical IK implementation.  It delegates
every solve and collision check to MoveIt through the open-source ``pymoveit2``
ROS 2 client.  Continuity comes from using the preceding MoveIt solution as the
seed for the next solve.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, sqrt
from typing import Iterable, Sequence

from pymoveit2 import MoveIt2
from sensor_msgs.msg import JointState


class IKError(RuntimeError):
    """Base error raised when MoveIt cannot provide a usable IK solution."""


class IKFailure(IKError):
    """MoveIt failed to solve one waypoint."""

    def __init__(self, waypoint_index: int, position: Sequence[float]):
        super().__init__(
            "MoveIt failed to solve collision-free IK for waypoint "
            f"{waypoint_index}: {tuple(float(value) for value in position)}"
        )
        self.waypoint_index = waypoint_index
        self.position = tuple(float(value) for value in position)


class IKDiscontinuity(IKError):
    """The solver changed branch too far between adjacent waypoints."""

    def __init__(
        self,
        waypoint_index: int,
        joint_name: str,
        delta: float,
        max_delta: float,
    ):
        super().__init__(
            f"IK discontinuity at waypoint {waypoint_index}, joint {joint_name}: "
            f"delta={delta:.6g}, limit={max_delta:.6g}"
        )
        self.waypoint_index = waypoint_index
        self.joint_name = joint_name
        self.delta = delta
        self.max_delta = max_delta


@dataclass(frozen=True)
class IKOptions:
    """Controls exposed by the continuous-IK stage.

    ``orientation_xyzw`` is required by the MoveIt service message even when
    the selected kinematics plugin is configured with ``position_only_ik``.
    ``max_joint_delta`` is an adjacent-waypoint branch-change guard; set it to
    zero only when that guard is intentionally disabled.
    """

    tool_link: str
    orientation_xyzw: tuple[float, float, float, float]
    max_joint_delta: float = 0.15
    service_timeout_s: float = 2.0


@dataclass(frozen=True)
class IKPath:
    """Ordered, collision-checked joint solutions returned by MoveIt."""

    joint_names: tuple[str, ...]
    states: tuple[JointState, ...]

    @property
    def first_state(self) -> JointState:
        if not self.states:
            raise IKError("IK path is empty")
        return self.states[0]


class ContinuousIK:
    """Solve a pose sequence with a persistent MoveIt IK seed."""

    def __init__(
        self,
        moveit: MoveIt2,
        joint_names: Sequence[str],
        options: IKOptions,
    ):
        self._moveit = moveit
        self._joint_names = tuple(joint_names)
        self._options = options
        self._orientation = self._normalise_quaternion(options.orientation_xyzw)
        if options.max_joint_delta < 0.0:
            raise ValueError("max_joint_delta must be non-negative")
        if options.service_timeout_s <= 0.0:
            raise ValueError("service_timeout_s must be positive")

    def solve(
        self,
        positions: Iterable[Sequence[float]],
        seed_state: JointState,
    ) -> IKPath:
        """Solve all positions, using each MoveIt result as the next seed.

        The first solution may differ from ``seed_state`` because the separate
        planning stage is responsible for moving the robot to it.  Continuity
        checks therefore begin at the second Cartesian waypoint.
        """

        previous = self._ordered_state(seed_state)
        solutions: list[JointState] = []

        for index, raw_position in enumerate(positions):
            position = self._position(raw_position)
            solution = self._moveit.compute_ik(
                position=position,
                quat_xyzw=self._orientation,
                ik_link_name=self._options.tool_link,
                start_joint_state=previous,
                wait_for_server_timeout_sec=self._options.service_timeout_s,
            )
            if solution is None:
                raise IKFailure(index, position)

            ordered = self._ordered_state(solution)
            if solutions and self._options.max_joint_delta > 0.0:
                self._check_continuity(index, previous, ordered)

            solutions.append(ordered)
            previous = ordered

        if not solutions:
            raise IKError("No Cartesian waypoints were supplied to continuous IK")

        return IKPath(joint_names=self._joint_names, states=tuple(solutions))

    def _ordered_state(self, state: JointState) -> JointState:
        positions = dict(zip(state.name, state.position))
        missing = [name for name in self._joint_names if name not in positions]
        if missing:
            raise IKError(f"Joint state is missing required joints: {missing}")

        result = JointState()
        result.header = state.header
        result.name = list(self._joint_names)
        result.position = [float(positions[name]) for name in self._joint_names]
        return result

    def _check_continuity(
        self,
        waypoint_index: int,
        previous: JointState,
        current: JointState,
    ) -> None:
        for joint_name, before, after in zip(
            self._joint_names, previous.position, current.position
        ):
            delta = abs(float(after) - float(before))
            if delta > self._options.max_joint_delta:
                raise IKDiscontinuity(
                    waypoint_index,
                    joint_name,
                    delta,
                    self._options.max_joint_delta,
                )

    @staticmethod
    def _position(values: Sequence[float]) -> tuple[float, float, float]:
        if len(values) != 3 or not all(isfinite(float(value)) for value in values):
            raise ValueError(f"IK position must contain three finite values, got {values}")
        return tuple(float(value) for value in values)  # type: ignore[return-value]

    @staticmethod
    def _normalise_quaternion(
        values: Sequence[float],
    ) -> tuple[float, float, float, float]:
        if len(values) != 4 or not all(isfinite(float(value)) for value in values):
            raise ValueError(
                "orientation_xyzw must contain four finite quaternion components"
            )
        length = sqrt(sum(float(value) ** 2 for value in values))
        if length == 0.0:
            raise ValueError("orientation_xyzw must not be the zero quaternion")
        return tuple(float(value) / length for value in values)  # type: ignore[return-value]
