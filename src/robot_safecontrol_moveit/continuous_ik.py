"""Continuous inverse kinematics backed by MoveIt's ``/compute_ik`` service.

This module deliberately contains no numerical IK implementation.  It delegates
every solve and collision check to MoveIt through the open-source ``pymoveit2``
ROS 2 client.  Continuity comes from using the preceding MoveIt solution as the
seed for the next solve.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, sqrt
from time import monotonic, sleep
from typing import Iterable, Sequence

from builtin_interfaces.msg import Duration
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from pymoveit2 import MoveIt2
from sensor_msgs.msg import JointState


class IKError(RuntimeError):
    """Base error raised when MoveIt cannot provide a usable IK solution."""


class IKServiceUnavailable(IKError):
    """The ``/compute_ik`` service is not reachable."""

    def __init__(self):
        super().__init__(
            "IK_SERVICE_UNAVAILABLE: /compute_ik service is not reachable. "
            "Ensure move_group is running."
        )


class IKFailure(IKError):
    """MoveIt replied to one waypoint with a non-success error code."""

    def __init__(self, waypoint_index: int, position: Sequence[float],
                 moveit_error_code: int | None = None):
        parts = [
            "MoveIt failed to solve collision-free IK for waypoint "
            f"{waypoint_index}: {tuple(float(value) for value in position)}"
        ]
        if moveit_error_code is not None:
            parts.append(f" [moveit_error_code={moveit_error_code}]")
        super().__init__("".join(parts))
        self.waypoint_index = waypoint_index
        self.position = tuple(float(value) for value in position)
        self.moveit_error_code = moveit_error_code


class IKResponseTimeout(IKError):
    """The available IK service did not return a usable response in time."""

    def __init__(self, detail: str):
        super().__init__(f"IK_RESPONSE_TIMEOUT: {detail}")


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
    planning_group: str = ""
    base_frame: str = ""
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
    """Solve a pose sequence with a persistent MoveIt IK seed.

    Uses a direct ``GetPositionIK`` service call so that MoveIt error
    codes are preserved and distinguishable from service-unavailable
    conditions.
    """

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
        self._node = getattr(moveit, "_node", None)
        self._ik_client = None
        if not options.planning_group:
            raise ValueError("planning_group must be non-empty")
        if not options.base_frame:
            raise ValueError("base_frame must be non-empty")
        if options.max_joint_delta < 0.0:
            raise ValueError("max_joint_delta must be non-negative")
        if options.service_timeout_s <= 0.0:
            raise ValueError("service_timeout_s must be positive")

    def solve(
        self,
        positions: Iterable[Sequence[float]],
        seed_state: JointState,
        *,
        orientations: Iterable[Sequence[float]] | None = None,
    ) -> IKPath:
        """Solve all positions, using each MoveIt result as the next seed.

        The first solution may differ from ``seed_state`` because the separate
        planning stage is responsible for moving the robot to it.  Continuity
        checks therefore begin at the second Cartesian waypoint.

        When *orientations* is supplied it must provide one quaternion
        (xyzw) per position; each waypoint is then solved with its own
        target orientation instead of the global default.
        """

        if self._node is None:
            raise IKServiceUnavailable()
        self._ik_client = self._node.create_client(GetPositionIK, "compute_ik")
        try:
            # Check service availability once before any solve attempt.
            if not self._is_ik_service_available():
                raise IKServiceUnavailable()

            previous = self._ordered_state(seed_state)
            solutions: list[JointState] = []

            # Convert to list so we can check length against orientations.
            position_list = list(positions)
            if not position_list:
                raise IKError("No Cartesian waypoints were supplied to continuous IK")

            orientation_list: list[Sequence[float]] | None
            if orientations is not None:
                orientation_list = [
                    self._normalise_quaternion(q) for q in orientations
                ]
                if len(orientation_list) != len(position_list):
                    raise IKError(
                        f"orientations count ({len(orientation_list)}) must match "
                        f"positions count ({len(position_list)})"
                    )
            else:
                orientation_list = None

            for index, raw_position in enumerate(position_list):
                position = self._position(raw_position)
                quat = (
                    self._orientation
                    if orientation_list is None
                    else orientation_list[index]
                )
                solution, error_code = self._compute_ik_direct(
                    position=position,
                    quat_xyzw=quat,
                    ik_link_name=self._options.tool_link,
                    start_joint_state=previous,
                )
                if solution is None:
                    raise IKFailure(index, position, moveit_error_code=error_code)

                ordered = self._ordered_state(solution)
                if solutions and self._options.max_joint_delta > 0.0:
                    self._check_continuity(index, previous, ordered)

                solutions.append(ordered)
                previous = ordered

            if not solutions:
                raise IKError("No Cartesian waypoints were supplied to continuous IK")

            return IKPath(joint_names=self._joint_names, states=tuple(solutions))
        finally:
            self._node.destroy_client(self._ik_client)
            self._ik_client = None

    # ------------------------------------------------------------------
    # Direct GetPositionIK call (preserves MoveIt error codes)
    # ------------------------------------------------------------------

    def _is_ik_service_available(self) -> bool:
        """Check whether ``/compute_ik`` is reachable via a short timeout."""
        try:
            return bool(self._ik_client and self._ik_client.wait_for_service(timeout_sec=1.0))
        except Exception:
            return False

    def _compute_ik_direct(
        self,
        position: tuple[float, float, float],
        quat_xyzw: tuple[float, float, float, float],
        ik_link_name: str,
        start_joint_state: JointState,
    ) -> tuple[JointState | None, int | None]:
        """Call ``compute_ik_async`` and inspect ``error_code.val``.

        Returns ``(solution, error_code)`` only after a real service response.
        A transport failure or a future that does not resolve is raised as
        :class:`IKResponseTimeout`, so callers never misreport it as
        ``GOAL_IK_FAILED``.
        """
        if self._ik_client is None:
            raise IKServiceUnavailable()

        request = GetPositionIK.Request()
        request.ik_request.group_name = self._options.planning_group
        request.ik_request.ik_link_name = ik_link_name
        request.ik_request.pose_stamped.header.frame_id = self._options.base_frame
        request.ik_request.pose_stamped.header.stamp = self._node.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose.position.x = position[0]
        request.ik_request.pose_stamped.pose.position.y = position[1]
        request.ik_request.pose_stamped.pose.position.z = position[2]
        request.ik_request.pose_stamped.pose.orientation.x = quat_xyzw[0]
        request.ik_request.pose_stamped.pose.orientation.y = quat_xyzw[1]
        request.ik_request.pose_stamped.pose.orientation.z = quat_xyzw[2]
        request.ik_request.pose_stamped.pose.orientation.w = quat_xyzw[3]
        request.ik_request.robot_state.joint_state = start_joint_state
        request.ik_request.robot_state.is_diff = False
        request.ik_request.avoid_collisions = True
        timeout_sec = int(self._options.service_timeout_s)
        timeout_nanosec = int(
            round((self._options.service_timeout_s - timeout_sec) * 1_000_000_000)
        )
        if timeout_nanosec == 1_000_000_000:
            timeout_sec += 1
            timeout_nanosec = 0
        request.ik_request.timeout = Duration(sec=timeout_sec, nanosec=timeout_nanosec)
        future = self._ik_client.call_async(request)

        deadline = monotonic() + self._options.service_timeout_s + 2.0
        while not future.done() and monotonic() < deadline:
            sleep(0.01)
        if not future.done():
            raise IKResponseTimeout(
                f"/compute_ik exceeded {self._options.service_timeout_s:.3f}s"
            )

        try:
            response = future.result()
        except Exception as exc:
            raise IKResponseTimeout(f"/compute_ik future raised {exc}") from exc

        if response is None:
            raise IKResponseTimeout("/compute_ik returned no response")

        error_val = response.error_code.val
        if error_val == MoveItErrorCodes.SUCCESS:
            return response.solution.joint_state, error_val
        return None, error_val

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
