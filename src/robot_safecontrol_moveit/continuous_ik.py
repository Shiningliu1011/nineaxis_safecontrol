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

import numpy as np

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
    ``natural_seeds`` holds candidate joint poses (warm-start priors) for the
    FIRST waypoint.  When non-empty the first IK solve is attempted from each
    candidate; the collision-free solutions are ranked by a naturalness cost
    (elbow down, upper/forearm bend, shoulder abduction, wrist neutral,
    joint-limit margin, rail preference, manipulability) and the best one
    seeds the rest of the path.  Empty keeps the historic single-seed path.
    """

    tool_link: str
    orientation_xyzw: tuple[float, float, float, float]
    planning_group: str = ""
    base_frame: str = ""
    max_joint_delta: float = 0.15
    service_timeout_s: float = 2.0
    natural_seeds: tuple[tuple[float, ...], ...] = ()


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
                if index == 0 and self._options.natural_seeds:
                    ordered, error_code = self._solve_natural_first(
                        position=position,
                        quat_xyzw=quat,
                        seed_state=previous,
                    )
                    if ordered is None:
                        raise IKFailure(index, position, moveit_error_code=error_code)
                else:
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

    # ------------------------------------------------------------------
    # Natural-configuration first solve (multi-seed + kinematics scoring)
    # ------------------------------------------------------------------

    def _solve_natural_first(
        self,
        position: tuple[float, float, float],
        quat_xyzw: tuple[float, float, float, float],
        seed_state: JointState,
    ) -> tuple[JointState | None, int | None]:
        """Solve the first waypoint from several seeds and keep the best pose.

        The seed-dependent MoveIt IK converges to the branch nearest the
        seed, so a random plant start pose can produce a highly extended,
        "elbow-out" configuration.  Try the current state plus each natural
        prior seed, score every collision-free solution with a naturalness
        cost (elbow down, arm bend, shoulder abduction, wrist neutrality,
        joint-limit margin, rail preference, manipulability) and use the
        winner to seed the rest of the continuous solve.

        Returns (ordered_best_solution, last_error_code).  When no candidate
        solves, ``None`` is returned with the last MoveIt error code.
        """
        candidates = [seed_state]
        for template in self._options.natural_seeds:
            candidates.append(self._template_state(template))

        solutions: list[JointState] = []
        last_error_code: int | None = None
        for candidate in candidates:
            solution, error_code = self._compute_ik_direct(
                position=position,
                quat_xyzw=quat_xyzw,
                ik_link_name=self._options.tool_link,
                start_joint_state=candidate,
            )
            if solution is None:
                last_error_code = error_code
                continue
            solutions.append(self._ordered_state(solution))

        if not solutions:
            return None, last_error_code

        kin = self._naturalness_kinematics()
        singularities: list[float] = []
        for state in solutions:
            try:
                singularities.append(
                    float(kin.singularity_index(np.asarray(state.position)))
                )
            except Exception:
                singularities.append(0.0)
        si_max = max(singularities)
        if si_max <= 0.0:
            si_max = 1.0

        scores: list[tuple[float, JointState, float]] = [
            (
                self._naturalness_cost(state, kin, si_max),
                state,
                si,
            )
            for state, si in zip(solutions, singularities)
        ]
        cost, best, best_si = min(scores, key=lambda item: item[0])

        node = self._node
        if node is not None:
            node.get_logger().info(
                "NATURAL_IK first-waypoint candidates: "
                f"{len(candidates)} seeds -> {len(solutions)} solutions; "
                f"chosen cost={cost:.4f} si={best_si:.5f}; "
                f"q={[float(v) for v in best.position]}"
            )
        return best, None

    def _template_state(self, template: Sequence[float]) -> JointState:
        if len(template) != len(self._joint_names):
            raise ValueError(
                f"natural seed must have {len(self._joint_names)} values, got {len(template)}"
            )
        state = JointState()
        state.name = list(self._joint_names)
        state.position = [float(value) for value in template]
        return state

    def _naturalness_kinematics(self):
        """Lazily built portable nine-axis kinematics for FK scoring."""
        if getattr(self, "_kin_cache", None) is None:
            from .oscbf_trajectory import bootstrap_portable, default_portable_root

            bootstrap_portable(default_portable_root())
            from work.nineaxis_kinematics import NineaxisKinematics

            self._kin_cache = NineaxisKinematics()
        return self._kin_cache

    @classmethod
    def _naturalness_cost(
        cls,
        state: JointState,
        kin,
        si_max: float,
    ) -> float:
        """Aggregate naturalness cost; lower is better (0..~4 range).

        Terms (all normalised to O(1)).  The task surface sits above the
        shoulder line, so the natural-looking "elbow down" posture is
        expressed as a clearly bent forearm rather than a squeezed wrist or
        an extended, elbow-out arm:
          * bend: upper-arm vs forearm angle kept inside [80, 150] deg.
          * shoulder abduction: |J2|+|J3| kept small (normalised by range).
          * wrist neutrality: max |J6..J9| kept small (normalised).
          * joint-limit margin: squared distance to the nearest limit.
          * rail preference: J1 kept near 0.32 m mid-stroke so the linear
            rail provides margin instead of forcing arm extension.
          * manipulability: singular-value product, higher is better.
        """
        q = np.asarray(state.position, dtype=float)
        transforms = kin.forward_kinematics(q)
        shoulder = transforms["Link2"][:3, 3]
        elbow = transforms["Link4"][:3, 3]   # 肘尖 (J4 轴点)
        wrist = transforms["Link6"][:3, 3]   # 腕 (J6 轴点)
        upper = elbow - shoulder
        fore = wrist - elbow
        upper_norm = float(np.linalg.norm(upper))
        fore_norm = float(np.linalg.norm(fore))
        if upper_norm < 1e-9 or fore_norm < 1e-9:
            bend_rad = 0.0
        else:
            cosine = float(np.dot(upper, fore) / (upper_norm * fore_norm))
            bend_rad = np.arccos(np.clip(cosine, -1.0, 1.0))
        # 工作点都位于肩部上方, "肘向下"的自然解释 = 前臂明显折弯(而非
        # 沿上臂方向伸直外张)且不向侧面张开: bend 目标区间 80..150 度。
        bend_deg = float(np.degrees(bend_rad))
        bend_penalty = max(0.0, 80.0 - bend_deg) + max(0.0, bend_deg - 150.0)
        shoulder_abduction = (abs(float(q[1])) + abs(float(q[2]))) / (2.0 * 1.5708)
        # 腕带 0.5 rad 的容忍带: 该任务需要腕轴做小幅调整, 但超出 0.5 rad
        # 的明显弯折线性处罚。(J5 是滚转, 不纳入; J6..J9 保留对俯仰的控制。)
        wrist_max = float(np.max(np.abs(q[6:9])))
        wrist_dev = max(wrist_max - 0.5, 0.0) / (1.48353 - 0.5)
        limits = kin.joint_limits
        ranges = limits.q_max - limits.q_min
        midpoints = 0.5 * (limits.q_max + limits.q_min)
        margin = float(
            np.min(2.0 * (limits.q_max - np.abs(q - midpoints)) / ranges)
        )
        margin_penalty = (1.0 - min(margin, 1.0)) ** 2
        # 优先利用导轨: 高位行程 (0.45 m 附近) 可显著减小肩/腕角度需求
        rail_dev = abs(float(q[0]) - 0.45) / 0.585
        manip = 0.0
        try:
            manip = float(kin.singularity_index(q)) / max(si_max, 1e-12)
        except Exception:
            manip = 0.0

        return (
            0.05 * bend_penalty
            + 0.4 * shoulder_abduction
            + 1.0 * wrist_dev
            + 1.3 * margin_penalty
            + 0.3 * rail_dev
            + 0.5 * (1.0 - manip)
        )

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
