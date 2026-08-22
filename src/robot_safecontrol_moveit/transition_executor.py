"""Pure-logic transition pipeline executor (no ROS imports).

The transition server node owns every ROS side effect — MoveIt service calls,
joint-state subscription, trajectory replay, the OSCBF handoff — and injects
them through :class:`TransitionPorts`.  This module owns the phase sequence,
the structured error codes, and the IK failure diagnostics, so the whole
pipeline can be exercised without a running graph.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from time import monotonic
from typing import Any, Callable

from .continuous_ik import IKError, IKServiceUnavailable
from .motion_planning import PlanningError, StateValidityError
from .task_target import (
    compute_first_task_orientation,
    load_first_task_target,
)
from .trajectory_execution import ExecutionError

# All codes considered a successful outcome (Issue #6).
SUCCESS_CODES = frozenset({
    "TRANSITION_PLANNED",
    "PLAN_ONLY_SUCCESS",
    "TRANSITION_REPLAYED",
    "TRANSITION_EXECUTED",
})

VALID_RESULT_MODES = frozenset({
    "plan_only",
    "joint_state_replay",
    "moveit_execute",
})


def _format_result(
    error_code: str, trajectory_points: int, planning_time_s: float, extra: str = ""
) -> str:
    parts = [
        f"error_code={error_code}",
        f"trajectory_points={trajectory_points}",
        f"planning_time={planning_time_s:.3f}",
    ]
    if extra:
        parts.append(extra)
    return "|".join(parts)


def _source_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _installed_share_file(relative_path: str) -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("robot_safecontrol_moveit")) / relative_path
    except (ImportError, LookupError):
        return Path("/__not_installed__")


def _resolve_path(param_value: str, default_relative: str) -> Path:
    if param_value.strip():
        path = Path(param_value).expanduser()
    else:
        installed = _installed_share_file(default_relative)
        path = installed if installed.is_file() else _source_root() / default_relative
    if not path.is_file():
        raise FileNotFoundError(f"File not found: {path}")
    return path


@dataclass(frozen=True)
class TransitionPorts:
    """Side-effect ports the phase machine drives; supplied by the ROS node."""

    log: Any
    get_parameter: Callable[[str], Any]
    check_moveit_services: Callable[[], tuple[bool, str]]
    wait_for_joint_state: Callable[..., Any]
    solve_task_state: Callable[..., Any]
    validate_state: Callable[[Any, str], None]
    plan_transition: Callable[[Any, Any], Any]
    planner_id: str
    replay: Callable[..., None]
    execute: Callable[[Any], Any]
    notify_oscbf_start: Callable[[], str]
    wait_for_plant_settle: Callable[[Any], None]
    planning_group: str
    base_frame: str
    tool_link: str


class TransitionExecutor:
    """Run one transition plan: health check, start state, goal IK, planning.

    Every phase failure returns a pipe-delimited ``error_code=...`` message
    instead of raising; the node translates the code into the Trigger
    response.
    """

    def __init__(self, ports: TransitionPorts) -> None:
        self._ports = ports

    def execute_plan(self) -> str:
        ports = self._ports
        get = ports.get_parameter
        plan_start = monotonic()
        topic = str(get("joint_state_topic").value)
        timeout = float(get("joint_state_timeout_s").value)
        max_age = float(get("max_joint_state_age_s").value)
        allow_fb = bool(get("allow_joint_state_fallback").value)

        # 0. MoveIt service health check (fail-fast before any IK call).
        services_ok, service_err = ports.check_moveit_services()
        if not services_ok:
            return _format_result(service_err, 0, monotonic() - plan_start)

        # 1. Get current joint state (Issue #3: no nested spin).
        try:
            start_state = ports.wait_for_joint_state(topic, timeout, max_age, allow_fb)
        except RuntimeError as e:
            return _format_result(str(e), 0, monotonic() - plan_start)

        ports.log.info("START_STATE_RECEIVED")

        # 2. Load the first target and make the collision-aware IK request.
        traj_file = _resolve_path(
            str(get("trajectory_mat").value), "data/nurbs/ik_input.mat"
        )
        offset = tuple(float(v) for v in get("trajectory_offset_m").value)
        max_pts = int(get("max_points").value)
        stride = int(get("point_stride").value)

        try:
            positions, _ = load_first_task_target(traj_file, offset, max_pts, stride)
        except Exception as e:
            return _format_result(
                "TRAJECTORY_LOAD_ERROR", 0, monotonic() - plan_start, f"detail={e}"
            )

        align_surface = bool(get("align_tool_x_to_surface_normal").value)
        cylinder_axis = tuple(float(v) for v in get("cylinder_axis_direction").value)
        orientation_xyzw = tuple(float(v) for v in get("orientation_xyzw").value)

        first_quat, per_point = compute_first_task_orientation(
            positions,
            align_tool_x_to_surface_normal=align_surface,
            cylinder_axis_direction=cylinder_axis,
            orientation_xyzw=orientation_xyzw,
            trajectory_mat=traj_file,
            offset_m=offset,
        )

        ports.log.info(
            f"FIRST_TARGET_POSITION={[f'{v:.4f}' for v in positions[0]]}"
        )
        ports.log.info(
            f"FIRST_TARGET_ORIENTATION=[{', '.join(f'{v:.6f}' for v in first_quat)}]"
        )

        try:
            first_goal = ports.solve_task_state(
                positions=positions,
                start_state=start_state,
                first_orientation=first_quat,
                per_point_orientations=per_point,
                max_joint_delta=float(get("max_joint_delta").value),
                ik_service_timeout_s=float(get("ik_service_timeout_s").value),
            )
        except IKServiceUnavailable as e:
            return _format_result(
                "IK_SERVICE_UNAVAILABLE", 0, monotonic() - plan_start, f"detail={e}"
            )
        except IKError as e:
            moveit_code = getattr(e, "moveit_error_code", None)
            # GOAL_IK_FAILED is reserved for a real /compute_ik response with
            # a non-SUCCESS MoveIt error code. Transport/timeouts retain their
            # own error code instead of being mislabeled as an IK solution.
            code = "GOAL_IK_FAILED" if moveit_code is not None else "IK_RESPONSE_TIMEOUT"
            extra = self._ik_failure_context(
                error=e,
                moveit_error_code=moveit_code,
                position=positions[0],
                orientation=first_quat,
                seed=start_state,
            )
            return _format_result(code, 0, monotonic() - plan_start, extra)

        ports.log.info("GOAL_IK_SUCCEEDED")

        # 3. Validate start and goal states (fail-closed).
        try:
            ports.validate_state(start_state, label="START_STATE")
        except StateValidityError as e:
            return _format_result(
                str(e).split(":")[0], 0, monotonic() - plan_start, f"detail={e}"
            )
        except PlanningError as e:
            return _format_result(
                "STATE_VALIDITY_SERVICE_UNAVAILABLE",
                0,
                monotonic() - plan_start,
                f"detail={e}",
            )

        try:
            ports.validate_state(first_goal, label="GOAL_STATE")
        except StateValidityError as e:
            return _format_result(
                str(e).split(":")[0], 0, monotonic() - plan_start, f"detail={e}"
            )
        except PlanningError as e:
            return _format_result(
                "STATE_VALIDITY_SERVICE_UNAVAILABLE",
                0,
                monotonic() - plan_start,
                f"detail={e}",
            )

        # 4. Plan with MoveIt's configured OMPL pipeline.
        try:
            transition = ports.plan_transition(start_state, first_goal)
        except PlanningError as e:
            return _format_result(
                "PLANNER_FAILED", 0, monotonic() - plan_start, f"detail={e}"
            )

        elapsed = monotonic() - plan_start
        mode = str(get("transition_result_mode").value)

        ports.log.info(
            f"TRANSITION_PLANNED: points={len(transition.points)}, "
            f"time={elapsed:.3f}s, planner={ports.planner_id}"
        )

        if mode == "plan_only":
            return _format_result("TRANSITION_PLANNED", len(transition.points), elapsed)

        if mode == "joint_state_replay":
            # Switch Viewer to ROS tracking for replay (Issue #7: fail on switch error).
            ports.log.info("Switching Viewer to ROS tracking for replay...")
            try:
                command_topic = str(get("oscbf_command_topic").value) or None
                ports.replay(
                    transition,
                    topic=str(get("replay_joint_state_topic").value),
                    rate_hz=float(get("replay_rate_hz").value),
                    command_topic=command_topic,
                )
            except ExecutionError as e:
                error_code = str(e).split(":", 1)[0]
                return _format_result(
                    error_code, len(transition.points), elapsed, f"detail={e}"
                )
            ports.log.info("TRANSITION_REPLAYED")
            if bool(get("notify_oscbf_start").value):
                ports.wait_for_plant_settle(transition)
                code = ports.notify_oscbf_start()
                ports.log.info(f"OSCBF_START_NOTIFY_RESULT={code}")
            return _format_result("TRANSITION_REPLAYED", len(transition.points), elapsed)

        if mode == "moveit_execute":
            result = ports.execute(transition)
            if result.succeeded:
                return _format_result("TRANSITION_EXECUTED", len(transition.points), elapsed)
            return _format_result("TRANSITION_EXECUTION_FAILED", len(transition.points), elapsed)

        return _format_result("TRANSITION_PLANNED", len(transition.points), elapsed)

    def _ik_failure_context(
        self,
        *,
        error: Exception,
        moveit_error_code: int | None,
        position: Any,
        orientation: Any,
        seed: Any,
    ) -> str:
        """Return machine-readable context for every failed goal IK request."""
        seed_positions = ",".join(f"{float(value):.6f}" for value in seed.position)
        return "|".join(
            [
                f"detail={error}",
                f"moveit_error_code={moveit_error_code if moveit_error_code is not None else 'NO_RESPONSE'}",
                "position=" + ",".join(f"{float(value):.6f}" for value in position),
                "orientation=" + ",".join(f"{float(value):.6f}" for value in orientation),
                f"planning_group={self._ports.planning_group}",
                f"base_frame={self._ports.base_frame}",
                f"tool_link={self._ports.tool_link}",
                "seed_names=" + ",".join(seed.name),
                f"seed_positions={seed_positions}",
                "avoid_collisions=true",
                f"timeout={float(self._ports.get_parameter('ik_service_timeout_s').value):.3f}",
            ]
        )


class AutoPlanLoop:
    """Autonomous experiment retry policy (pure logic).

    All readiness checks, the plan attempt, and the plant resampling are
    injected callables; the loop owns attempt counting, the retry decision,
    and the AUTO_PLAN_* log lines.  The controller only publishes its start
    service after JIT warm-up, so the loop waits for it before counting a
    plan failure.
    """

    def __init__(
        self,
        *,
        attempts: int,
        is_planning: Callable[[], bool],
        services_ready: Callable[[], tuple[bool, str]],
        oscbf_ready: Callable[[], bool],
        plan_once: Callable[[], tuple[bool, str]],
        randomize_plant: Callable[[], str],
        log: Any,
    ) -> None:
        self._max_attempts = attempts
        self._is_planning = is_planning
        self._services_ready = services_ready
        self._oscbf_ready = oscbf_ready
        self._plan_once = plan_once
        self._randomize_plant = randomize_plant
        self._log = log
        self._attempts = 0
        self._done = False

    @property
    def done(self) -> bool:
        return self._done

    @property
    def attempts_made(self) -> int:
        return self._attempts

    def tick(self) -> None:
        """One timer tick of the autonomous experiment."""
        if self._done or self._is_planning():
            return
        # Don't burn attempts on startup races: wait until MoveIt is actually
        # reachable before counting a plan failure.
        services_ok, _detail = self._services_ready()
        if not services_ok:
            return
        if not self._oscbf_ready():
            return
        self._attempts += 1
        success, message = self._plan_once()
        if success:
            self._done = True
            self._log.info(f"AUTO_PLAN_SUCCEEDED (attempt {self._attempts})")
            return
        if self._attempts >= self._max_attempts:
            self._done = True
            self._log.error(
                f"AUTO_PLAN_FAILED after {self._max_attempts} attempts: {message}"
            )
            return
        plant_code = self._randomize_plant()
        self._log.warn(
            f"AUTO_PLAN_RETRY {self._attempts}/{self._max_attempts}: "
            f"{message}; plant={plant_code}"
        )
