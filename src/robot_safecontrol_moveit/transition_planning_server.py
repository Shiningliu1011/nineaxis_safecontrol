"""Persistent planning server: one MoveIt client, repeated /plan_transition_once.

Usage::

    ros2 run robot_safecontrol_moveit transition_planning_server

Triggers a full transition plan on each ``std_srvs/Trigger`` call to
``/plan_transition_once``.  The response message encodes a pipe-delimited
key=value result so callers can parse structured information::

    error_code=START_STATE_UNAVAILABLE|trajectory_points=0|planning_time=0.0
"""

from __future__ import annotations

import os
import threading
from dataclasses import dataclass
from math import isfinite, sqrt

import rclpy
from pathlib import Path
from time import monotonic, sleep
from typing import Sequence

import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetMotionPlan,
    GetPlanningScene,
    GetPositionIK,
    GetStateValidity,
)
from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger

from .motion_planning import (
    CollisionObjectSpec,
    MotionPlanner,
    PlanningError,
    PlanningOptions,
    StateValidityError,
)
from .continuous_ik import IKError, IKServiceUnavailable
from .trajectory_execution import ExecutionError, TrajectoryExecutor
from .task_target import (
    compute_first_task_orientation,
    load_first_task_target,
    solve_first_task_state,
)

DEFAULT_JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9")

# All codes the server considers a successful outcome (Issue #6).
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

STATIC_COLLISION_OBJECT_TOPIC = "/static_collision_object"
DYNAMIC_COLLISION_OBJECT_TOPIC = "/collision_object"


def static_collision_qos(depth: int = 100) -> QoSProfile:
    """QoS contract for retained static scene objects."""
    return QoSProfile(
        depth=max(1, int(depth)),
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


def dynamic_collision_qos(depth: int = 100) -> QoSProfile:
    """QoS contract for live ADD/MOVE/REMOVE scene updates."""
    return QoSProfile(
        depth=max(1, int(depth)),
        durability=DurabilityPolicy.VOLATILE,
        reliability=ReliabilityPolicy.RELIABLE,
    )


# ---------------------------------------------------------------------------
# Obstacle registry (tracks latest dynamic obstacle state)
# ---------------------------------------------------------------------------


@dataclass
class ObstacleEntry:
    object_id: str
    shape: str
    dimensions: tuple[float, ...]
    position: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    frame_id: str
    active: bool
    revision: int


class ObstacleRegistry:
    """Thread-safe registry of dynamic obstacle state.

    Tracks pending REMOVE operations so the PlanningScene verifier can
    confirm objects have been fully removed (Issue #10.4).
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._objects: dict[str, ObstacleEntry] = {}
        self._revision = 0
        self._pending_removals: set[str] = set()

    def apply(self, msg: CollisionObject) -> None:
        op_raw = msg.operation
        if isinstance(op_raw, bytes):
            op_val = int.from_bytes(op_raw, "little")
        else:
            op_val = int(op_raw)
        object_id = msg.id

        with self._lock:
            self._revision += 1
            if op_val == 1:  # REMOVE
                entry = self._objects.get(object_id)
                if entry is not None:
                    entry.active = False
                    entry.revision = self._revision
                self._pending_removals.add(object_id)
            elif op_val in (0, 2, 3):  # ADD, APPEND, MOVE
                shape = "unknown"
                dims: tuple[float, ...] = ()
                if msg.primitives:
                    prim = msg.primitives[0]
                    from shape_msgs.msg import SolidPrimitive
                    prim_type = prim.type
                    if isinstance(prim_type, bytes):
                        prim_type = int.from_bytes(prim_type, "little")
                    shape = {SolidPrimitive.BOX: "box", SolidPrimitive.SPHERE: "sphere",
                             SolidPrimitive.CYLINDER: "cylinder"}.get(prim_type, "unknown")
                    dims = tuple(float(d) for d in prim.dimensions)
                elif object_id in self._objects:
                    shape = self._objects[object_id].shape
                    dims = self._objects[object_id].dimensions

                pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
                quat = (msg.pose.orientation.x, msg.pose.orientation.y,
                        msg.pose.orientation.z, msg.pose.orientation.w)

                self._objects[object_id] = ObstacleEntry(
                    object_id=object_id, shape=shape, dimensions=dims,
                    position=pos, quaternion_xyzw=quat,
                    frame_id=msg.header.frame_id, active=True,
                    revision=self._revision,
                )
                self._pending_removals.discard(object_id)

    @property
    def revision(self) -> int:
        with self._lock:
            return self._revision

    @property
    def pending_removals(self) -> frozenset[str]:
        with self._lock:
            return frozenset(self._pending_removals)

    def _clear_removal(self, object_id: str) -> None:
        with self._lock:
            self._pending_removals.discard(object_id)

    def active_objects(self) -> list[ObstacleEntry]:
        with self._lock:
            return [e for e in self._objects.values() if e.active]

    def active_object(self, object_id: str) -> ObstacleEntry | None:
        """Return an immutable-by-convention snapshot for one active object."""
        with self._lock:
            entry = self._objects.get(object_id)
            if entry is None or not entry.active:
                return None
            return ObstacleEntry(**entry.__dict__)


# ---------------------------------------------------------------------------
# Shared pipeline helpers
# ---------------------------------------------------------------------------


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


def load_collision_objects(path: Path, default_frame: str) -> tuple[CollisionObjectSpec, ...]:
    EXPECTED_DIMS = {"box": 3, "sphere": 1, "cylinder": 2}
    with path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    entries = document.get("obstacles", [])
    if not isinstance(entries, list):
        raise ValueError("obstacles.yaml must contain an 'obstacles' list")
    objects: list[CollisionObjectSpec] = []
    seen_ids: set[str] = set()
    for entry in entries:
        object_id = str(entry["id"])
        if object_id in seen_ids:
            raise ValueError(f"Duplicate obstacle ID: {object_id!r}")
        seen_ids.add(object_id)
        shape = str(entry["shape"])
        if shape not in EXPECTED_DIMS:
            raise ValueError(f"Unsupported shape {shape!r}")
        position = tuple(float(v) for v in entry["position"])
        dimensions = tuple(float(v) for v in entry["dimensions"])
        if len(dimensions) != EXPECTED_DIMS[shape] or any(v <= 0.0 for v in dimensions):
            raise ValueError(f"Invalid dimensions for {object_id!r}")
        raw_quat = tuple(float(v) for v in entry.get("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0]))
        norm = sqrt(sum(v * v for v in raw_quat))
        quat = tuple(v / norm for v in raw_quat) if norm > 1e-12 else (0.0, 0.0, 0.0, 1.0)
        objects.append(CollisionObjectSpec(
            object_id=object_id, shape=shape, position=position,
            dimensions=dimensions, frame_id=str(entry.get("frame_id", default_frame)),
            quaternion_xyzw=quat,
        ))
    return tuple(objects)


def _format_result(error_code: str, trajectory_points: int, planning_time_s: float,
                   extra: str = "") -> str:
    parts = [
        f"error_code={error_code}",
        f"trajectory_points={trajectory_points}",
        f"planning_time={planning_time_s:.3f}",
    ]
    if extra:
        parts.append(extra)
    return "|".join(parts)


# ---------------------------------------------------------------------------
# Planning server node
# ---------------------------------------------------------------------------


class TransitionPlanningServer(Node):
    """Persistent node: one MoveIt session, many /plan_transition_once calls."""

    def __init__(self) -> None:
        super().__init__("transition_planning_server")
        self._declare_parameters()
        self._validate_result_mode()

        joint_names = tuple(str(n) for n in self.get_parameter("joint_names").value)
        self._joint_names = joint_names
        self._planning_group = str(self.get_parameter("planning_group").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._tool_link = str(self.get_parameter("tool_link").value)

        # Init MoveIt2 once.
        self._moveit = MoveIt2(
            node=self, joint_names=list(joint_names),
            base_link_name=self._base_frame, end_effector_name=self._tool_link,
            group_name=self._planning_group, ignore_new_calls_while_executing=True,
        )

        planner_opts = PlanningOptions(
            pipeline_id=str(self.get_parameter("planning_pipeline").value),
            planner_id=str(self.get_parameter("planner_id").value),
            planning_time_s=float(self.get_parameter("planning_time_s").value),
            planning_attempts=int(self.get_parameter("planning_attempts").value),
            velocity_scale=float(self.get_parameter("velocity_scale").value),
            acceleration_scale=float(self.get_parameter("acceleration_scale").value),
            goal_joint_tolerance=float(self.get_parameter("goal_joint_tolerance").value),
        )
        self._planner = MotionPlanner(
            self,
            self._moveit,
            joint_names,
            planner_opts,
            planning_group=self._planning_group,
        )
        self._executor = TrajectoryExecutor(self, self._moveit, joint_names)

        # Static objects are received through a retained topic, registered
        # locally, and applied explicitly through ApplyPlanningScene. Dynamic
        # objects remain on MoveIt's normal volatile collision-object topic.
        obstacles_file = _resolve_path(
            str(self.get_parameter("obstacles_file").value), "config/obstacles.yaml"
        )
        self._static_obstacle_specs = load_collision_objects(
            obstacles_file, self._base_frame
        )
        self._static_registry = ObstacleRegistry()
        self._registry = ObstacleRegistry()
        self._collision_sub = self.create_subscription(
            CollisionObject,
            str(self.get_parameter("collision_object_topic").value),
            self._registry.apply,
            dynamic_collision_qos(),
        )
        self._static_collision_sub = self.create_subscription(
            CollisionObject,
            str(self.get_parameter("static_collision_object_topic").value),
            self._static_registry.apply,
            static_collision_qos(),
        )

        # Planning service.
        self._planning = False
        self._srv = self.create_service(
            Trigger, "/plan_transition_once", self._plan_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        # Health-check clients for MoveIt services (created once, reused).
        self._ik_check_client = self.create_client(
            GetPositionIK, "compute_ik"
        )
        self._validity_check_client = self.create_client(
            GetStateValidity, "check_state_validity"
        )
        self._scene_check_client = self.create_client(
            GetPlanningScene, "get_planning_scene"
        )
        self._apply_scene_client = self.create_client(
            ApplyPlanningScene, "apply_planning_scene"
        )

        # Install static geometry as soon as both retained messages and the
        # MoveIt service are available. The per-request synchronization below
        # repeats verification before IK, so this eager installation is not a
        # timing-dependent fallback.
        self._static_scene_applied_revision = -1
        self._static_scene_apply_lock = threading.Lock()
        self._static_install_timer = self.create_timer(
            0.2, self._install_static_scene_timer,
            callback_group=ReentrantCallbackGroup(),
        )
        self._plan_check_client = self.create_client(
            GetMotionPlan, "plan_kinematic_path"
        )

        self.get_logger().info(
            f"TRANSITION_SERVER_STARTED pid={os.getpid()} "
            f"config=mujoco_transition_runtime.yaml"
        )

        self.get_logger().info(
            f"Loaded {len(self._static_obstacle_specs)} static obstacle(s): "
            f"{[s.object_id for s in self._static_obstacle_specs]}; "
            f"waiting on {self.get_parameter('static_collision_object_topic').value}"
        )

        self.get_logger().info(
            "Transition planning server ready on /plan_transition_once"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("joint_names", list(DEFAULT_JOINT_NAMES))
        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tool_link", "tool0")
        self.declare_parameter("trajectory_mat", "")
        self.declare_parameter("trajectory_offset_m", [0.0, 0.343, 1.587])
        self.declare_parameter("obstacles_file", "")
        self.declare_parameter("max_points", 1)
        self.declare_parameter("point_stride", 1)
        self.declare_parameter("joint_state_topic", "/mujoco_joint_states")
        self.declare_parameter("joint_state_timeout_s", 3.0)
        self.declare_parameter("max_joint_state_age_s", 1.0)
        self.declare_parameter("allow_joint_state_fallback", False)
        self.declare_parameter(
            "static_collision_object_topic", STATIC_COLLISION_OBJECT_TOPIC
        )
        self.declare_parameter("collision_object_topic", DYNAMIC_COLLISION_OBJECT_TOPIC)
        self.declare_parameter("scene_sync_timeout_s", 3.0)
        self.declare_parameter("scene_position_tolerance_m", 0.001)
        self.declare_parameter("scene_dimension_tolerance_m", 0.001)
        self.declare_parameter("scene_orientation_tolerance", 0.001)
        self.declare_parameter("planning_pipeline", "ompl")
        self.declare_parameter(
            "planner_id", "AEBRRTstarFaithfulConfigDefault"
        )
        self.declare_parameter("planning_time_s", 10.0)
        # One faithful AEB-RRT* solve per transition.  Multiple attempts make
        # MoveIt invoke ParallelPlan/PathHybridization over concurrent AEB
        # candidates, which is not part of the requested planner core.
        self.declare_parameter("planning_attempts", 1)
        self.declare_parameter("velocity_scale", 0.2)
        self.declare_parameter("acceleration_scale", 0.2)
        self.declare_parameter("goal_joint_tolerance", 0.001)
        self.declare_parameter("transition_result_mode", "plan_only")
        self.declare_parameter("replay_joint_state_topic", "/mujoco_joint_states")
        self.declare_parameter("replay_rate_hz", 30.0)
        self.declare_parameter("orientation_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("max_joint_delta", 0.15)
        self.declare_parameter("ik_service_timeout_s", 2.0)
        self.declare_parameter("align_tool_x_to_surface_normal", True)
        self.declare_parameter("cylinder_axis_direction", [0.0, 1.0, 0.0])

    def _validate_result_mode(self) -> None:
        mode = str(self.get_parameter("transition_result_mode").value)
        if mode not in VALID_RESULT_MODES:
            raise ValueError(
                f"transition_result_mode={mode!r} is invalid; "
                f"must be one of {sorted(VALID_RESULT_MODES)}"
            )

    def _check_moveit_services(self) -> tuple[bool, str]:
        """Verify all required MoveIt services are available.

        Returns (ok, error_code_string).  Called at the start of every
        planning request so that a missing ``move_group`` is detected
        immediately (~1 s) rather than surfacing as a misleading
        ``GOAL_IK_FAILED`` after a 5 s timeout.
        """
        checks = [
            (self._ik_check_client, "IK_SERVICE_UNAVAILABLE"),
            (self._validity_check_client, "STATE_VALIDITY_SERVICE_UNAVAILABLE"),
            (self._scene_check_client, "PLANNING_SCENE_SERVICE_UNAVAILABLE"),
            (self._apply_scene_client, "APPLY_PLANNING_SCENE_SERVICE_UNAVAILABLE"),
            (self._plan_check_client, "MOTION_PLAN_SERVICE_UNAVAILABLE"),
        ]
        for client, error_code in checks:
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error(
                    f"MoveIt service check failed: {error_code}"
                )
                return False, error_code
        self.get_logger().info("MOVEIT_SERVICES_READY")
        return True, ""

    def _plan_callback(self, request, response):
        """Handle one /plan_transition_once request."""
        if self._planning:
            response.success = False
            response.message = _format_result("PLANNING_ALREADY_RUNNING", 0, 0.0)
            return response

        self._planning = True
        try:
            result_msg = self._execute_plan()
            # Parse error_code to determine success (Issue #6).
            code = result_msg.split("|")[0].split("=", 1)[-1] if "|" in result_msg else ""
            response.success = code in SUCCESS_CODES
            response.message = result_msg
        except Exception as exc:
            # rclpy's logger does not support the stdlib ``exc_info`` option.
            # Keep the service alive and return a structured failure instead.
            self.get_logger().error(f"Planning failed: {exc}")
            response.success = False
            response.message = _format_result(
                f"UNEXPECTED_ERROR", 0, 0.0, f"detail={exc}"
            )
        finally:
            self._planning = False
        return response

    def _execute_plan(self) -> str:
        plan_start = monotonic()
        topic = str(self.get_parameter("joint_state_topic").value)
        timeout = float(self.get_parameter("joint_state_timeout_s").value)
        max_age = float(self.get_parameter("max_joint_state_age_s").value)
        allow_fb = bool(self.get_parameter("allow_joint_state_fallback").value)

        # 0. MoveIt service health check (fail-fast before any IK call).
        services_ok, service_err = self._check_moveit_services()
        if not services_ok:
            return _format_result(service_err, 0, monotonic() - plan_start)

        # 1. Get current joint state (Issue #3: no nested spin).
        try:
            start_state = self._wait_for_joint_state(topic, timeout, max_age, allow_fb)
        except RuntimeError as e:
            return _format_result(str(e), 0, monotonic() - plan_start)

        self.get_logger().info("START_STATE_RECEIVED")

        # 2. Synchronize and verify the PlanningScene before attempting a
        # collision-aware IK request. This is deliberately fail-closed.
        scene_timeout = float(self.get_parameter("scene_sync_timeout_s").value)
        pos_tol = float(self.get_parameter("scene_position_tolerance_m").value)
        dim_tol = float(self.get_parameter("scene_dimension_tolerance_m").value)
        ori_tol = float(self.get_parameter("scene_orientation_tolerance").value)
        scene_ok, scene_err = self._synchronize_scene(
            timeout_s=scene_timeout,
            pos_tol=pos_tol,
            dim_tol=dim_tol,
            ori_tol=ori_tol,
        )
        if not scene_ok:
            return _format_result(scene_err, 0, monotonic() - plan_start)

        # 3. Load the first target and make the collision-aware IK request.
        traj_file = _resolve_path(
            str(self.get_parameter("trajectory_mat").value), "data/nurbs/ik_input.mat"
        )
        offset = tuple(float(v) for v in self.get_parameter("trajectory_offset_m").value)
        max_pts = int(self.get_parameter("max_points").value)
        stride = int(self.get_parameter("point_stride").value)

        try:
            positions, _ = load_first_task_target(traj_file, offset, max_pts, stride)
        except Exception as e:
            return _format_result("TRAJECTORY_LOAD_ERROR", 0, monotonic() - plan_start,
                                  f"detail={e}")

        align_surface = bool(self.get_parameter("align_tool_x_to_surface_normal").value)
        cylinder_axis = tuple(float(v) for v in
                              self.get_parameter("cylinder_axis_direction").value)
        orientation_xyzw = tuple(float(v) for v in
                                 self.get_parameter("orientation_xyzw").value)

        first_quat, per_point = compute_first_task_orientation(
            positions,
            align_tool_x_to_surface_normal=align_surface,
            cylinder_axis_direction=cylinder_axis,
            orientation_xyzw=orientation_xyzw,
            trajectory_mat=traj_file,
            offset_m=offset,
        )

        self.get_logger().info(
            f"FIRST_TARGET_POSITION={[f'{v:.4f}' for v in positions[0]]}"
        )
        self.get_logger().info(
            f"FIRST_TARGET_ORIENTATION=[{', '.join(f'{v:.6f}' for v in first_quat)}]"
        )

        try:
            first_goal = solve_first_task_state(
                moveit=self._moveit,
                joint_names=self._joint_names,
                tool_link=self._tool_link,
                positions=positions,
                start_state=start_state,
                first_orientation=first_quat,
                per_point_orientations=per_point,
                max_joint_delta=float(self.get_parameter("max_joint_delta").value),
                ik_service_timeout_s=float(self.get_parameter("ik_service_timeout_s").value),
                base_frame=self._base_frame,
                planning_group=self._planning_group,
                logger=self.get_logger(),
            )
        except IKServiceUnavailable as e:
            return _format_result("IK_SERVICE_UNAVAILABLE", 0, monotonic() - plan_start,
                                  f"detail={e}")
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

        self.get_logger().info("GOAL_IK_SUCCEEDED")

        # 4. Validate start and goal states (fail-closed).
        try:
            self._planner.validate_state(start_state, label="START_STATE")
        except StateValidityError as e:
            return _format_result(str(e).split(":")[0], 0, monotonic() - plan_start,
                                  f"detail={e}")
        except PlanningError as e:
            return _format_result("STATE_VALIDITY_SERVICE_UNAVAILABLE", 0,
                                  monotonic() - plan_start, f"detail={e}")

        try:
            self._planner.validate_state(first_goal, label="GOAL_STATE")
        except StateValidityError as e:
            return _format_result(str(e).split(":")[0], 0, monotonic() - plan_start,
                                  f"detail={e}")
        except PlanningError as e:
            return _format_result("STATE_VALIDITY_SERVICE_UNAVAILABLE", 0,
                                  monotonic() - plan_start, f"detail={e}")

        # 5. Plan with MoveIt's configured OMPL pipeline.
        try:
            transition = self._planner.plan_transition(start_state, first_goal)
        except PlanningError as e:
            return _format_result("PLANNER_FAILED", 0, monotonic() - plan_start,
                                  f"detail={e}")

        elapsed = monotonic() - plan_start
        mode = str(self.get_parameter("transition_result_mode").value)

        self.get_logger().info(
            f"TRANSITION_PLANNED: points={len(transition.points)}, "
            f"time={elapsed:.3f}s, planner={self._planner._options.planner_id}"
        )

        if mode == "plan_only":
            return _format_result("TRANSITION_PLANNED", len(transition.points), elapsed)

        if mode == "joint_state_replay":
            # Switch Viewer to tracking mode (Issue #7: fail on switch error).
            self.get_logger().info("Switching Viewer to ROS tracking for replay...")
            try:
                self._executor.replay(
                    transition,
                    topic=str(self.get_parameter("replay_joint_state_topic").value),
                    rate_hz=float(self.get_parameter("replay_rate_hz").value),
                    switch_viewer_to_tracking=True,
                )
            except ExecutionError as e:
                error_code = str(e).split(":", 1)[0]
                return _format_result(
                    error_code,
                    len(transition.points),
                    elapsed,
                    f"detail={e}",
                )
            self.get_logger().info("TRANSITION_REPLAYED")
            return _format_result("TRANSITION_REPLAYED", len(transition.points), elapsed)

        if mode == "moveit_execute":
            result = self._executor.execute(transition, dry_run=False, wait=True)
            if result.succeeded:
                return _format_result("TRANSITION_EXECUTED", len(transition.points), elapsed)
            return _format_result("TRANSITION_EXECUTION_FAILED", len(transition.points), elapsed)

        return _format_result("TRANSITION_PLANNED", len(transition.points), elapsed)

    def _ik_failure_context(
        self,
        *,
        error: Exception,
        moveit_error_code: int | None,
        position: Sequence[float],
        orientation: Sequence[float],
        seed: JointState,
    ) -> str:
        """Return machine-readable context for every failed goal IK request."""
        seed_positions = ",".join(f"{float(value):.6f}" for value in seed.position)
        return "|".join(
            [
                f"detail={error}",
                f"moveit_error_code={moveit_error_code if moveit_error_code is not None else 'NO_RESPONSE'}",
                "position=" + ",".join(f"{float(value):.6f}" for value in position),
                "orientation=" + ",".join(f"{float(value):.6f}" for value in orientation),
                f"planning_group={self._planning_group}",
                f"base_frame={self._base_frame}",
                f"tool_link={self._tool_link}",
                "seed_names=" + ",".join(seed.name),
                f"seed_positions={seed_positions}",
                "avoid_collisions=true",
                f"timeout={float(self.get_parameter('ik_service_timeout_s').value):.3f}",
            ]
        )

    # ------------------------------------------------------------------
    # Static registry -> ApplyPlanningScene -> full scene verification
    # ------------------------------------------------------------------

    def _install_static_scene_timer(self) -> None:
        """Eagerly install a complete retained static registry once.

        This lets operators inspect the PlanningScene immediately after
        startup. A failed or incomplete timer attempt is simply retried; no
        planning request can proceed until ``_synchronize_scene`` performs its
        own complete, fail-closed check.
        """
        if self._planning:
            return
        revision = self._static_registry.revision
        if revision == self._static_scene_applied_revision:
            return

        pos_tol = float(self.get_parameter("scene_position_tolerance_m").value)
        dim_tol = float(self.get_parameter("scene_dimension_tolerance_m").value)
        ori_tol = float(self.get_parameter("scene_orientation_tolerance").value)
        registry_error = self._validate_static_registry(pos_tol, dim_tol, ori_tol)
        if registry_error:
            return

        if not self._static_scene_apply_lock.acquire(blocking=False):
            return
        try:
            applied, _ = self._apply_static_scene(monotonic() + 0.5)
            if applied:
                self._static_scene_applied_revision = revision
        finally:
            self._static_scene_apply_lock.release()

    def _synchronize_scene(
        self,
        *,
        timeout_s: float,
        pos_tol: float,
        dim_tol: float,
        ori_tol: float,
    ) -> tuple[bool, str]:
        """Install retained static objects and verify the complete scene.

        Static messages are intentionally not sent to MoveIt's volatile
        ``/collision_object`` subscription.  Waiting for the retained registry
        makes startup deterministic; applying the exact registry snapshot via
        the service avoids a durability mismatch and allows late Viewer/server
        start without duplicated geometry.
        """
        deadline = monotonic() + timeout_s
        while monotonic() < deadline:
            registry_error = self._validate_static_registry(pos_tol, dim_tol, ori_tol)
            if registry_error == "STATIC_REGISTRY_PENDING":
                sleep(0.02)
                continue
            if registry_error:
                return False, registry_error

            with self._static_scene_apply_lock:
                applied, apply_error = self._apply_static_scene(deadline)
            if not applied:
                return False, apply_error
            self._static_scene_applied_revision = self._static_registry.revision

            return self._verify_scene_content(
                set_timeout=max(0.0, deadline - monotonic()),
                pos_tol=pos_tol,
                dim_tol=dim_tol,
                ori_tol=ori_tol,
            )

        return False, "SCENE_SYNC_TIMEOUT"

    def _validate_static_registry(
        self,
        pos_tol: float,
        dim_tol: float,
        ori_tol: float,
    ) -> str:
        """Check that every configured static object was received intact."""
        for spec in self._static_obstacle_specs:
            entry = self._static_registry.active_object(spec.object_id)
            if entry is None:
                return "STATIC_REGISTRY_PENDING"
            error = self._validate_entry_against_spec(
                entry, spec, pos_tol, dim_tol, ori_tol
            )
            if error:
                return error
        return ""

    def _apply_static_scene(self, deadline: float) -> tuple[bool, str]:
        """Apply the verified static registry as one atomic planning-scene diff."""
        remaining_s = deadline - monotonic()
        if remaining_s <= 0.0:
            return False, "SCENE_SYNC_TIMEOUT"
        if not self._apply_scene_client.wait_for_service(
            timeout_sec=min(0.5, remaining_s)
        ):
            return False, "APPLY_PLANNING_SCENE_SERVICE_UNAVAILABLE"

        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.world.collision_objects = [
            self._collision_object_from_spec(spec)
            for spec in self._static_obstacle_specs
        ]
        future = self._apply_scene_client.call_async(request)
        while not future.done() and monotonic() < deadline:
            sleep(0.01)
        if not future.done():
            return False, "APPLY_PLANNING_SCENE_TIMEOUT"
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"ApplyPlanningScene request failed: {exc}")
            return False, "APPLY_PLANNING_SCENE_FAILED"
        if response is None or not response.success:
            return False, "APPLY_PLANNING_SCENE_FAILED"
        self.get_logger().info(
            f"STATIC_SCENE_APPLIED: {len(self._static_obstacle_specs)} object(s)"
        )
        return True, ""

    @staticmethod
    def _collision_object_from_spec(spec: CollisionObjectSpec) -> CollisionObject:
        """Build an ADD message suitable for an ApplyPlanningScene diff."""
        message = CollisionObject()
        message.id = spec.object_id
        message.operation = CollisionObject.ADD
        message.header.frame_id = spec.frame_id
        message.pose = Pose()
        message.pose.position.x = float(spec.position[0])
        message.pose.position.y = float(spec.position[1])
        message.pose.position.z = float(spec.position[2])
        message.pose.orientation.x = float(spec.quaternion_xyzw[0])
        message.pose.orientation.y = float(spec.quaternion_xyzw[1])
        message.pose.orientation.z = float(spec.quaternion_xyzw[2])
        message.pose.orientation.w = float(spec.quaternion_xyzw[3])
        primitive = SolidPrimitive()
        primitive.type = {
            "box": SolidPrimitive.BOX,
            "sphere": SolidPrimitive.SPHERE,
            "cylinder": SolidPrimitive.CYLINDER,
        }[spec.shape]
        primitive.dimensions = [float(value) for value in spec.dimensions]
        message.primitives = [primitive]
        return message

    @staticmethod
    def _validate_entry_against_spec(
        entry: ObstacleEntry,
        spec: CollisionObjectSpec,
        pos_tol: float,
        dim_tol: float,
        ori_tol: float,
    ) -> str:
        """Validate a retained-registry entry before applying it to MoveIt."""
        if entry.shape != spec.shape:
            return "STATIC_REGISTRY_TYPE_MISMATCH"
        if entry.frame_id != spec.frame_id:
            return "STATIC_REGISTRY_FRAME_MISMATCH"
        if len(entry.dimensions) != len(spec.dimensions) or any(
            abs(actual - expected) > dim_tol
            for actual, expected in zip(entry.dimensions, spec.dimensions)
        ):
            return "STATIC_REGISTRY_SIZE_MISMATCH"
        if any(
            abs(actual - expected) > pos_tol
            for actual, expected in zip(entry.position, spec.position)
        ):
            return "STATIC_REGISTRY_POSE_MISMATCH"
        diff = sqrt(
            sum(
                (actual - expected) ** 2
                for actual, expected in zip(entry.quaternion_xyzw, spec.quaternion_xyzw)
            )
        )
        inverse_diff = sqrt(
            sum(
                (actual + expected) ** 2
                for actual, expected in zip(entry.quaternion_xyzw, spec.quaternion_xyzw)
            )
        )
        if min(diff, inverse_diff) > ori_tol:
            return "STATIC_REGISTRY_ORIENTATION_MISMATCH"
        return ""

    # ------------------------------------------------------------------
    #  Issue #3: no nested rclpy.spin_once / spin_until_future_complete
    # ------------------------------------------------------------------

    def _wait_for_joint_state(self, topic: str, timeout_s: float, max_age_s: float,
                               allow_fallback: bool) -> JointState:
        """Wait for a fresh JointState on *topic* using threading.Event.

        The MultiThreadedExecutor dispatches the subscription callback in a
        background thread, so we only need to block until the Event is set.
        """
        from threading import Event
        names_set = set(self._joint_names)
        received = Event()
        latest: JointState | None = None

        def _cb(msg: JointState) -> None:
            nonlocal latest
            if names_set.issubset(msg.name):
                latest = msg
                received.set()

        sub = self.create_subscription(JointState, topic, _cb, qos_profile_sensor_data)
        try:
            if not received.wait(timeout_s):
                if not allow_fallback:
                    raise RuntimeError("START_STATE_UNAVAILABLE")
                return self._fallback_joint_state(max_age_s)

            stamp_sec = latest.header.stamp.sec + latest.header.stamp.nanosec * 1e-9
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            age_s = now_sec - stamp_sec
            if age_s > max_age_s:
                if not allow_fallback:
                    raise RuntimeError("START_STATE_UNAVAILABLE")
                return self._fallback_joint_state(max_age_s)

            positions = dict(zip(latest.name, latest.position))
            if len(latest.position) != len(latest.name):
                raise RuntimeError("START_STATE_INCOMPLETE")
            result = JointState()
            result.header = latest.header
            result.name = list(self._joint_names)
            result.position = [float(positions[n]) for n in self._joint_names]
            if not all(isfinite(float(p)) for p in result.position):
                raise RuntimeError("START_STATE_NON_FINITE")
            self.get_logger().info(
                f"Planning start state source: {topic}; age={age_s:.3f}s"
            )
            return result
        finally:
            self.destroy_subscription(sub)

    def _fallback_joint_state(self, max_age_s: float) -> JointState:
        names_set = set(self._joint_names)
        state = self._moveit.joint_state
        if state is None or not names_set.issubset(state.name):
            raise RuntimeError("START_STATE_UNAVAILABLE")
        stamp_sec = state.header.stamp.sec + state.header.stamp.nanosec * 1e-9
        if stamp_sec == 0.0:
            raise RuntimeError("START_STATE_STALE")
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - stamp_sec > max_age_s:
            raise RuntimeError("START_STATE_STALE")
        positions = dict(zip(state.name, state.position))
        result = JointState()
        result.header = state.header
        result.name = list(self._joint_names)
        result.position = [float(positions[n]) for n in self._joint_names]
        self.get_logger().warning(
            f"WARNING: start state using MoveIt fallback, not mujoco topic"
        )
        return result

    # ------------------------------------------------------------------
    #  Issue #10: complete PlanningScene verification
    # ------------------------------------------------------------------

    def _verify_scene_content(self, set_timeout: float, pos_tol: float,
                               dim_tol: float, ori_tol: float) -> tuple[bool, str]:
        """Check that PlanningScene contains expected objects with correct data.

        Validates: ID, primitive type, dimensions, position, orientation,
        and confirms REMOVE operations have taken effect (Issue #10).
        """
        scene_client = self._planner._planning_scene_client
        if not scene_client.wait_for_service(timeout_sec=2.0):
            return False, "PLANNING_SCENE_SERVICE_UNAVAILABLE"

        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY

        deadline = monotonic() + set_timeout
        while monotonic() < deadline:
            future = scene_client.call_async(request)
            # Non-spin polling (Issue #3).
            while not future.done() and monotonic() < deadline:
                sleep(0.01)
            if not future.done():
                continue
            response = future.result()
            if response is None:
                continue

            scene_objects = {
                obj.id: obj for obj in response.scene.world.collision_objects
            }

            # Check REMOVE pending (Issue #10.4).
            for removed_id in self._registry.pending_removals:
                if removed_id in scene_objects:
                    return False, "SCENE_OBJECT_REMOVE_PENDING"
                else:
                    self._registry._clear_removal(removed_id)

            # Check static obstacles with full validation.
            for spec in self._static_obstacle_specs:
                obj = scene_objects.get(spec.object_id)
                if obj is None:
                    # Missing — will retry
                    break

                err = self._validate_scene_object(obj, spec, pos_tol, dim_tol, ori_tol)
                if err:
                    break
            else:
                # All static objects validated.

                # Check dynamic objects with full validation.
                for entry in self._registry.active_objects():
                    obj = scene_objects.get(entry.object_id)
                    if obj is None:
                        break

                    err = self._validate_scene_object_against_entry(
                        obj, entry, pos_tol, dim_tol, ori_tol
                    )
                    if err:
                        break
                else:
                    # All checks passed.
                    static_ids = {s.object_id for s in self._static_obstacle_specs}
                    missing = static_ids - set(scene_objects.keys())
                    if missing:
                        sleep(0.05)
                        continue

                    self.get_logger().info(
                        f"SCENE_SYNCED: {len(scene_objects)} object(s) in PlanningScene, "
                        f"revision={self._registry.revision}"
                    )
                    return True, ""

            sleep(0.05)

        return False, "SCENE_SYNC_TIMEOUT"

    def _validate_scene_object(self, obj, spec: CollisionObjectSpec,
                                pos_tol: float, dim_tol: float,
                                ori_tol: float) -> str:
        """Validate one scene object against a static spec. Returns "" if OK."""
        # Primitive type check.
        if not obj.primitives:
            return "SCENE_OBJECT_TYPE_MISMATCH"
        prim = obj.primitives[0]
        shape_map = {1: "box", 2: "sphere", 3: "cylinder"}  # SolidPrimitive enum
        prim_type_raw = prim.type
        if isinstance(prim_type_raw, bytes):
            prim_type_raw = int.from_bytes(prim_type_raw, "little")
        actual_shape = shape_map.get(int(prim_type_raw), "unknown")
        if actual_shape != spec.shape:
            return "SCENE_OBJECT_TYPE_MISMATCH"

        # Dimensions check.
        expected_dims = {"box": 3, "sphere": 1, "cylinder": 2}
        nd = expected_dims.get(spec.shape, 0)
        for i in range(nd):
            if abs(float(prim.dimensions[i]) - spec.dimensions[i]) > dim_tol:
                return "SCENE_OBJECT_SIZE_MISMATCH"

        # Position check.
        if abs(obj.pose.position.x - spec.position[0]) > pos_tol:
            return "SCENE_OBJECT_POSE_MISMATCH"
        if abs(obj.pose.position.y - spec.position[1]) > pos_tol:
            return "SCENE_OBJECT_POSE_MISMATCH"
        if abs(obj.pose.position.z - spec.position[2]) > pos_tol:
            return "SCENE_OBJECT_POSE_MISMATCH"

        # Orientation check (q and -q equivalence, Issue #10.3).
        q_expected = spec.quaternion_xyzw
        q_actual = (
            obj.pose.orientation.x,
            obj.pose.orientation.y,
            obj.pose.orientation.z,
            obj.pose.orientation.w,
        )
        diff1 = sqrt(sum((a - b) ** 2 for a, b in zip(q_expected, q_actual)))
        diff2 = sqrt(sum((a + b) ** 2 for a, b in zip(q_expected, q_actual)))
        if min(diff1, diff2) > ori_tol:
            return "SCENE_OBJECT_ORIENTATION_MISMATCH"

        return ""

    def _validate_scene_object_against_entry(self, obj, entry: ObstacleEntry,
                                              pos_tol: float, dim_tol: float,
                                              ori_tol: float) -> str:
        """Validate one scene object against a registry entry. Returns "" if OK."""
        if not obj.primitives:
            return "SCENE_OBJECT_TYPE_MISMATCH"
        prim = obj.primitives[0]
        shape_map = {1: "box", 2: "sphere", 3: "cylinder"}
        prim_type_raw = prim.type
        if isinstance(prim_type_raw, bytes):
            prim_type_raw = int.from_bytes(prim_type_raw, "little")
        actual_shape = shape_map.get(int(prim_type_raw), "unknown")
        if actual_shape != entry.shape:
            return "SCENE_OBJECT_TYPE_MISMATCH"

        # Position check.
        if abs(obj.pose.position.x - entry.position[0]) > pos_tol:
            return "SCENE_OBJECT_POSE_MISMATCH"
        if abs(obj.pose.position.y - entry.position[1]) > pos_tol:
            return "SCENE_OBJECT_POSE_MISMATCH"
        if abs(obj.pose.position.z - entry.position[2]) > pos_tol:
            return "SCENE_OBJECT_POSE_MISMATCH"

        # Orientation check.
        q_expected = entry.quaternion_xyzw
        q_actual = (
            obj.pose.orientation.x,
            obj.pose.orientation.y,
            obj.pose.orientation.z,
            obj.pose.orientation.w,
        )
        diff1 = sqrt(sum((a - b) ** 2 for a, b in zip(q_expected, q_actual)))
        diff2 = sqrt(sum((a + b) ** 2 for a, b in zip(q_expected, q_actual)))
        if min(diff1, diff2) > ori_tol:
            return "SCENE_OBJECT_ORIENTATION_MISMATCH"

        return ""

def main(args: Sequence[str] | None = None) -> int:
    rclpy.init(args=args)
    node = TransitionPlanningServer()
    # The Trigger callback waits for subscriptions and MoveIt futures, so use
    # an explicit worker count rather than inheriting a one-core affinity.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
