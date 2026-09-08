"""Persistent planning server: one MoveIt client, repeated /plan_transition_once.

Usage::

    ros2 run robot_safecontrol_moveit transition_planning_server

Triggers a full transition plan on each ``std_srvs/Trigger`` call to
``/plan_transition_once``.  The response message encodes a pipe-delimited
key=value result so callers can parse structured information::

    error_code=START_STATE_UNAVAILABLE|trajectory_points=0|planning_time=0.0

The phase machine itself lives in :mod:`transition_executor` (no ROS); this
node only adapts ROS side effects to its ports.
"""

from __future__ import annotations

import os
from functools import partial
from math import isfinite
from time import monotonic, sleep
from typing import Sequence

import numpy as np
import rclpy
from moveit_msgs.srv import (
    GetMotionPlan,
    GetPositionIK,
    GetStateValidity,
)
from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .motion_planning import MotionPlanner, PlanningOptions
from .robot_spec import DEFAULT_JOINT_NAMES
from .ros_conventions import JOINT_STATE_TOPIC
from .task_target import solve_first_task_state
from .trajectory_execution import TrajectoryExecutor
from .transition_executor import (
    SUCCESS_CODES,
    VALID_RESULT_MODES,
    AutoPlanLoop,
    TransitionExecutor,
    TransitionPorts,
    _format_result,
)


def notify_oscbf_start(
    node: Node,
    service_name: str,
    *,
    timeout_s: float = 2.0,
) -> str:
    """Tell the OSCBF controller to begin tracking; return its response code."""
    client = node.create_client(Trigger, service_name)
    try:
        if not client.wait_for_service(timeout_sec=timeout_s):
            return "START_SERVICE_UNAVAILABLE"
        future = client.call_async(Trigger.Request())
        deadline = monotonic() + timeout_s
        while not future.done() and monotonic() < deadline:
            sleep(0.01)
        if not future.done():
            return "START_SERVICE_TIMEOUT"
        result = future.result()
        if result is not None and result.success:
            return result.message
        return "START_SERVICE_FAILED"
    finally:
        node.destroy_client(client)


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

        # Planning service.
        self._planning = False
        self._pipeline = TransitionExecutor(self._build_ports())
        self._srv = self.create_service(
            Trigger, "/plan_transition_once", self._plan_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        # Persistent joint-state subscription (avoids create/destroy race
        # with the MultiThreadedExecutor — see Issue InvalidHandle).
        self._js_latest: JointState | None = None
        self._js_names_set = set(joint_names)
        self._js_event: "Event" = __import__("threading").Event()
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._persistent_js_cb,
            qos_profile_sensor_data,
        )

        # Health-check clients for MoveIt services (created once, reused).
        self._ik_check_client = self.create_client(
            GetPositionIK, "compute_ik"
        )
        self._validity_check_client = self.create_client(
            GetStateValidity, "check_state_validity"
        )
        self._plan_check_client = self.create_client(
            GetMotionPlan, "plan_kinematic_path"
        )

        # Autonomous experiment mode: plan one transition from the current
        # (possibly randomised) plant pose with no keyboard interaction.
        self._auto_plan_loop: AutoPlanLoop | None = None
        if bool(self.get_parameter("auto_plan_once").value):
            self._auto_plan_loop = AutoPlanLoop(
                attempts=int(self.get_parameter("auto_plan_attempts").value),
                is_planning=lambda: self._planning,
                services_ready=self._check_moveit_services,
                oscbf_ready=self._oscbf_start_service_ready,
                plan_once=self._plan_once,
                randomize_plant=self._randomize_plant_start,
                log=self.get_logger(),
            )
            # Reentrant group: the auto-plan callback blocks while waiting for
            # joint states / service replies, and must not stall the default
            # group's subscription and client callbacks (same pattern as the
            # /plan_transition_once service).
            self.create_timer(
                1.0,
                self._auto_plan_tick,
                callback_group=ReentrantCallbackGroup(),
            )

        self.get_logger().info(
            f"TRANSITION_SERVER_STARTED pid={os.getpid()} "
            f"config=mujoco_transition_runtime.yaml"
        )

        self.get_logger().info(
            "Transition planning server ready on /plan_transition_once"
        )

    def _persistent_js_cb(self, msg: JointState) -> None:
        """Persistent joint-state callback — never destroyed."""
        if self._js_names_set.issubset(msg.name):
            self._js_latest = msg
            self._js_event.set()

    def _wait_for_js_snapshot(self, timeout_s: float) -> JointState | None:
        """Block until a fresh joint state arrives (uses persistent sub)."""
        self._js_event.clear()
        self._js_event.wait(timeout_s)
        return self._js_latest

    def _build_ports(self) -> TransitionPorts:
        """Wire the phase machine to this node's ROS side effects."""
        log = self.get_logger()
        solve = partial(
            solve_first_task_state,
            moveit=self._moveit,
            joint_names=self._joint_names,
            tool_link=self._tool_link,
            base_frame=self._base_frame,
            planning_group=self._planning_group,
            logger=log,
        )
        return TransitionPorts(
            log=log,
            get_parameter=self.get_parameter,
            check_moveit_services=self._check_moveit_services,
            wait_for_joint_state=self._wait_for_joint_state,
            solve_task_state=solve,
            validate_state=self._planner.validate_state,
            plan_transition=self._planner.plan_transition,
            planner_id=str(self.get_parameter("planner_id").value),
            replay=self._replay_transition,
            execute=self._execute_transition,
            notify_oscbf_start=self._notify_oscbf_start,
            wait_for_plant_settle=self._wait_for_plant_settle,
            planning_group=self._planning_group,
            base_frame=self._base_frame,
            tool_link=self._tool_link,
        )

    def _replay_transition(
        self,
        transition,
        *,
        topic,
        rate_hz,
        command_topic,
        time_scale=1.0,
        min_duration_s=0.0,
    ):
        self._executor.replay(
            transition,
            topic=topic,
            rate_hz=rate_hz,
            command_topic=command_topic,
            time_scale=time_scale,
            min_duration_s=min_duration_s,
        )

    def _execute_transition(self, transition):
        return self._executor.execute(transition, dry_run=False, wait=True)

    def _notify_oscbf_start(self) -> str:
        return notify_oscbf_start(
            self, str(self.get_parameter("oscbf_start_service").value)
        )

    def _plan_once(self) -> tuple[bool, str]:
        """Run one plan through the service callback; return (success, message)."""
        request = Trigger.Request()
        response = Trigger.Response()
        self._plan_callback(request, response)
        return bool(response.success), str(response.message)

    def _auto_plan_tick(self) -> None:
        """Autonomous experiment tick; the retry policy lives in AutoPlanLoop."""
        assert self._auto_plan_loop is not None
        self._auto_plan_loop.tick()

    def _oscbf_start_service_ready(self) -> bool:
        """True when the controller's start service is discoverable."""
        if not bool(self.get_parameter("notify_oscbf_start").value):
            return True
        service = str(self.get_parameter("oscbf_start_service").value)
        client = self.create_client(Trigger, service)
        try:
            return client.service_is_ready()
        finally:
            self.destroy_client(client)

    def _randomize_plant_start(self) -> str:
        """Ask the plant to resample its start pose for the next attempt."""
        service = str(
            self.get_parameter("oscbf_plant_randomize_service").value
        )
        client = self.create_client(Trigger, service)
        try:
            if not client.wait_for_service(timeout_sec=2.0):
                return "PLANT_RANDOMIZE_UNAVAILABLE"
            future = client.call_async(Trigger.Request())
            deadline = monotonic() + 2.0
            while not future.done() and monotonic() < deadline:
                sleep(0.01)
            if not future.done():
                return "PLANT_RANDOMIZE_TIMEOUT"
            result = future.result()
            if result is not None and result.success:
                return result.message
            return "PLANT_RANDOMIZE_FAILED"
        finally:
            self.destroy_client(client)

    def _wait_for_plant_settle(
        self,
        transition,
        *,
        timeout_s: float = 5.0,
        tolerance: float = 0.01,
    ) -> None:
        """Wait until the plant converges to the final replayed waypoint.

        Uses the persistent joint-state subscription to avoid the
        create/destroy race with MultiThreadedExecutor (InvalidHandle).
        """
        target = np.asarray(transition.points[-1].positions, dtype=float)
        deadline = monotonic() + timeout_s
        while monotonic() < deadline:
            msg = self._wait_for_js_snapshot(0.05)
            if msg is not None:
                positions = dict(zip(msg.name, msg.position))
                q = np.asarray(
                    [positions[name] for name in self._joint_names],
                    dtype=float,
                )
                if np.max(np.abs(q - target)) < tolerance:
                    self.get_logger().info(
                        "PLANT_SETTLED: "
                        f"max|q-target|={float(np.max(np.abs(q - target))):.4f}"
                    )
                    return
        self.get_logger().warn(
            "PLANT_SETTLE_TIMEOUT: starting tracking anyway"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("joint_names", list(DEFAULT_JOINT_NAMES))
        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tool_link", "tool0")
        self.declare_parameter("trajectory_mat", "")
        self.declare_parameter("trajectory_offset_m", [0.0, 0.343, 1.587])
        self.declare_parameter("max_points", 1)
        self.declare_parameter("point_stride", 1)
        self.declare_parameter("joint_state_topic", JOINT_STATE_TOPIC)
        self.declare_parameter("joint_state_timeout_s", 3.0)
        self.declare_parameter("max_joint_state_age_s", 1.0)
        self.declare_parameter("allow_joint_state_fallback", False)
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
        self.declare_parameter("replay_joint_state_topic", JOINT_STATE_TOPIC)
        self.declare_parameter("replay_rate_hz", 100.0)
        # >1 时把过渡时间剖面按比例拉长, 大行程过渡不再被压成一闪而过的疾驰
        self.declare_parameter("replay_time_scale", 3.0)
        self.declare_parameter("replay_min_duration_s", 6.0)
        self.declare_parameter("oscbf_command_topic", "")
        self.declare_parameter("notify_oscbf_start", False)
        self.declare_parameter(
            "oscbf_start_service", "/oscbf_controller/start_tracking"
        )
        self.declare_parameter("auto_plan_once", False)
        self.declare_parameter("auto_plan_attempts", 5)
        self.declare_parameter(
            "oscbf_plant_randomize_service", "/oscbf_plant/randomize"
        )
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
            result_msg = self._pipeline.execute_plan()
            # Parse error_code to determine success (Issue #6).
            code = result_msg.split("|")[0].split("=", 1)[-1] if "|" in result_msg else ""
            response.success = code in SUCCESS_CODES
            response.message = result_msg
        except Exception as exc:
            # rclpy's logger does not support the stdlib ``exc_info`` option.
            # Keep the service alive and return a structured failure instead.
            import traceback
            self.get_logger().error(
                f"Planning failed: {exc}\n{traceback.format_exc()}"
            )
            response.success = False
            response.message = _format_result(
                f"UNEXPECTED_ERROR", 0, 0.0, f"detail={exc}"
            )
        finally:
            self._planning = False
        return response

    # ------------------------------------------------------------------
    #  Issue #3: no nested rclpy.spin_once / spin_until_future_complete
    # ------------------------------------------------------------------

    def _wait_for_joint_state(self, topic: str, timeout_s: float, max_age_s: float,
                               allow_fallback: bool) -> JointState:
        """Wait for a fresh JointState using the persistent subscription."""
        msg = self._wait_for_js_snapshot(timeout_s)
        if msg is None:
            if not allow_fallback:
                raise RuntimeError("START_STATE_UNAVAILABLE")
            return self._fallback_joint_state(max_age_s)

        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        age_s = now_sec - stamp_sec
        if age_s > max_age_s:
            if not allow_fallback:
                raise RuntimeError("START_STATE_UNAVAILABLE")
            return self._fallback_joint_state(max_age_s)

        positions = dict(zip(msg.name, msg.position))
        if len(msg.position) != len(msg.name):
            raise RuntimeError("START_STATE_INCOMPLETE")
        result = JointState()
        result.header = msg.header
        result.name = list(self._joint_names)
        result.position = [float(positions[n]) for n in self._joint_names]
        if not all(isfinite(float(p)) for p in result.position):
            raise RuntimeError("START_STATE_NON_FINITE")
        self.get_logger().info(
            f"Planning start state source: {topic}; age={age_s:.3f}s"
        )
        return result

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
