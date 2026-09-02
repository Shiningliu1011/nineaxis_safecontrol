"""Full-phase unit tests for the pure-logic transition executor.

Every phase of :class:`TransitionExecutor` — health check, start state, goal
IK, fail-closed validity, planning, replay/execute, and the autonomous retry
loop — is exercised through fake ports, with no ROS graph required.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

# Ensure the source tree is importable.
_src = Path(__file__).resolve().parents[1] / "src"
if str(_src) not in sys.path:
    sys.path.insert(0, str(_src))

from sensor_msgs.msg import JointState

from robot_safecontrol_moveit.continuous_ik import (
    IKError,
    IKFailure,
    IKServiceUnavailable,
)
from robot_safecontrol_moveit.motion_planning import PlanningError, StateValidityError
from robot_safecontrol_moveit.trajectory_execution import ExecutionError
from robot_safecontrol_moveit.transition_executor import (
    AutoPlanLoop,
    TransitionExecutor,
)
import robot_safecontrol_moveit.transition_executor as transition_executor


def _result_code(result: str) -> str:
    fields = dict(item.split("=", 1) for item in result.split("|") if "=" in item)
    return fields["error_code"]


def _joint_state(names=("J1", "J2"), positions=(0.1, -0.2)) -> JointState:
    state = JointState()
    state.name = list(names)
    state.position = [float(value) for value in positions]
    return state


class _FakeLogger:
    def __init__(self):
        self.messages: list[tuple[str, str]] = []

    def info(self, message, *args, **kwargs):
        self.messages.append(("info", str(message)))

    def warn(self, message, *args, **kwargs):
        self.messages.append(("warn", str(message)))

    def error(self, message, *args, **kwargs):
        self.messages.append(("error", str(message)))


class _Ports:
    """Configurable fake of :class:`TransitionPorts`."""

    DEFAULTS = {
        "joint_state_topic": "/mujoco_joint_states",
        "joint_state_timeout_s": 1.0,
        "max_joint_state_age_s": 0.5,
        "allow_joint_state_fallback": False,
        "trajectory_mat": "",
        "trajectory_offset_m": (0.0, 0.0, 0.0),
        "max_points": 1,
        "point_stride": 1,
        "align_tool_x_to_surface_normal": False,
        "cylinder_axis_direction": (0.0, 1.0, 0.0),
        "orientation_xyzw": (0.0, 0.0, 0.0, 1.0),
        "max_joint_delta": 0.25,
        "ik_service_timeout_s": 1.0,
        "transition_result_mode": "plan_only",
        "replay_joint_state_topic": "/mujoco_joint_states",
        "replay_rate_hz": 30.0,
        "replay_time_scale": 1.5,
        "replay_min_duration_s": 4.0,
        "oscbf_command_topic": "",
        "notify_oscbf_start": False,
    }

    def __init__(self, **overrides):
        self._params = dict(self.DEFAULTS)
        self._params.update(overrides)
        self.events: list[str] = []
        self.log = _FakeLogger()
        self.start_state = _joint_state()
        self.goal_state = _joint_state(positions=(0.3, -0.1))
        self.trajectory = SimpleNamespace(points=[object(), object()])
        self.planner_id = "AEBRRTstarFaithfulConfigDefault"
        self.planning_group = "arm"
        self.base_frame = "base_link"
        self.tool_link = "tool0"
        # Failure switches.
        self.services_error: str | None = None
        self.joint_state_error: RuntimeError | None = None
        self.solve_error: Exception | None = None
        self.start_validity_error: Exception | None = None
        self.goal_validity_error: Exception | None = None
        self.plan_error: Exception | None = None
        self.replay_error: Exception | None = None
        self.execute_result = SimpleNamespace(succeeded=True)
        self.notify_code = "TRACKING_STARTED"

    def get_parameter(self, name):
        return SimpleNamespace(value=self._params[name])

    def check_moveit_services(self):
        self.events.append("services")
        if self.services_error is not None:
            return False, self.services_error
        return True, ""

    def wait_for_joint_state(self, *args):
        self.events.append("joint_state")
        if self.joint_state_error is not None:
            raise self.joint_state_error
        return self.start_state

    def solve_task_state(self, **kwargs):
        self.events.append("ik")
        if self.solve_error is not None:
            raise self.solve_error
        return self.goal_state

    def validate_state(self, state, *, label):
        self.events.append(f"validate:{label}")
        if label == "START_STATE" and self.start_validity_error is not None:
            raise self.start_validity_error
        if label == "GOAL_STATE" and self.goal_validity_error is not None:
            raise self.goal_validity_error

    def plan_transition(self, start_state, goal_state):
        self.events.append("plan")
        if self.plan_error is not None:
            raise self.plan_error
        return self.trajectory

    def replay(self, trajectory, *, topic, rate_hz, command_topic, time_scale=1.0, min_duration_s=0.0):
        self.events.append("replay")
        if self.replay_error is not None:
            raise self.replay_error

    def execute(self, trajectory):
        self.events.append("execute")
        return self.execute_result

    def notify_oscbf_start(self):
        self.events.append("notify")
        return self.notify_code

    def wait_for_plant_settle(self, transition):
        self.events.append("settle")


def _patch_task_inputs():
    """Replace the trajectory-loading helpers; keep the phase machine real."""
    return (
        patch.object(
            transition_executor,
            "_resolve_path",
            return_value=Path("/tmp/ik_input.mat"),
        ),
        patch.object(
            transition_executor,
            "load_first_task_target",
            side_effect=lambda *args, **kwargs: ([(0.4, 0.2, 0.8)], [0.0]),
        ),
    )


class TestPhaseMachineFailures(unittest.TestCase):
    def _run(self, ports):
        patches = _patch_task_inputs()
        with patches[0], patches[1]:
            return TransitionExecutor(ports).execute_plan()

    def test_services_unavailable_fails_fast(self):
        ports = _Ports()
        ports.services_error = "IK_SERVICE_UNAVAILABLE"
        result = self._run(ports)
        self.assertEqual(_result_code(result), "IK_SERVICE_UNAVAILABLE")
        self.assertEqual(ports.events, ["services"])

    def test_joint_state_unavailable(self):
        ports = _Ports()
        ports.joint_state_error = RuntimeError("START_STATE_UNAVAILABLE")
        result = self._run(ports)
        self.assertEqual(_result_code(result), "START_STATE_UNAVAILABLE")
        self.assertNotIn("ik", ports.events)

    def test_trajectory_load_error(self):
        ports = _Ports()
        with patch.object(
            transition_executor,
            "_resolve_path",
            return_value=Path("/tmp/ik_input.mat"),
        ), patch.object(
            transition_executor,
            "load_first_task_target",
            side_effect=ValueError("not a supported ik_input.mat file"),
        ):
            result = TransitionExecutor(ports).execute_plan()
        self.assertEqual(_result_code(result), "TRAJECTORY_LOAD_ERROR")
        self.assertIn("detail=", result)

    def test_ik_service_unavailable(self):
        ports = _Ports()
        ports.solve_error = IKServiceUnavailable()
        result = self._run(ports)
        self.assertEqual(_result_code(result), "IK_SERVICE_UNAVAILABLE")
        self.assertNotIn("plan", ports.events)

    def test_ik_failure_with_moveit_code_reports_goal_ik_failed_with_context(self):
        ports = _Ports()
        ports.solve_error = IKFailure(0, (0.4, 0.2, 0.8), moveit_error_code=-31)
        result = self._run(ports)
        self.assertEqual(_result_code(result), "GOAL_IK_FAILED")
        self.assertIn("moveit_error_code=-31", result)
        self.assertIn("planning_group=arm", result)
        self.assertIn("base_frame=base_link", result)
        self.assertIn("tool_link=tool0", result)
        self.assertIn("seed_names=J1,J2", result)
        self.assertIn("avoid_collisions=true", result)

    def test_ik_failure_without_moveit_code_reports_timeout(self):
        ports = _Ports()
        ports.solve_error = IKError("no response")
        result = self._run(ports)
        self.assertEqual(_result_code(result), "IK_RESPONSE_TIMEOUT")
        self.assertIn("moveit_error_code=NO_RESPONSE", result)

    def test_goal_state_collision(self):
        ports = _Ports()
        ports.goal_validity_error = StateValidityError("GOAL_STATE_COLLISION: hit")
        result = self._run(ports)
        self.assertEqual(_result_code(result), "GOAL_STATE_COLLISION")
        self.assertNotIn("plan", ports.events)

    def test_validity_service_unavailable(self):
        ports = _Ports()
        ports.goal_validity_error = PlanningError("check_state_validity timed out")
        result = self._run(ports)
        self.assertEqual(_result_code(result), "STATE_VALIDITY_SERVICE_UNAVAILABLE")

    def test_planner_failed(self):
        ports = _Ports()
        ports.plan_error = PlanningError("PLANNER_FAILED: no solution")
        result = self._run(ports)
        self.assertEqual(_result_code(result), "PLANNER_FAILED")

    def test_unknown_result_mode_falls_back_to_planned(self):
        ports = _Ports(transition_result_mode="bogus_mode")
        result = self._run(ports)
        self.assertEqual(_result_code(result), "TRANSITION_PLANNED")
        self.assertNotIn("replay", ports.events)
        self.assertNotIn("execute", ports.events)


class TestPhaseMachineSuccessModes(unittest.TestCase):
    def _run(self, ports):
        patches = _patch_task_inputs()
        with patches[0], patches[1]:
            return TransitionExecutor(ports).execute_plan()

    def test_plan_only_success(self):
        ports = _Ports()
        result = self._run(ports)
        self.assertEqual(_result_code(result), "TRANSITION_PLANNED")
        self.assertIn("trajectory_points=2", result)
        self.assertEqual(
            ports.events,
            ["services", "joint_state", "ik",
             "validate:START_STATE", "validate:GOAL_STATE", "plan"],
        )

    def test_replay_with_notify_settles_before_notifying(self):
        ports = _Ports(
            transition_result_mode="joint_state_replay",
            notify_oscbf_start=True,
        )
        result = self._run(ports)
        self.assertEqual(_result_code(result), "TRANSITION_REPLAYED")
        self.assertEqual(ports.events[-3:], ["replay", "settle", "notify"])
        self.assertIn(("info", "OSCBF_START_NOTIFY_RESULT=TRACKING_STARTED"),
                      ports.log.messages)

    def test_replay_error_keeps_its_structured_code(self):
        ports = _Ports(transition_result_mode="joint_state_replay")
        ports.replay_error = ExecutionError("VIEWER_SWITCH_FAILED: switch refused")
        result = self._run(ports)
        self.assertEqual(_result_code(result), "VIEWER_SWITCH_FAILED")
        self.assertIn("detail=", result)

    def test_replay_without_notify_skips_settle_and_notify(self):
        ports = _Ports(transition_result_mode="joint_state_replay")
        result = self._run(ports)
        self.assertEqual(_result_code(result), "TRANSITION_REPLAYED")
        self.assertNotIn("settle", ports.events)
        self.assertNotIn("notify", ports.events)

    def test_moveit_execute_success(self):
        ports = _Ports(transition_result_mode="moveit_execute")
        result = self._run(ports)
        self.assertEqual(_result_code(result), "TRANSITION_EXECUTED")
        self.assertIn("execute", ports.events)

    def test_moveit_execute_failure(self):
        ports = _Ports(transition_result_mode="moveit_execute")
        ports.execute_result = SimpleNamespace(succeeded=False)
        result = self._run(ports)
        self.assertEqual(_result_code(result), "TRANSITION_EXECUTION_FAILED")


class TestAutoPlanLoop(unittest.TestCase):
    def _loop(self, attempts, succeed_after, *, planning=False,
              services_ok=True, oscbf_ok=True):
        calls = {"n": 0}
        logs = []
        log = _FakeLogger()
        log.messages = logs

        def is_planning():
            return planning

        def services_ready():
            return (True, "") if services_ok else (False, "MOTION_PLAN_SERVICE_UNAVAILABLE")

        def oscbf_ready():
            return oscbf_ok

        def plan_once():
            calls["n"] += 1
            success = calls["n"] >= succeed_after
            return success, "TRANSITION_REPLAYED" if success else "START_STATE_IN_COLLISION"

        def randomize():
            return "RANDOMIZED"

        return AutoPlanLoop(
            attempts=attempts,
            is_planning=is_planning,
            services_ready=services_ready,
            oscbf_ready=oscbf_ready,
            plan_once=plan_once,
            randomize_plant=randomize,
            log=log,
        ), calls, logs

    def test_succeeds_on_first_attempt(self):
        loop, calls, logs = self._loop(attempts=3, succeed_after=1)
        loop.tick()
        self.assertTrue(loop.done)
        self.assertEqual(loop.attempts_made, 1)
        self.assertEqual(calls["n"], 1)
        self.assertIn(("info", "AUTO_PLAN_SUCCEEDED (attempt 1)"), logs)

    def test_retries_then_reports_failure(self):
        loop, calls, logs = self._loop(attempts=2, succeed_after=99)
        loop.tick()
        self.assertFalse(loop.done)
        loop.tick()
        self.assertTrue(loop.done)
        self.assertEqual(calls["n"], 2)
        self.assertTrue(any(
            level == "warn" and "AUTO_PLAN_RETRY 1/2" in message
            for level, message in logs
        ))
        self.assertTrue(any(
            level == "error" and "AUTO_PLAN_FAILED after 2 attempts" in message
            for level, message in logs
        ))

    def test_skips_tick_while_a_plan_is_running(self):
        loop, calls, _ = self._loop(attempts=3, succeed_after=1, planning=True)
        loop.tick()
        self.assertEqual(loop.attempts_made, 0)
        self.assertEqual(calls["n"], 0)

    def test_waits_for_moveit_services(self):
        loop, calls, _ = self._loop(attempts=3, succeed_after=1, services_ok=False)
        loop.tick()
        self.assertEqual(calls["n"], 0)

    def test_waits_for_the_oscbf_start_service(self):
        loop, calls, _ = self._loop(attempts=3, succeed_after=1, oscbf_ok=False)
        loop.tick()
        self.assertEqual(calls["n"], 0)


if __name__ == "__main__":
    unittest.main()
