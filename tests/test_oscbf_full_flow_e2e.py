"""G5 acceptance: zero -> transition -> OSCBF tracking, end to end.

No MoveIt/move_group here: the plant and controller run in one process, a
transition command is injected exactly the way the planning server's replay
does, and the controller's start service gates the handoff.  This pins the
orchestration contract; the heavyweight MoveIt launch is verified separately.
"""

from __future__ import annotations

import os
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
_SRC = REPO_ROOT / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


_DOMAIN_ID = 160 + (os.getpid() % 20)
_STATE_TOPIC = "/mujoco_joint_states"
_COMMAND_TOPIC = "/oscbf_command"
_START_SERVICE = "/oscbf_controller/start_tracking"
def _path_start_configuration() -> np.ndarray:
    """The verified IK start pose exactly on the calibrated path start."""
    portable_tests = str(REPO_ROOT / "portable_oscbf" / "tests")
    if portable_tests not in sys.path:
        sys.path.insert(0, portable_tests)
    for entry in (
        str(REPO_ROOT / "portable_oscbf"),
        str(REPO_ROOT / "portable_oscbf" / "work"),
        str(REPO_ROOT / "portable_oscbf" / "vendor" / "dpax"),
    ):
        if entry not in sys.path:
            sys.path.insert(0, entry)
    from test_baseline_tracking import _work_start_configuration
    from work.ik_data_loader import load_repository_trajectory

    trajectory = load_repository_trajectory(
        str(REPO_ROOT / "data" / "nurbs" / "ik_input.mat")
    )
    trajectory.set_surface_normal_orientation([0.0, 1.0, 0.0])
    return _work_start_configuration(trajectory)


@pytest.fixture(scope="module")
def closed_loop():
    from robot_safecontrol_moveit.oscbf_controller import OscbfController
    from robot_safecontrol_moveit.oscbf_plant import OscbfPlant

    context = Context()
    rclpy.init(context=context, domain_id=_DOMAIN_ID)
    portable = str(REPO_ROOT / "portable_oscbf")
    plant = OscbfPlant(
        node_name="oscbf_plant_e2e",
        context=context,
        parameter_overrides=[
            rclpy.parameter.Parameter("portable_oscbf_root", value=portable),
        ],
    )
    controller = OscbfController(
        node_name="oscbf_controller_e2e",
        context=context,
        parameter_overrides=[
            rclpy.parameter.Parameter("portable_oscbf_root", value=portable),
            rclpy.parameter.Parameter(
                "trajectory_mat",
                value=str(REPO_ROOT / "data" / "nurbs" / "ik_input.mat"),
            ),
            rclpy.parameter.Parameter(
                "portable_config_yaml",
                value=str(REPO_ROOT / "portable_oscbf" / "config" / "nineaxis.yaml"),
            ),
            rclpy.parameter.Parameter("wait_for_start", value=True),
        ],
    )
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(plant)
    executor.add_node(controller)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield {
            "plant": plant,
            "controller": controller,
            "context": context,
            "executor": executor,
        }
    finally:
        executor.shutdown()
        plant.destroy_node()
        controller.destroy_node()
        rclpy.shutdown(context=context)


def test_zero_transition_then_tracking(closed_loop):
    plant = closed_loop["plant"]
    controller = closed_loop["controller"]
    context = closed_loop["context"]
    executor = closed_loop["executor"]
    path_start = _path_start_configuration()

    probe = rclpy.create_node("full_flow_probe", context=context)
    plant_states = []
    commands = []
    probe.create_subscription(
        JointState, _STATE_TOPIC,
        lambda message: plant_states.append(
            np.asarray(message.position, dtype=float)
        ),
        qos_profile_sensor_data,
    )
    probe.create_subscription(
        JointState, _COMMAND_TOPIC,
        lambda message: commands.append(message),
        qos_profile_sensor_data,
    )
    probe_pub = probe.create_publisher(
        JointState, _COMMAND_TOPIC, qos_profile_sensor_data
    )
    executor.add_node(probe)

    try:
        # 1. Plant starts at zero and publishes continuously; the controller
        #    must stay silent until the transition completes.
        time.sleep(1.0)
        assert plant_states and np.max(np.abs(plant_states[-1])) < 1e-6
        assert not commands, "controller commanded before the start signal"

        # 2. Transition replay drives the plant to the path start.
        command = JointState()
        command.name = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"]
        command.position = [float(value) for value in path_start]
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            probe_pub.publish(command)
            time.sleep(0.05)
            if plant_states and np.linalg.norm(
                plant_states[-1] - path_start
            ) < 0.01:
                break
        assert np.linalg.norm(plant.state - path_start) < 0.01, (
            "plant did not follow the transition replay"
        )
        transition_command_count = len(commands)

        # 3. Handoff: the planning server's completion signal starts tracking.
        client_node = rclpy.create_node("full_flow_start_client", context=context)
        client = client_node.create_client(Trigger, _START_SERVICE)
        executor.add_node(client_node)
        deadline = time.monotonic() + 5.0
        while not client.service_is_ready() and time.monotonic() < deadline:
            time.sleep(0.05)
        assert client.service_is_ready()
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        assert future.done() and future.result().success

        # 4. Controller takes over /oscbf_command and the plant keeps moving.
        before = plant.state.copy()
        deadline = time.monotonic() + 8.0
        while (
            time.monotonic() < deadline
            and len(commands) <= transition_command_count
        ):
            time.sleep(0.05)
        assert commands, "controller published no commands after the handoff"
        executor.remove_node(client_node)
        # Let the closed loop run through the slow trajectory start ramp; the
        # reference must advance and drag the plant along with it.
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            if controller.progress_snapshot().get("source_time_s", 0.0) > 0.5:
                break
            time.sleep(0.1)
        snapshot = controller.progress_snapshot()
        assert snapshot["ready"] and snapshot["steps"] >= 200
        assert np.isfinite(snapshot["pos_error_m"])
        assert np.isfinite(snapshot["orient_error_rad"])
        assert snapshot["source_time_s"] > 0.5, (
            f"reference stuck at source_time={snapshot['source_time_s']:.3f}s"
        )
        assert np.linalg.norm(plant.state - before) > 0.005, (
            "plant did not track after the handoff"
        )
        # Regression gate: a perfectly-converged plant publishes exactly the
        # command state over and over.  There must be no echo filtering on the
        # state topic (commands and states are separate streams).
        state_pub = probe.create_publisher(
            JointState, _STATE_TOPIC, qos_profile_sensor_data
        )
        converged = JointState()
        converged.name = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"]
        converged.position = [float(value) for value in plant.state]
        steps_before = len(controller._step_durations)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            state_pub.publish(converged)
            time.sleep(0.02)
        assert len(controller._step_durations) > steps_before, (
            "controller stalled on a converged (exact) plant state"
        )
        client_node.destroy_node()
    finally:
        executor.remove_node(probe)
        probe.destroy_node()
