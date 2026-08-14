"""G4 gates: transition replay drives the plant and notifies the controller."""

from __future__ import annotations

import os
import sys
import threading
import time
from pathlib import Path

import numpy as np


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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


_DOMAIN_ID = 150 + (os.getpid() % 20)
_JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"]


def _trajectory() -> JointTrajectory:
    trajectory = JointTrajectory()
    trajectory.joint_names = list(_JOINTS)
    for offset in (0.0, 0.1, 0.2):
        point = JointTrajectoryPoint()
        point.positions = [offset] * 9
        point.time_from_start.sec = int(offset * 10)
        trajectory.points.append(point)
    return trajectory


def test_replay_dual_publishes_state_and_command():
    from robot_safecontrol_moveit.trajectory_execution import TrajectoryExecutor

    context = Context()
    rclpy.init(context=context, domain_id=_DOMAIN_ID)
    node = rclpy.create_node("replay_dual_probe", context=context)
    state_messages = []
    command_messages = []
    node.create_subscription(
        JointState, "/mujoco_joint_states",
        lambda message: state_messages.append(message),
        qos_profile_sensor_data,
    )
    node.create_subscription(
        JointState, "/oscbf_command",
        lambda message: command_messages.append(message),
        qos_profile_sensor_data,
    )
    executor = MultiThreadedExecutor(num_threads=2, context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    executor_obj = TrajectoryExecutor(node, None, tuple(_JOINTS))
    replay_thread = threading.Thread(
        target=executor_obj.replay,
        kwargs={
            "trajectory": _trajectory(),
            "topic": "/mujoco_joint_states",
            "rate_hz": 100.0,
            "switch_viewer_to_tracking": False,
            "command_topic": "/oscbf_command",
        },
        daemon=True,
    )
    replay_thread.start()
    try:
        deadline = time.monotonic() + 10.0
        while (
            time.monotonic() < deadline
            and (not state_messages or not command_messages)
        ):
            time.sleep(0.05)
        assert state_messages, "replay published no viewer state"
        assert command_messages, "replay published no actuator commands"
        assert np.allclose(
            np.asarray(state_messages[-1].position, dtype=float),
            np.asarray(command_messages[-1].position, dtype=float),
        )
    finally:
        replay_thread.join(timeout=2.0)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)


def test_notify_oscbf_start_returns_service_response():
    from robot_safecontrol_moveit.transition_planning_server import (
        notify_oscbf_start,
    )

    context = Context()
    rclpy.init(context=context, domain_id=_DOMAIN_ID)
    node = rclpy.create_node("handoff_service_probe", context=context)

    def _handler(request, response):
        response.success = True
        response.message = "TRACKING_STARTED"
        return response

    node.create_service(Trigger, "/test/start_tracking", _handler)
    executor = MultiThreadedExecutor(num_threads=2, context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        code = notify_oscbf_start(
            node, "/test/start_tracking", timeout_s=5.0
        )
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
    assert code == "TRACKING_STARTED"
