"""M10 acceptance smoke tests for the ``oscbf_controller`` ROS 2 node.

These tests deliberately avoid MoveIt: they construct the controller with
parameter injection, exercise its pure ``step_once`` method, and verify the
published ``JointState`` over a short-lived ROS graph on an isolated DDS
domain.  The node's JAX warm-up runs once in a module-scoped fixture.
"""

from __future__ import annotations

import os
import re
import sys
import time
from pathlib import Path

# Keep the single-threaded XLA contract of the portable suite before the node
# imports JAX inside its constructor.
os.environ.setdefault("XLA_FLAGS", "--xla_cpu_multi_thread_eigen=false")
os.environ.setdefault("JAX_NUM_THREADS", "1")

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
_SRC = REPO_ROOT / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


_DOMAIN_ID = 130 + (os.getpid() % 20)
_STATE_TOPIC = "/mujoco_joint_states"
_COMMAND_TOPIC = "/oscbf_command"
_START_SERVICE = "/oscbf_controller/start_tracking"
_START_Q = np.array([
    0.2303562, 0.1112539, 1.0167209, -0.6810303, -1.8294025,
    -0.4664294, 0.4743473, -1.0429228, 0.0289233,
])


@pytest.fixture(scope="module")
def controller_fixture(tmp_path_factory):
    from robot_safecontrol_moveit.oscbf_controller import OscbfController

    context = Context()
    rclpy.init(context=context, domain_id=_DOMAIN_ID)
    perf_path = tmp_path_factory.mktemp("oscbf_m10") / "perf.md"
    node = OscbfController(
        node_name="oscbf_controller_smoke",
        context=context,
        parameter_overrides=[
            rclpy.parameter.Parameter(
                "portable_oscbf_root",
                value=str(REPO_ROOT / "portable_oscbf"),
            ),
            rclpy.parameter.Parameter(
                "trajectory_mat",
                value=str(REPO_ROOT / "data" / "nurbs" / "ik_input.mat"),
            ),
            rclpy.parameter.Parameter(
                "portable_config_yaml",
                value=str(REPO_ROOT / "portable_oscbf" / "config" / "nineaxis.yaml"),
            ),
            rclpy.parameter.Parameter("perf_report_path", value=str(perf_path)),
            rclpy.parameter.Parameter("publish_frequency_hz", value=100.0),
            rclpy.parameter.Parameter("wait_for_start", value=True),
        ],
    )
    try:
        yield {"node": node, "context": context, "perf_path": perf_path}
    finally:
        node.write_perf_report()
        node.destroy_node()
        rclpy.shutdown(context=context)


def _in_bounds(node, positions: np.ndarray) -> bool:
    lower, upper = node._limits
    return bool(
        np.all(np.isfinite(positions))
        and np.all(positions >= lower - 1e-9)
        and np.all(positions <= upper + 1e-9)
    )


def test_state_subscription_uses_deep_best_effort_queue():
    from robot_safecontrol_moveit.oscbf_controller import _state_subscription_qos

    qos = _state_subscription_qos()
    assert qos.depth == 20
    assert qos.reliability == rclpy.qos.ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == rclpy.qos.DurabilityPolicy.VOLATILE


def test_node_starts_without_move_group(controller_fixture):
    node = controller_fixture["node"]
    assert node.get_name() == "oscbf_controller_smoke"
    assert node._loop.path_is_configured


def test_step_once_returns_valid_safe_state(controller_fixture):
    node = controller_fixture["node"]
    step = node.step_once(_START_Q)
    q_next = step["q_next"]
    assert q_next.shape == (9,)
    assert _in_bounds(node, q_next)
    assert step["u_safe"].shape == (9,)
    assert step["err_6d"].shape == (6,)
    assert np.all(np.isfinite(step["u_safe"]))
    assert np.all(np.isfinite(step["err_6d"]))
    assert step["qp_ok"]
    assert step["min_obs_dist"] > 0.0


def test_no_command_before_start_signal(controller_fixture):
    node = controller_fixture["node"]
    context = controller_fixture["context"]

    probe = rclpy.create_node(
        "oscbf_gate_probe", context=context
    )
    received = []

    def _on_state(message: JointState) -> None:
        received.append(message)

    probe.create_subscription(
        JointState, _COMMAND_TOPIC, _on_state, qos_profile_sensor_data
    )
    plant_pub = probe.create_publisher(
        JointState, _STATE_TOPIC, qos_profile_sensor_data
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(probe)
    executor.add_node(node)

    plant = JointState()
    plant.name = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"]
    plant.position = [float(value) for value in _START_Q]

    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        plant_pub.publish(plant)
        executor.spin_once(timeout_sec=0.2)

    executor.remove_node(node)
    probe.destroy_node()
    assert not received, "controller published before the start signal"


def test_start_signal_unlocks_safe_state(controller_fixture):
    from std_srvs.srv import Trigger

    node = controller_fixture["node"]
    context = controller_fixture["context"]

    client_node = rclpy.create_node("oscbf_start_client", context=context)
    client = client_node.create_client(Trigger, _START_SERVICE)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(client_node)
    executor.add_node(node)
    deadline = time.monotonic() + 5.0
    while not client.service_is_ready() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
    assert client.service_is_ready(), "start service never became ready"
    future = client.call_async(Trigger.Request())
    deadline = time.monotonic() + 5.0
    while not future.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
    assert future.done(), "start service call timed out"
    assert future.result().success
    executor.remove_node(client_node)
    executor.remove_node(node)
    client_node.destroy_node()

    probe = rclpy.create_node("oscbf_controller_smoke_probe", context=context)
    received = []
    probe.create_subscription(
        JointState, _COMMAND_TOPIC,
        lambda message: received.append(message),
        qos_profile_sensor_data,
    )
    plant_pub = probe.create_publisher(
        JointState, _STATE_TOPIC, qos_profile_sensor_data
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(probe)
    executor.add_node(node)

    plant = JointState()
    plant.name = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"]
    plant_positions = _START_Q.copy()
    plant_positions[0] += 0.01
    plant.position = [float(value) for value in plant_positions]

    deadline = time.monotonic() + 15.0
    output = None
    while time.monotonic() < deadline:
        plant_pub.publish(plant)
        executor.spin_once(timeout_sec=0.2)
        for message in received:
            positions = np.asarray(message.position, dtype=float)
            if positions.shape == (9,) and np.max(
                np.abs(positions - plant_positions)
            ) > 1e-6:
                output = message
                break
        if output is not None:
            break

    executor.remove_node(node)
    probe.destroy_node()
    assert output is not None, "controller published no safe state after start"
    assert output.name == ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"]
    assert len(output.position) == 9
    assert _in_bounds(node, np.asarray(output.position, dtype=float))


def test_progress_snapshot_reports_tracking_state(controller_fixture):
    node = controller_fixture["node"]
    snapshot = node.progress_snapshot()
    assert snapshot["tracking_started"]
    assert snapshot["ready"]
    assert snapshot["steps"] >= 1
    assert 0.0 <= snapshot["arc_fraction"] <= 1.0
    assert np.isfinite(snapshot["cross_track_error_m"])
    assert np.isfinite(snapshot["latency_p95_ms"])


def test_perf_report_p95_within_budget(controller_fixture):
    node = controller_fixture["node"]
    context = controller_fixture["context"]

    # Drive a short burst of plant states so the controller accumulates real
    # step-latency samples, independent of the publish test's side effects.
    if len(node._step_durations) < 20:
        probe = rclpy.create_node("oscbf_perf_probe", context=context)
        publisher = probe.create_publisher(
            JointState, _STATE_TOPIC, qos_profile_sensor_data
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(probe)
        executor.add_node(node)
        plant = JointState()
        plant.name = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"]
        plant.position = [float(value) for value in _START_Q]
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            publisher.publish(plant)
            executor.spin_once(timeout_sec=0.05)
        executor.remove_node(node)
        probe.destroy_node()

    node.write_perf_report()
    text = controller_fixture["perf_path"].read_text(encoding="utf-8")
    match = re.search(r"p95: ([0-9.]+) ms", text)
    assert match is not None, f"missing p95 in report:\n{text}"
    p95 = float(match.group(1))
    assert 0.0 < p95 < 10.0, f"p95 step latency {p95:.3f} ms out of budget"
    # Keep the acceptance evidence in the repository output directory too.
    evidence_path = REPO_ROOT / "output" / "oscbf_m10_perf.md"
    evidence_path.parent.mkdir(parents=True, exist_ok=True)
    evidence_path.write_text(text, encoding="utf-8")
