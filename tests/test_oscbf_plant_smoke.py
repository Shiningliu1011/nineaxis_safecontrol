"""G3 acceptance gates for the jerk-limited actuator plant."""

from __future__ import annotations

import sys
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
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


_DOMAIN_ID = 140 + (__import__("os").getpid() % 20)
_STATE_TOPIC = "/mujoco_joint_states"
_TARGET = np.array([
    0.23, 0.11, 1.01, -0.68, -1.82, -0.46, 0.47, -1.04, 0.02,
])


@pytest.fixture(scope="module")
def plant_fixture():
    from robot_safecontrol_moveit.oscbf_plant import OscbfPlant

    context = Context()
    rclpy.init(context=context, domain_id=_DOMAIN_ID)
    node = OscbfPlant(
        node_name="oscbf_plant_smoke",
        context=context,
        parameter_overrides=[
            rclpy.parameter.Parameter(
                "portable_oscbf_root",
                value=str(REPO_ROOT / "portable_oscbf"),
            ),
        ],
    )
    yield {"node": node, "context": context}
    node.destroy_node()
    rclpy.shutdown(context=context)


def test_plant_starts_at_zero(plant_fixture):
    node = plant_fixture["node"]
    assert np.allclose(node.state, np.zeros(9))
    assert node._q_min[0] == pytest.approx(0.0)
    assert node._q_max[0] == pytest.approx(0.585)


def test_plant_converges_and_respects_limits(plant_fixture):
    node = plant_fixture["node"]
    node._target = _TARGET.copy()
    dt_plant = 1.0 / node._frequency_hz
    previous = node.state.copy()
    max_step = 0.0
    for _ in range(300):
        q = node.step_plant()
        max_step = max(max_step, float(np.max(np.abs(q - previous))))
        previous = q
        assert np.all(q >= node._q_min - 1e-12)
        assert np.all(q <= node._q_max + 1e-12)

    final_error = float(np.linalg.norm(node.state - _TARGET))
    assert final_error < 1e-4, (
        f"plant did not converge to the command: error {final_error:.3e}"
    )
    assert max_step <= float(np.max(node._dq_max)) * dt_plant * 1.01 + 1e-12, (
        f"plant violated the velocity limit: {max_step:.4f}"
    )


def test_plant_publishes_continuously_without_command(plant_fixture):
    node = plant_fixture["node"]
    context = plant_fixture["context"]
    probe = rclpy.create_node("oscbf_plant_probe", context=context)
    received = []
    probe.create_subscription(
        JointState,
        _STATE_TOPIC,
        lambda message: received.append(message),
        qos_profile_sensor_data,
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(probe)
    executor.add_node(node)
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
    executor.remove_node(node)
    probe.destroy_node()
    assert len(received) > 50, (
        f"plant did not publish continuously: {len(received)} messages in 2s"
    )


def test_randomized_start_and_resample_stay_within_limits(plant_fixture):
    from std_srvs.srv import Trigger

    node = plant_fixture["node"]
    node._start_rng = np.random.default_rng(7)
    margin = 0.05
    first = node._sample_start_pose()
    assert np.all(first >= node._q_min + margin - 1e-12)
    assert np.all(first <= node._q_max - margin + 1e-12)

    response = Trigger.Response()
    node._randomize_callback(Trigger.Request(), response)
    assert response.success
    assert np.any(np.abs(node.state - first) > 1e-6), (
        "randomize service must resample a different pose"
    )
    assert np.all(node.state >= node._q_min + margin - 1e-12)
    assert np.all(node.state <= node._q_max - margin + 1e-12)
