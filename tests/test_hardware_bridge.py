"""Containment tests: ROS objects and raising doubles only; no CAN device."""

import numpy as np
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from robot_safecontrol_moveit.hardware_bridge import HardwareBridge, main
from robot_safecontrol_moveit.robot_spec import DEFAULT_JOINT_NAMES
from robot_safecontrol_moveit.unit_conversion import (
    JointCalibrationTable, JointNodeMap, PerJointCalibration, TransmissionSpec,
)


class NoIOBackend:
    def send(self, *args, **kwargs):
        pytest.fail("CAN send reached")

    def recv(self, *args, **kwargs):
        pytest.fail("CAN recv reached")

    def close(self):
        pytest.fail("bridge acquired backend ownership")


@pytest.fixture
def ros_context():
    rclpy.init(args=[])
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture(autouse=True)
def forbid_bus_construction(monkeypatch):
    from robot_safecontrol_moveit import socketcan_backend

    def forbidden(*args, **kwargs):
        pytest.fail("CAN bus constructed")

    monkeypatch.setattr(socketcan_backend.SocketCANBus, "__init__", forbidden)


def command():
    msg = JointState()
    msg.name = list(DEFAULT_JOINT_NAMES)
    msg.position = [0.2] * len(DEFAULT_JOINT_NAMES)
    return msg


@pytest.mark.parametrize("mode", ["sim", "shadow"])
def test_nonlive_has_no_bus_io_timer_or_state_publisher(ros_context, mode):
    node = HardwareBridge(hardware_mode=mode)
    try:
        assert node.get_name() == "hardware_bridge"
        assert node.hardware_mode == mode
        assert not any(pub.msg_type is JointState for pub in node.publishers)
        assert list(node.timers) == []
        command_subs = [sub for sub in node.subscriptions if sub.msg_type is JointState]
        assert len(command_subs) == (1 if mode == "shadow" else 0)
        node._on_command(command())
        assert not node._feedback_state().feedback_ok
    finally:
        node.destroy_node()


@pytest.mark.parametrize("mode", ["sim", "shadow"])
def test_nonlive_rejects_even_an_injected_backend(ros_context, mode):
    with pytest.raises(RuntimeError, match="do not accept CAN"):
        HardwareBridge(hardware_mode=mode, backend=NoIOBackend())


@pytest.mark.parametrize("overrides", [
    {},
    {"backend": NoIOBackend()},
    {"j1_transmission": TransmissionSpec(lead_mm_per_rev=10.0)},
    {
        "backend": NoIOBackend(),
        "j1_transmission": TransmissionSpec(lead_mm_per_rev=10.0),
        "node_map": JointNodeMap.from_joint_list(DEFAULT_JOINT_NAMES),
        "calibration": JointCalibrationTable.from_entries(
            PerJointCalibration(j) for j in DEFAULT_JOINT_NAMES
        ),
        "feedback_timeout_s": 0.2,
        "command_timeout_s": 0.2,
    },
])
def test_live_is_rejected_before_control_entities(ros_context, monkeypatch, overrides):
    original_publisher = HardwareBridge.create_publisher
    original_subscription = HardwareBridge.create_subscription

    def publisher(self, msg_type, *args, **kwargs):
        if msg_type is JointState:
            pytest.fail("control publisher created before rejecting live")
        return original_publisher(self, msg_type, *args, **kwargs)

    def subscription(self, msg_type, *args, **kwargs):
        if msg_type is JointState:
            pytest.fail("control subscriber created before rejecting live")
        return original_subscription(self, msg_type, *args, **kwargs)

    def timer(*args, **kwargs):
        pytest.fail("timer created before rejecting live")

    # Node itself creates parameter-event infrastructure; it is not a control publisher.
    monkeypatch.setattr(HardwareBridge, "create_publisher", publisher)
    monkeypatch.setattr(HardwareBridge, "create_subscription", subscription)
    monkeypatch.setattr(HardwareBridge, "create_timer", timer)
    with pytest.raises(RuntimeError, match="live disabled"):
        HardwareBridge(hardware_mode="live", **overrides)


def test_command_and_acknowledgement_cannot_create_feedback(ros_context):
    node = HardwareBridge(hardware_mode="shadow")
    try:
        node._on_command(command())
        state, safe = node._recorder.records[-1]
        assert np.isnan(state.q).all()
        assert np.isnan(state.qdot).all()
        assert state.stamp_s == float("-inf")
        assert not state.feedback_ok
        assert not state.watchdog_ok
        assert safe.is_stop
        assert np.allclose(node._last_requested_command.q, command().position)
        # Even a stale legacy cache must not become a feedback source.
        node._last_feedback_q = np.ones(len(DEFAULT_JOINT_NAMES))
        reason = node.safety_gate.latched_stop_reason
        assert not node.acknowledge_stop()
        assert node.safety_gate.latched_stop_reason == reason
        node._on_command(command())
        state, safe = node._recorder.records[-1]
        assert not state.feedback_ok
        assert np.isnan(state.q).all()
        assert safe.is_stop
    finally:
        node.destroy_node()


def test_mode_cannot_change_after_construction(ros_context):
    node = HardwareBridge(hardware_mode="shadow")
    try:
        result = node.set_parameters([Parameter("hardware_mode", value="live")])[0]
        assert not result.successful
        assert node.hardware_mode == "shadow"
        with pytest.raises(AttributeError):
            node.hardware_mode = "live"
    finally:
        node.destroy_node()


def test_explicit_mode_cannot_hide_ros_override(ros_context):
    with pytest.raises(ValueError, match="override conflict"):
        HardwareBridge(
            hardware_mode="shadow",
            parameter_overrides=[Parameter("hardware_mode", value="live")],
        )


@pytest.mark.parametrize("mode", ["sim", "shadow"])
def test_main_consumes_ros_mode_override_and_cleans_up(monkeypatch, mode):
    seen = []

    def inspect_node(node):
        seen.append(node.hardware_mode)
        assert not any(pub.msg_type is JointState for pub in node.publishers)
        assert list(node.timers) == []

    monkeypatch.setattr(rclpy, "spin", inspect_node)
    main(["--ros-args", "-p", f"hardware_mode:={mode}"])
    assert seen == [mode]
    assert not rclpy.ok()


@pytest.mark.parametrize("mode,error", [("live", RuntimeError), ("typo", ValueError)])
def test_main_rejects_mode_and_cleans_up(monkeypatch, mode, error):
    def forbidden(node):
        pytest.fail("spin reached for rejected mode")

    monkeypatch.setattr(rclpy, "spin", forbidden)
    with pytest.raises(error):
        main(["--ros-args", "-p", f"hardware_mode:={mode}"])
    assert not rclpy.ok()


def test_no_global_gate_state(ros_context):
    first = HardwareBridge(hardware_mode="shadow")
    second = HardwareBridge(hardware_mode="shadow")
    try:
        first._on_command(command())
        assert first.safety_gate.latched_stop_reason
        assert not second.safety_gate.latched_stop_reason
        assert second._recorder.records == []
    finally:
        first.destroy_node()
        second.destroy_node()
