"""Hardware bridge under fail-closed containment.

sim is inert. shadow records rejected commands without a CAN backend.
live is unavailable until backend, configuration, calibration and real
feedback freshness/watchdog qualification are implemented and reviewed.
No state stream is published while real feedback is unavailable.
"""

from __future__ import annotations

import time
from typing import Optional, Sequence

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .hardware_contract import (
    CommandSafetyGate,
    HardwareState,
    PositionCommand,
    ShadowCommandRecorder,
    WatchdogConfig,
)
from .robot_spec import DEFAULT_JOINT_NAMES
from .ros_conventions import OSCBF_COMMAND_TOPIC, state_stream_qos
from .socketcan_backend import CANBusBackend
from .unit_conversion import (
    JointCalibrationTable,
    JointNodeMap,
    TransmissionSpec,
)

_N_JOINTS = len(DEFAULT_JOINT_NAMES)


class HardwareBridge(Node):
    """Non-transmitting bridge; live readiness has no bypass parameter."""

    def __init__(
        self,
        *,
        backend: CANBusBackend | None = None,
        hardware_mode: str | None = None,
        j1_transmission: TransmissionSpec | None = None,
        calibration: JointCalibrationTable | None = None,
        node_map: JointNodeMap | None = None,
        poll_frequency_hz: float = 100.0,
        feedback_timeout_s: float = 0.2,
        command_timeout_s: float = 0.2,
        velocity_limit: float = 3.0,
        dq_limit: float = 0.01,
        parameter_overrides=None,
    ) -> None:
        super().__init__("hardware_bridge", parameter_overrides=parameter_overrides)
        try:
            mode = self.declare_parameter(
                "hardware_mode", "sim" if hardware_mode is None else hardware_mode,
                ParameterDescriptor(read_only=True),
            ).value
            if mode not in ("sim", "shadow", "live"):
                raise ValueError(f"invalid hardware_mode: {mode!r}")
            if hardware_mode is not None and mode != hardware_mode:
                raise ValueError("hardware_mode constructor/ROS override conflict")
            if mode == "live":
                raise RuntimeError(
                    "live disabled: backend/configuration/calibration and real "
                    "feedback freshness/watchdog qualification are incomplete"
                )
            if backend is not None:
                raise RuntimeError("sim/shadow do not accept CAN backends")
            # Old hardware constructor arguments must not silently become proof
            # of readiness, or appear to configure an inactive hardware path.
            if (j1_transmission is not None or calibration is not None
                    or node_map is not None or poll_frequency_hz != 100.0):
                raise ValueError("hardware configuration is unavailable in containment")
            self._hardware_mode = mode
            self._watchdog = WatchdogConfig(
                feedback_timeout_s=feedback_timeout_s,
                command_timeout_s=command_timeout_s,
                qdot_limit=np.full(_N_JOINTS, velocity_limit),
                dq_per_command_limit=np.full(_N_JOINTS, dq_limit),
            )
            self._safety_gate = CommandSafetyGate(self._watchdog)
            self._recorder = ShadowCommandRecorder() if mode == "shadow" else None
            self._last_requested_command: PositionCommand | None = None
            self._last_command: PositionCommand | None = None
            self._cmd_sub = None
            if mode == "shadow":
                self._cmd_sub = self.create_subscription(
                    JointState, OSCBF_COMMAND_TOPIC, self._on_command,
                    state_stream_qos(),
                )
            # No bus, polling timer or state publisher exists in containment.
            self.get_logger().info(
                f"hardware_bridge: mode={mode}, feedback=unavailable, transmission=disabled"
            )
        except Exception:
            self.destroy_node()
            raise

    @property
    def hardware_mode(self) -> str:
        return self._hardware_mode

    def _feedback_state(self) -> HardwareState:
        """Unknown is never replaced by command, zeros or a new receive stamp."""
        return HardwareState(
            q=np.full(_N_JOINTS, np.nan),
            qdot=np.full(_N_JOINTS, np.nan),
            stamp_s=float("-inf"),
            feedback_ok=False,
            watchdog_ok=False,
            mode=self.hardware_mode,
        )

    def _on_command(self, msg: JointState) -> None:
        if self.hardware_mode != "shadow":
            return
        if len(msg.position) != _N_JOINTS:
            self.get_logger().warn("command joint count mismatch")
            return
        now = time.monotonic()
        requested = PositionCommand(
            q=np.array(msg.position, dtype=float),
            stamp_s=now,
            valid_until_s=now + self._watchdog.command_timeout_s,
            source="controller",
            mode="shadow",
        )
        self._last_requested_command = requested
        state = self._feedback_state()
        safe = self._safety_gate.evaluate(state, requested, now_s=now)
        self._last_command = safe
        self._recorder.record(state, safe)

    def acknowledge_stop(self) -> bool:
        """An operator acknowledgement cannot manufacture healthy feedback."""
        return self._safety_gate.acknowledge_stop(
            self._feedback_state(), now_s=time.monotonic(),
        )

    @property
    def safety_gate(self) -> CommandSafetyGate:
        return self._safety_gate


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = HardwareBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
