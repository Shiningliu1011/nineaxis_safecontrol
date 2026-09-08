"""Jerk-limited actuator stand-in for the OSCBF closed-loop demo.

Replaces the throwaway "echo bridge": this node is a proper first-order
position loop with acceleration/jerk limits (``SCurveDriverSimulator``), and
it publishes its state continuously whether or not a command arrived — the
same way real encoders do.  A dropped BEST_EFFORT command therefore cannot
stall the loop.
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .oscbf_trajectory import bootstrap_portable, default_portable_root
from .robot_spec import DEFAULT_JOINT_NAMES
from .ros_conventions import (
    JOINT_STATE_TOPIC,
    OSCBF_COMMAND_TOPIC,
    state_stream_qos,
)


class OscbfPlant(Node):
    """Simulated actuator that follows position commands like a real driver."""

    def __init__(
        self,
        *,
        node_name: str = "oscbf_plant",
        parameter_overrides: Optional[List] = None,
        context=None,
    ) -> None:
        super().__init__(
            node_name,
            context=context,
            parameter_overrides=parameter_overrides,
        )
        self._declare_parameters()
        portable_root = Path(
            str(self.get_parameter("portable_oscbf_root").value)
        )
        if str(portable_root) == ".":
            portable_root = default_portable_root()
        bootstrap_portable(portable_root)

        from work.actuator_limits import load_actuator_limit_profile
        from work.driver_simulator import SCurveDriverSimulator
        from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX

        robot = NineaxisManipulatorJAX()
        self._q_min = np.asarray(robot.joint_lower_limits, dtype=float)
        self._q_max = np.asarray(robot.joint_upper_limits, dtype=float)
        profile = load_actuator_limit_profile()
        self._dq_max = np.asarray(robot.joint_max_velocities, dtype=float)

        self._frequency_hz = float(
            self.get_parameter("publish_frequency_hz").value
        )
        dt_plant = 1.0 / self._frequency_hz
        self._driver = SCurveDriverSimulator(
            dq_max=self._dq_max,
            ddq_max=np.asarray(profile.acceleration_limits, dtype=float),
            jerk_time=float(self.get_parameter("jerk_time").value),
            kp=float(self.get_parameter("position_gain").value),
            dt=dt_plant,
        )
        self._joint_names = [
            str(name) for name in self.get_parameter("joint_names").value
        ]
        self._start_rng = np.random.default_rng(
            int(self.get_parameter("start_seed").value)
        )
        start = [
            float(value) for value in self.get_parameter("start_position").value
        ]
        if len(start) != 9:
            raise ValueError("start_position must contain exactly 9 values")
        if bool(self.get_parameter("randomize_start").value):
            self._q = self._sample_start_pose()
        else:
            self._q = np.clip(
                np.asarray(start, dtype=float), self._q_min, self._q_max
            )
        self._target: Optional[np.ndarray] = None
        self._commands_received = 0
        self._last_command_time = None

        from std_srvs.srv import Trigger

        self._randomize_service = self.create_service(
            Trigger, "/oscbf_plant/randomize", self._randomize_callback
        )

        self.create_subscription(
            JointState,
            str(self.get_parameter("command_topic").value),
            self._on_command,
            state_stream_qos(),
        )
        self._state_pub = self.create_publisher(
            JointState,
            str(self.get_parameter("state_topic").value),
            state_stream_qos(),
        )
        self.create_timer(dt_plant, self._tick)
        self.get_logger().info(
            "oscbf_plant ready: "
            f"command={self.get_parameter('command_topic').value}, "
            f"state={self.get_parameter('state_topic').value} @ "
            f"{self._frequency_hz:.1f} Hz, start="
            f"{[round(float(v), 3) for v in self._q]}"
        )

    def _sample_start_pose(self) -> np.ndarray:
        """Uniformly sample a pose inside the joint limits (margin applied)."""
        margin = float(self.get_parameter("start_margin").value)
        lower = self._q_min + margin
        upper = self._q_max - margin
        # The persistent generator advances on every sample, so retries land
        # on different poses instead of repeating the seeded pose.
        return np.asarray(self._start_rng.uniform(lower, upper), dtype=float)

    def _randomize_callback(self, request, response):
        self._q = self._sample_start_pose()
        self._target = None
        self._driver.reset()
        response.success = True
        response.message = "RANDOMIZED: " + ",".join(
            f"{float(value):.4f}" for value in self._q
        )
        self.get_logger().info(
            f"plant start pose randomised: {[round(float(v), 3) for v in self._q]}"
        )
        return response

    def _declare_parameters(self) -> Path:
        share_dir = Path()
        try:
            from ament_index_python.packages import get_package_share_directory

            share_dir = Path(
                get_package_share_directory("robot_safecontrol_moveit")
            )
        except Exception:
            share_dir = Path(__file__).resolve().parents[2]
        defaults = {
            "command_topic": OSCBF_COMMAND_TOPIC,
            "state_topic": JOINT_STATE_TOPIC,
            "joint_names": list(DEFAULT_JOINT_NAMES),
            "publish_frequency_hz": 100.0,
            "jerk_time": 0.08,
            "position_gain": 80.0,
            "portable_oscbf_root": str(share_dir / "portable_oscbf"),
            "start_position": [0.0] * 9,
            "randomize_start": False,
            "start_seed": 0,
            "start_margin": 0.05,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        if not 1.0 <= float(self.get_parameter("publish_frequency_hz").value) <= 1000.0:
            raise ValueError("publish_frequency_hz must be in [1, 1000]")
        if float(self.get_parameter("jerk_time").value) <= 0.0:
            raise ValueError("jerk_time must be positive")
        if float(self.get_parameter("position_gain").value) <= 0.0:
            raise ValueError("position_gain must be positive")
        if float(self.get_parameter("start_margin").value) < 0.0:
            raise ValueError("start_margin must be non-negative")
        return share_dir

    def _extract_positions(self, message: JointState) -> Optional[np.ndarray]:
        if len(message.position) == 9 and not message.name:
            return np.asarray(message.position, dtype=float)
        if set(message.name) != set(self._joint_names):
            return None
        order = {name: index for index, name in enumerate(message.name)}
        return np.asarray(
            [message.position[order[name]] for name in self._joint_names],
            dtype=float,
        )

    def _on_command(self, message: JointState) -> None:
        positions = self._extract_positions(message)
        if positions is None or not np.all(np.isfinite(positions)):
            return
        clipped = np.clip(positions, self._q_min, self._q_max)
        large_jump = (
            self._target is None
            or float(np.max(np.abs(clipped - self._target))) > 0.01
        )
        self._target = clipped
        self._commands_received += 1
        self._last_command_time = self.get_clock().now().nanoseconds * 1e-9
        if large_jump:
            # Phase-switch smoothing only: large jumps (transition start) ramp
            # through the S-curve, while small tracking commands pass through
            # the stable first-order position loop.
            self._driver.begin_transition()

    def step_plant(self) -> np.ndarray:
        """Advance the simulated actuator one control period (pure method)."""
        if self._target is None:
            v_target = np.zeros(9)
        else:
            v_target = np.clip(
                float(self.get_parameter("position_gain").value)
                * (self._target - self._q),
                -self._dq_max,
                self._dq_max,
            )
        v = self._driver.step(v_target)
        dt_plant = 1.0 / self._frequency_hz
        self._q = np.clip(
            self._q + v * dt_plant, self._q_min, self._q_max
        )
        return self._q.copy()

    def _tick(self) -> None:
        q = self.step_plant()
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self._joint_names)
        message.position = [float(value) for value in q]
        self._state_pub.publish(message)

    @property
    def state(self) -> np.ndarray:
        return self._q.copy()

    @property
    def commands_received(self) -> int:
        return self._commands_received


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = OscbfPlant()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
