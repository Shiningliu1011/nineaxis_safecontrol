"""Trajectory execution through MoveIt's ``/execute_trajectory`` action."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from time import monotonic, sleep
from typing import Sequence

from bisect import bisect_right

from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
from pymoveit2 import MoveIt2, MoveIt2State
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class ExecutionError(RuntimeError):
    """The trajectory cannot be safely submitted to MoveIt for execution."""


@dataclass(frozen=True)
class ExecutionResult:
    submitted: bool
    completed: bool
    succeeded: bool


class TrajectoryExecutor:
    """Validates, submits, waits for, and cancels MoveIt trajectories."""

    def __init__(self, node, moveit: MoveIt2, joint_names: Sequence[str]):
        self._node = node
        self._moveit = moveit
        self._joint_names = tuple(joint_names)

    def execute(
        self,
        trajectory: JointTrajectory,
        *,
        dry_run: bool = False,
        wait: bool = True,
    ) -> ExecutionResult:
        """Submit a MoveIt-planned/timed trajectory only when explicitly enabled."""
        self._validate_trajectory(trajectory)
        if dry_run:
            self._node.get_logger().info(
                "Dry run: trajectory was validated but not sent to /execute_trajectory."
            )
            return ExecutionResult(submitted=False, completed=False, succeeded=False)

        if self._moveit.query_state() != MoveIt2State.IDLE:
            raise ExecutionError("MoveIt already has a motion request or execution in progress")

        self._moveit.execute(trajectory)
        if not wait:
            return ExecutionResult(submitted=True, completed=False, succeeded=False)

        succeeded = self._moveit.wait_until_executed()
        if not succeeded:
            error_code = self._moveit.get_last_execution_error_code()
            raise ExecutionError(f"MoveIt execution failed: {error_code}")
        return ExecutionResult(submitted=True, completed=True, succeeded=True)

    def replay(
        self,
        trajectory: JointTrajectory,
        *,
        topic: str = "/joint_states",
        rate_hz: float = 30.0,
        command_topic: str | None = None,
        time_scale: float = 1.0,
        min_duration_s: float = 0.0,
    ) -> ExecutionResult:
        """Publish trajectory points for visualization and, optionally, as
        actuator commands on ``command_topic`` (the OSCBF plant follows the
        same replayed transition before the controller takes over).

        ``time_scale`` above 1.0 stretches the replayed time profile (the
        time-optimal transition over metres of joint travel otherwise reads
        as one fast "swing" and is easy to perceive as jerky motion).
        ``min_duration_s`` raises the effective scale so a short transition
        is never replayed as a blink-of-an-eye snap.
        """
        self._validate_trajectory(trajectory)

        # Pause the JointStateBroadcaster so it doesn't publish zero positions.
        broadcaster_was_active = self._switch_broadcaster(activate=False)

        pub = self._node.create_publisher(
            JointState, topic, qos_profile_sensor_data
        )
        command_pub = None
        if command_topic:
            command_pub = self._node.create_publisher(
                JointState, command_topic, qos_profile_sensor_data
            )
        sleep(0.3)

        self._node.get_logger().info(
            f"Replaying {len(trajectory.points)} trajectory points to {topic} "
            f"at {rate_hz} Hz"
        )

        period = 1.0 / rate_hz
        scale = float(time_scale) if time_scale > 0.0 else 1.0
        try:
            times = self._point_times(trajectory)
            if times is not None:
                # TOPP/平滑时间参数化给出的非均匀时间被等速回放丢弃会让
                # 稀疏段每步跳变 (观感甩臂); 按时间戳在点间线性插值重采样,
                # 保留规划出的速度剖面。time_scale>1 时按比例拉长整个剖面。
                if float(min_duration_s) > 0.0 and times[-1] * scale < min_duration_s:
                    scale = min_duration_s / times[-1]
                times_scaled = [t * scale for t in times]
                self._node.get_logger().info(
                    f"  time profile: raw={times[-1]:.2f}s "
                    f"time_scale={float(time_scale):.2f} "
                    f"min_duration={float(min_duration_s):.2f}s "
                    f"-> scaled={times_scaled[-1]:.2f}s"
                )
                self._replay_interpolated(
                    pub, command_pub, trajectory, times_scaled, period
                )
            else:
                # planner 结果未填 time_from_start: 按 rate_hz 等速, 但把
                # time_scale/min_duration 折算到帧间隔上, 避免过短快放。
                n = len(trajectory.points)
                period_eff = period * scale
                if float(min_duration_s) > 0.0 and n * period_eff < min_duration_s:
                    period_eff = min_duration_s / max(n, 1)
                self._node.get_logger().info(
                    f"  no time_from_start: uniform replay "
                    f"{n} pts at {1.0 / period_eff:.0f} Hz "
                    f"(scaled {period_eff:.2f}s step)"
                )
                self._replay_uniform(pub, command_pub, trajectory, period_eff)
        finally:
            self._node.get_logger().info("Replay complete.")
            self._node.destroy_publisher(pub)
            if command_pub is not None:
                self._node.destroy_publisher(command_pub)
            if broadcaster_was_active:
                self._switch_broadcaster(activate=True)

        return ExecutionResult(submitted=True, completed=True, succeeded=True)

    @staticmethod
    def _point_times(trajectory: JointTrajectory) -> list[float] | None:
        """Return per-point time_from_start seconds, or None if untimed."""
        points = trajectory.points
        if not points or len(points) < 2:
            return None
        times = [
            float(point.time_from_start.sec) + point.time_from_start.nanosec * 1e-9
            for point in points
        ]
        if times[0] <= 0.0 or times[-1] <= times[0]:
            return None
        if any(t2 <= t1 for t1, t2 in zip(times, times[1:])):
            return None
        return times

    def _replay_interpolated(
        self,
        pub,
        command_pub,
        trajectory: JointTrajectory,
        times: list[float],
        period: float,
    ) -> None:
        """Replay at ``rate_hz`` with linear interpolation between waypoints."""
        points = trajectory.points
        positions = [
            [float(v) for v in point.positions] for point in points
        ]
        step_count = max(1, int(round(times[-1] / period)))
        start = monotonic()
        for frame in range(1, step_count + 1):
            target_t = times[-1] * frame / step_count
            index = bisect_right(times, target_t) - 1
            index = max(0, min(int(index), len(points) - 2))
            t0, t1 = times[index], times[index + 1]
            span = max(t1 - t0, 1e-9)
            ratio = min(max((target_t - t0) / span, 0.0), 1.0)
            pos = [
                a + (b - a) * ratio
                for a, b in zip(positions[index], positions[index + 1])
            ]
            self._publish_replay_point(
                pub, command_pub, trajectory.joint_names, pos
            )
            sleep(max(0.0, start + target_t - monotonic()))
        self._publish_replay_point(
            pub, command_pub, trajectory.joint_names, positions[-1]
        )

    def _replay_uniform(
        self,
        pub,
        command_pub,
        trajectory: JointTrajectory,
        period: float,
    ) -> None:
        """Fallback equal-rate playback for trajectories without time data."""
        for index, point in enumerate(trajectory.points):
            self._publish_replay_point(
                pub, command_pub, trajectory.joint_names, point.positions
            )
            if index % 10 == 0 or index == len(trajectory.points) - 1:
                self._node.get_logger().info(
                    f"  Point {index + 1}/{len(trajectory.points)}"
                )
            sleep(period)

    def _publish_replay_point(
        self,
        pub,
        command_pub,
        joint_names,
        positions,
    ) -> None:
        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = list(joint_names)
        msg.position = [float(v) for v in positions]
        pub.publish(msg)
        if command_pub is not None:
            command_pub.publish(msg)

    def _switch_broadcaster(self, *, activate: bool) -> bool:
        """Activate or deactivate the JointStateBroadcaster (non-spin)."""
        client = self._node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        if not client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warning(
                "controller_manager service not available; cannot pause broadcaster"
            )
            return False

        request = SwitchController.Request()
        if activate:
            request.activate_controllers = ["joint_state_broadcaster"]
            request.deactivate_controllers = []
        else:
            request.activate_controllers = []
            request.deactivate_controllers = ["joint_state_broadcaster"]
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.timeout = Duration(sec=1)

        future = client.call_async(request)
        deadline = monotonic() + 2.0
        while not future.done() and monotonic() < deadline:
            sleep(0.01)
        if not future.done():
            self._node.get_logger().warning("Failed to switch JointStateBroadcaster: timeout")
            return False
        result = future.result()
        if result is not None and result.ok:
            state = "resumed" if activate else "paused"
            self._node.get_logger().info(f"JointStateBroadcaster {state}")
            return True
        self._node.get_logger().warning("Failed to switch JointStateBroadcaster")
        return False

    def stop(self) -> None:
        """Request cancellation of a currently executing MoveIt trajectory."""
        if self._moveit.query_state() == MoveIt2State.EXECUTING:
            self._moveit.cancel_execution()

    def _validate_trajectory(self, trajectory: JointTrajectory) -> None:
        if tuple(trajectory.joint_names) != self._joint_names:
            raise ExecutionError(
                "Trajectory joint order must exactly match the MoveIt group: "
                f"{list(self._joint_names)}"
            )
        if not trajectory.points:
            raise ExecutionError("Trajectory has no points")

        previous_time: float | None = None
        for index, point in enumerate(trajectory.points):
            if len(point.positions) != len(self._joint_names) or not all(
                isfinite(float(value)) for value in point.positions
            ):
                raise ExecutionError(f"Trajectory point {index} has invalid positions")
            point_time = self._seconds(point.time_from_start)
            if point_time < 0.0 or (
                previous_time is not None and point_time <= previous_time
            ):
                raise ExecutionError(
                    f"Trajectory point {index} does not have a strictly increasing time_from_start"
                )
            previous_time = point_time

    @staticmethod
    def _seconds(duration: Duration) -> float:
        return float(duration.sec) + float(duration.nanosec) / 1_000_000_000.0
