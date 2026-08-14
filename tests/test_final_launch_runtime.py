"""Headless runtime coverage for the unified final MoveIt launch.

This intentionally includes the real final launch file instead of rebuilding
its node actions in the test.  The Viewer is disabled through its supported
``start_viewer`` argument, so this can run in CI without a display server.
"""

from __future__ import annotations

from collections import Counter
import os
from pathlib import Path
import time
import unittest

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import launch_testing.markers
from moveit_msgs.srv import (
    GetMotionPlan,
    GetPlanningScene,
    GetPositionIK,
    GetStateValidity,
)
import pytest
import rclpy
import numpy as np
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


_PACKAGE_NAME = "robot_safecontrol_moveit"
_LAUNCH_FILENAME = "mujoco_transition_final.launch.py"
_STARTUP_TIMEOUT_S = 45.0
_POLL_PERIOD_S = 0.1

# The final launch uses root-level node and service names.  Give this test an
# isolated, ROS-supported domain range so an interactive demo in domain zero
# cannot make the exact-instance assertion pass or fail by accident.
_TEST_DOMAIN_ID = 72 + (os.getpid() % 20)

_CORE_NODES = (
    "/robot_state_publisher",
    "/transition_planning_server",
    "/oscbf_controller",
)
_MOVEIT_SERVICES = (
    ("/compute_ik", GetPositionIK),
    ("/check_state_validity", GetStateValidity),
    ("/get_planning_scene", GetPlanningScene),
    ("/plan_kinematic_path", GetMotionPlan),
)


def _final_launch_path() -> Path:
    """Resolve the installed launch when available, otherwise the source file."""
    try:
        installed = (
            Path(get_package_share_directory(_PACKAGE_NAME))
            / "launch"
            / _LAUNCH_FILENAME
        )
        if installed.is_file():
            return installed
    except PackageNotFoundError:
        pass

    source = Path(__file__).resolve().parents[1] / "launch" / _LAUNCH_FILENAME
    if source.is_file():
        return source
    raise FileNotFoundError(
        f"Could not find {_LAUNCH_FILENAME} in the installed package or source tree"
    )


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description() -> LaunchDescription:
    # Set this before launch starts so all four launched processes inherit the
    # same isolated domain as the probe node created by the test below.
    os.environ["ROS_DOMAIN_ID"] = str(_TEST_DOMAIN_ID)
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(_final_launch_path())),
                launch_arguments={"start_viewer": "false"}.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


def _fully_qualified_name(name: str, namespace: str) -> str:
    namespace = namespace.rstrip("/")
    if not namespace:
        return f"/{name.lstrip('/')}"
    return f"{namespace}/{name.lstrip('/')}"


class TestFinalLaunchRuntime(unittest.TestCase):
    """Assert the real launch reaches the expected headless ROS graph."""

    def setUp(self) -> None:
        # The final launch delays its optional Viewer by three seconds.  Check
        # the graph after that window too, so ``start_viewer:=false`` is
        # verified rather than merely observed before the timer could fire.
        self._viewer_check_not_before = time.monotonic() + 4.0
        self._context = Context()
        rclpy.init(context=self._context, domain_id=_TEST_DOMAIN_ID)
        self._node = rclpy.create_node(
            "final_launch_runtime_probe",
            context=self._context,
        )
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._clients = []

    def tearDown(self) -> None:
        for client in self._clients:
            self._node.destroy_client(client)
        self._executor.remove_node(self._node)
        self._executor.shutdown()
        self._node.destroy_node()
        if self._context.ok():
            rclpy.shutdown(context=self._context)

    def _node_counts(self) -> Counter[str]:
        return Counter(
            _fully_qualified_name(name, namespace)
            for name, namespace in self._node.get_node_names_and_namespaces()
        )

    def _wait_for_core_nodes(self, deadline: float) -> Counter[str]:
        counts: Counter[str] = Counter()
        while time.monotonic() < deadline:
            counts = self._node_counts()
            # Humble's MoveIt executable creates an internal node with the
            # public name as well as ``move_group_private_*``. The launch file
            # itself has one move_group process/action (covered structurally),
            # while ROS graph enumeration legitimately reports both internals.
            if (
                counts["/move_group"] >= 1
                and all(counts[node_name] == 1 for node_name in _CORE_NODES)
            ):
                return counts
            self._executor.spin_once(timeout_sec=_POLL_PERIOD_S)

        self.fail(
            "Timed out waiting for exactly one instance of each final-launch "
            f"core node. Observed node counts: {dict(sorted(counts.items()))}"
        )

    def test_core_nodes_and_moveit_services_are_reachable(self) -> None:
        deadline = time.monotonic() + _STARTUP_TIMEOUT_S
        self._wait_for_core_nodes(deadline)

        for service_name, service_type in _MOVEIT_SERVICES:
            client = self._node.create_client(service_type, service_name)
            self._clients.append(client)
            remaining = deadline - time.monotonic()
            self.assertGreater(
                remaining,
                0.0,
                f"Startup timeout elapsed before {service_name} became available",
            )
            self.assertTrue(
                client.wait_for_service(timeout_sec=remaining),
                f"MoveIt service {service_name} was not reachable within "
                f"{_STARTUP_TIMEOUT_S:.0f} seconds",
            )

        while time.monotonic() < self._viewer_check_not_before:
            rclpy.spin_once(self._node, timeout_sec=_POLL_PERIOD_S)

        counts = self._node_counts()
        self.assertGreaterEqual(counts["/move_group"], 1)
        for node_name in _CORE_NODES:
            self.assertEqual(
                counts[node_name],
                1,
                f"Expected exactly one {node_name}; graph was {dict(sorted(counts.items()))}",
            )
        self.assertNotIn(
            "/mujoco_joint_state_viewer",
            counts,
            "The Viewer must not start in the headless runtime test",
        )

    def test_oscbf_controller_closes_the_safe_loop(self) -> None:
        """M11 e2e: the launched controller publishes a valid safe state."""
        received = []

        def _on_state(message: JointState) -> None:
            received.append(message)

        self._node.create_subscription(
            JointState,
            "/mujoco_joint_states",
            _on_state,
            qos_profile_sensor_data,
        )
        plant_pub = self._node.create_publisher(
            JointState, "/mujoco_joint_states", qos_profile_sensor_data
        )

        plant = JointState()
        plant.name = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"]
        plant.position = [
            0.2303562, 0.1112539, 1.0167209, -0.6810303, -1.8294025,
            -0.4664294, 0.4743473, -1.0429228, 0.0289233,
        ]
        plant.position[0] += 0.01
        plant_positions = np.asarray(plant.position, dtype=float)

        # The controller needs its JIT warm-up plus a control tick; the
        # deadline is generous because the launched graph shares the CPU.
        deadline = time.monotonic() + 360.0
        output = None
        while time.monotonic() < deadline:
            plant_pub.publish(plant)
            self._executor.spin_once(timeout_sec=0.5)
            for message in received:
                positions = np.asarray(message.position, dtype=float)
                if positions.shape == (9,) and np.max(
                    np.abs(positions - plant_positions)
                ) > 1e-6:
                    output = message
                    break
            if output is not None:
                break

        self.assertIsNotNone(
            output, "no safe state received from oscbf_controller"
        )
        assert output.name == ["J1", "J2", "J3", "J4", "J5",
                               "J6", "J7", "J8", "J9"]
        positions = np.asarray(output.position, dtype=float)
        assert np.all(np.isfinite(positions))

        log_dir = Path(__file__).resolve().parents[1] / "output"
        log_dir.mkdir(parents=True, exist_ok=True)
        (log_dir / "oscbf_m11_e2e.log").write_text(
            "M11 end-to-end: oscbf_controller published a valid 9-joint "
            f"safe state on /mujoco_joint_states (q0={positions[0]:.6f} m) "
            "in the headless launch.\n",
            encoding="utf-8",
        )


if __name__ == "__main__":
    unittest.main()
