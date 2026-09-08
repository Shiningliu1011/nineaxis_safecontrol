"""Launch-description behaviour tests using real launch action objects."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

import yaml
from launch import LaunchContext
from launch.actions import TimerAction
from launch_ros.actions import Node


_PROJECT = Path(__file__).resolve().parents[1]
_SRC = _PROJECT / "src"
_LAUNCH_DIR = _PROJECT / "launch"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))


def _load_launch_module(filename: str):
    path = _LAUNCH_DIR / filename
    module_name = f"_launch_test_{path.stem.replace('.', '_')}"
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot import launch file: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _make_share_directory() -> tempfile.TemporaryDirectory[str]:
    temporary = tempfile.TemporaryDirectory()
    config = Path(temporary.name) / "config"
    config.mkdir()
    (config / "mujoco_transition_runtime.yaml").touch()
    return temporary


def _render(substitutions) -> str:
    context = LaunchContext()
    return "".join(
        substitution.perform(context) for substitution in substitutions
    )


def _all_nodes(actions) -> list[Node]:
    nodes: list[Node] = []
    for action in actions:
        if isinstance(action, Node):
            nodes.append(action)
        nested_actions = getattr(action, "actions", ())
        if nested_actions:
            nodes.extend(_all_nodes(nested_actions))
    return nodes


def _node_identity(node: Node) -> tuple[str, str, str]:
    # launch_ros keeps these immutable launch-time values internally until it
    # expands substitutions against a LaunchContext.
    return (
        node._Node__package,
        node._Node__node_executable,
        node._Node__node_name,
    )


def _node_remappings(node: Node) -> set[tuple[str, str]]:
    return {
        (_render(source), _render(target))
        for source, target in node._Node__remappings
    }


def _node_parameters(node: Node) -> dict[str, object]:
    parameters: dict[str, object] = {}
    for parameter_set in node._Node__parameters:
        if not isinstance(parameter_set, dict):
            continue
        for key, value in parameter_set.items():
            rendered_key = _render(key)
            if isinstance(value, tuple):
                # launch_ros serialises scalar parameter values as YAML before
                # passing them to the node, including an explicit document end.
                parameters[rendered_key] = yaml.safe_load(_render(value))
            else:
                parameters[rendered_key] = value
    return parameters


class TestFinalLaunchDescription(unittest.TestCase):
    """Validate final-launch topology by instantiating its action graph."""

    def _description(self):
        module = _load_launch_module("mujoco_transition_final.launch.py")
        temporary = _make_share_directory()
        self.addCleanup(temporary.cleanup)
        module.get_package_share_directory = lambda _: temporary.name
        module.build_moveit_params = lambda share_dir: {
            "robot_description": "<robot name='test_robot'/>"
        }
        return module.generate_launch_description()

    def test_final_launch_creates_the_expected_node_topology(self) -> None:
        description = self._description()
        nodes = _all_nodes(description.entities)
        identities = [_node_identity(node) for node in nodes]

        self.assertEqual(
            identities.count(
                ("moveit_ros_move_group", "move_group", "move_group")
            ),
            1,
        )
        self.assertEqual(
            identities.count(
                (
                    "robot_state_publisher",
                    "robot_state_publisher",
                    "robot_state_publisher",
                )
            ),
            1,
        )
        self.assertEqual(
            identities.count(
                (
                    "robot_safecontrol_moveit",
                    "transition_planning_server",
                    "transition_planning_server",
                )
            ),
            1,
        )
        self.assertEqual(
            identities.count(
                (
                    "robot_safecontrol_moveit",
                    "mujoco_viewer",
                    "mujoco_joint_state_viewer",
                )
            ),
            1,
        )
        self.assertEqual(
            identities.count(
                (
                    "robot_safecontrol_moveit",
                    "oscbf_controller",
                    "oscbf_controller",
                )
            ),
            1,
        )
        self.assertEqual(
            identities.count(
                (
                    "robot_safecontrol_moveit",
                    "oscbf_plant",
                    "oscbf_plant",
                )
            ),
            1,
        )
        self.assertEqual(
            identities.count(
                (
                    "robot_safecontrol_moveit",
                    "hardware_bridge",
                    "hardware_bridge",
                )
            ),
            1,
        )
        self.assertEqual(
            identities.count(
                (
                    "robot_safecontrol_moveit",
                    "perception_bridge",
                    "perception_bridge",
                )
            ),
            1,
        )
        self.assertTrue(
            any(
                isinstance(action, TimerAction)
                for action in description.entities
            )
        )

    def test_start_oscbf_controller_argument_defaults_to_true(self) -> None:
        description = self._description()
        arguments = {
            action.name: _render(action.default_value)
            for action in description.entities
            if hasattr(action, "name") and hasattr(action, "default_value")
        }
        self.assertIn("start_oscbf_controller", arguments)
        self.assertEqual(arguments["start_oscbf_controller"], "true")
        self.assertIn("start_oscbf_plant", arguments)
        self.assertEqual(arguments["start_oscbf_plant"], "false")
        self.assertIn("oscbf_wait_for_start", arguments)
        self.assertEqual(arguments["oscbf_wait_for_start"], "false")
        self.assertIn("transition_replay_topic", arguments)
        self.assertEqual(arguments["transition_replay_topic"], "/mujoco_joint_states")
        self.assertIn("oscbf_randomize_start", arguments)
        self.assertEqual(arguments["oscbf_randomize_start"], "false")
        self.assertIn("auto_plan_once", arguments)
        self.assertEqual(arguments["auto_plan_once"], "true")
        self.assertIn("hardware_mode", arguments)
        self.assertEqual(arguments["hardware_mode"], "sim")
        self.assertIn("start_perception", arguments)
        self.assertEqual(arguments["start_perception"], "false")

    def test_move_group_rsp_and_server_share_mujoco_joint_state_remap(self) -> None:
        description = self._description()
        nodes = {
            (node._Node__package, node._Node__node_executable): node
            for node in _all_nodes(description.entities)
        }
        expected_remap = ("joint_states", "/mujoco_joint_states")

        self.assertIn(
            expected_remap,
            _node_remappings(nodes[("moveit_ros_move_group", "move_group")]),
        )
        self.assertIn(
            expected_remap,
            _node_remappings(
                nodes[("robot_state_publisher", "robot_state_publisher")]
            ),
        )
        self.assertIn(
            expected_remap,
            _node_remappings(
                nodes[(
                    "robot_safecontrol_moveit",
                    "transition_planning_server",
                )]
            ),
        )


class TestViewerOnlyLaunchDescription(unittest.TestCase):
    """The standalone Viewer launch should construct only the Viewer node."""

    def test_viewer_launch_constructs_one_viewer_node(self) -> None:
        module = _load_launch_module("mujoco_viewer.launch.py")
        temporary = _make_share_directory()
        self.addCleanup(temporary.cleanup)
        module.get_package_share_directory = lambda _: temporary.name

        description = module.generate_launch_description()
        nodes = _all_nodes(description.entities)

        self.assertEqual(
            [_node_identity(node) for node in nodes],
            [
                (
                    "robot_safecontrol_moveit",
                    "mujoco_viewer",
                    "mujoco_joint_state_viewer",
                )
            ],
        )


if __name__ == "__main__":
    unittest.main()
