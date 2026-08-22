"""Production-configuration contract for the AEB-RRT* MoveIt pipeline.

The default transition path must name the concrete PlannerManager plugin and
the concrete planner configuration.  Merely keeping the C++ implementation in
the repository is not sufficient: MoveIt must receive these parameters through
the same loader used by ``mujoco_transition_final.launch.py``.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import yaml


_PROJECT = Path(__file__).resolve().parents[1]
_SRC = _PROJECT / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from robot_safecontrol_moveit.moveit_runtime_config import (  # noqa: E402
    build_moveit_params,
)


_AEB_PLUGIN = "aeb_rrtstar_ompl/AEBRRTstarPlannerManager"
_AEB_PLANNER_ID = "AEBRRTstarFaithfulConfigDefault"
_AEB_PLANNER_TYPE = "geometric::AEBRRTstarFaithful"


def _yaml(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


class TestAEBRRTstarRuntimeContract(unittest.TestCase):
    """The launch-time parameter graph must select the AEB faithful mode."""

    @classmethod
    def setUpClass(cls) -> None:
        cls._ompl_path = (
            _PROJECT
            / "models"
            / "ninezzhou_moveit_config"
            / "config"
            / "ompl_planning.yaml"
        )
        cls._runtime_path = _PROJECT / "config" / "mujoco_transition_runtime.yaml"
        cls._ompl = _yaml(cls._ompl_path)
        cls._runtime = _yaml(cls._runtime_path)["transition_planning_server"][
            "ros__parameters"
        ]

    def test_moveit_pipeline_loads_the_aeb_planner_manager(self) -> None:
        self.assertEqual(self._ompl["planning_plugin"], _AEB_PLUGIN)

    def test_faithful_planner_configuration_is_registered_for_arm(self) -> None:
        faithful = self._ompl["planner_configs"][_AEB_PLANNER_ID]
        self.assertEqual(faithful["type"], _AEB_PLANNER_TYPE)
        self.assertTrue(faithful["stop_on_first_solution"])
        self.assertEqual(
            faithful["simplify_solutions"],
            "0",
            "Faithful AEB paths must not be rewritten by OMPL's generic simplifier",
        )
        self.assertEqual(faithful["interpolate"], "1")
        self.assertEqual(faithful["longest_valid_segment_fraction"], 0.001)

        arm_configs = self._ompl["arm"]["planner_configs"]
        self.assertEqual(
            arm_configs[0],
            _AEB_PLANNER_ID,
            "The first arm planner is the MoveIt default when a request omits planner_id",
        )
        self.assertIn(
            "RRTConnectkConfigDefault",
            arm_configs,
            "Keep an explicit, known rollback planner available",
        )

    def test_persistent_transition_server_requests_aeb_faithful(self) -> None:
        self.assertEqual(self._runtime["planning_pipeline"], "ompl")
        self.assertEqual(self._runtime["planner_id"], _AEB_PLANNER_ID)
        self.assertEqual(self._runtime["planning_attempts"], 1)

    def test_launch_parameter_builder_preserves_the_aeb_configuration(self) -> None:
        """Exercise the actual loader consumed by the final launch file."""
        parameters = build_moveit_params(_PROJECT)

        self.assertEqual(parameters["default_planning_pipeline"], "ompl")
        self.assertEqual(parameters["ompl"]["planning_plugin"], _AEB_PLUGIN)
        self.assertEqual(
            parameters["ompl"]["planner_configs"][_AEB_PLANNER_ID]["type"],
            _AEB_PLANNER_TYPE,
        )


if __name__ == "__main__":
    unittest.main()
