"""Validate that model files are installable and resolvable.

These tests verify that setup.py correctly lists model files for
installation and that they can be resolved through the standard
ament_python package share mechanism.
"""

import unittest
from pathlib import Path


_PROJECT = Path(__file__).resolve().parents[1]
_MODELS = _PROJECT / "models"
_URDF = _MODELS / "ninezzhou" / "urdf" / "ninezzhou.urdf"
_MESHES_DIR = _MODELS / "ninezzhou" / "meshes"
_MOVEIT_CONFIG_DIR = _MODELS / "ninezzhou_moveit_config" / "config"


class TestModelFilesExist(unittest.TestCase):
    """Verify model files are present in the source tree."""

    def test_urdf_exists(self) -> None:
        self.assertTrue(_URDF.is_file(), f"URDF not found at {_URDF}")

    def test_urdf_contains_base_link(self) -> None:
        text = _URDF.read_text()
        self.assertIn('name="base_link"', text)

    def test_urdf_contains_tool0(self) -> None:
        text = _URDF.read_text()
        self.assertIn('name="tool0"', text)

    def test_urdf_contains_joints(self) -> None:
        text = _URDF.read_text()
        for j in range(1, 10):
            self.assertIn(f'name="J{j}"', text, f"J{j} not in URDF")

    def test_mesh_directory_exists(self) -> None:
        self.assertTrue(_MESHES_DIR.is_dir(), f"Mesh dir not found at {_MESHES_DIR}")

    def test_mesh_directory_has_stl_files(self) -> None:
        stls = list(_MESHES_DIR.glob("*.STL"))
        self.assertGreater(len(stls), 0, f"No STL files in {_MESHES_DIR}")


class TestMoveItConfigFilesExist(unittest.TestCase):
    """Verify MoveIt configuration files are present in the source tree."""

    def test_srdf_exists(self) -> None:
        self.assertTrue(
            (_MOVEIT_CONFIG_DIR / "ninezzhou.srdf").is_file(),
            "SRDF not found",
        )

    def test_kinematics_yaml_exists(self) -> None:
        self.assertTrue(
            (_MOVEIT_CONFIG_DIR / "kinematics.yaml").is_file(),
            "kinematics.yaml not found",
        )

    def test_ompl_planning_yaml_exists(self) -> None:
        self.assertTrue(
            (_MOVEIT_CONFIG_DIR / "ompl_planning.yaml").is_file(),
            "ompl_planning.yaml not found",
        )

    def test_joint_limits_yaml_exists(self) -> None:
        self.assertTrue(
            (_MOVEIT_CONFIG_DIR / "joint_limits.yaml").is_file(),
            "joint_limits.yaml not found",
        )

    def test_srdf_defines_arm_group(self) -> None:
        text = (_MOVEIT_CONFIG_DIR / "ninezzhou.srdf").read_text()
        self.assertIn('group name="arm"', text)

    def test_srdf_defines_chain(self) -> None:
        text = (_MOVEIT_CONFIG_DIR / "ninezzhou.srdf").read_text()
        self.assertIn('base_link="base_link"', text)
        self.assertIn('tip_link="tool0"', text)


class TestSetupPyDeclaresModels(unittest.TestCase):
    """Verify setup.py declares model files for installation."""

    @classmethod
    def setUpClass(cls) -> None:
        cls._text = (_PROJECT / "setup.py").read_text()

    def test_urdf_in_data_files(self) -> None:
        self.assertIn("ninezzhou/urdf", self._text)

    def test_meshes_in_data_files(self) -> None:
        self.assertIn("ninezzhou/meshes", self._text)

    def test_moveit_config_in_data_files(self) -> None:
        self.assertIn("ninezzhou_moveit_config/config", self._text)

    def test_glob_models_used(self) -> None:
        # setup.py should use _glob_models helper to find model files.
        self.assertIn("_glob_models", self._text)


class TestMoveItRuntimeConfig(unittest.TestCase):
    """Verify MoveItRuntimeConfig can load from source tree.

    These tests validate the config loader using the source-tree paths
    (not the installed share directory), since we may not have done
    ``colcon build`` yet.
    """

    def test_can_construct_from_share_dir(self) -> None:
        from robot_safecontrol_moveit.moveit_runtime_config import (
            build_moveit_params,
            load_urdf,
            load_srdf,
            validate_urdf_frames,
        )
        # Use the source tree as a simulated share directory.
        share = _PROJECT
        urdf = load_urdf(share)
        self.assertIn("base_link", urdf)
        validate_urdf_frames(urdf)  # should not raise

        srdf = load_srdf(share)
        self.assertIn("arm", srdf)

        params = build_moveit_params(share)
        self.assertIn("robot_description", params)
        self.assertIn("robot_description_semantic", params)
        self.assertIn("robot_description_kinematics", params)
        self.assertIn("robot_description_planning", params)

    def test_validate_frames_raises_on_missing(self) -> None:
        from robot_safecontrol_moveit.moveit_runtime_config import (
            validate_urdf_frames,
        )
        with self.assertRaises(ValueError) as ctx:
            validate_urdf_frames("<robot></robot>")
        self.assertIn("ROBOT_MODEL_FRAME_MISSING", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
