from pathlib import Path
import sys

import numpy as np
import pytest
import yaml
from urdf_parser_py.urdf import URDF


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "portable_oscbf"))

from robot_safecontrol_moveit.robot_spec import DEFAULT_JOINT_NAMES
from work.actuator_limits import JOINT_NAMES, load_actuator_limit_profile
from work.ik_data_loader import load_kinematics_config
from work.kinematics_data import JOINT_POSITION_LOWER, JOINT_POSITION_UPPER
from work.nineaxis_kinematics import NineaxisKinematics


URDF_PATH = ROOT / "models" / "ninezzhou" / "urdf" / "ninezzhou.urdf"
NINEAXIS_PATH = ROOT / "portable_oscbf" / "config" / "nineaxis.yaml"
ROBOT_PARAMS_PATH = ROOT / "portable_oscbf" / "config" / "robot_params.yaml"
ACTUATOR_PATH = ROOT / "portable_oscbf" / "config" / "actuator_modules.yaml"


def test_runtime_joint_identity_and_limits_match_project_sources():
    robot = URDF.from_xml_string(URDF_PATH.read_bytes())
    active = [
        joint for joint in robot.joints if joint.type in {"prismatic", "revolute"}
    ]
    names = tuple(joint.name for joint in active)
    lower = np.asarray([joint.limit.lower for joint in active], dtype=float)
    upper = np.asarray([joint.limit.upper for joint in active], dtype=float)

    assert names == DEFAULT_JOINT_NAMES == JOINT_NAMES
    np.testing.assert_array_equal(lower, JOINT_POSITION_LOWER)
    np.testing.assert_array_equal(upper, JOINT_POSITION_UPPER)

    kinematics = NineaxisKinematics()
    np.testing.assert_array_equal(kinematics.joint_limits.q_min, lower)
    np.testing.assert_array_equal(kinematics.joint_limits.q_max, upper)

    profile = load_actuator_limit_profile(ACTUATOR_PATH)
    assert profile.joint_names == names
    assert profile.source_path == ACTUATOR_PATH
    np.testing.assert_array_equal(
        kinematics.joint_limits.dq_max, profile.velocity_limits
    )
    np.testing.assert_array_equal(
        kinematics.joint_limits.ddq_max, profile.acceleration_limits
    )

    assert load_kinematics_config(NINEAXIS_PATH)["ee_center"] == [0.0, 0.343, 1.387]
    assert "joint_limits" not in yaml.safe_load(NINEAXIS_PATH.read_text(encoding="utf-8"))
    assert "joint_limits" not in yaml.safe_load(ROBOT_PARAMS_PATH.read_text(encoding="utf-8"))


def test_trajectory_config_rejects_joint_order_change(tmp_path):
    document = yaml.safe_load(NINEAXIS_PATH.read_text(encoding="utf-8"))
    document["joint_names"][0], document["joint_names"][1] = (
        document["joint_names"][1],
        document["joint_names"][0],
    )
    path = tmp_path / "nineaxis.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")

    with pytest.raises(ValueError, match="joint_names"):
        load_kinematics_config(path)


def test_trajectory_config_rejects_handwritten_joint_limits(tmp_path):
    document = yaml.safe_load(NINEAXIS_PATH.read_text(encoding="utf-8"))
    document["joint_limits"] = {"q_min": [0.0] * len(JOINT_NAMES)}
    path = tmp_path / "nineaxis.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")

    with pytest.raises(ValueError, match="generated joint limits"):
        load_kinematics_config(path)
