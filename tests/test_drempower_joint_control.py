from math import nan
from pathlib import Path

import pytest
import yaml

from robot_safecontrol_moveit.drempower_joint_control import DrEmpowerJointControl
from robot_safecontrol_moveit.unit_conversion import JOINT_NAMES, JointNodeMap


CONFIG = Path(__file__).resolve().parents[1] / "config/hardware_joint_zero.yaml"


def control():
    return DrEmpowerJointControl(CONFIG, JointNodeMap.from_joint_list(JOINT_NAMES))


@pytest.mark.parametrize("joint,sign", [
    ("J2", -1), ("J3", 1), ("J4", -1), ("J5", 1),
    ("J6", -1), ("J7", -1), ("J8", -1), ("J9", -1),
])
def test_real_configuration_command_and_feedback(joint, sign):
    api = control()
    offset = api.converter.calibration.for_joint(joint).zero_offset_deg
    for angle in (-5, 5):
        args = api.position_arguments(joint, angle, speed_rpm=0.1, accel_rpm_s=0.1)
        assert args == dict(id_num=int(joint[1:]), angle=pytest.approx(offset + sign * angle),
                            speed=0.1, param=0.1, mode=1)
        state = api.decode_state(joint, args["angle"], sign * 0.1)
        assert state.angle_deg == pytest.approx(angle)
        assert state.speed_rpm == pytest.approx(0.1)


@pytest.mark.parametrize("joint,angle,speed", [
    ("J1", 5, 0.1), ("J10", 5, 0.1), ("J2", nan, 0.1),
    ("J2", 5, 0), ("J2", 5, -1), ("J2", 5, nan), ("J2", 5, 300),
])
def test_invalid_command_rejected(joint, angle, speed):
    with pytest.raises(ValueError):
        control().position_arguments(joint, angle, speed_rpm=speed, accel_rpm_s=0.1)


def test_feedback_rejects_nonfinite():
    with pytest.raises(ValueError):
        control().decode_state("J2", nan, 0)
    with pytest.raises(ValueError):
        control().decode_state("J2", 0, nan)


def test_relative_target_preserves_nonzero_offset(tmp_path):
    data = yaml.safe_load(CONFIG.read_text())
    data["J2"]["zero_offset_deg"] = 37.0
    path = tmp_path / "calibration.yaml"
    path.write_text(yaml.safe_dump(data))
    api = DrEmpowerJointControl(path, JointNodeMap.from_joint_list(JOINT_NAMES))
    state = api.decode_state("J2", 42, 0)
    args = api.position_arguments("J2", state.angle_deg + 5, speed_rpm=0.1, accel_rpm_s=0.1)
    assert args["angle"] == pytest.approx(37)


@pytest.mark.parametrize("value", [0, 1.5, True, "-1"])
def test_invalid_sign_configuration_rejected(tmp_path, value):
    data = yaml.safe_load(CONFIG.read_text())
    data["J2"]["sign"] = value
    path = tmp_path / "calibration.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        DrEmpowerJointControl(path, JointNodeMap.from_joint_list(JOINT_NAMES))
