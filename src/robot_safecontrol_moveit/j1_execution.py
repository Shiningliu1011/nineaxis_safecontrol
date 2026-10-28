from dataclasses import dataclass
from decimal import Decimal
from math import isfinite
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from .drempower_joint_control import JointFeedback
from .motor_motion_limits import MAX_J1_FEEDBACK_RPM, validate_motion_parameters
from .single_joint_execution import ExecutionLimits
from .unit_conversion import (
    JointCalibrationTable, JointNodeMap, PerJointCalibration, TransmissionSpec, UnitConverter,
)


@dataclass(frozen=True)
class J1ScrewControl:
    converter: UnitConverter
    lower_mm: float
    upper_mm: float
    max_step_mm: float
    axis_type = "prismatic"

    @classmethod
    def from_root(cls, root: Path):
        with (root / "config/j1_transmission.yaml").open(encoding="utf-8") as stream:
            spec = yaml.safe_load(stream)
        expected = {"lead_mm_per_rev", "ratio_motor_rev_per_lead_rev", "lower_mm", "upper_mm", "max_step_mm"}
        if not isinstance(spec, dict) or set(spec) != expected:
            raise ValueError("J1 传动配置字段无效")
        if not all(type(value) in (float, int) and isfinite(value) for value in spec.values()):
            raise ValueError("J1 传动配置必须为有限数值")
        if spec != dict(lead_mm_per_rev=5, ratio_motor_rev_per_lead_rev=1,
                        lower_mm=0, upper_mm=500, max_step_mm=50):
            raise ValueError("J1 配置与用户确认的传动、行程或单次位移不符")
        with (root / "config/hardware_joint_zero.yaml").open(encoding="utf-8") as stream:
            calibration = yaml.safe_load(stream)["J1"]
        if (set(calibration) != {"sign", "zero_offset_deg"} or calibration["sign"] != -1
                or type(calibration["zero_offset_deg"]) not in (float, int)
                or not isfinite(calibration["zero_offset_deg"])):
            raise ValueError("J1 零位或方向配置无效")
        with (root / "config/drempower.yaml").open(encoding="utf-8") as stream:
            nodes = yaml.safe_load(stream)["/**"]["ros__parameters"]["node_ids"]
        node_map = JointNodeMap(nodes)
        if node_map.node_of("J1") != 1:
            raise ValueError("J1 节点必须为本次确认的 ID 1")
        joints = [joint for joint in ET.parse(root / "models/ninezzhou/urdf/ninezzhou.urdf")
                  .getroot().findall("joint") if joint.get("name") == "J1"]
        if len(joints) != 1 or joints[0].get("type") != "prismatic":
            raise ValueError("模型必须提供唯一的 J1 移动关节")
        limit = joints[0].find("limit")
        if limit is None or not {"lower", "upper"} <= limit.attrib.keys():
            raise ValueError("J1 模型缺少位置范围")
        if (float(limit.attrib["lower"]) != spec["lower_mm"] / 1000
                or float(limit.attrib["upper"]) != spec["upper_mm"] / 1000):
            raise ValueError("模型行程与已确认的 J1 运行范围不符")
        origin, axis = joints[0].find("origin"), joints[0].find("axis")
        if (origin is None or axis is None
                or [float(value) for value in origin.get("xyz", "").split()] != [0, 0, 0.085]
                or [float(value) for value in origin.get("rpy", "").split()] != [0, 0, 0]
                or [float(value) for value in axis.get("xyz", "").split()] != [0, 0, 1]):
            raise ValueError("J1 模型起点或轴向与已确认的坐标定义不符")
        converter = UnitConverter(
            node_map=node_map,
            calibration=JointCalibrationTable.from_entries([PerJointCalibration("J1", **calibration)]),
            j1_transmission=TransmissionSpec(spec["lead_mm_per_rev"], spec["ratio_motor_rev_per_lead_rev"]),
        )
        return cls(converter, spec["lower_mm"], spec["upper_mm"], spec["max_step_mm"])

    @property
    def degrees_per_mm(self):
        return 360 / (self.converter.j1_transmission.m_per_motor_rev * 1000)

    def _node(self, joint):
        if joint != "J1":
            raise ValueError("该执行接口只支持 J1")
        return self.converter.node_map.node_of(joint)

    def position_arguments(self, joint, angle_deg, *, speed_rpm, accel_rpm_s):
        node = self._node(joint)
        validate_motion_parameters(speed_rpm, accel_rpm_s, axis_type=self.axis_type)
        position_mm = angle_deg / self.degrees_per_mm
        if not isfinite(position_mm) or not self.lower_mm <= position_mm <= self.upper_mm:
            raise ValueError("J1 目标位置必须在 0–500 mm 内")
        target = self.converter.joint_to_encoder(joint, position_mm / 1000)
        return dict(id_num=node, angle=target, speed=speed_rpm, param=accel_rpm_s, mode=1)

    def decode_state(self, joint, angle_deg, speed_rpm):
        self._node(joint)
        if not isfinite(speed_rpm):
            raise ValueError("J1 速度反馈必须为有限数值")
        position_mm = self.converter.encoder_to_joint(joint, angle_deg) * 1000
        sign = self.converter.calibration.for_joint(joint).sign
        return JointFeedback(position_mm * self.degrees_per_mm, sign * speed_rpm)

    def set_angle(self, vendor, joint, angle_deg, *, speed_rpm, accel_rpm_s):
        arguments = self.position_arguments(joint, angle_deg, speed_rpm=speed_rpm,
                                            accel_rpm_s=accel_rpm_s)
        if vendor.set_angle(**arguments) is not True:
            raise RuntimeError("J1 位置命令失败")

    def limits(self, *, speed_mm_s: float, accel_mm_s2: float):
        rpm_per_mm_s = self.degrees_per_mm / 6
        return ExecutionLimits(
            # 测量范围计入既有 0.01° 噪声；发送目标仍严格检查 0–500 mm。
            lower_deg=self.lower_mm * self.degrees_per_mm - 0.01,
            upper_deg=self.upper_mm * self.degrees_per_mm + 0.01,
            max_step_deg=self.max_step_mm * self.degrees_per_mm,
            speed_rpm=float(Decimal(str(speed_mm_s)) * Decimal(str(rpm_per_mm_s))),
            accel_rpm_s=float(Decimal(str(accel_mm_s2)) * Decimal(str(rpm_per_mm_s))),
            max_feedback_rpm=MAX_J1_FEEDBACK_RPM,
            axis_type=self.axis_type,
        )
