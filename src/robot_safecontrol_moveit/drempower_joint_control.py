from dataclasses import dataclass
from math import degrees, isfinite, radians
from pathlib import Path
from types import ModuleType

import yaml

from .unit_conversion import (
    JOINT_NAMES, ROTARY_JOINTS, JointCalibrationTable, JointNodeMap,
    PerJointCalibration, UnitConverter,
)


@dataclass(frozen=True)
class JointFeedback:
    angle_deg: float
    speed_rpm: float


class DrEmpowerJointControl:
    # 本接口转换坐标；执行保护由调用程序负责。构造时不打开串口。
    def __init__(self, calibration_path: Path, node_map: JointNodeMap):
        with Path(calibration_path).open(encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        if not isinstance(data, dict) or set(data) != set(JOINT_NAMES):
            raise ValueError("方向配置必须完整包含 J1–J9")
        entries = []
        for joint, entry in data.items():
            if not isinstance(entry, dict) or set(entry) != {"sign", "zero_offset_deg"}:
                raise ValueError(f"{joint} 标定字段无效")
            sign = entry["sign"]
            offset = entry["zero_offset_deg"]
            if type(sign) is not int or sign not in (-1, 1):
                raise ValueError(f"{joint} sign 必须为整数 ±1")
            if type(offset) not in (int, float) or not isfinite(offset):
                raise ValueError(f"{joint} zero_offset_deg 必须为有限数值")
            entries.append(PerJointCalibration(joint, sign, offset))
        self.converter = UnitConverter(
            node_map=node_map,
            calibration=JointCalibrationTable.from_entries(entries),
        )

    def _node(self, joint: str) -> int:
        if joint not in ROTARY_JOINTS:
            raise ValueError("此接口仅支持 J2–J9")
        return self.converter.node_map.node_of(joint)

    def position_arguments(
        self, joint: str, angle_deg: float, *, speed_rpm: float, accel_rpm_s: float,
    ) -> dict:
        node = self._node(joint)
        for name, value in (("speed_rpm", speed_rpm), ("accel_rpm_s", accel_rpm_s)):
            if not isfinite(value) or not 0.01 <= value < 300:
                raise ValueError(f"{name} 必须在 [0.01, 300) 内")
        target = self.converter.joint_to_encoder(joint, radians(angle_deg))
        return dict(id_num=node, angle=target, speed=speed_rpm, param=accel_rpm_s, mode=1)

    def decode_state(self, joint: str, angle_deg: float, speed_rpm: float) -> JointFeedback:
        self._node(joint)
        if not isfinite(speed_rpm):
            raise ValueError("电机速度反馈必须为有限数值")
        angle = degrees(self.converter.encoder_to_joint(joint, angle_deg))
        sign = self.converter.calibration.for_joint(joint).sign
        return JointFeedback(angle, sign * speed_rpm)

    def get_state(self, vendor: ModuleType, joint: str) -> JointFeedback:
        node = self._node(joint)
        state = vendor.get_state(id_num=node)
        if vendor.READ_FLAG != 1 or not isinstance(state, (list, tuple)) or len(state) != 2:
            raise RuntimeError(f"{joint} 厂家库未返回有效反馈")
        if vendor.motor_state[node - 1][4] != 0:
            raise RuntimeError(f"{joint} 电机报告 axis_error")
        return self.decode_state(joint, state[0], state[1])

    def set_angle(
        self, vendor: ModuleType, joint: str, angle_deg: float,
        *, speed_rpm: float, accel_rpm_s: float,
    ) -> None:
        arguments = self.position_arguments(
            joint, angle_deg, speed_rpm=speed_rpm, accel_rpm_s=accel_rpm_s,
        )
        if vendor.set_angle(**arguments) is not True:
            raise RuntimeError(f"{joint} 厂家库位置命令失败")

    def step_angle(
        self, vendor: ModuleType, joint: str, delta_deg: float,
        *, speed_rpm: float, accel_rpm_s: float,
    ) -> None:
        # 读取前完成输入校验；使用单节点绝对目标表达相对运动。
        self.position_arguments(
            joint, delta_deg, speed_rpm=speed_rpm, accel_rpm_s=accel_rpm_s,
        )
        state = self.get_state(vendor, joint)
        self.set_angle(
            vendor, joint, state.angle_deg + delta_deg,
            speed_rpm=speed_rpm, accel_rpm_s=accel_rpm_s,
        )
