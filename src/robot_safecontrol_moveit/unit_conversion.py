"""单位换算与零位偏移——纯逻辑，无 ROS / 无 I/O。

真机坐标系约定：

    电机编码器读数（输出轴 deg，含零位偏移与方向符号）
    → 关节 URDF 坐标系（J1: m；J2–J9: rad）

换算公式（每关节独立）：

    q_urdf = sign * (reading_deg - zero_offset_deg) * scale
    scale：旋转轴 = π/180 (rad/deg)；J1 = 丝杆换算 m_per_deg

**J1 契约**：J1 是棱柱关节，其 deg↔m 换算依赖丝杆传动实测（导程、
二级传动比）。``j1_transmission`` 未注入时本模块拒绝一切 J1 换算
（fail-closed），与安全网关的 hardware-executable 语义一致。

零位标定（机械零位标记法）：手动摆机械零位后 set_zero 固化电机零位，
再把每关节的符号与（若电机零位 ≠ 机械零位）偏移量写入标定表；
本模块只消费标定表，不生成它。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Mapping

import numpy as np

from .robot_spec import DEFAULT_JOINT_NAMES

RAD_PER_DEG = float(np.pi / 180.0)
JOINT_NAMES: tuple[str, ...] = DEFAULT_JOINT_NAMES
ROTARY_JOINTS = tuple(f"J{i}" for i in range(2, 10))


class J1TransmissionMissingError(RuntimeError):
    """J1 丝杆传动未标定：任何 J1 单位换算都被拒绝。"""


def _validate_joint(joint: str) -> str:
    if joint not in JOINT_NAMES:
        raise ValueError(f"未知关节 {joint!r}; 有效: {', '.join(JOINT_NAMES)}")
    return joint


@dataclass(frozen=True)
class JointNodeMap:
    """关节 ↔ CAN 节点地址映射（9 关节，节点 1..63 且唯一）。"""

    node_by_joint: Mapping[str, int]

    def __post_init__(self) -> None:
        if set(self.node_by_joint) != set(JOINT_NAMES):
            raise ValueError(
                "映射必须恰好包含 9 个关节: " + ", ".join(JOINT_NAMES))
        values = [int(v) for v in self.node_by_joint.values()]
        if len(set(values)) != len(values):
            raise ValueError("node_id 必须唯一")
        if any(not 1 <= v <= 63 for v in values):
            raise ValueError("node_id 必须在 [1, 63]")

    @classmethod
    def from_joint_list(cls, order: Iterable[str]) -> "JointNodeMap":
        """按关节顺序分配 1..N 的默认映射（推荐 J1–J9 = 1–9，实读后固化）。"""
        joints = tuple(_validate_joint(j) for j in order)
        if len(joints) != len(JOINT_NAMES):
            raise ValueError(f"需要 {len(JOINT_NAMES)} 个关节，got {len(joints)}")
        return cls({joint: index + 1 for index, joint in enumerate(joints)})

    def node_of(self, joint: str) -> int:
        if joint not in self.node_by_joint:
            raise KeyError(joint)
        return int(self.node_by_joint[joint])

    def joint_of(self, node_id: int) -> str:
        for joint, value in self.node_by_joint.items():
            if int(value) == int(node_id):
                return joint
        raise KeyError(f"node_id {node_id} 未映射")


@dataclass(frozen=True)
class TransmissionSpec:
    """J1 丝杆传动参数（用户实测后注入；来源：参考项目实机计划 §3）。"""

    lead_mm_per_rev: float          # 丝杆导程：丝杆转一圈推进的毫米数
    ratio_motor_rev_per_lead_rev: float = 1.0  # 电机转数 / 丝杆转数 (>1: 电机更快)
    efficiency: float = 1.0         # 传动效率（力换算用；线速度不受影响）

    def __post_init__(self) -> None:
        if not 0.0 < float(self.lead_mm_per_rev):
            raise ValueError("lead_mm_per_rev 必须为正")
        if not 0.0 < float(self.ratio_motor_rev_per_lead_rev):
            raise ValueError("ratio_motor_rev_per_lead_rev 必须为正")
        if not 0.0 < float(self.efficiency) <= 1.0:
            raise ValueError("efficiency 必须在 (0, 1]")

    @property
    def m_per_motor_rev(self) -> float:
        """电机输出轴转一圈 → J1 平移距离（米）。"""
        return float(self.lead_mm_per_rev) / 1000.0 / float(
            self.ratio_motor_rev_per_lead_rev)


@dataclass(frozen=True)
class PerJointCalibration:
    """单关节标定：方向符号 + 电机零位偏移（编码器读数 deg → URDF 零位）。"""

    joint: str
    sign: int = 1
    zero_offset_deg: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "joint", _validate_joint(self.joint))
        if int(self.sign) not in (1, -1):
            raise ValueError(f"sign 必须为 ±1，got {self.sign}")
        object.__setattr__(self, "sign", int(self.sign))
        object.__setattr__(self, "zero_offset_deg", float(self.zero_offset_deg))


@dataclass(frozen=True)
class JointCalibrationTable:
    """逐关节标定表；缺失关节回退默认（sign=1, offset=0）。"""

    entries: Mapping[str, PerJointCalibration]

    @classmethod
    def from_entries(cls, entries: Iterable[PerJointCalibration]) -> "JointCalibrationTable":
        table = {}
        for entry in entries:
            if entry.joint in table:
                raise ValueError(f"关节 {entry.joint} 标定重复")
            table[entry.joint] = entry
        return cls(table)

    def for_joint(self, joint: str) -> PerJointCalibration:
        return self.entries.get(_validate_joint(joint), PerJointCalibration(joint))


class UnitConverter:
    """编码器读数（deg）⇄ 关节 URDF 值（J1: m / J2–J9: rad）。"""

    def __init__(
        self, *, node_map: JointNodeMap,
        calibration: JointCalibrationTable,
        j1_transmission: TransmissionSpec | None = None,
    ) -> None:
        self.node_map = node_map
        self.calibration = calibration
        self.j1_transmission = j1_transmission

    @property
    def is_j1_ready(self) -> bool:
        return self.j1_transmission is not None

    def _rad_per_encoder_deg(self, joint: str) -> float:
        if joint == "J1":
            if self.j1_transmission is None:
                raise J1TransmissionMissingError(
                    "J1 丝杆传动未标定：禁止转换（fail-closed）；"
                    "注入 TransmissionSpec（导程/传动比实测）后放行")
            return float(self.j1_transmission.m_per_motor_rev / 360.0)
        return RAD_PER_DEG

    def encoder_to_joint(self, joint: str, reading_deg: float) -> float:
        """电机读数 deg → 关节 URDF 值。"""
        entry = self.calibration.for_joint(joint)
        reading = float(reading_deg)
        if not np.isfinite(reading):
            raise ValueError(f"{joint} 读数必须有限: {reading}")
        return entry.sign * (reading - entry.zero_offset_deg) \
            * self._rad_per_encoder_deg(entry.joint)

    def joint_to_encoder(self, joint: str, q: float) -> float:
        """关节 URDF 值 → 电机读数 deg。"""
        entry = self.calibration.for_joint(joint)
        value = float(q)
        if not np.isfinite(value):
            raise ValueError(f"{joint} 值必须有限: {value}")
        return entry.sign * value / self._rad_per_encoder_deg(entry.joint) \
            + entry.zero_offset_deg

    def encoder_to_joints(self, readings_deg: np.ndarray) -> np.ndarray:
        readings = np.asarray(readings_deg, dtype=float).reshape(-1)
        if readings.shape != (len(JOINT_NAMES),):
            raise ValueError(f"读数形状必须为 ({len(JOINT_NAMES)},)")
        return np.array(
            [self.encoder_to_joint(joint, float(reading))
             for joint, reading in zip(JOINT_NAMES, readings)])

    def joints_to_encoder(self, q: np.ndarray) -> np.ndarray:
        values = np.asarray(q, dtype=float).reshape(-1)
        if values.shape != (len(JOINT_NAMES),):
            raise ValueError(f"值形状必须为 ({len(JOINT_NAMES)},)")
        return np.array(
            [self.joint_to_encoder(joint, float(value))
             for joint, value in zip(JOINT_NAMES, values)])
