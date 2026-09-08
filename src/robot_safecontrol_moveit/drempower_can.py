"""DrEmpower CAN 帧编解码器（纯函数，无 I/O / 无 ROS）。

协议事实来源：参考项目对新协议的摘要审查（real_robot_hardware_source_review.md）：

    CAN ID = (node_id << 5) | cmd_byte      # 11-bit standard frame
    node_id 1..63，0 = 广播
    位置命令 0x19: data[0:4] float32 目标角 (deg 小端)
                  data[4:6] int16  速度/时间 (*100)
                  data[6:8] int16  滤波/加速度 (*100)
    系统命令 0x08: data[0:4] uint32 order_num
    属性读 0x1E / 属性写 0x1F：地址 u16 + 类型化值（布局为本模块假定，见
    ``encode_property_write`` docstring；待厂商库确认（P1）后锁定）
    反馈帧: data[0:4] float32 位置 (deg)
            data[4:6] int16  速度 (rpm * 0.01)
            data[6:8] int16  转矩 (Nm * 0.01)
            响应 CAN ID bit1=traj_done, bit2=axis_error

本模块只做字节级编解码与校验；发送/接收由后端负责。
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Union

# --- 帧布局常量 ---------------------------------------------------------------
NODE_SHIFT = 5
MAX_NODE_ID = 63
BROADCAST_NODE_ID = 0

CMD_POSITION_ANGLE_MODE0 = 0x19  # 位置模式 0：轨迹跟踪
CMD_PRESET_ANGLE = 0x0C          # 多机同步预置角
CMD_SYSTEM = 0x08                 # 系统命令
CMD_PROPERTY_READ = 0x1E
CMD_PROPERTY_WRITE = 0x1F

# 系统命令 order_num（data[0:4]）
SYSTEM_ORDER_SAVE = 0x01
SYSTEM_ORDER_REBOOT = 0x03
SYSTEM_ORDER_CLEAR_ERROR = 0x04
SYSTEM_ORDER_SET_ZERO = 0x05
SYSTEM_ORDER_ESTOP = 0x06
SYSTEM_ORDER_START_ANGLE_TRACKING = 0x10
SYSTEM_ORDER_START_SPEED = 0x13

# 属性地址表（键值来自协议摘要；地址分布待厂商库确认）
PROP_VBUS_VOLTAGE = 1
PROP_AXIS_CURRENT_STATE = 30002
PROP_AXIS_CONFIG_CAN_NODE_ID = 31001
PROP_AXIS_ENCODER_POS = 35001

# 使能/失能流程需要的属性地址：协议摘要只给了属性名
# （axis.requested_state）未列地址。此常量是本模块对 30002 的沿用假定，
# 待厂商库（P1）确认；不得作为已核实硬件参数执行。
PROP_AXIS_REQUESTED_STATE = 30002

AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP = 8

# 反馈状态位（响应 CAN ID 上）
FLAG_TRAJ_DONE = 0x02
FLAG_AXIS_ERROR = 0x04

_FRAME_LEN = 8

Value = Union[int, float]


@dataclass(frozen=True)
class MotorFeedback:
    """解码后的电机反馈帧（输出轴）。"""

    node_id: int
    pos_deg: float
    vel_rpm: float
    torque_nm: float
    traj_done: bool
    axis_error: bool


def can_id(node_id: int, cmd_byte: int) -> int:
    """标准 11-bit 帧 ID：``(node_id << 5) | cmd_byte``。"""
    if not 0 <= node_id <= MAX_NODE_ID:
        raise ValueError(f"node_id 必须在 [0, {MAX_NODE_ID}]，got {node_id}")
    if not 0 <= cmd_byte <= 0x1F:
        raise ValueError(f"cmd_byte 必须为 5-bit，got {cmd_byte:#x}")
    return (node_id << NODE_SHIFT) | cmd_byte


def can_node_id(frame_id: int) -> tuple[int, int]:
    """从 CAN ID 还原 (node_id, cmd_byte)。"""
    return (frame_id >> NODE_SHIFT) & MAX_NODE_ID, frame_id & 0x1F


def _int16_scaled(value: float, name: str) -> int:
    scaled = float(value) * 100.0
    if not math.isfinite(scaled):
        raise ValueError(f"{name} 必须是有限值，got {value}")
    rounded = int(round(scaled))
    if not -32768 <= rounded <= 32767:
        raise ValueError(f"{name}*100 超出 int16 范围: {rounded}")
    return rounded


def encode_position(
    node_id: int, target_deg: float, *, speed: float, filter_accel: float
) -> bytes:
    """0x19 位置命令（轨迹跟踪模式）→ 8 字节数据。

    ``speed``/``filter_accel`` 协议单位为 *100 编码。字段单位与取值范围
    以厂商库（P1）为准，锁定后不许变动。
    """
    if not math.isfinite(target_deg):
        raise ValueError(f"target_deg 必须是有限值，got {target_deg}")
    return struct.pack(
        "<fhh",
        float(target_deg),
        _int16_scaled(speed, "speed"),
        _int16_scaled(filter_accel, "filter_accel"),
    )


def encode_system(node_id: int, order_num: int) -> bytes:
    """0x08 系统命令：order_num 写入 data[0:4]（小端 u32），其余零。"""
    if not 0 <= int(order_num) <= 0xFFFFFFFF:
        raise ValueError(f"order_num 必须为 u32，got {order_num}")
    return struct.pack("<I", int(order_num)) + b"\x00" * 4


def _as_u16(value: int, name: str) -> int:
    if not 0 <= int(value) <= 0xFFFF:
        raise ValueError(f"{name} 必须为 u16，got {value}")
    return int(value)


def encode_property_read(node_id: int, address: int) -> bytes:
    """0x1E 属性读帧：地址 u16 写入 data[0:2]（布局假定，待厂商库确认）。"""
    return struct.pack("<H", _as_u16(address, "address")) + b"\x00" * 6


def encode_property_write(
    node_id: int, address: int, value: Value, value_kind: str
) -> bytes:
    """0x1F 属性写帧：data[0:2]=u16 地址，随后是类型化值：
    ``u16`` → data[2:4]；``u32`` → data[2:6]；``f32`` → data[2:6]。

    除地址外，值区布局为本模块假定（协议摘要未给出 0x1F 数据布局），
    待厂商库（P1）确认后锁定。
    """
    address_value = _as_u16(address, "address")
    if value_kind == "u16":
        if not 0 <= int(value) <= 0xFFFF:
            raise ValueError(f"u16 值越界: {value}")
        body = struct.pack("<H", int(value)) + b"\x00" * 4
    elif value_kind == "u32":
        if not 0 <= int(value) <= 0xFFFFFFFF:
            raise ValueError(f"u32 值越界: {value}")
        body = struct.pack("<I", int(value)) + b"\x00" * 2
    elif value_kind == "f32":
        if not math.isfinite(float(value)):
            raise ValueError(f"f32 值必须有限: {value}")
        body = struct.pack("<f", float(value)) + b"\x00" * 2
    else:
        raise ValueError(f"未知 value_kind: {value_kind}")
    return struct.pack("<H", address_value) + body


def decode_feedback(frame_id: int, data: bytes) -> MotorFeedback:
    """反馈帧解码：位置 (deg) / 速度 (rpm*0.01) / 转矩 (Nm*0.01) + 状态位。"""
    if len(data) != _FRAME_LEN:
        raise ValueError(f"反馈帧长必须为 {_FRAME_LEN}，got {len(data)}")
    pos_deg, vel_code, torque_code = struct.unpack("<fhh", data)
    if not all(math.isfinite(v) for v in (pos_deg, vel_code, torque_code)):
        raise ValueError("反馈帧含非有限值")
    node_id, cmd_byte = can_node_id(frame_id)
    return MotorFeedback(
        node_id=node_id,
        pos_deg=float(pos_deg),
        vel_rpm=float(vel_code) * 0.01,
        torque_nm=float(torque_code) * 0.01,
        traj_done=bool(frame_id & FLAG_TRAJ_DONE),
        axis_error=bool(frame_id & FLAG_AXIS_ERROR),
    )


def enable_sequence(node_id: int) -> list[tuple[int, bytes]]:
    """使能序列（厂商流程）：clear_error → 写 requested_state=CLOSED_LOOP。"""
    return [
        (can_id(node_id, CMD_SYSTEM), encode_system(node_id, SYSTEM_ORDER_CLEAR_ERROR)),
        (
            can_id(node_id, CMD_PROPERTY_WRITE),
            encode_property_write(
                node_id, PROP_AXIS_REQUESTED_STATE, AXIS_STATE_CLOSED_LOOP, "u16"
            ),
        ),
    ]


def disable_sequence(node_id: int) -> list[tuple[int, bytes]]:
    """失能序列（厂商流程）：写 requested_state=IDLE。"""
    return [
        (
            can_id(node_id, CMD_PROPERTY_WRITE),
            encode_property_write(
                node_id, PROP_AXIS_REQUESTED_STATE, AXIS_STATE_IDLE, "u16"
            ),
        ),
    ]
