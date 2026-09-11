"""DrEmpower CAN 帧编解码器（纯函数，无 I/O / 无 ROS）。

协议事实来源：本地厂家 CAN 通讯协议 v2.0（第7–9页）与
linux-socketcan v1.0 的 interface_enums.py；设备固件兼容性仍待现场核验。

    CAN ID = (node_id << 5) | cmd_byte      # 11-bit standard frame
    node_id 1..63，0 = 广播
    位置命令 0x19: data[0:4] float32 目标角 (deg 小端)
                  data[4:6] int16  速度/时间 (*100)
                  data[6:8] int16  滤波/加速度 (*100)
    系统命令 0x08: data[0:4] uint32 order_num
    属性读 0x1E / 属性写 0x1F：地址 u16 + 类型码 u16 + 值/补零
    本地厂家 linux-socketcan v1.0 profile；尚不证明设备固件兼容。
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

# 厂家 interface_enums.py：current_state=30002，requested_state=30003。
PROP_AXIS_REQUESTED_STATE = 30003
PROP_AXIS_ERROR = 30001

PROPERTY_TYPES = {"f32": (0, "f"), "u16": (1, "H"), "s16": (2, "h"),
                  "u32": (3, "I"), "s32": (4, "i")}

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
    scaled = float(value) / 0.01
    if not math.isfinite(scaled):
        raise ValueError(f"{name} 必须是有限值，got {value}")
    rounded = int(scaled)
    if not -32768 <= rounded <= 32767:
        raise ValueError(f"{name}*100 超出 int16 范围: {rounded}")
    return rounded


def encode_position(
    node_id: int, target_deg: float, *, speed: float, filter_accel: float
) -> bytes:
    """0x19 位置命令（轨迹跟踪模式）→ 8 字节数据。

    ``speed`` 为 rpm，历史参数名 ``filter_accel`` 在 mode0 表示输入滤波带宽。
    按厂家源码取绝对值并截断；带宽超过300显式拒绝，避免厂家静默截断。
    """
    if not math.isfinite(target_deg):
        raise ValueError(f"target_deg 必须是有限值，got {target_deg}")
    can_id(node_id, CMD_POSITION_ANGLE_MODE0)
    if abs(filter_accel) > 300:
        raise ValueError("mode0 input filter bandwidth must be <= 300")
    return struct.pack(
        "<fhh",
        float(target_deg),
        _int16_scaled(abs(speed), "speed"),
        _int16_scaled(abs(filter_accel), "filter_accel"),
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


def _property_type(value_kind: str) -> tuple[int, str]:
    try:
        return PROPERTY_TYPES[value_kind]
    except KeyError:
        raise ValueError(f"unknown value_kind: {value_kind}") from None


def encode_property_read(node_id: int, address: int, value_kind: str = "u32") -> bytes:
    """Typed property read. Quick-state request is separately all-zero bytes."""
    can_id(node_id, CMD_PROPERTY_READ)
    code, _ = _property_type(value_kind)
    return struct.pack("<HHI", _as_u16(address, "address"), code, 0)


def encode_property_write(
    node_id: int, address: int, value: Value, value_kind: str
) -> bytes:
    """Vendor v1.0: address u16, type u16, value, zero padding to 8 bytes."""
    can_id(node_id, CMD_PROPERTY_WRITE)
    code, fmt = _property_type(value_kind)
    if not math.isfinite(float(value)):
        raise ValueError("property value must be finite")
    if fmt != "f" and int(value) != value:
        raise ValueError("integer property requires an integer value")
    try:
        body = struct.pack("<" + fmt, value if fmt == "f" else int(value))
    except (struct.error, OverflowError) as exc:
        raise ValueError("property value out of range") from exc
    return (struct.pack("<HH", _as_u16(address, "address"), code) + body).ljust(8, b"\x00")


def decode_property_reply(frame_id: int, data: bytes, *, node_id: int,
                          address: int, value_kind: str = "u32") -> Value:
    """Reject wrong node, command, address, type or frame length."""
    code, fmt = _property_type(value_kind)
    if (frame_id != can_id(node_id, CMD_PROPERTY_READ) or len(data) != 8
            or data[:4] != struct.pack("<HH", address, code)):
        raise ValueError("unexpected property response")
    value = struct.unpack_from("<" + fmt, data, 4)[0]
    if not math.isfinite(float(value)):
        raise ValueError("nonfinite property response")
    return value


def decode_feedback(frame_id: int, data: bytes) -> MotorFeedback:
    """反馈帧解码：位置 (deg) / 速度 (rpm*0.01) / 转矩 (Nm*0.01) + 状态位。"""
    if not 0 < frame_id <= 0x7FF or frame_id >> 5 == 0 or not frame_id & 1:
        raise ValueError("feedback requires a nonbroadcast standard ID with bit0 set")
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
                node_id, PROP_AXIS_REQUESTED_STATE, AXIS_STATE_CLOSED_LOOP, "u32"
            ),
        ),
    ]


def disable_sequence(node_id: int) -> list[tuple[int, bytes]]:
    """失能序列（厂商流程）：写 requested_state=IDLE。"""
    return [
        (
            can_id(node_id, CMD_PROPERTY_WRITE),
            encode_property_write(
                node_id, PROP_AXIS_REQUESTED_STATE, AXIS_STATE_IDLE, "u32"
            ),
        ),
    ]
