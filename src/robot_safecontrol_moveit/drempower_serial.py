from math import isfinite
from pathlib import Path
import time

from construct import Bytes, Const, Int16ub, Struct, Terminated
import serial

from .drempower_can import (
    CMD_POSITION_ANGLE_MODE1, CMD_PROPERTY_READ, can_id, decode_feedback,
    decode_property_reply, encode_property_read, encode_trapezoid_position,
)


# 厂家 DrEmpower_can.py 的 can_to_uart/uart_to_can 标准模式布局。
SERIAL_CAN_FRAME = Struct(
    "header" / Const(bytes.fromhex("aa0000080000")),
    "frame_id" / Int16ub,
    "data" / Bytes(8),
    Terminated,
)


class DrEmpowerSerial:
    def __init__(self, port: str, *, command_node: int | None = None):
        if not Path(port).exists():
            raise FileNotFoundError(port)
        if command_node is not None and (type(command_node) is not int or not 2 <= command_node <= 9):
            raise ValueError("运动连接必须明确指定单个 ID 2–9")
        self.command_node = command_node
        self.READ_FLAG = 0
        self.enable_replay_state = 0
        self._pending = None
        self._failed = False
        self._latest = {}
        self.motor_state = {}
        self.frames = []
        self.reply_state_enabled = None
        self.uart = serial.Serial(port, 115200, timeout=0.08, write_timeout=0.08, exclusive=True)
        if self.uart.in_waiting:
            self.uart.close()
            raise RuntimeError("串口存在未请求数据")

    def _write_frame(self, node, cmd, data):
        if self._failed or self._pending is not None or self.uart.in_waiting:
            raise RuntimeError("串口会话无效或存在未处理数据")
        if type(node) is not int or not 2 <= node <= 9:
            raise ValueError("仅支持单节点 ID 2–9")
        request = SERIAL_CAN_FRAME.build(dict(frame_id=can_id(node, cmd), data=bytes(data)))
        self.frames.append(dict(direction="tx", monotonic_s=time.monotonic(), raw_hex=request.hex()))
        self._failed = True
        if self.uart.write(request) != len(request):
            raise RuntimeError("串口发送不完整")
        self._failed = False

    def send_command(self, id_num, cmd, data, rtr=0):
        if cmd != CMD_PROPERTY_READ or rtr != 0:
            raise ValueError("原始发送接口仅允许属性读取")
        self._write_frame(id_num, cmd, data)
        self._pending = id_num
        self.READ_FLAG = 0

    def receive_data(self, return_id=False):
        if self._pending is None or self._failed:
            raise RuntimeError("没有有效的待读取请求")
        # 不完整或迟到的事务禁止自动重试，防止旧回复被作为新反馈。
        self._failed = True
        raw = self.uart.read(16)
        self.frames.append(dict(direction="rx", monotonic_s=time.monotonic(), raw_hex=raw.hex()))
        parsed = SERIAL_CAN_FRAME.parse(raw)
        if parsed.frame_id > 0x7FF or parsed.frame_id >> 5 != self._pending:
            raise ValueError("反馈节点或 CAN ID 无效")
        if parsed.frame_id & 0x1F in (1, 3, 5, 7):
            feedback = decode_feedback(parsed.frame_id, parsed.data)
            self._latest[feedback.node_id] = (feedback, time.monotonic())
            self.motor_state[feedback.node_id - 1] = [
                feedback.pos_deg, feedback.vel_rpm, feedback.torque_nm,
                int(feedback.traj_done), int(feedback.axis_error),
            ]
        self._pending = None
        self._failed = False
        self.READ_FLAG = 1
        result = [8, parsed.frame_id >> 8, parsed.frame_id & 0xFF, *parsed.data]
        return result if return_id else result[3:]

    def set_angle(self, id_num, angle, speed, param, mode):
        if self.command_node is None or id_num != self.command_node or mode != 1:
            raise ValueError("连接没有该节点的梯形位置命令权限")
        if not all(isfinite(value) for value in (angle, speed, param)):
            raise ValueError("运动参数必须为有限数值")
        if not 0.01 <= speed <= 0.1 or not 0.01 <= param <= 0.1:
            raise ValueError("单关节检查速度及加速度上限为 0.1")
        if id_num not in self._latest:
            raise RuntimeError("发送前缺少实际反馈")
        feedback, stamp = self._latest[id_num]
        if time.monotonic() - stamp > 0.2 or feedback.axis_error:
            raise RuntimeError("发送前反馈超时或存在驱动故障")
        if abs(angle - feedback.pos_deg) > 5.00001:
            raise ValueError("电机目标与当前反馈相差超过 5°")
        if self.reply_state_enabled is None:
            raise RuntimeError("发送前必须核对设备自动反馈设置")
        payload = encode_trapezoid_position(id_num, angle, speed=speed, accel=param)
        self._write_frame(id_num, CMD_POSITION_ANGLE_MODE1, payload)
        if self.reply_state_enabled:
            self._pending = id_num
            raw = self.receive_data(return_id=True)
            acknowledgement = decode_feedback((raw[1] << 8) | raw[2], bytes(raw[3:]))
            if acknowledgement.axis_error:
                raise RuntimeError("位置命令回复报告驱动故障")
        return True

    def prepare_motion(self):
        if self.command_node is None:
            raise ValueError("读取连接没有运动节点")
        self.send_command(self.command_node, CMD_PROPERTY_READ,
                          encode_property_read(self.command_node, 31214))
        raw = self.receive_data(return_id=True)
        value = decode_property_reply((raw[1] << 8) | raw[2], bytes(raw[3:]),
                                      node_id=self.command_node, address=31214)
        if value not in (0, 1):
            raise ValueError("设备自动反馈设置无效")
        self.reply_state_enabled = bool(value)

    def get_state(self, id_num):
        self.send_command(id_num, CMD_PROPERTY_READ, bytes(8))
        raw = self.receive_data(return_id=True)
        frame_id = (raw[1] << 8) | raw[2]
        if frame_id & 0x1F not in (1, 3, 5, 7):
            raise ValueError("快速状态回复命令类型无效")
        feedback = decode_feedback(frame_id, bytes(raw[3:]))
        return [feedback.pos_deg, feedback.vel_rpm]

    def close(self):
        self.uart.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
