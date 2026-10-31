from math import isfinite
from pathlib import Path
import time

from construct import Bytes, Const, Int16ub, Struct, Terminated
import serial

from .drempower_can import (
    CMD_POSITION_ANGLE_MODE1, CMD_PROPERTY_READ, CMD_PROPERTY_WRITE, CMD_SYSTEM,
    SYSTEM_ORDER_SAVE, MotorFeedback, can_id, decode_feedback, decode_property_reply,
    encode_property_read, encode_property_write, encode_system, encode_trapezoid_position,
)
from .motor_motion_limits import MotionBounds, feedback_speed_limit, validate_motion_parameters


# 厂家 DrEmpower_can.py 的 can_to_uart/uart_to_can 标准模式布局。
SERIAL_CAN_FRAME = Struct(
    "header" / Const(bytes.fromhex("aa0000080000")),
    "frame_id" / Int16ub,
    "data" / Bytes(8),
    Terminated,
)


class DrEmpowerSerial:
    def __init__(
        self, port: str, *, command_node: int | None = None, axis_type: str = "revolute",
        position_nodes: dict[int, str] | None = None,
        motion_bounds: dict[int, MotionBounds] | None = None,
    ):
        if not Path(port).exists():
            raise FileNotFoundError(port)
        feedback_speed_limit(axis_type)
        if position_nodes is not None:
            if command_node is not None or position_nodes != {
                1: "prismatic", **{node: "revolute" for node in range(2, 10)},
            } or any(type(node) is not int for node in position_nodes):
                raise ValueError("九轴位置连接必须明确包含 ID 1–9 及对应关节类型")
        if command_node is not None:
            valid_node = (command_node == 1 if axis_type == "prismatic"
                          else type(command_node) is int and 2 <= command_node <= 9)
            if type(command_node) is not int or not valid_node:
                raise ValueError("直线运动必须指定 ID 1；旋转运动必须指定 ID 2–9")
        self.command_node = command_node
        self.axis_type = axis_type
        self.position_nodes = dict(position_nodes or {})
        self.motion_bounds = dict(motion_bounds or {})
        if self.motion_bounds and (not self.position_nodes or set(self.motion_bounds) != set(self.position_nodes)):
            raise ValueError("调试参数范围必须对应完整九轴位置连接")
        self._reply_states: dict[int, bool] = {}
        self.READ_FLAG = 0
        self.enable_replay_state = 0
        self._pending = None
        self._failed = False
        self._latest: dict[int, tuple[MotorFeedback, float]] = {}
        self.motor_state: dict[int, list[float | int]] = {}
        self.frames: list[dict[str, str | float]] = []
        self.reply_state_enabled: bool | None = None
        self.uart = serial.Serial(port, 115200, timeout=0.08, write_timeout=0.08, exclusive=True)
        if self.uart.in_waiting:
            self.uart.close()
            raise RuntimeError("串口存在未请求数据")

    def _write_frame(self, node, cmd, data):
        if self._failed or self._pending is not None or self.uart.in_waiting:
            raise RuntimeError("串口会话无效或存在未处理数据")
        if type(node) is not int or not 1 <= node <= 9:
            raise ValueError("仅支持单节点 ID 1–9")
        if cmd != CMD_PROPERTY_READ:
            if node in self.position_nodes:
                if cmd != CMD_POSITION_ANGLE_MODE1:
                    raise ValueError("九轴位置连接仅允许单节点位置模式 1")
            elif node != self.command_node:
                raise ValueError("连接没有该节点的写入权限")
            allowed = {CMD_POSITION_ANGLE_MODE1}
            if self.axis_type == "prismatic":
                allowed.update((CMD_PROPERTY_WRITE, CMD_SYSTEM))
            if cmd not in allowed:
                raise ValueError("不支持该写入命令")
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
        if (mode != 1 or (id_num not in self.position_nodes
                          and (self.command_node is None or id_num != self.command_node))):
            raise ValueError("连接没有该节点的梯形位置命令权限")
        if not all(isfinite(value) for value in (angle, speed, param)):
            raise ValueError("运动参数必须为有限数值")
        validate_motion_parameters(speed, param, axis_type=self.position_nodes.get(id_num, self.axis_type),
                                   bounds=self.motion_bounds.get(id_num))
        if id_num not in self._latest:
            raise RuntimeError("发送前缺少实际反馈")
        feedback, stamp = self._latest[id_num]
        if time.monotonic() - stamp > 0.2 or feedback.axis_error:
            raise RuntimeError("发送前反馈超时或存在驱动故障")
        reply_enabled = self._reply_states.get(id_num) if self.position_nodes else self.reply_state_enabled
        if reply_enabled is None:
            raise RuntimeError("发送前必须核对设备自动反馈设置")
        payload = encode_trapezoid_position(id_num, angle, speed=speed, accel=param)
        self._write_frame(id_num, CMD_POSITION_ANGLE_MODE1, payload)
        if reply_enabled:
            self._pending = id_num
            raw = self.receive_data(return_id=True)
            acknowledgement = decode_feedback((raw[1] << 8) | raw[2], bytes(raw[3:]))
            if acknowledgement.axis_error:
                raise RuntimeError("位置命令回复报告驱动故障")
        return True

    def read_property(self, node: int, address: int, kind: str = "u32"):
        self.send_command(node, CMD_PROPERTY_READ, encode_property_read(node, address, kind))
        raw = self.receive_data(return_id=True)
        return decode_property_reply((raw[1] << 8) | raw[2], bytes(raw[3:]),
                                     node_id=node, address=address, value_kind=kind)

    def configure_j1_limits(self, angle_min: float, angle_max: float):
        if self.command_node != 1 or self.axis_type != "prismatic":
            raise ValueError("配置 J1 限位需要明确的直线轴连接")
        if (not all(isfinite(value) for value in (angle_min, angle_max))
                or abs(angle_max - angle_min - 36000) > 1e-6):
            raise ValueError("J1 限位必须对应已确认的 500 mm 行程")
        if 1 not in self._latest:
            raise RuntimeError("限位配置前缺少实际反馈")
        feedback, stamp = self._latest[1]
        if (time.monotonic() - stamp > 0.2 or feedback.axis_error or not feedback.traj_done
                or not angle_min - 0.01 <= feedback.pos_deg <= angle_max + 0.01):
            raise RuntimeError("限位配置前反馈或当前位置无效")
        for address, value, kind in ((38010, angle_min, "f32"), (38011, angle_max, "f32"),
                                     (31202, 1, "u32")):
            self._write_frame(1, CMD_PROPERTY_WRITE, encode_property_write(1, address, value, kind))
            time.sleep(0.02)
            actual = self.read_property(1, address, kind)
            if actual != value:
                raise RuntimeError(f"J1 参数 {address} 回读不符：{actual}，目标 {value}")
        self._write_frame(1, CMD_SYSTEM, encode_system(1, SYSTEM_ORDER_SAVE))
        time.sleep(0.02)
        return {"min_deg": self.read_property(1, 38010, "f32"),
                "max_deg": self.read_property(1, 38011, "f32"),
                "enabled": self.read_property(1, 31202)}

    def prepare_motion(self, node: int | None = None):
        node = self.command_node if node is None else node
        if node is None or (node not in self.position_nodes and node != self.command_node):
            raise ValueError("读取连接没有运动节点")
        self.send_command(node, CMD_PROPERTY_READ, encode_property_read(node, 31214))
        raw = self.receive_data(return_id=True)
        value = decode_property_reply((raw[1] << 8) | raw[2], bytes(raw[3:]),
                                      node_id=node, address=31214)
        if value not in (0, 1):
            raise ValueError("设备自动反馈设置无效")
        if self.position_nodes:
            self._reply_states[node] = bool(value)
        else:
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
