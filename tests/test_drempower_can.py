"""DrEmpower CAN 帧编解码器测试。

纯函数层（无 ROS / 无 I/O），覆盖协议摘要中的帧布局：
CAN ID = (node_id << 5) | cmd_byte、0x19 位置命令、0x08 系统命令、
0x1E/0x1F 属性读写、反馈帧解码、使能/失能序列。
"""

import struct

import numpy as np
import pytest

from robot_safecontrol_moveit.drempower_can import (
    AXIS_STATE_CLOSED_LOOP,
    AXIS_STATE_IDLE,
    CMD_POSITION_ANGLE_MODE0,
    CMD_PROPERTY_READ,
    CMD_PROPERTY_WRITE,
    CMD_SYSTEM,
    SYSTEM_ORDER_CLEAR_ERROR,
    SYSTEM_ORDER_ESTOP,
    SYSTEM_ORDER_SET_ZERO,
    MotorFeedback,
    can_id,
    can_node_id,
    decode_feedback,
    disable_sequence,
    enable_sequence,
    encode_position,
    encode_property_read,
    encode_property_write,
    encode_system,
)


class TestCanId:
    def test_formula(self) -> None:
        # (3 << 5) | 0x19 = 0x79
        assert can_id(3, CMD_POSITION_ANGLE_MODE0) == 0x79

    def test_broadcast(self) -> None:
        # 广播节点 0：CAN ID 只剩命令字节
        assert can_id(0, CMD_SYSTEM) == 0x08

    def test_roundtrip(self) -> None:
        assert can_node_id(can_id(9, 0x1C)) == (9, 0x1C)

    def test_out_of_range_rejected(self) -> None:
        with pytest.raises(ValueError):
            can_id(64, CMD_SYSTEM)
        with pytest.raises(ValueError):
            can_id(1, 0x20)


class TestEncodePosition:
    def test_bytes_match_protocol(self) -> None:
        data = encode_position(1, 90.0, speed=2.0, filter_accel=5.0)
        assert data == struct.pack("<fhh", 90.0, 200, 500)
        assert len(data) == 8
        # float32(90.0) == 0x42B40000，小端为 00 00 B4 42
        assert data[0:4] == b"\x00\x00\xb4\x42"

    def test_negative_and_high_speed(self) -> None:
        data = encode_position(2, -33.5, speed=0.06, filter_accel=-1.0)
        assert data == struct.pack("<fhh", -33.5, 6, -100)

    def test_invalid_rejected(self) -> None:
        with pytest.raises(ValueError):
            encode_position(1, float("nan"), speed=0.0, filter_accel=0.0)
        with pytest.raises(ValueError):
            encode_position(1, 90.0, speed=400.0, filter_accel=0.0)


class TestEncodeSystem:
    def test_clear_error(self) -> None:
        assert encode_system(1, SYSTEM_ORDER_CLEAR_ERROR) == (
            struct.pack("<I", 0x04) + b"\x00" * 4
        )

    def test_set_zero_and_estop(self) -> None:
        assert encode_system(0, SYSTEM_ORDER_SET_ZERO) == (
            struct.pack("<I", 0x05) + b"\x00" * 4
        )
        assert encode_system(0, SYSTEM_ORDER_ESTOP) == (
            struct.pack("<I", 0x06) + b"\x00" * 4
        )

    def test_invalid_order_rejected(self) -> None:
        with pytest.raises(ValueError):
            encode_system(1, -1)
        with pytest.raises(ValueError):
            encode_system(1, 2**32)


class TestFeedbackDecode:
    def test_roundtrip_values(self) -> None:
        data = struct.pack("<fhh", 180.0, 318, -25)
        # 响应 ID 带状态位：bit1=traj_done, bit2=axis_error
        resp_id = can_id(5, CMD_POSITION_ANGLE_MODE0) | 0x02 | 0x04
        fb = decode_feedback(resp_id, data)
        assert isinstance(fb, MotorFeedback)
        assert fb.node_id == 5
        assert fb.pos_deg == pytest.approx(180.0, abs=1e-4)
        assert fb.vel_rpm == pytest.approx(3.18, abs=1e-6)
        assert fb.torque_nm == pytest.approx(-0.25, abs=1e-6)
        assert fb.traj_done is True
        assert fb.axis_error is True

    def test_status_bits_clear(self) -> None:
        data = struct.pack("<fhh", 0.0, 0, 0)
        # 0x19 = 0b11001：bit1/bit2 均为 0，不带状态位
        fb = decode_feedback(can_id(1, CMD_POSITION_ANGLE_MODE0), data)
        assert fb.traj_done is False
        assert fb.axis_error is False

    def test_malformed_rejected(self) -> None:
        with pytest.raises(ValueError):
            decode_feedback(0x1E, b"\x00" * 7)
        with pytest.raises(ValueError):
            decode_feedback(0x1E, struct.pack("<fhh", float("inf"), 0, 0))


class TestPropertyFrames:
    def test_read_frame_layout(self) -> None:
        data = encode_property_read(3, 31001)
        assert data == struct.pack("<H", 31001) + b"\x00" * 6

    def test_write_u16_layout(self) -> None:
        data = encode_property_write(3, 31001, 3, "u16")
        assert data == struct.pack("<HH", 31001, 3) + b"\x00" * 4

    def test_write_u32_layout(self) -> None:
        data = encode_property_write(3, 30001, 123, "u32")
        assert data == struct.pack("<H", 30001) + struct.pack("<I", 123) + b"\x00" * 2

    def test_write_f32_layout(self) -> None:
        data = encode_property_write(3, 35001, 34.5, "f32")
        assert data == struct.pack("<H", 35001) + struct.pack("<f", 34.5) + b"\x00" * 2

    def test_invalid_kind_rejected(self) -> None:
        with pytest.raises(ValueError):
            encode_property_write(3, 31001, 1, "bool")


class TestEnableDisableSequence:
    def test_enable_order(self) -> None:
        frames = enable_sequence(2)
        assert len(frames) == 2
        # 先清错，再写 requested_state=8（闭环）
        assert frames[0] == (
            can_id(2, CMD_SYSTEM), encode_system(2, SYSTEM_ORDER_CLEAR_ERROR)
        )
        assert frames[1][0] == can_id(2, CMD_PROPERTY_WRITE)
        assert struct.unpack("<HH", frames[1][1][0:4]) == (30002, AXIS_STATE_CLOSED_LOOP)

    def test_disable_order(self) -> None:
        frames = disable_sequence(2)
        assert len(frames) == 1
        assert frames[0][0] == can_id(2, CMD_PROPERTY_WRITE)
        assert struct.unpack("<HH", frames[0][1][0:4]) == (30002, AXIS_STATE_IDLE)


def test_protocol_constants_are_commitments() -> None:
    """命令字与量纲（*100 编码 / *0.01 解码）是协议承诺，锁定后不得随意改。"""
    assert CMD_POSITION_ANGLE_MODE0 == 0x19
    assert CMD_SYSTEM == 0x08
    assert CMD_PROPERTY_READ == 0x1E
    assert CMD_PROPERTY_WRITE == 0x1F


def test_no_numpy_state_leakage() -> None:
    """编解码器不持有可变全局状态（每次调用产生全新帧）。"""
    a = encode_position(1, 10.0, speed=1.0, filter_accel=1.0)
    b = encode_position(1, 10.0, speed=1.0, filter_accel=1.0)
    assert np.array_equal(np.frombuffer(a, dtype=np.uint8), np.frombuffer(b, dtype=np.uint8))
