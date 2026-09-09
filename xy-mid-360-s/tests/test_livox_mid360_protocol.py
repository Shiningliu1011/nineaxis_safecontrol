#!/usr/bin/env python3
"""Protocol-layer regression tests for the MID-360 Python driver.

No hardware and no network: every frame is built synthetically and round-tripped
through the parser.  Covers the checksums, control frame encode/decode,
key_value parsing and point cloud decoding (both Cartesian formats, tag
filtering, CRC behaviour).
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "tests"))

from livox_mid360_synth import build_cloud_packet  # noqa: E402

from xy_mid_360_s.protocol import (  # noqa: E402
    CMD_DISCOVERY,
    CMD_PARAM_CONFIG,
    CMD_PARAM_INQUIRE,
    CMD_TYPE_ACK,
    DATA_TYPE_CARTESIAN_LOW,
    KEY_WORK_STATE,
    MID360_LINE_NUM,
    PCL_HDR_LEN,
    SENDER_HOST,
    SENDER_LIDAR,
    SOF,
    TIME_TYPE_GPS,
    TIME_TYPE_NO_SYNC,
    TIME_TYPE_PTP,
    WORK_STATE_NAMES,
    build_ctrl_frame,
    build_param_config,
    build_param_inquire,
    crc16_ccitt,
    crc32_ieee,
    parse_cloud_packet,
    parse_ctrl_frame,
    parse_key_value_list,
    packet_time_base_ns,
    point_interval_ns,
)


# ── checksums ────────────────────────────────────────────────────────────────

def test_crc16_ccitt_false_known_vector():
    # Standard check value for CRC-16/CCITT-FALSE.
    assert crc16_ccitt(b"123456789") == 0x29B1


def test_crc32_ieee_known_vector():
    assert crc32_ieee(b"123456789") == 0xCBF43926


# ── control frames ───────────────────────────────────────────────────────────

def test_ctrl_frame_roundtrip():
    payload = b"\x01\x02\x03\x04"
    raw = build_ctrl_frame(CMD_PARAM_CONFIG, payload, seq=42)
    assert len(raw) == 24 + len(payload)
    assert raw[0] == SOF
    frame = parse_ctrl_frame(raw)
    assert frame is not None
    assert frame["cmd_id"] == CMD_PARAM_CONFIG
    assert frame["seq_num"] == 42
    assert frame["sender"] == SENDER_HOST
    assert frame["data"] == payload


def test_ctrl_frame_accepts_device_sender_and_ack():
    raw = build_ctrl_frame(CMD_DISCOVERY, b"\x00\x23", seq=7,
                           cmd_type=CMD_TYPE_ACK, sender=SENDER_LIDAR)
    frame = parse_ctrl_frame(raw)
    assert frame is not None
    assert frame["sender"] == SENDER_LIDAR
    assert frame["cmd_type"] == CMD_TYPE_ACK
    assert frame["data"] == b"\x00\x23"


def test_ctrl_frame_rejects_corrupted_header_crc():
    raw = bytearray(build_ctrl_frame(CMD_PARAM_INQUIRE, b"\xaa", seq=1))
    raw[18] ^= 0xFF  # header CRC-16
    assert parse_ctrl_frame(bytes(raw)) is None


def test_ctrl_frame_rejects_corrupted_data_crc():
    raw = bytearray(build_ctrl_frame(CMD_PARAM_INQUIRE, b"\xaa\xbb", seq=1))
    raw[-1] ^= 0xFF  # data byte, CRC-32 now wrong
    assert parse_ctrl_frame(bytes(raw)) is None


def test_ctrl_frame_rejects_short_and_bad_sof():
    assert parse_ctrl_frame(b"\x00" * 10) is None
    raw = bytearray(build_ctrl_frame(CMD_DISCOVERY, b"", seq=1))
    raw[0] = 0xAB
    assert parse_ctrl_frame(bytes(raw)) is None


def test_ctrl_frame_rejects_length_beyond_datagram():
    raw = bytearray(build_ctrl_frame(CMD_DISCOVERY, b"", seq=1))
    struct.pack_into("<H", raw, 2, 4096)
    assert parse_ctrl_frame(bytes(raw)) is None


# ── key_value payloads ───────────────────────────────────────────────────────

def test_build_param_config_layout():
    data = build_param_config([(KEY_WORK_STATE, b"\x01")])
    key_num, rsvd = struct.unpack_from("<HH", data, 0)
    assert (key_num, rsvd) == (1, 0)
    key, val_len = struct.unpack_from("<HH", data, 4)
    assert (key, val_len) == (KEY_WORK_STATE, 1)
    assert data[8:] == b"\x01"


def test_build_param_inquire_lists_keys():
    data = build_param_inquire([0x8000, 0x8006])
    assert struct.unpack_from("<HH", data, 0) == (2, 0)
    assert struct.unpack_from("<HH", data, 4) == (0x8000, 0x8006)


def test_parse_key_value_list_inquire_shape():
    # ret(1) + param_num(2) + key(2) + len(2) + value  (no reserved field)
    data = struct.pack("<BH", 0, 1) + struct.pack("<HH", 0x8006, 1) + b"\x09"
    assert parse_key_value_list(data) == {0x8006: b"\x09"}


def test_parse_key_value_list_push_shape():
    # key_num(2) + rsvd(2) + entries (device pushes have no return code)
    data = struct.pack("<HH", 1, 0) + struct.pack("<HH", 0x8007, 4) + struct.pack("<i", 2537)
    assert parse_key_value_list(data) == {0x8007: struct.pack("<i", 2537)}


def test_parse_key_value_list_truncated_returns_empty():
    data = struct.pack("<BH", 0, 2) + struct.pack("<HH", 0x8006, 4) + b"\x01"
    assert parse_key_value_list(data) == {}


# ── point cloud packets ──────────────────────────────────────────────────────

def test_cloud_packet_decode_cartesian_high():
    xyz = np.array([[1.0, -2.5, 0.25], [0.001, 0.002, 0.003]], dtype=np.float64)
    raw = build_cloud_packet(xyz, intensity=[7, 200], tag=[0, 0], timestamp_ns=123456789)
    packet = parse_cloud_packet(raw)
    assert packet is not None
    assert packet.dot_num == 2
    np.testing.assert_allclose(packet.points[:, :3], xyz, atol=1e-3)
    np.testing.assert_allclose(packet.points[:, 3], [7.0, 200.0])
    assert packet.points.dtype == np.float32
    assert packet.tag.tolist() == [0, 0]
    # line is index % 4 within the packet
    assert packet.line.tolist() == [0, 1]
    assert packet.timestamp_ns == 123456789
    assert packet.time_type == TIME_TYPE_NO_SYNC


def test_cloud_packet_line_wraps_at_four():
    xyz = np.zeros((6, 3), dtype=np.float64)
    packet = parse_cloud_packet(build_cloud_packet(xyz))
    assert packet is not None
    assert packet.line.tolist() == [i % MID360_LINE_NUM for i in range(6)]


def test_cloud_packet_keeps_all_points_by_default():
    xyz = np.zeros((3, 3), dtype=np.float64)
    raw = build_cloud_packet(xyz, tag=[0, 1, 2])
    packet = parse_cloud_packet(raw)
    assert packet is not None
    assert packet.points.shape[0] == 3
    assert packet.tag.tolist() == [0, 1, 2]


def test_cloud_packet_can_drop_low_confidence_points():
    xyz = np.zeros((3, 3), dtype=np.float64)
    raw = build_cloud_packet(xyz, tag=[0, 1, 2])
    packet = parse_cloud_packet(raw, keep_all_points=False)
    assert packet is not None
    assert packet.points.shape[0] == 1
    assert packet.tag.tolist() == [0]


def test_cloud_packet_accepts_zero_crc_from_firmware():
    xyz = np.zeros((2, 3), dtype=np.float64)
    packet = parse_cloud_packet(build_cloud_packet(xyz, crc32="zero"))
    assert packet is not None


def test_cloud_packet_rejects_bad_crc():
    xyz = np.zeros((2, 3), dtype=np.float64)
    assert parse_cloud_packet(build_cloud_packet(xyz, crc32="bad")) is None


def test_cloud_packet_decode_cartesian_low_centimetres():
    xyz = np.array([[1.23, -0.45, 0.06]], dtype=np.float64)
    raw = build_cloud_packet(xyz, data_type=DATA_TYPE_CARTESIAN_LOW)
    packet = parse_cloud_packet(raw)
    assert packet is not None
    assert packet.data_type == DATA_TYPE_CARTESIAN_LOW
    np.testing.assert_allclose(packet.points[:, :3], xyz, atol=1e-2)


def test_cloud_packet_rejects_truncated_body():
    raw = build_cloud_packet(np.zeros((4, 3)))
    assert parse_cloud_packet(raw[:PCL_HDR_LEN + 10]) is None
    assert parse_cloud_packet(b"\x00" * 12) is None


def test_cloud_packet_rejects_unsupported_data_type():
    raw = bytearray(build_cloud_packet(np.zeros((2, 3))))
    raw[10] = 3  # spherical
    assert parse_cloud_packet(bytes(raw)) is None


# ── time helpers ─────────────────────────────────────────────────────────────

def test_time_type_values_match_upstream_enum():
    # livox_ros_driver2 src/comm/comm.h: NoSync=0, GptpOrPtp=1, Gps=2
    assert (TIME_TYPE_NO_SYNC, TIME_TYPE_PTP, TIME_TYPE_GPS) == (0, 1, 2)


def test_packet_time_base_uses_device_time_when_synced():
    assert packet_time_base_ns(TIME_TYPE_PTP, 111, 222) == 111
    assert packet_time_base_ns(TIME_TYPE_GPS, 111, 222) == 111


def test_packet_time_base_uses_host_time_without_sync():
    assert packet_time_base_ns(TIME_TYPE_NO_SYNC, 111, 222) == 222


def test_point_interval_matches_upstream_formula():
    # upstream: time_interval * 100 / dot_num (integer division, ns)
    assert point_interval_ns(1000, 96) == 1041
    assert point_interval_ns(1000, 0) == 0


def test_work_state_names_cover_mid360_ready():
    assert WORK_STATE_NAMES[0x01] == "scanning"
    assert WORK_STATE_NAMES[0x09] == "ready"
