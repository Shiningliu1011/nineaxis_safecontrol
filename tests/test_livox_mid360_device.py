#!/usr/bin/env python3
"""Device-layer tests against a fake MID-360S responder on loopback.

The fake device implements just enough of the control protocol to exercise
``Mid360Device`` without hardware: it echoes ACKs with the request's seq_num and
lets each test declare what the device "answers".
"""

from __future__ import annotations

import socket
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Callable, Dict, Optional

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))
sys.path.insert(0, str(REPO_ROOT / "tests"))

from livox_mid360_synth import build_ack, config_ack, inquire_ack  # noqa: E402

from robot_safecontrol_moveit.livox_mid360.device import Mid360Device  # noqa: E402
from robot_safecontrol_moveit.livox_mid360.protocol import (  # noqa: E402
    CMD_PARAM_CONFIG,
    CMD_PARAM_INQUIRE,
    CMD_REBOOT,
    KEY_FOV_CFG0,
    KEY_FOV_CFG_EN,
    KEY_IMU_HOST_CFG,
    KEY_LIDAR_IP,
    KEY_LIDAR_TEMP,
    KEY_PCL_DATA_TYPE,
    KEY_PCL_HOST_CFG,
    KEY_PATTERN_MODE,
    KEY_SN,
    KEY_STATE_HOST_CFG,
    KEY_WORK_MODE,
    KEY_WORK_STATE,
    WORK_READY,
    WORK_SAMPLING,
    parse_ctrl_frame,
)


class FakeDevice(threading.Thread):
    """Loopback UDP responder for control commands."""

    def __init__(self) -> None:
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.1)
        self.port = self.sock.getsockname()[1]
        self._stop_event = threading.Event()
        self.requests: list = []
        self.handlers: Dict[int, Callable[[bytes, int], Optional[bytes]]] = {}

    def run(self) -> None:
        while not self._stop_event.is_set():
            try:
                raw, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            frame = parse_ctrl_frame(raw)
            if frame is None:
                continue
            self.requests.append(frame)
            handler = self.handlers.get(frame["cmd_id"])
            reply = handler(frame["data"], frame["seq_num"]) if handler else None
            if reply is not None:
                self.sock.sendto(reply, addr)

    def stop(self) -> None:
        self._stop_event.set()
        self.sock.close()
        self.join(timeout=1.0)


@pytest.fixture()
def fake_device():
    device = FakeDevice()
    device.start()
    yield device
    device.stop()


def _device(fake_device: FakeDevice, **kwargs) -> Mid360Device:
    defaults = dict(cmd_timeout=0.5, cmd_retries=1)
    defaults.update(kwargs)
    return Mid360Device("127.0.0.1", "127.0.0.1",
                        ctrl_port=fake_device.port, **defaults)


# ── queries ──────────────────────────────────────────────────────────────────

def test_query_sn(fake_device):
    fake_device.handlers[CMD_PARAM_INQUIRE] = lambda data, seq: inquire_ack(
        [(KEY_SN, b"3WEDH1A0012345\x00")], seq=seq)
    device = _device(fake_device)
    try:
        assert device.query_sn() == "3WEDH1A0012345"
        assert device.sn == "3WEDH1A0012345"
    finally:
        device.close()


def test_query_state_maps_ready(fake_device):
    fake_device.handlers[CMD_PARAM_INQUIRE] = lambda data, seq: inquire_ack(
        [(KEY_WORK_STATE, struct.pack("<B", WORK_READY))], seq=seq)
    device = _device(fake_device)
    try:
        assert device.query_state() == "ready"
    finally:
        device.close()


def test_query_temperature(fake_device):
    fake_device.handlers[CMD_PARAM_INQUIRE] = lambda data, seq: inquire_ack(
        [(KEY_LIDAR_TEMP, struct.pack("<i", 2537))], seq=seq)
    device = _device(fake_device)
    try:
        # device reports 0.01 °C units; the driver rounds to 0.1 °C
        assert device.query_temperature() == pytest.approx(25.4)
    finally:
        device.close()


def test_query_fov_parses_bounds_and_enable_bit(fake_device):
    fov_value = struct.pack("<iiiiI", 10, 350, -5, 50, 0)

    def handler(data: bytes, seq: int) -> bytes:
        return inquire_ack(
            [(KEY_FOV_CFG0, fov_value), (KEY_FOV_CFG_EN, struct.pack("<B", 0x01))],
            seq=seq)

    fake_device.handlers[CMD_PARAM_INQUIRE] = handler
    device = _device(fake_device)
    try:
        config = device.query_fov(0)
    finally:
        device.close()
    assert config == {
        "yaw_start": 10.0, "yaw_stop": 350.0,
        "pitch_start": -5.0, "pitch_stop": 50.0, "enable": True,
    }


def test_query_lidar_ip(fake_device):
    ip_bytes = (socket.inet_aton("192.168.1.115") + socket.inet_aton("255.255.255.0")
                + socket.inet_aton("0.0.0.0"))
    fake_device.handlers[CMD_PARAM_INQUIRE] = lambda data, seq: inquire_ack(
        [(KEY_LIDAR_IP, ip_bytes)], seq=seq)
    device = _device(fake_device)
    try:
        assert device.query_lidar_ip() == {
            "ip": "192.168.1.115", "subnet": "255.255.255.0", "gateway": "0.0.0.0"}
    finally:
        device.close()


# ── writes ───────────────────────────────────────────────────────────────────

def test_set_work_mode_sends_sampling(fake_device):
    fake_device.handlers[CMD_PARAM_CONFIG] = lambda data, seq: config_ack(seq=seq)
    device = _device(fake_device)
    try:
        assert device.set_work_mode(WORK_SAMPLING) is True
    finally:
        device.close()
    request = fake_device.requests[-1]
    assert request["cmd_id"] == CMD_PARAM_CONFIG
    # key_num=1, rsvd=0, key=0x001A, len=1, value=0x01
    assert struct.unpack_from("<HH", request["data"], 0) == (1, 0)
    assert struct.unpack_from("<HH", request["data"], 4) == (KEY_WORK_MODE, 1)
    assert request["data"][8] == WORK_SAMPLING


def test_set_work_mode_reports_failure(fake_device):
    fake_device.handlers[CMD_PARAM_CONFIG] = lambda data, seq: config_ack(
        seq=seq, ret_code=1)
    device = _device(fake_device)
    try:
        assert device.set_work_mode(WORK_SAMPLING) is False
    finally:
        device.close()


def test_configure_push_destinations_sends_all_keys(fake_device):
    fake_device.handlers[CMD_PARAM_CONFIG] = lambda data, seq: config_ack(seq=seq)
    device = _device(fake_device, pcl_host_port=56301, state_host_port=56201,
                     imu_host_port=56401)
    try:
        assert device.configure_push_destinations() is True
    finally:
        device.close()
    data = fake_device.requests[-1]["data"]
    key_num = struct.unpack_from("<H", data, 0)[0]
    keys = []
    pos = 4
    for _ in range(key_num):
        key, val_len = struct.unpack_from("<HH", data, pos)
        keys.append(key)
        pos += 4 + val_len
    assert keys == [KEY_PCL_DATA_TYPE, KEY_PCL_HOST_CFG,
                    KEY_STATE_HOST_CFG, KEY_IMU_HOST_CFG]


def test_set_fov_preserves_other_profile_enable_bit(fake_device):
    """Read-modify-write: profile 1 stays enabled when writing profile 0."""
    fake_device.handlers[CMD_PARAM_INQUIRE] = lambda data, seq: inquire_ack(
        [(KEY_FOV_CFG_EN, struct.pack("<B", 0x02))], seq=seq)
    fake_device.handlers[CMD_PARAM_CONFIG] = lambda data, seq: config_ack(seq=seq)
    device = _device(fake_device)
    try:
        assert device.set_fov(0, 0, 360, -7, 52, True) is True
    finally:
        device.close()
    data = fake_device.requests[-1]["data"]
    # Second entry is the enable byte: bit0 (new) and bit1 (preserved).
    pos = 4
    key, val_len = struct.unpack_from("<HH", data, pos)
    pos += 4 + val_len
    key2, val_len2 = struct.unpack_from("<HH", data, pos)
    assert key2 == KEY_FOV_CFG_EN
    assert data[pos + 4] == 0x03


def test_set_pattern_mode_and_reboot(fake_device):
    fake_device.handlers[CMD_PARAM_CONFIG] = lambda data, seq: config_ack(seq=seq)
    fake_device.handlers[CMD_REBOOT] = lambda data, seq: build_ack(
        CMD_REBOOT, struct.pack("<B", 0), seq=seq)
    device = _device(fake_device)
    try:
        assert device.set_pattern_mode(0) is True
        assert device.reboot(1500) is True
    finally:
        device.close()
    assert fake_device.requests[-1]["data"] == struct.pack("<H", 1500)


def test_set_fov_rejects_invalid_profile(fake_device):
    device = _device(fake_device)
    try:
        with pytest.raises(ValueError):
            device.set_fov(2, 0, 360, -7, 52, True)
    finally:
        device.close()


# ── failure paths ────────────────────────────────────────────────────────────

def test_timeout_returns_empty_and_false():
    """No responder: queries return empty/None, writes return False."""
    device = Mid360Device("127.0.0.1", "127.0.0.1", ctrl_port=1,
                          cmd_timeout=0.2, cmd_retries=1)
    try:
        assert device.query_sn() == ""
        assert device.query_state() is None
        assert device.set_work_mode(WORK_SAMPLING) is False
    finally:
        device.close()
