#!/usr/bin/env python3
"""Stream-layer tests: frame assembly and loopback UDP reception.

Uses synthetic packets over 127.0.0.1 — no LiDAR, no device state changes.
"""

from __future__ import annotations

import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "tests"))

from livox_mid360_synth import build_cloud_packet, build_state_push  # noqa: E402

from xy_mid_360_s.protocol import (  # noqa: E402
    WORK_STATE_NORMAL,
    parse_cloud_packet,
)
from xy_mid_360_s.stream import (  # noqa: E402
    CloudReceiver,
    FrameAssembler,
    StateMonitor,
)


def _packet(n: int, base_ns: int, *, time_type: int = 0, dot_num: int = 0):
    """Decode a synthetic packet whose first point time is ``base_ns``."""
    xyz = np.zeros((n, 3), dtype=np.float64)
    raw = build_cloud_packet(xyz, timestamp_ns=base_ns, time_type=time_type,
                             time_interval_raw=1000)
    packet = parse_cloud_packet(raw)
    assert packet is not None
    return packet


# ── FrameAssembler ───────────────────────────────────────────────────────────

def test_assembler_emits_frame_after_interval():
    assembler = FrameAssembler(frame_interval_s=0.1)
    # NO_SYNC packets are stamped with host time, so pass the matching host clock.
    assert assembler.add_packet(_packet(10, 1_000_000_000),
                                host_now_ns=1_000_000_000) == []
    frames = assembler.add_packet(_packet(10, 1_200_000_000),
                                  host_now_ns=1_200_000_000)
    assert len(frames) == 1
    frame = frames[0]
    assert frame.size == 20
    assert frame.base_time_ns == 1_000_000_000
    assert frame.timestamp_ns[0] == 1_000_000_000
    # second packet's points continue the absolute time base
    assert frame.timestamp_ns[10] >= 1_200_000_000


def test_assembler_uses_host_time_without_sync():
    assembler = FrameAssembler(frame_interval_s=0.1)
    packet = _packet(2, 123, time_type=0)
    frames = assembler.add_packet(packet, host_now_ns=9_000_000_000)
    assert frames == []  # first packet only starts a frame
    assert assembler.flush().base_time_ns == 9_000_000_000


def test_assembler_uses_device_time_when_ptp_synced():
    assembler = FrameAssembler(frame_interval_s=0.1)
    packet = _packet(2, 123, time_type=2)  # GPS/PTP
    assembler.add_packet(packet, host_now_ns=9_000_000_000)
    assert assembler.flush().base_time_ns == 123


def test_assembler_flushes_on_backwards_time():
    assembler = FrameAssembler(frame_interval_s=10.0)
    assembler.add_packet(_packet(3, 5_000_000_000), host_now_ns=5_000_000_000)
    frames = assembler.add_packet(_packet(3, 1_000_000_000), host_now_ns=1_000_000_000)
    assert len(frames) == 1
    assert frames[0].size == 3
    # the later packet starts a fresh frame
    assert assembler.flush().base_time_ns == 1_000_000_000


def test_assembler_flushes_at_max_points():
    assembler = FrameAssembler(frame_interval_s=10.0, max_points=5)
    frames = assembler.add_packet(_packet(6, 1_000_000_000), host_now_ns=0)
    assert len(frames) == 1
    assert frames[0].size == 6
    assert assembler.flush() is None


def test_assembler_rejects_non_positive_interval():
    with pytest.raises(ValueError):
        FrameAssembler(frame_interval_s=0.0)


# ── CloudReceiver over loopback ──────────────────────────────────────────────

def _send_udp(port: int, payload: bytes) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(payload, ("127.0.0.1", port))
    finally:
        sock.close()


def test_cloud_receiver_delivers_frames_over_udp():
    received: list = []
    done = threading.Event()

    def _on_frame(frame) -> None:
        received.append(frame)
        done.set()

    receiver = CloudReceiver(host_ip="127.0.0.1", port=0, frame_interval_s=0.001,
                             on_frame=_on_frame, recv_timeout=0.05)
    receiver.start()
    try:
        xyz = np.array([[0.5, 0.25, -0.125]], dtype=np.float64)
        _send_udp(receiver.port, build_cloud_packet(xyz, intensity=[42], timestamp_ns=7))
        assert done.wait(2.0), "no frame delivered"
    finally:
        receiver.stop()
    assert receiver.stats["packets"] == 1
    assert receiver.stats["frames"] == 1
    frame = received[0]
    assert frame.src_ip == "127.0.0.1"
    np.testing.assert_allclose(frame.points[:, :3], xyz, atol=1e-3)
    np.testing.assert_allclose(frame.points[:, 3], [42.0])


def test_cloud_receiver_counts_dropped_packets():
    receiver = CloudReceiver(host_ip="127.0.0.1", port=0, frame_interval_s=0.1)
    receiver.start()
    try:
        _send_udp(receiver.port, b"\x00" * 8)  # too short to parse
        deadline = time.time() + 2.0
        while receiver.stats["dropped_packets"] == 0 and time.time() < deadline:
            time.sleep(0.01)
    finally:
        receiver.stop()
    assert receiver.stats["dropped_packets"] == 1
    assert receiver.stats["frames"] == 0


def test_cloud_receiver_reports_packet_age():
    receiver = CloudReceiver(host_ip="127.0.0.1", port=0, frame_interval_s=0.001)
    receiver.start()
    try:
        assert receiver.last_packet_age_s() is None
        _send_udp(receiver.port, build_cloud_packet(np.zeros((1, 3))))
        deadline = time.time() + 2.0
        while receiver.last_packet_age_s() is None and time.time() < deadline:
            time.sleep(0.01)
        age = receiver.last_packet_age_s()
        assert age is not None and age < 1.0
    finally:
        receiver.stop()


# ── StateMonitor over loopback ───────────────────────────────────────────────

def test_state_monitor_decodes_status_and_temperature():
    events: list = []
    done = threading.Event()

    def _on_state(src_ip: str, state: dict) -> None:
        events.append((src_ip, state))
        done.set()

    monitor = StateMonitor(host_ip="127.0.0.1", port=0, on_state=_on_state)
    monitor.start()
    try:
        _send_udp(monitor.port, build_state_push(WORK_STATE_NORMAL, temperature_c=25.37))
        assert done.wait(2.0), "no state event delivered"
    finally:
        monitor.stop()
    src_ip, state = events[0]
    assert src_ip == "127.0.0.1"
    assert state["status"] == "scanning"
    assert state["temperature_c"] == pytest.approx(25.4, abs=0.01)
    assert monitor.status["127.0.0.1"] == "scanning"


def test_state_monitor_ignores_host_frames():
    events: list = []
    monitor = StateMonitor(host_ip="127.0.0.1", port=0,
                           on_state=lambda ip, st: events.append((ip, st)))
    monitor.start()
    try:
        from xy_mid_360_s.protocol import (
            CMD_DISCOVERY, build_ctrl_frame)
        # Host-originated frame (sender=HOST) must not be treated as a push.
        _send_udp(monitor.port, build_ctrl_frame(CMD_DISCOVERY, b"", seq=1))
        time.sleep(0.2)
    finally:
        monitor.stop()
    assert events == []
