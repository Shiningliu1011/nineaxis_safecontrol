"""Actual python-can peers; virtual != Linux vcan != hardware acceptance."""
import math
import os
import struct
import subprocess
import json
import threading
import time
import uuid

import pytest

can = pytest.importorskip("can")
from robot_safecontrol_moveit.python_can_backend import PythonCANBackend
from robot_safecontrol_moveit.hardware_probe import probe_nodes


@pytest.fixture
def peers():
    channel = str(uuid.uuid4())
    with PythonCANBackend(channel, interface="virtual") as backend:
        with can.Bus(interface="virtual", channel=channel, ignore_config=True) as peer:
            yield backend, peer


def message(frame_id=0x3E, data=bytes.fromhex("3275030008000000"), **kwargs):
    return can.Message(arbitration_id=frame_id, data=data, is_extended_id=False, **kwargs)


def test_two_peer_roundtrip_and_close(peers):
    backend, peer = peers
    assert backend.send(0x3E, bytes.fromhex("3275030000000000"))
    request = peer.recv(.1)
    assert request.arbitration_id == 0x3E
    assert bytes(request.data) == bytes.fromhex("3275030000000000")
    assert not request.is_extended_id
    peer.send(message())
    received = backend.recv(1, .1)
    assert received == (0x3E, bytes.fromhex("3275030008000000"))
    assert received is not None and math.isfinite(received.timestamp_s)
    backend.close()
    backend.close()
    assert not backend.send(0x3E, b"\0" * 8)
    with pytest.raises(RuntimeError):
        backend.receive(0)


def test_foreign_remote_extended_and_fd_frames_not_feedback(peers):
    backend, peer = peers
    peer.send(message(0x5E))
    peer.send(message(is_remote_frame=True))
    peer.send(can.Message(arbitration_id=0x3E, data=b"\0"*8, is_extended_id=True))
    peer.send(message(is_fd=True))
    peer.send(message(data=b"\0"*7))
    peer.send(message())
    assert backend.recv(1, .1) == (0x3E, bytes.fromhex("3275030008000000"))


def test_error_frame_and_send_failure_are_visible(peers, monkeypatch):
    backend, peer = peers
    peer.send(message(is_error_frame=True))
    with pytest.raises(RuntimeError, match="error frame"):
        backend.receive(.1)
    def fail(*args, **kwargs):
        raise can.CanOperationError("bus off")
    monkeypatch.setattr(backend._bus, "send", fail)
    assert not backend.send(0x3E, b"\0"*8)
    assert "bus off" in backend.last_error


def test_missing_socketcan_is_explicit():
    with pytest.raises(RuntimeError, match="Cannot open socketcan"):
        PythonCANBackend("no_can_13")


def exercise_nine_nodes(backend, peer):
    failures = []
    requests = []
    def respond():
        try:
            for address, kind in ((30002, 3), (30001, 3), (38007, 0), (38008, 0)):
                batch = []
                for _ in range(9):
                    msg = peer.recv(1)
                    assert msg is not None
                    requests.append(msg)
                    assert msg.arbitration_id & 31 == 0x1E
                    assert bytes(msg.data) == struct.pack("<HHI", address, kind, 0)
                    batch.append(msg)
                # Wrong address and unsolicited foreign node must not enter report.
                peer.send(message(0x3E, bytes.fromhex("ffff030008000000")))
                peer.send(message(0x7FE))
                for msg in reversed(batch):
                    node = msg.arbitration_id >> 5
                    value = 8 if address == 30002 else 0 if address == 30001 else node + .25
                    data = struct.pack("<HH", address, kind) + struct.pack("<I" if kind == 3 else "<f", value)
                    peer.send(message(msg.arbitration_id, data))
        except BaseException as exc:
            failures.append(exc)
    worker = threading.Thread(target=respond)
    worker.start()
    try:
        report = probe_nodes(backend, range(1, 10), timeout_s=.5)
    finally:
        worker.join(5)
    assert not worker.is_alive()
    assert not failures
    assert report["communication_complete"]
    assert report["frames_sent"] == 36
    assert not report["motion_ready"]
    for node in range(1, 10):
        assert report["readings"][str(node)]["output_position"]["value"] == node + .25
    assert len(requests) == 36


def test_virtual_nine_nodes_reversed_responses(peers):
    exercise_nine_nodes(*peers)


def test_queued_old_reply_and_missing_nodes_stay_unavailable(peers):
    backend, peer = peers
    peer.send(message())
    time.sleep(.001)
    report = probe_nodes(backend, range(1, 10), timeout_s=.01)
    assert not report["communication_complete"]
    assert all(not row for row in report["readings"].values())
    # Four shared deadlines, not 36 serial 10ms waits. Generous scheduling margin.
    assert report["elapsed_s"] < .25


def test_linux_vcan_nine_nodes():
    channel = os.environ.get("HARDWARE_TEST_VCAN")
    if not channel:
        pytest.skip("explicit Linux vcan acceptance via scripts/vcan_integration_test.sh")
    state = json.loads(subprocess.check_output(["ip", "-details", "-json", "link", "show", channel]))[0]
    assert state.get("linkinfo", {}).get("info_kind") == "vcan", "physical CAN tests forbidden"
    assert "UP" in state["flags"]
    with PythonCANBackend(channel) as backend:
        with can.Bus(interface="socketcan", channel=channel, ignore_config=True) as peer:
            exercise_nine_nodes(backend, peer)
