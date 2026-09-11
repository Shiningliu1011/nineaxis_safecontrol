import json
import subprocess
from types import SimpleNamespace

import pytest

from robot_safecontrol_moveit import hardware_probe as probe
from robot_safecontrol_moveit.robot_spec import DEFAULT_JOINT_NAMES


def connection():
    return {"interface": "can0", "bitrate": 1000000,
            "node_ids": {j: n for n, j in enumerate(DEFAULT_JOINT_NAMES, 1)}}


@pytest.mark.parametrize("kind,flags,bitrate,ready", [
    ("can", ["UP"], 1000000, True), ("can", ["UP"], 500000, False),
    ("can", [], 1000000, False), ("bridge", ["UP"], 1000000, False),
    ("vcan", ["UP"], None, True)])
def test_doctor_checks_transport_without_backend(monkeypatch, kind, flags, bitrate, ready):
    monkeypatch.setattr(probe.importlib.metadata, "version", lambda n: "4.6.1" if n == "python-can" else "5.4.1")
    state = [{"ifname": "can0", "flags": flags,
              "linkinfo": {"info_kind": kind, "info_data": {"bittiming": {"bitrate": bitrate}}}}]
    def run(command, **kwargs):
        assert command == ["ip", "-details", "-json", "link", "show"]
        return SimpleNamespace(stdout=json.dumps(state))
    monkeypatch.setattr(probe.subprocess, "run", run)
    result = probe.doctor(connection())
    assert result["transport_ready"] is ready
    assert result["frames_sent"] == 0
    assert result["can_opened"] is False
    assert result["motion_ready"] is False


def test_missing_interface_and_dependency(monkeypatch):
    def missing(name):
        raise probe.importlib.metadata.PackageNotFoundError(name)
    monkeypatch.setattr(probe.importlib.metadata, "version", missing)
    monkeypatch.setattr(probe.subprocess, "run", lambda *a, **k: SimpleNamespace(stdout="[]"))
    result = probe.doctor(connection())
    assert not result["transport_ready"]
    assert len(result["problems"]) == 3


def test_config_rejects_duplicate_ids(tmp_path):
    yaml = pytest.importorskip("yaml")
    cfg = connection()
    cfg["node_ids"]["J2"] = 1
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump({"/**": {"ros__parameters": {
        "can": {"interface": "can0", "bitrate": 1000000}, "node_ids": cfg["node_ids"]}}}))
    with pytest.raises(ValueError, match="unique"):
        probe.load_connection(str(path))


def test_probe_send_failure_aborts_without_more_requests():
    class Failed:
        def __init__(self): self.calls = []
        def send(self, frame_id, data):
            self.calls.append(frame_id)
            return False
    backend = Failed()
    with pytest.raises(RuntimeError, match="send failed"):
        probe.probe_nodes(backend, [1, 2])
    assert backend.calls == [0x3E]
