"""Non-motion diagnostics. doctor does not open CAN; probe sends typed reads only.

This tool has no position, enable, clear-error, reboot, calibration or stop
command. A successful response proves communication, not readiness to move.
"""
from __future__ import annotations

import argparse
import importlib.metadata
import json
import math
from pathlib import Path
import subprocess
import time

from .drempower_can import can_id, encode_property_read, decode_property_reply
from .robot_spec import DEFAULT_JOINT_NAMES

# Local vendor interface_enums.py profile, not inferred from our encoder.
PROBE_PROPERTIES = (
    ("current_state", 30002, "u32"),
    ("axis_error", 30001, "u32"),
    ("output_position", 38007, "f32"),
    ("output_velocity", 38008, "f32"),
)


def load_connection(path: str) -> dict:
    import yaml
    try:
        config = yaml.safe_load(Path(path).read_text())["/**"]["ros__parameters"]
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid connection YAML: {exc}") from exc
    interface = config["can"]["interface"]
    bitrate = config["can"]["bitrate"]
    nodes = config["node_ids"]
    if not isinstance(interface, str) or not interface or len(interface) > 15:
        raise ValueError("invalid CAN interface")
    if type(bitrate) is not int or bitrate <= 0:
        raise ValueError("invalid CAN bitrate")
    if set(nodes) != set(DEFAULT_JOINT_NAMES):
        raise ValueError("node mapping must contain all canonical joints")
    if any(type(v) is not int or not 1 <= v <= 63 for v in nodes.values()):
        raise ValueError("node IDs must be integers in 1..63")
    if len(set(nodes.values())) != len(nodes):
        raise ValueError("node IDs must be unique")
    return {"interface": interface, "bitrate": bitrate, "node_ids": nodes}


def doctor(connection: dict) -> dict:
    """Read process dependencies and netlink inventory, without opening a CAN socket."""
    problems = []
    versions = {}
    for package in ("python-can", "PyYAML"):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = None
            problems.append(f"missing dependency: {package}")
    if versions["python-can"] is not None and versions["python-can"] != "4.6.1":
        problems.append("unqualified python-can version; expected 4.6.1")
    interface_info = None
    try:
        result = subprocess.run(["ip", "-details", "-json", "link", "show"],
                                capture_output=True, text=True, check=True, timeout=5)
        interface_info = next((x for x in json.loads(result.stdout)
                               if x["ifname"] == connection["interface"]), None)
        if interface_info is None:
            problems.append(f"interface missing: {connection['interface']}")
        else:
            kind = interface_info.get("linkinfo", {}).get("info_kind")
            if kind not in ("can", "vcan"):
                problems.append("selected interface is not CAN/vcan")
            if "UP" not in interface_info.get("flags", []):
                problems.append("interface is down")
            info = interface_info.get("linkinfo", {}).get("info_data", {})
            if kind == "can":
                if info.get("bittiming", {}).get("bitrate") != connection["bitrate"]:
                    problems.append("CAN bitrate mismatch or unavailable")
                if info.get("state") in ("BUS-OFF", "STOPPED", "SLEEPING"):
                    problems.append(f"CAN state: {info['state']}")
    except (OSError, subprocess.SubprocessError, ValueError) as exc:
        problems.append(f"cannot inspect interfaces: {exc}")
    return {"operation": "doctor", "can_opened": False, "frames_sent": 0,
            "connection": connection, "dependencies": versions,
            "interface_info": interface_info, "transport_ready": not problems,
            "problems": problems, "motion_ready": False,
            "motion_blockers": ["live bridge remains disabled",
                                "device firmware/profile and units not qualified",
                                "mapping, zero/direction and J1 transmission require field evidence",
                                "independent watchdog and physical stopping require field acceptance"]}


def probe_nodes(backend, node_ids, *, timeout_s: float = 0.2) -> dict:
    """Bound each property round across all nodes, preserving original RX times.

    CAN has no transaction ID: timestamps exclude already-queued frames but cannot
    establish device sample age or disambiguate a delayed matching reply. Run as
    sole requester, never use this report as controller/watchdog feedback.
    """
    nodes = tuple(node_ids)
    if (not nodes or len(nodes) != len(set(nodes))
            or any(type(n) is not int or not 1 <= n <= 63 for n in nodes)
            or not math.isfinite(timeout_s) or timeout_s <= 0):
        raise ValueError("invalid probe nodes or timeout")
    readings = {str(n): {} for n in nodes}
    sent = 0
    started = time.monotonic()
    for name, address, kind in PROBE_PROPERTIES:
        deadline = time.monotonic() + timeout_s
        pending = {}
        for node in nodes:
            if time.monotonic() >= deadline:
                raise TimeoutError("probe send round exceeded deadline")
            requested_at = time.time()
            if not backend.send(can_id(node, 0x1E), encode_property_read(node, address, kind)):
                raise RuntimeError(f"CAN read send failed for node {node}")
            sent += 1
            pending[node] = requested_at
        while pending:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            frame = backend.receive(remaining)
            if frame is None:
                break
            node = frame.frame_id >> 5
            if (node not in pending or not math.isfinite(frame.timestamp_s)
                    or frame.timestamp_s < pending[node] or frame.timestamp_s > time.time()):
                continue
            try:
                value = decode_property_reply(frame.frame_id, frame.data, node_id=node,
                                              address=address, value_kind=kind)
            except ValueError:
                continue
            readings[str(node)][name] = {"value": value, "rx_wall_time_s": frame.timestamp_s}
            del pending[node]
    complete = all(len(row) == len(PROBE_PROPERTIES) for row in readings.values())
    return {"operation": "probe", "frames_sent": sent, "readings": readings,
            "communication_complete": complete, "elapsed_s": time.monotonic() - started,
            "motion_ready": False, "device_sample_age_known": False}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("doctor", "probe"))
    parser.add_argument("--config", required=True, help="existing drempower.yaml")
    parser.add_argument("--nodes", type=int, nargs="+", help="explicit IDs for non-motion probe")
    parser.add_argument("--timeout", type=float, default=0.2)
    args = parser.parse_args(argv)
    try:
        connection = load_connection(args.config)
        report = doctor(connection)
        if args.operation == "probe":
            if not args.nodes:
                parser.error("probe requires explicit --nodes; no automatic scan")
            if (len(args.nodes) != len(set(args.nodes))
                    or any(not 1 <= n <= 63 for n in args.nodes)
                    or not math.isfinite(args.timeout) or args.timeout <= 0):
                parser.error("unique nodes in 1..63 and finite positive timeout required")
            if not report["transport_ready"]:
                print(json.dumps(report, indent=2, ensure_ascii=False))
                return 2
            from .python_can_backend import PythonCANBackend
            with PythonCANBackend(connection["interface"]) as backend:
                report = probe_nodes(backend, args.nodes, timeout_s=args.timeout)
            ok = report["communication_complete"]
        else:
            ok = report["transport_ready"]
        print(json.dumps(report, indent=2, ensure_ascii=False))
        return 0 if ok else 2
    except (OSError, RuntimeError, ValueError, KeyError, TypeError, ImportError) as exc:
        print(json.dumps({"error": str(exc), "motion_ready": False}, ensure_ascii=False))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
