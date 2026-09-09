"""``livox_mid360_tool`` — device bring-up and diagnostics for MID-360/MID-360S.

Adapted for robot_safecontrol from the user-supplied ``tmbs-main`` archive: the
source driver exposed the same operations only through its web backend.  Here
they are a CLI, so device identity, network configuration, work mode and raw
point cloud flow can be checked from a terminal without ROS or the C++ SDK.

Examples::

    livox_mid360_tool discover --host-ip 192.168.1.5
    livox_mid360_tool info --ip 192.168.1.115 --host-ip 192.168.1.5
    livox_mid360_tool configure --ip 192.168.1.115 --host-ip 192.168.1.5
    livox_mid360_tool mode set sampling --ip 192.168.1.115
    livox_mid360_tool stream --seconds 10 --host-ip 192.168.1.5

Commands that change device state are marked in their help text; nothing here
runs automatically.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from typing import Any, Dict, List, Optional

from robot_safecontrol_moveit.livox_mid360.device import Mid360Device
from robot_safecontrol_moveit.livox_mid360.discovery import discover_subnet, query_sn
from robot_safecontrol_moveit.livox_mid360.protocol import (
    PORT_PUSH_PCL,
    PORT_PUSH_STATE,
    WORK_READY,
    WORK_SAMPLING,
    WORK_STANDBY,
)
from robot_safecontrol_moveit.livox_mid360.stream import CloudReceiver

_WORK_MODES = {"sampling": WORK_SAMPLING, "standby": WORK_STANDBY, "ready": WORK_READY}


def _device(args: argparse.Namespace) -> Mid360Device:
    return Mid360Device(args.ip, args.host_ip or "")


def _print(value: Any) -> None:
    print(json.dumps(value, indent=2, ensure_ascii=False, default=str))


# ── commands ─────────────────────────────────────────────────────────────────

def cmd_discover(args: argparse.Namespace) -> int:
    targets = [t for t in (args.targets or "").split(",") if t]
    found = discover_subnet(args.subnet, timeout=args.timeout,
                            host_ip=args.host_ip or "", targets=targets)
    if not found:
        print("no device found (check host_ip/NIC and that the device is powered)")
        return 1
    _print(found)
    return 0


def cmd_info(args: argparse.Namespace) -> int:
    device = _device(args)
    try:
        info: Dict[str, Any] = {
            "ip": args.ip,
            "sn": device.query_sn() or None,
            "status": device.query_state(),
            "temperature_c": device.query_temperature(),
            "ip_config": device.query_lidar_ip(),
            "pattern_mode": device.query_pattern_mode(),
            "fov_enable": device.query_fov_enable(),
            "fov_profile_0": device.query_fov(0),
            "fov_profile_1": device.query_fov(1),
        }
    finally:
        device.close()
    _print(info)
    return 0


def cmd_sn(args: argparse.Namespace) -> int:
    sn = query_sn(args.ip, timeout=args.timeout, local_ip=args.host_ip or "")
    if not sn:
        print(f"no SN response from {args.ip}")
        return 1
    print(sn)
    return 0


def cmd_configure(args: argparse.Namespace) -> int:
    """Point the device's push destinations at this host (state-changing)."""
    device = _device(args)
    try:
        ok = device.configure_push_destinations()
    finally:
        device.close()
    return 0 if ok else 1


def cmd_mode(args: argparse.Namespace) -> int:
    device = _device(args)
    try:
        if args.action == "get":
            status = device.query_state()
            if status is None:
                print("no response")
                return 1
            print(status)
            return 0
        mode = _WORK_MODES[args.value]
        ok = device.set_work_mode(mode)
        if ok and args.value == "sampling":
            device.configure_push_destinations()
        return 0 if ok else 1
    finally:
        device.close()


def cmd_ip(args: argparse.Namespace) -> int:
    device = _device(args)
    try:
        if args.action == "get":
            config = device.query_lidar_ip()
            if config is None:
                print("no response")
                return 1
            _print(config)
            return 0
        ok = device.set_lidar_ip(args.new_ip, args.subnet, args.gateway)
        if ok and args.reboot:
            device.reboot()
        return 0 if ok else 1
    finally:
        device.close()


def cmd_fov(args: argparse.Namespace) -> int:
    device = _device(args)
    try:
        if args.action == "get":
            config = device.query_fov(args.profile)
            if config is None:
                print("no response")
                return 1
            _print(config)
            return 0
        return 0 if device.set_fov(
            args.profile, args.yaw_start, args.yaw_stop,
            args.pitch_start, args.pitch_stop, args.enable) else 1
    finally:
        device.close()


def cmd_pattern(args: argparse.Namespace) -> int:
    device = _device(args)
    try:
        if args.action == "get":
            mode = device.query_pattern_mode()
            if mode is None:
                print("no response")
                return 1
            print(mode)
            return 0
        return 0 if device.set_pattern_mode(args.mode) else 1
    finally:
        device.close()


def cmd_reboot(args: argparse.Namespace) -> int:
    device = _device(args)
    try:
        return 0 if device.reboot(args.delay_ms) else 1
    finally:
        device.close()


def cmd_stream(args: argparse.Namespace) -> int:
    """Listen on the point cloud port and report frame/point rates."""
    frames: List[int] = []
    points: List[int] = []

    def _on_frame(frame) -> None:
        frames.append(frame.size)
        points.append(frame.size)
        if args.verbose:
            print(f"frame: {frame.size} points, base_time_ns={frame.base_time_ns}, "
                  f"src={frame.src_ip}")

    receiver = CloudReceiver(
        host_ip=args.host_ip or "", port=args.port,
        frame_interval_s=1.0 / args.rate, on_frame=_on_frame)
    receiver.start()
    print(f"listening on {args.host_ip or '0.0.0.0'}:{receiver.port} for {args.seconds:.0f}s "
          f"(device must already push here; use 'configure' first)")
    try:
        time.sleep(args.seconds)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()
    stats = receiver.stats
    summary = {
        "packets": stats["packets"],
        "points": stats["points"],
        "frames": stats["frames"],
        "dropped_packets": stats["dropped_packets"],
        "avg_points_per_frame": (sum(frames) / len(frames)) if frames else 0.0,
        "observed_frame_hz": (len(frames) / args.seconds) if args.seconds > 0 else 0.0,
    }
    _print(summary)
    return 0 if stats["frames"] else 1


# ── argument parsing ─────────────────────────────────────────────────────────

def _add_device_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--ip", required=True, help="device IPv4 address")
    parser.add_argument("--host-ip", default="", help="local address to bind/route from")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="livox_mid360_tool",
        description="MID-360/MID-360S device tool (Livox Ethernet protocol, no ROS/SDK)")
    parser.add_argument("--verbose", "-v", action="store_true", help="debug logging")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("discover", help="broadcast-discover devices on a subnet")
    p.add_argument("--subnet", default="192.168.1", help="three-octet prefix")
    p.add_argument("--host-ip", default="",
                   help="source address for the unicast fallback probes")
    p.add_argument("--timeout", type=float, default=2.0)
    p.add_argument("--targets", default="", help="extra IPs to probe by unicast SN query")
    p.set_defaults(func=cmd_discover)

    p = sub.add_parser("info", help="read identity, state and configuration")
    _add_device_args(p)
    p.set_defaults(func=cmd_info)

    p = sub.add_parser("sn", help="read the serial number")
    _add_device_args(p)
    p.add_argument("--timeout", type=float, default=1.5)
    p.set_defaults(func=cmd_sn)

    p = sub.add_parser("configure", help="set push destinations to this host (state-changing)")
    _add_device_args(p)
    p.set_defaults(func=cmd_configure)

    p = sub.add_parser("mode", help="get/set work mode")
    _add_device_args(p)
    p.add_argument("action", choices=["get", "set"])
    p.add_argument("value", nargs="?", choices=sorted(_WORK_MODES), help="mode for 'set'")
    p.set_defaults(func=cmd_mode)

    p = sub.add_parser("ip", help="get/set the device IP configuration")
    _add_device_args(p)
    p.add_argument("action", choices=["get", "set"])
    p.add_argument("--new-ip", help="new device address (for 'set')")
    p.add_argument("--subnet", default="255.255.255.0")
    p.add_argument("--gateway", default="0.0.0.0")
    p.add_argument("--reboot", action="store_true", help="reboot after writing")
    p.set_defaults(func=cmd_ip)

    p = sub.add_parser("fov", help="get/set FOV profiles")
    _add_device_args(p)
    p.add_argument("action", choices=["get", "set"])
    p.add_argument("--profile", type=int, default=0, choices=[0, 1])
    p.add_argument("--yaw-start", type=float, default=0.0)
    p.add_argument("--yaw-stop", type=float, default=360.0)
    p.add_argument("--pitch-start", type=float, default=-7.0)
    p.add_argument("--pitch-stop", type=float, default=52.0)
    p.add_argument("--enable", action="store_true")
    p.set_defaults(func=cmd_fov)

    p = sub.add_parser("pattern", help="get/set the scan pattern mode")
    _add_device_args(p)
    p.add_argument("action", choices=["get", "set"])
    p.add_argument("--mode", type=int, choices=[0, 1, 2], default=0)
    p.set_defaults(func=cmd_pattern)

    p = sub.add_parser("reboot", help="reboot the device (state-changing)")
    _add_device_args(p)
    p.add_argument("--delay-ms", type=int, default=2000)
    p.set_defaults(func=cmd_reboot)

    p = sub.add_parser("stream", help="listen for pushed point clouds and report rates")
    p.add_argument("--host-ip", default="")
    p.add_argument("--port", type=int, default=PORT_PUSH_PCL)
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--rate", type=float, default=10.0, help="expected frame rate (Hz)")
    p.set_defaults(func=cmd_stream)

    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    return int(args.func(args) or 0)


if __name__ == "__main__":
    sys.exit(main())
