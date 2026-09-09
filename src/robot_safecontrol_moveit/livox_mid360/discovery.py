"""Subnet discovery for MID-360/MID-360S devices.

Adapted for robot_safecontrol from the user-supplied ``tmbs-main`` archive
(``backend/app/drivers/mid360_driver.py``, ``discover_subnet``/``discover_device``
and the module-level ``_quick_query_sn``).

Two paths are kept, for the reason the source driver kept them:

* **Broadcast** to ``<subnet>.255:56000`` (and the limited broadcast address)
  with command ``0x0000``.  The socket binds to ``INADDR_ANY``, not ``host_ip``:
  the device answers a discovery broadcast with a *broadcast-destined* reply,
  and the kernel only delivers those to an any-bound socket.  Measured on
  hardware (ticket #31 follow-up): binding to the interface address receives
  zero replies, binding to ``0.0.0.0`` receives the ACK.  The subnet-directed
  broadcast still leaves through the right NIC because the route for
  ``<subnet>.255`` selects it.
* **Unicast SN probe** on ``:56100``, for environments where broadcast does not
  cross (NAT/WSL/containers) or when the device address is already known.

Neither path changes device state.

Model identification: the broadcast ACK's second payload byte carries the device
type — **MID-360 = 9, MID-360S = 35** (this project's device is a MID-360S; the
type-35 value was confirmed on hardware in the ticket #31 online run).  Both are
accepted here.  A unicast SN probe proves only that *a* Livox device answers.
"""

from __future__ import annotations

import logging
import socket
import time
from typing import Any, Dict, List, Optional, Sequence

from robot_safecontrol_moveit.livox_mid360.protocol import (
    ACCEPTED_DEV_TYPES,
    CMD_DISCOVERY,
    CMD_PARAM_INQUIRE,
    CMD_TYPE_ACK,
    DEV_TYPE_NAMES,
    KEY_SN,
    PORT_BCAST_CMD,
    PORT_CTRL,
    build_ctrl_frame,
    build_param_inquire,
    parse_ctrl_frame,
)

logger = logging.getLogger(__name__)


def query_sn(ip: str, *, port: int = PORT_CTRL, timeout: float = 1.5,
             local_ip: str = "") -> str:
    """Unicast ``KEY_SN`` query; returns the serial number or ``""``.

    A non-empty result proves a Livox device answers at ``ip:port`` even when
    broadcast discovery is unavailable.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        if local_ip:
            sock.bind((local_ip, 0))
        sock.settimeout(timeout)
        frame = build_ctrl_frame(CMD_PARAM_INQUIRE, build_param_inquire([KEY_SN]), seq=1)
        sock.sendto(frame, (ip, port))
        raw, _addr = sock.recvfrom(4096)
        resp = parse_ctrl_frame(raw)
        if resp and resp["cmd_id"] == CMD_PARAM_INQUIRE \
                and resp["cmd_type"] == CMD_TYPE_ACK:
            data = resp["data"]
            if data and len(data) > 7 and data[0] == 0:
                return data[7:].split(b"\x00")[0].decode("ascii", errors="replace").strip()
    except OSError as exc:
        logger.debug("SN query to %s failed: %s", ip, exc)
    finally:
        sock.close()
    return ""


def discover_subnet(subnet: str = "192.168.1", *, timeout: float = 2.0,
                    host_ip: str = "", targets: Optional[Sequence[str]] = None
                    ) -> List[Dict[str, Any]]:
    """Broadcast-discover devices, then enrich each hit with its serial number.

    :param subnet:  three-octet prefix, e.g. ``"192.168.1"``
    :param timeout: total collection window in seconds; the probe is re-sent
                    after every 1 s receive timeout
    :param host_ip: source address for the unicast fallback probes below; the
                    broadcast socket itself must stay any-bound (see module
                    docstring)
    :param targets: extra addresses probed by unicast SN query afterwards
    """
    ping = build_ctrl_frame(CMD_DISCOVERY, b"", seq=0)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", 0))
    sock.settimeout(1.0)
    subnet_bcast = f"{subnet}.255"

    def _send_ping() -> None:
        for dst in (subnet_bcast, "255.255.255.255"):
            try:
                sock.sendto(ping, (dst, PORT_BCAST_CMD))
            except OSError as exc:
                logger.warning("discovery: broadcast to %s failed: %s", dst, exc)

    found: List[Dict[str, Any]] = []
    seen: set = set()
    try:
        _send_ping()
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                _send_ping()
                continue
            frame = parse_ctrl_frame(data)
            if not frame or frame["cmd_id"] != CMD_DISCOVERY \
                    or frame["cmd_type"] != CMD_TYPE_ACK:
                continue
            ip = addr[0]
            if ip == host_ip or ip in seen:
                continue
            payload = frame["data"]
            if len(payload) < 2 or payload[0] != 0:
                continue
            dev_type = payload[1]
            if dev_type not in ACCEPTED_DEV_TYPES:
                logger.info("discovery: %s reports dev_type=%d (not MID-360/MID-360S), skipped",
                            ip, dev_type)
                continue
            seen.add(ip)
            found.append({
                "ip": ip,
                "device_type": dev_type,
                "model": DEV_TYPE_NAMES[dev_type],
                "sn": "",
            })
    finally:
        sock.close()

    for device in found:
        device["sn"] = query_sn(device["ip"])

    for target_ip in (targets or ()):
        if target_ip in seen:
            continue
        sn = query_sn(target_ip, local_ip=host_ip)
        if sn:
            seen.add(target_ip)
            # A unicast SN probe proves a Livox device answers, but the SN
            # response carries no model field: do not claim MID-360 vs MID-360S.
            found.append({
                "ip": target_ip,
                "device_type": None,
                "model": "unknown (unicast probe; use broadcast discovery for the model)",
                "sn": sn,
            })
            logger.info("discovery: %s answered a unicast SN probe", target_ip)
    return found


def discover_device(ip: str, *, timeout: float = 1.5, host_ip: str = ""
                    ) -> Optional[Dict[str, Any]]:
    """Probe one address by unicast SN query (NAT/WSL-safe).

    The returned ``device_type``/``model`` stay ``None``/``unknown``: only the
    broadcast discovery ACK carries the device type (MID-360 = 9, MID-360S = 35).
    """
    sn = query_sn(ip, timeout=timeout, local_ip=host_ip)
    if not sn:
        return None
    return {
        "ip": ip,
        "device_type": None,
        "model": "unknown (unicast probe)",
        "sn": sn,
    }
