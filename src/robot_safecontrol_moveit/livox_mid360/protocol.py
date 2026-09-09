"""Livox MID-360 / MID-360S Ethernet protocol: control frames and point cloud packets.

Pure Python, stdlib + numpy only — no ROS, no vendor SDK, no platform coupling.

Provenance
----------
Adapted for robot_safecontrol from the user-supplied ``tmbs-main`` archive
(``backend/app/drivers/mid360_driver.py``), which implements the Livox LiDAR
Communication Protocol v1.4.x.  The protocol facts below were cross-checked
against the pinned upstream sources already vendored in this repo's scratch
tree: ``livox_ros_driver2`` 1.2.6 (``src/comm/comm.h``, ``src/comm/pub_handler.cpp``)
and ``Livox-SDK2`` v1.3.1 (``include/livox_lidar_def.h``).

What differs from the source driver
-----------------------------------
* stdlib ``logging`` instead of the tmbs platform logger; no ``app.*`` imports.
* Point cloud decode keeps **every** point by default and returns ``tag``/``line``
  so the ROS adapter can emit the exact upstream PointCloud2 layout.  The source
  driver dropped points whose tag confidence bits were non-zero; that is now an
  explicit opt-in (``keep_all_points=False``).
* ``data_type=2`` (Cartesian low, centimetre int16) is decoded too; the source
  driver only handled ``data_type=1``.

Protocol summary (MID-360S, UDP)
-------------------------------
===  =========  ==========================================================
Port Direction  Use
===  =========  ==========================================================
56000  host→dev  broadcast discovery only (cmd 0x0000)
56100  both      control command channel (device listens here)
56200  dev→host  device status push, source port (dest configurable, 56201)
56300  dev→host  point cloud push, source port (dest configurable, 56301)
56400  dev→host  IMU push, source port (dest configurable, 56401)
===  =========  ==========================================================

Control frame header (24 bytes, little-endian)::

    offset size field
    0      1    sof          = 0xAA
    1      1    version      = 0x00
    2      2    length       whole frame length including header and data
    4      4    seq_num
    8      2    cmd_id
    10     1    cmd_type     0x00=REQ, 0x01=ACK
    11     1    sender_type  0x00=host, 0x01=device
    12     6    reserved
    18     2    crc16        CRC-16/CCITT-FALSE over bytes 0..17
    20     4    crc32        CRC-32 (IEEE 802.3) over the data field, 0 if empty
    24     n    data

Point cloud packet (36-byte header, no control header)::

    offset size field
    0      1    version
    1      2    length
    3      2    time_interval  unit 0.1 us, whole packet span
    5      2    dot_num        points in this packet
    7      2    udp_cnt
    9      1    frame_cnt
    10     1    data_type      1=Cartesian high (int32 mm), 2=Cartesian low (int16 cm)
    11     1    time_type      0=no sync, 1=gPTP/PTP, 2=GPS
    12     12   reserved
    24     4    crc32          MID-360 firmware writes 0; verified only when non-zero
    28     8    timestamp      device ns since epoch when time_type != 0
    36     n    data

``data_type=1`` points are 14 bytes each: ``x(i32 mm) y(i32 mm) z(i32 mm)
reflectivity(u8) tag(u8)``.  ``tag`` bits 0..1 are the confidence/echo field
(0 = high confidence); ``line`` is derived as ``index % 4`` for MID-360 family,
matching ``kLineNumberMid360`` in the upstream driver.
"""

from __future__ import annotations

import binascii
import logging
import socket
import struct
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

logger = logging.getLogger(__name__)

# ── Ports ────────────────────────────────────────────────────────────────────
PORT_BCAST_CMD = 56000
PORT_CTRL = 56100
PORT_SRC_STATE = 56200
PORT_PUSH_STATE = 56201
PORT_SRC_PCL = 56300
PORT_PUSH_PCL = 56301
PORT_SRC_IMU = 56400
PORT_PUSH_IMU = 56401

# ── Control frame constants ──────────────────────────────────────────────────
SOF = 0xAA
PROTO_VER = 0x00

CMD_TYPE_REQ = 0x00
CMD_TYPE_ACK = 0x01

SENDER_HOST = 0x00
SENDER_LIDAR = 0x01

CMD_DISCOVERY = 0x0000
CMD_PARAM_CONFIG = 0x0100
CMD_PARAM_INQUIRE = 0x0101
CMD_REBOOT = 0x0200

CTRL_HDR_LEN = 24
CTRL_HDR_FMT = "<BBH IH BB 6s H I"  # 24 bytes, no padding with "<"

# ── Device types (broadcast discovery ACK, data[1]) ──────────────────────────
DEV_TYPE_MID360 = 9
DEV_TYPE_MID360S = 35
DEV_TYPE_NAMES = {DEV_TYPE_MID360: "MID-360", DEV_TYPE_MID360S: "MID-360S"}
ACCEPTED_DEV_TYPES = frozenset(DEV_TYPE_NAMES)

# ── Parameter keys ───────────────────────────────────────────────────────────
KEY_PCL_DATA_TYPE = 0x0000
KEY_PATTERN_MODE = 0x0001
KEY_LIDAR_IP = 0x0004
KEY_STATE_HOST_CFG = 0x0005
KEY_PCL_HOST_CFG = 0x0006
KEY_IMU_HOST_CFG = 0x0007
KEY_FOV_CFG0 = 0x0015
KEY_FOV_CFG1 = 0x0016
KEY_FOV_CFG_EN = 0x0017
KEY_WORK_MODE = 0x001A
KEY_SN = 0x8000
KEY_WORK_STATE = 0x8006
KEY_LIDAR_TEMP = 0x8007

# ── Writable work modes (KEY_WORK_MODE) ──────────────────────────────────────
WORK_SAMPLING = 0x01  # Normal: scanning, laser and motor on
WORK_STANDBY = 0x02   # cold-start standby: scanning module and laser off
WORK_SLEEP = 0x03     # deep sleep
WORK_READY = 0x09     # MID-360/HAP ready: motor running, laser off, fast to SAMPLING

# ── Read-only work states (KEY_WORK_STATE) ───────────────────────────────────
WORK_STATE_NORMAL = 0x01
WORK_STATE_WAKEUP = 0x02
WORK_STATE_SLEEP = 0x03
WORK_STATE_ERROR = 0x04
WORK_STATE_SELFTEST = 0x05
WORK_STATE_MOTOR_STARTING = 0x06
WORK_STATE_MOTOR_STOPPING = 0x07
WORK_STATE_UPGRADE = 0x08
WORK_STATE_READY = 0x09

WORK_STATE_NAMES: Dict[int, str] = {
    WORK_STATE_NORMAL: "scanning",
    WORK_STATE_WAKEUP: "standby",
    WORK_STATE_SLEEP: "sleep",
    WORK_STATE_ERROR: "error",
    WORK_STATE_SELFTEST: "selftest",
    WORK_STATE_MOTOR_STARTING: "motor_starting",
    WORK_STATE_MOTOR_STOPPING: "motor_stopping",
    WORK_STATE_UPGRADE: "upgrade",
    WORK_STATE_READY: "ready",
}

# ── Point cloud packet constants ─────────────────────────────────────────────
PCL_HDR_FMT = "<B H H H H B B B 12s I Q"  # 36 bytes
PCL_HDR_LEN = struct.calcsize(PCL_HDR_FMT)
assert PCL_HDR_LEN == 36, PCL_HDR_LEN

DATA_TYPE_CARTESIAN_HIGH = 1  # int32 mm
DATA_TYPE_CARTESIAN_LOW = 2   # int16 cm
DATA_TYPE_SPHERICAL = 3       # uint32 depth mm + uint16 theta/phi (not decoded here)

# TimestampType in livox_ros_driver2 src/comm/comm.h:
# NoSync=0, GptpOrPtp=1, Gps=2. Only NoSync makes the driver fall back to host
# time; both synced values carry a device timestamp in the packet.
TIME_TYPE_NO_SYNC = 0
TIME_TYPE_PTP = 1   # gPTP or PTP
TIME_TYPE_GPS = 2

MID360_LINE_NUM = 4  # kLineNumberMid360 in livox_ros_driver2 src/comm/comm.h

POINT_SIZE_HIGH = 14  # x,y,z int32 + reflectivity u8 + tag u8
POINT_SIZE_LOW = 8    # x,y,z int16 + reflectivity u8 + tag u8

TAG_CONFIDENCE_MASK = 0x03  # tag bits 0..1: 0 = high confidence

# Point ceiling for a single assembled frame — the primary framing rule, not
# just a runaway guard.
#
# Fast-DDS (this project's RMW, rmw_fastrtps_cpp) carries a sample over its
# shared-memory transport only up to 512 KiB (524,288 B); a larger sample falls
# back to fragmented UDP and arrives at a fraction of the publish rate.  In the
# upstream point layout one point costs 26 B plus ~190 B of serialization
# overhead per message, so a frame must stay at or below ~20,150 points.
# Measured on hardware with a best-effort reader (sweep saved under
# .scratch/livox-hw-test/sweep_*.log): 20,000 points (520,000 B) and 20,100
# points (522,600 B) deliver a steady 10 Hz, while 20,160 points (524,160 B)
# collapse to 3.6 Hz with multi-second gaps and 21,000 points to 2.2 Hz.
#
# 20,000 points also matches the upstream driver, which publishes ~20,000
# points per frame at publish_freq=10 (the MID-360/MID-360S streams 200,000
# points/s), so consumers are already sized for it.  Raising this cap requires
# raising the SHM limit for every participant, publisher and subscriber alike.
MAX_FRAME_POINTS = 20_000


# ── CRC helpers ──────────────────────────────────────────────────────────────

def _build_crc16_table() -> List[int]:
    table: List[int] = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
        table.append(crc)
    return table


_CRC16_TABLE = _build_crc16_table()


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, no reflection."""
    crc = 0xFFFF
    for byte in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC16_TABLE[((crc >> 8) ^ byte) & 0xFF]
    return crc


def crc32_ieee(data: bytes) -> int:
    """CRC-32 (IEEE 802.3), as used by the Livox data-field checksum."""
    return binascii.crc32(data) & 0xFFFFFFFF


# ── Control frame build / parse ──────────────────────────────────────────────

def build_ctrl_frame(cmd_id: int, data: bytes = b"", seq: int = 0,
                     cmd_type: int = CMD_TYPE_REQ,
                     sender: int = SENDER_HOST) -> bytes:
    """Build a control frame.

    ``sender`` defaults to the host; tests and simulators use
    ``SENDER_LIDAR`` to construct device-initiated frames (state pushes, ACKs).
    """
    total_len = CTRL_HDR_LEN + len(data)
    hdr_nochek = struct.pack(
        "<BBH IH BB 6s",
        SOF, PROTO_VER, total_len,
        seq, cmd_id,
        cmd_type, sender,
        b"\x00" * 6,
    )  # 18 bytes
    crc16 = crc16_ccitt(hdr_nochek)
    crc32 = crc32_ieee(data) if data else 0
    return hdr_nochek + struct.pack("<HI", crc16, crc32) + data


def parse_ctrl_frame(raw: bytes) -> Optional[Dict[str, Any]]:
    """Parse a control frame; return its fields or ``None`` when invalid.

    Both checksums are verified: a frame with a bad CRC-16 header or a bad
    CRC-32 data field is rejected (logged at debug level).
    """
    if len(raw) < CTRL_HDR_LEN or raw[0] != SOF:
        return None
    try:
        (_sof, version, length, seq_num, cmd_id, cmd_type, sender,
         _resv, crc16, crc32) = struct.unpack_from(CTRL_HDR_FMT, raw, 0)
    except struct.error:
        return None
    if length < CTRL_HDR_LEN or length > len(raw):
        return None
    data_field = raw[CTRL_HDR_LEN:length]
    expected_crc16 = crc16_ccitt(raw[:18])
    if expected_crc16 != crc16:
        logger.debug("CRC16 mismatch: recv=%04x calc=%04x — frame discarded",
                     crc16, expected_crc16)
        return None
    if data_field:
        expected_crc32 = crc32_ieee(data_field)
        if expected_crc32 != crc32:
            logger.debug("CRC32 mismatch: recv=%08x calc=%08x — frame discarded",
                         crc32, expected_crc32)
            return None
    return {
        "version": version,
        "length": length,
        "seq_num": seq_num,
        "cmd_id": cmd_id,
        "cmd_type": cmd_type,
        "sender": sender,
        "crc16": crc16,
        "crc32": crc32,
        "data": data_field,
    }


def build_key_value(key: int, value: bytes) -> bytes:
    """One ``key_value`` entry: key(u16) + value_len(u16) + value."""
    return struct.pack("<HH", key, len(value)) + value


def build_param_config(kv_pairs: Sequence[Tuple[int, bytes]]) -> bytes:
    """Data field for ``CMD_PARAM_CONFIG``: key_num + rsvd + key_value_list."""
    kv_bytes = b"".join(build_key_value(k, v) for k, v in kv_pairs)
    return struct.pack("<HH", len(kv_pairs), 0) + kv_bytes


def build_param_inquire(keys: Sequence[int]) -> bytes:
    """Data field for ``CMD_PARAM_INQUIRE``: key_num + rsvd + key list."""
    return struct.pack("<HH", len(keys), 0) + b"".join(
        struct.pack("<H", key) for key in keys)


def parse_key_value_list(data: bytes) -> Dict[int, bytes]:
    """Parse a device response body into ``{key: value}``.

    Two shapes exist (see ``LivoxLidarDiagInternalInfoResponse`` and
    ``LivoxLidarKeyValueParam`` in the vendor SDK):

    * inquiry ACK: ``ret_code(1) + param_num(2) + {key(2), length(2), value}``
    * device push: ``key_num(2) + rsvd(2) + {key(2), length(2), value}``

    Returns an empty dict when the body does not parse as either shape.
    """
    if len(data) >= 3 and data[0] == 0:  # ret_code 0 = success
        parsed = _parse_entries(data, 3, struct.unpack_from("<H", data, 1)[0])
        if parsed is not None:
            return parsed
    if len(data) >= 4:
        parsed = _parse_entries(data, 4, struct.unpack_from("<H", data, 0)[0])
        if parsed is not None:
            return parsed
    return {}


def _parse_entries(data: bytes, pos: int, key_num: int) -> Optional[Dict[int, bytes]]:
    """Parse ``key_num`` ``{key, length, value}`` entries from ``pos``."""
    if key_num == 0 or key_num > 64:
        return None
    out: Dict[int, bytes] = {}
    for _ in range(key_num):
        if pos + 4 > len(data):
            return None
        key, val_len = struct.unpack_from("<HH", data, pos)
        pos += 4
        if pos + val_len > len(data):
            return None
        out[key] = data[pos:pos + val_len]
        pos += val_len
    return out


def ip_to_bytes(ip: str) -> bytes:
    return socket.inet_aton(ip)


def ip_config_value(host_ip: str, dest_port: int, src_port: int) -> bytes:
    """Push-destination value: IP(4) + dest_port(u16) + src_port(u16)."""
    return ip_to_bytes(host_ip) + struct.pack("<HH", dest_port, src_port)


# ── Point cloud packet parse ─────────────────────────────────────────────────

@dataclass
class CloudPacket:
    """One decoded UDP point cloud packet."""

    points: np.ndarray        # (N, 4) float32 [x, y, z, intensity] in metres
    tag: np.ndarray           # (N,) uint8
    line: np.ndarray          # (N,) uint8, index % 4 (MID-360 family)
    data_type: int
    time_type: int
    time_interval_raw: int    # raw field, unit 0.1 us
    dot_num: int
    udp_cnt: int
    frame_cnt: int
    timestamp_ns: int         # packet timestamp field as reported by the device

    @property
    def empty(self) -> bool:
        return self.points.shape[0] == 0


def parse_cloud_packet(raw: bytes, *,
                       keep_all_points: bool = True) -> Optional[CloudPacket]:
    """Decode a point cloud UDP packet.

    ``keep_all_points=False`` drops points whose ``tag`` confidence bits are
    non-zero (the source driver's behaviour).  Returns ``None`` when the packet
    is too short, carries an unsupported ``data_type``, or fails CRC-32.
    """
    if len(raw) < PCL_HDR_LEN:
        return None
    try:
        (version, length, time_interval, dot_num, udp_cnt, frame_cnt,
         data_type, time_type, _resv, crc32, timestamp) = struct.unpack_from(
            PCL_HDR_FMT, raw, 0)
    except struct.error:
        return None
    if version != 0x05:
        logger.debug("unexpected point cloud version %d (expected 5) — decoding anyway",
                     version)
    if data_type not in (DATA_TYPE_CARTESIAN_HIGH, DATA_TYPE_CARTESIAN_LOW):
        logger.debug("unsupported data_type %d — packet skipped", data_type)
        return None
    point_size = POINT_SIZE_HIGH if data_type == DATA_TYPE_CARTESIAN_HIGH else POINT_SIZE_LOW
    end = PCL_HDR_LEN + dot_num * point_size
    if dot_num == 0 or end > len(raw):
        return None
    # MID-360 firmware leaves crc32 at 0 in point cloud pushes; verify only
    # when the device actually computed it.
    if crc32 != 0 and crc32_ieee(raw[28:end]) != crc32:
        logger.debug("point cloud CRC32 mismatch — packet discarded")
        return None

    body = raw[PCL_HDR_LEN:end]
    if data_type == DATA_TYPE_CARTESIAN_HIGH:
        rec = np.frombuffer(body, dtype=np.dtype([
            ("x", "<i4"), ("y", "<i4"), ("z", "<i4"),
            ("intensity", "u1"), ("tag", "u1")]))
        xyz = np.stack([rec["x"], rec["y"], rec["z"]], axis=1).astype(np.float32) / 1000.0
    else:  # DATA_TYPE_CARTESIAN_LOW
        rec = np.frombuffer(body, dtype=np.dtype([
            ("x", "<i2"), ("y", "<i2"), ("z", "<i2"),
            ("intensity", "u1"), ("tag", "u1")]))
        xyz = np.stack([rec["x"], rec["y"], rec["z"]], axis=1).astype(np.float32) / 100.0

    tag = np.ascontiguousarray(rec["tag"], dtype=np.uint8)
    if not keep_all_points:
        keep = (tag & TAG_CONFIDENCE_MASK) == 0
        xyz = xyz[keep]
        tag = tag[keep]
        rec = rec[keep]
    n = xyz.shape[0]
    intensity = np.ascontiguousarray(rec["intensity"], dtype=np.float32)
    points = np.hstack([xyz, intensity.reshape(-1, 1)]).astype(np.float32, copy=False)
    line = (np.arange(n, dtype=np.uint32) % MID360_LINE_NUM).astype(np.uint8)
    return CloudPacket(
        points=points,
        tag=tag,
        line=line,
        data_type=data_type,
        time_type=time_type,
        time_interval_raw=time_interval,
        dot_num=dot_num,
        udp_cnt=udp_cnt,
        frame_cnt=frame_cnt,
        timestamp_ns=int(timestamp),
    )


def packet_time_base_ns(time_type: int, device_timestamp_ns: int,
                        host_now_ns: int) -> int:
    """Resolve the packet time base, mirroring upstream ``GetEthPacketTimestamp``.

    When the device is PTP/GPS synced (``time_type`` 1 or 2) the device timestamp
    is used; otherwise the driver stamps the packet with host time at receive.
    """
    if time_type in (TIME_TYPE_PTP, TIME_TYPE_GPS):
        return int(device_timestamp_ns)
    return int(host_now_ns)


def point_interval_ns(time_interval_raw: int, dot_num: int) -> int:
    """Per-point time step inside a packet, in ns (upstream: interval*100/dot_num)."""
    if dot_num <= 0:
        return 0
    return int(time_interval_raw * 100 // dot_num)
