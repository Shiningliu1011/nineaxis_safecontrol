"""Synthetic MID-360 frames for unit tests — no hardware, no network peer.

Builders here mirror what a real device sends, so the protocol/stream/ROS layers
can be exercised end to end over loopback UDP.
"""

from __future__ import annotations

import struct
from typing import Optional, Sequence

import numpy as np

from xy_mid_360_s.protocol import (
    CMD_TYPE_ACK,
    CMD_TYPE_REQ,
    KEY_LIDAR_TEMP,
    KEY_WORK_STATE,
    PCL_HDR_FMT,
    POINT_SIZE_HIGH,
    POINT_SIZE_LOW,
    SENDER_LIDAR,
    build_ctrl_frame,
    build_key_value,
    crc32_ieee,
)


def build_cloud_packet(
    xyz_m: np.ndarray,
    *,
    intensity: Optional[Sequence[int]] = None,
    tag: Optional[Sequence[int]] = None,
    data_type: int = 1,
    time_type: int = 0,
    timestamp_ns: int = 0,
    time_interval_raw: int = 1000,
    udp_cnt: int = 0,
    frame_cnt: int = 0,
    crc32: str = "valid",
) -> bytes:
    """Serialise points into a Livox point cloud UDP packet.

    ``xyz_m`` is (N, 3) in metres.  ``crc32`` selects the firmware behaviour:
    ``"valid"`` writes the real checksum, ``"zero"`` writes 0 (MID-360 pushes),
    ``"bad"`` writes a wrong value.
    """
    xyz = np.asarray(xyz_m, dtype=np.float64).reshape(-1, 3)
    n = xyz.shape[0]
    scale = 1000.0 if data_type == 1 else 100.0
    fmt = "<iiiBB" if data_type == 1 else "<hhhBB"
    point_size = POINT_SIZE_HIGH if data_type == 1 else POINT_SIZE_LOW
    refl = np.zeros(n, dtype=np.uint8) if intensity is None else np.asarray(intensity, dtype=np.uint8)
    tags = np.zeros(n, dtype=np.uint8) if tag is None else np.asarray(tag, dtype=np.uint8)
    body = b"".join(
        struct.pack(fmt, int(round(xyz[i, 0] * scale)), int(round(xyz[i, 1] * scale)),
                    int(round(xyz[i, 2] * scale)), int(refl[i]), int(tags[i]))
        for i in range(n))

    header_len = 36
    length = header_len + n * point_size
    if crc32 == "valid":
        crc_value = crc32_ieee(struct.pack("<Q", timestamp_ns) + body)
    elif crc32 == "bad":
        crc_value = 0xDEADBEEF
    else:
        crc_value = 0
    header = struct.pack(
        PCL_HDR_FMT, 0x05, length, time_interval_raw, n, udp_cnt, frame_cnt,
        data_type, time_type, b"\x00" * 12, crc_value, timestamp_ns)
    return header + body


def build_state_push(work_state: int, *, temperature_c: Optional[float] = None,
                     seq: int = 1) -> bytes:
    """Device-initiated state push (sender=LIDAR, cmd_type=REQ)."""
    entries = [build_key_value(KEY_WORK_STATE, struct.pack("<B", work_state))]
    if temperature_c is not None:
        entries.append(build_key_value(
            KEY_LIDAR_TEMP, struct.pack("<i", int(round(temperature_c * 100)))))
    data = struct.pack("<HH", len(entries), 0) + b"".join(entries)
    return build_ctrl_frame(0x0100, data, seq=seq, cmd_type=CMD_TYPE_REQ,
                            sender=SENDER_LIDAR)


def build_ack(cmd_id: int, data: bytes, *, seq: int) -> bytes:
    """Device ACK frame matching a request's cmd_id and seq_num."""
    return build_ctrl_frame(cmd_id, data, seq=seq, cmd_type=CMD_TYPE_ACK,
                            sender=SENDER_LIDAR)


def inquire_ack(key_values: Sequence[tuple], *, seq: int, cmd_id: int = 0x0101,
                ret_code: int = 0) -> bytes:
    """PARAM_INQUIRE ACK body: ret(1) + param_num(2) + {key, length, value}.

    Matches ``LivoxLidarDiagInternalInfoResponse`` in the vendor SDK — the
    response has **no** reserved field (unlike the request, and unlike the
    device's own push frames).
    """
    entries = b"".join(build_key_value(k, v) for k, v in key_values)
    data = struct.pack("<BH", ret_code, len(key_values)) + entries
    return build_ack(cmd_id, data, seq=seq)


def config_ack(*, seq: int, ret_code: int = 0) -> bytes:
    """PARAM_CONFIG ACK: a single return code byte."""
    return build_ack(0x0100, struct.pack("<B", ret_code), seq=seq)
