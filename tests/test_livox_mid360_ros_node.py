#!/usr/bin/env python3
"""ROS-adapter tests: PointCloud2 layout and bridge compatibility.

Follows the repo convention of not spinning a node: only the pure
``frame_to_pointcloud2`` conversion is exercised.  The layout asserted here is
the one recorded from the official ``livox_ros_driver2`` on real MID-360S
hardware (ticket #31 online run), so a swap of data source is field-for-field
transparent to ``perception_bridge``.
"""

from __future__ import annotations

import array
import sys
import time
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from sensor_msgs.msg import PointCloud2, PointField  # noqa: E402

from robot_safecontrol_moveit.livox_mid360.ros_node import (  # noqa: E402
    _POINT_DTYPE,
    frame_to_pointcloud2,
)
from robot_safecontrol_moveit.livox_mid360.protocol import (  # noqa: E402
    MAX_FRAME_POINTS,
)
from robot_safecontrol_moveit.livox_mid360.stream import CloudFrame  # noqa: E402
from robot_safecontrol_moveit.perception_bridge import _points_xyz  # noqa: E402

# Layout recorded from the official driver on hardware (point_step 26).
_OFFICIAL_FIELDS = [
    ("x", 0, PointField.FLOAT32),
    ("y", 4, PointField.FLOAT32),
    ("z", 8, PointField.FLOAT32),
    ("intensity", 12, PointField.FLOAT32),
    ("tag", 16, PointField.UINT8),
    ("line", 17, PointField.UINT8),
    ("timestamp", 18, PointField.FLOAT64),
]


def _frame(n: int = 3, base_ns: int = 1_788_952_568_081_652_000) -> CloudFrame:
    rng = np.random.default_rng(7)
    xyz = rng.uniform(-2.0, 2.0, size=(n, 3)).astype(np.float32)
    intensity = rng.uniform(0, 255, size=(n, 1)).astype(np.float32)
    points = np.hstack([xyz, intensity])
    times = base_ns + np.arange(n, dtype=np.int64) * 1000
    return CloudFrame(
        points=points,
        tag=np.arange(n, dtype=np.uint8),
        line=(np.arange(n) % 4).astype(np.uint8),
        timestamp_ns=times.astype(np.float64),
        base_time_ns=base_ns,
        src_ip="192.168.1.115",
    )


def test_point_dtype_is_packed_to_26_bytes():
    assert _POINT_DTYPE.itemsize == 26
    assert [(f[0], _POINT_DTYPE.fields[f[0]][1]) for f in _POINT_DTYPE.descr] == [
        ("x", 0), ("y", 4), ("z", 8), ("intensity", 12),
        ("tag", 16), ("line", 17), ("timestamp", 18)]


def test_pointcloud2_layout_matches_official_driver():
    frame = _frame()
    msg = frame_to_pointcloud2(frame, frame_id="livox_frame",
                               stamp_ns=frame.base_time_ns)
    assert msg.point_step == 26
    assert msg.height == 1
    assert msg.width == frame.size
    assert msg.row_step == 26 * frame.size
    assert msg.is_bigendian is False
    assert msg.is_dense is True
    assert msg.header.frame_id == "livox_frame"
    assert msg.header.stamp.sec == 1_788_952_568
    assert msg.header.stamp.nanosec == 81_652_000
    assert [(f.name, f.offset, f.datatype) for f in msg.fields] == _OFFICIAL_FIELDS


def test_bridge_decoder_reads_our_pointcloud2():
    """perception_bridge._points_xyz consumes the message unchanged."""
    frame = _frame()
    msg = frame_to_pointcloud2(frame, frame_id="livox_frame",
                               stamp_ns=frame.base_time_ns)
    decoded = _points_xyz(msg)
    assert decoded.shape == (frame.size, 3)
    np.testing.assert_allclose(decoded, frame.points[:, :3], rtol=0, atol=1e-6)


def test_tag_line_timestamp_fields_round_trip():
    frame = _frame()
    msg = frame_to_pointcloud2(frame, frame_id="livox_frame",
                               stamp_ns=frame.base_time_ns)
    decoded = np.frombuffer(msg.data, dtype=_POINT_DTYPE)
    np.testing.assert_array_equal(decoded["tag"], frame.tag)
    np.testing.assert_array_equal(decoded["line"], frame.line)
    np.testing.assert_allclose(decoded["timestamp"], frame.timestamp_ns)
    np.testing.assert_allclose(decoded["intensity"], frame.points[:, 3])


def test_payload_assignment_stays_off_the_per_element_path():
    """Guard the hardware-measured bottleneck: bytes → uint8[] is 33 ms for a
    20k-point frame because rclpy converts element by element, which stalls the
    receive thread and overflows the kernel UDP buffer.  array.array copies the
    buffer in C; the bound below is ~50x the measured cost and ~6x below the
    regression."""
    frame = _frame(20_000)
    t0 = time.perf_counter()
    msg = frame_to_pointcloud2(frame, frame_id="livox_frame", stamp_ns=frame.base_time_ns)
    elapsed = time.perf_counter() - t0
    assert isinstance(msg.data, array.array) and msg.data.typecode == "B"
    assert len(msg.data) == 20_000 * 26
    assert elapsed < 5e-3, f"frame_to_pointcloud2 took {elapsed*1e3:.1f} ms"


def test_frame_cap_keeps_messages_inside_the_rmw_shared_memory_limit():
    """Fast-DDS carries a sample over its shared-memory transport only up to
    512 KiB; above that it falls back to fragmented UDP and most frames are
    lost.  Measured on hardware with a best-effort reader: 20,100 points
    (522,600 B) delivered a steady 10 Hz, 20,160 points (524,160 B) collapsed
    to 3.6 Hz with multi-second gaps.  The frame cap must leave room for the
    CDR overhead that sits on top of the point payload, so allow 4 KiB."""
    overhead_allowance = 4 * 1024  # header, field metadata, encapsulation
    assert MAX_FRAME_POINTS * _POINT_DTYPE.itemsize + overhead_allowance < 512 * 1024
    frame = _frame(MAX_FRAME_POINTS)
    msg = frame_to_pointcloud2(frame, frame_id="livox_frame",
                               stamp_ns=frame.base_time_ns)
    assert len(msg.data) == MAX_FRAME_POINTS * 26
    assert len(msg.data) + overhead_allowance < 512 * 1024


def test_empty_frame_produces_valid_message():
    frame = CloudFrame(
        points=np.empty((0, 4), dtype=np.float32),
        tag=np.empty(0, dtype=np.uint8),
        line=np.empty(0, dtype=np.uint8),
        timestamp_ns=np.empty(0, dtype=np.float64),
        base_time_ns=123,
    )
    msg = frame_to_pointcloud2(frame, frame_id="livox_frame", stamp_ns=123)
    assert msg.width == 0
    assert msg.row_step == 0
    assert len(msg.data) == 0
    assert _points_xyz(msg).shape == (0, 3)
