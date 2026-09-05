#!/usr/bin/env python3
"""Regression tests for PerceptionBridge._points_xyz (mixed-dtype PointCloud2 decode).

覆盖 1ebfe09 引入的 Livox 风格绕行解码：float32 x/y/z + uint8 tag/line 混合字段。
断言输出严格为 (N, 3)（不是 (N, 3, 1)）且数值一致，并对 format 假设
（is_bigendian / datatype / count / row_step）收紧。
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from sensor_msgs.msg import PointCloud2, PointField  # noqa: E402

from robot_safecontrol_moveit.perception_bridge import _points_xyz  # noqa: E402

_LIVOX_POINT_STEP = 16  # 3x float32 + uint8 tag + uint8 line + 6 bytes pad


def _livox_style_msg(xyz: np.ndarray) -> PointCloud2:
    """PointCloud2 in the Livox Mid-360S layout (mixed float32/uint8 fields)."""
    n = xyz.shape[0]
    payload = np.zeros((n, _LIVOX_POINT_STEP), dtype=np.uint8)
    for k, name in enumerate(("x", "y", "z")):
        col = np.ascontiguousarray(xyz[:, k], dtype=np.float32)
        payload[:, k * 4:(k + 1) * 4] = col.view(np.uint8).reshape(n, 4)
    payload[:, 12] = 1  # tag
    payload[:, 13] = 3  # line
    msg = PointCloud2()
    msg.header.frame_id = "livox_frame"
    msg.height = 1
    msg.width = n
    msg.is_bigendian = False
    msg.point_step = _LIVOX_POINT_STEP
    msg.row_step = _LIVOX_POINT_STEP * n
    msg.is_dense = True
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="tag", offset=12, datatype=PointField.UINT8, count=1),
        PointField(name="line", offset=13, datatype=PointField.UINT8, count=1),
    ]
    msg.data = payload.tobytes()
    return msg


def _homogeneous_msg(xyz: np.ndarray) -> PointCloud2:
    """All-float32 x/y/z cloud exercising the primary read_points_numpy path."""
    n = xyz.shape[0]
    pts = np.ascontiguousarray(xyz, dtype=np.float32)
    msg = PointCloud2()
    msg.header.frame_id = "camera_frame"
    msg.height = 1
    msg.width = n
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * n
    msg.is_dense = True
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = pts.tobytes()
    return msg


def test_mixed_dtype_fallback_returns_strict_2d_and_exact_xyz():
    xyz = np.array([
        [1.25, -2.5, 3.75],
        [0.1, 0.2, 0.3],
        [-5.0, 6.0, -7.0],
        [2.0, -3.0, 4.0],
        [0.5, 0.25, 0.125],
    ], dtype=np.float64)
    out = _points_xyz(_livox_style_msg(xyz))
    assert len(out.shape) == 2, "decoder must return (N, 3), not (N, 3, 1)"
    assert out.shape == (5, 3)
    np.testing.assert_allclose(out, xyz.astype(np.float32), rtol=0, atol=1e-6)


def test_homogeneous_dtype_happy_path():
    xyz = np.array([
        [0.0, 1.0, 2.0],
        [3.0, -4.0, 5.0],
        [-1.5, 0.5, -0.25],
        [2.75, -0.125, 1.0],
    ], dtype=np.float64)
    out = _points_xyz(_homogeneous_msg(xyz))
    assert out.shape == (4, 3)
    np.testing.assert_allclose(out, xyz.astype(np.float32), rtol=0, atol=1e-6)


def test_big_endian_fallback_rejected():
    msg = _livox_style_msg(np.zeros((3, 3)))
    msg.is_bigendian = True
    with pytest.raises(ValueError, match="big-endian"):
        _points_xyz(msg)


def test_non_float32_xyz_field_rejected():
    msg = _livox_style_msg(np.zeros((3, 3)))
    msg.fields[0].datatype = PointField.UINT32
    with pytest.raises(ValueError, match="FLOAT32"):
        _points_xyz(msg)


def test_bad_row_step_rejected():
    msg = _livox_style_msg(np.zeros((3, 3)))
    msg.row_step = msg.point_step * (msg.width + 1)
    with pytest.raises(ValueError, match="row_step"):
        _points_xyz(msg)
