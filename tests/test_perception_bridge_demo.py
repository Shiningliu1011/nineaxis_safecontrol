#!/usr/bin/env python3
"""ROS 侧感知解码/契约测试 (不启动节点, 只测纯解码函数)。

覆盖 perception_demo 的消息解码逻辑 + obstacle_params.yaml 到 spec 的契约。
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))
sys.path.insert(0, str(REPO_ROOT / "portable_oscbf"))
sys.path.insert(0, str(REPO_ROOT / "portable_oscbf" / "work"))

from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout  # noqa: E402

from robot_safecontrol_moveit.perception_demo import (  # noqa: E402
    decode_esdf_message,
    decode_tracks_message,
)
from work.perception_config import load_point_cloud_collision, spec_of  # noqa: E402
from work.safety_snapshot import MAX_DYNAMIC_TRACKS  # noqa: E402
from work.safety_snapshot import preprocess_points, build_distance_field  # noqa: E402


def _esdf_msg(grid: np.ndarray) -> Float32MultiArray:
    shape = grid.shape
    strides = [shape[2] * shape[1], shape[2], 1]
    msg = Float32MultiArray()
    msg.layout = MultiArrayLayout(
        dim=[
            MultiArrayDimension(label="x", size=int(shape[0]), stride=int(strides[0])),
            MultiArrayDimension(label="y", size=int(shape[1]), stride=int(strides[1])),
            MultiArrayDimension(label="z", size=int(shape[2]), stride=int(strides[2])),
        ],
        data_offset=0,
    )
    msg.data = grid.astype(np.float32).ravel().tolist()
    return msg


def test_esdf_round_trip():
    """bridge 的 Float32MultiArray 布局 → demo 解码 → shape/dtype 一致。"""
    grid = np.random.default_rng(1).random((55, 41, 41)).astype(np.float32)
    decoded, shape = decode_esdf_message(_esdf_msg(grid))
    assert shape == grid.shape
    assert decoded.shape == grid.shape
    assert decoded.dtype == np.float32
    np.testing.assert_allclose(decoded, grid, rtol=1e-6)


def test_esdf_layout_matches_spec_shape():
    """bridge/demo 共用 obstacle_params.yaml → spec.shape 与 esdf 布局一致。"""
    spec = spec_of(load_point_cloud_collision())
    grid = np.zeros(spec.shape, dtype=np.float32)
    decoded, _ = decode_esdf_message(_esdf_msg(grid))
    assert decoded.shape == spec.shape


def test_tracks_round_trip():
    slots = np.zeros((MAX_DYNAMIC_TRACKS, 10), dtype=np.float32)
    slots[0, 0:3] = [0.1, 0.4, 1.3]
    slots[0, 3] = 0.05
    slots[0, 7] = 1.0
    msg = Float32MultiArray()
    msg.data = slots.ravel().tolist()
    decoded = decode_tracks_message(msg)
    assert decoded.shape == (MAX_DYNAMIC_TRACKS, 10)
    np.testing.assert_allclose(decoded[0], slots[0], rtol=1e-6)


def test_tracks_too_short_rejected():
    msg = Float32MultiArray()
    msg.data = [0.0] * 10  # 只给 1 个槽
    with pytest.raises(ValueError):
        decode_tracks_message(msg)


def test_preprocess_to_esdf_in_base_link():
    """传感器点云 (相机系) + 4x4 → preprocess → build_distance_field → 有效栅格。"""
    spec = spec_of(load_point_cloud_collision())
    rng = np.random.default_rng(2)
    # 相机系点云 (Z 前向), 经 4x4 变换进入 base_link workspace。
    sensor_pts = np.column_stack([
        rng.uniform(-0.3, 0.3, 1500),
        rng.uniform(0.1, 0.4, 1500),
        rng.uniform(0.9, 1.2, 1500),
    ])
    # 单位变换 (占位: world_frame==input 时 bridge 用 identity/camera_to_world_static)。
    world = preprocess_points(sensor_pts, np.eye(4), spec)
    assert world.shape[1] == 3 and len(world) > 0
    sdf = build_distance_field(world, spec)
    assert sdf.shape == spec.shape
    assert sdf.dtype == np.float32
    assert float(sdf.min()) == 0.0  # 有占据体素
    assert float(sdf.max()) > 0.0
