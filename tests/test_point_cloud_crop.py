from copy import deepcopy

import numpy as np
import pytest

from robot_safecontrol_moveit.livox_mid360.ros_node import frame_to_pointcloud2
from robot_safecontrol_moveit.livox_mid360.stream import CloudFrame
from robot_safecontrol_moveit.point_cloud_crop import CropRegion, cloud_xyz, crop_cloud


def make_cloud(xyz):
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    count = len(xyz)
    frame = CloudFrame(
        points=np.column_stack([xyz, np.arange(count, dtype=np.float32)]),
        tag=np.arange(count, dtype=np.uint8),
        line=np.arange(count, dtype=np.uint8) % 4,
        timestamp_ns=1_790_000_000_000_000_000 + np.arange(count, dtype=np.float64) * 10000,
        base_time_ns=1_790_000_000_000_000_000,
    )
    return deepcopy(frame_to_pointcloud2(frame, frame_id="livox_frame", stamp_ns=frame.base_time_ns))


def test_boundaries_invalid_returns_and_complete_records():
    cloud = make_cloud([
        [-1, 0, 0], [1, 0, 0], [0, -1, 0], [0, 1, 0], [0, 0, -1], [0, 0, 1],
        [0.25, 0.25, 0.25], [1.0001, 0, 0], [0, 0, 0],
        [float("nan"), 0, 0], [0, float("inf"), 0], [-1.0001, 0, 0],
    ])
    original = deepcopy(cloud)
    result = crop_cloud(cloud, CropRegion([0, 0, 0], [2, 2, 2]))
    assert result.width == 7
    assert result.data == original.data[:7 * cloud.point_step]
    assert result.header == original.header
    assert result.fields == original.fields
    assert result.point_step == original.point_step == 26
    assert result.row_step == 7 * 26
    assert cloud == original


def test_crop_is_relative_to_selected_center():
    cloud = make_cloud([[0.5, 0, 0], [5, 0, 0], [5.4, 0, 0], [6, 0, 0]])
    result = crop_cloud(cloud, CropRegion([5, 0, 0], [1, 1, 1]))
    np.testing.assert_array_equal(cloud_xyz(result), cloud_xyz(cloud)[1:3])


def test_empty_region_and_empty_input_keep_layout():
    cloud = make_cloud([[5, 0, 0]])
    region = CropRegion([0, 0, 0], [2, 2, 2])
    empty = crop_cloud(cloud, region)
    assert empty.width == empty.row_step == len(empty.data) == 0
    assert empty.point_step == cloud.point_step
    assert empty.header == cloud.header
    assert crop_cloud(empty, region) == empty


@pytest.mark.parametrize("center,size", [
    ([0, 0], [2, 2, 2]), ([0, float("nan"), 0], [2, 2, 2]),
    ([0, 0, 0], [0, 2, 2]), ([0, 0, 0], [-1, 2, 2]),
    ([0, 0, 0], [2, float("inf"), 2]),
])
def test_invalid_region_rejected(center, size):
    with pytest.raises(ValueError):
        CropRegion(center, size)


@pytest.mark.parametrize("change,match", [
    ("endian", "little-endian"), ("row", "row_step"),
    ("data", "data"), ("missing_x", "标量字段 x"), ("integer_x", "FLOAT32"),
])
def test_unsupported_cloud_rejected_without_mutation(change, match):
    cloud = make_cloud([[1, 0, 0]])
    if change == "endian":
        cloud.is_bigendian = True
    elif change == "row":
        cloud.row_step += 1
    elif change == "data":
        cloud.data.pop()
    elif change == "missing_x":
        cloud.fields = cloud.fields[1:]
    elif change == "integer_x":
        cloud.fields[0].datatype = 6
    original = deepcopy(cloud)
    with pytest.raises(ValueError, match=match):
        crop_cloud(cloud, CropRegion([0, 0, 0], [2, 2, 2]))
    assert cloud == original
