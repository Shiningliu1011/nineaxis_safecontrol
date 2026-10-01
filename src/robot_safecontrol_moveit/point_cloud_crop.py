from __future__ import annotations

import array
from dataclasses import dataclass

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


@dataclass(frozen=True)
class CropRegion:
    center: tuple[float, float, float]
    size: tuple[float, float, float]

    def __post_init__(self) -> None:
        for name in ("center", "size"):
            values = np.asarray(getattr(self, name), dtype=np.float64)
            if values.shape != (3,) or not np.isfinite(values).all():
                raise ValueError(f"{name} 必须包含三个有限数值")
            object.__setattr__(self, name, tuple(float(v) for v in values))
        if min(self.size) <= 0:
            raise ValueError("size 的三个尺寸必须大于零")
        if not np.isfinite(self.bounds).all():
            raise ValueError("裁剪边界超出有限数值范围")

    @property
    def bounds(self) -> np.ndarray:
        center = np.asarray(self.center)
        half = np.asarray(self.size) / 2
        return np.array([center - half, center + half])


def cloud_xyz(cloud: PointCloud2) -> np.ndarray:
    # Humble read_points 不处理行间填充，大端读取还会原地修改输入。
    if cloud.is_bigendian:
        raise ValueError("实验输入必须为 little-endian PointCloud2")
    if cloud.point_step <= 0 or cloud.height < 1:
        raise ValueError("point_step 和 height 必须大于零")
    if cloud.row_step != cloud.width * cloud.point_step:
        raise ValueError("实验输入不支持 row_step 行间填充")
    if len(cloud.data) != cloud.height * cloud.row_step:
        raise ValueError("PointCloud2 data 长度与 row_step、height 不一致")
    fields = {field.name: field for field in cloud.fields}
    for name in ("x", "y", "z"):
        if name not in fields or fields[name].count != 1:
            raise ValueError(f"缺少标量字段 {name}")
        if fields[name].datatype not in (PointField.FLOAT32, PointField.FLOAT64):
            raise ValueError(f"{name} 必须为 FLOAT32 或 FLOAT64")
    points = point_cloud2.read_points(cloud, field_names=["x", "y", "z"])
    return np.column_stack([points[name] for name in ("x", "y", "z")])


def crop_mask(xyz: np.ndarray, region: CropRegion) -> np.ndarray:
    xyz = np.asarray(xyz)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("xyz 必须为 (N, 3)")
    lower, upper = region.bounds
    # 零向量为当前 Livox 数据流中的无效回波；边界上的有效点保留。
    return (
        np.isfinite(xyz).all(axis=1)
        & np.any(xyz != 0, axis=1)
        & np.all(xyz >= lower, axis=1)
        & np.all(xyz <= upper, axis=1)
    )


def crop_cloud(cloud: PointCloud2, region: CropRegion) -> PointCloud2:
    selected = crop_mask(cloud_xyz(cloud), region)
    # 按完整点记录复制，保留全部字段、逐点时间与填充字节。
    records = np.frombuffer(cloud.data, dtype=np.uint8).reshape(-1, cloud.point_step)
    data = array.array("B", records[selected].tobytes())
    count = int(selected.sum())
    output = PointCloud2(
        header=cloud.header,
        height=1,
        width=count,
        fields=cloud.fields,
        is_bigendian=cloud.is_bigendian,
        point_step=cloud.point_step,
        row_step=cloud.point_step * count,
        is_dense=cloud.is_dense,
    )
    output.data = data
    return output
