import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2


parser = argparse.ArgumentParser()
parser.add_argument("scene")
args = parser.parse_args()
root = Path(__file__).resolve().parent
scene = root / args.scene
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(scene), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("cdr", "cdr"))
clouds = []
info = None
while reader.has_next():
    topic, raw, stamp = reader.read_next()
    if topic == "/livox/lidar":
        msg = deserialize_message(raw, PointCloud2)
        points = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        data = np.column_stack([points[name] for name in ("x", "y", "z")])
        clouds.append(data)
    elif topic == "/camera/color/camera_info" and info is None:
        info = deserialize_message(raw, CameraInfo)
assert clouds and info is not None, "采集缺少点云或相机内参"
xyz = np.concatenate(clouds)
xyz = xyz[np.linalg.norm(xyz, axis=1) > 0.1]
np.save(scene / "xyz.npy", xyz)
image = cv2.imread(str(root / f"{args.scene}-color.png"))
assert image is not None
matrix = np.array(info.k).reshape(3, 3)
distortion = np.array(info.d)
rectified = cv2.undistort(image, matrix, distortion)
assert cv2.imwrite(str(scene / "rectified.png"), rectified)
corners, ids, rejected = cv2.aruco.detectMarkers(
    rectified, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250))
summary = {"points": len(xyz), "frames": len(clouds),
           "camera_matrix": matrix.tolist(), "distortion": distortion.tolist(),
           "marker_ids": [] if ids is None else ids.flatten().tolist(),
           "marker_corners": [c.tolist() for c in corners],
           "xyz_percentiles": np.percentile(xyz, [1, 25, 50, 75, 99], axis=0).tolist()}
(scene / "inspection.json").write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
# 将真实雷达点投影为角度图，亮度表示距离，供检查共同视野使用。
ranges = np.linalg.norm(xyz, axis=1)
azimuth = np.arctan2(xyz[:, 1], xyz[:, 0])
elevation = np.arctan2(xyz[:, 2], np.linalg.norm(xyz[:, :2], axis=1))
u = np.clip(((azimuth + np.pi) / (2 * np.pi) * 1800).astype(int), 0, 1799)
v = np.clip(((np.pi / 2 - elevation) / np.pi * 900).astype(int), 0, 899)
projection = np.zeros((900, 1800), np.uint8)
order = np.argsort(ranges)[::-1]
projection[v[order], u[order]] = np.clip(255 - ranges[order] * 25, 1, 255).astype(np.uint8)
assert cv2.imwrite(str(scene / "range.png"), projection)
