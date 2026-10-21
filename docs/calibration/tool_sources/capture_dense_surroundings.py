import json
import time
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2

work = Path(__file__).resolve().parent
root = work.parents[1]
clouds = []
latest = {}
def xyz(message):
    points = point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)
    values = np.column_stack([points[key] for key in ("x", "y", "z")])
    return values[np.isfinite(values).all(axis=1) & (np.linalg.norm(values, axis=1) > 0.01)]
def lidar_callback(message):
    assert message.header.frame_id == "livox_frame"
    clouds.append(xyz(message))
def camera_callback(message):
    latest["camera"] = message
def image_callback(message):
    latest["image"] = message
rclpy.init()
node = Node("capture_dense_surroundings")
subscriptions = [
    node.create_subscription(PointCloud2, "/livox/lidar", lidar_callback, qos_profile_sensor_data),
    node.create_subscription(PointCloud2, "/camera/depth_registered/points", camera_callback, qos_profile_sensor_data),
    node.create_subscription(Image, "/camera/color/image_raw", image_callback, qos_profile_sensor_data)]
start = time.monotonic()
while time.monotonic() - start < 10:
    rclpy.spin_once(node, timeout_sec=0.1)
duration = time.monotonic() - start
node.destroy_node()
rclpy.shutdown()
assert len(clouds) >= 70 and "camera" in latest and "image" in latest
assert latest["camera"].header.frame_id == "camera_color_optical_frame"
lidar = np.concatenate(clouds)
camera = xyz(latest["camera"])
calibration = json.loads((root / "docs/calibration/lidar_camera_20261021.json").read_text())
assert calibration["validation_passed"]
matrix = np.array(calibration["matrix"])
lidar_camera = lidar @ matrix[:3, :3].T + matrix[:3, 3]
image = CvBridge().imgmsg_to_cv2(latest["image"], desired_encoding="bgr8")
assert cv2.imwrite(str(work / "dense-environment.png"), image)
np.savez_compressed(work / "dense_clouds.npz", lidar=lidar, lidar_camera=lidar_camera, camera=camera)
report = {"duration_s": duration, "lidar_frames": len(clouds), "lidar_points": len(lidar),
          "camera_points": len(camera), "time_synchronization_verified": False,
          "static_accumulation": True}
(work / "dense_capture.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report))
