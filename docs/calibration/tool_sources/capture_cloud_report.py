import json
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties, fontManager
from matplotlib.ft2font import FT2Font
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

root = Path(__file__).resolve().parents[2]
work = Path(__file__).resolve().parent
output = root / "picture"
output.mkdir(exist_ok=True)
chinese_path = "/home/lsn/.local/share/fonts/robot_figures/simsun.ttc"
english_path = "/home/lsn/.local/share/fonts/robot_figures/Times.TTF"
fontManager.addfont(chinese_path)
fontManager.addfont(english_path)
chinese = FontProperties(fname=chinese_path)
english = FontProperties(fname=english_path)
assert chinese.get_name() == "SimSun"
assert english.get_name() == "Times New Roman"
chinese_text = "激光雷达与相机点云采集雷达点云深度相机点云共同视野空间投影当前采集空间变换采用已验证外参设备时间同步尚未验证显示抽样完整数据已保存"
assert all(ord(char) in FT2Font(chinese_path).get_charmap() for char in chinese_text)
plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams["axes.unicode_minus"] = False

rclpy.init()
node = Node("capture_cloud_report")
samples = {"lidar": [], "camera": []}
latest = {}
def callback(name, message):
    samples[name].append(time.monotonic())
    latest[name] = message
subscriptions = [node.create_subscription(PointCloud2, topic,
                 lambda message, name=name: callback(name, message), qos_profile_sensor_data)
                 for name, topic in [("lidar", "/livox/lidar"), ("camera", "/camera/depth_registered/points")]]
start = time.monotonic()
while time.monotonic() - start < 6:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
assert all(len(value) >= 2 for value in samples.values()), {key: len(value) for key, value in samples.items()}
clouds = {}
statistics = {}
for name, message in latest.items():
    values = point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)
    points = np.column_stack([values[key] for key in ("x", "y", "z")])
    points = points[np.isfinite(points).all(axis=1) & (np.linalg.norm(points, axis=1) > 0.01)]
    assert len(points) > 100
    clouds[name] = points
    statistics[name] = {"topic": "/livox/lidar" if name == "lidar" else "/camera/depth_registered/points",
                        "frame_id": message.header.frame_id, "valid_points_last_frame": len(points),
                        "received_frames": len(samples[name]),
                        "observed_hz": (len(samples[name]) - 1) / (samples[name][-1] - samples[name][0]),
                        "header_stamp_s": message.header.stamp.sec + message.header.stamp.nanosec * 1e-9}
assert statistics["lidar"]["frame_id"] == "livox_frame", statistics
assert statistics["camera"]["frame_id"] == "camera_color_optical_frame", statistics
calibration = json.loads((root / "docs/calibration/lidar_camera_20261021.json").read_text())
assert calibration["validation_passed"]
matrix = np.array(calibration["matrix"])
transformed = clouds["lidar"] @ matrix[:3, :3].T + matrix[:3, 3]
statistics["time_synchronization_verified"] = False
statistics["collection_duration_s"] = time.monotonic() - start
np.savez_compressed(work / "current_clouds.npz", lidar=clouds["lidar"], camera=clouds["camera"], lidar_camera=transformed)
(work / "current_cloud_report.json").write_text(json.dumps(statistics, indent=2))

fig = plt.figure(figsize=(18, 8), facecolor="white")
fig.suptitle("激光雷达与相机点云采集", fontproperties=chinese, fontsize=25, y=0.96)
axes = [fig.add_subplot(1, 3, index + 1, projection="3d") for index in range(3)]
def draw(axis, points, title, fixed_color=None):
    selected = points[::max(1, int(np.ceil(len(points) / 25000)))]
    axis.scatter(selected[:, 0], selected[:, 2], -selected[:, 1], s=0.5,
                 c=fixed_color if fixed_color else selected[:, 2], cmap=None if fixed_color else "viridis", alpha=0.65)
    axis.set_title(title, fontproperties=chinese, fontsize=18, pad=16)
    axis.set_xlabel("X (m)", fontproperties=english)
    axis.set_ylabel("Z (m)", fontproperties=english)
    axis.set_zlabel("-Y (m)", fontproperties=english)
    axis.view_init(elev=23, azim=-65)
    axis.set_box_aspect((1, 1.2, 0.8))
draw(axes[0], transformed, "雷达点云")
draw(axes[1], clouds["camera"], "深度相机点云")
camera = clouds["camera"]
low, high = np.percentile(camera, [1, 99], axis=0)
inside = ((transformed >= low) & (transformed <= high)).all(axis=1)
assert inside.sum() > 10
draw(axes[2], camera, "共同视野空间投影", "#3284bf")
draw(axes[2], transformed[inside], "共同视野空间投影", "#e6672c")
axes[2].text2D(0.06, 0.02, "Camera: blue   LiDAR: orange", transform=axes[2].transAxes,
               fontproperties=english, fontsize=12)
for index, name in enumerate(["lidar", "camera"]):
    data = statistics[name]
    fig.text(0.18 + index * 0.32, 0.20,
             f'{data["valid_points_last_frame"]:,} points/frame | {data["observed_hz"]:.1f} Hz\n{data["received_frames"]} frames received / 6 s',
             fontproperties=english, fontsize=14, ha="center")
fig.text(0.5, 0.11, "当前采集；空间变换采用已验证外参；设备时间同步尚未验证", fontproperties=chinese, ha="center", fontsize=14)
fig.text(0.5, 0.065, "显示抽样；完整数据已保存", fontproperties=chinese, ha="center", fontsize=12)
fig.subplots_adjust(left=0.025, right=0.975, top=0.84, bottom=0.28, wspace=0.09)
destination = output / "lidar_camera_current_capture.png"
fig.savefig(destination, dpi=180, facecolor="white")
plt.close(fig)
print(json.dumps(statistics, indent=2))
print(destination)
