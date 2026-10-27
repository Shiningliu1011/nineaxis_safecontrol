import argparse
import json
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties, fontManager
from matplotlib.ft2font import FT2Font
import numpy as np
import open3d as o3d
from scipy.optimize import linear_sum_assignment

parser = argparse.ArgumentParser()
parser.add_argument("--calibration", required=True, type=Path)
parser.add_argument("--scene", required=True)
args = parser.parse_args()
work = Path(__file__).resolve().parent
root = work.parents[1]
calibration = json.loads(args.calibration.read_text())
assert calibration["validation_passed"]
assert calibration["frame_from"] == "livox_frame"
assert calibration["frame_to"] == "camera_color_optical_frame"
assert args.scene in [item["scene"] for item in calibration["validation"]]
matrix = np.array(calibration["matrix"])
info = json.loads((work / args.scene / "inspection.json").read_text())
intrinsic = np.array(info["camera_matrix"])
photo = cv2.imread(str(work / args.scene / "rectified.png"))
assert photo is not None
cloud = o3d.io.read_point_cloud(str(work / args.scene / "lidar_in_camera_frame.pcd"))
points = np.asarray(cloud.points)
assert len(points) > 1000 and np.isfinite(points).all()
points = points[(points[:, 2] > 0.1) & (np.linalg.norm(points, axis=1) < 5.0)]
pixels, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), intrinsic, None)
pixels = pixels[:, 0]
inside = ((pixels[:, 0] >= 0) & (pixels[:, 0] < photo.shape[1])
          & (pixels[:, 1] >= 0) & (pixels[:, 1] < photo.shape[0]))
points, pixels = points[inside], pixels[inside]
# 同一图像小格保留最近雷达点，控制图中点数并保留可见表面。
cells = np.floor(pixels / 3).astype(int)
keys = cells[:, 1] * ((photo.shape[1] + 2) // 3) + cells[:, 0]
order = np.argsort(points[:, 2])
_, first = np.unique(keys[order], return_index=True)
points, pixels = points[order[first]], pixels[order[first]]
result_dir = work / f"{args.scene}-result"
lidar_centers = np.asarray(o3d.io.read_point_cloud(str(result_dir / "detected_lidar_centers.pcd")).points)
camera_centers = np.asarray(o3d.io.read_point_cloud(str(result_dir / "detected_camera_centers.pcd")).points)
predicted = lidar_centers @ matrix[:3, :3].T + matrix[:3, 3]
costs = np.linalg.norm(predicted[:, None] - camera_centers[None, :], axis=2)
rows, columns = linear_sum_assignment(costs)
center_pixels, _ = cv2.projectPoints(predicted, np.zeros(3), np.zeros(3), intrinsic, None)
target_pixels, _ = cv2.projectPoints(camera_centers[columns], np.zeros(3), np.zeros(3), intrinsic, None)
cn_path = "/home/lsn/.local/share/fonts/robot_figures/simsun.ttc"
en_path = "/home/lsn/.local/share/fonts/robot_figures/Times.TTF"
fontManager.addfont(cn_path)
fontManager.addfont(en_path)
cn = FontProperties(fname=cn_path)
assert cn.get_name() == "SimSun"
assert FontProperties(fname=en_path).get_name() == "Times New Roman"
plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams["axes.unicode_minus"] = False
fig, (full_axis, board_axis) = plt.subplots(1, 2, figsize=(17, 7), facecolor="white")
rgb = cv2.cvtColor(photo, cv2.COLOR_BGR2RGB)
full_axis.imshow(rgb)
artist = full_axis.scatter(pixels[:, 0], pixels[:, 1], c=points[:, 2],
                           s=0.9, cmap="turbo", vmin=0.3, vmax=5.0, alpha=0.8, linewidths=0)
full_axis.set(xlim=(0, photo.shape[1]), ylim=(photo.shape[0], 0))
full_axis.axis("off")
full_axis.set_title("标定后的雷达点云投影", fontproperties=cn, fontsize=19)
fig.colorbar(artist, ax=full_axis, shrink=0.75, label="Camera depth (m)")
board_axis.imshow(rgb)
board_axis.scatter(target_pixels[:, 0, 0], target_pixels[:, 0, 1], s=90,
                   marker="o", facecolors="none", edgecolors="lime", linewidths=1.5,
                   label="Camera board centers")
board_axis.scatter(center_pixels[:, 0, 0], center_pixels[:, 0, 1], s=80,
                   marker="+", c="red", linewidths=1.5, label="Projected LiDAR centers")
all_corners = np.array(info["marker_corners"]).reshape(-1, 2)
board_axis.set(xlim=(all_corners[:, 0].min() - 30, all_corners[:, 0].max() + 30),
               ylim=(all_corners[:, 1].max() + 25, all_corners[:, 1].min() - 25))
board_axis.axis("off")
board_axis.set_title("独立验收：四个圆孔中心", fontproperties=cn, fontsize=19)
board_axis.legend(loc="lower center", fontsize=11)
fig.text(0.5, 0.035, "静态标定板检验", fontproperties=cn, fontsize=15, ha="center")
fig.tight_layout(rect=(0, 0.06, 1, 1))
for text in fig.findobj(matplotlib.text.Text):
    if any(ord(char) > 127 for char in text.get_text()):
        assert all(ord(char) in FT2Font(cn_path).get_charmap()
                   for char in text.get_text() if not char.isspace())
output = root / "picture" / "lidar_camera_calibration_20261026.png"
fig.savefig(output, dpi=160, bbox_inches="tight", pad_inches=0.15)
plt.close(fig)
print(json.dumps({"image": str(output), "plotted_lidar_points": len(points),
                  "center_projection_errors_px": np.linalg.norm(center_pixels[:, 0] - target_pixels[:, 0], axis=1).tolist()}))
