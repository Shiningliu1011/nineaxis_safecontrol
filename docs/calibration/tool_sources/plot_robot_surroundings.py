import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties, fontManager
from matplotlib.ft2font import FT2Font
import numpy as np

work = Path(__file__).resolve().parent
root = work.parents[1]
cn_path = "/home/lsn/.local/share/fonts/robot_figures/simsun.ttc"
en_path = "/home/lsn/.local/share/fonts/robot_figures/Times.TTF"
fontManager.addfont(cn_path)
fontManager.addfont(en_path)
cn = FontProperties(fname=cn_path)
assert cn.get_name() == "SimSun"
assert FontProperties(fname=en_path).get_name() == "Times New Roman"
plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams["axes.unicode_minus"] = False
data = np.load(work / "current_clouds.npz")
image = cv2.imread(str(work / "current-environment.png"))
assert image is not None
info = json.loads((work / "scene07/inspection.json").read_text())
intrinsic = np.array(info["camera_matrix"])
image = cv2.undistort(image, intrinsic, np.array(info["distortion"]))
lower = np.array([-1.5, -0.7, 0.3])
upper = np.array([1.2, 1.15, 4.8])
clouds = {}
for name, key in [("camera", "camera"), ("lidar", "lidar_camera")]:
    points = data[key]
    points = points[((points >= lower) & (points <= upper)).all(axis=1)]
    assert len(points) > 100
    _, indices = np.unique(np.floor(points / 0.008).astype(int), axis=0, return_index=True)
    clouds[name] = points[indices]
camera = clouds["camera"]
pixels, _ = cv2.projectPoints(camera, np.zeros(3), np.zeros(3), intrinsic, None)
pixels = np.rint(pixels[:, 0]).astype(int)
valid = (pixels[:, 0] >= 0) & (pixels[:, 0] < image.shape[1]) & (pixels[:, 1] >= 0) & (pixels[:, 1] < image.shape[0])
camera, pixels = camera[valid], pixels[valid]
colors = image[pixels[:, 1], pixels[:, 0], ::-1] / 255.0
fig = plt.figure(figsize=(16, 12), facecolor="white")
fig.suptitle("机械臂及周边观测", fontproperties=cn, fontsize=24, y=0.97)
layout = fig.add_gridspec(2, 2, left=0.065, right=0.97, bottom=0.12, top=0.91, hspace=0.32, wspace=0.23)
photo = fig.add_subplot(layout[0, 0])
photo.imshow(cv2.cvtColor(image[230:800, 310:1030], cv2.COLOR_BGR2RGB))
photo.set_title("机械臂附近彩色画面", fontproperties=cn, fontsize=17)
photo.axis("off")
spatial = fig.add_subplot(layout[0, 1], projection="3d")
step = max(1, int(np.ceil(len(camera) / 65000)))
selected = camera[::step]
spatial.scatter(selected[:, 0], selected[:, 2], -selected[:, 1], c=colors[::step], s=0.7, alpha=0.9)
spatial.set_title("局部相机彩色点云", fontproperties=cn, fontsize=17)
spatial.set(xlim=(-1.5, 1.2), ylim=(0.3, 4.8), zlim=(-1.15, 0.7), xlabel="X (m)", ylabel="Z (m)", zlabel="-Y (m)")
spatial.set_box_aspect((2.7, 4.5, 1.85))
spatial.view_init(elev=27, azim=-65)
top = fig.add_subplot(layout[1, 0])
side = fig.add_subplot(layout[1, 1])
for name, color, label in [("camera", "#3c8fc5", "Camera"), ("lidar", "#e86128", "LiDAR")]:
    points = clouds[name]
    step = max(1, int(np.ceil(len(points) / 45000)))
    points = points[::step]
    top.scatter(points[:, 0], points[:, 2], c=color, s=0.9 if name == "camera" else 3, alpha=0.4 if name == "camera" else 0.8, label=label)
    side.scatter(points[:, 2], -points[:, 1], c=color, s=0.9 if name == "camera" else 3, alpha=0.4 if name == "camera" else 0.8, label=label)
top.set(xlim=(-1.5, 1.2), ylim=(0.3, 4.8), xlabel="X (m)", ylabel="Z (m)")
side.set(xlim=(0.3, 4.8), ylim=(-1.15, 0.7), xlabel="Z (m)", ylabel="-Y (m)")
top.set_title("沿相机上下方向查看", fontproperties=cn, fontsize=17)
side.set_title("沿相机左右方向查看", fontproperties=cn, fontsize=17)
for axis in (top, side):
    axis.set_aspect("equal", adjustable="box")
    axis.grid(alpha=0.2)
    axis.legend(loc="upper right", markerscale=3)
fig.text(0.5, 0.065, "局部范围：相机前方零点三至四点八米；蓝色为相机点云，橙色为雷达点云", fontproperties=cn, fontsize=13, ha="center")
fig.text(0.5, 0.035, "坐标以相机为原点；空白区域可能没有观测；设备时间同步尚未验证", fontproperties=cn, fontsize=12, ha="center")
for text in fig.findobj(matplotlib.text.Text):
    if any(ord(char) > 127 for char in text.get_text()):
        assert all(ord(char) in FT2Font(cn_path).get_charmap() for char in text.get_text() if not char.isspace())
destination = root / "picture/robot_surroundings_capture.png"
fig.savefig(destination, dpi=180)
plt.close(fig)
assert cv2.imread(str(destination)).shape[:2] == (2160, 2880)
print(json.dumps({"roi_camera_frame_min_m": lower.tolist(), "roi_camera_frame_max_m": upper.tolist(),
                  "display_camera_points": len(camera), "display_lidar_points": len(clouds["lidar"])}))
print(destination)
