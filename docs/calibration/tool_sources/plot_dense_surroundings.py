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
assert cn.get_name() == "SimSun" and FontProperties(fname=en_path).get_name() == "Times New Roman"
plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams["axes.unicode_minus"] = False
data = np.load(work / "dense_clouds.npz")
report = json.loads((work / "dense_capture.json").read_text())
info = json.loads((work / "scene07/inspection.json").read_text())
intrinsic = np.array(info["camera_matrix"])
photo = cv2.imread(str(work / "dense-environment.png"))
assert photo is not None
photo = cv2.undistort(photo, intrinsic, np.array(info["distortion"]))
u_min, u_max, v_min, v_max = 350, 850, 270, 800
fig, axes = plt.subplots(1, 3, figsize=(18, 8.8), facecolor="white")
fig.suptitle("机械臂周边：相同视角放大查看", fontproperties=cn, fontsize=25, y=0.96)
axes[0].imshow(cv2.cvtColor(photo, cv2.COLOR_BGR2RGB))
axes[0].set_title("彩色画面参照", fontproperties=cn, fontsize=19, pad=14)
counts = {}
for axis, name, title in [(axes[1], "lidar_camera", "雷达点云：累计十秒"), (axes[2], "camera", "相机深度点云")]:
    points = data[name]
    points = points[(points[:, 2] > 0.2) & (points[:, 2] < 5.0)]
    pixels, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), intrinsic, None)
    pixels = pixels[:, 0]
    selected = (pixels[:, 0] >= u_min) & (pixels[:, 0] < u_max) & (pixels[:, 1] >= v_min) & (pixels[:, 1] < v_max)
    pixels, points = pixels[selected], points[selected]
    assert len(points) > 100
    counts[name] = len(points)
    order = np.argsort(points[:, 2])[::-1]
    axis.set_facecolor("#101a29")
    artist = axis.scatter(pixels[order, 0], pixels[order, 1], c=points[order, 2], s=1.0,
                          cmap="turbo", vmin=0.3, vmax=5, linewidths=0, rasterized=True)
    axis.set_title(title, fontproperties=cn, fontsize=19, pad=14)
    axis.text(0.5, -0.055, f"{len(points):,} measured points", transform=axis.transAxes,
              ha="center", fontsize=15)
for axis in axes:
    axis.set_xlim(u_min, u_max)
    axis.set_ylim(v_max, v_min)
    axis.set_aspect("equal")
    axis.set_xticks([])
    axis.set_yticks([])
fig.subplots_adjust(left=0.025, right=0.965, top=0.86, bottom=0.21, wspace=0.055)
bar_axis = fig.add_axes([0.37, 0.13, 0.46, 0.018])
fig.colorbar(artist, cax=bar_axis, orientation="horizontal", label="Z (m)")
fig.text(0.5, 0.045, "颜色表示距相机的前方距离；深色空白表示没有点云；静态累计，时间同步尚未验证", fontproperties=cn, fontsize=13, ha="center")
for text in fig.findobj(matplotlib.text.Text):
    if any(ord(char) > 127 for char in text.get_text()):
        assert all(ord(char) in FT2Font(cn_path).get_charmap() for char in text.get_text() if not char.isspace())
destination = root / "picture/robot_surroundings_dense.png"
fig.savefig(destination, dpi=200)
plt.close(fig)
assert cv2.imread(str(destination)).shape[:2] == (1760, 3600)
report["projected_roi_points"] = counts
(work / "dense_capture.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report))
print(destination)
