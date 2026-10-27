import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d

parser = argparse.ArgumentParser()
parser.add_argument("--calibration", required=True)
parser.add_argument("--scene", required=True)
args = parser.parse_args()
root = Path(__file__).resolve().parent
calibration = json.loads(Path(args.calibration).read_text())
assert calibration["validation_passed"], "必须通过独立验收"
assert calibration["frame_from"] == "livox_frame"
assert calibration["frame_to"] == "camera_color_optical_frame"
matrix = np.array(calibration["matrix"])
points = np.load(root / args.scene / "xyz.npy")
cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
cloud = cloud.voxel_down_sample(0.01)
cloud.transform(matrix)
assert o3d.io.write_point_cloud(str(root / args.scene / "lidar_in_camera_frame.pcd"), cloud)
points = np.asarray(cloud.points)
points = points[points[:, 2] > 0.1]
info = json.loads((root / args.scene / "inspection.json").read_text())
image = cv2.imread(str(root / args.scene / "rectified.png"))
assert image is not None
intrinsic = np.array(info["camera_matrix"])
pixels, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), intrinsic, None)
pixels = np.rint(pixels[:, 0]).astype(int)
inside = (pixels[:, 0] >= 0) & (pixels[:, 0] < image.shape[1]) & (pixels[:, 1] >= 0) & (pixels[:, 1] < image.shape[0])
points, pixels = points[inside], pixels[inside]
assert len(points) > 100
# 每个像素保留距离相机最近的雷达点，减小相机视线中的重复投影。
order = np.argsort(points[:, 2])
_, first = np.unique(pixels[order, 1] * image.shape[1] + pixels[order, 0], return_index=True)
selected = order[first]
points, pixels = points[selected], pixels[selected]
colored = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
colored.colors = o3d.utility.Vector3dVector(image[pixels[:, 1], pixels[:, 0], ::-1].astype(float) / 255.0)
assert o3d.io.write_point_cloud(str(root / args.scene / "colored_lidar_in_camera_frame.pcd"), colored)
print(json.dumps({"frame": "camera_color_optical_frame", "transformed_points": len(cloud.points),
                  "colored_points": len(points), "scene": args.scene, "static_scene_only": True}))
