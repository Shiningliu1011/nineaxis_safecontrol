import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import pdist

parser = argparse.ArgumentParser()
parser.add_argument("scenes", nargs="+")
args = parser.parse_args()
root = Path(__file__).resolve().parent
expected = np.sort([0.4, 0.4, 0.5, 0.5, np.hypot(0.4, 0.5), np.hypot(0.4, 0.5)])
model = np.array([[-0.25, 0.2, 0.0], [0.25, 0.2, 0.0],
                  [-0.25, -0.2, 0.0], [0.25, -0.2, 0.0]])
for scene in args.scenes:
    info = json.loads((root / scene / "inspection.json").read_text())
    camera_quality = json.loads((root / scene / "camera-quality.json").read_text())
    assert info["frames"] >= 70 and sorted(info["marker_ids"]) == [1, 2, 3, 4]
    assert camera_quality["marker_reprojection_max_px"] < 3.0
    centers = {}
    distances = {}
    for sensor in ("lidar", "camera"):
        path = root / f"{scene}-result" / f"detected_{sensor}_centers.pcd"
        points = np.asarray(o3d.io.read_point_cloud(str(path)).points)
        assert points.shape == (4, 3) and np.isfinite(points).all(), str(path)
        distances[sensor] = np.sort(pdist(points))
        assert np.max(np.abs(distances[sensor] - expected)) <= 0.020, str(path)
        centers[sensor] = points
    rotation, _ = cv2.Rodrigues(np.array(camera_quality["rvec"]))
    predicted = model @ rotation.T + np.array(camera_quality["tvec"])
    costs = np.linalg.norm(predicted[:, None] - centers["camera"][None, :], axis=2)
    rows, columns = linear_sum_assignment(costs)
    camera_consistency = costs[rows, columns]
    assert camera_consistency.max() <= 0.005, camera_consistency.tolist()
    report = {"scene": scene, "lidar_frames": info["frames"],
              "lidar_points": info["points"],
              "marker_reprojection_max_px": camera_quality["marker_reprojection_max_px"],
              "center_pair_distances_m": {key: value.tolist() for key, value in distances.items()},
              "camera_center_consistency_max_m": float(camera_consistency.max()),
              "measurements_passed": True}
    (root / scene / "measurement-quality.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report))
