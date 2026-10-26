import json
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

work = Path(__file__).resolve().parent
root = work.parents[1]
result = json.loads((root / "docs/calibration/lidar_camera_20261026.json").read_text())
candidate = json.loads((work / "candidate-three-poses.json").read_text())
assert result["validation_passed"]
assert np.array_equal(result["matrix"], candidate["matrix"])
assert [item["scene"] for item in result["training"]] == ["scene03", "scene04", "scene05"]
assert [item["scene"] for item in result["validation"]] == ["scene06"]
matrix = np.array(result["matrix"])
assert np.isfinite(matrix).all()
assert np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-8)
assert np.isclose(np.linalg.det(matrix[:3, :3]), 1.0)
scene = work / "scene06"
raw_points = np.load(scene / "xyz.npy")
assert np.isfinite(raw_points).all()
expected = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(raw_points))
expected = expected.voxel_down_sample(0.01)
expected.transform(matrix)
cloud = o3d.io.read_point_cloud(str(scene / "lidar_in_camera_frame.pcd"))
colored = o3d.io.read_point_cloud(str(scene / "colored_lidar_in_camera_frame.pcd"))
points = np.asarray(cloud.points)
colored_points = np.asarray(colored.points)
colors = np.asarray(colored.colors)
assert points.shape == np.asarray(expected.points).shape and len(points) > 1000
assert colored_points.shape == colors.shape and len(colored_points) > 1000
assert np.isfinite(points).all() and np.isfinite(colored_points).all()
assert np.isfinite(colors).all() and colors.min() >= 0.0 and colors.max() <= 1.0
tree = cKDTree(points)
sample = np.asarray(expected.points)[np.linspace(0, len(points) - 1, 2000, dtype=int)]
errors, _ = tree.query(sample)
assert errors.max() < 0.0002
color_sample = colored_points[np.linspace(0, len(colored_points) - 1, 2000, dtype=int)]
color_errors, _ = tree.query(color_sample)
assert color_errors.max() < 0.00001
source_names = ["inspect_scene.py", "check_camera.py", "check_measurements.py",
                "evaluate_calibration.py", "apply_calibration.py", "plot_calibration.py",
                "verify_artifacts.py", "scene02.yaml", "solve_scene.sh"]
for name in source_names:
    assert (work / name).read_bytes() == (root / "docs/calibration/tool_sources_20261026" / name).read_bytes(), name
for relative in ["docs/calibration/lidar_camera_20261026.md",
                 "docs/calibration/lidar_camera_20261026.json",
                 "picture/lidar_camera_calibration_20261026.png"]:
    assert (root / relative).is_file(), relative
report = {"validation_passed": True, "held_out_matrix_unchanged": True,
          "transformed_points": len(points), "colored_points": len(colored_points),
          "serialization_max_error_m": float(errors.max()),
          "colored_subset_max_error_m": float(color_errors.max()),
          "source_snapshots_equal": True}
(work / "artifact-check.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report))
