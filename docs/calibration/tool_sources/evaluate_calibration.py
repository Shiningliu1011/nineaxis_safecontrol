import argparse
import itertools
import json
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation
from scipy.spatial.distance import pdist


def read_centers(root, scene, sensor):
    path = root / f"{scene}-result" / f"detected_{sensor}_centers.pcd"
    points = np.asarray(o3d.io.read_point_cloud(str(path)).points)
    assert points.shape == (4, 3) and np.isfinite(points).all(), str(path)
    expected = np.sort([0.4, 0.4, 0.5, 0.5, np.hypot(0.4, 0.5), np.hypot(0.4, 0.5)])
    assert np.max(np.abs(np.sort(pdist(points)) - expected)) <= 0.02, str(path)
    return points


def fit(source, target):
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    rotation, _ = Rotation.align_vectors(target - target_mean, source - source_mean)
    matrix = rotation.as_matrix()
    translation = target_mean - matrix @ source_mean
    return matrix, translation


def match(source, target, matrix, translation):
    prediction = source @ matrix.T + translation
    distance = np.linalg.norm(prediction[:, None, :] - target[None, :, :], axis=2)
    rows, columns = linear_sum_assignment(distance)
    assert np.array_equal(rows, np.arange(4))
    return target[columns], columns


def evaluate(root, scene, matrix, translation):
    source = read_centers(root, scene, "lidar")
    target = read_centers(root, scene, "camera")
    target, assignment = match(source, target, matrix, translation)
    prediction = source @ matrix.T + translation
    errors = np.linalg.norm(prediction - target, axis=1)
    info = json.loads((root / scene / "inspection.json").read_text())
    intrinsic = np.array(info["camera_matrix"])
    predicted_pixels, _ = cv2.projectPoints(prediction, np.zeros(3), np.zeros(3), intrinsic, None)
    target_pixels, _ = cv2.projectPoints(target, np.zeros(3), np.zeros(3), intrinsic, None)
    pixel_errors = np.linalg.norm(predicted_pixels[:, 0] - target_pixels[:, 0], axis=1)
    plane = np.asarray(o3d.io.read_point_cloud(str(root / f"{scene}-result" / "board_plane.pcd")).points)
    assert len(plane) > 100
    _, _, basis = np.linalg.svd(target - target.mean(axis=0))
    plane_errors = np.abs((plane @ matrix.T + translation - target.mean(axis=0)) @ basis[-1])
    return {"scene": scene, "assignment": assignment.tolist(),
            "center_errors_m": errors.tolist(), "center_rmse_m": float(np.sqrt(np.mean(errors**2))),
            "center_max_m": float(errors.max()), "reprojection_errors_px": pixel_errors.tolist(),
            "plane_distance_median_m": float(np.median(plane_errors)),
            "plane_distance_p95_m": float(np.percentile(plane_errors, 95))}


parser = argparse.ArgumentParser()
parser.add_argument("--train", nargs="+", required=True)
parser.add_argument("--validation", nargs="+", default=[])
parser.add_argument("--output", required=True)
args = parser.parse_args()
assert len(args.train) >= 3, "联合标定需要至少三个姿态"
assert not set(args.train) & set(args.validation), "验收数据必须独立"
root = Path(__file__).resolve().parent
sources = [read_centers(root, scene, "lidar") for scene in args.train]
targets = [read_centers(root, scene, "camera") for scene in args.train]
candidates = []
# 矩形孔位具有对称性，通过多个真实姿态共同确定中心对应关系。
for permutation in itertools.permutations(range(4)):
    matrix, translation = fit(sources[0], targets[0][list(permutation)])
    for iteration in range(10):
        matched = [match(s, t, matrix, translation)[0] for s, t in zip(sources, targets)]
        matrix, translation = fit(np.concatenate(sources), np.concatenate(matched))
    residuals = np.concatenate([s @ matrix.T + translation - t for s, t in zip(sources, matched)])
    rmse = float(np.sqrt(np.mean(np.sum(residuals**2, axis=1))))
    candidates.append((rmse, matrix, translation))
candidates.sort(key=lambda item: item[0])
rmse, matrix, translation = candidates[0]
assert np.isfinite(matrix).all() and np.isfinite(translation).all()
assert np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-8)
assert np.isclose(np.linalg.det(matrix), 1.0)
transform = np.eye(4)
transform[:3, :3], transform[:3, 3] = matrix, translation
training = [evaluate(root, scene, matrix, translation) for scene in args.train]
validation = [evaluate(root, scene, matrix, translation) for scene in args.validation]
alternatives = [item[0] for item in candidates
                if Rotation.from_matrix(item[1] @ matrix.T).magnitude() > 0.05
                or np.linalg.norm(item[2] - translation) > 0.05]
alternative_rmse = min(alternatives) if alternatives else None
unambiguous = alternative_rmse is None or alternative_rmse > rmse + 0.020
passed = bool(validation) and unambiguous and all(
    item["center_max_m"] <= 0.020 and item["plane_distance_p95_m"] <= 0.020
    and max(item["reprojection_errors_px"]) <= 5.0 for item in training + validation)
result = {"frame_from": "livox_frame", "frame_to": "camera_color_optical_frame",
          "units": "m", "matrix": transform.tolist(), "training_rmse_m": rmse,
          "training": training, "validation": validation, "validation_passed": passed,
          "alternative_correspondence_rmse_m": alternative_rmse,
          "acceptance": {"center_max_m": 0.020, "plane_p95_m": 0.020, "reprojection_max_px": 5.0},
          "board_dimensions_source": "/home/lsn/robot/fast-calib标定板介绍.zip",
          "lidar_serial": "ARMCP7D0030415", "camera_serial": "CP28563000GD",
          "time_synchronization_verified": False,
          "base_link_transform_verified": False}
Path(args.output).write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
if validation and not passed:
    raise SystemExit("独立验收未满足 20 mm 条件")
