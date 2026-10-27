import argparse
import json
from pathlib import Path

import cv2
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("scenes", nargs="+")
args = parser.parse_args()
root = Path(__file__).resolve().parent
for scene in args.scenes:
    info = json.loads((root / scene / "inspection.json").read_text())
    image = cv2.imread(str(root / scene / "rectified.png"))
    parameters = cv2.aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    corners, ids, _ = cv2.aruco.detectMarkers(image, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250), parameters=parameters)
    assert sorted(ids.ravel()) == [1, 2, 3, 4]
    positions = {1: (-0.55, 0.35), 2: (0.55, 0.35), 3: (-0.55, -0.35), 4: (0.55, -0.35)}
    objects = []
    for marker_id in ids.ravel():
        x, y = positions[marker_id]
        objects.extend([[x - 0.1, y + 0.1, 0], [x + 0.1, y + 0.1, 0], [x + 0.1, y - 0.1, 0], [x - 0.1, y - 0.1, 0]])
    objects = np.array(objects, dtype=np.float64)
    pixels = np.concatenate(corners).reshape(-1, 2)
    intrinsic = np.array(info["camera_matrix"])
    success, rotation, translation = cv2.solvePnP(objects, pixels, intrinsic, None)
    assert success
    projected, _ = cv2.projectPoints(objects, rotation, translation, intrinsic, None)
    errors = np.linalg.norm(projected[:, 0] - pixels, axis=1)
    report = {"scene": scene, "marker_reprojection_rmse_px": float(np.sqrt(np.mean(errors ** 2))),
              "marker_reprojection_max_px": float(errors.max()), "rvec": rotation.ravel().tolist(),
              "tvec": translation.ravel().tolist()}
    (root / scene / "camera-quality.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report))
    assert errors.max() < 3.0, report
