import hashlib
import json
from pathlib import Path
import tarfile

import cv2
import numpy as np
import open3d as o3d

root = Path(__file__).resolve().parents[3]
archive_root = Path(__file__).resolve().parent
filenames = {"independent_clouds.npz", "current-camera.png", "inspection.json",
             "fusion-report.json", "camera_roi.pcd", "lidar_roi.pcd",
             "merged_camera_frame.pcd", "lidar-panorama.png",
             "check_visibility.py", "render_fusion.py"}
captures = [("pose01", "fused-cloud-20261026-gyrIKgQI"),
            ("pose02", "fused-cloud-20261026-ITy6es0e")]
reports = []
for pose, directory in captures:
    source = root / ".scratch" / directory
    archive_path = archive_root / f"{pose}.tar.gz"
    members_report = []
    with tarfile.open(archive_path, "r:gz") as archive:
        members = {member.name: member for member in archive.getmembers() if member.isfile()}
        assert set(members) == {f"{directory}/{name}" for name in filenames}
        for name in sorted(filenames):
            stored = archive.extractfile(members[f"{directory}/{name}"]).read()
            original = (source / name).read_bytes()
            assert stored == original, f"{pose}/{name}"
            members_report.append({"file": name, "bytes": len(stored),
                                   "sha256": hashlib.sha256(stored).hexdigest()})
    inspection = json.loads((source / "inspection.json").read_text())
    report = json.loads((source / "fusion-report.json").read_text())
    with np.load(source / "independent_clouds.npz", allow_pickle=False) as arrays:
        assert arrays["lidar"].shape == (inspection["lidar_points"], 4)
        assert arrays["camera"].shape == (inspection["camera_points"], 3)
        assert np.isfinite(arrays["lidar"]).all() and np.isfinite(arrays["camera"]).all()
    counts = {}
    for name, expected in [("camera_roi", report["camera_roi_points"]),
                           ("lidar_roi", report["lidar_roi_points"]),
                           ("merged_camera_frame", report["merged_points"])]:
        cloud = o3d.io.read_point_cloud(str(source / f"{name}.pcd"))
        points = np.asarray(cloud.points)
        colors = np.asarray(cloud.colors)
        assert points.shape == colors.shape == (expected, 3)
        assert np.isfinite(points).all() and np.isfinite(colors).all()
        assert colors.min() >= 0.0 and colors.max() <= 1.0
        counts[name] = expected
    assert counts["merged_camera_frame"] == counts["camera_roi"] + counts["lidar_roi"]
    image = cv2.imread(str(source / "current-camera.png"))
    assert image is not None and image.shape == (800, 1280, 3)
    rendered = cv2.imread(str(root / report["image"]))
    assert rendered is not None and min(rendered.shape[:2]) > 1000
    reports.append({"pose": pose, "archive": str(archive_path.relative_to(root)),
                    "archive_bytes": archive_path.stat().st_size,
                    "archive_sha256": hashlib.sha256(archive_path.read_bytes()).hexdigest(),
                    "source_directory": str(source.relative_to(root)),
                    "archive_matches_source": True, "original_array_shapes_verified": True,
                    "point_cloud_counts": counts, "image": report["image"],
                    "files": members_report})
print(json.dumps({"verification_passed": True, "captures": reports}, indent=2))
