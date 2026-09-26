from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--julia", type=Path, required=True)
    parser.add_argument("--project", type=Path, required=True)
    parser.add_argument("--work-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    portable = Path(__file__).resolve().parents[3]
    sys.path.insert(0, str(portable))
    from work.nineaxis_kinematics import NineaxisKinematics

    q = [0.15, 0.1, -0.2, 0.3, 0.15, -0.25, 0.2, 0.05, -0.1]
    cases = []
    for name, distance in (("sphere-separated", 0.3), ("sphere-contact", 0.1), ("sphere-intersecting", 0.03)):
        cases.append({"name": name, "links": ["base_link", "Link1"],
                      "centers": [[0.0]*3, [distance, 0.0, 0.0]],
                      "radii": [[0.04]*3, [0.06]*3], "q": [0.0]*9})
    for name, radii in (
        ("size-disparity", [[1e-5]*3, [0.3]*3]),
        ("rotated", [[0.04, 0.003, 0.09], [0.01, 0.03, 0.06]]),
        ("near-degenerate", [[0.1, 0.0001, 0.03], [0.001, 0.03, 0.02]]),
    ):
        cases.append({"name": name, "links": ["base_link", "Link9"],
                      "centers": [[0.12, 0.07, -0.01], [0.04, -0.02, 0.01]],
                      "radii": radii, "q": q})
    rows = []
    for case in cases:
        poses = NineaxisKinematics().forward_kinematics(np.array(case["q"]))
        row = []
        for name, center, radii in zip(case["links"], case["centers"], case["radii"]):
            pose = poses[name]
            row.extend(pose[:3, :3] @ center + pose[:3, 3])
            row.extend((pose[:3, :3] @ np.diag(radii)).flatten(order="F"))
        rows.append(row)
    args.work_dir.mkdir(parents=True, exist_ok=True)
    inputs, outputs = args.work_dir / "inputs.csv", args.work_dir / "scales.csv"
    np.savetxt(inputs, rows, delimiter=",")
    source = Path(__file__).with_name("reference.jl")
    subprocess.run([str(args.julia), f"--project={args.project}", str(source), str(inputs), str(outputs)], check=True)
    scales = np.loadtxt(outputs)
    if len(scales) != len(cases) or not np.all(np.isfinite(scales)):
        raise ValueError("official DCOL returned invalid results")
    for case, scale in zip(cases, scales):
        case["proximity_scale"] = float(scale)
    revision = subprocess.check_output(["git", "-C", str(args.project), "rev-parse", "HEAD"], text=True).strip()
    document = {
        "source": "https://github.com/kevin-tracy/DifferentiableCollisions.jl",
        "revision": revision, "pdip_tol": 1e-12,
        "julia_version": subprocess.check_output([str(args.julia), "--version"], text=True).strip(),
        "reference_script_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "input_sha256": hashlib.sha256(inputs.read_bytes()).hexdigest(), "cases": cases,
    }
    args.output.write_text(json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
