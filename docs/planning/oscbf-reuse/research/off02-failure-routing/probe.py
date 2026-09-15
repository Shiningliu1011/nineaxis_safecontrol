"""Read-only point checks for OFF-02 decisions; not a region certificate.

Run from any cwd: python3 <path-to-this-file> > probe.json
Only stdout is written. No ROS, processes, hardware or production mutations.
"""
import hashlib
import itertools
import json
from pathlib import Path
import sys

import numpy as np
import fcl

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "portable_oscbf/work").is_dir())
sys.path.insert(0, str(ROOT / "portable_oscbf"))
from work.nineaxis_kinematics import NineaxisKinematics
from work import obb_collision_model as model

robot = NineaxisKinematics()
names = model.OBB_LINK_NAMES
pairs = [(i, j) for i, j in itertools.combinations(range(10), 2) if j-i >= 2]
online = {tuple(p) for p in model.OBB_COLLISION_PAIRS.tolist()}


def evaluate(q):
    transforms = robot.forward_kinematics(q)
    poses = []
    for i, name in enumerate(names):
        t = transforms[name]
        poses.append((t[:3, :3] @ model.OBB_LOCAL_ROTATIONS[i],
                      t[:3, :3] @ model.OBB_LOCAL_CENTERS_M[i] + t[:3, 3]))
    rows = []
    for i, j in pairs:
        ra, ca = poses[i]
        rb, cb = poses[j]
        ha, hb = model.OBB_HALF_EXTENTS_M[[i, j]]
        axes = list(ra.T) + list(rb.T) + [np.cross(a, b) for a in ra.T for b in rb.T]
        gaps = []
        for axis in axes:
            length = np.linalg.norm(axis)
            if length <= 1e-12:
                continue
            axis = axis / length
            gaps.append(float(abs(axis @ (cb-ca)) - abs(axis @ ra) @ ha - abs(axis @ rb) @ hb))
        gap = max(gaps)
        # 1e-9 is an exploratory numerical band, not an admission threshold.
        state = "separated" if gap > 1e-9 else "overlap" if gap < -1e-9 else "boundary_unknown"
        a = fcl.CollisionObject(fcl.Box(*(2*ha)), fcl.Transform(ra, ca))
        b = fcl.CollisionObject(fcl.Box(*(2*hb)), fcl.Transform(rb, cb))
        result = fcl.CollisionResult()
        fcl.collide(a, b, fcl.CollisionRequest(), result)
        rows.append({"pair": [names[i], names[j]], "online": (i, j) in online,
                     "sat_state": state, "max_axis_gap_m": gap,
                     "fcl_box_collision": bool(result.is_collision)})
    return rows


seeds = [np.zeros(9)] + [np.array([j1, j2, .8, j4, 0, 0, 0, 0, 0])
    for j1, j2, j4 in [(0.30, 0, .9), (.30, -.9, .9), (.47, 0, .9),
                       (.47, -.9, .9), (.58, 0, 1.3), (.58, -.6, 1.3)]]
samples = [{"label": "zero" if k == 0 else f"natural_seed_{k}",
            "q": q.tolist(), "pairs": evaluate(q)} for k, q in enumerate(seeds)]
files = ["portable_oscbf/work/obb_collision_model.py", "portable_oscbf/work/kinematics_data.py",
         "portable_oscbf/work/nineaxis_kinematics.py", "src/robot_safecontrol_moveit/task_target.py"]
print(json.dumps({"evidence": "point checks only; seeds are NOT solved IK states",
    "numpy": np.__version__, "python_fcl": getattr(fcl, "__version__", "unknown"),
    "sha256": {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in files},
    "samples": samples}, indent=2))
