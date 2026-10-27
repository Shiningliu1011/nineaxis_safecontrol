import numpy as np


def _evaluate(u, v, cone=False):
    u, v = np.asarray(u, dtype=float).reshape(-1), np.asarray(v, dtype=float).reshape(-1)
    if u.shape != v.shape or np.any(~np.isfinite(u)) or np.any(~np.isfinite(v)):
        raise ValueError("PipePose:Model u/v 需要等长有限向量")
    dr = 0.6 if cone else 0
    radius = 30 + dr * u
    c, s = np.cos(v), np.sin(v)
    point = np.column_stack((radius * c, radius * s, (80 if cone else 0) + u))
    normal = np.column_stack((c, s, np.full_like(u, -dr))) / np.sqrt(1 + dr**2)
    pu = np.column_stack((dr * c, dr * s, np.ones_like(u)))
    pv = np.column_stack((-radius * s, radius * c, np.zeros_like(u)))
    return point, normal, pu, pv


def pipe_pose_test_model() -> dict:
    return {"name": "V3 verified crease fixture", "lengthUnit": "mm", "coordinateFrame": "W", "segments": [
        {"name": "straight", "canvasLength": 80, "uRange": np.array([0, 80]), "thetaOffsetRad": 0, "thetaSign": 1, "evaluate": _evaluate},
        {"name": "cone", "canvasLength": 80, "uRange": np.array([0, 80]), "thetaOffsetRad": 0, "thetaSign": 1, "evaluate": lambda u, v: _evaluate(u, v, True)},
    ]}
