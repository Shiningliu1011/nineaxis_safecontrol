from itertools import pairwise

import numpy as np


def _finite_vector(value, name, *, allow_empty=False):
    array = np.asarray(value)
    if array.dtype.kind not in "iuf" or array.ndim > 2:
        raise ValueError(f"{name} 必须是有限实数向量")
    if array.ndim == 2 and min(array.shape) != 1 and array.size:
        raise ValueError(f"{name} 必须是有限实数向量")
    array = array.astype(float).reshape(-1)
    if not np.all(np.isfinite(array)) or (not allow_empty and array.size == 0):
        raise ValueError(f"{name} 必须是有限实数向量")
    return array


def _finite_scalar(value, name, *, positive=False, nonnegative=False):
    array = np.asarray(value)
    if array.dtype.kind not in "iuf" or array.size != 1:
        raise ValueError(f"{name} 必须是有限实数标量")
    result = float(array.reshape(-1)[0])
    if not np.isfinite(result) or (positive and result <= 0) or (nonnegative and result < 0):
        raise ValueError(f"{name} 的数值范围无效")
    return result


def _paired_parameters(u, v):
    u = _finite_vector(u, "u", allow_empty=True)
    v = _finite_vector(v, "v", allow_empty=True)
    if len(u) != len(v):
        raise ValueError("u 和 v 必须等长")
    return u, v


def _evaluate_segment(u, v, spec):
    u, v = _paired_parameters(u, v)
    tolerance = 1e-10 * max(1, spec["limit"])
    if np.any(u < -tolerance) or np.any(u > spec["limit"] + tolerance):
        raise ValueError("u 超出本管段的 uRange")
    u = np.clip(u, 0, spec["limit"])
    c, d = np.cos(v), np.sin(v)
    z = np.zeros_like(u)
    if spec["type"] == "bend":
        cu, su = np.cos(u), np.sin(u)
        q = spec["Rb"] - spec["r0"] * c
        p = np.column_stack((spec["Rb"] * (1 - cu) + spec["r0"] * c * cu,
                             spec["r0"] * d, q * su))
        n = np.column_stack((c * cu, d, -c * su))
        pu = np.column_stack((q * su, z, q * cu))
        pv = np.column_stack((-spec["r0"] * d * cu, spec["r0"] * c,
                              spec["r0"] * d * su))
    else:
        x = u / spec["limit"]
        h = 10 * x**3 - 15 * x**4 + 6 * x**5
        dh = 30 * x**2 * (1 - x)**2 / spec["limit"]
        r = spec["r0"] + (spec["r1"] - spec["r0"]) * h
        dr = (spec["r1"] - spec["r0"]) * dh
        p = np.column_stack((r * c, r * d, u))
        n = np.column_stack((c, d, -dr)) / np.sqrt(1 + dr**2)[:, None]
        pu = np.column_stack((dr * c, dr * d, np.ones_like(u)))
        pv = np.column_stack((-r * d, r * c, z))
    rotation, translation = spec["R"], spec["t"]
    return (p @ rotation.T + translation, n @ rotation.T,
            pu @ rotation.T, pv @ rotation.T)


def combined_pipe_model():
    cfg = {
        "straight1Length": 100.0, "reducer1Length": 70.0,
        "bendRadius": 90.0, "bendAngle": np.pi / 2,
        "straight2Length": 100.0, "reducer2Length": 80.0,
        "inletRadius": 20.0, "middleRadius": 30.0, "outletRadius": 18.0,
    }
    for key, value in cfg.items():
        _finite_scalar(value, key, positive=True)
    if cfg["bendRadius"] <= cfg["middleRadius"] or cfg["bendAngle"] > np.pi:
        raise ValueError("弯管半径或 bendAngle 无效")
    names = ["直管 1", "变径管 1", "弯管", "直管 2", "变径管 2"]
    kinds = ["straight", "reducer", "bend", "straight", "reducer"]
    limits = [cfg["straight1Length"], cfg["reducer1Length"], cfg["bendAngle"],
              cfg["straight2Length"], cfg["reducer2Length"]]
    r0 = [cfg["inletRadius"], cfg["inletRadius"], cfg["middleRadius"],
          cfg["middleRadius"], cfg["middleRadius"]]
    r1 = [cfg["inletRadius"], cfg["middleRadius"], cfg["middleRadius"],
          cfg["middleRadius"], cfg["outletRadius"]]
    segments = []
    rotation, translation = np.eye(3), np.zeros(3)
    for index, kind in enumerate(kinds):
        spec = {"type": kind, "limit": limits[index], "r0": r0[index],
                "r1": r1[index], "Rb": cfg["bendRadius"],
                "R": rotation.copy(), "t": translation.copy()}
        transform = np.eye(4)
        transform[:3, :3], transform[:3, 3] = rotation, translation
        segment = {
            "name": names[index], "type": kind, "canvasLength": limits[index],
            "uRange": np.array([0.0, limits[index]]), "thetaOffsetRad": 0.0,
            "thetaSign": 1, "radiusStart": r0[index], "radiusEnd": r1[index],
            "transform": transform,
            "evaluate": lambda u, v, spec=spec: _evaluate_segment(u, v, spec),
        }
        segments.append(segment)
        if kind == "bend":
            beta = limits[index]
            segment["canvasLength"] = cfg["bendRadius"] * beta
            translation = translation + rotation @ np.array([
                cfg["bendRadius"] * (1 - np.cos(beta)), 0,
                cfg["bendRadius"] * np.sin(beta)])
            rotation = rotation @ np.array([
                [np.cos(beta), 0, np.sin(beta)], [0, 1, 0],
                [-np.sin(beta), 0, np.cos(beta)]])
        else:
            translation = translation + rotation @ np.array([0, 0, limits[index]])
    breaks = np.concatenate(([0.0], np.cumsum([s["canvasLength"] for s in segments])))
    v = np.linspace(0, 2 * np.pi, 361)
    report = np.empty((4, 2))
    for index, (a, b) in enumerate(pairwise(segments)):
        pa, na, _, _ = a["evaluate"](np.full_like(v, a["uRange"][1]), v)
        pb, nb, _, _ = b["evaluate"](np.full_like(v, b["uRange"][0]), v)
        report[index] = [np.max(np.linalg.norm(pa - pb, axis=1)),
                         np.max(np.linalg.norm(na - nb, axis=1))]
    if np.any(report[:, 0] > 1e-3) or np.any(report[:, 1] > 1e-4):
        raise ValueError("接缝位置或外法向不连续")
    return {
        "name": "管状组合：直管—变径管—弯管—直管—变径管", "lengthUnit": "mm",
        "coordinateFrame": "工件坐标系 W", "parameters": cfg, "segments": segments,
        "breaks": breaks, "totalLength": breaks[-1], "outletCenter": translation,
        "joinReport": report,
    }
