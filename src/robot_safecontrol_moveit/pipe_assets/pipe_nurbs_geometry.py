from functools import lru_cache
from pathlib import Path

import numpy as np
from numpy.polynomial.legendre import leggauss
from scipy.interpolate import BSpline


def _spline(geo):
    control = np.asarray(geo["ctrl_pts"])
    knots = np.asarray(geo["knots"])
    if np.iscomplexobj(control):
        raise ValueError("pipe_nurbs_geometry:ControlPoints：控制点需要实数")
    if np.iscomplexobj(knots):
        raise ValueError("pipe_nurbs_geometry:KnotLayout：节点需要实数")
    control, knots = control.astype(float), knots.astype(float).ravel()
    if control.ndim != 2 or control.shape[1] != 3 or len(control) < 4 or not np.isfinite(control).all():
        raise ValueError("pipe_nurbs_geometry:ControlPoints：需要至少四个有限三维控制点")
    if (geo["p"] != 3 or len(knots) != len(control) + 4 or not np.isfinite(knots).all()
            or np.any(np.diff(knots) < 0) or np.count_nonzero(knots == 0) != 4
            or np.count_nonzero(knots == 1) != 4 or np.any(knots[:4] != 0)
            or np.any(knots[-4:] != 1)):
        raise ValueError("pipe_nurbs_geometry:KnotLayout：需要 [0,1] 三次夹持节点")
    interior = knots[(knots > 0) & (knots < 1)]
    if np.any(np.diff(interior) <= 0):
        raise ValueError("pipe_nurbs_geometry:DiscontinuousCurve：内部节点必须保持 C2 连续")
    if "w" in geo:
        weights = np.asarray(geo["w"]).ravel()
        if len(weights) != len(control) or not np.all(weights == 1):
            raise ValueError("pipe_nurbs_geometry:Weights：源接口需要单位权重")
    return BSpline(knots, control, 3, extrapolate=False)


def _parameters(u):
    values = np.asarray(u)
    if np.iscomplexobj(values):
        raise ValueError("pipe_nurbs_geometry:Parameter：曲线参数需要实数")
    values = values.astype(float).ravel()
    tolerance = 8 * np.finfo(float).eps
    if not np.isfinite(values).all() or np.any(values < -tolerance) or np.any(values > 1 + tolerance):
        raise ValueError("pipe_nurbs_geometry:Parameter：曲线参数必须位于 [0,1]")
    return np.clip(values, 0, 1)


@lru_cache(maxsize=2)
def _gauss_rule(order):
    return leggauss(order)


def _integrate(spline, low, high, order=16):
    low, high = np.broadcast_arrays(np.asarray(low, dtype=float), np.asarray(high, dtype=float))
    nodes, weights = _gauss_rule(order)
    half = (high - low) / 2
    u = (low + high)[..., None] / 2 + half[..., None] * nodes
    speed = np.linalg.norm(spline(u, nu=1), axis=-1)
    return half * (speed @ weights)


def _arc(geo, block_u):
    block_u = _parameters(block_u)
    if len(block_u) < 2 or block_u[0] != 0 or block_u[-1] != 1 or np.any(np.diff(block_u) <= 0):
        raise ValueError("pipe_nurbs_geometry:Blocks：分块参数需要从 0 严格递增到 1")
    spline = _spline(geo)
    breaks = np.unique(np.r_[spline.t, block_u])
    for depth in range(19):
        low, high = breaks[:-1], breaks[1:]
        value = _integrate(spline, low, high, 16)
        error = np.abs(value - _integrate(spline, low, high, 8))
        refine = error > np.maximum(5e-13, 2e-10 * (high - low))
        if not np.any(refine):
            break
        if depth == 18:
            raise RuntimeError("pipe_nurbs_geometry:ArcIntegration：弧长积分达到分段上限")
        breaks = np.unique(np.r_[breaks, (low[refine] + high[refine]) / 2])
    if not np.isfinite(value).all() or np.any(value <= 0):
        raise ValueError("pipe_nurbs_geometry:DegenerateCurve：曲线包含零长度或无效区间")
    distance = np.r_[0., np.cumsum(value)]
    return {"u": breaks, "s": distance, "block_u": block_u,
            "block_s": distance[np.searchsorted(breaks, block_u)], "total_length": distance[-1],
            "integration_error_estimate": float(np.sum(error)), "inversion_tolerance_mm": 1e-10}


def _invert(geo, arc, target):
    target = np.asarray(target)
    if np.iscomplexobj(target):
        raise ValueError("pipe_nurbs_geometry:ArcTarget：目标弧长需要实数")
    target = target.astype(float).ravel()
    grid_u, grid_s = np.asarray(arc["u"]), np.asarray(arc["s"])
    if not np.isfinite(target).all() or np.any(target < -1e-9) or np.any(target > grid_s[-1] + 1e-9):
        raise ValueError("pipe_nurbs_geometry:ArcTarget：目标弧长超出曲线范围")
    target = np.clip(target, 0, grid_s[-1])
    index = np.clip(np.searchsorted(grid_s, target, side="right") - 1, 0, len(grid_s) - 2)
    base_u, base_s = grid_u[index], grid_s[index]
    left, right = base_u.copy(), grid_u[index + 1].copy()
    u = left + (right - left) * (target - base_s) / (grid_s[index + 1] - base_s)
    exact_left, exact_right = target == base_s, target == grid_s[index + 1]
    u[exact_left], u[exact_right] = left[exact_left], right[exact_right]
    active = ~(exact_left | exact_right)
    residual = np.zeros_like(u)
    spline = _spline(geo)
    for _ in range(24):
        rows = np.flatnonzero(active)
        if not len(rows):
            break
        residual[rows] = base_s[rows] + _integrate(spline, base_u[rows], u[rows]) - target[rows]
        done = np.abs(residual[rows]) <= arc["inversion_tolerance_mm"]
        active[rows[done]] = False
        rows = rows[~done]
        if not len(rows):
            continue
        above = residual[rows] > 0
        right[rows[above]] = u[rows[above]]
        left[rows[~above]] = u[rows[~above]]
        speed = np.linalg.norm(spline(u[rows], nu=1), axis=1)
        if np.any(speed <= 0):
            raise ValueError("pipe_nurbs_geometry:DegenerateCurve：反解遇到曲线驻点")
        candidate = u[rows] - residual[rows] / speed
        invalid = ~np.isfinite(candidate) | (candidate <= left[rows]) | (candidate >= right[rows])
        candidate[invalid] = (left[rows[invalid]] + right[rows[invalid]]) / 2
        u[rows] = candidate
    rows = np.flatnonzero(active)
    residual[rows] = base_s[rows] + _integrate(spline, base_u[rows], u[rows]) - target[rows]
    if np.any(np.abs(residual) > 5e-9):
        raise RuntimeError(f"pipe_nurbs_geometry:ArcInversion：最大残差 {np.max(np.abs(residual))} mm")
    return u, residual


def pipe_nurbs_geometry(action, *args):
    action = str(action).lower()
    if action == "load":
        folder = Path(args[0])
        control_file, knot_file = folder / "control_points.txt", folder / "knot_vector.txt"
        geo = {"control_file": str(control_file), "knot_file": str(knot_file),
               "ctrl_pts": np.loadtxt(control_file, ndmin=2), "knots": np.loadtxt(knot_file).ravel(), "p": 3}
        geo["w"] = np.ones(len(geo["ctrl_pts"]))
        _spline(geo)
        return geo
    if action == "eval":
        spline = _spline(args[0])
        u = _parameters(args[1])
        return tuple(spline(u, nu=order) for order in range(4))
    if action == "arc":
        return _arc(*args)
    if action == "invert":
        return _invert(*args)
    raise ValueError(f"pipe_nurbs_geometry:Action：未知操作 {action}")
