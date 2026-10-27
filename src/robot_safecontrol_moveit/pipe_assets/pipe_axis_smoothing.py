import numpy as np
import osqp
from scipy import sparse

from .pipe_pose_math import (
    _parameters,
    _polynomial_data,
    _polynomial_jet,
    pipe_pose_math,
)


def _normalize_axis(axis):
    axis = dict(axis)
    axis["breaks_mm"], axis["bezier"] = _polynomial_data(axis["breaks_mm"], axis["bezier"])
    for name in ("norm_lower_bound_by_interval", "sample_arc_mm", "sample_weights", "arc_quadrature_weights"):
        if name in axis:
            axis[name] = np.asarray(axis[name]).reshape(-1)
    for name, columns in (("guides", 3), ("raw_axes", 3), ("smoothing_regions_mm", 2)):
        if name in axis:
            axis[name] = np.asarray(axis[name]).reshape(-1, columns)
    return axis


def _evaluate(axis, arc, side="right"):
    arc = _parameters(arc)
    tolerance = 64 * np.spacing(max(1, axis["length_mm"]))
    if np.any((arc < -tolerance) | (arc > axis["length_mm"] + tolerance)):
        raise ValueError("PipePose:Arc 弧长超出范围")
    arc = np.clip(arc, 0, axis["length_mm"])
    z, zd, zdd, _ = _polynomial_jet(axis["breaks_mm"], axis["bezier"], arc, side)
    a, ad, add, radius = pipe_pose_math("normalize", z, zd, zdd)
    return {"tool_axis_ref_W": a, "daxis_darc": ad, "d2axis_darc2": add, "z": z, "dz_darc": zd, "d2z_darc2": zdd, "z_norm": radius}


def _merge_regions(regions, length):
    regions = np.asarray(regions, dtype=float).reshape(-1, 2)
    if not len(regions):
        return regions
    regions = np.clip(regions, 0, length)
    regions = regions[np.argsort(regions[:, 0])]
    result = [regions[0].copy()]
    for region in regions[1:]:
        if region[0] <= result[-1][1]:
            result[-1][1] = max(result[-1][1], region[1])
        else:
            result.append(region.copy())
    return np.asarray([region for region in result if region[1] > region[0]]).reshape(-1, 2)


def _build(arc, raw, cfg, event_arc=()):
    arc = _parameters(arc)
    raw = np.asarray(raw, dtype=float)
    if len(arc) < 2 or arc[0] != 0 or np.any(np.diff(arc) <= 0) or raw.shape != (len(arc), 3) or np.any(~np.isfinite(raw)) or np.any(np.abs(np.linalg.norm(raw, axis=1) - 1) > cfg["normal_tolerance"]):
        raise ValueError("PipePose:AxisData 需要递增弧长与单位方向 N×3")
    length = arc[-1]
    turn = np.arctan2(np.linalg.norm(np.cross(raw[:-1], raw[1:]), axis=1), np.einsum("ij,ij->i", raw[:-1], raw[1:]))
    if np.any(turn > np.deg2rad(175)):
        raise ValueError("PipePose:DirectionBranch 近反向方向需要明确的中间方向")
    centers = np.r_[_parameters(event_arc), (arc[:-1] + arc[1:])[turn / np.diff(arc) > cfg["direction_rate_threshold_rad_per_mm"]] / 2]
    added = np.column_stack((centers - cfg["region_halfwidth_mm"], centers + cfg["region_halfwidth_mm"]))
    regions = _merge_regions(np.vstack((np.asarray(cfg["smoothing_regions_mm"]).reshape(-1, 2), added)), length)
    weight = np.full(len(arc), cfg["normal_weight"], dtype=float)
    for low, high in regions:
        weight[(arc >= low) & (arc <= high)] = cfg["region_weight"]
    if np.size(cfg["axis_breaks_mm"]):
        breaks = _parameters(cfg["axis_breaks_mm"])
        if breaks[0] != 0 or breaks[-1] != length or np.any(np.diff(breaks) <= 0):
            raise ValueError("PipePose:AxisKnots 节点必须从零递增到总弧长")
    else:
        breaks = np.linspace(0, length, max(1, int(np.ceil(length / cfg["axis_knot_spacing_mm"]))) + 1)
    knots = np.r_[np.zeros(6), breaks[1:-1] / length, np.ones(6)]
    basis = pipe_pose_math("basis", knots, 5, arc / length)[0]
    count = basis.shape[1]
    delta = np.r_[np.diff(arc) / 2, 0] + np.r_[0, np.diff(arc) / 2]
    x, w = pipe_pose_math("gauss", 6)
    width = np.diff(breaks)
    queries = breaks[:-1, None] + width[:, None] * (x + 1) / 2
    quadrature = (width[:, None] * w / 2).ravel()
    _, _, d2, d3 = pipe_pose_math("basis", knots, 5, queries.ravel() / length)
    d2, d3 = d2 / length**2, d3 / length**3
    hessian = basis.T @ ((weight * delta)[:, None] * basis) + cfg["lambda2_mm4"] * d2.T @ (quadrature[:, None] * d2) + cfg["lambda3_mm6"] * d3.T @ (quadrature[:, None] * d3)
    hessian = (hessian + hessian.T) / 2
    linear = -2 * basis.T @ ((weight * delta)[:, None] * raw)
    constraints = np.zeros((6 * len(width), 3 * count))
    extractions, guides = [], []
    for index, (low, high) in enumerate(zip(breaks[:-1], breaks[1:])):
        extraction = pipe_pose_math("extract", knots, 5, low / length, high / length)
        guide = np.array([np.interp((low + high) / 2, arc, raw[:, coordinate]) for coordinate in range(3)])
        if np.linalg.norm(guide) < 0.05:
            raise ValueError("PipePose:DirectionBranch 区间方向无法确定")
        guide /= np.linalg.norm(guide)
        for coordinate in range(3):
            constraints[6 * index:6 * (index + 1), coordinate * count:(coordinate + 1) * count] = -guide[coordinate] * extraction
        extractions.append(extraction)
        guides.append(guide)
    upper = np.full(len(constraints), -(cfg["rho_z"] + cfg["constraint_guard"]))
    matrix = 2 * sparse.kron(sparse.eye(3), sparse.csc_matrix(hessian), format="csc")
    constraint_matrix = sparse.csc_matrix(constraints)
    q = linear.ravel(order="F")
    solver = osqp.OSQP()
    solver.setup(P=sparse.triu(matrix, format="csc"), q=q, A=constraint_matrix, l=np.full(len(upper), -np.inf), u=upper, eps_abs=1e-11, eps_rel=1e-11, max_iter=10000, polishing=True, verbose=False)
    solved = solver.solve(raise_error=True)
    if solved.info.status != "solved" or solved.x is None or np.any(~np.isfinite(solved.x)) or np.any(~np.isfinite(solved.y)):
        raise RuntimeError(f"PipePose:AxisQP 求解失败：{solved.info.status}")
    slack = upper - constraints @ solved.x
    residuals = {"stationarity": float(np.max(np.abs(matrix @ solved.x + q + constraint_matrix.T @ solved.y))), "primal": float(max(0, -np.min(slack))), "complementarity": float(np.max(np.abs(slack * solved.y))), "dual_nonnegative": float(max(0, -np.min(solved.y)))}
    if max(residuals.values()) > 1e-7 or np.max(-slack) > cfg["constraint_guard"] / 2:
        raise RuntimeError(f"PipePose:AxisQP KKT 检查失败：{residuals}")
    controls = solved.x.reshape(count, 3, order="F")
    bezier = np.stack([item @ controls for item in extractions], axis=2)
    guides = np.asarray(guides)
    lower = np.min(np.einsum("ijk,kj->ik", bezier, guides), axis=0) - cfg["constraint_guard"] / 4
    if np.min(lower) < cfg["rho_z"]:
        raise RuntimeError("PipePose:AxisCertificate 区间下界未通过")
    axis = {"format_version": 1, "degree": 5, "order": 6, "knots": knots, "controls": controls, "breaks_mm": breaks, "bezier": bezier, "length_mm": length, "guides": guides, "norm_lower_bound_by_interval": lower, "rho_z": cfg["rho_z"], "certificate": "min_j e_k dot Z_kj minus rounding guard; convex hull on entire span", "smoothing_regions_mm": regions, "sample_arc_mm": arc, "raw_axes": raw, "sample_weights": weight, "arc_quadrature_weights": delta, "config": cfg, "solver": {"exitflag": 1, "objective": float(solved.info.obj_val), "output": {"solver": "OSQP", "status": solved.info.status, "iterations": solved.info.iter, "kkt_residuals": residuals}}, "regularization_units": "lambda2: mm^4; lambda3: mm^6; derivatives and integrals in mm"}
    a = _evaluate(axis, arc)["tool_axis_ref_W"]
    error = np.arctan2(np.linalg.norm(np.cross(a, raw), axis=1), np.einsum("ij,ij->i", a, raw))
    index = np.argmax(error)
    axis["deviation"] = {"max_rad": error[index], "max_arc_mm": arc[index], "arc_weighted_rms_rad": np.sqrt(np.dot(delta, error**2) / np.sum(delta)), "sample_error_rad": error, "region_max_rad": np.asarray([np.max(error[(arc >= low) & (arc <= high)]) for low, high in regions])}
    return axis


def pipe_axis_smoothing(action: str, *arguments):
    if action in ("eval", "controls"):
        arguments = (_normalize_axis(arguments[0]), *arguments[1:])
    if action == "build":
        return _build(*arguments)
    if action == "eval":
        return _evaluate(*arguments)
    if action == "controls":
        axis, low, high = arguments
        breaks = np.asarray(axis["breaks_mm"])
        index = min(np.searchsorted(breaks, (low + high) / 2, side="right") - 1, len(breaks) - 2)
        if low < breaks[index] - 1e-9 or high > breaks[index + 1] + 1e-9 or high <= low:
            raise ValueError("PipePose:CertificateSpan 区间跨越 axis 节点")
        query = np.linspace(0, 1, 6)
        value = _polynomial_jet(breaks, axis["bezier"], low + (high - low) * query)[0]
        return np.linalg.solve(pipe_pose_math("bernstein", 5, query), value)
    raise ValueError(f"PipePose:AxisAction 未知动作：{action}")
