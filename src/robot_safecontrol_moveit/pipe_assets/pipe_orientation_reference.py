import numpy as np
from scipy.integrate import solve_ivp

from .pipe_axis_smoothing import _normalize_axis, pipe_axis_smoothing
from .pipe_pose_math import (
    _parameters,
    _polynomial_data,
    _polynomial_jet,
    pipe_pose_math,
)


def _normalize_orientation(reference):
    reference = dict(reference)
    reference["breaks_mm"], reference["bezier"] = _polynomial_data(reference["breaks_mm"], reference["bezier"])
    reference["axis"] = _normalize_axis(reference["axis"])
    for name in ("projection_lower_bound", "rotation_travel_bound_rad"):
        if name in reference:
            reference[name] = np.asarray(reference[name]).reshape(-1)
    for name, columns in (("node_x", 3), ("node_dx", 3), ("node_d2x", 3), ("quaternion_reference", 4)):
        if name in reference:
            reference[name] = np.asarray(reference[name]).reshape(-1, columns)
    return reference


def _evaluate(reference, arc, side="right"):
    arc = _parameters(arc)
    axis = pipe_axis_smoothing("eval", reference["axis"], arc, side)
    a, ad, add = (axis[key] for key in ("tool_axis_ref_W", "daxis_darc", "d2axis_darc2"))
    h, hd, hdd, _ = _polynomial_jet(reference["breaks_mm"], reference["bezier"], arc, side)
    d0 = np.einsum("ij,ij->i", a, h)
    d1 = np.einsum("ij,ij->i", ad, h) + np.einsum("ij,ij->i", a, hd)
    d2 = np.einsum("ij,ij->i", add, h) + 2 * np.einsum("ij,ij->i", ad, hd) + np.einsum("ij,ij->i", a, hdd)
    q = h - a * d0[:, None]
    qd = hd - ad * d0[:, None] - a * d1[:, None]
    qdd = hdd - add * d0[:, None] - 2 * ad * d1[:, None] - a * d2[:, None]
    x, xd, xdd, radius = pipe_pose_math("normalize", q, qd, qdd)
    y = np.cross(a, x)
    yd = np.cross(ad, x) + np.cross(a, xd)
    ydd = np.cross(add, x) + 2 * np.cross(ad, xd) + np.cross(a, xdd)
    matrix = np.stack((x, y, a), axis=2)
    derivative = np.stack((xd, yd, ad), axis=2)
    second = np.stack((xdd, ydd, add), axis=2)

    def vector(m):
        m = (m - m.transpose(0, 2, 1)) / 2
        return np.column_stack((m[:, 2, 1], m[:, 0, 2], m[:, 1, 0]))

    beta = vector(derivative @ matrix.transpose(0, 2, 1))
    betad = vector(second @ matrix.transpose(0, 2, 1) + derivative @ derivative.transpose(0, 2, 1))
    quaternion = pipe_pose_math("quat", matrix.transpose(1, 2, 0))
    if np.size(reference["quaternion_reference"]):
        index = np.clip(np.searchsorted(reference["breaks_mm"], arc, side=side) - 1, 0, len(reference["breaks_mm"]) - 2)
        baseline = np.asarray(reference["quaternion_reference"])[index]
        quaternion[np.einsum("ij,ij->i", quaternion, baseline) < 0] *= -1
    return {**axis, "R_WT": matrix.transpose(1, 2, 0), "dR_darc": derivative.transpose(1, 2, 0), "d2R_darc2": second.transpose(1, 2, 0), "quat_wxyz": quaternion, "beta_W": beta, "dbeta_darc_W": betad, "projection_norm": radius, "minimal_twist_residual_per_mm": np.linalg.norm(xd + a * np.einsum("ij,ij->i", ad, x)[:, None], axis=1)}


def _build(axis, tangent, cfg):
    length = axis["length_mm"]
    grid = np.linspace(0, length, max(1, int(np.ceil(length / cfg["frame_step_mm"]))) + 1)
    axis_breaks = np.asarray(axis["breaks_mm"])
    grid = grid[np.min(np.abs(grid[:, None] - axis_breaks), axis=1) > 1e-11 * max(1, length)]
    breaks = np.unique(np.r_[grid, axis_breaks])
    a = pipe_axis_smoothing("eval", axis, [0])["tool_axis_ref_W"][0]
    initial = np.asarray(cfg["initial_x_W"] if np.size(cfg["initial_x_W"]) else tangent).reshape(3)
    initial = initial - np.dot(initial, a) * a
    # 默认滚转基准来自切向；显式退化输入直接报告。
    if np.linalg.norm(initial) < 1e-10:
        raise ValueError("PipePose:InitialAxis initial_x_W 或起始切向与工具轴平行")
    initial /= np.linalg.norm(initial)

    def rhs(arc, x):
        value = pipe_axis_smoothing("eval", axis, [arc])
        return -value["tool_axis_ref_W"][0] * np.dot(value["daxis_darc"][0], x)

    for iteration in range(int(cfg["max_frame_refinements"]) + 1):
        solution = solve_ivp(rhs, (0, length), initial, rtol=cfg["ode_rel_tol"], atol=cfg["ode_abs_tol"], max_step=np.min(np.diff(breaks)), dense_output=True)
        if not solution.success:
            raise RuntimeError(f"PipePose:FrameIntegration {solution.message}")
        values = solution.sol(breaks).T
        sample = pipe_axis_smoothing("eval", axis, breaks)
        a, ad, add = (sample[key] for key in ("tool_axis_ref_W", "daxis_darc", "d2axis_darc2"))
        values -= a * np.einsum("ij,ij->i", a, values)[:, None]
        values /= np.linalg.norm(values, axis=1)[:, None]
        derivative = -a * np.einsum("ij,ij->i", ad, values)[:, None]
        second = -ad * np.einsum("ij,ij->i", ad, values)[:, None] - a * (np.einsum("ij,ij->i", add, values) + np.einsum("ij,ij->i", ad, derivative))[:, None]
        controls, lower, travel = [], [], []
        for index, width in enumerate(np.diff(breaks)):
            h = pipe_pose_math("hermite", values[index], derivative[index], second[index], values[index + 1], derivative[index + 1], second[index + 1], width)
            z = pipe_axis_smoothing("controls", axis, breaks[index], breaks[index + 1])
            cross = pipe_pose_math("cross_controls", z, h)
            middle = pipe_pose_math("beval", cross, [0.5])[0][0]
            if np.linalg.norm(middle) <= np.finfo(float).tiny:
                raise RuntimeError("PipePose:FrameCertificate 投影引导方向退化")
            direction = middle / np.linalg.norm(middle)
            upper_z = np.max(np.linalg.norm(z, axis=1))
            guide = np.sum(z, axis=0)
            if np.linalg.norm(guide) < 1e-12:
                raise RuntimeError("PipePose:FrameCertificate axis 引导方向退化")
            min_z = max(axis["rho_z"], np.min(z @ (guide / np.linalg.norm(guide))))
            bound = np.min(cross @ direction) / upper_z - cfg["constraint_guard"]
            axis_rate = np.max(np.linalg.norm(5 * np.diff(z, axis=0) / width, axis=1)) / min_z
            hmax = np.max(np.linalg.norm(h, axis=1))
            hprime = np.max(np.linalg.norm(5 * np.diff(h, axis=0) / width, axis=1))
            xrate = (hprime + 2 * axis_rate * hmax) / max(bound, np.finfo(float).tiny)
            controls.append(h)
            lower.append(bound)
            travel.append(width * (axis_rate + xrate))
        lower, travel = np.asarray(lower), np.asarray(travel)
        bad = (lower < cfg["rho_projection"]) | (travel >= np.pi / 2)
        if not np.any(bad):
            break
        if iteration == cfg["max_frame_refinements"]:
            raise RuntimeError("PipePose:FrameCertificate 投影与 quaternion 区间检查未通过")
        breaks = np.unique(np.r_[breaks, (breaks[:-1][bad] + breaks[1:][bad]) / 2])
    reference = {"format_version": 1, "axis": axis, "breaks_mm": breaks, "bezier": np.stack(controls, axis=2), "node_x": values, "node_dx": derivative, "node_d2x": second, "initial_x_W": initial, "initial_fallback_used": False, "projection_lower_bound": lower, "rotation_travel_bound_rad": travel, "refinements": iteration, "projection_certificate": "degree-10 Bernstein cross(z,h) halfspace / convex-hull upper norm(z)", "quaternion_certificate": "integral |beta| <= interval_length*(axis_derivative_bound + x_derivative_bound) < pi/2", "config": cfg, "quaternion_reference": np.empty((0, 4))}
    quaternion = _evaluate(reference, breaks)["quat_wxyz"]
    for index in range(1, len(quaternion)):
        if np.dot(quaternion[index], quaternion[index - 1]) < 0:
            quaternion[index] *= -1
    reference["quaternion_reference"] = quaternion
    return reference


def pipe_orientation_reference(action: str, *arguments):
    if action == "build":
        return _build(_normalize_axis(arguments[0]), *arguments[1:])
    if action == "eval":
        return _evaluate(_normalize_orientation(arguments[0]), *arguments[1:])
    raise ValueError(f"PipePose:OrientationAction 未知动作：{action}")
