from math import comb

import numpy as np
from scipy.interpolate import BPoly, BSpline
from scipy.spatial.transform import Rotation


def _parameters(value):
    if not np.isrealobj(value):
        raise ValueError("PipePose:Progress 参数必须为有限实数")
    result = np.asarray(value, dtype=float).reshape(-1)
    if np.any(~np.isfinite(result)):
        raise ValueError("PipePose:Progress 参数必须为有限实数")
    return result


def _basis(knots, degree, parameters):
    knots = _parameters(knots)
    degree = int(degree)
    parameters = _parameters(parameters)
    count = len(knots) - degree - 1
    if degree < 0 or count <= degree or np.any(np.diff(knots) < 0):
        raise ValueError("PipePose:Basis 节点或 degree 无效")
    if np.any((parameters < knots[degree]) | (parameters > knots[-degree - 1])):
        raise ValueError("PipePose:Basis 参数超出样条范围")
    spline = BSpline(knots, np.eye(count), degree, extrapolate=False)
    return tuple(spline(parameters, nu=order) if order <= degree else np.zeros((len(parameters), count)) for order in range(4))


def _bernstein(degree, parameters):
    degree = int(degree)
    parameters = _parameters(parameters)
    if degree < 0:
        raise ValueError("PipePose:Bernstein degree 必须非负")
    return BPoly(np.eye(degree + 1)[:, None, :], [0, 1])(parameters)


def _beval(controls, parameters, width=1):
    controls = np.asarray(controls, dtype=float)
    width = float(width)
    if controls.ndim != 2 or not np.all(np.isfinite(controls)) or not np.isfinite(width) or width <= 0:
        raise ValueError("PipePose:Polynomial controls 或 width 无效")
    curve = BPoly(controls[:, None, :], [0, width])
    parameters = _parameters(parameters) * width
    return tuple(curve(parameters, nu=order) for order in range(3))


def _normalize(z, zd, zdd):
    z, zd, zdd = (np.asarray(value, dtype=float) for value in (z, zd, zdd))
    if z.ndim != 2 or z.shape[1] != 3 or z.shape != zd.shape or z.shape != zdd.shape:
        raise ValueError("PipePose:Degenerate 归一化输入必须为同形状 N×3")
    radius = np.linalg.norm(z, axis=1)
    if np.any(~np.isfinite(radius) | (radius < 1e-12)):
        raise ValueError("PipePose:Degenerate 无法归一化退化向量")
    axis = z / radius[:, None]
    rd = np.einsum("ij,ij->i", axis, zd)
    ad = (zd - axis * rd[:, None]) / radius[:, None]
    rdd = np.einsum("ij,ij->i", ad, zd) + np.einsum("ij,ij->i", axis, zdd)
    add = (zdd - axis * rdd[:, None] - 2 * ad * rd[:, None]) / radius[:, None]
    return axis, ad, add, radius


def _arc_at(geometry, arc, parameters):
    from .pipe_nurbs_geometry import pipe_nurbs_geometry

    parameters = _parameters(parameters)
    if np.any((parameters < 0) | (parameters > 1)):
        raise ValueError("PipePose:Progress progress 必须位于 [0,1]")
    nodes = np.asarray(arc["u"]).reshape(-1)
    values = np.asarray(arc["s"]).reshape(-1)
    index = np.clip(np.searchsorted(nodes, parameters, side="right") - 1, 0, len(nodes) - 2)
    x, weight = np.polynomial.legendre.leggauss(16)
    low = nodes[index]
    half = (parameters - low) / 2
    query = (parameters + low)[:, None] / 2 + half[:, None] * x
    derivative = pipe_nurbs_geometry("eval", geometry, query.ravel())[1]
    result = values[index] + half * (np.linalg.norm(derivative, axis=1).reshape(-1, 16) @ weight)
    result[parameters == 0] = 0
    result[parameters == 1] = arc["total_length"]
    return result


def _cross_controls(first, second):
    first, second = np.asarray(first), np.asarray(second)
    p, q = len(first) - 1, len(second) - 1
    result = np.zeros((p + q + 1, 3))
    for i in range(p + 1):
        for j in range(q + 1):
            result[i + j] += comb(p, i) * comb(q, j) / comb(p + q, i + j) * np.cross(first[i], second[j])
    return result


def _hermite(x0, d0, dd0, x1, d1, dd1, width):
    x0, d0, dd0, x1, d1, dd1 = (np.asarray(value, dtype=float) for value in (x0, d0, dd0, x1, d1, dd1))
    return np.stack((x0, x0 + width * d0 / 5, x0 + 2 * width * d0 / 5 + width**2 * dd0 / 20, x1 - 2 * width * d1 / 5 + width**2 * dd1 / 20, x1 - width * d1 / 5, x1))


def _polynomial_data(breaks, controls):
    breaks = _parameters(breaks)
    if len(breaks) < 2 or np.any(np.diff(breaks) <= 0) or not np.isrealobj(controls):
        raise ValueError("PipePose:Polynomial 区间节点与控制向量无效")
    controls = np.asarray(controls, dtype=float)
    if controls.ndim == 2 and len(breaks) == 2:
        controls = controls[:, :, None]
    if controls.ndim != 3 or controls.shape[1] != 3 or controls.shape[2] != len(breaks) - 1 or not np.all(np.isfinite(controls)):
        raise ValueError("PipePose:Polynomial 控制向量需要 degree+1×3×区间数量")
    return breaks, controls


def _polynomial_jet(breaks, controls, parameters, side="right"):
    parameters = _parameters(parameters)
    breaks, controls = _polynomial_data(breaks, controls)
    tolerance = 64 * np.spacing(max(1, abs(breaks[-1])))
    if side not in ("left", "right") or np.any((parameters < breaks[0] - tolerance) | (parameters > breaks[-1] + tolerance)):
        raise ValueError("PipePose:Arc side 或弧长范围无效")
    parameters = np.clip(parameters, breaks[0], breaks[-1])
    coefficients = np.transpose(controls, (0, 2, 1))
    curve = BPoly(coefficients, breaks, extrapolate=False)
    values = [curve(parameters, nu=order) for order in range(4)]
    if side == "left":
        for index, boundary in enumerate(breaks[1:-1], 1):
            active = parameters == boundary
            if np.any(active):
                local = BPoly(coefficients[:, index - 1:index, :], breaks[index - 1:index + 1])
                for order in range(4):
                    values[order][active] = local(boundary, nu=order)
    return tuple(values)


def pipe_pose_math(action: str, *arguments):
    if action == "basis":
        return _basis(*arguments)
    if action == "bernstein":
        return _bernstein(*arguments)
    if action == "beval":
        return _beval(*arguments)
    if action == "extract":
        knots, degree, low, high = arguments
        query = np.linspace(0, 1, int(degree) + 1)
        return np.linalg.solve(_bernstein(degree, query), _basis(knots, degree, low + (high - low) * query)[0])
    if action == "normalize":
        return _normalize(*arguments)
    if action == "gauss":
        return np.polynomial.legendre.leggauss(int(arguments[0]))
    if action == "arc":
        return _arc_at(*arguments)
    if action == "quat":
        matrix = np.asarray(arguments[0], dtype=float)
        if matrix.ndim == 2:
            matrix = matrix[:, :, None]
        quaternion = Rotation.from_matrix(matrix.transpose(2, 0, 1)).as_quat()
        return quaternion[:, [3, 0, 1, 2]]
    if action == "cross_controls":
        return _cross_controls(*arguments)
    if action == "hermite":
        return _hermite(*arguments)
    raise ValueError(f"PipePose:MathAction 未知动作：{action}")
