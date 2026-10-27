import numpy as np
from scipy.interpolate import BPoly, CubicSpline

from .combined_pipe_model import _finite_scalar, _finite_vector
from .pipe_surface_eval import _canvas_points


def pipe_path_curve(model, anchors, pathDefinition=None):
    if pathDefinition is None or (isinstance(pathDefinition, (list, tuple, np.ndarray))
                                  and np.asarray(pathDefinition).size == 0):
        pathDefinition = {"type": "c2-spline"}
    if not isinstance(pathDefinition, dict) or "type" not in pathDefinition:
        raise ValueError("路径定义必须包含 type")
    pathDefinition = dict(pathDefinition)
    anchors = _canvas_points(anchors, "anchors")
    total_length = _finite_scalar(model["totalLength"], "totalLength", positive=True)
    if np.any(anchors[:, 0] < 0) or np.any(anchors[:, 0] > total_length):
        raise ValueError("关键点超出管道画布范围")
    if len(anchors):
        step = np.linalg.norm(np.diff(anchors, axis=0) / [total_length, 360], axis=1)
        anchors = anchors[np.concatenate(([True], step > 1e-12))]
    kind = pathDefinition["type"]
    if kind not in ("c2-spline", "analytic-sine"):
        raise ValueError(f"未知的路径类型：{kind}")
    breaks = np.array([0.0, 1.0])
    interpolant = None
    if kind == "analytic-sine":
        fields = ("xStart", "xEnd", "thetaMidDeg", "thetaAmplitudeDeg", "cycles")
        if not set(fields).issubset(pathDefinition):
            raise ValueError("解析正弦路径的参数不完整")
        for field in fields:
            pathDefinition[field] = _finite_scalar(pathDefinition[field], field)
        if (pathDefinition["xStart"] < 0 or pathDefinition["xEnd"] > total_length
                or pathDefinition["xEnd"] <= pathDefinition["xStart"] or pathDefinition["cycles"] <= 0):
            raise ValueError("解析正弦路径的轴向范围或周期数无效")
        if len(anchors):
            ta = ((anchors[:, 0] - pathDefinition["xStart"])
                  / (pathDefinition["xEnd"] - pathDefinition["xStart"]))
            expected = (pathDefinition["thetaMidDeg"] + pathDefinition["thetaAmplitudeDeg"]
                        * np.sin(2 * np.pi * pathDefinition["cycles"] * ta))
            if np.any(ta < -1e-12) or np.any(ta > 1 + 1e-12) or np.any(np.abs(expected - anchors[:, 1]) > 1e-7):
                raise ValueError("解析定义与关键点不匹配；编辑后的路径需要使用 c2-spline")
    elif len(anchors) > 1:
        breaks = np.concatenate(([0.0], np.cumsum(np.linalg.norm(
            np.diff(anchors, axis=0) / [total_length, 360], axis=1))))
        breaks /= breaks[-1]
        spline = CubicSpline(breaks, anchors, axis=0)
        velocity, acceleration = spline(breaks, 1), spline(breaks, 2)
        widths = np.diff(breaks)
        # 共用端点导数，并约束轴向 Bezier 控制点位于模型范围内。
        for index, position in enumerate(anchors[:, 0]):
            shifts = []
            if index > 0:
                width = widths[index - 1]
                shifts.extend((-width * velocity[index, 0] / 5,
                               -2 * width * velocity[index, 0] / 5 + width**2 * acceleration[index, 0] / 20))
            if index < len(anchors) - 1:
                width = widths[index]
                shifts.extend((width * velocity[index, 0] / 5,
                               2 * width * velocity[index, 0] / 5 + width**2 * acceleration[index, 0] / 20))
            shifts = np.asarray(shifts)
            scale = 1.0
            if np.any(shifts > 0):
                scale = min(scale, np.min((total_length - position) / shifts[shifts > 0]))
            if np.any(shifts < 0):
                scale = min(scale, np.min(-position / shifts[shifts < 0]))
            scale = max(0, scale)
            velocity[index, 0] *= scale
            acceleration[index, 0] *= scale
        interpolant = BPoly.from_derivatives(
            breaks, [np.array([p, v, a]) for p, v, a in zip(anchors, velocity, acceleration)])

    def evaluate(t):
        t = _finite_vector(t, "路径参数", allow_empty=True)
        if np.any(t < 0) or np.any(t > 1):
            raise ValueError("路径参数必须位于 [0,1]")
        if kind == "analytic-sine":
            omega, amplitude = 2 * np.pi * pathDefinition["cycles"], pathDefinition["thetaAmplitudeDeg"]
            span = pathDefinition["xEnd"] - pathDefinition["xStart"]
            uv = np.column_stack((pathDefinition["xStart"] + span * t,
                                  pathDefinition["thetaMidDeg"] + amplitude * np.sin(omega * t)))
            d1 = np.column_stack((np.full_like(t, span), amplitude * omega * np.cos(omega * t)))
            d2 = np.column_stack((np.zeros_like(t), -amplitude * omega**2 * np.sin(omega * t)))
        elif len(anchors) < 2:
            uv = np.zeros((len(t), 2)) if not len(anchors) else np.tile(anchors[0], (len(t), 1))
            d1, d2 = np.zeros_like(uv), np.zeros_like(uv)
        else:
            uv, d1, d2 = interpolant(t), interpolant(t, 1), interpolant(t, 2)
        return uv, d1, d2

    return {"anchors": anchors, "pathDefinition": pathDefinition, "breaks": breaks, "evaluate": evaluate}
