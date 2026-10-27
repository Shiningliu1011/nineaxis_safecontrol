import numpy as np

from .pipe_path_curve import pipe_path_curve
from .pipe_reference_path import pipe_reference_path
from .pipe_surface_eval import pipe_surface_eval


def pipe_path_curvature(model, anchors, offsetMM, pathDefinition=None):
    reference = pipe_reference_path(model, anchors, offsetMM, None, pathDefinition)
    curve = pipe_path_curve(model, anchors, reference["pathDefinition"])
    if len(curve["anchors"]) < 2:
        parameter = reference["pathParameter"]
    else:
        parameter = np.unique(np.concatenate((reference["pathParameter"], np.linspace(0, 1, 1201))))
    uv = curve["evaluate"](parameter)[0]
    if not len(parameter):
        uv = np.empty((0, 2))
    tcp, surface, normal, segmentId, localUV = pipe_surface_eval(model, uv, offsetMM)
    count = len(parameter)
    d1, d2 = np.zeros((count, 3)), np.zeros((count, 3))
    if len(curve["anchors"]) > 1:
        h = 2e-5
        central = (parameter >= 2 * h) & (parameter <= 1 - 2 * h)
        groups = (np.flatnonzero(central), np.flatnonzero(parameter < 2 * h), np.flatnonzero(parameter > 1 - 2 * h))
        offsets = ((-2, -1, 0, 1, 2), (0, 1, 2, 3, 4), (-4, -3, -2, -1, 0))
        first_weights = ((1, -8, 0, 8, -1), (-25, 48, -36, 16, -3), (3, -16, 36, -48, 25))
        second_weights = ((-1, 16, -30, 16, -1), (35, -104, 114, -56, 11), (11, -56, 114, -104, 35))
        for ids, group_offsets, first_coefficients, second_coefficients in zip(groups, offsets, first_weights, second_weights):
            if not len(ids):
                continue
            first, second = np.zeros((len(ids), 3)), np.zeros((len(ids), 3))
            for shift, first_weight, second_weight in zip(group_offsets, first_coefficients, second_coefficients):
                sampled_uv = curve["evaluate"](parameter[ids] + h * shift)[0]
                # 使用当前位置作为原点，减少较大工件坐标引起的数值误差。
                points = pipe_surface_eval(model, sampled_uv, offsetMM)[0] - tcp[ids]
                first += first_weight * points
                second += second_weight * points
            d1[ids], d2[ids] = first / (12 * h), second / (12 * h**2)
    speed = np.linalg.norm(d1, axis=1)
    valid = (np.isfinite(speed) & (speed > max(1, np.max(speed)) * 1e-9)) if len(speed) else np.empty(0, dtype=bool)
    kappa = np.full(count, np.nan)
    kappa[valid] = np.linalg.norm(np.cross(d1[valid], d2[valid]), axis=1) / speed[valid]**3
    valid &= np.isfinite(kappa)
    kappa[~valid] = np.nan
    arc = (np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(tcp, axis=0), axis=1))))
           if len(tcp) else np.empty(0))
    return {
        "parameter": parameter, "pathParameter": parameter, "arcLengthMM": arc,
        "kappa": kappa, "valid": valid, "canvasUV": uv, "tcpPoints": tcp,
        "surfacePoints": surface, "normals": normal, "segmentId": segmentId,
        "localUV": localUV, "pathDefinition": reference["pathDefinition"],
        "firstDerivative": d1, "secondDerivative": d2, "lengthUnit": "mm", "curvatureUnit": "1/mm",
        "method": "连续参考 TCP 曲面映射的五点差分",
    }
