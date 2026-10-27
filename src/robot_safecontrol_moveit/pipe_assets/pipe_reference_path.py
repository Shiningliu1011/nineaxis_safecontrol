import numpy as np
from scipy.optimize import brentq

from .combined_pipe_model import _finite_scalar
from .pipe_path_curve import pipe_path_curve
from .pipe_surface_eval import pipe_surface_eval


def pipe_reference_path(model, anchors, offsetMM, spacingMM=None, pathDefinition=None):
    offsetMM = _finite_scalar(offsetMM, "offsetMM", nonnegative=True)
    if spacingMM is not None and np.asarray(spacingMM).size:
        spacingMM = _finite_scalar(spacingMM, "spacingMM", positive=True)
    else:
        spacingMM = None
    curve = pipe_path_curve(model, anchors, pathDefinition)
    anchors = curve["anchors"]
    evaluate = curve["evaluate"]
    if len(anchors) == 0:
        uv, parameter = np.empty((0, 2)), np.empty(0)
    elif len(anchors) == 1:
        uv, parameter = anchors.copy(), np.array([0.0])
    else:
        tq = np.unique(np.concatenate((curve["breaks"], np.linspace(0, 1, max(601, 30 * len(anchors))))))
        for boundary in model["breaks"][1:-1]:
            f = evaluate(tq)[0][:, 0] - boundary
            crosses = np.flatnonzero(f[:-1] * f[1:] < 0)
            extra = [brentq(lambda z, boundary=boundary: evaluate([z])[0][0, 0] - boundary,
                            tq[index], tq[index + 1], xtol=np.finfo(float).eps,
                            rtol=4 * np.finfo(float).eps) for index in crosses]
            tq = np.unique(np.concatenate((tq, extra)))
        for _ in range(4):
            sampled = evaluate(tq)[0]
            mid_x = (sampled[:-1, 0] + sampled[1:, 0]) / 2
            local_length = np.zeros_like(mid_x)
            for index, segment in enumerate(model["segments"]):
                mask = ((mid_x >= model["breaks"][index] - 1e-8)
                        & (mid_x <= model["breaks"][index + 1] + 1e-8))
                local_length[mask] = segment["canvasLength"]
            if np.any(local_length <= 0):
                raise ValueError("参考路径超出曲面参数范围")
            parts = np.maximum(1, np.ceil(np.abs(np.diff(sampled[:, 0])) / local_length * 48 - 1e-8)).astype(int)
            refine = np.flatnonzero(parts > 1)
            if not len(refine):
                break
            extra = [np.linspace(tq[index], tq[index + 1], parts[index] + 1)[1:-1] for index in refine]
            tq = np.unique(np.concatenate([tq, *extra]))
        if spacingMM is not None:
            max_step = max(model["totalLength"] / 20000, spacingMM / 3)
            for _ in range(8):
                points = pipe_surface_eval(model, evaluate(tq)[0], offsetMM)[0]
                bad = np.linalg.norm(np.diff(points, axis=0), axis=1) > max_step
                if not np.any(bad) or len(tq) > 100000:
                    break
                indices = np.flatnonzero(bad)
                tq = np.sort(np.concatenate((tq, (tq[indices] + tq[indices + 1]) / 2)))
            points = pipe_surface_eval(model, evaluate(tq)[0], offsetMM)[0]
            arc = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))))
            keep = np.concatenate(([True], np.diff(arc) > 1e-12))
            arc, tq = arc[keep], tq[keep]
            if len(arc) > 1 and arc[-1] > 0:
                if arc[-1] / spacingMM > 200000:
                    raise ValueError("采样点超过 200000，请增大 spacingMM")
                count = max(1, int(np.ceil(arc[-1] / spacingMM)))
                targets = np.linspace(0, arc[-1], count + 1)
                parameter = np.interp(targets, arc, tq)
                uv = evaluate(parameter)[0]
            else:
                uv, parameter = anchors[:1].copy(), np.array([0.0])
        else:
            parameter, uv = tq, evaluate(tq)[0]
    tcp, points, normals, ids, local = pipe_surface_eval(model, uv, offsetMM)
    arc = (np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(tcp, axis=0), axis=1))))
           if len(tcp) else np.empty(0))
    return {
        "schemaVersion": 2, "modelName": model["name"], "coordinateFrame": model["coordinateFrame"],
        "lengthUnit": "mm", "offsetMM": offsetMM, "sampleSpacingMM": spacingMM,
        "pathDefinition": curve["pathDefinition"], "pathParameter": parameter,
        "keypointsCanvas": anchors, "canvasUV": uv, "localUV": local, "segmentId": ids,
        "surfacePoints": points, "tcpPoints": tcp, "normals": normals, "toolAxis": -normals,
        "arcLengthMM": arc,
        "note": "C2 参数参考线或解析示例的几何参考采样点；三维 NURBS 需要检查离面误差和曲率，工具滚转角需要独立确定。",
    }
