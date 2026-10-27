import numpy as np

from .combined_pipe_model import _finite_scalar, _finite_vector


def _checked_surface(segment, u, v):
    output = segment["evaluate"](u, v)
    if not isinstance(output, (tuple, list)) or len(output) < 2:
        raise ValueError("evaluate 必须输出位置和单位外法向")
    p, n = np.asarray(output[0]), np.asarray(output[1])
    count = np.asarray(u).size
    if (p.shape != (count, 3) or n.shape != p.shape
            or p.dtype.kind not in "iuf" or n.dtype.kind not in "iuf"
            or not np.all(np.isfinite(p)) or not np.all(np.isfinite(n))):
        raise ValueError("evaluate 必须输出两个有限实数 N×3 数组")
    if np.any(np.abs(np.linalg.norm(n, axis=1) - 1) > 1e-5):
        raise ValueError("evaluate 必须返回单位外法向")
    return p.astype(float), n.astype(float)


def pipe_prepare_model(model):
    if not isinstance(model, dict) or "segments" not in model:
        raise ValueError("model.segments 必须是非空结构数组")
    if not isinstance(model["segments"], (list, tuple)) or not model["segments"]:
        raise ValueError("model.segments 必须是非空结构数组")
    model = dict(model)
    model["segments"] = [dict(segment) for segment in model["segments"]]
    model.setdefault("name", "管道模型")
    model.setdefault("coordinateFrame", "工件坐标系 W")
    model.setdefault("lengthUnit", "mm")
    if model["lengthUnit"] != "mm":
        raise ValueError("请将位置和画布长度统一换算为 mm")
    for segment in model["segments"]:
        if not {"name", "canvasLength", "uRange", "evaluate"}.issubset(segment):
            raise ValueError("每段需要 name、canvasLength、uRange 和 evaluate")
        segment["canvasLength"] = _finite_scalar(segment["canvasLength"], "canvasLength", positive=True)
        interval = _finite_vector(segment["uRange"], "uRange")
        if interval.size != 2 or interval[1] <= interval[0] or not callable(segment["evaluate"]):
            raise ValueError("uRange 必须递增，evaluate 必须可调用")
        segment["uRange"] = interval
        if "thetaOffsetRad" not in segment or np.asarray(segment["thetaOffsetRad"]).size == 0:
            segment["thetaOffsetRad"] = 0.0
        if "thetaSign" not in segment or np.asarray(segment["thetaSign"]).size == 0:
            segment["thetaSign"] = 1
        segment["thetaOffsetRad"] = _finite_scalar(segment["thetaOffsetRad"], "thetaOffsetRad")
        segment["thetaSign"] = _finite_scalar(segment["thetaSign"], "thetaSign")
        if segment["thetaSign"] not in (-1, 1):
            raise ValueError("thetaSign 必须为 +1 或 -1")
    model["breaks"] = np.concatenate(([0.0], np.cumsum([s["canvasLength"] for s in model["segments"]])))
    model["totalLength"] = model["breaks"][-1]
    model["joinReport"] = np.zeros((len(model["segments"]) - 1, 2))
    angle = np.linspace(0, 2 * np.pi, 49)
    for index, segment in enumerate(model["segments"]):
        u, v = np.meshgrid(np.linspace(*segment["uRange"], 7), angle, indexing="ij")
        p, n = _checked_surface(segment, u.ravel(), v.ravel())
        p0, n0 = _checked_surface(segment, u.ravel(), v.ravel() + 2 * np.pi)
        if np.max(np.linalg.norm(p - p0, axis=1)) > 1e-5 or np.max(np.linalg.norm(n - n0, axis=1)) > 1e-5:
            raise ValueError(f"第 {index + 1} 段周向参数不满足 2*pi 周期")
        if index + 1 < len(model["segments"]):
            other = model["segments"][index + 1]
            p1, n1 = _checked_surface(segment, np.full_like(angle, segment["uRange"][1]),
                                      segment["thetaSign"] * angle + segment["thetaOffsetRad"])
            p2, n2 = _checked_surface(other, np.full_like(angle, other["uRange"][0]),
                                      other["thetaSign"] * angle + other["thetaOffsetRad"])
            report = [np.max(np.linalg.norm(p1 - p2, axis=1)), np.max(np.linalg.norm(n1 - n2, axis=1))]
            model["joinReport"][index] = report
            if report[0] > 1e-3 or report[1] > 1e-4:
                raise ValueError(f"第 {index + 1}/{index + 2} 段接缝不连续：位置 {report[0]:.3g} mm，法向差 {report[1]:.3g}")
    return model
