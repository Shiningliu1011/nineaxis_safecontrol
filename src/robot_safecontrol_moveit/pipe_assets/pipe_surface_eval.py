import numpy as np

from .combined_pipe_model import _finite_scalar
from .pipe_prepare_model import _checked_surface


def _canvas_points(value, name="canvasUV"):
    array = np.asarray(value)
    if array.size == 0 and array.ndim == 1:
        array = np.empty((0, 2))
    if (array.ndim != 2 or array.shape[1] != 2 or array.dtype.kind not in "iuf"
            or not np.all(np.isfinite(array))):
        raise ValueError(f"{name} 必须是有限实数 N×2 数组")
    return array.astype(float)


def pipe_surface_eval(model, canvasUV, offsetMM):
    offsetMM = _finite_scalar(offsetMM, "offsetMM", nonnegative=True)
    canvasUV = _canvas_points(canvasUV)
    if not isinstance(model, dict) or not {"segments", "breaks", "totalLength"}.issubset(model):
        raise ValueError("model 必须经过 pipe_prepare_model 校验")
    x = canvasUV[:, 0]
    if np.any(x < -1e-9) or np.any(x > model["totalLength"] + 1e-9):
        raise ValueError("路径超出管道画布范围")
    x = np.clip(x, 0, model["totalLength"])
    angle = np.deg2rad(canvasUV[:, 1])
    count = len(canvasUV)
    surface = np.zeros((count, 3))
    normal = np.zeros_like(surface)
    segmentId = np.searchsorted(model["breaks"][1:-1], x, side="right") + 1
    localUV = np.zeros((count, 2))
    for index, segment in enumerate(model["segments"], 1):
        mask = segmentId == index
        if not np.any(mask):
            continue
        u = (segment["uRange"][0] + (x[mask] - model["breaks"][index - 1])
             / segment["canvasLength"] * np.diff(segment["uRange"])[0])
        v = segment["thetaSign"] * angle[mask] + segment["thetaOffsetRad"]
        p, n = _checked_surface(segment, u, v)
        surface[mask], normal[mask] = p, n
        localUV[mask] = np.column_stack((u, v))
    tcp = surface + offsetMM * normal
    return tcp, surface, normal, segmentId, localUV
