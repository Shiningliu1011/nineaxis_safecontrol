import numpy as np

from .combined_pipe_model import _paired_parameters


def _surface(u, v, *, reducer):
    u, v = _paired_parameters(u, v)
    length = 140 if reducer else 180
    if np.any(u < 0) or np.any(u > length):
        raise ValueError("u 超出本管段的 uRange")
    c, s = np.cos(v), np.sin(v)
    if reducer:
        t = u / 140
        h = 35 * t**4 - 84 * t**5 + 70 * t**6 - 20 * t**7
        dh = t**3 * (1 - t)**3
        radius, dr = 35 + 20 * h, 20 * dh
        z = 180 + u
    else:
        radius, dr, z = np.full_like(u, 35.0), np.zeros_like(u), u
    p = np.column_stack((radius * c, radius * s, z))
    n = np.column_stack((c, s, -dr)) / np.sqrt(1 + dr**2)[:, None]
    pu = np.column_stack((dr * c, dr * s, np.ones_like(u)))
    pv = np.column_stack((-radius * s, radius * c, np.zeros_like(u)))
    return p, n, pu, pv


def pipe_demo_model():
    return {
        "name": "测试：直管 + 光滑变径段", "lengthUnit": "mm",
        "coordinateFrame": "工件坐标系 W",
        "segments": [
            {"name": "直管", "canvasLength": 180.0, "uRange": np.array([0.0, 180.0]),
             "thetaOffsetRad": 0.0, "thetaSign": 1,
             "evaluate": lambda u, v: _surface(u, v, reducer=False)},
            {"name": "变径段", "canvasLength": 140.0, "uRange": np.array([0.0, 140.0]),
             "thetaOffsetRad": 0.0, "thetaSign": 1,
             "evaluate": lambda u, v: _surface(u, v, reducer=True)},
        ],
    }
