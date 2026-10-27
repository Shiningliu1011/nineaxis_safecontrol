import numpy as np


def example_path(model, thetaRangeDeg=(-180, 540)):
    limits = np.asarray(thetaRangeDeg, dtype=float)
    if limits.shape != (2,) or not np.all(np.isfinite(limits)) or not 0 < np.ptp(limits) <= 3600:
        raise ValueError("ThetaRangeDeg 必须递增且跨度不超过 3600 deg")
    length = model["totalLength"]
    middle, amplitude = np.mean(limits), min(85, np.ptp(limits) * 0.2)
    if len(model["segments"]) > 2:
        x = np.unique(np.r_[np.linspace(0, length, 4 * len(model["segments"])), model["breaks"]])
        begin, end, cycles = 0.0, length, 2
    else:
        begin, end, cycles = 0.05 * length, 0.95 * length, 1
        x = np.linspace(begin, end, 18)
    definition = {"type": "analytic-sine", "xStart": begin, "xEnd": end,
                  "thetaMidDeg": middle, "thetaAmplitudeDeg": amplitude, "cycles": cycles}
    points = np.column_stack((x, middle + amplitude * np.sin(2 * np.pi * cycles * (x - begin) / (end - begin))))
    return points, definition
