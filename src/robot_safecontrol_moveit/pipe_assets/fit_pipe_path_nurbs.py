from pathlib import Path

import numpy as np
from scipy.interpolate import make_interp_spline

from .mat_io import load_mat


def _points(value):
    array = np.asarray(value)
    if (array.ndim != 2 or array.shape[1] != 3 or len(array) == 0
            or not np.issubdtype(array.dtype, np.number) or np.iscomplexobj(array)
            or not np.isfinite(array).all()):
        raise ValueError("PipeNURBS:InvalidCoordinates：TCP 路径需要非空有限实数 N×3 数组")
    return array.astype(float)


def fit_pipe_path_nurbs(pathFile, outputFolder="", showFigure=False):
    if showFigure not in (False, True):
        raise ValueError("showFigure 必须为逻辑值")
    filename = Path(pathFile)
    if not filename.is_file():
        raise FileNotFoundError(filename)
    if filename.suffix.lower() == ".mat":
        source = load_mat(filename)
        trajectory = source.get("trajectory", {})
        has_trajectory = isinstance(trajectory, dict) and "tcpPoints" in trajectory
        if "data_points" in source:
            raw = _points(source["data_points"])
        elif has_trajectory:
            raw = _points(trajectory["tcpPoints"])
        else:
            raise ValueError("PipeNURBS:MissingCoordinates：需要 data_points 或 trajectory.tcpPoints")
        if has_trajectory:
            tcp = _points(trajectory["tcpPoints"])
            if raw.shape != tcp.shape or np.any(np.abs(raw - tcp) > 1e-9):
                raise ValueError("PipeNURBS:ConflictingCoordinates：两份 TCP 坐标不一致")
            if str(trajectory.get("lengthUnit", "mm")).lower() != "mm":
                raise ValueError("PipeNURBS:InvalidUnit：TCP 坐标需要 mm")
    elif filename.suffix.lower() == ".csv":
        table = np.genfromtxt(filename, delimiter=",", names=True, encoding="utf-8")
        required = ("tcp_x_mm", "tcp_y_mm", "tcp_z_mm")
        if not set(required).issubset(table.dtype.names or ()):
            raise ValueError("PipeNURBS:MissingCoordinates：CSV 缺少 tcp_x_mm/tcp_y_mm/tcp_z_mm")
        source = {"table": {name: np.atleast_1d(table[name]) for name in table.dtype.names}}
        raw = _points(np.column_stack([table[name] for name in required]))
    else:
        raise ValueError("PipeNURBS:UnsupportedFormat：需要 MAT 或 CSV")
    keep = np.ones(len(raw), dtype=bool)
    last = 0
    for index in range(1, len(raw)):
        if np.linalg.norm(raw[index] - raw[last]) <= 1e-9:
            keep[index] = False
        else:
            last = index
    points = raw[keep]
    count = len(points)
    if count < 4:
        raise ValueError("PipeNURBS:TooFewPoints：三次插值至少需要四个有效点")
    parameters = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    parameters /= parameters[-1]
    parameters[-1] = 1.
    if not np.isfinite(parameters).all() or np.any(np.diff(parameters) <= 0):
        raise ValueError("PipeNURBS:InvalidParameters：弦长参数需要严格递增")
    knots = np.r_[np.zeros(4), [np.mean(parameters[j + 1:j + 4]) for j in range(count - 4)], np.ones(4)]
    spline = make_interp_spline(parameters, points, k=3, t=knots)
    errors = np.linalg.norm(spline(parameters) - points, axis=1)
    if (not np.isfinite(spline.c).all() or not np.isfinite(errors).all()
            or np.max(errors) > 1e-9 * max(1., np.max(np.abs(points)))):
        raise RuntimeError("PipeNURBS:InterpolationError：输入点插值误差未通过数值检查")
    curve_parameters = np.r_[
        (parameters[:-1, None] + np.diff(parameters)[:, None] * np.arange(5) / 5).ravel(), 1.,
    ]
    curve_parameters[::5] = parameters
    curve = spline(curve_parameters)
    obj = {"ControlPoints": spline.c.copy(), "UVector": knots, "Weights": np.ones(count),
           "CurveData": curve, "DataPoint": points, "Number_of_ControlPoints": count, "Degree": 3}
    inputs = np.full_like(curve, np.nan)
    inputs[::5] = points
    input_rows = np.full(len(curve), np.nan)
    input_rows[::5] = np.flatnonzero(keep) + 1
    component_errors = curve - inputs
    names = ("curve_index", "u", "fit_x_mm", "fit_y_mm", "fit_z_mm", "is_input_point", "input_row",
             "input_x_mm", "input_y_mm", "input_z_mm", "error_x_mm", "error_y_mm", "error_z_mm", "point_error_mm")
    table = np.column_stack([
        np.arange(1, len(curve) + 1), curve_parameters, curve,
        (np.arange(len(curve)) % 5 == 0), input_rows, inputs, component_errors,
        np.linalg.norm(component_errors, axis=1),
    ])
    result = {
        "source_file": str(filename), "source": source, "length_unit": "mm",
        "fit_method": "cubic global interpolation, chord-length parameters, averaged knots, unit weights",
        "degree": 3, "data_points": points, "input_indices": np.flatnonzero(keep) + 1,
        "removed_input_indices": np.flatnonzero(~keep) + 1, "duplicate_tolerance_mm": 1e-9,
        "data_parameters": parameters, "control_points": obj["ControlPoints"], "knot_vector": knots,
        "weights": obj["Weights"], "curve_parameters": curve_parameters, "curve_points": curve,
        "point_errors_mm": errors, "max_point_error_mm": float(np.max(errors)),
        "rms_point_error_mm": float(np.sqrt(np.mean(errors ** 2))),
        "curve_table": {name: table[:, index] for index, name in enumerate(names)},
        "note": "插值误差针对原始 TCP 点；点间曲线的表面约束和姿态需要独立验证。",
    }
    if outputFolder:
        destination = Path(outputFolder)
        destination.mkdir(parents=True, exist_ok=True)
        np.savetxt(destination / "control_points.txt", obj["ControlPoints"], fmt="%.17g")
        np.savetxt(destination / "knot_vector.txt", knots[None, :], fmt="%.17g")
        np.savetxt(destination / "curve_data.csv", table, delimiter=",", header=",".join(names),
                   comments="", fmt="%.17g")
    if showFigure:
        from .plot_style import configure_fonts

        configure_fonts()
        import matplotlib.pyplot as plt

        figure = plt.figure()
        axes = figure.add_subplot(projection="3d")
        axes.plot(*curve.T, label="NURBS 插值曲线")
        axes.scatter(*points[::max(1, int(np.ceil(count / 30)))].T, label="原始 TCP 点")
        axes.set(xlabel="X (mm)", ylabel="Y (mm)", zlabel="Z (mm)")
        axes.legend()
    return obj, result
