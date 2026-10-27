import csv
from pathlib import Path

import numpy as np

from .mat_io import load_mat, save_mat
from .pipe_surface_eval import pipe_surface_eval


def model_signature(model):
    x, angle = np.meshgrid(np.linspace(0, model["totalLength"], 31), [0, 90, 180, 270], indexing="ij")
    uv = np.column_stack((x.ravel(order="F"), angle.ravel(order="F")))
    _, points, normals, _, _ = pipe_surface_eval(model, uv, 0)
    return np.column_stack((points, normals))


def export_path(model, trajectory, settings, stem):
    stem = Path(stem)
    if stem.suffix.lower() == ".mat":
        stem = stem.with_suffix("")
    if not stem.parent.is_dir():
        raise FileNotFoundError(stem.parent)
    files = [Path(str(stem) + ".mat"), Path(str(stem) + ".csv")]
    save_mat(files[0], {
        "trajectory": trajectory,
        "data_points": trajectory["tcpPoints"],
        "settings": settings,
        "modelSignature": model_signature(model),
        "modelInfo": {
            "name": model["name"], "coordinateFrame": model["coordinateFrame"],
            "segmentNames": [segment["name"] for segment in model["segments"]],
            "breaks": model["breaks"],
        },
    })
    values = np.column_stack((
        np.arange(1, len(trajectory["tcpPoints"]) + 1), trajectory["canvasUV"],
        trajectory["segmentId"], trajectory["localUV"], trajectory["surfacePoints"],
        trajectory["tcpPoints"], trajectory["normals"], trajectory["toolAxis"], trajectory["arcLengthMM"],
    ))
    names = ("index", "canvas_s_mm", "theta_deg", "segment_id", "local_u", "local_v_rad",
             "surface_x_mm", "surface_y_mm", "surface_z_mm", "tcp_x_mm", "tcp_y_mm", "tcp_z_mm",
             "normal_x", "normal_y", "normal_z", "tool_z_x", "tool_z_y", "tool_z_z", "arc_mm")
    with files[1].open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(names)
        writer.writerows(values)
    return files


def load_editable_path(model, filename):
    saved = load_mat(filename)
    required = {"trajectory", "settings", "modelSignature", "modelInfo"}
    if not required <= saved.keys():
        raise ValueError("文件缺少可编辑路径信息")
    signature, expected = np.asarray(saved["modelSignature"]), model_signature(model)
    if signature.shape != expected.shape or not np.all(np.isfinite(signature)) or np.max(np.abs(signature - expected)) > 1e-6:
        raise ValueError("路径文件的管道几何不同")
    info = saved["modelInfo"]
    if not np.array_equal(np.ravel(info["breaks"]), model["breaks"]) or info["coordinateFrame"] != model["coordinateFrame"]:
        raise ValueError("路径文件的管道范围或坐标系不同")
    points = np.asarray(saved["trajectory"]["keypointsCanvas"])
    if points.shape == (2,):
        points = points[None, :]
    if points.ndim != 2 or points.shape[1] != 2 or not np.isrealobj(points) or not np.all(np.isfinite(points)):
        raise ValueError("路径文件的关键点必须为有限实数 N×2 数组")
    saved["trajectory"]["keypointsCanvas"] = points.astype(float)
    return saved
