import csv
import hashlib
import importlib
from copy import deepcopy
from pathlib import Path

import numpy as np
from scipy.interpolate import PchipInterpolator, PPoly
from scipy.optimize import brentq

from .mat_io import load_mat
from .pipe_nurbs_geometry import pipe_nurbs_geometry
from .pipe_path_curve import pipe_path_curve
from .pipe_pose_math import _parameters
from .pipe_pose_prepare_model import pipe_pose_prepare_model
from .pipe_surface_eval import pipe_surface_eval

MATLAB_MODEL_HASHES = {
    "pipe_demo_model": "d3d7d0dd04565c69233447203bc9338611e9ced0401668beeae6ac7bcd762b0d",
    "pipe_prepare_model": "2c4986b31f7017bb092b88019b5b097983d7c8c5a30afaf120ffc657d1e49928",
    "pipe_pose_prepare_model": "b2c164d8e9a700ef51467bb0cc2e7182e16f1a06230aedddab1dd867154d4dd9",
    "pipe_surface_eval": "efb9790629e5c207f8d4d71fed9ecbf0dbf246b0f51e42abe87b93adbd55b717",
    "pipe_path_curve": "cdc8946e9b9c4d59d0db829b86cebd47ce1870ae5d13a0cabe921b94c4d698bd",
}


def _module(name):
    if not isinstance(name, str) or not name.isidentifier():
        raise ValueError("PipePose:ModelSource model_factory 需要合法模块名称")
    return importlib.import_module(f"{__package__}.{name}")


def _sources(names):
    return {name: Path(_module(name).__file__).read_text() for name in names}


def _strip_handles(value):
    if isinstance(value, dict):
        return {name: _strip_handles(item) for name, item in value.items() if not callable(item)}
    if isinstance(value, (list, tuple)):
        return [_strip_handles(item) for item in value]
    return value


def _equal(first, second):
    if isinstance(first, dict) and isinstance(second, dict):
        return set(first) == set(second) and all(_equal(first[name], second[name]) for name in first)
    if isinstance(first, (list, tuple)) or isinstance(second, (list, tuple)):
        return len(first) == len(second) and all(_equal(a, b) for a, b in zip(first, second))
    a, b = np.asarray(first), np.asarray(second)
    if a.size == 0 and b.size == 0:
        return True
    return a.squeeze().shape == b.squeeze().shape and np.array_equal(a.squeeze(), b.squeeze(), equal_nan=a.dtype.kind in "fc" and b.dtype.kind in "fc")


def _require_close(first, second, tolerance, code):
    first, second = np.asarray(first), np.asarray(second)
    if first.shape != second.shape or not np.isrealobj(first) or not np.isrealobj(second) or np.any(~np.isfinite(first)) or np.any(~np.isfinite(second)) or np.any(np.abs(first - second) > tolerance):
        raise ValueError(f"PipePose:{code} 来源、模型或曲线身份检查失败")


def _factory(name, arguments):
    return getattr(_module(name), name)(*list(arguments))


def _signature(model):
    axial, angle = np.meshgrid(np.linspace(0, model["totalLength"], 31), [0, 90, 180, 270], indexing="ij")
    _, position, normal, _, _ = pipe_surface_eval(model, np.column_stack((axial.ravel(order="F"), angle.ravel(order="F"))), 0)
    return np.column_stack((position, normal))


def _seam_roots(curve, model):
    result = []
    samples = np.linspace(0, 1, 6)
    for low, high in zip(curve["breaks"][:-1], curve["breaks"][1:]):
        coordinates = curve["evaluate"](low + (high - low) * samples)[0][:, 0]
        coefficients = np.polynomial.polynomial.polyfit(samples, coordinates, 5)
        for seam in model["breaks"][1:-1]:
            polynomial = coefficients.copy()
            polynomial[0] -= seam
            while len(polynomial) > 1 and abs(polynomial[-1]) < 1e-10 * max(1, np.max(np.abs(polynomial))):
                polynomial = polynomial[:-1]
            if len(polynomial) < 2:
                continue
            roots = np.polynomial.Polynomial(polynomial).roots()
            roots = roots[(np.abs(roots.imag) < 1e-8) & (roots.real >= -1e-10) & (roots.real <= 1 + 1e-10)].real
            result.extend(low + (high - low) * np.clip(roots, 0, 1))
    result = np.sort(result)
    if not len(result):
        return np.empty(0)
    result = result[np.r_[True, np.diff(result) > 1e-8]]
    return result[(result > 0) & (result < 1)]


def _pairs_csv(filename):
    with Path(filename).open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        fields = {"is_input_point", "input_row", "u", "input_x_mm", "input_y_mm", "input_z_mm"}
        if not fields.issubset(reader.fieldnames or ()):
            raise ValueError("PipePose:Pairs CSV 缺少原始点关联字段")
        rows = [row for row in reader if float(row["is_input_point"]) == 1]
    return {"input_indices": np.asarray([float(row["input_row"]) for row in rows]), "data_parameters": np.asarray([float(row["u"]) for row in rows]), "data_points": np.asarray([[float(row[name]) for name in ("input_x_mm", "input_y_mm", "input_z_mm")] for row in rows])}


def _build(geometry, source, pairs, cfg):
    source_file = str(source) if isinstance(source, (str, Path)) else ""
    if source_file:
        source = load_mat(source_file)
    if not {"trajectory", "modelSignature", "modelInfo", "settings"}.issubset(source):
        raise ValueError("PipePose:Source 缺少原始路径和模型资料")
    trajectory = source["trajectory"]
    required = {"pathParameter", "pathDefinition", "keypointsCanvas", "canvasUV", "tcpPoints", "surfacePoints", "normals", "toolAxis", "localUV", "segmentId", "offsetMM", "coordinateFrame", "lengthUnit"}
    if not required.issubset(trajectory):
        raise ValueError("PipePose:Source 原始轨迹字段不完整")
    model = pipe_pose_prepare_model(_factory(cfg["model_factory"], cfg["model_arguments"]), cfg["verified_normal_jump_seams"])
    if trajectory["lengthUnit"] != "mm" or trajectory["coordinateFrame"] != model["coordinateFrame"] or source["modelInfo"]["coordinateFrame"] != model["coordinateFrame"] or not _equal(source["modelInfo"]["breaks"], model["breaks"]):
        raise ValueError("PipePose:ModelMismatch 单位、坐标或分段范围不同")
    signature = _signature(model)
    _require_close(signature, source["modelSignature"], cfg["source_tolerance_mm"], "ModelMismatch")
    if abs(source["settings"]["OffsetMM"] - trajectory["offsetMM"]) > cfg["source_tolerance_mm"]:
        raise ValueError("PipePose:Source 工作距离元数据不同")
    tau = _parameters(trajectory["pathParameter"])
    count = len(trajectory["tcpPoints"])
    if len(tau) != count or np.any((tau < 0) | (tau > 1)) or np.any(np.diff(tau) <= 0):
        raise ValueError("PipePose:Source 原始 pathParameter 需要严格递增")
    curve = pipe_path_curve(model, trajectory["keypointsCanvas"], trajectory["pathDefinition"])
    uv = curve["evaluate"](tau)[0]
    _require_close(uv, trajectory["canvasUV"], cfg["source_tolerance_mm"], "PathMismatch")
    tcp, surface, normal, segment, local = pipe_surface_eval(model, uv, trajectory["offsetMM"])
    for a, b, tolerance, code in ((tcp, trajectory["tcpPoints"], cfg["source_tolerance_mm"], "SourceTCP"), (surface, trajectory["surfacePoints"], cfg["source_tolerance_mm"], "SourceSurface"), (normal, trajectory["normals"], cfg["normal_tolerance"], "NormalOrientation"), (-normal, trajectory["toolAxis"], cfg["normal_tolerance"], "NormalOrientation"), (local, trajectory["localUV"], cfg["source_tolerance_mm"], "PathMismatch"), (segment, trajectory["segmentId"], 0, "PathMismatch")):
        _require_close(a, b, tolerance, code)
    if "data_points" in source:
        _require_close(source["data_points"], tcp, cfg["source_tolerance_mm"], "SourceTCP")
    index, parameters = _parameters(pairs["input_indices"]), _parameters(pairs["data_parameters"])
    if len(index) != len(parameters) or len(index) < 4 or np.any(index != np.round(index)) or np.any((index < 1) | (index > count)) or np.any(np.diff(index) <= 0) or np.any(np.diff(parameters) <= 0) or parameters[0] != 0 or parameters[-1] != 1:
        raise ValueError("PipePose:Pairs 原始行号与曲线参数关联无效")
    keep = [0]
    for row in range(1, count):
        if np.linalg.norm(tcp[row] - tcp[keep[-1]]) > 1e-9:
            keep.append(row)
    if not np.array_equal(index, np.asarray(keep) + 1):
        raise ValueError("PipePose:Pairs 原始行号与连续重复点筛选不同")
    index = index.astype(int) - 1
    expected = np.r_[0, np.cumsum(np.linalg.norm(np.diff(tcp[index], axis=0), axis=1))]
    expected /= expected[-1]
    _require_close(parameters, expected, 2e-12, "Pairs")
    _require_close(pairs["data_points"], tcp[index], cfg["source_tolerance_mm"], "SourceMismatch")
    _require_close(pipe_nurbs_geometry("eval", geometry, parameters)[0], tcp[index], cfg["source_tolerance_mm"], "SourceMismatch")
    if "control_points" in pairs:
        _require_close(geometry["ctrl_pts"], pairs["control_points"], 0, "SourceMismatch")
    if "knot_vector" in pairs:
        _require_close(np.ravel(geometry["knots"]), np.ravel(pairs["knot_vector"]), 0, "SourceMismatch")
    mapping = PchipInterpolator(parameters, tau[index])
    mapping_pp = {"form": "pp", "breaks": mapping.x, "coefs": mapping.c.T, "pieces": len(mapping.x) - 1, "order": 4, "dim": 1}
    events_tau = _seam_roots(curve, model)
    events_u = np.asarray([brentq(lambda value: mapping(value) - t, 0, 1, xtol=1e-14) for t in events_tau if tau[index[0]] <= t <= tau[index[-1]]])
    return {"format_version": 1, "source_file": source_file, "source_snapshot": deepcopy(source), "geometry": geometry, "input_indices": index + 1, "data_parameters": parameters, "path_tau": tau[index], "mapping_pp": mapping_pp, "model_factory": cfg["model_factory"], "model_arguments": cfg["model_arguments"], "verified_normal_jump_seams": cfg["verified_normal_jump_seams"], "model_numeric": _strip_handles(model), "model_signature": signature, "model_sources": _sources((cfg["model_factory"], "pipe_prepare_model", "pipe_pose_prepare_model", "pipe_surface_eval", "pipe_path_curve")), "source_language": "Python", "source_validated": True, "surface_distance_validation": "deferred", "working_distance_validation": "deferred", "events_tau": events_tau, "events_u": np.unique(events_u[(events_u > 0) & (events_u < 1)]), "mapping_method": "PCHIP of original row-index pairs; not a nearest projection"}


def _restore_model(reference):
    python_source = reference.get("source_language") == "Python"
    for name, source in reference["model_sources"].items():
        if python_source:
            if _sources((name,))[name] != source:
                raise ValueError(f"PipePose:ModelVersion 模型依赖发生变化：{name}")
        elif name not in MATLAB_MODEL_HASHES or hashlib.sha256(source.encode()).hexdigest() != MATLAB_MODEL_HASHES[name]:
            raise ValueError(f"PipePose:ModelVersion 未知 MATLAB 模型来源：{name}")
    model = pipe_pose_prepare_model(_factory(reference["model_factory"], reference["model_arguments"]), reference["verified_normal_jump_seams"])
    numeric = deepcopy(reference["model_numeric"])
    if isinstance(numeric["segments"], dict):
        fields = numeric["segments"]
        numeric["segments"] = [{name: value[index] for name, value in fields.items()} for index in range(len(fields["name"]))]
    if not _equal(_strip_handles(model), numeric):
        raise ValueError("PipePose:ModelVersion 模型数值发生变化")
    _require_close(_signature(model), reference["model_signature"], 1e-6, "ModelVersion")
    return model


def _evaluate(reference, parameters):
    parameters = _parameters(parameters)
    if np.any((parameters < 0) | (parameters > 1)):
        raise ValueError("PipePose:Progress progress 必须位于 [0,1]")
    model = _restore_model(reference)
    trajectory = reference["source_snapshot"]["trajectory"]
    curve = pipe_path_curve(model, trajectory["keypointsCanvas"], trajectory["pathDefinition"])
    pp = reference["mapping_pp"]
    tau = PPoly(np.asarray(pp["coefs"]).T, np.asarray(pp["breaks"]).reshape(-1))(parameters)
    uv = curve["evaluate"](tau)[0]
    _, surface, normal, segment, local = pipe_surface_eval(model, uv, 0)
    return {"path_tau": tau, "canvas_uv": uv, "local_uv": local, "segment_id": segment, "normal_raw_W": normal, "tool_axis_raw_W": -normal, "source_surface_mm": surface}


def pipe_surface_correspondence(action: str, *arguments):
    if action == "build":
        return _build(*arguments)
    if action == "eval":
        return _evaluate(*arguments)
    if action == "pairs_csv":
        return _pairs_csv(*arguments)
    if action == "model":
        return _restore_model(*arguments)
    raise ValueError(f"PipePose:SourceAction 未知动作：{action}")
