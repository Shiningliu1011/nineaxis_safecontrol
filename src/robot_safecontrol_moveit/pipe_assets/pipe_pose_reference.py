import hashlib

import numpy as np

from .mat_io import load_mat
from .pipe_axis_smoothing import _normalize_axis, pipe_axis_smoothing
from .pipe_nurbs_geometry import pipe_nurbs_geometry
from .pipe_orientation_reference import (
    _normalize_orientation,
    pipe_orientation_reference,
)
from .pipe_pose_math import _parameters, pipe_pose_math
from .pipe_surface_correspondence import _equal, _sources, pipe_surface_correspondence

MATLAB_IMPLEMENTATION_HASHES = {
    "pipe_pose_reference": "41cef99524dd6fe3ba828a98df13659d84f41784068c8d623048ea9b7de49c8e",
    "pipe_pose_math": "d5ed276ca06ce3108f9161adfce6b02a41dfd405cf54bb684553cfdb0ce3313a",
    "pipe_axis_smoothing": "06820c9a4f7b48aca63e6ac70c23118d163a5134328a3b5ca3ffd8b39cac0b45",
    "pipe_orientation_reference": "118935bdb4e8af2a541ba5a566245c4176bec0c4397d908a08d087bdde058e71",
    "pipe_surface_correspondence": "703cf7ac7d81f9e0464112786e06616138c5e846c5c9d0cfde36ea4f340326c2",
    "pipe_pose_options": "dfb6ed71a7c47417a07f17f506c1ee0fd2f4d8d45936677122bd855d0447d291",
    "pipe_pose_time_law": "c2e996a2641bdacc0180efa4a1df9ecc1fadc7917a12967b661ffde4d74b09d7",
    "pipe_pose_validation": "e597cb709dbed44016195fc4db9320a6c39fceaca456997037c4ea3838458b83",
    "pipe_pose_schedule": "56ba3854cf0d42ce19c84da6ee813907eb9a206613b9d6e4910c109715d8c699",
    "pipe_pose_angular_jerk": "d8236ba6abbc48718d563171916460254a1239a7b217b602ac7132d475966511",
}


def _same_geometry(first, second):
    return all(_equal(first[name], second[name]) for name in ("ctrl_pts", "knots", "w", "p"))


def _stationary_check(geometry):
    breaks = np.unique(geometry["knots"])
    for low, high in zip(breaks[:-1], breaks[1:]):
        middle, width = (low + high) / 2, (high - low) / 2
        _, d1, d2, d3 = pipe_nurbs_geometry("eval", geometry, [middle])
        squared = np.polynomial.Polynomial([0.0])
        for coordinate in range(3):
            component = np.polynomial.Polynomial([d1[0, coordinate], width * d2[0, coordinate], width**2 * d3[0, coordinate] / 2])
            squared += component * component
        roots = squared.deriv().roots()
        roots = roots[(np.abs(roots.imag) < 1e-9) & (np.abs(roots.real) < 1)].real
        tangent = pipe_nurbs_geometry("eval", geometry, middle + width * np.r_[-1, roots, 1])[1]
        if np.any(np.linalg.norm(tangent, axis=1) < 1e-10):
            raise ValueError("PipePose:StationaryCurve 曲线包含驻点")


def _build(geometry, correspondence, cfg):
    if not _same_geometry(geometry, correspondence["geometry"]) or not correspondence["source_validated"]:
        raise ValueError("PipePose:SourceMismatch 曲线与原始关联不同")
    _stationary_check(geometry)
    arc = pipe_nurbs_geometry("arc", geometry, [0, 1])
    length = arc["total_length"]
    events = pipe_pose_math("arc", geometry, arc, correspondence["events_u"])
    epsilon = min(1e-6, length * 1e-8)
    query = np.unique(np.r_[np.linspace(0, length, max(4, int(np.ceil(length / cfg["axis_sample_spacing_mm"]))) + 1), np.maximum(0, events - epsilon), events, np.minimum(length, events + epsilon)])
    parameters = pipe_nurbs_geometry("invert", geometry, arc, query)[0]
    raw = pipe_surface_correspondence("eval", correspondence, parameters)
    axis = pipe_axis_smoothing("build", query, raw["tool_axis_raw_W"], cfg, events)
    tangent = pipe_nurbs_geometry("eval", geometry, [0])[1][0]
    orientation = pipe_orientation_reference("build", axis, tangent, cfg)
    names = ("pipe_pose_reference", "pipe_pose_math", "pipe_axis_smoothing", "pipe_orientation_reference", "pipe_surface_correspondence", "pipe_pose_options", "pipe_pose_time_law", "pipe_pose_validation", "pipe_pose_schedule", "pipe_pose_angular_jerk")
    return {"format_version": 1, "scheme_version": "V3", "implementation_language": "Python", "geometry": geometry, "arc_table": arc, "correspondence": correspondence, "axis": axis, "orientation": orientation, "config": cfg, "events_arc_mm": events, "metadata": {"axis_policy": "normal_soft_follow", "progress_definition": "u", "smoothing_coordinate": "final_curve_arclength_mm", "task_definition": "position_3d_plus_smoothed_axis_2d", "roll_is_hard_task": False, "rotation_convention": "R_WT maps tool coordinates to workpiece; tool +Z is reference axis", "quaternion_convention": "Hamilton wxyz; active tool-to-workpiece", "surface_distance_validation": "deferred", "working_distance_validation": "deferred", "periodic_roll_closure": "not_requested_not_verified", "algorithm_defaults": "numerical choices; physical limits require configuration"}, "implementation_sources": _sources(names)}


def _assemble(reference, arc, parameters):
    pose = pipe_orientation_reference("eval", reference["orientation"], arc)
    pose.update(pipe_surface_correspondence("eval", reference["correspondence"], parameters))
    position, d1, d2, d3 = pipe_nurbs_geometry("eval", reference["geometry"], parameters)
    q = np.linalg.norm(d1, axis=1)
    if np.any(q < 1e-10):
        raise ValueError("PipePose:StationaryCurve 曲线包含驻点")
    qu = np.einsum("ij,ij->i", d1, d2) / q
    pose.update({"progress": parameters, "u": parameters, "arc_mm": arc, "position_mm": position, "dposition_du": d1, "d2position_du2": d2, "d3position_du3": d3, "dposition_darc": d1 / q[:, None], "d2position_darc2": d2 / q[:, None]**2 - d1 * (qu / q**3)[:, None], "position_parameter_speed": q, "position_parameter_acc": qu, "daxis_du": pose["daxis_darc"] * q[:, None], "d2axis_du2": pose["d2axis_darc2"] * q[:, None]**2 + pose["daxis_darc"] * qu[:, None]})
    pose["axis_smoothing_error"] = np.arctan2(np.linalg.norm(np.cross(pose["tool_axis_ref_W"], pose["tool_axis_raw_W"]), axis=1), np.einsum("ij,ij->i", pose["tool_axis_ref_W"], pose["tool_axis_raw_W"]))
    return pose


def _expand(value, count, label):
    value = _parameters(value)
    if len(value) == 1:
        value = np.full(count, value[0])
    if len(value) != count:
        raise ValueError(f"PipePose:TimeState {label} 数量与查询点不同")
    return value


def _eval_progress(reference, parameters):
    parameters = _parameters(parameters)
    arc = pipe_pose_math("arc", reference["geometry"], reference["arc_table"], parameters)
    return _assemble(reference, arc, parameters)


def _eval_state(reference, arc, velocity, acceleration, time=None, parameters=None):
    arc = _parameters(arc)
    velocity = _expand(velocity, len(arc), "speed")
    acceleration = _expand(acceleration, len(arc), "acceleration")
    if np.any(velocity < 0):
        raise ValueError("PipePose:TimeState 弧长速度需要非负")
    if parameters is None or not np.size(parameters):
        parameters = pipe_nurbs_geometry("invert", reference["geometry"], reference["arc_table"], arc)[0]
    else:
        parameters = _parameters(parameters)
        if len(parameters) != len(arc) or np.max(np.abs(pipe_pose_math("arc", reference["geometry"], reference["arc_table"], parameters) - arc), initial=0) > 1e-7:
            raise ValueError("PipePose:Synchronization progress 与 arc_mm 不对应")
    pose = _assemble(reference, arc, parameters)
    pose.update({"time": np.full(len(arc), np.nan) if time is None or not np.size(time) else _expand(time, len(arc), "time"), "arc_speed": velocity, "tangent_acc": acceleration, "omega_W": pose["beta_W"] * velocity[:, None], "alpha_W": pose["dbeta_darc_W"] * velocity[:, None]**2 + pose["beta_W"] * acceleration[:, None], "velocity_mm_s": pose["dposition_darc"] * velocity[:, None], "acceleration_mm_s2": pose["d2position_darc2"] * velocity[:, None]**2 + pose["dposition_darc"] * acceleration[:, None], "progress_speed": velocity / pose["position_parameter_speed"], "progress_acc": acceleration / pose["position_parameter_speed"] - velocity**2 * pose["position_parameter_acc"] / pose["position_parameter_speed"]**3})
    return pose


def _load(filename, expectedGeometry=None, expectedConfig=None):
    saved = load_mat(filename)
    if "reference" not in saved:
        raise ValueError("PipePose:ReferenceFile 缺少 reference")
    reference = saved["reference"]
    if reference["format_version"] != 1 or reference["scheme_version"] != "V3":
        raise ValueError("PipePose:ReferenceVersion 未知版本")
    reference["axis"] = _normalize_axis(reference["axis"])
    reference["orientation"] = _normalize_orientation(reference["orientation"])
    python_source = reference.get("implementation_language") == "Python"
    for name, source in reference["implementation_sources"].items():
        if python_source:
            if _sources((name,))[name] != source:
                raise ValueError(f"PipePose:ReferenceVersion Python 源码发生变化：{name}")
        elif name not in MATLAB_IMPLEMENTATION_HASHES or hashlib.sha256(source.encode()).hexdigest() != MATLAB_IMPLEMENTATION_HASHES[name]:
            raise ValueError(f"PipePose:ReferenceVersion 未知 MATLAB 参考来源：{name}")
    pipe_surface_correspondence("model", reference["correspondence"])
    if expectedGeometry is not None and not _same_geometry(reference["geometry"], expectedGeometry):
        raise ValueError("PipePose:ReferenceIdentity 曲线身份不同")
    if expectedConfig is not None and not _equal(reference["config"], expectedConfig):
        raise ValueError("PipePose:ReferenceIdentity 配置不同")
    return reference


def pipe_pose_reference(action: str, *arguments):
    if action == "build":
        return _build(*arguments)
    if action == "load":
        return _load(*arguments)
    if action == "eval_progress":
        return _eval_progress(*arguments)
    if action == "eval_arclength_state":
        return _eval_state(*arguments)
    raise ValueError(f"PipePose:Action 未知动作：{action}")
