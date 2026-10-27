from pathlib import Path

import numpy as np

from .mat_io import load_mat
from .pipe_nurbs_geometry import pipe_nurbs_geometry
from .pipe_pose_export import pipe_pose_export
from .pipe_pose_options import pipe_pose_options
from .pipe_pose_plot import pipe_pose_plot
from .pipe_pose_reference import pipe_pose_reference
from .pipe_pose_schedule import pipe_pose_schedule
from .pipe_pose_validation import pipe_pose_validation
from .pipe_surface_correspondence import pipe_surface_correspondence


def A_4_zitai_gouzao(options=None, sourceFile=None, outputDir=None, fitDir=None, offlineFile=None) -> dict:
    cfg = pipe_pose_options(options)
    fit_directory = Path.cwd() if fitDir is None else Path(fitDir)
    output = fit_directory / "pose_output" if outputDir is None else Path(outputDir)
    offline_file = fit_directory / "sudu_yilai/sudu_fenpei/nurbs_blocks.mat" if offlineFile is None else Path(offlineFile)
    geometry = pipe_nurbs_geometry("load", fit_directory)
    pairs = pipe_surface_correspondence("pairs_csv", fit_directory / "curve_data.csv")
    if sourceFile is None:
        candidates = []
        for directory in (fit_directory, fit_directory.parent, fit_directory.parent.parent):
            candidates = sorted(directory.glob("pipe_path_*.mat"))
            if candidates:
                break
        matches = []
        indices = np.asarray(pairs["input_indices"], dtype=int) - 1
        for filename in candidates:
            original = load_mat(filename)
            if "trajectory" not in original or "tcpPoints" not in original["trajectory"]:
                raise ValueError(f"PipePose:Source 缺少原始 TCP：{filename}")
            points = np.asarray(original["trajectory"]["tcpPoints"])
            if len(points) > np.max(indices) and points[indices].shape == pairs["data_points"].shape and np.max(np.abs(points[indices] - pairs["data_points"])) <= cfg["source_tolerance_mm"]:
                matches.append(filename)
        if len(matches) != 1:
            raise ValueError(f"PipePose:SourceSelection 原始关联必须唯一，当前匹配数量：{len(matches)}")
        sourceFile = matches[0]
    correspondence = pipe_surface_correspondence("build", geometry, sourceFile, pairs, cfg)
    reference = pipe_pose_reference("build", geometry, correspondence, cfg)
    geometric = pipe_pose_validation("geometry", reference, cfg)
    if not geometric["passed"]:
        raise ValueError(f"PipePose:GeometryValidation 姿态几何检查未通过：{geometric}")
    plan, data, pose, schedule = pipe_pose_schedule(reference, load_mat(offline_file), cfg)
    validation = {"geometry": geometric, "schedule": schedule, "passed": geometric["passed"] and schedule["passed"]}
    exported = pipe_pose_export(output, reference, plan, data, pose, validation)
    result = {"reference": reference, "plan": plan, "data": data, "pose": pose, "validation": validation, "ik_input": exported, "output_directory": str(output)}
    if cfg["show_figures"]:
        result["figures"] = pipe_pose_plot(data, pose)
    return result
