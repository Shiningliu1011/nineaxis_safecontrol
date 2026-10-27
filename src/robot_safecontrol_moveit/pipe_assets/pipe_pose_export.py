import csv
from pathlib import Path

import numpy as np

from .mat_io import save_mat
from .pipe_pose_time_law import _normalize_plan


def pipe_pose_export(outputDir, reference: dict, plan: dict, data: dict, pose: dict, validation: dict) -> dict:
    if not validation["passed"]:
        raise ValueError("PipePose:UnverifiedExport 仅允许导出通过检查的结果")
    plan = _normalize_plan(plan)
    output = Path(outputDir)
    output.mkdir(parents=True, exist_ok=True)
    save_mat(output / "fit_reference.mat", {"fit_reference": reference["correspondence"]})
    save_mat(output / "pose_reference.mat", {"reference": reference, "validation": validation})
    save_mat(output / "pose_results.mat", {"plan": plan, "data": data, "pose": pose, "validation": validation})
    count = len(data["time"])
    result = {
        "format_version": 4, "scheme_version": "V3", "Ts": plan["Ts"], "num_points": count, "num_blocks": len(plan["na"]),
        "point_index": np.arange(1, count + 1), "time_series": data["time"], "u_series": data["u"], "position_series": data["position"], "velocity_series": data["velocity"], "speed_series": data["v"], "acceleration_series": data["acceleration"], "acceleration_norm_series": np.linalg.norm(data["acceleration"], axis=1), "jerk_series": data["jerk"], "jerk_norm_series": np.linalg.norm(data["jerk"], axis=1), "tangent_acc_cmd_series": data["a"], "tangent_jerk_cmd_series": data["j"], "tangent_acc_projection_series": data["a"][:, None] * data["tangent"], "tangent_jerk_projection_series": data["j"][:, None] * data["tangent"], "feedrate_cmd_series": data["v"], "step_feedrate_series": data["step_speed"], "block_index": data["block_index"], "boundary_point_index": data["boundary_index"], "boundary_step_index": data["boundary_index"], "chord_error_series": validation["schedule"]["position"]["chord_error_series"], "source_curve": reference["geometry"], "validation": validation, "pose": pose, "metadata": reference["metadata"], "remark": "Position 3D + smooth tool-axis 2D; roll is nominal, not a sixth hard task; not flange attitude or validated robot IK.",
    }
    for name in ("progress", "arc_mm", "path_tau", "normal_raw_W", "tool_axis_raw_W", "tool_axis_ref_W", "axis_smoothing_error", "R_WT", "dR_darc", "d2R_darc2", "quat_wxyz", "beta_W", "dbeta_darc_W", "omega_W", "alpha_W", "angular_jerk_W"):
        result[name] = pose[name]
    save_mat(output / "ik_input.mat", {"ik_input": result})
    fields = ("time_s", "progress_u", "arc_mm", "speed_mm_s", "tangent_acc_mm_s2", "axis_error_rad", "omega_rad_s", "alpha_rad_s2")
    values = np.column_stack((data["time"], data["u"], data["s"], data["v"], data["a"], pose["axis_smoothing_error"], np.linalg.norm(pose["omega_W"], axis=1), np.linalg.norm(pose["alpha_W"], axis=1)))
    with (output / "pose_diagnostics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        writer.writerows(values)
    return result
