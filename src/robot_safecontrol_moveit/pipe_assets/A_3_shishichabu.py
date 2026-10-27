from pathlib import Path

import numpy as np

from .mat_io import load_mat, save_mat
from .pipe_feed_profile import pipe_feed_profile
from .pipe_nurbs_geometry import pipe_nurbs_geometry


def A_3_shishichabu(plan_file, output_dir):
    offline = load_mat(plan_file)
    if (offline.get("format_version") not in (2, 3)
            or not all(name in offline for name in ("source_curve", "motion_plan", "arc_table", "limits", "validation"))):
        raise ValueError("PipeMotion:StaleOffline：离线计划格式或必需字段无效")
    if not offline["validation"]["passed"]:
        raise ValueError("PipeMotion:UnverifiedOffline：离线计划没有通过验证")
    geo, plan = offline["source_curve"], offline["motion_plan"]
    current = pipe_nurbs_geometry("load", Path(geo["control_file"]).parent)
    if any(not np.array_equal(np.asarray(current[name]), np.asarray(geo[name])) for name in ("ctrl_pts", "knots", "w", "p")):
        raise ValueError("PipeMotion:CurveMismatch：当前控制点和节点与离线计划不一致")
    data = pipe_feed_profile("sample", geo, offline["arc_table"], plan)
    validation = pipe_feed_profile("check", geo, data, plan, offline["limits"])
    if not validation["passed"]:
        raise RuntimeError("PipeMotion:ValidationFailed：插补结果没有通过全部约束")
    n, block_count = len(data["u"]), len(np.atleast_1d(plan["na"]))
    result = {"Ts": plan["Ts"], "num_points": n, "num_blocks": block_count, "point_index": np.arange(1, n + 1),
              "time_series": data["time"], "u_series": data["u"], "position_series": data["position"],
              "block_index": data["block_index"], "block_step_index": data["block_step_index"],
              "boundary_point_index": data["boundary_index"], "boundary_step_index": data["boundary_index"],
              "speed_series": data["v"], "feedrate_cmd_series": data["v"], "step_feedrate_series": data["step_speed"],
              "actual_feedrate_series": np.r_[0., data["step_speed"]], "velocity_series": data["velocity"],
              "acceleration_series": data["acceleration"], "jerk_series": data["jerk"],
              "acceleration_norm_series": np.linalg.norm(data["acceleration"], axis=1),
              "jerk_norm_series": np.linalg.norm(data["jerk"], axis=1), "tangent_acc_cmd_series": data["a"],
              "tangent_jerk_cmd_series": data["j"], "tangent_acc_projection_series": data["a"][:, None] * data["tangent"],
              "tangent_jerk_projection_series": data["j"][:, None] * data["tangent"],
              "chord_error_series": validation["chord_error_series"], "source_curve": geo, "validation": validation}
    debug = []
    for k in range(block_count):
        i0, i1 = data["boundary_index"][k:k + 2] - 1
        debug.append({"block_id": k + 1, "u_start": data["u"][i0], "u_end": data["u"][i1],
                      "planned_steps": i1 - i0, "actual_new_points": i1 - i0, "exit_reason": "exact_arc_length_end",
                      "time_start": data["time"][i0], "time_end": data["time"][i1], "u_last": data["u"][i1],
                      "position_error_mm": validation["block_position_error_mm"][k + 1]})
    result["block_debug"] = debug
    ik_input = {name: value for name, value in result.items() if name not in ("block_step_index", "actual_feedrate_series")}
    ik_input["remark"] = "位置型反解输入，工具轴与连续姿态由姿态模块生成。"
    ik_input["field_description"] = {
        "Ts": "固定采样周期，s", "num_points": "点数", "num_blocks": "分段数量", "point_index": "采样点序号",
        "time_series": "采样时刻，s", "u_series": "NURBS 参数 u", "position_series": "XYZ 位置，mm",
        "block_index": "所属分段", "velocity_series": "XYZ 瞬时速度，mm/s", "speed_series": "瞬时进给速度，mm/s",
        "acceleration_series": "完整 XYZ 加速度，mm/s²", "acceleration_norm_series": "完整加速度模长，mm/s²",
        "jerk_series": "完整 XYZ jerk，mm/s³", "jerk_norm_series": "完整 jerk 模长，mm/s³",
        "tangent_acc_cmd_series": "进给时间律切向加速度，mm/s²", "tangent_acc_projection_series": "切向加速度的 XYZ 投影",
        "tangent_jerk_cmd_series": "进给速度二阶时间导数，mm/s³", "tangent_jerk_projection_series": "切向标量 jerk 的 XYZ 投影",
        "feedrate_cmd_series": "完整已验证进给命令，mm/s", "step_feedrate_series": "每周期弦长除以 Ts，长度为点数减一，mm/s",
        "boundary_point_index": "每块准确起止点索引", "boundary_step_index": "边界点索引",
        "chord_error_series": "每周期七个内点到线段的最大距离，mm", "source_curve": "控制点、节点与来源路径",
        "validation": "完整运动验证指标", "block_debug": "分块执行状态", "remark": "数据适用范围", "field_description": "字段说明",
    }
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    save_mat(output / "realtime_interpolation_results.mat", result)
    save_mat(output / "ik_input.mat", {"ik_input": ik_input})
    return result
