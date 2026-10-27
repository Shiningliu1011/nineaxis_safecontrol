import numpy as np

from .pipe_feed_profile import pipe_feed_profile
from .pipe_pose_angular_jerk import pipe_pose_angular_jerk
from .pipe_pose_reference import _same_geometry, pipe_pose_reference
from .pipe_pose_time_law import _normalize_plan, pipe_pose_time_law
from .pipe_pose_validation import pipe_pose_validation


def pipe_pose_schedule(reference: dict, offline: dict, cfg: dict):
    required = {"source_curve", "motion_plan", "limits", "validation", "arc_table"}
    if not required.issubset(offline) or not offline["validation"]["passed"]:
        raise ValueError("PipePose:Offline 需要通过检查的当前 A2 计划")
    if not _same_geometry(reference["geometry"], offline["source_curve"]):
        raise ValueError("PipePose:OfflineMismatch A2 与姿态参考的曲线不同")
    if abs(offline["arc_table"]["total_length"] - reference["arc_table"]["total_length"]) > 1e-7:
        raise ValueError("PipePose:OfflineMismatch A2 与姿态参考的弧长不同")
    original = _normalize_plan(offline["motion_plan"])
    plan, factor, history = original, 1, []
    for attempt in range(5):
        motion = pipe_pose_validation("motion", reference, plan, cfg)
        history.append([factor, motion["omega"]["measured_max"], motion["alpha"]["measured_max"]])
        if not motion["peak_converged"] or not motion["derivatives_passed"] or not motion["continuity_passed"]:
            raise ValueError("PipePose:MotionValidation 角运动极值、导数或连续性检查未通过")
        omega_factor = motion["omega"]["measured_max"] / float(np.asarray(cfg["omega_max_rad_s"]).item()) if np.size(cfg["omega_max_rad_s"]) else 1
        alpha_factor = np.sqrt(motion["alpha"]["measured_max"] / float(np.asarray(cfg["alpha_max_rad_s2"]).item())) if np.size(cfg["alpha_max_rad_s2"]) else 1
        needed = max(1, omega_factor, alpha_factor)
        if needed <= 1 + 1e-9:
            break
        if not cfg["allow_time_stretch"]:
            raise ValueError("PipePose:AngularLimit 角运动超出上限且未允许时间伸长")
        requested = factor * needed * (1 + max(0.005, 4 * cfg["peak_relative_tolerance"]))
        plan = pipe_pose_time_law("stretch", original, requested)
        factor = plan["pose_time_stretch"]
        if factor > cfg["max_time_scale"]:
            raise ValueError("PipePose:TimeScale 所需时间伸长超过配置")
        if attempt == 4:
            raise ValueError("PipePose:AngularLimit 时间伸长后仍未满足角运动上限")
    if np.size(cfg["minimum_cruise_speed_mm_s"]):
        middle = np.asarray(plan["stage_speed"] if "stage_speed" in plan else plan["vfs"])
        if np.any(middle[np.asarray(plan["nc"]) > 0] < cfg["minimum_cruise_speed_mm_s"]):
            raise ValueError("PipePose:MinimumFeed 共同调速与最低巡航进给冲突")
    data = pipe_feed_profile("sample", reference["geometry"], offline["arc_table"], plan)
    position = pipe_feed_profile("check", reference["geometry"], data, plan, offline["limits"])
    if not position["passed"]:
        raise ValueError("PipePose:PositionValidation 共同计划未满足位置运动上限")
    pose = pipe_pose_reference("eval_arclength_state", reference, data["s"], data["v"], data["a"], data["time"], data["u"])
    state = pipe_pose_time_law("eval", plan, data["time"])
    state_error = max(float(np.max(np.abs(state[key] - data[key]))) for key in ("s", "v", "a"))
    sync_error = float(np.max(np.linalg.norm(pose["position_mm"] - data["position"], axis=1)))
    if state_error > 1e-7 or sync_error > 1e-9 or np.max(np.abs(np.diff(data["time"]) - plan["Ts"])) > 1e-10:
        raise ValueError("PipePose:Synchronization 连续时间律、A3 与姿态采样不同步")
    angular = pipe_pose_angular_jerk("sample", reference, plan, data["time"])
    pose.update({"angular_jerk_W": angular["angular_jerk_W"], "angular_jerk_plot_time": angular["plot_time_s"], "angular_jerk_plot_W": angular["plot_values_W"], "angular_jerk_break_time": angular["break_time_s"], "angular_jerk_left_W": angular["break_left_W"], "angular_jerk_right_W": angular["break_right_W"], "angular_jerk_description": angular["description"]})
    plan = dict(plan, validation=position)
    duration = float(sum(np.sum(np.asarray(original[name], dtype=float)) for name in ("na", "nc", "nd")) * original["Ts"])
    audit = {"position": position, "angular": motion, "time_stretch": factor, "original_duration_s": duration, "duration_s": data["time"][-1], "time_scale_history": np.asarray(history), "fixed_period_s": plan["Ts"], "time_law_state_residual": state_error, "position_sync_residual_mm": sync_error, "minimum_cruise_speed": cfg["minimum_cruise_speed_mm_s"], "passed": motion["passed"] and position["passed"]}
    return plan, data, pose, audit
