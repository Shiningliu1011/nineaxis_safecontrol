import numpy as np


def pipe_pose_options(overrides=None) -> dict:
    cfg = {
        "axis_knot_spacing_mm": 8, "axis_sample_spacing_mm": 0.5,
        "lambda2_mm4": 4, "lambda3_mm6": 4, "rho_z": 0.15, "rho_projection": 0.25,
        "normal_weight": 1, "region_weight": 0.15, "region_halfwidth_mm": 8,
        "direction_rate_threshold_rad_per_mm": 0.08, "smoothing_regions_mm": np.empty((0, 2)),
        "axis_breaks_mm": np.empty(0), "soft_axis_error_rad": np.empty(0), "hard_axis_error_rad": np.empty(0),
        "omega_max_rad_s": np.empty(0), "alpha_max_rad_s2": np.empty(0), "minimum_cruise_speed_mm_s": np.empty(0),
        "initial_x_W": np.empty(0), "frame_step_mm": 2, "ode_rel_tol": 1e-10, "ode_abs_tol": 1e-12,
        "max_frame_refinements": 10, "source_tolerance_mm": 1e-6, "normal_tolerance": 1e-7,
        "rotation_tolerance": 2e-9, "continuity_tolerance": 2e-8, "derivative_tolerance": 2e-5,
        "constraint_guard": 1e-8, "peak_subdivision": 4, "peak_relative_tolerance": 2e-4,
        "max_peak_refinements": 4, "max_time_scale": 100, "allow_time_stretch": True,
        "verified_normal_jump_seams": np.empty(0, dtype=int), "model_factory": "pipe_demo_model", "model_arguments": [], "show_figures": True,
    }
    if overrides is not None:
        if not isinstance(overrides, dict) or set(overrides) - set(cfg):
            raise ValueError(f"PipePose:Options 未知配置字段：{set(overrides) - set(cfg) if isinstance(overrides, dict) else type(overrides)}")
        cfg.update(overrides)
    positive = ("axis_knot_spacing_mm", "axis_sample_spacing_mm", "rho_z", "rho_projection", "normal_weight", "region_weight", "region_halfwidth_mm", "direction_rate_threshold_rad_per_mm", "frame_step_mm", "ode_rel_tol", "ode_abs_tol", "source_tolerance_mm", "normal_tolerance", "rotation_tolerance", "continuity_tolerance", "derivative_tolerance", "constraint_guard", "peak_relative_tolerance", "max_time_scale")
    for name in positive + ("lambda2_mm4", "lambda3_mm6", "max_frame_refinements", "max_peak_refinements", "peak_subdivision"):
        value = np.asarray(cfg[name])
        if value.size != 1 or not np.isrealobj(value) or not np.all(np.isfinite(value)) or float(value.item()) < (0 if name.startswith("lambda") else np.finfo(float).tiny):
            raise ValueError(f"PipePose:Options {name} 必须为有限标量并满足范围")
        if name in ("max_frame_refinements", "max_peak_refinements", "peak_subdivision") and float(value.item()) != int(value.item()):
            raise ValueError(f"PipePose:Options {name} 必须为正整数")
    for name in ("soft_axis_error_rad", "hard_axis_error_rad", "omega_max_rad_s", "alpha_max_rad_s2", "minimum_cruise_speed_mm_s"):
        value = np.asarray(cfg[name])
        if value.size and (value.size != 1 or not np.isrealobj(value) or not np.all(np.isfinite(value)) or value.item() <= 0):
            raise ValueError(f"PipePose:Options {name} 必须为空或有限正标量")
    rho_z = float(np.asarray(cfg["rho_z"]).item())
    rho_projection = float(np.asarray(cfg["rho_projection"]).item())
    max_time_scale = float(np.asarray(cfg["max_time_scale"]).item())
    max_peak_refinements = int(np.asarray(cfg["max_peak_refinements"]).item())
    if rho_z >= 1 or rho_projection >= 1 or max_time_scale < 1 or max_peak_refinements < 2:
        raise ValueError("PipePose:Options 范数下界、时间倍数或极值加密次数无效")
    for name in ("show_figures", "allow_time_stretch"):
        if np.asarray(cfg[name]).size != 1 or cfg[name] not in (True, False, 0, 1):
            raise ValueError(f"PipePose:Options {name} 必须为布尔值")
    initial = np.asarray(cfg["initial_x_W"], dtype=float)
    if initial.size and (initial.size != 3 or np.any(~np.isfinite(initial))):
        raise ValueError("PipePose:Options initial_x_W 需要有限三维向量")
    regions = np.asarray(cfg["smoothing_regions_mm"], dtype=float)
    if regions.size == 0:
        regions = np.empty((0, 2))
    if regions.ndim != 2 or regions.shape[1] != 2 or np.any(~np.isfinite(regions)) or np.any(regions[:, 1] <= regions[:, 0]):
        raise ValueError("PipePose:Options smoothing_regions_mm 需要递增区间 N×2")
    cfg["smoothing_regions_mm"] = regions
    seams = np.asarray(cfg["verified_normal_jump_seams"], dtype=float).reshape(-1)
    if np.any(~np.isfinite(seams)) or np.any(seams < 1) or np.any(seams != np.round(seams)):
        raise ValueError("PipePose:Options verified_normal_jump_seams 需要正整数编号")
    cfg["verified_normal_jump_seams"] = seams.astype(int)
    return cfg
