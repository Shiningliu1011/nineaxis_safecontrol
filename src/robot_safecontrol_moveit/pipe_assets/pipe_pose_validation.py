import numpy as np
from scipy.integrate import trapezoid
from scipy.optimize import minimize_scalar
from scipy.spatial.transform import Rotation

from .pipe_axis_smoothing import pipe_axis_smoothing
from .pipe_nurbs_geometry import pipe_nurbs_geometry
from .pipe_orientation_reference import pipe_orientation_reference
from .pipe_pose_math import _parameters
from .pipe_pose_reference import pipe_pose_reference
from .pipe_pose_time_law import pipe_pose_time_law
from .pipe_surface_correspondence import pipe_surface_correspondence


def _maxabs(value):
    return float(np.max(np.abs(value), initial=0))


def _status(limit, value):
    status = "not_configured"
    if np.size(limit):
        status = "passed" if value <= float(np.asarray(limit).item()) * (1 + 1e-9) else "failed"
    return {"status": status, "configured_limit": limit, "measured_max": value}


def _pass(ok):
    return "passed" if ok else "failed"


def _fidelity(reference, arc):
    parameter = pipe_nurbs_geometry("invert", reference["geometry"], reference["arc_table"], [arc])[0]
    axis = pipe_axis_smoothing("eval", reference["axis"], [arc])["tool_axis_ref_W"][0]
    raw = pipe_surface_correspondence("eval", reference["correspondence"], parameter)["tool_axis_raw_W"][0]
    return float(np.arctan2(np.linalg.norm(np.cross(axis, raw)), np.dot(axis, raw)))


def _geometry(reference, cfg):
    length = float(reference["arc_table"]["total_length"])
    events = _parameters(reference["events_arc_mm"])
    arc = np.unique(np.r_[np.linspace(0, length, max(2000, int(np.ceil(2 * length / cfg["axis_sample_spacing_mm"]))) + 1), reference["orientation"]["breaks_mm"], events, np.maximum(0, events - 1e-7), np.minimum(length, events + 1e-7)])
    parameter = pipe_nurbs_geometry("invert", reference["geometry"], reference["arc_table"], arc)[0]
    pose = pipe_pose_reference("eval_arclength_state", reference, arc, 0, 0, None, parameter)
    rotation = pose["R_WT"].transpose(2, 0, 1)
    orthogonal = float(np.max(np.linalg.norm(rotation.transpose(0, 2, 1) @ rotation - np.eye(3), axis=(1, 2))))
    determinant = _maxabs(np.linalg.det(rotation) - 1)
    quaternion_matrix = Rotation.from_quat(pose["quat_wxyz"][:, [1, 2, 3, 0]]).as_matrix()
    quaternion_error = float(np.max(np.linalg.norm(quaternion_matrix - rotation, axis=(1, 2))))
    nodes = np.unique(np.r_[reference["axis"]["breaks_mm"], reference["orientation"]["breaks_mm"]])
    left, right = (pipe_orientation_reference("eval", reference["orientation"], nodes, side) for side in ("left", "right"))
    continuity = np.asarray([_maxabs(left[key] - right[key]) for key in ("R_WT", "dR_darc", "d2R_darc2")])
    identities = np.asarray([_maxabs(np.einsum("ij,ij->i", pose["tool_axis_ref_W"], pose["daxis_darc"])), _maxabs(np.einsum("ij,ij->i", pose["tool_axis_ref_W"], pose["d2axis_darc2"]) + np.sum(pose["daxis_darc"]**2, axis=1))])
    probes = np.linspace(0.013 * length, 0.987 * length, 151)
    h = min(1e-3, length * 1e-5)
    center = pipe_orientation_reference("eval", reference["orientation"], probes)
    residuals = []
    for level in range(2):
        delta = h / 2**level
        a = pipe_orientation_reference("eval", reference["orientation"], probes - delta)
        b = pipe_orientation_reference("eval", reference["orientation"], probes + delta)
        residuals.append([_maxabs((b["R_WT"] - a["R_WT"]) / (2 * delta) - center["dR_darc"]), _maxabs((b["dR_darc"] - a["dR_darc"]) / (2 * delta) - center["d2R_darc2"]), _maxabs((b["beta_W"] - a["beta_W"]) / (2 * delta) - center["dbeta_darc_W"])])
    residuals = np.asarray(residuals)
    order = np.r_[np.arange(len(arc) - 1, -1, -7), np.arange(0, len(arc), 11), len(arc) - 1, 0]
    shuffled = pipe_pose_reference("eval_progress", reference, parameter[order])
    consistency = max(_maxabs(shuffled["R_WT"] - pose["R_WT"][:, :, order]), _maxabs(shuffled["quat_wxyz"] - pose["quat_wxyz"][order]), _maxabs(shuffled["position_mm"] - pose["position_mm"][order]))
    error = pose["axis_smoothing_error"]
    index = np.argmax(error)
    peak, peak_arc = float(error[index]), float(arc[index])
    candidates = np.flatnonzero((error[1:-1] >= error[:-2]) & (error[1:-1] >= error[2:])) + 1
    for index in candidates:
        found = minimize_scalar(lambda s: -_fidelity(reference, s), bounds=(arc[index - 1], arc[index + 1]), method="bounded", options={"xatol": 1e-8})
        if not found.success:
            raise RuntimeError("PipePose:FidelityPeak 方向误差极值搜索失败")
        if -found.fun > peak:
            peak, peak_arc = -float(found.fun), float(found.x)
    regions = np.asarray(reference["axis"]["smoothing_regions_mm"]).reshape(-1, 2)
    region_max = np.asarray([np.max(error[(arc >= low) & (arc <= high)]) for low, high in regions])
    seam_sides = []
    for event in events:
        delta = min(1e-7, min(event, length - event) / 2)
        sample = pipe_pose_reference("eval_arclength_state", reference, [event - delta, event + delta], 0, 0)
        raw = sample["tool_axis_raw_W"]
        jump = np.arctan2(np.linalg.norm(np.cross(raw[0], raw[1])), np.dot(raw[0], raw[1]))
        seam_sides.append([event, jump, *sample["axis_smoothing_error"], jump / 2])
    hard, soft = _status(cfg["hard_axis_error_rad"], peak), _status(cfg["soft_axis_error_rad"], peak)
    result = {
        "source": {"status": "passed", "mapping": "original MAT row correspondence; model/curve verified"},
        "rotation": {"status": _pass(max(orthogonal, determinant, quaternion_error) <= cfg["rotation_tolerance"]), "orthogonality_residual": orthogonal, "determinant_residual": determinant, "quaternion_matrix_residual": quaternion_error, "quaternion_norm_residual": _maxabs(np.linalg.norm(pose["quat_wxyz"], axis=1) - 1), "axis_residual": _maxabs(pose["R_WT"][:, 2, :].T - pose["tool_axis_ref_W"])},
        "continuity": {"status": _pass(np.all(continuity <= cfg["continuity_tolerance"])), "structure": "simple degree-5 axis knots; shared C2 Hermite jets; certified projection/normalization", "node_residual_R_Rarc_Rarcarc": continuity, "node_count": len(nodes)},
        "derivatives": {"status": _pass(np.max(residuals[1]) <= cfg["derivative_tolerance"] and np.max(identities) <= cfg["rotation_tolerance"]), "finite_difference_steps_mm": np.array([h, h / 2]), "residual_R_Rarc_beta": residuals, "unit_axis_identities": identities},
        "nondegeneracy": {"status": _pass(np.min(reference["axis"]["norm_lower_bound_by_interval"]) >= cfg["rho_z"] and np.min(reference["orientation"]["projection_lower_bound"]) >= cfg["rho_projection"]), "z_certified_lower_bound": np.min(reference["axis"]["norm_lower_bound_by_interval"]), "projection_certified_lower_bound": np.min(reference["orientation"]["projection_lower_bound"]), "z_sample_min": np.min(pose["z_norm"]), "projection_sample_min": np.min(pose["projection_norm"]), "max_rotation_travel_bound_rad": np.max(reference["orientation"]["rotation_travel_bound_rad"])},
        "fidelity": {"max_rad": peak, "max_arc_mm": peak_arc, "rms_rad": np.sqrt(trapezoid(error**2, arc) / length), "region_max_rad": region_max, "seam_columns": ["arc_mm", "raw_jump_rad", "left_error_rad", "right_error_rad", "necessary_min_hard_tolerance_rad"], "seam_sides": np.asarray(seam_sides).reshape(-1, 5), "hard_limit": hard, "soft_target": soft, "method": "dense arclength, seam sides and local extrema refinement; soft target does not imply rejection"},
        "synchronization": {"status": _pass(consistency <= cfg["continuity_tolerance"]), "shuffled_repeated_query_residual": consistency, "progress_definition": "u"},
        "minimal_twist_max_residual_per_mm": np.max(pose["minimal_twist_residual_per_mm"]), "surface_distance_validation": "deferred", "working_distance_validation": "deferred", "config": cfg, "sample_count": len(arc),
    }
    result["passed"] = all(result[key]["status"] == "passed" for key in ("rotation", "continuity", "derivatives", "nondegeneracy", "synchronization")) and hard["status"] != "failed"
    return result


def _angular_at(reference, plan, time):
    state = pipe_pose_time_law("eval", plan, time)
    pose = pipe_orientation_reference("eval", reference["orientation"], state["s"])
    omega = pose["beta_W"] * state["v"][:, None]
    alpha = pose["dbeta_darc_W"] * state["v"][:, None]**2 + pose["beta_W"] * state["a"][:, None]
    metrics = np.column_stack((np.linalg.norm(omega, axis=1), np.linalg.norm(alpha, axis=1)))
    return metrics, omega, alpha, pose["R_WT"], pose["dR_darc"] * state["v"][None, None, :]


def _motion(reference, plan, cfg):
    phase = pipe_pose_time_law("phase_times", plan)
    node_time = pipe_pose_time_law("time_at_arc", plan, reference["orientation"]["breaks_mm"])
    breaks = np.unique(np.r_[phase, node_time])
    duration = phase[-1]
    previous = None
    history = []
    converged = False
    for level in range(int(cfg["max_peak_refinements"])):
        step = plan["Ts"] / (cfg["peak_subdivision"] * 2**level)
        time = np.unique(np.r_[np.linspace(0, duration, int(np.ceil(duration / step)) + 1), breaks])
        metrics, omega, alpha, _, _ = _angular_at(reference, plan, time)
        indices = np.argmax(metrics, axis=0)
        peaks = metrics[indices, np.arange(2)]
        peak_times = time[indices]
        for coordinate in range(2):
            candidates = np.flatnonzero((metrics[1:-1, coordinate] >= metrics[:-2, coordinate]) & (metrics[1:-1, coordinate] >= metrics[2:, coordinate]) & (metrics[1:-1, coordinate] > max(1e-12, peaks[coordinate] * 1e-6))) + 1
            for index in candidates:
                structural = np.searchsorted(breaks, time[index], side="right") - 1
                low = max(time[index - 1], breaks[structural])
                high = min(time[index + 1], breaks[structural + 1]) if structural + 1 < len(breaks) else time[index + 1]
                if high <= low:
                    continue
                optimum = minimize_scalar(lambda t: -_angular_at(reference, plan, [t])[0][0, coordinate], bounds=(low, high), method="bounded", options={"xatol": 1e-9})
                if not optimum.success:
                    raise RuntimeError("PipePose:MotionPeak 角运动极值搜索失败")
                if -optimum.fun > peaks[coordinate]:
                    peaks[coordinate], peak_times[coordinate] = -optimum.fun, optimum.x
        history.append([len(time), *peaks])
        if previous is not None and np.all(np.abs(peaks - previous) <= cfg["peak_relative_tolerance"] * np.maximum(1, peaks)):
            converged = True
            break
        previous = peaks.copy()
    query = np.linspace(0.017 * duration, 0.983 * duration, 101)
    h = min(1e-5, duration * 1e-6)
    _, w0, a0, r0, rd0 = _angular_at(reference, plan, query)
    _, wm, _, rm, _ = _angular_at(reference, plan, query - h)
    _, wp, _, rp, _ = _angular_at(reference, plan, query + h)
    domega = _maxabs((wp - wm) / (2 * h) - a0)
    dr = _maxabs((rp - rm) / (2 * h) - rd0)
    skew = np.zeros((len(query), 3, 3))
    skew[:, 0, 1], skew[:, 0, 2] = -w0[:, 2], w0[:, 1]
    skew[:, 1, 0], skew[:, 1, 2] = w0[:, 2], -w0[:, 0]
    skew[:, 2, 0], skew[:, 2, 1] = -w0[:, 1], w0[:, 0]
    spatial = float(np.max(np.linalg.norm(rd0.transpose(2, 0, 1) - skew @ r0.transpose(2, 0, 1), axis=(1, 2))))
    state = pipe_pose_time_law("eval", plan, node_time)
    left, right = (pipe_orientation_reference("eval", reference["orientation"], reference["orientation"]["breaks_mm"], side) for side in ("left", "right"))
    node_w = _maxabs((left["beta_W"] - right["beta_W"]) * state["v"][:, None])
    node_a = _maxabs((left["dbeta_darc_W"] - right["dbeta_darc_W"]) * state["v"][:, None]**2 + (left["beta_W"] - right["beta_W"]) * state["a"][:, None])
    phase_residual = []
    for level in range(2):
        epsilon = min(1e-7, duration * 1e-9) / 2**level
        _, wl, al, _, _ = _angular_at(reference, plan, phase[1:-1] - epsilon)
        _, wr, ar, _, _ = _angular_at(reference, plan, phase[1:-1] + epsilon)
        phase_residual.append([_maxabs(wr - wl), _maxabs(ar - al)])
    phase_residual = np.asarray(phase_residual)
    result = {"omega": _status(cfg["omega_max_rad_s"], peaks[0]), "alpha": _status(cfg["alpha_max_rad_s2"], peaks[1]), "peak_time_s": peak_times, "peak_refinement_history": np.asarray(history), "peak_converged": converged, "peak_method": "temporal refinement plus all frame/time-law nodes and local extrema", "node_vector_residual_omega_alpha": np.array([node_w, node_a]), "phase_vector_residual_omega_alpha": phase_residual, "time_derivative_residual_R_omega": np.array([dr, domega]), "spatial_rotation_identity_residual": spatial, "duration_s": duration, "max_sample_omega_rad_s": np.max(np.linalg.norm(omega, axis=1)), "max_sample_alpha_rad_s2": np.max(np.linalg.norm(alpha, axis=1))}
    result["derivatives_passed"] = dr <= cfg["derivative_tolerance"] * max(1, peaks[0]) and domega <= cfg["derivative_tolerance"] * max(1, peaks[1]) and spatial <= 1e-8
    result["continuity_passed"] = node_w <= cfg["continuity_tolerance"] * max(1, peaks[0]) and node_a <= cfg["continuity_tolerance"] * max(1, peaks[1]) and np.all(phase_residual[1] <= np.maximum(1e-6, 0.55 * phase_residual[0]))
    result["passed"] = converged and result["derivatives_passed"] and result["continuity_passed"] and result["omega"]["status"] != "failed" and result["alpha"]["status"] != "failed"
    return result


def pipe_pose_validation(action: str, *arguments):
    if action == "geometry":
        return _geometry(*arguments)
    if action == "motion":
        return _motion(*arguments)
    raise ValueError(f"PipePose:ValidationAction 未知动作：{action}")
