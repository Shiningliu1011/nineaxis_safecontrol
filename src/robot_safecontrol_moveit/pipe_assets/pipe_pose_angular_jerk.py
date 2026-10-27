import numpy as np

from .pipe_orientation_reference import pipe_orientation_reference
from .pipe_pose_math import _parameters, _polynomial_jet, pipe_pose_math
from .pipe_pose_time_law import _normalize_plan, pipe_pose_time_law


def _normalize3(z, z1, z2, z3):
    a, a1, a2, radius = pipe_pose_math("normalize", z, z1, z2)
    r1 = np.einsum("ij,ij->i", a, z1)
    r2 = np.einsum("ij,ij->i", a1, z1) + np.einsum("ij,ij->i", a, z2)
    r3 = np.einsum("ij,ij->i", a2, z1) + 2 * np.einsum("ij,ij->i", a1, z2) + np.einsum("ij,ij->i", a, z3)
    return a, a1, a2, (z3 - a * r3[:, None] - 3 * a1 * r2[:, None] - 3 * a2 * r1[:, None]) / radius[:, None]


def _jerk_side(plan, time, side):
    durations = np.column_stack([np.asarray(plan[key], dtype=float) for key in ("na", "nc", "nd")]) * float(plan["Ts"])
    middle = np.asarray(plan["stage_speed"] if "stage_speed" in plan else plan["vfs"])
    deltas = np.column_stack((middle - plan["vs"], np.zeros(len(middle)), plan["ve"] - middle)).ravel()
    durations = durations.ravel()
    starts = np.r_[0, np.cumsum(durations)]
    positive = durations > 0
    boundaries = np.r_[starts[:-1][positive], starts[-1]]
    durations, deltas = durations[positive], deltas[positive]
    time = _parameters(time).copy()
    nearest = np.argmin(np.abs(time[:, None] - boundaries), axis=1)
    snap = np.abs(time - boundaries[nearest]) <= 64 * np.spacing(max(1, starts[-1]))
    time[snap] = boundaries[nearest[snap]]
    index = np.clip(np.searchsorted(boundaries, time, side=side) - 1, 0, len(durations) - 1)
    return deltas[index] * np.pi**2 / (2 * durations[index]**2) * np.cos(np.pi * (time - boundaries[index]) / durations[index])


def _evaluate(reference, plan, time, side="right"):
    if side not in ("left", "right"):
        raise ValueError("PipePose:JerkSide side 必须为 left 或 right")
    time = _parameters(time)
    state = pipe_pose_time_law("eval", plan, time)
    arc = state["s"].copy()
    frame_breaks = np.asarray(reference["orientation"]["breaks_mm"])
    frame_time = pipe_pose_time_law("time_at_arc", plan, frame_breaks)
    nearest = np.argmin(np.abs(time[:, None] - frame_time), axis=1)
    snap = np.abs(time - frame_time[nearest]) <= 64 * np.spacing(max(1, frame_time[-1]))
    arc[snap] = frame_breaks[nearest[snap]]
    pose = pipe_orientation_reference("eval", reference["orientation"], arc, side)
    z3 = _polynomial_jet(reference["axis"]["breaks_mm"], reference["axis"]["bezier"], arc, side)[3]
    a, a1, a2, a3 = _normalize3(pose["z"], pose["dz_darc"], pose["d2z_darc2"], z3)
    h, h1, h2, h3 = _polynomial_jet(reference["orientation"]["breaks_mm"], reference["orientation"]["bezier"], arc, side)
    s0 = np.einsum("ij,ij->i", a, h)
    s1 = np.einsum("ij,ij->i", a1, h) + np.einsum("ij,ij->i", a, h1)
    s2 = np.einsum("ij,ij->i", a2, h) + 2 * np.einsum("ij,ij->i", a1, h1) + np.einsum("ij,ij->i", a, h2)
    s3 = np.einsum("ij,ij->i", a3, h) + 3 * np.einsum("ij,ij->i", a2, h1) + 3 * np.einsum("ij,ij->i", a1, h2) + np.einsum("ij,ij->i", a, h3)
    q = h - a * s0[:, None]
    q1 = h1 - a1 * s0[:, None] - a * s1[:, None]
    q2 = h2 - a2 * s0[:, None] - 2 * a1 * s1[:, None] - a * s2[:, None]
    q3 = h3 - a3 * s0[:, None] - 3 * a2 * s1[:, None] - 3 * a1 * s2[:, None] - a * s3[:, None]
    x, x1, x2, x3 = _normalize3(q, q1, q2, q3)
    y3 = np.cross(a3, x) + 3 * np.cross(a2, x1) + 3 * np.cross(a1, x2) + np.cross(a, x3)
    rotation3 = np.stack((x3, y3, a3), axis=2)
    rotation, derivative, second = (np.transpose(pose[key], (2, 0, 1)) for key in ("R_WT", "dR_darc", "d2R_darc2"))
    matrix = rotation3 @ rotation.transpose(0, 2, 1) + 2 * second @ derivative.transpose(0, 2, 1) + derivative @ second.transpose(0, 2, 1)
    matrix = (matrix - matrix.transpose(0, 2, 1)) / 2
    beta2 = np.column_stack((matrix[:, 2, 1], matrix[:, 0, 2], matrix[:, 1, 0]))
    jt = _jerk_side(plan, time, side)
    gamma = beta2 * state["v"][:, None]**3 + 3 * pose["dbeta_darc_W"] * (state["v"] * state["a"])[:, None] + pose["beta_W"] * jt[:, None]
    if np.any(~np.isfinite(gamma)):
        raise ValueError("PipePose:AngularJerk angular_jerk_W 包含非有限值")
    return {"time": time, "angular_jerk_W": gamma, "d2beta_darc2_W": beta2, "tangent_jerk": jt, "side": side, "description": "piecewise analytic d(alpha_W)/dt; internal right limit, terminal left limit; C2 does not imply continuous jerk"}


def pipe_pose_angular_jerk(action: str, reference, plan, time, side="right"):
    plan = _normalize_plan(plan)
    if action == "eval":
        return _evaluate(reference, plan, time, side)
    if action != "sample":
        raise ValueError(f"PipePose:JerkAction 未知动作：{action}")
    result = _evaluate(reference, plan, time)
    frame_time = pipe_pose_time_law("time_at_arc", plan, reference["orientation"]["breaks_mm"])
    phase_time = pipe_pose_time_law("phase_times", plan)
    nodes = np.unique(np.r_[frame_time, phase_time])
    left, right = (_evaluate(reference, plan, nodes, direction)["angular_jerk_W"] for direction in ("left", "right"))
    time = _parameters(time)
    regular = np.min(np.abs(time[:, None] - nodes), axis=1) > 64 * np.spacing(max(1, nodes[-1]))
    keys = np.vstack((np.column_stack((time[regular], np.zeros(np.sum(regular)))), np.column_stack((nodes, -np.ones(len(nodes)))), np.column_stack((nodes, np.zeros(len(nodes)))), np.column_stack((nodes, np.ones(len(nodes))))))
    values = np.vstack((result["angular_jerk_W"][regular], left, np.full((len(nodes), 3), np.nan), right))
    order = np.lexsort((keys[:, 1], keys[:, 0]))
    result.update({"break_time_s": nodes, "break_left_W": left, "break_right_W": right, "plot_time_s": keys[order, 0], "plot_values_W": values[order]})
    return result
