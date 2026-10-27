import time

import numpy as np

from .pipe_joint_schedule import JointScheduleError, pipe_joint_schedule
from .pipe_motion_trace import pipe_motion_trace
from .pipe_nurbs_geometry import pipe_nurbs_geometry


def _stage_speed(plan):
    return np.asarray(plan.get("stage_speed", plan["vfs"]), dtype=float).ravel()


def _sine_integral(t, Ta, Tc, Td, Vs, Vp, Ve):
    t = np.asarray(t, dtype=float)
    Sa = (Vs + Vp) * Ta / 2
    s = Sa + Vp * (t - Ta)
    v, a, j = np.full_like(t, Vp), np.zeros_like(t), np.zeros_like(t)
    if Ta > 0:
        indices = t <= Ta
        x, delta = t[indices], Vp - Vs
        s[indices] = Vs * x + delta * (x - Ta / np.pi * np.sin(np.pi * x / Ta)) / 2
        v[indices] = Vs + delta * (1 - np.cos(np.pi * x / Ta)) / 2
        a[indices] = delta * np.pi / (2 * Ta) * np.sin(np.pi * x / Ta)
        j[indices] = delta * np.pi ** 2 / (2 * Ta ** 2) * np.cos(np.pi * x / Ta)
    if Td > 0:
        indices = t > Ta + Tc if Ta + Tc else t >= 0
        x, delta = t[indices] - Ta - Tc, Ve - Vp
        s[indices] = Sa + Vp * Tc + Vp * x + delta * (x - Td / np.pi * np.sin(np.pi * x / Td)) / 2
        v[indices] = Vp + delta * (1 - np.cos(np.pi * x / Td)) / 2
        a[indices] = delta * np.pi / (2 * Td) * np.sin(np.pi * x / Td)
        j[indices] = delta * np.pi ** 2 / (2 * Td ** 2) * np.cos(np.pi * x / Td)
    return s, v, a, j


def _sample(geo, arc, plan):
    started = time.monotonic()
    na, nc, nd = [np.asarray(plan[name], dtype=float).ravel() for name in ("na", "nc", "nd")]
    if (not np.isfinite(np.r_[na, nc, nd]).all() or np.any(np.r_[na, nc, nd] < 0)
            or np.any(np.r_[na, nc, nd] != np.round(np.r_[na, nc, nd]))):
        raise ValueError("PipeMotion:Cycles：周期数量需要有限非负整数")
    N = (na + nc + nd).astype(int)
    total = int(N.sum()) + 1
    if total > 2_000_000 or np.any(N <= 0):
        raise ValueError("PipeMotion:PointBudget：采样点数量或块周期无效")
    dt = float(plan["Ts"])
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("PipeMotion:SamplingTime：Ts 需要有限正数")
    data = {name: np.zeros(total) for name in ("s", "v", "a", "j", "block_index", "block_step_index")}
    boundary = np.r_[0, np.cumsum(N)]
    middle = _stage_speed(plan)
    block_s = np.asarray(plan["block_s"])
    for k, steps in enumerate(N):
        s, v, a, j = _sine_integral(np.arange(steps + 1) * dt, na[k] * dt, nc[k] * dt, nd[k] * dt,
                                    plan["vs"][k], middle[k], plan["ve"][k])
        length = block_s[k + 1] - block_s[k]
        if abs(s[-1] - length) > 1e-8:
            raise RuntimeError(f"PipeMotion:LengthClosure：第 {k + 1} 块解析积分未闭合")
        s[-1], v[-1], a[-1] = length, plan["ve"][k], 0.
        begin = int(k > 0)
        rows = slice(boundary[k] + begin, boundary[k + 1] + 1)
        for name, values in zip(("s", "v", "a", "j"), (block_s[k] + s, v, a, j)):
            data[name][rows] = values[begin:]
        data["block_index"][rows] = k + 1
        data["block_step_index"][rows] = np.arange(begin, steps + 1)
    data["boundary_index"] = boundary + 1
    data["time"] = np.arange(total) * dt
    data["u"], data["arc_residual"] = pipe_nurbs_geometry("invert", geo, arc, data["s"])
    data["position"], Cu, Cuu, Cuuu = pipe_nurbs_geometry("eval", geo, data["u"])
    q = np.linalg.norm(Cu, axis=1)
    if np.any(q < 1e-10):
        raise ValueError("PipeMotion:StationaryCurve：曲线包含驻点")
    dot = np.sum(Cu * Cuu, axis=1)
    qu = dot / q
    quu = (np.sum(Cuu ** 2, axis=1) + np.sum(Cu * Cuuu, axis=1)) / q - dot ** 2 / q ** 3
    ud = data["v"] / q
    udd = data["a"] / q - data["v"] ** 2 * qu / q ** 3
    uj = (data["j"] / q - 3 * data["v"] * data["a"] * qu / q ** 3
          - data["v"] ** 3 * quu / q ** 4 + 3 * data["v"] ** 3 * qu ** 2 / q ** 5)
    data["tangent"] = Cu / q[:, None]
    data["velocity"] = Cu * ud[:, None]
    data["acceleration"] = Cuu * ud[:, None] ** 2 + Cu * udd[:, None]
    data["jerk"] = Cuuu * ud[:, None] ** 3 + 3 * Cuu * (ud * udd)[:, None] + Cu * uj[:, None]
    data["kappa"] = np.linalg.norm(np.cross(Cu, Cuu), axis=1) / q ** 3
    data["step_speed"] = np.linalg.norm(np.diff(data["position"], axis=0), axis=1) / dt
    pipe_motion_trace("add", "sample", time.monotonic() - started)
    return data


def pipe_feed_profile(action, *args):
    if action in ("sample", "check"):
        plan = dict(args[2])
        for name in ("block_u", "block_s", "vs", "ve", "vfs", "stage_speed", "na", "nc", "nd", "length_residual",
                     "original_boundary_u", "kept_boundary_indices"):
            if name in plan:
                plan[name] = np.atleast_1d(plan[name])
        args = (*args[:2], plan, *args[3:])
    if action == "sample":
        return _sample(*args)
    if action == "check":
        return _check(*args)
    if action == "build":
        return _build(*args)
    raise ValueError(f"PipeMotion:UnknownAction：未知操作 {action}")


def _motion_values(plan, time_values):
    na, nc, nd = [np.asarray(plan[name], dtype=float).ravel() for name in ("na", "nc", "nd")]
    starts = np.r_[0., np.cumsum((na + nc + nd) * plan["Ts"])]
    time_values = np.clip(np.asarray(time_values, dtype=float).ravel(), 0, starts[-1])
    blocks = np.clip(np.searchsorted(starts, time_values, side="right") - 1, 0, len(na) - 1)
    s, v, a = np.zeros_like(time_values), np.zeros_like(time_values), np.zeros_like(time_values)
    middle = _stage_speed(plan)
    for k in range(len(na)):
        rows = blocks == k
        distance, v[rows], a[rows], _ = _sine_integral(time_values[rows] - starts[k], na[k] * plan["Ts"],
                                                     nc[k] * plan["Ts"], nd[k] * plan["Ts"],
                                                     plan["vs"][k], middle[k], plan["ve"][k])
        s[rows] = plan["block_s"][k] + distance
    return s, v, a


def _time_at_distance(plan, s):
    s = np.asarray(s, dtype=float).ravel()
    na, nc, nd = [np.asarray(plan[name], dtype=float).ravel() for name in ("na", "nc", "nd")]
    starts = np.r_[0., np.cumsum((na + nc + nd) * plan["Ts"])]
    blocks = np.clip(np.searchsorted(plan["block_s"], s, side="right") - 1, 0, len(na) - 1)
    result, middle = np.zeros_like(s), _stage_speed(plan)
    for k in range(len(na)):
        rows = np.flatnonzero(blocks == k)
        if not len(rows):
            continue
        target = s[rows] - plan["block_s"][k]
        low, high = np.zeros(len(rows)), np.full(len(rows), starts[k + 1] - starts[k])
        for _ in range(50):
            local = (low + high) / 2
            distance = _sine_integral(local, na[k] * plan["Ts"], nc[k] * plan["Ts"], nd[k] * plan["Ts"],
                                       plan["vs"][k], middle[k], plan["ve"][k])[0]
            less = distance < target
            low[less], high[~less] = local[less], local[~less]
        result[rows] = starts[k] + (low + high) / 2
    return result


def _geometry_metrics(geo, arc, plan, time_values, known_u=None):
    s, v, a = _motion_values(plan, time_values)
    s = np.clip(s, 0, arc["total_length"])
    u = np.asarray(known_u).ravel() if known_u is not None else pipe_nurbs_geometry("invert", geo, arc, s)[0]
    _, Cu, Cuu, Cuuu = pipe_nurbs_geometry("eval", geo, u)
    q = np.linalg.norm(Cu, axis=1)
    if np.any(q < 1e-10):
        raise ValueError("PipeMotion:StationaryCurve：连续几何验限发现驻点")
    tangent = Cu / q[:, None]
    dot = np.sum(Cu * Cuu, axis=1)
    qu = dot / q
    quu = (np.sum(Cuu ** 2, axis=1) + np.sum(Cu * Cuuu, axis=1)) / q - dot ** 2 / q ** 3
    Css = Cuu / q[:, None] ** 2 - Cu * (qu / q ** 3)[:, None]
    Csss = (Cuuu / q[:, None] ** 3 - 3 * Cuu * (qu / q ** 4)[:, None]
            - Cu * (quu / q ** 4)[:, None] + 3 * Cu * (qu ** 2 / q ** 5)[:, None])
    normal_third = Csss - np.sum(Csss * tangent, axis=1)[:, None] * tangent
    normal_jerk = np.linalg.norm(3 * Css * (v * a)[:, None] + normal_third * v[:, None] ** 3, axis=1)
    kappa = np.linalg.norm(np.cross(Cu, Cuu), axis=1) / q ** 3
    metrics = np.column_stack([kappa * v ** 2, normal_jerk, kappa ** 2 * v ** 3])
    if not np.isfinite(metrics).all():
        raise RuntimeError("PipeMotion:ContinuousGeometry：几何验限出现非有限值")
    return metrics, s, u


def _collect(out, s, metrics, plan, lim):
    if not len(metrics):
        return
    names = ("max_normal_acceleration", "max_normal_jerk", "max_curvature_jerk")
    for index, name in enumerate(names):
        out[name] = max(out[name], float(np.max(metrics[:, index])))
    if len(plan["block_s"]) <= 2:
        return
    factors = np.max(np.column_stack([np.sqrt(metrics[:, 0] / lim["An"]),
                                       np.cbrt(metrics[:, 1] / lim["Jn"]),
                                       np.cbrt(metrics[:, 2] / lim["Jn"])]), axis=1)
    violated = np.flatnonzero(factors > 1)
    if len(violated):
        nearest = np.argmin(np.abs(s[violated, None] - np.asarray(plan["block_s"])[None, 1:-1]), axis=1) + 1
        for index in np.unique(nearest):
            out["boundary_violation_factor"][index] = max(out["boundary_violation_factor"][index],
                                                          float(np.max(factors[violated[nearest == index]])))


def _refine(geo, arc, plan, lim, out, times, s, metrics, node_time):
    starts = np.r_[0., np.cumsum((np.asarray(plan["na"], float) + plan["nc"] + plan["nd"]) * plan["Ts"])]
    breaks = np.unique(np.r_[starts, starts[:-1] + plan["na"] * plan["Ts"],
                             starts[:-1] + (plan["na"] + plan["nc"]) * plan["Ts"], node_time])
    left, right, width = breaks[:-1], breaks[1:], np.diff(breaks)
    inset = np.minimum(width / 8, np.maximum(32 * np.spacing(max(1., starts[-1])), width * 1e-10))
    extra = np.unique(np.r_[breaks, left + inset, right - inset, left + width / 4,
                            left + width / 2, left + 3 * width / 4])
    extra_metrics, extra_s, _ = _geometry_metrics(geo, arc, plan, extra)
    _collect(out, extra_s, extra_metrics, plan, lim)
    times, order = np.unique(np.r_[times, extra], return_index=True)
    metrics = np.vstack([metrics, extra_metrics])[order]
    a, b, types = [], [], []
    flat = 128 * np.spacing([max(1., lim["An"]), max(1., lim["Jn"]), max(1., lim["Jn"])])
    for start, end in zip(left, right):
        rows = np.flatnonzero((times > start) & (times < end))
        if len(rows) < 3:
            continue
        for metric in range(3):
            values = metrics[rows, metric]
            peaks = np.flatnonzero((values[1:-1] >= values[:-2]) & (values[1:-1] >= values[2:])
                                   & (np.maximum(values[1:-1] - values[:-2], values[1:-1] - values[2:]) > flat[metric]))
            for peak in peaks:
                a.append(times[rows[peak]])
                b.append(times[rows[peak + 2]])
                types.append(metric)
            if np.max(values) > flat[metric]:
                a.extend([times[rows[0]], times[rows[-2]]])
                b.extend([times[rows[1]], times[rows[-1]]])
                types.extend([metric, metric])
    tolerance = max(2e-12, 64 * np.spacing(max(1., starts[-1])))
    audit = {"method": "smooth-interval bounded local maxima", "smooth_intervals": len(width),
             "extra_probe_count": len(extra), "candidate_count": len(a), "time_tolerance_s": tolerance,
             "time": np.zeros(0), "parameter": np.zeros(0), "arc_length_mm": np.zeros(0),
             "metric_index": np.zeros(0, dtype=int), "values": np.zeros((0, 3))}
    if not a:
        return audit
    # 保留源程序的批量黄金分割，所有搜索区间均位于单个光滑几何和运动阶段。
    a, b, types = np.asarray(a), np.asarray(b), np.asarray(types, dtype=int)
    ratio = (np.sqrt(5) - 1) / 2
    x1, x2 = b - ratio * (b - a), a + ratio * (b - a)
    rows = np.arange(len(a))
    f1 = _geometry_metrics(geo, arc, plan, x1)[0][rows, types]
    f2 = _geometry_metrics(geo, arc, plan, x2)[0][rows, types]
    best_time, best_value = x1.copy(), f1.copy()
    better = f2 > best_value
    best_time[better], best_value[better] = x2[better], f2[better]
    for _ in range(64):
        active = b - a > tolerance
        if not np.any(active):
            break
        move_right, move_left = active & (f1 < f2), active & (f1 >= f2)
        a[move_right], x1[move_right], f1[move_right] = x1[move_right], x2[move_right], f2[move_right]
        b[move_left], x2[move_left], f2[move_left] = x2[move_left], x1[move_left], f1[move_left]
        x2[move_right] = a[move_right] + ratio * (b[move_right] - a[move_right])
        x1[move_left] = b[move_left] - ratio * (b[move_left] - a[move_left])
        selected = np.flatnonzero(active)
        at = np.where(move_right[selected], x2[selected], x1[selected])
        values = _geometry_metrics(geo, arc, plan, at)[0][np.arange(len(selected)), types[selected]]
        chosen_right = move_right[selected]
        f2[selected[chosen_right]], f1[selected[~chosen_right]] = values[chosen_right], values[~chosen_right]
        better = values > best_value[selected]
        best_time[selected[better]], best_value[selected[better]] = at[better], values[better]
    values, refined_s, refined_u = _geometry_metrics(geo, arc, plan, best_time)
    _collect(out, refined_s, values, plan, lim)
    audit.update(time=best_time, parameter=refined_u, arc_length_mm=refined_s, metric_index=types + 1, values=values)
    return audit


def _dense(geo, arc, plan, lim):
    subdivision = max(4, int(np.ceil(plan["Ts"] / .0005)))
    probe = plan | {"Ts": plan["Ts"] / subdivision}
    for name in ("na", "nc", "nd"):
        probe[name] = np.asarray(plan[name], dtype=float) * subdivision
    fine = _sample(geo, arc, probe)
    tangent_jerk = np.sum(fine["jerk"] * fine["tangent"], axis=1)
    metrics = np.column_stack([fine["kappa"] * fine["v"] ** 2,
                               np.linalg.norm(fine["jerk"] - tangent_jerk[:, None] * fine["tangent"], axis=1),
                               fine["kappa"] ** 2 * fine["v"] ** 3])
    out = {"time_step": probe["Ts"], "time_points": len(fine["u"]), "max_normal_acceleration": 0.,
           "max_normal_jerk": 0., "max_curvature_jerk": 0., "node_side_points": 0,
           "boundary_violation_factor": np.ones(len(plan["block_u"]))}
    _collect(out, fine["s"], metrics, plan, lim)
    knots = np.asarray(geo["knots"])
    nodes = np.unique(np.r_[knots[(knots > 0) & (knots < 1)], np.asarray(plan.get("original_boundary_u", [0, 1]))[1:-1]])
    node_time = np.zeros(0)
    if len(nodes):
        all_nodes = np.r_[0, nodes, 1]
        step = np.minimum.reduce([np.full(len(nodes), 1e-9), (nodes - all_nodes[:-2]) / 1000,
                                   (all_nodes[2:] - nodes) / 1000])
        node_u = np.unique(np.r_[nodes - step, nodes, nodes + step])
        node_arc = pipe_nurbs_geometry("arc", geo, np.r_[0., node_u, 1.])
        node_s = node_arc["block_s"][1:-1]
        all_time = _time_at_distance(plan, node_s)
        node_time = all_time[np.searchsorted(node_u, nodes)]
        node_metrics = _geometry_metrics(geo, arc, plan, all_time, node_u)[0]
        _collect(out, node_s, node_metrics, plan, lim)
        out["node_side_points"] = len(node_u)
    out["continuous_extrema"] = _refine(geo, arc, plan, lim, out, fine["time"], fine["s"], metrics, node_time)
    return out


def _check(geo, data, plan, lim):
    started = time.monotonic()
    dt, middle = plan["Ts"], _stage_speed(plan)
    step_a, step_j = np.diff(data["step_speed"]) / dt, np.diff(data["step_speed"], n=2) / dt ** 2
    ramp_a, ramp_j = [], []
    for name, speed in (("na", plan["vs"]), ("nd", plan["ve"])):
        counts = np.asarray(plan[name], dtype=float)
        active = counts > 0
        delta = np.abs(middle - speed)[active]
        ramp_a.extend(np.pi * delta / (2 * counts[active] * dt))
        ramp_j.extend(np.pi ** 2 * delta / (2 * (counts[active] * dt) ** 2))
    def maxima(values):
        return float(np.max(values, initial=0.))
    normal_j = data["jerk"] - np.sum(data["jerk"] * data["tangent"], axis=1)[:, None] * data["tangent"]
    out = {
        "max_speed": maxima(np.r_[data["v"], data["step_speed"]]),
        "max_tangent_acceleration": maxima(np.r_[np.abs(data["a"]), ramp_a, np.abs(step_a)]),
        "max_tangent_jerk": maxima(np.r_[np.abs(data["j"]), ramp_j, np.abs(step_j)]),
        "max_normal_acceleration": maxima(data["kappa"] * data["v"] ** 2),
        "max_curvature_jerk": maxima(data["kappa"] ** 2 * data["v"] ** 3),
        "max_normal_jerk": maxima(np.linalg.norm(normal_j, axis=1)),
        "max_xyz_jerk": maxima(np.linalg.norm(data["jerk"], axis=1)),
        "max_step_tangent_acceleration": maxima(np.abs(step_a)), "max_step_tangent_jerk": maxima(np.abs(step_j)),
        "max_command_tangent_acceleration": maxima(np.abs(data["a"])), "max_command_tangent_jerk": maxima(np.abs(data["j"])),
    }
    P0, P1 = data["position"][:-1], data["position"][1:]
    line = P1 - P0
    chord = np.zeros(len(P0))
    for fraction in (.125, .25, .375, .5, .625, .75, .875):
        Q = pipe_nurbs_geometry("eval", geo, data["u"][:-1] + fraction * np.diff(data["u"]))[0]
        alpha = np.sum((Q - P0) * line, axis=1) / np.maximum(np.sum(line ** 2, axis=1), np.finfo(float).tiny)
        chord = np.maximum(chord, np.linalg.norm(Q - P0 - np.clip(alpha, 0, 1)[:, None] * line, axis=1))
    targets = pipe_nurbs_geometry("eval", geo, plan["block_u"])[0]
    errors = np.linalg.norm(data["position"][data["boundary_index"] - 1] - targets, axis=1)
    phase_tol = 1e-10 * max(1., np.max(plan["vfs"]))
    zero_phase = bool(np.all((plan["na"] > 0) | (np.abs(middle - plan["vs"]) <= phase_tol))
                      and np.all((plan["nd"] > 0) | (np.abs(middle - plan["ve"]) <= phase_tol)))
    ceiling = plan.get("speed_ceiling", plan.get("seed_peak_limit", np.inf))
    no_inflation = bool(np.all(plan["vfs"] <= ceiling + phase_tol))
    no_valley = bool(np.all(middle >= np.minimum(plan["vs"], plan["ve"]) - phase_tol))
    peak_ok = bool(np.all(np.abs(plan["vfs"] - np.maximum.reduce([plan["vs"], middle, plan["ve"]])) <= phase_tol))
    cycles = np.r_[plan["na"], plan["nc"], plan["nd"]]
    shape_ok = bool(zero_phase and no_inflation and no_valley and peak_ok and np.all(cycles >= 0)
                    and np.all(cycles == np.rint(cycles)) and np.all(np.abs(plan["ve"][:-1] - plan["vs"][1:]) <= phase_tol))
    out.update(chord_error_series=chord, max_chord_error_mm=maxima(chord), block_position_error_mm=errors,
               endpoint_error_mm=errors[-1], max_arc_residual_mm=maxima(np.abs(data["arc_residual"])),
               max_length_closure_mm=maxima(np.abs(plan["length_residual"])), start_speed=data["v"][0],
               end_speed=data["v"][-1], last_interval_speed=data["step_speed"][-1],
               min_parameter_step=float(np.min(np.diff(data["u"]))), point_count=len(data["u"]), duration_s=data["time"][-1],
               position_tolerance_mm=lim["position_tol"], zero_phase_ok=zero_phase, no_peak_inflation=no_inflation,
               no_internal_valley=no_valley, peak_field_ok=peak_ok, shape_ok=shape_ok)
    out["geometry_ok"] = bool(shape_ok and np.isfinite(np.r_[data["position"].ravel(), data["v"], data["a"], data["j"], data["u"], data["s"]]).all()
                              and np.all(np.diff(data["u"]) > 0) and np.all(data["v"] >= -1e-12)
                              and np.max(errors) <= lim["position_tol"] and out["max_arc_residual_mm"] <= lim["position_tol"]
                              and out["max_length_closure_mm"] <= lim["position_tol"] and abs(data["v"][0]) <= 1e-12 and abs(data["v"][-1]) <= 1e-12)
    if plan.get("version", 0) >= 3:
        arc = pipe_nurbs_geometry("arc", geo, plan["block_u"])
        dense = _dense(geo, arc, plan, lim)
        out["dense_geometry"] = dense
        for name in ("max_normal_acceleration", "max_normal_jerk", "max_curvature_jerk"):
            out[name] = max(out[name], dense[name])
    out["passed"] = bool(out["geometry_ok"] and out["max_speed"] <= lim["F"]
                         and out["max_tangent_acceleration"] <= lim["At"] and out["max_tangent_jerk"] <= lim["Jt"]
                         and out["max_normal_acceleration"] <= lim["An"] and out["max_curvature_jerk"] <= lim["Jn"]
                         and out["max_normal_jerk"] <= lim["Jn"] and out["max_chord_error_mm"] <= lim["delta"])
    pipe_motion_trace("add", "complete_check", time.monotonic() - started)
    return out


def _speed_limit(kappa, lim):
    limit = np.full_like(kappa, float(lim["F"]))
    active = kappa > 1e-14
    k = kappa[active]
    radius = 1 / k
    limit[active] = np.minimum.reduce([limit[active], np.sqrt(lim["An"] / k),
                                       np.cbrt(lim["Jn"] / k ** 2),
                                       2 / lim["Ts"] * np.sqrt(np.maximum(2 * radius * lim["delta"] - lim["delta"] ** 2, 0))])
    return limit


def _violation_factor(validation, lim):
    return max(validation["max_speed"] / lim["F"], np.sqrt(validation["max_tangent_acceleration"] / lim["At"]),
               np.cbrt(validation["max_tangent_jerk"] / lim["Jt"]), np.sqrt(validation["max_normal_acceleration"] / lim["An"]),
               np.cbrt(validation["max_curvature_jerk"] / lim["Jn"]), np.cbrt(validation["max_normal_jerk"] / lim["Jn"]),
               np.sqrt(validation["max_chord_error_mm"] / lim["delta"]))


def _motion_key(plan):
    names = ("Ts", "block_u", "block_s", "vs", "ve", "vfs", "na", "nc", "nd", "stage_speed",
             "length_residual", "speed_ceiling", "seed_peak_limit", "version", "original_boundary_u")
    return tuple((name, np.asarray(plan[name]).dtype.str, np.asarray(plan[name]).shape, np.asarray(plan[name]).tobytes())
                 for name in names if name in plan)


def _coarsen(geo, arc, seed, lim, physical, effective, best, validation):
    if len(best["block_u"]) <= 2:
        best["boundary_coarsening_audit"] = []
        return best, validation
    audit, original_count = [], len(seed["block_u"])
    budget = 2 * original_count + 4
    cache = {_motion_key(best): validation}

    def evaluate(keep, scale):
        candidate, checked = None, None
        item = {"removed_original_indices": np.flatnonzero(~keep) + 1, "scale": scale, "duration_s": np.nan,
                "passed": False, "accepted": False, "terminal_peak_candidate": False,
                "skipped_slower_duration_s": [], "error": ""}
        for peak in (False, True):
            candidate_effective, feedback_history = effective.copy(), []
            trial = None
            for refinement in range(1, 4):
                try:
                    trial = pipe_joint_schedule(arc, seed, physical, lim, scale,
                                                {"effective_boundary_limit": candidate_effective,
                                                 "retained_boundary_mask": keep, "terminal_peak_candidate": peak})
                except JointScheduleError as error:
                    if candidate is None:
                        item["error"] = str(error)
                    break
                trial.update(version=3, time_scale=scale, speed_ceiling=lim["F"] / scale,
                             attempts=best["attempts"], solver_failure_count=best["solver_failure_count"],
                             original_boundary_limits=physical, original_boundary_kappa=best["original_boundary_kappa"],
                             geometry_feedback_history=best["geometry_feedback_history"], candidate_geometry_feedback=feedback_history.copy())
                duration = np.sum(trial["na"] + trial["nc"] + trial["nd"]) * trial["Ts"]
                best_duration = np.sum(best["na"] + best["nc"] + best["nd"]) * best["Ts"]
                if duration > best_duration:
                    item["skipped_slower_duration_s"].append(duration)
                    if candidate is None:
                        item.update(duration_s=duration, error="候选周期数量超过当前已验证计划")
                    break
                key = _motion_key(trial)
                test = cache.get(key)
                if test is None:
                    data = _sample(geo, arc, trial)
                    test = _check(geo, data, trial, lim)
                    cache[key] = test
                    if len(cache) > 64:
                        del cache[next(iter(cache))]
                else:
                    pipe_motion_trace("count", "identical_validation_hits", 1)
                if candidate is None or (test["passed"] and (not checked["passed"] or test["duration_s"] < checked["duration_s"])):
                    candidate, checked = trial, test
                    item.update(duration_s=test["duration_s"], passed=test["passed"], terminal_peak_candidate=peak, error="")
                if test["passed"] or not test["geometry_ok"] or len(trial["na"]) == 1 or refinement == 3:
                    break
                local = test["dense_geometry"]["boundary_violation_factor"]
                bad = np.flatnonzero(local > 1)
                bad = bad[(bad > 0) & (bad < len(trial["block_u"]) - 1)]
                if not len(bad):
                    break
                original = trial["kept_boundary_indices"][bad] - 1
                speeds = np.r_[trial["vs"], trial["ve"][-1]]
                old = candidate_effective[original].copy()
                candidate_effective[original] = np.minimum(old, speeds[bad]) / (1.02 * local[bad])
                feedback_history.append({"iteration": refinement, "original_indices": original + 1, "old_cap": old,
                                         "new_cap": candidate_effective[original].copy(), "violation_factor": local[bad]})
            if trial is not None and len(trial["na"]) == 1:
                break
        return candidate, checked, item

    def improves(candidate, test):
        time_tolerance = 64 * np.spacing(max(1., validation["duration_s"]))
        variation_tolerance = 64 * np.spacing(max(1., best["total_velocity_variation"]))
        return (test["duration_s"] < validation["duration_s"] - time_tolerance
                or (abs(test["duration_s"] - validation["duration_s"]) <= time_tolerance
                    and candidate["total_velocity_variation"] <= best["total_velocity_variation"] + variation_tolerance))

    keep = best["retained_boundary_mask"].copy()
    candidate, test, item = evaluate(keep, best["time_scale"])
    if candidate is not None and test["passed"] and test["duration_s"] < validation["duration_s"]:
        item["accepted"] = True
        best, validation, effective = candidate, test, candidate["original_effective_boundary_limit"]
    audit.append(item)
    keep = np.zeros(original_count, dtype=bool)
    keep[[0, -1]] = True
    scale = best["time_scale"]
    for _ in range(3):
        candidate, test, item = evaluate(keep, scale)
        if candidate is not None and test["passed"]:
            item["accepted"] = bool(improves(candidate, test))
            if item["accepted"]:
                best, validation, effective = candidate, test, candidate["original_effective_boundary_limit"]
            audit.append(item)
            break
        audit.append(item)
        if candidate is None or not test["geometry_ok"]:
            break
        scale *= max(1.02, 1.02 * _violation_factor(test, lim))
    while len(best["block_u"]) > 2 and len(audit) < budget:
        improved = False
        for original in best["kept_boundary_indices"][1:-1] - 1:
            keep = best["retained_boundary_mask"].copy()
            keep[original] = False
            candidate, test, item = evaluate(keep, best["time_scale"])
            if candidate is not None and test["passed"]:
                item["accepted"] = bool(improves(candidate, test))
                if item["accepted"]:
                    best, validation, effective = candidate, test, candidate["original_effective_boundary_limit"]
                    improved = True
            audit.append(item)
            if improved or len(audit) >= budget:
                break
        if not improved:
            break
    best["boundary_coarsening_audit"] = audit
    return best, validation


def _build(geo, arc, seed, lim):
    started = time.monotonic()
    if any(not np.isscalar(lim[name]) or not np.isfinite(lim[name]) or lim[name] <= 0
           for name in ("F", "At", "Jt", "An", "Jn", "Ts", "delta", "position_tol")):
        raise ValueError("PipeMotion:Limits：所有运动限制需要有限正数")
    _, first, second, _ = pipe_nurbs_geometry("eval", geo, seed["block_u"])
    speed = np.linalg.norm(first, axis=1)
    if np.any(speed < 1e-10):
        raise ValueError("PipeMotion:StationaryCurve：关键位置包含驻点")
    kappa = np.linalg.norm(np.cross(first, second), axis=1) / speed ** 3
    physical = _speed_limit(kappa, lim)
    physical[[0, -1]] = 0.
    effective, scale, solver_failures, history = physical.copy(), 1., 0, []
    for attempt in range(1, 25):
        pipe_motion_trace("note", f"联合调度第 {attempt} 轮，随后完整验证约束。")
        try:
            plan = pipe_joint_schedule(arc, seed, physical, lim, scale, {"effective_boundary_limit": effective})
        except JointScheduleError as error:
            if not str(error).startswith("PipeMotion:JointInfeasible"):
                raise
            solver_failures += 1
            # 无可行候选时扩大原优化的时间尺度，全部几何和物理约束继续完整验收。
            if solver_failures >= 4:
                positive = effective[effective > 0]
                if not len(positive):
                    raise JointScheduleError("PipeMotion:Infeasible：没有正速度边界可供调度") from error
                scale = max(scale * 1.1, lim["F"] / np.min(positive) * (1 + 1e-8))
            else:
                scale *= 1.05
            continue
        plan.update(version=3, time_scale=scale, attempts=attempt, solver_failure_count=solver_failures,
                    original_boundary_limits=physical, original_boundary_kappa=kappa,
                    geometry_feedback_history=history.copy(), speed_ceiling=lim["F"] / scale)
        validation = _check(geo, _sample(geo, arc, plan), plan, lim)
        if validation["passed"]:
            plan, validation = _coarsen(geo, arc, seed, lim, physical, effective, plan, validation)
            validation = _check(geo, _sample(geo, arc, plan), plan, lim)
            if not validation["passed"]:
                raise RuntimeError("PipeMotion:FinalValidation：最终完整验证失败")
            plan["validation"] = validation
            pipe_motion_trace("add", "build", time.monotonic() - started)
            return plan, validation
        if not validation["geometry_ok"]:
            raise RuntimeError("PipeMotion:GeometryMismatch：几何闭合或运动连续性验证失败")
        if attempt > 1 and len(plan["na"]) > 1:
            local = validation["dense_geometry"]["boundary_violation_factor"]
            bad = np.flatnonzero(local > 1)
            bad = bad[(bad > 0) & (bad < len(plan["block_u"]) - 1)]
            if len(bad):
                original = plan["kept_boundary_indices"][bad] - 1
                speeds = np.r_[plan["vs"], plan["ve"][-1]]
                old = effective[original].copy()
                effective[original] = np.minimum(old, speeds[bad]) / (1.02 * local[bad])
                history.append({"attempt": attempt, "original_indices": original + 1, "old_cap": old,
                                "new_cap": effective[original].copy(), "violation_factor": local[bad]})
                continue
        scale *= max(1.02, 1.02 * _violation_factor(validation, lim))
    raise JointScheduleError("PipeMotion:Infeasible：24 次联合规划仍未满足全部约束")
