import time
from pathlib import Path

import numpy as np

from .mat_io import save_mat
from .pipe_curvature_keypoints import pipe_curvature_keypoints
from .pipe_feed_profile import pipe_feed_profile
from .pipe_motion_trace import pipe_motion_trace
from .pipe_nurbs_geometry import pipe_nurbs_geometry


def _scan_geometry(geo, options):
    dt, feed = options["Ts"], options["F"]

    def evaluate(u):
        P, D1, D2, _ = pipe_nurbs_geometry("eval", geo, [u])
        speed = np.linalg.norm(D1[0])
        if speed < 1e-8:
            raise ValueError("PipeMotion:StationaryCurve：PCI 扫描发现驻点")
        kappa = np.linalg.norm(np.cross(D1[0], D2[0])) / speed ** 3
        radius = 1 / max(kappa, 1e-6)
        chord_limit = min(feed, 2 / dt * np.sqrt(max(radius ** 2 - (radius - options["delta"]) ** 2, 0)))
        curvature_limit = options["kcbc"] / (options["kcbc"] + kappa) * feed
        desired = max(min(chord_limit, curvature_limit, feed), options["Fmin"])
        return P[0], D1[0], D2[0], speed, kappa, desired

    u_values, positions, kappas, desired_values = [], [], [], []
    current = 0.
    for k in range(3):
        position, first, second, speed, kappa, desired = evaluate(current)
        u_values.append(current)
        positions.append(position)
        kappas.append(kappa)
        desired_values.append(desired)
        if k < 2:
            increment = desired * dt / speed - np.dot(first, second) * desired ** 2 * dt ** 2 / (2 * speed ** 4)
            current = min(max(current + increment, current + 1e-12), .25 if k == 0 else .5)
    minimum_increment, maximum_u = 1e-10, 1 - 1e-10
    while u_values[-1] < .9999 and len(u_values) < 20000:
        previous = u_values[-1]
        predicted = min(max(3 * u_values[-1] - 3 * u_values[-2] + u_values[-3], previous + minimum_increment), maximum_u)
        if predicted <= previous + minimum_increment:
            raise RuntimeError("PipeMotion:PCIProgress：预测步长不足，扫描终止")
        next_u = predicted
        for _ in range(21):
            position = pipe_nurbs_geometry("eval", geo, [next_u])[0][0]
            velocity = np.linalg.norm(position - positions[-1]) / dt
            denominator = desired_values[-1] - options["beta"] * (desired_values[-1] - velocity)
            if abs(denominator) < 1e-10:
                raise RuntimeError("PipeMotion:PCICorrection：进给修正分母无效")
            alpha = np.clip(desired_values[-1] / denominator, .1, 2.)
            next_u = min(max(previous + alpha * (next_u - previous), previous + minimum_increment), maximum_u)
            if abs(desired_values[-1] - velocity) / desired_values[-1] <= options["eps_PCI"]:
                break
        if next_u <= previous + minimum_increment:
            raise RuntimeError("PipeMotion:PCIProgress：校正后没有有效推进，扫描终止")
        position, _, _, _, kappa, desired = evaluate(next_u)
        u_values.append(next_u)
        positions.append(position)
        kappas.append(kappa)
        desired_values.append(desired)
    if u_values[-1] < .9999:
        raise RuntimeError("PipeMotion:IncompleteScan：扫描步数耗尽")
    if u_values[-1] < 1.:
        position, _, _, _, kappa, desired = evaluate(1.)
        u_values.append(1.)
        positions.append(position)
        kappas.append(kappa)
        desired_values.append(desired)
    return np.asarray(u_values), np.asarray(positions), np.asarray(kappas), np.asarray(desired_values)


def A_2_lixianjisuan(fit_dir, output_dir, options=None):
    started = time.monotonic()
    defaults = {"delta": .001, "F": 60., "At": 100., "Jt": 16400., "An": 300., "Jn": 16400.,
                "Ts": .002, "eps": .0001, "eps_PCI": .001, "beta": .9, "Fmin": 10., "kcbc": 1.01,
                "position_tol": 1e-6, "max_samples": 2_000_000}
    if options is not None and (not isinstance(options, dict) or set(options) - set(defaults)):
        raise ValueError("PipeMotion:Options：未知规划参数")
    options = defaults | (options or {})
    if any(not np.isscalar(value) or not np.isfinite(value) or value <= 0 for value in options.values()):
        raise ValueError("PipeMotion:Options：规划参数需要有限正数")
    if options["Fmin"] > options["F"] or not 0 < options["beta"] < 1:
        raise ValueError("PipeMotion:Options：Fmin 或 beta 无效")
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    pipe_motion_trace("reset")
    geo = pipe_nurbs_geometry("load", fit_dir)
    u, positions, kappa, desired = _scan_geometry(geo, options)
    scan_arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(positions, axis=0), axis=1))]
    threshold = min(8 * options["delta"] / ((options["F"] * options["Ts"]) ** 2 + 4 * options["delta"] ** 2),
                    options["An"] / options["F"] ** 2, np.sqrt(options["Jn"] / options["F"] ** 3))
    peak_options = {"MinPeakProminence": 1e-6, "MaxRelativeSaddleDepth": .005,
                    "MaxPeakDistanceMM": 2., "MaxCurvatureSpan": .15}
    indices, peak_audit = pipe_curvature_keypoints(u, kappa, threshold, scan_arc, peak_options)
    block_u = np.unique(np.r_[0., u[indices - 1], 1.])
    arc = pipe_nurbs_geometry("arc", geo, block_u)
    limits = {name: options[name] for name in ("F", "At", "Jt", "An", "Jn", "Ts", "delta", "position_tol", "max_samples")}
    seed = {"block_u": block_u, "vs": np.zeros(len(block_u) - 1), "ve": np.zeros(len(block_u) - 1),
            "vfs": np.full(len(block_u) - 1, limits["F"]), "na": np.zeros(len(block_u) - 1),
            "nc": np.zeros(len(block_u) - 1), "nd": np.zeros(len(block_u) - 1)}
    plan, validation = pipe_feed_profile("build", geo, arc, seed, limits)
    raw_limits = plan["original_boundary_limits"]
    seed.update(vs=raw_limits[:-1], ve=raw_limits[1:])
    result = {name: options[name] for name in defaults}
    result.update(format_version=3, source_curve=geo, arc_table=arc | {"block_u": plan["block_u"], "block_s": plan["block_s"]},
                  motion_plan=plan, validation=validation, limits=limits, block_u=plan["block_u"],
                  block_V=np.r_[plan["vs"][0], plan["ve"]], block_S=np.diff(plan["block_s"]),
                  sched_vs=plan["vs"], sched_ve=plan["ve"], sched_vfs=plan["vfs"], sched_na=plan["na"], sched_nc=plan["nc"], sched_nd=plan["nd"],
                  compensation_method=plan["compensation_method"], peak_adjustment=plan["peak_adjustment"],
                  raw_boundary_u=block_u, raw_block_S=np.diff(arc["block_s"]), raw_boundary_limits=raw_limits,
                  seed=seed, peak_selection_options=peak_options, peak_selection_audit=peak_audit,
                  u_list=u, kappa_list=kappa, scan_arc_mm=scan_arc, V_star_list=desired)
    pipe_motion_trace("add", "a2_total", time.monotonic() - started)
    result["performance"] = pipe_motion_trace("get")
    save_mat(output / "nurbs_blocks.mat", result)
    return result
