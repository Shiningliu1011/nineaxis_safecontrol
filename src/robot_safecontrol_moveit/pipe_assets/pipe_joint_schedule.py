import time

import numpy as np
from scipy.optimize import linprog

from .pipe_motion_trace import pipe_motion_trace


class JointScheduleError(RuntimeError):
    pass


def _ceil_integer(value):
    value = np.asarray(value, dtype=float)
    nearest = np.rint(value)
    value = np.where(np.abs(value - nearest) <= 64 * np.spacing(np.maximum(1., np.abs(value))), nearest, value)
    return np.ceil(value).astype(int)


def _phase_cycles(delta, lim):
    if delta <= 64 * np.spacing(max(1., abs(delta))):
        return 0
    return max(1, int(_ceil_integer(max(np.pi * delta / (2 * lim["At"] * lim["Ts"]),
                                       np.pi / lim["Ts"] * np.sqrt(delta / (2 * lim["Jt"]))))))


def _reachable_speed(low, high, distance, lim):
    def length(value):
        return lim["Ts"] * _phase_cycles(abs(value - low), lim) * (value + low) / 2

    if length(high) <= distance:
        return high
    left, right = low, high
    for _ in range(60):
        middle = (left + right) / 2
        if length(middle) <= distance:
            left = middle
        else:
            right = middle
    return left


def _fitting_peak(vs, ve, cap, distance, lim):
    def length(value):
        return lim["Ts"] * (_phase_cycles(value - vs, lim) * (vs + value)
                            + _phase_cycles(value - ve, lim) * (ve + value)) / 2

    left, right = max(vs, ve), max(vs, ve, cap)
    if length(left) > distance + 1e-9 * max(1., distance):
        raise JointScheduleError("PipeMotion:JointInitial：边界无法在当前块长内完成整数周期过渡")
    if length(right) <= distance:
        return right
    for _ in range(60):
        middle = (left + right) / 2
        if length(middle) <= distance:
            left = middle
        else:
            right = middle
    return left


def _check_budget(counts, lim):
    if not np.isfinite(counts).all() or counts.sum() + 1 > lim.get("max_samples", 2_000_000):
        raise JointScheduleError("PipeMotion:JointPointBudget：整数周期数量超出采样预算")


def _initial_counts(S, caps, C, lim, terminal_monotone):
    n, dt = len(S), lim["Ts"]
    B, ceiling = np.minimum(caps, C), np.full(n, C)
    B[[0, -1]] = 0.
    for _ in range(64):
        before = B.copy()
        for k in range(n):
            if B[k + 1] > B[k]:
                B[k + 1] = _reachable_speed(B[k], B[k + 1], S[k], lim)
        for k in range(n - 1, -1, -1):
            if B[k] > B[k + 1]:
                B[k] = _reachable_speed(B[k + 1], B[k], S[k], lim)
        if np.max(np.abs(B - before)) <= 1e-11 * max(1., C):
            break
    base = np.zeros((n, 3), dtype=int)
    for k in range(n):
        if terminal_monotone and k == n - 1:
            base[k, 2], ceiling[k] = _phase_cycles(B[k], lim), B[k]
        else:
            ceiling[k] = _fitting_peak(B[k], B[k + 1], ceiling[k], S[k], lim)
            base[k, 0] = _phase_cycles(ceiling[k] - B[k], lim)
            base[k, 2] = _phase_cycles(ceiling[k] - B[k + 1], lim)
    counts, M = base.copy(), ceiling.copy()
    for k in range(n):
        if terminal_monotone and k == n - 1:
            if B[k] <= 0:
                raise JointScheduleError("PipeMotion:JointInitial：非零末段需要正入口速度")
            na, nd = 0, max(base[k, 2], _phase_cycles(B[k], lim))
            nc = max(0, int(_ceil_integer(S[k] / (dt * B[k]) - nd / 2)))
            if nc + nd / 2 <= 0:
                nc = 1
            B[k] = min(B[k], S[k] / (dt * (nc + nd / 2)))
            M[k], ceiling[k] = B[k], B[k]
        else:
            cap = max(ceiling[k], np.max(B[k:k + 2]))
            na = max(base[k, 0], _phase_cycles(cap - B[k], lim))
            nd = max(base[k, 2], _phase_cycles(cap - B[k + 1], lim))
            ramp = dt * (na * (B[k] + cap) + nd * (B[k + 1] + cap)) / 2
            nc = max(0, int(_ceil_integer((S[k] - ramp) / (dt * cap))))
            if nc + (na + nd) / 2 <= 0:
                nc = 1
            denominator = nc + (na + nd) / 2
            middle = (S[k] / dt - (na * B[k] + nd * B[k + 1]) / 2) / denominator
            if middle < np.max(B[k:k + 2]) - 1e-12 * max(1., C):
                ratio = min(1., S[k] / (ramp + dt * nc * cap))
                if not np.isfinite(ratio) or ratio <= 0:
                    raise JointScheduleError("PipeMotion:JointInitial：初排比例无效")
                B[k:k + 2] *= ratio
                cap *= ratio
                middle = (S[k] / dt - (na * B[k] + nd * B[k + 1]) / 2) / denominator
            ceiling[k], M[k] = cap, middle
        counts[k] = [na, nc, nd]
    _check_budget(counts, lim)
    return counts, {"boundary": B, "peak": M}


def _solve_candidate(S, caps, C, lim, counts, modes, relax_dynamics=False, allow_underfill=False):
    started = time.monotonic()
    n = len(S)
    ib, im = np.arange(n + 1), np.arange(n + 1, 2 * n + 1)
    iz1, iz2 = np.arange(2 * n + 1, 3 * n + 1), np.arange(3 * n + 1, 4 * n + 1)
    nv = 4 * n + 1 + int(relax_dynamics)
    dynamic = nv - 1
    A, rhs, E, erhs = [], [], [], []
    coverage = np.zeros(nv)
    for k, (a, c, d) in enumerate(counts):
        row = np.zeros(nv)
        row[[ib[k], im[k], ib[k + 1]]] = [a / 2, (a + d) / 2 + c, d / 2]
        normalizer = max(1., a + c + d)
        (A if allow_underfill else E).append(row / normalizer)
        (rhs if allow_underfill else erhs).append(S[k] / (lim["Ts"] * C * normalizer))
        if allow_underfill:
            coverage -= row
        for phase in range(2):
            row = np.zeros(nv)
            if phase == 0:
                row[im[k]], row[ib[k]], z, cycles = 1., -1., iz1[k], a
            else:
                row[ib[k + 1]], row[im[k]], z, cycles = 1., -1., iz2[k], d
            if cycles == 0:
                E.append(row.copy())
                erhs.append(0.)
            else:
                duration = cycles * lim["Ts"]
                allowed = min(2 * lim["At"] * duration / np.pi, 2 * lim["Jt"] * duration ** 2 / np.pi ** 2) / C
                for sign in (1, -1):
                    bound = sign * row
                    if relax_dynamics:
                        bound[dynamic] = -allowed
                    A.append(bound)
                    rhs.append(0. if relax_dynamics else allowed)
            for sign in (1, -1):
                absolute = sign * row
                absolute[z] = -1.
                A.append(absolute)
                rhs.append(0.)
        rise, fall = np.zeros(nv), np.zeros(nv)
        rise[im[k]], rise[ib[k]] = 1., -1.
        fall[ib[k + 1]], fall[im[k]] = 1., -1.
        if modes[k] == "peak":
            A.extend([-rise, fall])
        elif modes[k] == "accelerate":
            A.extend([-rise, -fall])
        elif modes[k] == "decelerate":
            A.extend([rise, fall])
        else:
            raise ValueError("PipeMotion:JointMode：未知运动阶段")
        rhs.extend([0., 0.])
    lb, ub = np.zeros(nv), np.ones(nv)
    ub[ib] = caps / C
    objective = np.zeros(nv)
    objective[np.r_[iz1, iz2]] = 1.
    if allow_underfill:
        objective = coverage
    info = {"exitflag": 0, "message": "", "solver_calls": 0, "dynamics_factor": 1., "phase": "not_started", "status": -1}

    def solve(f, phase):
        result = linprog(f, A_ub=np.asarray(A), b_ub=np.asarray(rhs),
                         A_eq=np.asarray(E) if E else None, b_eq=np.asarray(erhs) if E else None,
                         bounds=list(zip(lb, ub)), method="highs-ds",
                         options={"maxiter": 20000, "primal_feasibility_tolerance": 1e-9,
                                  "dual_feasibility_tolerance": 1e-9})
        info.update(exitflag=1 if result.success else -result.status, message=result.message,
                    solver_calls=info["solver_calls"] + 1, phase=phase, status=int(result.status))
        return result

    if relax_dynamics:
        lb[dynamic], ub[dynamic] = 1., np.inf
        dynamic_objective = np.zeros(nv)
        dynamic_objective[dynamic] = 1.
        result = solve(dynamic_objective, "dynamics_projection")
        if not result.success:
            info["dynamics_factor"] = np.nan
            return None, info
        info["dynamics_factor"] = result.fun
        row = np.zeros(nv)
        row[dynamic] = 1.
        A.append(row)
        rhs.append(result.fun + 1e-10 * max(1., result.fun))
    result = solve(objective, "primary")
    if not result.success:
        return None, info
    A.append(objective.copy())
    rhs.append(result.fun + 1e-10 * max(1., result.fun))
    secondary = np.zeros(nv)
    secondary[ib[1:-1]] = -1.
    secondary_result = solve(secondary, "secondary")
    if not secondary_result.success:
        return None, info
    x = secondary_result.x
    B, M = C * x[ib], C * x[im]
    a, c, d = counts.T
    residual = lim["Ts"] * ((a * B[:-1] + d * B[1:]) / 2 + ((a + d) / 2 + c) * M) - S
    inequality_error = np.max(np.asarray(A) @ x - rhs, initial=0.)
    equality_error = np.max(np.abs(np.asarray(E) @ x - erhs), initial=0.) if E else 0.
    if (not np.isfinite(x).all() or np.any(residual > 1e-8) or (not allow_underfill and np.any(np.abs(residual) > 1e-8))
            or inequality_error > 1e-7 or equality_error > 1e-8
            or np.any(x < lb - 1e-8) or np.any(x > ub + 1e-8)):
        info.update(exitflag=-998, message=f"LP 原始残差复核失败：{np.max(np.abs(residual))} mm")
        return None, info
    solution = {"boundary": B, "middle": M, "total_variation": float(np.sum(np.abs(M - B[:-1]) + np.abs(B[1:] - M))),
                "boundary_sum": float(B.sum()), "length_deficit": -residual}
    pipe_motion_trace("add", "linear_candidate", time.monotonic() - started)
    return solution, info


def pipe_joint_schedule(arc, seed, physical_boundary_limit, lim, scale, feedback=None):
    started = time.monotonic()
    feedback = {} if feedback is None else feedback
    if not np.isscalar(scale) or not np.isfinite(scale) or scale <= 0:
        raise ValueError("PipeMotion:JointInput：时间倍率需要有限正数")
    if any(not np.isscalar(lim[name]) or not np.isfinite(lim[name]) or lim[name] <= 0 for name in ("Ts", "F", "At", "Jt")):
        raise ValueError("PipeMotion:JointInput：运动限制和周期需要有限正数")
    original_u, original_s = np.asarray(seed["block_u"]).ravel(), np.asarray(arc["block_s"]).ravel()
    physical = np.asarray(physical_boundary_limit, dtype=float).ravel().copy()
    if (len(original_u) != len(original_s) or len(physical) != len(original_s) or len(original_s) < 2
            or not np.isfinite(np.r_[original_u, original_s]).all() or np.any(np.diff(original_u) <= 0)
            or np.any(np.diff(original_s) <= 0) or np.isnan(physical).any() or np.any(physical < 0)):
        raise ValueError("PipeMotion:JointInput：边界、弧长和速度限制必须逐项对应")
    physical[[0, -1]] = 0.
    effective = np.asarray(feedback.get("effective_boundary_limit", physical), dtype=float).ravel().copy()
    if (len(effective) != len(physical) or np.isnan(effective).any() or np.any(effective < 0)
            or np.any(effective > physical + 64 * np.spacing(np.maximum(1., physical)))):
        raise ValueError("PipeMotion:JointFeedback：反馈上限无效")
    effective[[0, -1]] = 0.
    allow_terminal = feedback.get("terminal_peak_candidate", False)
    if allow_terminal not in (False, True):
        raise ValueError("PipeMotion:JointFeedback：末段模式需要逻辑值")
    requested = np.asarray(feedback.get("retained_boundary_mask", np.ones(len(physical), dtype=bool))).ravel().copy()
    if len(requested) != len(physical) or np.any(~np.isin(requested, [0, 1])):
        raise ValueError("PipeMotion:JointFeedback：保留掩码无效")
    requested = requested.astype(bool)
    requested[[0, -1]] = True
    nominal = lim["F"] / scale
    active = effective < nominal - 64 * np.spacing(max(1., nominal))
    kept = np.unique(np.r_[0, np.flatnonzero(active & requested), len(original_u) - 1])
    block_u, block_s = original_u[kept], original_s[kept]
    S, C = np.diff(block_s), nominal * (1 - 1e-8)
    caps = np.minimum(nominal, effective[kept]) * (1 - 1e-8)
    planning = lim | {"At": lim["At"] * (1 - 1e-8), "Jt": lim["Jt"] * (1 - 1e-8)}
    n = len(S)
    terminal = n > 1 and caps[-2] > 0 and not allow_terminal
    audits, projections = [], []
    candidate_count, solver_calls, best = 0, 0, None

    def evaluate(counts, modes, relax=False, underfill=False):
        nonlocal solver_calls
        solution, info = _solve_candidate(S, caps, C, planning, counts, modes, relax, underfill)
        solver_calls += info["solver_calls"]
        return solution, info

    def record(counts, solution, info):
        nonlocal candidate_count
        candidate_count += 1
        audits.append({"planning_fraction": 1., "cycles": counts.copy(), "exitflag": info["exitflag"],
                       "feasible": solution is not None, "total_variation": np.nan if solution is None else solution["total_variation"],
                       "message": info["message"], "phase": info["phase"], "status": info["status"]})

    for padding in (0, 1, 2, 4, 8):
        counts, temporary = _initial_counts(S, caps, C, planning, terminal)
        counts[:, (0, 2)] += padding
        modes = ["peak"] * n
        if terminal:
            counts[-1, 0], modes[-1] = 0, "decelerate"
        solution, info = evaluate(counts, modes)
        record(counts, solution, info)
        for coordination in range(24):
            if solution is not None or candidate_count >= 128:
                break
            old = counts.copy()
            underfill = coordination % 2 == 0
            projected, projection_info = evaluate(counts, modes, not underfill, underfill)
            if projected is None:
                underfill = not underfill
                projected, projection_info = evaluate(counts, modes, not underfill, underfill)
            if projected is None:
                break
            if underfill:
                rate = projected["middle"].copy()
                rate[rate <= 64 * np.spacing(max(1., C))] = C
                deficit = np.maximum(projected["length_deficit"], 0.)
                deficit[deficit <= 1e-8] = 0.
                counts[:, 1] += _ceil_integer(deficit / (lim["Ts"] * rate))
            else:
                for k in range(n):
                    counts[k, 0] = max(counts[k, 0], _phase_cycles(abs(projected["middle"][k] - projected["boundary"][k]), planning))
                    counts[k, 2] = max(counts[k, 2], _phase_cycles(abs(projected["middle"][k] - projected["boundary"][k + 1]), planning))
            projections.append({"cycles_before": old, "cycles_after": counts.copy(), "boundary": projected["boundary"],
                                "middle": projected["middle"], "exitflag": projection_info["exitflag"],
                                "dynamics_factor": projection_info["dynamics_factor"],
                                "kind": "distance_underfill" if underfill else "dynamics_minimax",
                                "length_deficit": projected["length_deficit"]})
            _check_budget(counts, lim)
            solution, info = evaluate(counts, modes)
            record(counts, solution, info)
        if solution is not None:
            best = solution
            break
    if best is None:
        raise JointScheduleError(f"PipeMotion:JointInfeasible：有限整数候选均不可行：{audits[-1]['phase']}，status={audits[-1]['status']}，{audits[-1]['message']}")
    initial_counts = counts.copy()
    for k in range(n):
        if terminal and k == n - 1:
            continue
        for mode in ("accelerate", "decelerate"):
            trial_modes = modes.copy()
            trial_modes[k] = mode
            solution, info = evaluate(counts, trial_modes)
            record(counts, solution, info)
            tolerance = 1e-8 * max(1., C)
            if solution is not None and (solution["total_variation"] < best["total_variation"] - tolerance
                    or (solution["total_variation"] <= best["total_variation"] + tolerance
                        and solution["boundary_sum"] > best["boundary_sum"] + tolerance)):
                best, modes = solution, trial_modes
    for k in range(n):
        if not counts[k, 1]:
            continue
        trial = counts.copy()
        trial[k, 1] -= 1
        if not trial[k].sum():
            continue
        solution, info = evaluate(trial, modes)
        record(trial, solution, info)
        if solution is not None and solution["total_variation"] <= best["total_variation"] + 1e-8 * max(1., C):
            best, counts = solution, trial
    _check_budget(counts, lim)
    B, M = best["boundary"], best["middle"]
    vs, ve = B[:-1], B[1:]
    peak = np.maximum.reduce([vs, M, ve])
    na, nc, nd = counts.T
    residual = lim["Ts"] * ((na * vs + nd * ve) / 2 + ((na + nd) / 2 + nc) * M) - S
    delta = np.column_stack([np.abs(M - vs), np.abs(ve - M)])
    duration = lim["Ts"] * counts[:, (0, 2)]
    acceleration, jerk = np.zeros((n, 2)), np.zeros((n, 2))
    nonzero = duration > 0
    acceleration[nonzero] = np.pi * delta[nonzero] / (2 * duration[nonzero])
    jerk[nonzero] = np.pi ** 2 * delta[nonzero] / (2 * duration[nonzero] ** 2)
    tolerance = 1e-7 * max(1., C)
    shape_ok = bool(np.all(delta[~nonzero] <= tolerance) and np.all(M >= np.minimum(vs, ve) - tolerance)
                    and (not terminal or (abs(M[-1] - vs[-1]) <= tolerance and ve[-1] <= M[-1] + tolerance)))
    if (not np.isfinite(np.r_[B, M]).all() or np.any(np.r_[B, M] < -1e-8) or np.any(B > caps + tolerance)
            or np.any(peak > C + tolerance) or np.any(np.abs(residual) > 1e-8) or not shape_ok
            or np.any(acceleration > lim["At"] * (1 + 1e-7)) or np.any(jerk > lim["Jt"] * (1 + 1e-7))):
        raise JointScheduleError("PipeMotion:JointClosure：位移、形状或解析运动约束未通过")
    stage_modes = []
    for start, middle, end in zip(vs, M, ve):
        stage_modes.append("constant" if max(start, middle, end) - min(start, middle, end) <= tolerance else
                           "accelerate" if middle >= start - tolerance and end >= middle - tolerance else
                           "decelerate" if middle <= start + tolerance and end <= middle + tolerance else "peak")
    retained = np.zeros(len(original_u), dtype=bool)
    retained[kept] = True
    plan = {"Ts": lim["Ts"], "block_u": block_u, "block_s": block_s, "vs": vs, "ve": ve, "vfs": peak,
            "stage_speed": M, "na": na, "nc": nc, "nd": nd, "length_residual": residual,
            "stage_mode": stage_modes, "stage_mode_constraint": modes,
            "original_boundary_u": original_u, "original_boundary_s": original_s,
            "original_physical_boundary_limit": physical, "original_effective_boundary_limit": effective,
            "kept_boundary_indices": kept + 1, "requested_retained_boundary_mask": requested,
            "retained_boundary_mask": retained, "omitted_physical_active_boundary_indices": np.flatnonzero(active & ~requested) + 1,
            "requires_dense_geometry_validation": True, "removed_boundary_indices": np.flatnonzero(~retained) + 1,
            "physical_boundary_limit": physical[kept], "effective_boundary_limit": effective[kept], "boundary_speed_cap": caps,
            "global_speed_cap": nominal, "solver_speed_cap": C, "nominal_boundary_speed_cap": np.minimum(nominal, effective[kept]),
            "numerical_relative_margin": 1e-8, "numerical_margin_purpose": "floating_point_solver_and_arclength_inversion",
            "nominal_tangent_limits": np.array([lim["At"], lim["Jt"]]),
            "solver_tangent_limits": np.array([planning["At"], planning["Jt"]]), "block_peak_ceiling": np.full(n, C),
            "boundary_reduction": caps - B, "block_boundary_adjusted": (caps - B)[:-1] > tolerance,
            "initial_cycle_counts": initial_counts, "integer_cycle_counts": counts,
            "integer_schedule_passes": 1 + len(projections), "dynamic_projection_history": projections,
            "candidate_integer_schedules": audits, "integer_candidates_evaluated": candidate_count, "lp_solver_calls": solver_calls,
            "initial_planning_boundary": temporary["boundary"], "initial_planning_peak": temporary["peak"],
            "total_velocity_variation": best["total_variation"], "analytic_phase_acceleration": acceleration,
            "analytic_phase_jerk": jerk, "terminal_monotone": terminal, "terminal_peak_candidate_requested": bool(allow_terminal),
            "integer_feasibility_candidate_budget": 128, "stage_optimization_candidate_budget": 3 * n, "shape_ok": shape_ok,
            "peak_adjustment": peak - nominal, "peak_adjustment_reference": "global_speed_cap",
            "compensation_method": "joint_boundary_stage_integer_lp", "time_scale": scale}
    plan["block_boundary_adjusted"] = ((caps - B)[:-1] > tolerance) | ((caps - B)[1:] > tolerance)
    pipe_motion_trace("add", "joint_schedule", time.monotonic() - started)
    return plan
