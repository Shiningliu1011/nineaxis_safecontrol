#!/usr/bin/env python3
"""Ticket 18 isolation experiment: the elastic-QP admission trap.

Read-only w.r.t. product code.  Uses only the existing production kernel
(``work.jax_control_facade.JaxControlLoop`` -> ``work.jax_kernel_factory`` ->
``cbfpy`` -> ``qpax``), plus a direct ``qpax`` call on a hand-built QP.

Scenarios
---------
A  production-style path tracking, no obstacles  -> slack noise floor
B  one obstacle placed so the measured clearance starts at -5 mm
C  same obstacle geometry on the bare ``tracking_step`` entry point, 5 ticks
D  a hand-built *provably infeasible* QP solved by ``qpax.solve_qp`` (hard)
   and ``qpax.solve_qp_elastic`` (elastic) to compare what the solver returns

Run (from the repository root):

    PYTHONPATH=portable_oscbf/vendor/dpax JAX_PLATFORMS=cpu \
    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
    python3 docs/planning/oscbf-reuse/research/18-elastic-qp-admission/probe_elastic_qp_admission.py
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[5]

# Make the portable package importable the same way tests/_path_setup.py does.
for _entry in (REPO_ROOT / "portable_oscbf", REPO_ROOT / "portable_oscbf" / "work"):
    if str(_entry) not in sys.path:
        sys.path.insert(0, str(_entry))

import jax  # noqa: E402

jax.config.update("jax_enable_x64", True)

import jax.numpy as jnp  # noqa: E402
import numpy as np  # noqa: E402
import qpax  # noqa: E402

from work.ik_data_loader import load_repository_trajectory  # noqa: E402
from work.jax_control_facade import JaxControlLoop  # noqa: E402
from work.path_following import PathFollowingConfig  # noqa: E402

TRAJECTORY = REPO_ROOT / "data" / "nurbs" / "ik_input.mat"
OUT_DIR = Path(__file__).resolve().parent
MAX_OBSTACLES = 8

# Production profile, config/oscbf_controller.yaml (2026-09-12):
#   dt=0.01 dt_path=0.01 publish=100Hz kp_pos=160 kp_orient=10 kp_joint=0.45
#   w_pos=40 w_orient=10 w_joint=0.1 temporal_lambda=0.2 enable_x64=true
#   solver_tol=0.001 task_mode=tool_axis_5d nullspace_speed_limit=0.18
#   damping=0.05 reference_lead_m=0.01
PROD = dict(
    dt=0.01,
    dt_path=0.01,
    kp_pos=160.0,
    kp_orient=10.0,
    kp_joint=0.45,
    w_pos=40.0,
    w_orient=10.0,
    w_joint=0.1,
    temporal_lambda=0.2,
    solver_tol=1e-3,
    nullspace_speed_limit=0.18,
    damping=0.05,
)

# portable_oscbf/config/nineaxis.yaml: dynamic_obstacles.d_safe = 0.08
# portable_oscbf/config/nineaxis.yaml: dynamic_obstacles.cbf_alpha = 1.5
OBS_D_SAFE = 0.08
OBS_ALPHA = 1.5


# ----------------------------------------------------------------------------
# helpers
# ----------------------------------------------------------------------------
def empty_obstacles():
    return dict(
        obs_pos=np.zeros((MAX_OBSTACLES, 3)),
        obs_radii=np.zeros(MAX_OBSTACLES),
        obs_enabled=np.zeros(MAX_OBSTACLES),
        obs_d_safe=np.full(MAX_OBSTACLES, OBS_D_SAFE),
        obs_vel=np.zeros((MAX_OBSTACLES, 3)),
        obs_radius_dot=np.zeros(MAX_OBSTACLES),
        obs_alpha=np.full(MAX_OBSTACLES, OBS_ALPHA),
    )


def one_obstacle(center, radius):
    obs = empty_obstacles()
    obs["obs_pos"][0] = np.asarray(center, dtype=float)
    obs["obs_radii"][0] = float(radius)
    obs["obs_enabled"][0] = 1.0
    return obs


_RAW_H_CACHE = {}


def _make_raw_h(loop):
    def evaluate(q, obs_pos, obs_radii, obs_enabled, obs_d_safe, obs_vel,
                 obs_radius_dot, obs_alpha, u_prev):
        return loop._cbf.h_2(q, obs_pos, obs_radii, obs_enabled, obs_d_safe,
                             obs_vel, obs_radius_dot, obs_alpha, u_prev)

    return jax.jit(evaluate)


def raw_h(loop, q, obs, u_prev=None):
    """Evaluate the config's raw barrier vector h(q) through the public CBF API.

    This is the same expression the kernel evaluates for ``h_vals`` (the kernel
    divides the cbfpy QP row bound by ``obstacle_h_baseline_alpha``; with a
    velocity-level model ``f == 0`` the row bound is exactly ``alpha(h)``).

    Jitted once per loop: an eager call would compile the internal
    ``lax.cond`` branches on every invocation and exhaust LLVM memory.
    """
    fn = _RAW_H_CACHE.get(id(loop))
    if fn is None:
        fn = _make_raw_h(loop)
        _RAW_H_CACHE[id(loop)] = fn
    u_prev = np.zeros(9) if u_prev is None else u_prev
    return np.asarray(
        fn(
            jnp.asarray(q),
            jnp.asarray(obs["obs_pos"]),
            jnp.asarray(obs["obs_radii"]),
            jnp.asarray(obs["obs_enabled"]),
            jnp.asarray(obs["obs_d_safe"]),
            jnp.asarray(obs["obs_vel"]),
            jnp.asarray(obs["obs_radius_dot"]),
            jnp.asarray(obs["obs_alpha"]),
            jnp.asarray(u_prev),
        )
    )


def row_slices(loop):
    cfg = loop._config
    n_joint = 2 * 9
    n_self = int(cfg.self_collision_pairs.shape[0])
    n_obs = int(cfg.num_obstacle_constraints)
    return {
        "joint": (0, n_joint),
        "self_collision": (n_joint, n_joint + n_self),
        "obstacle": (n_joint + n_self, n_joint + n_self + n_obs),
        "singularity": (n_joint + n_self + n_obs, n_joint + n_self + n_obs + 1),
    }


def group_minima(loop, h):
    out = {}
    for name, (a, b) in row_slices(loop).items():
        out[name] = float(np.min(h[a:b]))
    return out


def build_loop():
    loop = JaxControlLoop(
        dt=PROD["dt"],
        dt_path=PROD["dt_path"],
        w_pos=PROD["w_pos"],
        w_orient=PROD["w_orient"],
        w_joint=PROD["w_joint"],
        temporal_lambda=PROD["temporal_lambda"],
        enable_x64=True,
        solver_tol=PROD["solver_tol"],
        task_mode="tool_axis_5d",
    )
    trajectory = load_repository_trajectory(str(TRAJECTORY))
    loop.configure_path(
        trajectory.path_geometry(),
        PathFollowingConfig(
            reference_lead_m=0.01,
            maximum_tool_axis_speed_rad_s=2.0,
        ),
    )
    loop.init_cbf()
    return loop


def tick(loop, *, q, path_state, obs, u_prev):
    return loop.path_tracking_step(
        q=np.asarray(q, dtype=float),
        path_state=np.asarray(path_state, dtype=float),
        kp_pos=PROD["kp_pos"],
        kp_orient=PROD["kp_orient"],
        kp_joint=PROD["kp_joint"],
        q_des=np.asarray(q, dtype=float),
        nullspace_speed_limit=PROD["nullspace_speed_limit"],
        damping=PROD["damping"],
        u_safe_prev=np.asarray(u_prev, dtype=float),
        **obs,
    )


def record(loop, q, result, obs, label):
    h = raw_h(loop, q, obs)
    row = {
        "label": label,
        "min_h": float(np.min(h)),
        "min_h_by_group": group_minima(loop, h),
        "delta_slack": float(result.delta_slack),
        "qp_ok": bool(result.qp_ok),
        "qp_iterations": int(loop.last_qp_iterations),
        "primal_residual_row_units": float(loop.last_qp_primal_residual),
        "terminal_kkt_residual": float(loop.last_qp_terminal_kkt_residual),
        "terminal_kkt_accepted": bool(loop.last_qp_terminal_kkt_accepted),
        "min_obs_dist_m": float(result.min_obs_dist),
        "u_max_abs": float(np.max(np.abs(result.u_safe))),
        "u_norm": float(np.linalg.norm(result.u_safe)),
        "q_next_moved_norm": float(np.linalg.norm(result.q_next - q)),
        "q_next": [float(v) for v in result.q_next],
        "u_safe": [float(v) for v in result.u_safe],
        "feedrate_m_s": float(result.feedrate_m_s),
        "limiting_reason_code": int(result.limiting_reason_code),
    }
    return row


def print_row(row):
    g = row["min_h_by_group"]
    print(
        f"{row['label']:>10}  min_h={row['min_h']:+.6f}  "
        f"(joint={g['joint']:+.4f} self={g['self_collision']:+.6f} "
        f"obs={g['obstacle']:+.6f} sing={g['singularity']:+.4f})  "
        f"slack={row['delta_slack']:.3e}  qp_ok={row['qp_ok']!s:<5} "
        f"primal_res={row['primal_residual_row_units']:.3e}  "
        f"|u|={row['u_norm']:.5f}  |dq|={row['q_next_moved_norm']:.6f}"
    )


# ----------------------------------------------------------------------------
# scenario A: slack noise floor on a clean production-style run
# ----------------------------------------------------------------------------
def scenario_a(loop, n_ticks=60):
    q = np.zeros(9)
    path_state = loop.initial_path_state()
    u_prev = np.zeros(9)
    obs = empty_obstacles()
    rows = []
    for i in range(n_ticks):
        result = tick(loop, q=q, path_state=path_state, obs=obs, u_prev=u_prev)
        rows.append(record(loop, q, result, obs, label=f"A+{i:03d}"))
        q = np.asarray(result.q_next, dtype=float)
        path_state = np.asarray(result.path_state, dtype=float)
        u_prev = np.asarray(result.u_safe, dtype=float)
    summary = {
        "ticks": n_ticks,
        "max_delta_slack": max(r["delta_slack"] for r in rows),
        "min_h_over_run": min(r["min_h"] for r in rows),
        "all_qp_ok": all(r["qp_ok"] for r in rows),
        "max_primal_residual_row_units": max(
            r["primal_residual_row_units"] for r in rows
        ),
    }
    return rows, summary


# ----------------------------------------------------------------------------
# scenario B: measured state already inside the obstacle margin
# ----------------------------------------------------------------------------
def place_obstacle_at_target_h(loop, q, target_h=-0.005):
    """Slide a 1 mm sphere along a ray from a robot collision sphere outward
    until the measured obstacle clearance h is as close as possible to
    ``target_h`` (default -5 mm, i.e. 5 mm inside the 80 mm obstacle margin).
    """
    data = np.asarray(loop.robot.environment_collision_data(jnp.asarray(q)))
    anchor = data[np.argmax(data[:, 2])][:3]  # highest sphere, world frame
    direction = anchor - np.asarray(loop.robot.ee_position(jnp.asarray(q)))
    norm = np.linalg.norm(direction)
    direction = direction / norm if norm > 1e-9 else np.array([0.0, 0.0, 1.0])
    offsets = np.linspace(0.0, 0.40, 1601)
    q_j = jnp.asarray(q)

    def min_obstacle_h(offset):
        center = anchor + offset * direction
        obs_pos = jnp.zeros((MAX_OBSTACLES, 3)).at[0].set(center)
        obs_radii = jnp.zeros(MAX_OBSTACLES).at[0].set(0.001)
        obs_enabled = jnp.zeros(MAX_OBSTACLES).at[0].set(1.0)
        obs_d_safe = jnp.full(MAX_OBSTACLES, OBS_D_SAFE)
        rows = loop._config._compute_dcol_obstacle_constraints(
            q_j, obs_pos, obs_radii, obs_enabled, obs_d_safe)
        return jnp.min(rows)

    values = np.asarray(
        jax.jit(jax.vmap(min_obstacle_h))(jnp.asarray(offsets)))
    index = int(np.argmin(np.abs(values - target_h)))
    center = anchor + offsets[index] * direction
    return center, float(values[index]), float(offsets[index])


def scenario_b(loop, target_h=-0.005, n_ticks=5):
    q = np.zeros(9)
    center, measured_min_h, offset = place_obstacle_at_target_h(loop, q, target_h)
    obs = one_obstacle(center, 0.001)
    path_state = loop.initial_path_state()
    u_prev = np.zeros(9)
    rows = []
    for i in range(n_ticks):
        result = tick(loop, q=q, path_state=path_state, obs=obs, u_prev=u_prev)
        rows.append(record(loop, q, result, obs, label=f"B+{i:03d}"))
        q = np.asarray(result.q_next, dtype=float)
        path_state = np.asarray(result.path_state, dtype=float)
        u_prev = np.asarray(result.u_safe, dtype=float)
    meta = {
        "obstacle_center_m": [float(v) for v in center],
        "obstacle_radius_m": 0.001,
        "obs_d_safe_m": OBS_D_SAFE,
        "obs_alpha": OBS_ALPHA,
        "ray_offset_m": offset,
        "measured_min_h_at_start_m": measured_min_h,
        "target_h_m": target_h,
    }
    return rows, meta


def scenario_initial_r_margin(loop, q):
    """The clearance without an obstacle, for reference."""
    h = raw_h(loop, q, empty_obstacles())
    return group_minima(loop, h)


# ----------------------------------------------------------------------------
# scenario C: a real conflict -- the required escape speed leaves the
# actuator box, so no u satisfies every row and the obstacle row
# ----------------------------------------------------------------------------
def scenario_c(loop, radii=None, n_ticks=8):
    """Grow an enclosing obstacle sphere until the CBF rows stop being
    satisfiable inside the joint-velocity box, then watch what the kernel
    returns."""
    q0 = np.zeros(9)
    anchor = np.asarray(loop.robot.ee_position(jnp.asarray(q0)), dtype=float)
    if radii is None:
        radii = [0.02, 0.05, 0.10, 0.20, 0.30, 0.50, 0.80, 1.20, 2.00, 3.00]
    path_state0 = loop.initial_path_state()
    scan = []
    for radius in radii:
        obs = one_obstacle(anchor, radius)
        result = tick(loop, q=q0, path_state=path_state0, obs=obs,
                      u_prev=np.zeros(9))
        h = raw_h(loop, q0, obs)
        scan.append({
            "radius_m": float(radius),
            "min_h_m": float(np.min(h)),
            "min_obstacle_h_m": group_minima(loop, h)["obstacle"],
            "delta_slack_row_units": float(result.delta_slack),
            "qp_ok": bool(result.qp_ok),
            "u_norm": float(np.linalg.norm(result.u_safe)),
            "q_next_moved_norm": float(np.linalg.norm(result.q_next - q0)),
            "qp_iterations": int(loop.last_qp_iterations),
        })
    # smallest radius whose single step already needs relaxation
    chosen = next((item for item in scan
                   if item["delta_slack_row_units"] > 1e-6), scan[-1])

    q = q0
    path_state = loop.initial_path_state()
    u_prev = np.zeros(9)
    obs = one_obstacle(anchor, chosen["radius_m"])
    rows = []
    for i in range(n_ticks):
        result = tick(loop, q=q, path_state=path_state, obs=obs, u_prev=u_prev)
        rows.append(record(loop, q, result, obs, label=f"C+{i:03d}"))
        q = np.asarray(result.q_next, dtype=float)
        path_state = np.asarray(result.path_state, dtype=float)
        u_prev = np.asarray(result.u_safe, dtype=float)
    meta = {
        "obstacle_center_m": [float(v) for v in anchor],
        "chosen_radius_m": chosen["radius_m"],
        "obs_d_safe_m": OBS_D_SAFE,
        "obs_alpha": OBS_ALPHA,
        "joint_max_velocities_rad_s": [
            float(v) for v in np.asarray(loop.robot.joint_max_velocities)
        ],
        "scan": scan,
    }
    return rows, meta


# ----------------------------------------------------------------------------
# scenario E: barrier row vs actuator box -- a joint measured outside the
# joint-limit envelope
# ----------------------------------------------------------------------------
def scenario_e(loop, overshoots=(0.0, 0.005, 0.02, 0.05, 0.20)):
    lower = np.asarray(loop.robot.joint_lower_limits, dtype=float)
    vmax = np.asarray(loop.robot.joint_max_velocities, dtype=float)
    margin = float(loop._config.joint_limit_cbf_margin)
    alpha = float(loop._config.obstacle_h_baseline_alpha)
    path_state0 = loop.initial_path_state()
    obs = empty_obstacles()
    out = []
    for overshoot in overshoots:
        q0 = np.zeros(9)
        q0[0] = float(lower[0]) - float(overshoot)
        result = tick(loop, q=q0, path_state=path_state0, obs=obs,
                      u_prev=np.zeros(9))
        h = raw_h(loop, q0, obs)
        # The config's lower joint-limit row is h = q - lower - margin, so a
        # measured q below `lower` makes it negative.
        h_j1_lower = float(q0[0] - lower[0] - margin)
        out.append({
            "overshoot_below_lower_limit_rad": float(overshoot),
            "h_j1_lower_rad": h_j1_lower,
            "required_escape_rate_rad_s": alpha * -h_j1_lower,
            "j1_vmax_rad_s": float(vmax[0]),
            "min_h": float(np.min(h)),
            "u_j1_rad_s": float(result.u_safe[0]),
            "q_next_j1": float(result.q_next[0]),
            "delta_slack_row_units": float(result.delta_slack),
            "qp_ok": bool(result.qp_ok),
            "primal_residual_row_units": float(loop.last_qp_primal_residual),
        })
    return out


# ----------------------------------------------------------------------------
# scenario F: both collision distance kernels saturate at zero
# ----------------------------------------------------------------------------
def scenario_f(loop):
    from work import dpax_collision as D
    from work.obb_collision_model import (
        OBB_HALF_EXTENTS_M,
        OBB_LOCAL_CENTERS_M,
        OBB_LOCAL_ROTATIONS,
    )

    q = jnp.zeros(9)
    ee = np.asarray(loop.robot.ee_position(q), dtype=float)

    sphere = []
    for radius in (0.0, 0.001, 0.05, 0.30, 0.80):
        obs_pos = jnp.zeros((MAX_OBSTACLES, 3)).at[0].set(jnp.asarray(ee))
        obs_radii = jnp.zeros(MAX_OBSTACLES).at[0].set(radius)
        distances = np.asarray(
            D.obb_sphere_distances(q, obs_pos, obs_radii))[:, 0]
        sphere.append({
            "sphere_radius_m": float(radius),
            "min_surface_distance_m": float(np.min(distances)),
            "implied_min_h_m": float(np.min(distances)) - OBS_D_SAFE,
        })

    transforms = D.link_transforms(q)
    index = 0
    pairs = []
    for offset in (0.0, 0.001, 0.01, 0.05, 0.20):
        shifted = transforms[index].at[0:3, 3].add(
            jnp.asarray([offset, 0.0, 0.0]))
        value = D._obb_pair_distance_impl(
            transforms[index],
            OBB_LOCAL_CENTERS_M[index],
            OBB_LOCAL_ROTATIONS[index],
            OBB_HALF_EXTENTS_M[index],
            shifted,
            OBB_LOCAL_CENTERS_M[index],
            OBB_LOCAL_ROTATIONS[index],
            OBB_HALF_EXTENTS_M[index],
        )
        pairs.append({
            "x_offset_m": float(offset),
            "obb_half_extent_x_m": float(OBB_HALF_EXTENTS_M[index][0]),
            "returned_distance_m": float(value),
        })
    return {"obb_sphere": sphere, "obb_obb_same_link": pairs}


# ----------------------------------------------------------------------------
# scenario D: hard vs elastic qpax on a provably infeasible QP
# ----------------------------------------------------------------------------
def scenario_d():
    Q = jnp.eye(1)
    q = jnp.zeros(1)
    G = jnp.array([[1.0], [-1.0]])
    h = jnp.array([-1.0, -1.0])  # u >= 1 and u <= -1 : empty feasible set
    x, s, z, y, converged, iters = qpax.solve_qp(Q, q, jnp.zeros((0, 1)),
                                                 jnp.zeros(0), G, h)
    xe, te, s1, s2, z1, z2, conv_e, iters_e = qpax.solve_qp_elastic(
        Q, q, G, h, 1e5)
    return {
        "hard": {
            "x": [float(v) for v in np.asarray(x)],
            "slack_s": [float(v) for v in np.asarray(s)],
            "dual_z": [float(v) for v in np.asarray(z)],
            "converged": int(converged),
            "iterations": int(iters),
            "row_violation_Gx_minus_h": [
                float(v) for v in np.asarray(G @ x - h)
            ],
        },
        "elastic": {
            "x": [float(v) for v in np.asarray(xe)],
            "t": [float(v) for v in np.asarray(te)],
            "s1_equals_t": [float(v) for v in np.asarray(s1)],
            "s2": [float(v) for v in np.asarray(s2)],
            "converged": int(conv_e),
            "iterations": int(iters_e),
            "row_violation_Gx_minus_h": [
                float(v) for v in np.asarray(G @ xe - h)
            ],
        },
    }


def lgh_reachability(loop, q, obs, u_prev=None):
    """Row-wise |Lg h|_1 for the CBF rows: how much control authority each row has.

    A row with |Lg h|_1 == 0 cannot be influenced by any u, so if its bound is
    negative it is unsatisfiable no matter what the actuator box allows.
    """
    u_prev = np.zeros(9) if u_prev is None else u_prev
    args = (
        jnp.asarray(obs["obs_pos"]), jnp.asarray(obs["obs_radii"]),
        jnp.asarray(obs["obs_enabled"]), jnp.asarray(obs["obs_d_safe"]),
        jnp.asarray(obs["obs_vel"]), jnp.asarray(obs["obs_radius_dot"]),
        jnp.asarray(obs["obs_alpha"]), jnp.asarray(u_prev),
        None, None, None, 0.0, None, None,
    )
    g_matrix = np.asarray(
        loop._cbf.G_qp(jnp.asarray(q), jnp.zeros(9), *args))
    lgh = np.abs(g_matrix[: loop._cbf.num_cbf]).sum(axis=1)
    slices = row_slices(loop)
    return {
        "min_lgh_l1": float(np.min(lgh)),
        "max_lgh_l1": float(np.max(lgh)),
        "rows_with_zero_lgh": int(np.sum(lgh < 1e-9)),
        "obstacle_rows_lgh_l1": [
            float(v) for v in lgh[slices["obstacle"][0]:slices["obstacle"][1]]
        ],
    }


# ----------------------------------------------------------------------------
# scenario G: how much does the linear penalty change the outcome?
# ----------------------------------------------------------------------------
def scenario_g(loop, penalties=(1e3, 1e5, 1e7, 1e9), radius=0.05):
    """Same conflict as scenario C, four values of ``cbf_relaxation_penalty``.

    Only the in-process attribute is changed (the coefficient is read at trace
    time), then restored.  No repository file, config or threshold is edited.
    """
    q0 = np.zeros(9)
    anchor = np.asarray(loop.robot.ee_position(jnp.asarray(q0)), dtype=float)
    obs = one_obstacle(anchor, radius)
    path_state0 = loop.initial_path_state()
    original = loop._cbf.cbf_relaxation_penalty
    out = []
    try:
        for penalty in penalties:
            loop._cbf.cbf_relaxation_penalty = float(penalty)
            # The coefficient is read while tracing, and JAX keys its compile
            # cache on input avals only.  Without an explicit cache clear the
            # first compiled program (with the original penalty) is reused and
            # the experiment silently measures nothing.
            loop._path_tracking_fn.clear_cache()
            result = tick(loop, q=q0, path_state=path_state0, obs=obs,
                          u_prev=np.zeros(9))
            out.append({
                "cbf_relaxation_penalty": float(penalty),
                "jit_cache_cleared": True,
                "delta_slack_row_units": float(result.delta_slack),
                "primal_residual_row_units": float(
                    loop.last_qp_primal_residual),
                "u_norm": float(np.linalg.norm(result.u_safe)),
                "q_next_moved_norm": float(
                    np.linalg.norm(result.q_next - q0)),
                "qp_ok": bool(result.qp_ok),
                "qp_iterations": int(loop.last_qp_iterations),
            })
    finally:
        loop._cbf.cbf_relaxation_penalty = original
    return out, lgh_reachability(loop, q0, obs)


def gate_table(rows_by_scenario, epsilons=(1e-9, 1e-6, 1e-3, 1e-2)):
    """Post-hoc: what would ``max(slack) <= eps`` have done per tick?"""
    out = {}
    for name, rows in rows_by_scenario.items():
        out[name] = {
            "ticks": len(rows),
            "max_slack_over_run": max(r["delta_slack"] for r in rows),
            "freeze_ticks_if_eps": {
                f"{eps:g}": sum(1 for r in rows if r["delta_slack"] > eps)
                for eps in epsilons
            },
            "all_qp_ok": all(r["qp_ok"] for r in rows),
        }
    return out


def main():
    print(f"jax={jax.__version__} backend={jax.default_backend()} "
          f"x64={jax.config.jax_enable_x64} "
          f"threads(OPENBLAS={os.environ.get('OPENBLAS_NUM_THREADS')}, "
          f"OMP={os.environ.get('OMP_NUM_THREADS')})")
    loop = build_loop()
    print(f"relax_cbf={loop._config.relax_cbf} "
          f"cbf_relaxation_penalty={loop._config.cbf_relaxation_penalty} "
          f"solver_tol={loop._config.solver_tol} "
          f"alpha=({loop._config.obstacle_h_baseline_alpha}*h) "
          f"enable_rate_limit={loop.enable_rate_limit} "
          f"num_cbf={loop._cbf.num_cbf} rows={row_slices(loop)}")
    print(f"n_obb_links={loop._config.num_obb_links} "
          f"num_obstacle_constraints={loop._config.num_obstacle_constraints} "
          f"d_safe_collision={loop._config.d_safe_collision}")

    print("\n=== scenario A: no obstacles, 60 ticks ===")
    rows_a, summary_a = scenario_a(loop)
    for row in rows_a[:3] + rows_a[-2:]:
        print_row(row)
    print(f"  summary: {json.dumps(summary_a, indent=2)}")

    print("\n=== scenario B: measured clearance starts at ~-5 mm ===")
    rows_b, meta_b = scenario_b(loop)
    print(f"  placement: {json.dumps(meta_b, indent=2)}")
    for row in rows_b:
        print_row(row)

    print("\n=== reference: h without obstacle at q=0 ===")
    print(f"  {json.dumps(scenario_initial_r_margin(loop, np.zeros(9)), indent=2)}")

    print("\n=== scenario C: enclosing obstacle, bisected by radius ===")
    rows_c, meta_c = scenario_c(loop)
    for item in meta_c["scan"]:
        print(f"  r={item['radius_m']:5.2f} m  min_h={item['min_h_m']:+.6f}  "
              f"min_h_obs={item['min_obstacle_h_m']:+.6f}  "
              f"slack={item['delta_slack_row_units']:.3e}  "
              f"qp_ok={item['qp_ok']!s:<5} |u|={item['u_norm']:.5f} "
              f"|dq|={item['q_next_moved_norm']:.6f}")
    for row in rows_c:
        print_row(row)

    print("\n=== scenario E: joint measured outside the joint-limit envelope ===")
    rows_e = scenario_e(loop)
    print(json.dumps(rows_e, indent=2))

    print("\n=== scenario F: collision distance kernels saturate at zero ===")
    f = scenario_f(loop)
    print(json.dumps(f, indent=2))

    print("\n=== scenario D: infeasible QP, hard vs elastic qpax ===")
    d = scenario_d()
    print(json.dumps(d, indent=2))

    print("\n=== scenario G: sensitivity to the slack penalty ===")
    rows_g, reach_g = scenario_g(loop)
    print(json.dumps(rows_g, indent=2))
    print("row control authority |Lg h|_1 for the same state:")
    print(json.dumps(reach_g, indent=2))

    print("\n=== gate table: what max(slack) <= eps would have done ===")
    gates = gate_table({
        "A no obstacles": rows_a,
        "B clearance -5mm": rows_b,
        "C enclosing obstacle": rows_c,
    })
    print(json.dumps(gates, indent=2))

    payload = {
        "environment": {
            "jax": jax.__version__,
            "backend": jax.default_backend(),
            "enable_x64": bool(jax.config.jax_enable_x64),
            "openblas_num_threads": os.environ.get("OPENBLAS_NUM_THREADS"),
            "omp_num_threads": os.environ.get("OMP_NUM_THREADS"),
        },
        "production_profile": PROD,
        "config": {
            "relax_cbf": bool(loop._config.relax_cbf),
            "cbf_relaxation_penalty": float(loop._config.cbf_relaxation_penalty),
            "solver_tol": float(loop._config.solver_tol),
            "obstacle_h_baseline_alpha": float(loop._config.obstacle_h_baseline_alpha),
            "d_safe_collision_m": float(loop._config.d_safe_collision),
            "num_cbf": int(loop._cbf.num_cbf),
            "row_slices": {k: list(v) for k, v in row_slices(loop).items()},
            "enable_rate_limit": bool(loop.enable_rate_limit),
        },
        "scenario_a": {"rows": rows_a, "summary": summary_a},
        "scenario_b": {"rows": rows_b, "meta": meta_b},
        "scenario_c": {"rows": rows_c, "meta": meta_c},
        "scenario_d": d,
        "scenario_e": rows_e,
        "scenario_f": f,
        "scenario_g": {"rows": rows_g, "lgh_reachability": reach_g},
        "gate_table": gates,
    }
    out = OUT_DIR / "results.json"
    out.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(f"\nwrote {out}")


if __name__ == "__main__":
    raise SystemExit(main())
