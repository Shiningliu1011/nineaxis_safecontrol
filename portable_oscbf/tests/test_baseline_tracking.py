#!/usr/bin/env python3
"""M5 acceptance: baseline whole-kernel path tracking (reference semantics).

Runs the copied whole-kernel controller unchanged (relax_cbf=False, joint
mid-point null space, no controller-internal clipping) for >= 3000 steps on
the repository trajectory, asserts the M5 gates, and stores the input/output
sequence in ``output/baseline_tracking.npz`` for the M6 modular-JIT
equivalence test.

Note: M7 switched the controller to the elastic QP (relax_cbf=True) with
controller-internal clipping and DCOL obstacle rows.  Re-running this test
now regenerates the baseline under the current (M7+) semantics and remains a
closed-loop regression gate; the M6 equivalence gate skips when the M7
semantics are active.
"""

from __future__ import annotations

from pathlib import Path
from time import perf_counter

import _path_setup  # noqa: F401
import numpy as np
import pytest

from work.ik_data_loader import load_repository_trajectory
from work.jax_control_facade import JaxControlLoop
from work.nineaxis_kinematics import NineaxisKinematics
from work.path_following import PathFollowingConfig


REPO_ROOT = Path(__file__).resolve().parents[2]
CURRENT_TRAJECTORY = REPO_ROOT / "data" / "nurbs" / "ik_input.mat"
OUTPUT_PATH = REPO_ROOT / "output" / "baseline_tracking.npz"
NUM_STEPS = 3000
STEADY_START = 100


def _work_start_configuration(trajectory) -> np.ndarray:
    """IK the trajectory start pose into a joint configuration (M5 baseline).

    Mirrors the reference runner: the controller starts from the IK solution
    of the path start, so arc-length tracking only handles small errors.
    """

    target_position = trajectory.pos_world_at(0.0)
    target_rotation = trajectory.orientation_at(0.0)
    kinematics = NineaxisKinematics()
    rng = np.random.default_rng(0)
    seeds = [np.zeros(9)]
    lower = np.asarray(kinematics.joint_limits.q_min)
    upper = np.asarray(kinematics.joint_limits.q_max)
    for _ in range(24):
        seeds.append(rng.uniform(lower + 0.1, upper - 0.1))

    best_q = None
    best_error = float("inf")
    for seed in seeds:
        candidate = kinematics.ik(
            target_position, target_rotation, q_init=seed)
        if candidate is None:
            continue
        error = np.linalg.norm(
            kinematics.ee_position(candidate) - target_position)
        if error < best_error:
            best_error = error
            best_q = candidate
    assert best_q is not None, "IK failed for every seed"
    assert best_error < 1e-3, (
        f"IK start position error {best_error * 1000:.3f} mm")
    return best_q


def _run_baseline() -> tuple[dict, float, float]:
    trajectory = load_repository_trajectory(str(CURRENT_TRAJECTORY))
    geometry = trajectory.path_geometry()
    initial_q = _work_start_configuration(trajectory)

    loop = JaxControlLoop(dt=0.002, temporal_lambda=0.2, enable_x64=True)
    loop.configure_path(geometry, PathFollowingConfig())
    loop.init_cbf()

    q = initial_q.copy()
    path_state = loop.initial_path_state()
    common = dict(
        kp_pos=50.0,
        kp_orient=10.0,
        kp_joint=0.45,
        q_des=initial_q,
        nullspace_speed_limit=0.18,
        damping=1e-3,
    )

    q_sequence = [q.copy()]
    path_state_sequence = [path_state.copy()]
    u_safe_sequence = []
    err_sequence = []
    min_dist_sequence = []
    qp_ok_sequence = []

    step_times = []
    for _ in range(NUM_STEPS):
        step_start = perf_counter()
        result = loop.path_tracking_step(q=q, path_state=path_state, **common)
        step_times.append(perf_counter() - step_start)
        if _ == 0:
            first_call_s = step_times[0]
        q = np.asarray(result.q_next)
        path_state = np.asarray(result.path_state)
        q_sequence.append(q.copy())
        path_state_sequence.append(path_state.copy())
        u_safe_sequence.append(np.asarray(result.u_safe))
        err_sequence.append(np.asarray(result.err_6d))
        min_dist_sequence.append(float(result.min_obs_dist))
        qp_ok_sequence.append(bool(result.qp_ok))
    per_step_ms = float(np.mean(step_times[1:]) * 1000.0)

    arrays = {
        "q_sequence": np.stack(q_sequence),
        "path_state_sequence": np.stack(path_state_sequence),
        "u_safe_sequence": np.stack(u_safe_sequence),
        "err_6d_sequence": np.stack(err_sequence),
        "min_dist_sequence": np.asarray(min_dist_sequence),
        "qp_ok_sequence": np.asarray(qp_ok_sequence),
        "initial_q": initial_q,
        "per_step_ms_steady": np.asarray(per_step_ms),
        "first_call_s": np.asarray(first_call_s),
        # Semantic tag for downstream gates (M6): the M5 hard-QP baseline and
        # the post-M7 elastic-QP baseline must never be compared against each
        # other.  Bump this string whenever the controller semantics change.
        "semantics": np.asarray("m7_elastic_qp_v1"),
    }
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    np.savez(OUTPUT_PATH, **arrays)
    return arrays, first_call_s, per_step_ms


def test_baseline_path_tracking_gates_and_artifacts():
    arrays, first_call_s, per_step_ms = _run_baseline()

    q_sequence = arrays["q_sequence"]
    u_safe = arrays["u_safe_sequence"]
    err_6d = arrays["err_6d_sequence"]
    min_dist = arrays["min_dist_sequence"]
    qp_ok = arrays["qp_ok_sequence"]

    # AC5.1: 100% QP success.
    assert qp_ok.shape == (NUM_STEPS,)
    assert np.all(qp_ok), f"qp_ok failures: {np.sum(~qp_ok)}"

    # AC5.2: no NaN/Inf in state, command, error or margin.
    assert np.all(np.isfinite(q_sequence))
    assert np.all(np.isfinite(u_safe))
    assert np.all(np.isfinite(err_6d))
    assert np.all(np.isfinite(min_dist))

    # AC5.3: steady-state position error < 1 mm (first 100 steps excluded).
    steady_position_error = np.linalg.norm(
        err_6d[STEADY_START:, :3], axis=1)
    assert np.max(steady_position_error) < 1e-3, (
        f"steady max position error {np.max(steady_position_error) * 1000:.3f} mm")

    # AC5.4: positive dynamic margin throughout.
    assert np.all(min_dist > 0.0), (
        f"non-positive min distance at {np.where(min_dist <= 0)[0][:5]}")

    # AC5.5 / AC5.6: warm-up budget recorded and artifact saved.  The
    # 30-second warm-up target is a M6 (modular JIT) gate; the whole-kernel
    # reference implementation intentionally compiles slower (guide L94), so
    # M5 records the measured value against a generous 120 s budget.
    assert OUTPUT_PATH.is_file()
    assert first_call_s < 120.0, (
        f"first JIT call took {first_call_s:.1f}s (limit 120s in M5)")
    print(f"first_call_s={first_call_s:.2f} per_step_ms={per_step_ms:.3f}")


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
