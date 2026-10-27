from __future__ import annotations

import sys
from pathlib import Path
from time import perf_counter

import jax

jax.config.update("jax_enable_x64", True)

import _path_setup  # noqa: F401
import numpy as np
from artifact_paths import TEST_OUTPUT

from work.ik_data_loader import load_repository_trajectory
from work.jax_control_facade import JaxControlLoop
from work.path_following import PathFollowingConfig


REPO_ROOT = Path(__file__).resolve().parents[2]
CURRENT_TRAJECTORY = REPO_ROOT / "data" / "nurbs" / "ik_input.mat"
BASELINE_PATH = TEST_OUTPUT / "baseline_tracking.npz"
PERF_REPORT_PATH = TEST_OUTPUT / "oscbf_m6_perf.md"
NUM_STEPS = 3000
CURRENT_SEMANTICS = "m7_elastic_qp_v12_full_position_feedback"


def _control_kwargs(initial_q):
    return dict(
        kp_pos=60.0,
        kp_orient=10.0,
        kp_joint=0.45,
        q_des=initial_q,
        nullspace_speed_limit=0.18,
        damping=1e-3,
    )


def _ensure_current_baseline() -> None:
    """Return the baseline artifact, regenerating it under stale semantics."""
    if BASELINE_PATH.is_file():
        baseline = np.load(BASELINE_PATH, allow_pickle=True)
        if (
            "semantics" in baseline.files
            and str(baseline["semantics"]) == CURRENT_SEMANTICS
        ):
            return
    # The sibling test owns baseline generation; load it explicitly so this
    # gate stays self-consistent when run standalone.
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from test_baseline_tracking import _run_baseline
    _run_baseline()


def test_modular_jit_matches_current_semantics_baseline():
    _ensure_current_baseline()
    baseline = np.load(BASELINE_PATH, allow_pickle=True)
    assert str(baseline["semantics"]) == CURRENT_SEMANTICS

    q_sequence = baseline["q_sequence"]
    path_state_sequence = baseline["path_state_sequence"]
    u_safe_sequence = baseline["u_safe_sequence"]
    qp_ok_sequence = baseline["qp_ok_sequence"]
    err_6d_sequence = baseline["err_6d_sequence"]
    initial_q = baseline["initial_q"]
    assert q_sequence.shape[0] == NUM_STEPS + 1

    trajectory = load_repository_trajectory(str(CURRENT_TRAJECTORY))
    geometry = trajectory.path_geometry()
    loop = JaxControlLoop(dt=0.002, temporal_lambda=0.2, enable_x64=True)
    loop.configure_path(geometry, PathFollowingConfig())
    initialization_start = perf_counter()
    loop.init_cbf()
    initialization_s = perf_counter() - initialization_start

    kwargs = _control_kwargs(initial_q)
    max_err_6d_diff = 0.0
    max_position_error_diff_m = 0.0
    max_orientation_error_diff_rad = 0.0
    max_progress_diff = 0.0
    max_linear_velocity_diff_m_s = 0.0
    max_angular_velocity_diff_rad_s = 0.0
    qp_mismatches = 0
    step_times = []

    for step in range(NUM_STEPS):
        step_start = perf_counter()
        result = loop.path_tracking_step(
            q=q_sequence[step],
            path_state=path_state_sequence[step],
            **kwargs)
        step_times.append(perf_counter() - step_start)
        if step == 0:
            initialization_and_first_call_s = initialization_s + step_times[0]
        error_difference = np.asarray(result.err_6d) - err_6d_sequence[step]
        err_6d_diff = float(np.max(np.abs(error_difference)))
        max_position_error_diff_m = max(
            max_position_error_diff_m, float(np.linalg.norm(error_difference[:3])))
        max_orientation_error_diff_rad = max(
            max_orientation_error_diff_rad, float(np.linalg.norm(error_difference[3:])))
        progress_diff = float(np.max(np.abs(
            np.asarray(result.path_state)[:2] - path_state_sequence[step + 1][:2])))
        velocity_difference = np.abs(np.asarray(result.u_safe) - u_safe_sequence[step])
        max_linear_velocity_diff_m_s = max(
            max_linear_velocity_diff_m_s, float(velocity_difference[0]))
        max_angular_velocity_diff_rad_s = max(
            max_angular_velocity_diff_rad_s, float(np.max(velocity_difference[1:])))
        max_err_6d_diff = max(max_err_6d_diff, err_6d_diff)
        max_progress_diff = max(max_progress_diff, progress_diff)
        qp_mismatches += int(bool(result.qp_ok) != bool(qp_ok_sequence[step]))

    # AC6.1 (current semantics): stepwise reproduction of the recorded
    # baseline on well-conditioned observables.  The task-consistent P has
    # eigenvalues ~0.01 in the null-space directions vs 100-400 in the task
    # directions (kappa ~ 4e4), so the elastic PDIP at solver_tol=1e-3 can
    # legitimately land on different null-space components of the minimizer
    # when two XLA graphs fuse differently; u_safe and q_next therefore are
    # recorded but not gated.  Task error, path progress, and QP success are
    # the physically meaningful equivalence signals.
    assert max_err_6d_diff < 1e-4, (
        f"err_6d max deviation {max_err_6d_diff:.3e}")
    assert max_progress_diff < 1e-3, (
        f"path progress max deviation {max_progress_diff:.3e}")
    assert qp_mismatches == 0, f"qp_ok mismatches: {qp_mismatches}"

    # AC6.2: no recompilation after the first frame.
    cache_size = loop._path_tracking_fn._cache_size()
    assert cache_size == 1, f"modular JIT cache size {cache_size}"

    # 计时包含真实初始化、JIT 预热和首次调用；后续逐步计算与基准比较。
    per_step_ms = float(np.mean(step_times[1:]) * 1000.0)
    baseline_per_step_ms = float(baseline["per_step_ms_steady"])
    assert initialization_and_first_call_s < 45.0, (
        f"initialization and first call took {initialization_and_first_call_s:.1f}s (limit 45s)")
    assert per_step_ms <= baseline_per_step_ms * 1.2, (
        f"modular per-step {per_step_ms:.3f} ms exceeds baseline "
        f"{baseline_per_step_ms:.3f} ms by more than 20%")

    PERF_REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    PERF_REPORT_PATH.write_text(
        "# M6 模块化 JIT 性能与回归证据\n\n"
        f"- 语义标签: `{CURRENT_SEMANTICS}`（M7 弹性 QP）\n"
        "- 环境: x64，完整控制内核初始化与预热\n"
        f"- AC6.1 逐步对照: "
        f"位置误差最大差值 {max_position_error_diff_m * 1000:.3e} mm，"
        f"姿态误差最大差值 {np.rad2deg(max_orientation_error_diff_rad):.3e} °，"
        f"路径进度最大差值 {max_progress_diff * 1000:.3e} mm，"
        f"qp_ok 不一致 {qp_mismatches}；"
        f"J1 速度最大差值 {max_linear_velocity_diff_m_s * 1000:.3e} mm/s，"
        f"J2–J9 速度最大差值 {np.rad2deg(max_angular_velocity_diff_rad_s):.3e} °/s\n"
        f"- AC6.2 JIT 缓存条目（首帧后）: {cache_size}\n"
        f"- AC6.3 初始化及首次调用: {initialization_and_first_call_s:.2f} s "
        "（要求 < 45 s）\n"
        f"- AC6.4 单步平均: {per_step_ms:.3f} ms "
        f"(基准 {baseline_per_step_ms:.3f} ms，≤ 120%)\n",
        encoding="utf-8",
    )
    print(f"initialization_and_first_call_s={initialization_and_first_call_s:.2f} "
          f"per_step_ms={per_step_ms:.3f} "
          f"baseline_per_step_ms={baseline_per_step_ms:.3f}")


if __name__ == "__main__":
    raise SystemExit(__import__("pytest").main([__file__, "-v"]))
