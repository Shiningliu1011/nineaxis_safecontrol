#!/usr/bin/env python3
"""M6 acceptance: modular-JIT regression gate against the recorded baseline.

The original M6 gate proved bitwise equivalence between the monolithic M5
whole-kernel and the per-function ``@jax.jit`` refactor while the controller
still used the hard QP (relax_cbf=False).  M7 switched the production kernel
to the elastic QP, so that historical comparison can no longer be replayed
against a live monolithic implementation: the refactor replaced it.

This gate therefore compares the modular path kernel, step by step, against
the baseline artifact produced by ``test_baseline_tracking`` under the
*current* controller semantics (tagged inside the npz).  It also enforces the
remaining M6 acceptance criteria that stay meaningful for the modular build:
single JIT cache entry after the first frame, sub-30 s modular compilation,
and per-step cost within 20% of the baseline.  Results are written to
``output/oscbf_m6_perf.md``.
"""

from __future__ import annotations

import sys
from pathlib import Path
from time import perf_counter

import jax

jax.config.update("jax_enable_x64", True)

import _path_setup  # noqa: F401
import numpy as np

from work.ik_data_loader import load_repository_trajectory
from work.jax_control_facade import JaxControlLoop
from work.path_following import PathFollowingConfig


REPO_ROOT = Path(__file__).resolve().parents[2]
CURRENT_TRAJECTORY = REPO_ROOT / "data" / "nurbs" / "ik_input.mat"
BASELINE_PATH = REPO_ROOT / "output" / "baseline_tracking.npz"
PERF_REPORT_PATH = REPO_ROOT / "output" / "oscbf_m6_perf.md"
NUM_STEPS = 3000
CURRENT_SEMANTICS = "m7_elastic_qp_v1"


def _control_kwargs(initial_q):
    return dict(
        kp_pos=50.0,
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
    initial_q = baseline["initial_q"]
    assert q_sequence.shape[0] == NUM_STEPS + 1

    trajectory = load_repository_trajectory(str(CURRENT_TRAJECTORY))
    geometry = trajectory.path_geometry()
    loop = JaxControlLoop(dt=0.002, temporal_lambda=0.2, enable_x64=True)
    loop.configure_path(geometry, PathFollowingConfig())
    # Compile only the path modules (AC6.3 measures the modular path kernel).
    # The legacy step/tracking kernels are compiled by the normal warmup in
    # M5/M7 tests; they are not part of the modular path-following chain.
    original_warmup = loop._warmup
    loop._warmup = lambda: None
    loop.init_cbf()
    loop._warmup = original_warmup

    kwargs = _control_kwargs(initial_q)
    max_q_error = 0.0
    max_u_error = 0.0
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
            path_compile_s = step_times[0]
        q_error = float(np.max(np.abs(
            np.asarray(result.q_next) - q_sequence[step + 1])))
        u_error = float(np.max(np.abs(
            np.asarray(result.u_safe) - u_safe_sequence[step])))
        max_q_error = max(max_q_error, q_error)
        max_u_error = max(max_u_error, u_error)
        qp_mismatches += int(bool(result.qp_ok) != bool(qp_ok_sequence[step]))

    # AC6.1 (current semantics): stepwise reproduction of the recorded
    # baseline.  Independently compiled XLA modules legitimately differ at
    # the 1e-9 level (FMA/fusion), which amplifies over 3000 closed-loop
    # steps to ~1e-8 rad / ~2e-5 rad/s.  The gates 1e-6 m/rad and 1e-4 rad/s
    # are far below control accuracy while still proving equivalence.
    assert max_q_error < 1e-6, f"q_next max deviation {max_q_error:.3e}"
    assert max_u_error < 1e-4, f"u_safe max deviation {max_u_error:.3e}"
    assert qp_mismatches == 0, f"qp_ok mismatches: {qp_mismatches}"

    # AC6.2: no recompilation after the first frame.
    cache_size = loop._path_tracking_fn._cache_size()
    assert cache_size == 1, f"modular JIT cache size {cache_size}"

    # AC6.3 / AC6.4: warm-up budget (module compilation in init_cbf) and
    # steady per-step cost vs the baseline.
    per_step_ms = float(np.mean(step_times[1:]) * 1000.0)
    baseline_per_step_ms = float(baseline["per_step_ms_steady"])
    assert path_compile_s < 30.0, (
        f"modular path JIT compilation took {path_compile_s:.1f}s (limit 30s)")
    assert per_step_ms <= baseline_per_step_ms * 1.2, (
        f"modular per-step {per_step_ms:.3f} ms exceeds baseline "
        f"{baseline_per_step_ms:.3f} ms by more than 20%")

    PERF_REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    PERF_REPORT_PATH.write_text(
        "# M6 模块化 JIT 性能与回归证据\n\n"
        f"- 语义标签: `{CURRENT_SEMANTICS}`（M7 弹性 QP）\n"
        "- 环境: x64 / 单线程 XLA"
        "（`XLA_FLAGS=--xla_cpu_multi_thread_eigen=false`，"
        "`JAX_NUM_THREADS=1`）\n"
        f"- AC6.1 逐步对照: q_next 最大偏差 {max_q_error:.3e}，"
        f"u_safe 最大偏差 {max_u_error:.3e}，qp_ok 不一致 {qp_mismatches}\n"
        f"- AC6.2 JIT 缓存条目（首帧后）: {cache_size}\n"
        f"- AC6.3 模块化路径内核首次编译: {path_compile_s:.2f} s "
        "(阈值 < 30 s)\n"
        f"- AC6.4 单步平均: {per_step_ms:.3f} ms "
        f"(基线 {baseline_per_step_ms:.3f} ms，≤ 120%)\n"
        "\n> 注: 原始 M6 的「模块化 vs 整块内核逐位等价」在 M5 硬 QP 语义下"
        "已验收；M7 切换弹性 QP 后整块内核不复存在，当前门改为"
        "当前语义基线下的逐步回归 + 性能验收。\n",
        encoding="utf-8",
    )
    print(f"path_compile_s={path_compile_s:.2f} "
          f"per_step_ms={per_step_ms:.3f} "
          f"baseline_per_step_ms={baseline_per_step_ms:.3f}")


if __name__ == "__main__":
    raise SystemExit(__import__("pytest").main([__file__, "-v"]))
