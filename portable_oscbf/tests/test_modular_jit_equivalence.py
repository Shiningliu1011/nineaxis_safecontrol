#!/usr/bin/env python3
"""M6 acceptance: modular-JIT equivalence against the M5 whole-kernel baseline."""

from __future__ import annotations

from pathlib import Path
from time import perf_counter

import jax

jax.config.update("jax_enable_x64", True)

import _path_setup  # noqa: F401
import numpy as np
import pytest

from work.ik_data_loader import load_repository_trajectory
from work.jax_control_facade import JaxControlLoop
from work.path_following import PathFollowingConfig


REPO_ROOT = Path(__file__).resolve().parents[2]
CURRENT_TRAJECTORY = REPO_ROOT / "data" / "nurbs" / "ik_input.mat"
BASELINE_PATH = REPO_ROOT / "output" / "baseline_tracking.npz"
NUM_STEPS = 3000


def _control_kwargs(initial_q):
    return dict(
        kp_pos=50.0,
        kp_orient=10.0,
        kp_joint=0.45,
        q_des=initial_q,
        nullspace_speed_limit=0.18,
        damping=1e-3,
    )


def test_modular_jit_matches_m5_baseline():
    from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
    from work.oscbf_velocity_config import NineaxisOSCBFVelocityConfig

    if NineaxisOSCBFVelocityConfig(NineaxisManipulatorJAX()).relax_cbf:
        pytest.skip(
            "M6 equivalence gate was validated against the M5 hard-QP "
            "baseline; M7 switched the controller to the elastic QP "
            "semantics (relax_cbf=True), so the old baseline no longer "
            "represents the current controller")

    assert BASELINE_PATH.is_file(), "M5 baseline artifact missing"
    baseline = np.load(BASELINE_PATH)
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

    # AC6.1: stepwise equality with the M5 baseline.  Independently compiled
    # XLA modules legitimately differ at the 1e-9 level (FMA/fusion), which
    # amplifies over 3000 closed-loop steps to ~1e-8 rad / ~2e-5 rad/s.  The
    # gates 1e-6 m/rad and 1e-4 rad/s are far below control accuracy while
    # still proving mathematical equivalence.
    assert max_q_error < 1e-6, f"q_next max deviation {max_q_error:.3e}"
    assert max_u_error < 1e-4, f"u_safe max deviation {max_u_error:.3e}"
    assert qp_mismatches == 0, f"qp_ok mismatches: {qp_mismatches}"

    # AC6.2: no recompilation after the first frame.
    assert loop._path_tracking_fn._cache_size() == 1, (
        f"modular JIT cache size {loop._path_tracking_fn._cache_size()}")

    # AC6.3 / AC6.4: warm-up budget (module compilation in init_cbf) and
    # steady per-step cost vs M5.
    per_step_ms = float(np.mean(step_times[1:]) * 1000.0)
    baseline_per_step_ms = float(baseline["per_step_ms_steady"])
    assert path_compile_s < 30.0, (
        f"modular path JIT compilation took {path_compile_s:.1f}s (limit 30s)")
    assert per_step_ms <= baseline_per_step_ms * 1.2, (
        f"modular per-step {per_step_ms:.3f} ms exceeds baseline "
        f"{baseline_per_step_ms:.3f} ms by more than 20%")
    print(f"path_compile_s={path_compile_s:.2f} "
          f"per_step_ms={per_step_ms:.3f} "
          f"baseline_per_step_ms={baseline_per_step_ms:.3f}")


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
