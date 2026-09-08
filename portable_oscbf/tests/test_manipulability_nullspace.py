#!/usr/bin/env python3
"""M8 acceptance: manipulability-gradient null-space policy."""

from __future__ import annotations

from pathlib import Path

import jax

jax.config.update("jax_enable_x64", True)

import _path_setup  # noqa: F401
import numpy as np
import pytest

from work.ik_data_loader import load_repository_trajectory
from work.jax_control_facade import JaxControlLoop
from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
from work.nullspace_policy import ManipulabilityGradientPolicy
from work.path_following import PathFollowingConfig


REPO_ROOT = Path(__file__).resolve().parents[2]
CURRENT_TRAJECTORY = REPO_ROOT / "data" / "nurbs" / "ik_input.mat"
BASELINE_PATH = REPO_ROOT / "output" / "baseline_tracking.npz"
EVIDENCE_PATH = REPO_ROOT / "output" / "oscbf_m8_nullspace.md"

# Cross-test measurement log consumed by the final evidence writer.  Keys are
# the AC8 metrics; each stage test fills the entries it owns.
_EVIDENCE: dict = {}


def _random_qs(robot, count=24):
    rng = np.random.default_rng(20260806)
    lower = np.asarray(robot.joint_lower_limits)
    upper = np.asarray(robot.joint_upper_limits)
    margin = 0.05 * np.ones(9)
    margin[0] = 0.05
    return [rng.uniform(lower + margin, upper - margin) for _ in range(count)]


@pytest.fixture(scope="module")
def robot():
    return NineaxisManipulatorJAX()


@pytest.fixture(scope="module")
def policy(robot):
    return ManipulabilityGradientPolicy(robot)


def test_gradient_matches_finite_difference(robot, policy):
    # Fourth-order central differences (h=3e-5) keep truncation below the
    # autodiff reference; plain 1e-6 two-point differences hit ~1e-4 on
    # high-curvature (near-singular) samples.
    h = 3e-5
    max_relative = 0.0
    for q in _random_qs(robot, count=4):
        analytic = np.asarray(policy.gradient(
            jax.numpy.asarray(q), robot.ee_jacobian))
        finite = np.zeros(9)
        for k in range(9):
            samples = []
            for sign in (2, 1, -1, -2):
                q_sample = q.copy()
                q_sample[k] += sign * h
                samples.append(float(np.asarray(policy.manipulability(
                    jax.numpy.asarray(q_sample),
                    robot.ee_jacobian(jax.numpy.asarray(q_sample))))))
            phi_2p, phi_1p, phi_1m, phi_2m = samples
            finite[k] = (-phi_2p + 8.0 * phi_1p - 8.0 * phi_1m + phi_2m) / (
                12.0 * h)
        significant = np.abs(analytic) >= 1e-6
        if np.any(significant):
            relative = np.max(
                np.abs(analytic[significant] - finite[significant])
                / np.abs(analytic[significant]))
            max_relative = max(max_relative, float(relative))
            assert relative < 1e-4, (
                f"gradient relative error {relative:.2e} at q={q}")
    _EVIDENCE["fd_max_relative_error"] = max_relative


def test_nullspace_velocity_leaves_task_space(robot, policy):
    """J @ qdot_N must stay at the damped-pseudoinverse numerical level.

    The damping lambda=1e-3 in the nominal Jacobian hash makes
    J (I - J_hash J) ~ 1e-3 near singularities by design (guide §5.2 notes
    this leak and asks to record it); the gate is 1e-2, well below task-space
    speeds.
    """

    max_leakage = 0.0
    for q in _random_qs(robot, count=24):
        jacobian = np.asarray(robot.ee_jacobian(jax.numpy.asarray(q)))
        jacobian_hash = jacobian.T @ np.linalg.inv(
            jacobian @ jacobian.T + 1e-3 * np.eye(6))
        projector = np.eye(9) - jacobian_hash @ jacobian
        qdot_null = np.asarray(policy.nullspace_velocity(
            jax.numpy.asarray(q), jax.numpy.asarray(jacobian),
            jax.numpy.asarray(jacobian_hash),
            jax.numpy.asarray(projector),
            jax.numpy.asarray(robot.joint_max_velocities)))
        max_leakage = max(max_leakage, float(np.linalg.norm(
            jacobian @ qdot_null)))
        # Global scaling must respect the weighted speed cap.
        assert np.linalg.norm(qdot_null) <= 0.25 + 1e-9
    assert max_leakage < 1e-2, f"task leakage {max_leakage:.3e}"
    _EVIDENCE["max_leakage"] = max_leakage


def test_fixed_endpoint_manipulability_increases(robot, policy):
    loop = JaxControlLoop(
        dt=0.002, temporal_lambda=0.2, enable_x64=True,
        nullspace_policy=policy)
    loop.init_cbf()
    # The verified tracking start configuration; q=0 sits in a folded,
    # near-singular region where the elastic QP overrides self-motion.
    q = np.array([
        0.2303562, 0.1112539, 1.0167209, -0.6810303, -1.8294025,
        -0.4664294, 0.4743473, -1.0429228, 0.0289233,
    ])
    task_pos = np.asarray(loop.robot.ee_position(q))
    task_rot = np.asarray(loop.robot.ee_rotation(q))

    phis = []
    projected_norms = []
    for _ in range(2000):
        jacobian = np.asarray(loop.robot.ee_jacobian(q))
        phis.append(float(np.asarray(policy.manipulability(
            jax.numpy.asarray(q), jax.numpy.asarray(jacobian)))))
        jacobian_hash = jacobian.T @ np.linalg.inv(
            jacobian @ jacobian.T + 1e-3 * np.eye(6))
        projector = np.eye(9) - jacobian_hash @ jacobian
        gradient = np.asarray(policy.gradient(
            jax.numpy.asarray(q), loop.robot.ee_jacobian))
        projected_norms.append(float(np.linalg.norm(projector.T @ gradient)))
        result = loop.tracking_step(
            q=q, task_pos=task_pos, task_vel=np.zeros(3),
            task_rot=task_rot, task_omega=np.zeros(3),
            kp_pos=60.0, kp_orient=10.0, kp_joint=0.45, q_des=q,
            nullspace_speed_limit=0.18, damping=1e-3)
        q = np.asarray(result[0])

    phi_start = phis[0]
    phi_end = phis[-1]
    assert phi_end >= phi_start - 1e-9, (
        f"manipulability decreased: {phi_start:.6f} -> {phi_end:.6f}")
    mean_first = float(np.mean(projected_norms[:200]))
    mean_last = float(np.mean(projected_norms[-200:]))
    assert mean_last <= mean_first * 1.05 + 1e-6, (
        f"projected gradient norm did not decrease: "
        f"{mean_first:.4f} -> {mean_last:.4f}")
    _EVIDENCE["phi_start"] = float(phi_start)
    _EVIDENCE["phi_end"] = float(phi_end)
    _EVIDENCE["projected_grad_first"] = mean_first
    _EVIDENCE["projected_grad_last"] = mean_last
    print(f"phi {phi_start:.6f} -> {phi_end:.6f}, "
          f"|N^T g| {mean_first:.4f} -> {mean_last:.4f}")


def test_task_error_not_worse_than_baseline(robot, policy):
    baseline = np.load(BASELINE_PATH)
    baseline_err = baseline["err_6d_sequence"]
    baseline_pos = np.max(np.linalg.norm(baseline_err[-1000:, :3], axis=1))
    baseline_orient = np.max(np.linalg.norm(
        baseline_err[-1000:, 3:], axis=1))

    trajectory = load_repository_trajectory(str(CURRENT_TRAJECTORY))
    loop = JaxControlLoop(
        dt=0.002, temporal_lambda=0.2, enable_x64=True,
        nullspace_policy=policy)
    loop.configure_path(trajectory.path_geometry(), PathFollowingConfig())
    loop.init_cbf()
    q = baseline["initial_q"].copy()
    path_state = loop.initial_path_state()
    errors = []
    for _ in range(3000):
        result = loop.path_tracking_step(
            q=q, path_state=path_state, kp_pos=60.0, kp_orient=10.0,
            kp_joint=0.45, q_des=q, nullspace_speed_limit=0.18,
            damping=1e-3)
        path_state = result.path_state
        q = result.q_next
        errors.append(np.asarray(result.err_6d))
    errors = np.stack(errors)
    current_pos = np.max(np.linalg.norm(errors[-1000:, :3], axis=1))
    current_orient = np.max(np.linalg.norm(errors[-1000:, 3:], axis=1))
    assert current_pos <= baseline_pos * 1.2 + 1e-6, (
        f"position error degraded: {current_pos:.6f} vs {baseline_pos:.6f}")
    assert current_orient <= baseline_orient * 1.2 + 1e-6, (
        f"orientation error degraded: {current_orient:.6f} "
        f"vs {baseline_orient:.6f}")
    _EVIDENCE["baseline_pos_err"] = float(baseline_pos)
    _EVIDENCE["baseline_orient_err"] = float(baseline_orient)
    _EVIDENCE["nullspace_pos_err"] = float(current_pos)
    _EVIDENCE["nullspace_orient_err"] = float(current_orient)
    _EVIDENCE["pos_err_ratio"] = float(current_pos / max(baseline_pos, 1e-12))
    _EVIDENCE["orient_err_ratio"] = float(
        current_orient / max(baseline_orient, 1e-12))


def test_z_write_m8_evidence_report():
    """AC8 evidence artifact: trend records for the null-space strategy."""
    required = {
        "fd_max_relative_error", "max_leakage", "phi_start", "phi_end",
        "projected_grad_first", "projected_grad_last",
        "baseline_pos_err", "baseline_orient_err",
        "nullspace_pos_err", "nullspace_orient_err",
        "pos_err_ratio", "orient_err_ratio",
    }
    missing = sorted(required - set(_EVIDENCE))
    assert not missing, f"missing M8 evidence entries: {missing}"

    EVIDENCE_PATH.parent.mkdir(parents=True, exist_ok=True)
    EVIDENCE_PATH.write_text(
        "# M8 可操作度零空间证据\n\n"
        "策略: `ManipulabilityGradientPolicy`"
        "（k_m=0.15，v_N,max=0.25，l_c=0.4，ε=1e-6，"
        "激活关闭 ρ=1，低通 β=0.2，整体缩放 s_v）\n"
        f"- AC8.1 梯度有限差分对照最大相对误差: "
        f"{_EVIDENCE['fd_max_relative_error']:.3e}（阈值 < 1e-4）\n"
        f"- AC8.2 零空间残差 ‖J·qdot_N‖ 最大: "
        f"{_EVIDENCE['max_leakage']:.3e}（阻尼伪逆数值水平）\n"
        f"- AC8.3 固定末端 2000 步: φ "
        f"{_EVIDENCE['phi_start']:.6f} → {_EVIDENCE['phi_end']:.6f}；"
        f"‖Nᵀg‖ {_EVIDENCE['projected_grad_first']:.4f} → "
        f"{_EVIDENCE['projected_grad_last']:.4f}\n"
        f"- AC8.4 末端误差劣化: 位置 "
        f"{_EVIDENCE['pos_err_ratio']:.3f}×、姿态 "
        f"{_EVIDENCE['orient_err_ratio']:.3f}×（阈值 < 1.2×）\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
