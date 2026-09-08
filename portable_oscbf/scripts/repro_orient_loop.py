#!/usr/bin/env python3
"""Diagnostic closed loop: nominal 6-D OSC + production plant dynamics.

Mirrors the production timing (controller tick 100 Hz, kernel dt=0.002 s,
first-order plant kp=80 at 100 Hz) but without the CBF-QP, so the stability
of the nominal control law itself can be audited quickly.

Usage:
    XLA_FLAGS=--xla_cpu_multi_thread_eigen=false JAX_NUM_THREADS=1 \
    python3 portable_oscbf/scripts/repro_orient_loop.py [scenario]
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

_ROOT = Path(__file__).resolve().parents[1]
for entry in (_ROOT, _ROOT / "work"):
    text = str(entry)
    if text not in sys.path:
        sys.path.insert(0, text)

from work.ik_data_loader import load_repository_trajectory
from work.nineaxis_kinematics import NineaxisKinematics
from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
from work.path_following import PathFollower, PathFollowingConfig


MAT_PATH = "/home/lsn/robot_safecontrol/data/nurbs/ik_input.mat"
AXIS = np.array([0.0, 1.0, 0.0])


def _damped_pinv(J: np.ndarray, damping: float) -> np.ndarray:
    return J.T @ np.linalg.inv(J @ J.T + damping * np.eye(J.shape[0]))


def _ik_path_start(trajectory) -> np.ndarray:
    kin = NineaxisKinematics()
    target_pos = trajectory.pos_world_at(0.0)
    target_rot = trajectory.orientation_at(0.0)
    rng = np.random.default_rng(0)
    seeds = [np.zeros(9)]
    lower = np.asarray(kin.joint_limits.q_min)
    upper = np.asarray(kin.joint_limits.q_max)
    for _ in range(24):
        seeds.append(rng.uniform(lower + 0.1, upper - 0.1))
    best_q, best_err = None, float("inf")
    for seed in seeds:
        candidate = kin.ik(target_pos, target_rot, q_init=seed)
        if candidate is None:
            continue
        err = float(np.linalg.norm(kin.ee_position(candidate) - target_pos))
        if err < best_err:
            best_err, best_q = err, candidate
    assert best_q is not None
    return best_q


def _orientation_error_6d(ee_rot: np.ndarray, des_rot: np.ndarray) -> np.ndarray:
    return -0.5 * (
        np.cross(ee_rot[:, 0], des_rot[:, 0])
        + np.cross(ee_rot[:, 1], des_rot[:, 1])
        + np.cross(ee_rot[:, 2], des_rot[:, 2])
    )


def run(scenario: str) -> None:
    trajectory = load_repository_trajectory(MAT_PATH)
    if scenario != "aligned_fixed":
        trajectory.set_surface_normal_orientation(AXIS)
    geometry = trajectory.path_geometry()
    follower = PathFollower(
        geometry, PathFollowingConfig(reference_lead_m=0.005)
    )

    robot = NineaxisManipulatorJAX()
    dq_max = np.asarray(robot.joint_max_velocities, dtype=float)
    q_min = np.asarray(robot.joint_lower_limits, dtype=float)
    q_max = np.asarray(robot.joint_upper_limits, dtype=float)

    start_file = Path("/tmp/repro_start_q.npy")
    if start_file.is_file():
        q = np.load(start_file)
        print(f"loaded start q from {start_file}")
    else:
        q = _ik_path_start(trajectory)
    ee_pos = np.asarray(robot.ee_position(q))
    follower.reset_to_position(ee_pos)

    kp_pos, kp_orient = 60.0, 10.0
    feedforward = True
    inertia_weighted = False
    fixed_ref = scenario == "fixed_ref"
    use_midpoint = scenario == "midpoint"
    if scenario in ("no_ff", "kp_orient_40", "inertia", "fixed_ref", "midpoint"):
        if scenario == "no_ff":
            feedforward = False
        elif scenario == "kp_orient_40":
            kp_orient = 40.0
        elif scenario == "inertia":
            inertia_weighted = True

    print(f"scenario={scenario} kp_pos={kp_pos} kp_orient={kp_orient} "
          f"ff={feedforward} inertia={inertia_weighted} midpoint={use_midpoint}")
    print(f"q0={np.round(q, 4).tolist()}")

    qdot_null_limit = 0.18
    plant_kp = 80.0
    dt_kernel = 0.002
    dt_plant = 0.01
    damping = 1e-3

    max_orient = 0.0
    fixed_sample = None
    q_mid = 0.5 * (q_min + q_max)
    stuck_ticks = 0
    for tick in range(60000):
        # Reference advance with the same kernel dt used in production.
        step = follower.step(ee_pos, dt_s=dt_kernel)
        if fixed_ref:
            if fixed_sample is None:
                progress, _ = geometry.project_local(
                    np.asarray(robot.ee_position(q)), anchor_segment=0,
                    half_window_segments=96,
                )
                fixed_sample = geometry.sample(progress)
            sample = fixed_sample
            feedrate = 0.0
        else:
            sample = step.reference
            feedrate = step.feedrate_m_s
        ee_rot = np.asarray(robot.ee_rotation(q))
        J = np.asarray(robot.ee_jacobian(q))

        rot_err = _orientation_error_6d(ee_rot, sample.rotation)
        pos_err = ee_pos - sample.position_m
        tangent_err = sample.tangent * float(np.dot(sample.tangent, pos_err))
        pos_fb = pos_err - tangent_err

        if inertia_weighted:
            from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX as _R
            kin_np = NineaxisKinematics()
            # Dynamically consistent pseudo-inverse as in upstream OSCBF.
            M = np.asarray(kin_np.mass_matrix(q))
            M_inv = np.linalg.inv(M)
            task_inertia_inv = J @ M_inv @ J.T
            task_inertia = np.linalg.inv(task_inertia_inv)
            J_hash = M_inv @ J.T @ task_inertia
            null_proj = np.eye(9) - J_hash @ J
        else:
            J_hash = _damped_pinv(J, damping)
            null_proj = np.eye(9) - J_hash @ J

        twist_bias = np.concatenate([
            -kp_pos * pos_fb,
            -kp_orient * rot_err,
        ])
        if feedforward:
            twist_per_m = np.concatenate([sample.tangent, sample.omega_per_m])
        else:
            twist_per_m = np.concatenate([sample.tangent, np.zeros(3)])
        u_bias = J_hash @ twist_bias
        u_per_m = J_hash @ twist_per_m
        if use_midpoint:
            qdot_null = np.clip(-0.45 * (q - q_mid), -qdot_null_limit,
                                qdot_null_limit)
        else:
            qdot_null = np.zeros(9)  # production q_des == q
        u_nom = u_bias + null_proj @ qdot_null + u_per_m * feedrate
        u_nom = np.clip(u_nom, -dq_max, dq_max)

        q_cmd = np.clip(q + u_nom * dt_kernel, q_min, q_max)
        v = np.clip(plant_kp * (q_cmd - q), -dq_max, dq_max)
        q = np.clip(q + v * dt_plant, q_min, q_max)
        ee_pos = np.asarray(robot.ee_position(q))

        if tick % 100 == 0:
            orient_deg = float(np.degrees(np.linalg.norm(rot_err)))
            max_orient = max(max_orient, orient_deg)
            margin = float(np.min(np.minimum(q - q_min, q_max - q)))
            print(
                f"t={tick / 100:5.1f}s src={sample.source_time_s:6.3f}s "
                f"feed={feedrate:7.4f} pos_err={np.linalg.norm(pos_err) * 1000:7.3f}mm "
                f"orient_err={orient_deg:8.4f}deg cross={step.cross_track_error_m * 1000:7.3f}mm "
                f"limit={step.limiting_reason} jmargin={margin:6.4f}"
            )
        if max_orient > 45.0:
            print(f"ORIENTATION DIVERGED (>{45} deg) at t={tick / 100:.1f}s")
            break
        if sample.source_time_s >= 29.0:
            print(f"PATH COMPLETED at src={sample.source_time_s:.3f}s")
            break
    print(f"final max_orient={max_orient:.3f} deg")


if __name__ == "__main__":
    run(sys.argv[1] if len(sys.argv) > 1 else "baseline")
