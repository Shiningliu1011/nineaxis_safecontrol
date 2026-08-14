#!/usr/bin/env python3
"""Reproduce the 86.6% butterfly stall and compare null-space targets.

Seeds the full JAX path kernel at the observed live-stall configuration and
runs the closed loop (first-order plant, production timing) with
``q_des == q`` (current behaviour) versus ``q_des == joint midpoint``.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
for entry in (_ROOT, _ROOT / "work"):
    text = str(entry)
    if text not in sys.path:
        sys.path.insert(0, text)

from work.ik_data_loader import load_repository_trajectory
from work.jax_control_facade import JaxControlLoop
from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
from work.path_following import PathFollowingConfig, PathFollowerState
from work.task_mode_contract import TASK_MODE_POSE_6D, TASK_MODE_TOOL_AXIS_5D


MAT_PATH = "/home/lsn/robot_safecontrol/data/nurbs/ik_input.mat"
STALL_Q = np.array([
    0.5704153471002645,
    1.5532522484543416,
    -0.5068393789079734,
    -1.559580139500765,
    -0.7181008916335166,
    1.22521284222439,
    -0.7104289475893918,
    -1.4729951220316089,
    0.2259187742865759,
])


def run() -> None:
    trajectory = load_repository_trajectory(MAT_PATH)
    trajectory.set_surface_normal_orientation([0.0, 1.0, 0.0])
    geometry = trajectory.path_geometry()
    config = PathFollowingConfig(reference_lead_m=0.005)

    def build_loop(task_mode=TASK_MODE_POSE_6D):
        loop = JaxControlLoop(
            dt=0.002,
            w_pos=20.0,
            w_orient=10.0,
            w_joint=0.1,
            temporal_lambda=0.2,
            enable_x64=True,
            task_mode=task_mode,
        )
        loop.configure_path(geometry, config)
        loop.init_cbf()
        return loop

    loop6 = build_loop()

    robot = NineaxisManipulatorJAX()
    dq_max = np.asarray(robot.joint_max_velocities, dtype=float)
    q_min = np.asarray(robot.joint_lower_limits, dtype=float)
    q_max = np.asarray(robot.joint_upper_limits, dtype=float)
    q_mid = 0.5 * (q_min + q_max)

    # Seed the JAX path state at the current tool projection.
    ee_pos = np.asarray(robot.ee_position(STALL_Q))
    follower_state = PathFollowerState()
    # The projection window anchors to the nearest sample, otherwise a
    # mid-path seed would project onto the path start.
    anchor = int(np.argmin(
        np.einsum("ij,ij->i",
                  geometry.positions_m - ee_pos,
                  geometry.positions_m - ee_pos)
    ))
    projected, segment = geometry.project_local(
        ee_pos, anchor_segment=anchor,
        half_window_segments=config.projection_half_window_segments,
    )
    follower_state = PathFollowerState(
        reference_progress_m=min(
            geometry.total_length_m, projected + config.reference_lead_m
        ),
        projected_progress_m=projected,
        projection_segment=segment,
        endpoint_hold_s=0.0,
        completed=False,
    )
    path_state = np.array([
        follower_state.reference_progress_m,
        follower_state.projected_progress_m,
        float(follower_state.projection_segment),
        follower_state.endpoint_hold_s,
        float(follower_state.completed),
    ])

    scenarios = [
        ("q_des=q", loop6, False),
        ("q_des=mid", loop6, True),
        ("task_5d", build_loop(TASK_MODE_TOOL_AXIS_5D), False),
    ]
    for label, loop, use_midpoint in scenarios:
        q = STALL_Q.copy()
        state = path_state.copy()
        print(f"=== {label} ===")
        for tick in range(4000):
            q_des = q_mid if use_midpoint else q
            result = loop.path_tracking_step(
                q=q,
                path_state=state,
                kp_pos=60.0,
                kp_orient=10.0,
                kp_joint=0.45,
                q_des=q_des,
                nullspace_speed_limit=0.18,
            )
            state = np.asarray(result.path_state)
            q_cmd = np.clip(np.asarray(result.q_next), q_min, q_max)
            v = np.clip(80.0 * (q_cmd - q), -dq_max, dq_max)
            q = np.clip(q + v * 0.01, q_min, q_max)
            if tick % 100 == 0:
                margin = float(np.min(np.minimum(q - q_min, q_max - q)))
                print(
                    f"t={tick / 100:5.1f}s src={result.reference_source_time_s:6.3f}s "
                    f"feed={result.feedrate_m_s:7.4f} limit={result.limiting_reason_code} "
                    f"pos_err={np.linalg.norm(result.err_6d[:3]) * 1000:7.3f}mm "
                    f"orient_err={np.degrees(np.linalg.norm(result.err_6d[3:])):8.4f}deg "
                    f"jmargin={margin:6.4f} qp={result.qp_ok}"
                )
            if result.reference_source_time_s >= 29.0:
                print(f"COMPLETED at src={result.reference_source_time_s:.3f}")
                break


if __name__ == "__main__":
    run()
