#!/usr/bin/env python3
"""Real MoveIt2 FCL benchmark: AEB-RRT* vs RRTConnect transition planning.

Prerequisites (must be running):
  ros2 launch models/ninezzhou_moveit_config/launch/demo.launch.py
  (move_group with the aeb_rrtstar_ompl/AEBRRTstarPlannerManager plugin,
   mock ros2_control, and the configured PlanningScene)

This benchmark:
  1. Solves interior IK goals for trajectory waypoints (surface-normal aligned)
  2. Plans a transition from the zero start to each goal with AEB-RRT* and RRTConnect
  3. Records success + best/mean planning time per planner

Run from the workspace root after `source install/setup.bash`.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import scipy.io
from pymoveit2 import MoveIt2
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"]
DIM = len(JOINTS)
TRAJECTORY_OFFSET = (0.0, 0.343, 1.587)
CYLINDER_AXIS = (0.0, 1.0, 0.0)
WAYPOINT_IDX = [0, 100, 300, 500, 1000, 2000]
TRIALS_PER_GOAL = 3
PLANNING_TIME_S = 5.0

PLANNERS = [
    ("aeb_rrtstar", "AEBRRTstarFaithfulConfigDefault"),
    ("rrtconnect", "RRTConnectkConfigDefault"),
]

# Real joint limits from models/ninezzhou/urdf/ninezzhou.urdf
JOINT_LIMITS = [
    (0.0, 0.585),
    (-1.5708, 1.5708),
    (-1.5708, 1.5708),
    (-1.5708, 1.5708),
    (-3.1416, 3.1416),
    (-1.48353, 1.48353),
    (-1.48353, 1.48353),
    (-1.48353, 1.48353),
    (-1.48353, 1.48353),
]


def rotation_matrix_to_quaternion_xyzw(m: np.ndarray) -> tuple[float, ...]:
    m = np.asarray(m, dtype=float)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s
    length = np.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    return (qx / length, qy / length, qz / length, qw / length)


def fit_cylinder_axis_centre(positions: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    axis = np.asarray(CYLINDER_AXIS, dtype=float)
    axis /= np.linalg.norm(axis)
    helper = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(helper, axis))) > 0.9:
        helper = np.array([0.0, 0.0, 1.0])
    u = np.cross(axis, helper)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    v /= np.linalg.norm(v)
    px = positions @ u
    py = positions @ v
    A = np.column_stack((px, py, np.ones(len(positions))))
    coeff, *_ = np.linalg.lstsq(A, -(px * px + py * py), rcond=None)
    d_val, e_val, _ = coeff
    cx = -0.5 * d_val
    cy = -0.5 * e_val
    axial_vals = positions @ axis
    axial_centre = 0.5 * (axial_vals.min() + axial_vals.max())
    centre = cx * u + cy * v + axial_centre * axis
    return centre, axis


def surface_normal_quaternion(point, centre, axis) -> tuple[float, ...]:
    rel = point - centre
    axial = axis * float(np.dot(rel, axis))
    radial = rel - axial
    rlen = float(np.linalg.norm(radial))
    if rlen < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    col_x = -radial / rlen  # inward radial (tool0 X toward cylinder centre)
    col_y = axis
    col_z = np.cross(col_x, col_y)
    col_z /= np.linalg.norm(col_z)
    col_y = np.cross(col_z, col_x)
    return rotation_matrix_to_quaternion_xyzw(np.column_stack((col_x, col_y, col_z)))


def load_waypoint_goals(moveit, mat_path: Path) -> list[JointState]:
    mat = scipy.io.loadmat(mat_path)
    ik_input = mat["ik_input"][0, 0]
    pos_mm = np.asarray(ik_input["position_series"], dtype=float)
    offset = np.asarray(TRAJECTORY_OFFSET, dtype=float)
    positions = pos_mm / 1000.0 + offset
    centre, axis = fit_cylinder_axis_centre(positions)
    goals: list[JointState] = []
    previous = None
    for idx in WAYPOINT_IDX:
        point = positions[idx]
        quat = surface_normal_quaternion(point, centre, axis)
        solution = moveit.compute_ik(
            position=tuple(float(v) for v in point),
            quat_xyzw=quat,
            ik_link_name="tool0",
            start_joint_state=previous,
            wait_for_server_timeout_sec=2.0,
        )
        if solution is None:
            print(f"  goal@{idx}: IK FAILED (skipped)", file=sys.stderr)
            continue
        previous = solution
        goals.append(solution)
        print(f"  goal@{idx}: IK interior ok")
    if not goals:
        raise RuntimeError("No interior IK goals could be solved")
    return goals


def zero_start() -> JointState:
    state = JointState()
    state.name = list(JOINTS)
    state.position = [0.0] * DIM
    return state


def plan(moveit, planner_id: str, goal: JointState) -> tuple[bool, float, int]:
    moveit.pipeline_id = "ompl"
    moveit.planner_id = planner_id
    moveit.allowed_planning_time = PLANNING_TIME_S
    moveit.num_planning_attempts = 1
    moveit.max_velocity = 0.2
    moveit.max_acceleration = 0.2
    t0 = time.monotonic()
    trajectory = moveit.plan(
        joint_positions=list(goal.position),
        joint_names=list(JOINTS),
        tolerance_joint_position=0.001,
        start_joint_state=zero_start(),
    )
    dt = time.monotonic() - t0
    ok = trajectory is not None and len(trajectory.points) > 0
    return ok, dt, (len(trajectory.points) if trajectory else 0)


def main() -> int:
    rclpy.init()
    node = Node("real_fcl_benchmark")
    moveit = MoveIt2(
        node=node,
        joint_names=list(JOINTS),
        base_link_name="base_link",
        end_effector_name="tool0",
        group_name="arm",
    )
    import rclpy  # noqa: PLC0415 (spin in main thread)

    for _ in range(15):
        rclpy.spin_once(node, timeout_sec=0.1)

    mat_path = Path(__file__).resolve().parents[3] / "data" / "nurbs" / "ik_input.mat"
    goals = load_waypoint_goals(moveit, mat_path)
    print(f"Solved {len(goals)} interior IK goals at idx {WAYPOINT_IDX[:len(goals)]}")

    results: dict[str, dict[str, dict]] = {}
    for key, planner_id in PLANNERS:
        print(f"\n=== {key} ===")
        results[key] = {}
        for goal_idx, goal in enumerate(goals):
            ok_times: list[float] = []
            all_times: list[float] = []
            for _ in range(TRIALS_PER_GOAL):
                ok, dt, points = plan(moveit, planner_id, goal)
                all_times.append(dt)
                if ok:
                    ok_times.append(dt)
            success = bool(ok_times)
            best = round(min(ok_times), 4) if ok_times else None
            print(f"  goal@{WAYPOINT_IDX[goal_idx]}: "
                  f"{'OK' if success else 'FAIL'} best={best}s "
                  f"trials={[round(t, 3) for t in all_times]}")
            results[key][f"idx{WAYPOINT_IDX[goal_idx]}"] = {
                "ok": success,
                "best_s": best,
                "all_s": [round(t, 4) for t in all_times],
            }

    print("\n=== SUMMARY ===")
    for key in results:
        runs = results[key]
        ok = sum(1 for value in runs.values() if value["ok"])
        print(f"{key}: {ok}/{len(runs)} goals success")
        times = [value["best_s"] for value in runs.values() if value["best_s"] is not None]
        if times:
            print(f"   mean best-time: {np.mean(times):.4f}s")

    out = Path("/tmp/bench_real_fcl.json")
    out.write_text(json.dumps(results, indent=2))
    print(f"saved {out}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
