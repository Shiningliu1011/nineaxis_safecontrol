#!/usr/bin/env python3
"""Full live demo: transition planning, smooth execution, moving obstacle, redundancy.

Shows the complete pipeline in MuJoCo:
  Phase 1 — Arm at zero config (2s)
  Phase 2 — Plan zero->first_IK transition via AEB-RRT*
  Phase 3 — Smoothly replay the transition in MuJoCo (/joint_states)
  Phase 4 — Sweep a sphere across the link column DURING execution
  Phase 5 — Show redundant-DOF: two IK configs, same tool0, obstacle blocks one
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import rclpy
import scipy.io
from pymoveit2 import MoveIt2
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"]
TRAJECTORY_OFFSET = (0.0, 0.343, 1.587)
SPHERE_RADIUS = 0.07
REPLAY_SPEED = 12  # trajectory points per second


def main() -> int:
    rclpy.init()
    node = Node("full_demo")
    moveit = MoveIt2(
        node=node, joint_names=list(JOINTS),
        base_link_name="base_link", end_effector_name="tool0", group_name="arm",
    )
    js_pub = node.create_publisher(JointState, "/joint_states", 10)
    import rclpy as r

    for _ in range(15):
        r.spin_once(node, timeout_sec=0.1)

    # ---------------------------------------------------------------
    #  Prepare: solve IK for first waypoint (interior solution)
    # ---------------------------------------------------------------
    d = scipy.io.loadmat(Path(__file__).resolve().parents[2] / "data" / "nurbs" / "ik_input.mat")
    ik = d["ik_input"][0, 0]
    pos_mm = np.asarray(ik["position_series"], dtype=float)
    offset = np.asarray(TRAJECTORY_OFFSET, dtype=float)
    positions = pos_mm / 1000.0 + offset

    # Surface-normal quaternion
    axis = np.array([0.0, 1.0, 0.0])
    helper = np.array([1.0, 0.0, 0.0])
    u = np.cross(axis, helper); u /= np.linalg.norm(u)
    v = np.cross(axis, u); v /= np.linalg.norm(v)
    px = positions @ u; py = positions @ v
    A = np.column_stack((px, py, np.ones(len(px))))
    coeff, *_ = np.linalg.lstsq(A, -(px * px + py * py), rcond=None)
    cx, cy = -0.5 * coeff[0], -0.5 * coeff[1]
    av = positions @ axis; ac = 0.5 * (av.min() + av.max())
    centre = cx * u + cy * v + ac * axis
    rel = positions[0] - centre; radial = rel - axis * np.dot(rel, axis)
    rlen = np.linalg.norm(radial); col_x = -radial / rlen; col_y_ = axis
    col_z = np.cross(col_x, col_y_); col_z /= np.linalg.norm(col_z)
    col_y_ = np.cross(col_z, col_x)
    m = np.column_stack((col_x, col_y_, col_z)); tr = np.trace(m)
    if tr > 0: s = np.sqrt(tr + 1) * 2; qw = 0.25 * s; qx = (m[2, 1] - m[1, 2]) / s; qy = (m[0, 2] - m[2, 0]) / s; qz = (m[1, 0] - m[0, 1]) / s
    elif m[1, 1] > m[2, 2]: s = np.sqrt(1 + m[1, 1] - m[0, 0] - m[2, 2]) * 2; qw = (m[0, 2] - m[2, 0]) / s; qx = (m[0, 1] + m[1, 0]) / s; qy = 0.25 * s; qz = (m[1, 2] + m[2, 1]) / s
    else: s = np.sqrt(1 + m[2, 2] - m[0, 0] - m[1, 1]) * 2; qw = (m[1, 0] - m[0, 1]) / s; qx = (m[0, 2] + m[2, 0]) / s; qy = (m[1, 2] + m[2, 1]) / s; qz = 0.25 * s
    L = np.sqrt(qw * qw + qx * qx + qy * qy + qz * qz); quat = (qx / L, qy / L, qz / L, qw / L)

    # Default IK (links at x=0)
    sol_default = moveit.compute_ik(
        position=tuple(float(v) for v in positions[0]), quat_xyzw=quat,
        ik_link_name="tool0", start_joint_state=None, wait_for_server_timeout_sec=2.0,
    )
    ik_first = [float(dict(zip(sol_default.name, sol_default.position))[j]) for j in JOINTS]
    node.get_logger().info(f"First IK waypoint (interior): J1={ik_first[0]:.3f} ...")

    # Pad for JointState publishers (need 9 values)
    def _pub_js(vals):
        js = JointState(); js.name = list(JOINTS)
        js.position = [float(x) for x in vals]; js_pub.publish(js)

    # ---------------------------------------------------------------
    #  Phase 1: Show arm at zero config
    # ---------------------------------------------------------------
    node.get_logger().info("=== Phase 1: Arm at zero ===")
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.5:
        _pub_js([0.0] * 9); r.spin_once(node, timeout_sec=0.03); time.sleep(0.05)

    # ---------------------------------------------------------------
    #  Phase 2: Plan transition zero -> first_IK
    # ---------------------------------------------------------------
    node.get_logger().info("=== Phase 2: Planning transition zero -> first IK ===")
    moveit.pipeline_id = "ompl"; moveit.planner_id = "AEBRRTstarFaithfulConfigDefault"
    moveit.allowed_planning_time = 10.0; moveit.num_planning_attempts = 1
    moveit.max_velocity = 0.2; moveit.max_acceleration = 0.2
    t_plan = time.monotonic()
    traj = moveit.plan(
        joint_positions=ik_first, joint_names=list(JOINTS),
        tolerance_joint_position=0.001, start_joint_state=None,
    )
    dt_plan = time.monotonic() - t_plan
    if traj is None or not traj.points:
        node.get_logger().error("Planning failed!"); return 1
    node.get_logger().info(
        f"Planned {len(traj.points)} points in {dt_plan:.3f}s"
    )

    # ---------------------------------------------------------------
    #  Phase 3: Replay transition smoothly (arm moves in MuJoCo)
    # ---------------------------------------------------------------
    node.get_logger().info("=== Phase 3: Replaying transition ===")
    for pt in traj.points:
        _pub_js(pt.positions); r.spin_once(node, timeout_sec=0.01)
        time.sleep(1.0 / REPLAY_SPEED)

    # Hold at goal for 2s
    _pub_js(ik_first)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0:
        r.spin_once(node, timeout_sec=0.03); time.sleep(0.05)

    # ---------------------------------------------------------------
    #  Phase 4: Move sphere across link column WHILE arm is at goal
    # ---------------------------------------------------------------
    node.get_logger().info("=== Phase 4: Moving obstacle sweep (arm at goal) ===")
    moveit.add_collision_sphere(
        "demo_sphere", radius=SPHERE_RADIUS, position=(0.0, 0.0, -5.0),
        quat_xyzw=(0.0, 0.0, 0.0, 1.0), frame_id="base_link",
    )
    time.sleep(0.5)
    sweep_x = list(np.linspace(-0.30, 0.30, 16))
    for x in sweep_x:
        pos = (float(x), 0.343, 0.80)
        moveit.move_collision("demo_sphere", pos, (0.0, 0.0, 0.0, 1.0), "base_link")
        _pub_js(ik_first); r.spin_once(node, timeout_sec=0.05)
        time.sleep(0.25)
    moveit.remove_collision_object("demo_sphere")
    time.sleep(1.0)

    # ---------------------------------------------------------------
    #  Phase 5: Show redundant-DOF: same tool0, two joint configs
    # ---------------------------------------------------------------
    node.get_logger().info("=== Phase 5: Redundant-DOF avoidance demo ===")

    # Alt IK (links shifted to x≈-0.1, same tool0 position)
    seed = JointState(); seed.name = list(JOINTS)
    seed.position = [0.3, 0.5, -0.5, 0.5, -2.0, 0.5, 0.5, 0.5, -0.5]
    sol_alt = moveit.compute_ik(
        position=tuple(float(v) for v in positions[0]), quat_xyzw=quat,
        ik_link_name="tool0", start_joint_state=seed, wait_for_server_timeout_sec=2.0,
    )
    ik_alt = [float(dict(zip(sol_alt.name, sol_alt.position))[j]) for j in JOINTS]

    # Show alt config (arm links shifted left)
    node.get_logger().info("Showing alt IK (links at x≈-0.1)...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0:
        _pub_js(ik_alt); r.spin_once(node, timeout_sec=0.03); time.sleep(0.05)

    # Show default config (links at x=0)
    node.get_logger().info("Showing default IK (links at x=0)...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0:
        _pub_js(ik_first); r.spin_once(node, timeout_sec=0.03); time.sleep(0.05)

    # ADD obstacle at x=0 (blocks default but NOT alt)
    node.get_logger().info("Adding obstacle at x=0 (blocks default config)...")
    moveit.add_collision_box(
        "center_block", size=(0.08, 0.12, 0.40),
        position=(0.0, 0.35, 0.85), quat_xyzw=(0.0, 0.0, 0.0, 1.0), frame_id="base_link",
    )
    time.sleep(2.0)
    moveit.remove_collision_object("center_block")
    time.sleep(1.0)

    node.get_logger().info("Demo complete!")
    node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
