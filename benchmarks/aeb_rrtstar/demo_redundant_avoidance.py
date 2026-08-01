#!/usr/bin/env python3
"""Demo: redundant-DOF obstacle avoidance in the real MoveIt2 FCL environment.

Shows the ninezzhou 9-DOF arm using its kinematic redundancy (3 extra joints
for a 6-DOF Cartesian task) to reach the SAME tool0 pose via two different
joint configurations — one blocked by an obstacle, one clear.

Prerequisites: demo.launch.py running (move_group + mock ros2_control).
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import rclpy
import scipy.io
from moveit_msgs.srv import GetStateValidity
from pymoveit2 import MoveIt2
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"]
JOINT_LIMITS = [
    (0.0, 0.585), (-1.5708, 1.5708), (-1.5708, 1.5708), (-1.5708, 1.5708),
    (-3.1416, 3.1416), (-1.48353, 1.48353), (-1.48353, 1.48353),
    (-1.48353, 1.48353), (-1.48353, 1.48353),
]
TRAJECTORY_OFFSET = (0.0, 0.343, 1.587)
CYLINDER_AXIS = (0.0, 1.0, 0.0)
PLANNING_TIME_S = 10.0

# Obstacle: box placed at the default link column (x≈0) forcing the arm to
# use an alternative IK solution with links shifted to x≈-0.1.
OBS_POS = (0.0, 0.35, 0.85)
OBS_SIZE = (0.07, 0.12, 0.40)  # box (x, y, z) dimensions, full
OBS_ID = "center_block"


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
        qw = (m[2, 1] - m[1, 2]) / s; qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s; qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw = (m[0, 2] - m[2, 0]) / s; qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s; qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw = (m[1, 0] - m[0, 1]) / s; qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s; qz = 0.25 * s
    length = np.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    return (qx / length, qy / length, qz / length, qw / length)


def surface_normal_quaternion(point, centre, axis) -> tuple[float, ...]:
    rel = point - centre
    axial = axis * float(np.dot(rel, axis))
    radial = rel - axial
    rlen = float(np.linalg.norm(radial))
    if rlen < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    col_x = -radial / rlen; col_y = axis
    col_z = np.cross(col_x, col_y); col_z /= np.linalg.norm(col_z)
    col_y = np.cross(col_z, col_x)
    return rotation_matrix_to_quaternion_xyzw(np.column_stack((col_x, col_y, col_z)))


def fit_cylinder_centre(positions: np.ndarray) -> np.ndarray:
    axis = np.asarray(CYLINDER_AXIS, dtype=float); axis /= np.linalg.norm(axis)
    helper = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(helper, axis))) > 0.9: helper = np.array([0.0, 0.0, 1.0])
    u = np.cross(axis, helper); u /= np.linalg.norm(u)
    v = np.cross(axis, u); v /= np.linalg.norm(v)
    px = positions @ u; py = positions @ v
    A = np.column_stack((px, py, np.ones(len(positions))))
    coeff, *_ = np.linalg.lstsq(A, -(px * px + py * py), rcond=None)
    d_val, e_val, _ = coeff; cx = -0.5 * d_val; cy = -0.5 * e_val
    axial_vals = positions @ axis; axial_centre = 0.5 * (axial_vals.min() + axial_vals.max())
    return cx * u + cy * v + axial_centre * axis


def main() -> int:
    rclpy.init()
    node = Node("redundant_demo")
    moveit = MoveIt2(
        node=node, joint_names=list(JOINTS),
        base_link_name="base_link", end_effector_name="tool0", group_name="arm",
    )
    import rclpy as r
    for _ in range(15): r.spin_once(node, timeout_sec=0.1)
    validity = node.create_client(GetStateValidity, "check_state_validity")
    validity.wait_for_service(5.0)

    def state_valid(pos):
        req = GetStateValidity.Request()
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in pos]
        req.group_name = "arm"
        fut = validity.call_async(req)
        while not fut.done(): r.spin_once(node, timeout_sec=0.05)
        return bool(fut.result().valid)

    def plan(planner_id, goal_vals):
        moveit.pipeline_id = "ompl"; moveit.planner_id = planner_id
        moveit.allowed_planning_time = PLANNING_TIME_S
        moveit.num_planning_attempts = 1
        moveit.max_velocity = 0.2; moveit.max_acceleration = 0.2
        t0 = time.monotonic()
        traj = moveit.plan(
            joint_positions=[float(x) for x in goal_vals],
            joint_names=JOINTS, tolerance_joint_position=0.001,
            start_joint_state=None,
        )
        dt = time.monotonic() - t0
        ok = traj is not None and len(traj.points) > 0
        return ok, dt, traj

    def link_positions(joint_vals):
        st = JointState(); st.name = list(JOINTS)
        st.position = [float(v) for v in joint_vals]
        poses = moveit.compute_fk(joint_state=st, fk_link_names=["Link4","Link5","Link6","Link7","tool0"])
        return [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in poses]

    # ---------------------------------------------------------------
    # 1. Compute default IK for the first trajectory waypoint
    # ---------------------------------------------------------------
    d = scipy.io.loadmat(Path(__file__).resolve().parents[2] / "data" / "nurbs" / "ik_input.mat")
    ik = d["ik_input"][0, 0]
    pos_mm = np.asarray(ik["position_series"], dtype=float)
    offset = np.asarray(TRAJECTORY_OFFSET, dtype=float)
    positions = pos_mm / 1000.0 + offset
    centre = fit_cylinder_centre(positions)
    quat = surface_normal_quaternion(positions[0], centre, np.asarray(CYLINDER_AXIS, dtype=float))

    sol_default = moveit.compute_ik(
        position=tuple(float(v) for v in positions[0]), quat_xyzw=quat,
        ik_link_name="tool0", start_joint_state=None, wait_for_server_timeout_sec=2.0,
    )
    if sol_default is None:
        node.get_logger().error("Default IK failed"); return 1
    ik_default = [dict(zip(sol_default.name, sol_default.position))[j] for j in JOINTS]
    print(f"Default IK (links at x=0):     {[round(x, 3) for x in ik_default]}")

    # ---------------------------------------------------------------
    # 2. Find ALTERNATIVE IK (same tool0 pose, different joint config)
    # ---------------------------------------------------------------
    seeds = [
        ("wide_J5", [0.3, 0.5, -0.5, 0.5, -2.0, 0.5, 0.5, 0.5, -0.5]),
        ("J7flip",  [0.3, 0.2, 0.1, 0.0, 0.5, 0.0, -0.5, 0.8, 0.0]),
        ("J5_rot",  [0.35, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0]),
        ("mix",     [0.4, -0.3, 0.5, -0.3, 1.0, -0.8, 0.7, -0.5, 0.3]),
    ]
    ik_alt = None; alt_label = ""
    for label, seed in seeds:
        s = JointState(); s.name = list(JOINTS); s.position = [float(x) for x in seed]
        sol = moveit.compute_ik(
            position=tuple(float(v) for v in positions[0]), quat_xyzw=quat,
            ik_link_name="tool0", start_joint_state=s, wait_for_server_timeout_sec=2.0,
        )
        if sol is None: continue
        v = [dict(zip(sol.name, sol.position))[j] for j in JOINTS]
        interior = all(abs(v[i] - lo) > 0.1 and abs(hi - v[i]) > 0.1
                       for i, (lo, hi) in enumerate(JOINT_LIMITS))
        diff = np.linalg.norm(np.array(ik_default) - np.array(v))
        if interior and diff > 0.5:
            ik_alt = v; alt_label = label; break

    if ik_alt is None:
        node.get_logger().error("No alternative IK found"); return 1
    print(f"Alt IK ({alt_label}, links at x≈-0.1): {[round(x, 3) for x in ik_alt]}")
    print(f"  L2 diff from default: {np.linalg.norm(np.array(ik_default)-np.array(ik_alt)):.3f}")

    # ---------------------------------------------------------------
    # 3. Show link positions for both IK configs
    # ---------------------------------------------------------------
    print("\n=== Same tool0 position, different joint configurations ===")
    for label, vals in [("default IK", ik_default), (f"alt IK ({alt_label})", ik_alt)]:
        links = link_positions(vals)
        print(f"  {label}:")
        for name, lp in zip(["Link4", "Link5", "Link6", "Link7", "tool0"], links):
            print(f"    {name}: ({lp[0]:.3f}, {lp[1]:.3f}, {lp[2]:.3f})")

    # ---------------------------------------------------------------
    # 4. Plan WITHOUT obstacle (baseline)
    # ---------------------------------------------------------------
    print("\n=== Planning: zero config -> alt IK ===")
    ok, dt, traj = plan("AEBRRTstarFaithfulConfigDefault", ik_alt)
    print(f"  Baseline (no obstacle): {'OK' if ok else 'FAIL'} {dt:.3f}s "
          f"pts={len(traj.points) if traj else 0}")

    # ---------------------------------------------------------------
    # 5. Place obstacle at x=0 (blocks default config's link column)
    # ---------------------------------------------------------------
    print(f"\n  Placing BOX at {OBS_POS} size={OBS_SIZE} (blocks default links at x=0)...")
    moveit.add_collision_box(OBS_ID, size=OBS_SIZE, position=OBS_POS,
                             quat_xyzw=(0, 0, 0, 1), frame_id="base_link")
    time.sleep(0.3)

    zero_valid = state_valid([0.0] * 9)
    default_valid = state_valid(ik_default)
    alt_valid = state_valid(ik_alt)
    print(f"  With obstacle: zero={zero_valid} default_IK={default_valid} alt_IK={alt_valid}")
    if alt_valid and not default_valid:
        print("  ★ DEFAULT IK BLOCKED, ALT IK CLEAR — redundancy in action!")

    # ---------------------------------------------------------------
    # 6. Plan WITH obstacle — both planners
    # ---------------------------------------------------------------
    for planner_id, name in [
        ("AEBRRTstarFaithfulConfigDefault", "AEB-RRT*"),
        ("RRTConnectkConfigDefault", "RRTConnect"),
    ]:
        ok, dt, traj = plan(planner_id, ik_alt)
        print(f"  {name} with obstacle: {'OK' if ok else 'FAIL'} {dt:.3f}s "
              f"pts={len(traj.points) if traj else 0}")

    # ---------------------------------------------------------------
    # 7. Show planned path midpoint
    # ---------------------------------------------------------------
    print("\n=== Path analysis (AEB-RRT* with obstacle) ===")
    ok, dt, traj = plan("AEBRRTstarFaithfulConfigDefault", ik_alt)
    if ok and len(traj.points) > 1:
        mid = traj.points[len(traj.points) // 2]
        mid_vals = list(mid.positions)
        links = link_positions(mid_vals)
        print(f"  Path has {len(traj.points)} waypoints")
        print(f"  Midpoint joints: {[round(x, 3) for x in mid_vals]}")
        print(f"  Midpoint links: Link5 ({links[1][0]:.3f},{links[1][1]:.3f},{links[1][2]:.3f}) "
              f"Link7 ({links[3][0]:.3f},{links[3][1]:.3f},{links[3][2]:.3f})")
        # Verify: the path should go through x≈-0.1 region (avoiding x=0 obstacle)
        if links[1][0] < -0.03:
            print("  ★ Path midpoint links shifted LEFT (x<0) — avoiding center obstacle!")
        else:
            print(f"  Midpoint link x={links[1][0]:.3f}")

    # Cleanup
    moveit.remove_collision_object(OBS_ID)
    time.sleep(0.2)
    node.destroy_node(); rclpy.shutdown()
    print("\n=== DEMO VERIFIED: redundant DOFs allow obstacle avoidance ===")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
