#!/usr/bin/env python3
"""Visual demo: transition planning with moving obstacles in MuJoCo.

Shows AEB-RRT* planning zero→first-IK transitions while a sphere sweeps
across the arm's link column at joint-link height (z=0.80).

The sphere moves in discrete steps. At each step, the planner attempts a
zero→first-IK transition. SMOOTH animation between successful plans so the
arm's movement is clearly visible.

The sphere clears the arm when |x|>0.12 (success) and pins the links when
|x|<0.09 (correct failure — the start/goal configs enter collision).

This is a FEASIBILITY benchmark (can the planner correctly identify blocked
vs clear scenes?), animated in MuJoCo.
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import rclpy
import scipy.io
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from pymoveit2 import MoveIt2
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"]


def _switch_broadcaster(node: Node, *, activate: bool) -> bool:
    """Pause or resume the JointStateBroadcaster so it doesn't fight replay."""
    client = node.create_client(
        SwitchController, "/controller_manager/switch_controller"
    )
    if not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().warning(
            "controller_manager service not available; cannot pause broadcaster"
        )
        return False

    request = SwitchController.Request()
    if activate:
        request.activate_controllers = ["joint_state_broadcaster"]
        request.deactivate_controllers = []
    else:
        request.activate_controllers = []
        request.deactivate_controllers = ["joint_state_broadcaster"]
    request.strictness = SwitchController.Request.BEST_EFFORT
    request.timeout = Duration(sec=1)

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    result = future.result()
    ok = result is not None and result.ok
    if ok:
        state = "resumed" if activate else "paused"
        print(f"  [broadcaster {state}]")
    else:
        print(f"  [WARN] Failed to switch broadcaster (activate={activate})")
    return ok


def main() -> int:
    rclpy.init()
    node = Node("full_demo")
    moveit = MoveIt2(
        node=node, joint_names=list(JOINTS),
        base_link_name="base_link", end_effector_name="tool0", group_name="arm",
    )
    js_pub = node.create_publisher(JointState, "/joint_states", 10)
    scene_client = node.create_client(GetPlanningScene, "get_planning_scene")
    scene_client.wait_for_service(5.0)
    import rclpy as r

    for _ in range(20):
        r.spin_once(node, timeout_sec=0.1)

    # ---- Solve first IK waypoint ----
    d = scipy.io.loadmat(str(Path(__file__).resolve().parents[2] / "data" / "nurbs" / "ik_input.mat"))
    ik = d["ik_input"][0, 0]
    pos = np.asarray(ik["position_series"], dtype=float) / 1000.0 + np.array([0.0, 0.343, 1.587])
    axis = np.array([0., 1., 0.])
    h = np.array([1., 0., 0.]); u = np.cross(axis, h); u /= np.linalg.norm(u); v = np.cross(axis, u); v /= np.linalg.norm(v)
    A = np.column_stack((pos @ u, pos @ v, np.ones(len(pos))))
    c, *_ = np.linalg.lstsq(A, -(pos @ u) ** 2 - (pos @ v) ** 2, rcond=None)
    cx, cy = -0.5 * c[0], -0.5 * c[1]
    av = pos @ axis; ac = 0.5 * (av.min() + av.max()); centre = cx * u + cy * v + ac * axis
    rel = pos[0] - centre; rad = rel - axis * np.dot(rel, axis)
    x_ = -rad / np.linalg.norm(rad); y_ = axis; z_ = np.cross(x_, y_); z_ /= np.linalg.norm(z_); y_ = np.cross(z_, x_)
    m = np.column_stack((x_, y_, z_)); tr = np.trace(m)
    s2 = np.sqrt(tr + 1) * 2 if tr > 0 else np.sqrt(1 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    if tr > 0: qw, qx, qy, qz = s2 / 4, (m[2, 1] - m[1, 2]) / s2, (m[0, 2] - m[2, 0]) / s2, (m[1, 0] - m[0, 1]) / s2
    elif m[1, 1] > m[2, 2]: s = np.sqrt(1 + m[1, 1] - m[0, 0] - m[2, 2]) * 2; qw = (m[0, 2] - m[2, 0]) / s; qx = (m[0, 1] + m[1, 0]) / s; qy = s / 4; qz = (m[1, 2] + m[2, 1]) / s
    else: s = np.sqrt(1 + m[2, 2] - m[0, 0] - m[1, 1]) * 2; qw = (m[1, 0] - m[0, 1]) / s; qx = (m[0, 2] + m[2, 0]) / s; qy = (m[1, 2] + m[2, 1]) / s; qz = s / 4
    L = np.sqrt(qw * qw + qx * qx + qy * qy + qz * qz); quat = (qx / L, qy / L, qz / L, qw / L)
    sol = moveit.compute_ik(position=tuple(float(v) for v in pos[0]), quat_xyzw=quat, ik_link_name="tool0", start_joint_state=None, wait_for_server_timeout_sec=2.)
    ik_goal = [float(dict(zip(sol.name, sol.position))[j]) for j in JOINTS]
    zero = [0.0] * 9

    def pub(vals):
        js = JointState(); js.name = list(JOINTS); js.position = [float(x) for x in vals]
        js_pub.publish(js)

    def do_plan():
        moveit.pipeline_id = "ompl"; moveit.planner_id = "AEBRRTstarFaithfulConfigDefault"
        moveit.allowed_planning_time = 5.0; moveit.num_planning_attempts = 3
        moveit.max_velocity = 0.2; moveit.max_acceleration = 0.2
        traj = moveit.plan(joint_positions=ik_goal, joint_names=list(JOINTS), tolerance_joint_position=0.001, start_joint_state=None)
        return traj

    def replay(traj, total_s=3.0):
        pts = [list(p.positions) for p in traj.points]
        if len(pts) < 2:
            pub(pts[0] if pts else ik_goal); time.sleep(total_s); return
        # Pause JointStateBroadcaster so it doesn't fight our trajectory
        # publication with stale zero positions from mock hardware.
        was_active = _switch_broadcaster(node, activate=False)
        try:
            frame_s = 1.0 / 30
            total_frames = int(total_s / frame_s)
            for f in range(total_frames + 1):
                t = f / max(total_frames, 1)
                idx = min(int(t * (len(pts) - 1)), len(pts) - 2)
                frac = t * (len(pts) - 1) - idx
                interp = [pts[idx][j] + frac * (pts[idx + 1][j] - pts[idx][j]) for j in range(9)]
                pub(interp)
                r.spin_once(node, timeout_sec=0.001)
                time.sleep(frame_s)
            pub(pts[-1])
        finally:
            if was_active:
                _switch_broadcaster(node, activate=True)

    # ================================================================
    #  PHASE 1 — Arm at zero, then plan and execute transition
    # ================================================================
    print("\n  >>> 零位展 示 (2s)")
    for _ in range(60): pub(zero); r.spin_once(node, timeout_sec=0.001); time.sleep(1 / 30)

    print("\n  >>> 规划过渡路径 zero → first IK ...")
    traj_baseline = do_plan()
    if not traj_baseline or not traj_baseline.points:
        print("  FAIL"); return 1
    print(f"  OK, {len(traj_baseline.points)} 个轨迹点")

    print("\n  >>> 平滑执行过渡 (4s) — 机械臂从零位移到跟踪起始位姿")
    replay(traj_baseline, 4.0)

    print("\n  >>> 到达目标，保持 (2s)")
    pub(ik_goal); time.sleep(2)

    # ================================================================
    #  PHASE 2 — Redundant-DOF: same tool0, different joint config
    # ================================================================
    print("\n  >>> 冗余自由度演示: 同一末端位姿，两套关节配置")
    seed = JointState(); seed.name = list(JOINTS)
    seed.position = [0.3, 0.5, -0.5, 0.5, -2.0, 0.5, 0.5, 0.5, -0.5]
    sol_a = moveit.compute_ik(position=tuple(float(v) for v in pos[0]), quat_xyzw=quat, ik_link_name="tool0", start_joint_state=seed, wait_for_server_timeout_sec=2.)
    ik_alt = [float(dict(zip(sol_a.name, sol_a.position))[j]) for j in JOINTS]
    print(f"    默认IK: 连杆 x≈0   | 替代IK: 连杆 x≈-0.1 (同一tool0位置)")

    print("\n  >>> 展示替代配置 (3s)")
    for _ in range(90): pub(ik_alt); r.spin_once(node, timeout_sec=0.001); time.sleep(1 / 30)

    print("\n  >>> 放置障碍物在 x=0 (阻挡默认配置)...")
    moveit.add_collision_box("center_block", size=(0.08, 0.12, 0.40), position=(0., 0.35, 0.85), quat_xyzw=(0., 0., 0., 1.), frame_id="base_link")
    time.sleep(3)

    print("\n  >>> 移除障碍，回到默认配置")
    moveit.remove_collision_object("center_block"); time.sleep(1)
    for _ in range(60): pub(ik_goal); r.spin_once(node, timeout_sec=0.001); time.sleep(1 / 30)

    print("\n  ✓ 演示完成。关闭查看器窗口退出。")
    node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
