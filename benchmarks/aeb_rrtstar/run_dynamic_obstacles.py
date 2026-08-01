#!/usr/bin/env python3
"""Dynamic + suddenly-appearing obstacle feasibility benchmark (real MoveIt2 FCL).

Drives the live move_group planning scene through two obstacle scenarios and
checks whether AEB-RRT* (and RRTConnect baseline) still finds a collision-free
transition from the zero config to the first interior IK waypoint.

Prerequisites (must be running):
  ros2 launch models/ninezzhou_moveit_config/launch/demo.launch.py

Scenario A — moving obstacle: a sphere sweeps across the transition corridor.
  Each sample position is published via /collision_object MOVE, the scene is
  pose-synced, then both planners plan zero -> first_ik.  The sphere passes
  exactly through the corridor midpoint, so the sweep should show
  success(edge) -> blocked(centre) -> success(edge).

Scenario B — suddenly-appearing obstacle: a sphere is ADDed at / near the
  corridor midpoint (lateral offset sweep).  The old baseline path is checked
  against the current scene (expected to collide), then both planners replan.

Run:  source install/setup.bash && python3 benchmarks/aeb_rrtstar/run_dynamic_obstacles.py
"""

from __future__ import annotations

import json
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_real_fcl_benchmark as rfb  # noqa: E402


# ---------------------------------------------------------------------------
#  Scenario constants (this benchmark owns the scenario, no YAML)
# ---------------------------------------------------------------------------
SWEEP_LENGTH_M = 0.30
SWEEP_SAMPLES = 10
SPHERE_RADIUS_M = 0.06
LATERAL_OFFSETS_M = (0.0, 0.03, 0.06)
SERVICE_TIMEOUT_S = 5.0
SCENE_SYNC_TIMEOUT_S = 5.0
POSITION_TOLERANCE_M = 0.01

PLANNERS = [
    ("aeb_rrtstar", "AEBRRTstarFaithfulConfigDefault"),
    ("rrtconnect", "RRTConnectkConfigDefault"),
]


# ---------------------------------------------------------------------------
#  Service helpers (call_async + spin, never blocking Client.call())
# ---------------------------------------------------------------------------
def make_service_client(node: Node, srv_type, srv_name: str):
    client = node.create_client(srv_type, srv_name)
    if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S):
        raise RuntimeError(f"Service {srv_name} not available after {SERVICE_TIMEOUT_S}s")
    return client


def call_service(node: Node, client, request, timeout_s: float = SERVICE_TIMEOUT_S):
    if not client.wait_for_service(timeout_sec=min(0.5, timeout_s)):
        return None
    future = client.call_async(request)
    remaining_s = timeout_s
    while remaining_s > 0.0 and not future.done():
        rclpy.spin_once(node, timeout_sec=min(0.05, remaining_s))
        remaining_s -= 0.05
    if not future.done():
        return None
    return future.result()


def wait_obstacle_pose(
    node: Node,
    scene_client,
    object_id: str,
    target_pos,
    republish=None,
    tolerance_m: float = POSITION_TOLERANCE_M,
    timeout_s: float = SCENE_SYNC_TIMEOUT_S,
) -> bool:
    """Poll /get_planning_scene until the object pose matches target_pos.

    Pose-level sync is required for MOVE/ADD (ID presence alone is not enough).
    *republish* is a zero-arg callable re-sending the latest MOVE/ADD, invoked
    each loop iteration so a dropped first message is retried.
    """
    request = GetPlanningScene.Request()
    request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    target = np.asarray(target_pos, dtype=float)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if republish is not None:
            republish()
        response = call_service(node, scene_client, request, timeout_s=0.5)
        if response is not None:
            for obj in response.scene.world.collision_objects:
                if obj.id == object_id:
                    if np.linalg.norm(
                        np.asarray(
                            [obj.pose.position.x, obj.pose.position.y, obj.pose.position.z],
                            dtype=float,
                        )
                        - target
                    ) <= tolerance_m:
                        return True
        time.sleep(0.15)
    node.get_logger().warning(
        f"Timed out waiting for planning scene object {object_id} to reach "
        f"position {target_pos}"
    )
    return False


def state_valid(node: Node, validity_client, joints, positions) -> bool | None:
    """One /check_state_validity call.  None = service failure (not invalidity)."""
    request = GetStateValidity.Request()
    request.robot_state.joint_state.name = list(joints)
    request.robot_state.joint_state.position = [float(v) for v in positions]
    request.group_name = "arm"
    response = call_service(node, validity_client, request, timeout_s=1.0)
    return bool(response.valid) if response is not None else None


def check_path_validity(
    node: Node,
    validity_client,
    path_states,
    joints,
    edge_samples: int = 2,
) -> tuple[bool | None, bool | None, list[str]]:
    """Validate every waypoint + interpolated edge point against the current scene."""
    if not path_states:
        return None, None, []
    contacts: list[str] = []
    for state in path_states:
        valid = state_valid(node, validity_client, joints, state.position)
        if valid is False:
            contacts.append(f"state@{state.position}")
            return False, None, contacts
        if valid is None:
            return None, None, contacts
    for first, second in zip(path_states, path_states[1:]):
        for t in np.linspace(0.0, 1.0, edge_samples + 2)[1:-1]:
            interp = [
                float(a) + t * (float(b) - float(a))
                for a, b in zip(first.position, second.position)
            ]
            valid = state_valid(node, validity_client, joints, interp)
            if valid is False:
                contacts.append(f"edge@t={t:.2f}")
                return True, False, contacts
            if valid is None:
                return True, None, contacts
    return True, True, contacts


def plan_with_states(
    moveit, planner_id: str, goal: JointState
) -> tuple[bool, float, list[JointState]]:
    """Plan zero -> goal with the given planner and return per-point JointStates."""
    moveit.pipeline_id = "ompl"
    moveit.planner_id = planner_id
    moveit.allowed_planning_time = rfb.PLANNING_TIME_S
    moveit.num_planning_attempts = 1
    moveit.max_velocity = 0.2
    moveit.max_acceleration = 0.2
    t0 = time.monotonic()
    trajectory = moveit.plan(
        joint_positions=list(goal.position),
        joint_names=list(rfb.JOINTS),
        tolerance_joint_position=0.001,
        start_joint_state=rfb.zero_start(),
    )
    dt = time.monotonic() - t0
    if trajectory is None or not trajectory.points:
        return False, dt, []
    states: list[JointState] = []
    for point in trajectory.points:
        state = JointState()
        state.name = list(rfb.JOINTS)
        state.position = list(point.positions)
        states.append(state)
    return True, dt, states


def tool0_positions(
    node: Node, moveit, start: JointState, goal: JointState, n_samples: int = 16
) -> list[tuple[float, float, float]]:
    """Tool0 positions along the straight-line joint interpolation in base_link."""
    start_pos = np.asarray(start.position, dtype=float)
    goal_pos = np.asarray(goal.position, dtype=float)
    positions: list[tuple[float, float, float]] = []
    for t in np.linspace(0.0, 1.0, n_samples):
        state = JointState()
        state.name = list(start.name)
        state.position = list(start_pos + t * (goal_pos - start_pos))
        poses = moveit.compute_fk(joint_state=state, fk_link_names=["tool0"])
        if not poses:
            raise RuntimeError(f"compute_fk failed for interpolation t={t:.3f}")
        pose = poses[0]
        positions.append((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
    return positions


# ---------------------------------------------------------------------------
#  Result record
# ---------------------------------------------------------------------------
@dataclass
class RunRecord:
    scenario: str
    obstacle_id: str
    planner_id: str
    success: bool | None
    plan_time_s: float | None = None
    num_path_points: int | None = None
    all_states_valid: bool | None = None
    all_edges_valid: bool | None = None
    old_path_now_collides: bool | None = None
    scene_sync_ok: bool = True
    obstacle_position: tuple[float, float, float] | None = None
    obstacle_quaternion_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    sweep_t: float | None = None
    lateral_offset_m: float | None = None
    notes: str = ""
    extra: dict = field(default_factory=dict)


def record_to_dict(record: RunRecord) -> dict:
    data = {
        "scenario": record.scenario,
        "obstacle_id": record.obstacle_id,
        "shape": "sphere",
        "size": [SPHERE_RADIUS_M],
        "frame_id": "base_link",
        "planner_id": record.planner_id,
        "obstacle_position": list(record.obstacle_position) if record.obstacle_position else None,
        "obstacle_quaternion_xyzw": list(record.obstacle_quaternion_xyzw),
        "sweep_t": record.sweep_t,
        "lateral_offset_m": record.lateral_offset_m,
        "success": record.success,
        "plan_time_s": record.plan_time_s,
        "num_path_points": record.num_path_points,
        "all_states_valid": record.all_states_valid,
        "all_edges_valid": record.all_edges_valid,
        "old_path_now_collides": record.old_path_now_collides,
        "scene_sync_ok": record.scene_sync_ok,
        "notes": record.notes,
    }
    data.update(record.extra)
    return data


def summarize(records: list[RunRecord]) -> None:
    print("\n=== SUMMARY ===")
    print(f"{'scenario':<14} {'t/off':>6} {'planner':<12} {'ok':>4} {'time_s':>7} "
          f"{'states':>6} {'states_ok':>9} {'old_coll':>8}")
    for record in records:
        key = record.sweep_t if record.sweep_t is not None else record.lateral_offset_m
        old = record.old_path_now_collides
        print(
            f"{record.scenario:<14} {key!s:>6} {record.planner_id.split('_')[0]:<12} "
            f"{'OK' if record.success else 'FAIL' if record.success is False else '--':>4} "
            f"{record.plan_time_s if record.plan_time_s is not None else float('nan'):>7.3f} "
            f"{record.num_path_points if record.num_path_points is not None else 0:>6} "
            f"{record.all_states_valid if record.all_states_valid is not None else 'n/a':>9} "
            f"{old if old is not None else '':>8}"
        )
    ok_total = sum(1 for r in records if r.success is True)
    fail_total = sum(1 for r in records if r.success is False)
    print(f"\nTotal: {ok_total} OK / {fail_total} FAIL / "
          f"{len(records) - ok_total - fail_total} unknown")


def main() -> int:
    rclpy.init()
    node = Node("dynamic_obstacles_benchmark")
    moveit = rfb.MoveIt2(
        node=node,
        joint_names=list(rfb.JOINTS),
        base_link_name="base_link",
        end_effector_name="tool0",
        group_name="arm",
    )
    for _ in range(15):
        rclpy.spin_once(node, timeout_sec=0.1)

    scene_client = make_service_client(node, GetPlanningScene, "get_planning_scene")
    validity_client = make_service_client(node, GetStateValidity, "check_state_validity")

    mat_path = Path(__file__).resolve().parents[2] / "data" / "nurbs" / "ik_input.mat"
    goals = rfb.load_waypoint_goals(moveit, mat_path)
    if not goals:
        node.get_logger().error("No interior IK goals solved; aborting")
        return 1
    goal = goals[0]
    start = rfb.zero_start()

    records: list[RunRecord] = []
    output_dir = Path(__file__).resolve().parent / "dynamic_obstacles"
    output_dir.mkdir(parents=True, exist_ok=True)
    run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    json_path = output_dir / f"run_{run_stamp}.json"

    # ------------------------------------------------------------------
    #  Scenario A — moving obstacle sweep
    # ------------------------------------------------------------------
    node.get_logger().info("=== Scenario A: moving obstacle sweep ===")
    tool_path = tool0_positions(node, moveit, start, goal, n_samples=16)
    corridor_mid = np.asarray(tool_path[len(tool_path) // 2], dtype=float)
    horizontal = np.asarray(tool_path[-1], dtype=float) - np.asarray(tool_path[0], dtype=float)
    horizontal[2] = 0.0
    if np.linalg.norm(horizontal) < 1e-6:
        horizontal = np.array([1.0, 0.0, 0.0])
    sweep_dir = horizontal / np.linalg.norm(horizontal)
    node.get_logger().info(
        f"Corridor midpoint tool0={corridor_mid.round(3)} sweep_dir={sweep_dir.round(3)}"
    )

    def republish_sweep():
        moveit.move_collision(
            "dyn_sweep", tuple(float(v) for v in sweep_pos), (0.0, 0.0, 0.0, 1.0), "base_link"
        )

    # Initial ADD of the sweep sphere.
    moveit.add_collision_sphere(
        "dyn_sweep", radius=SPHERE_RADIUS_M, position=(0.0, 0.0, -5.0),
        quat_xyzw=(0.0, 0.0, 0.0, 1.0), frame_id="base_link",
    )
    for i in range(SWEEP_SAMPLES):
        t = i / (SWEEP_SAMPLES - 1)
        sweep_pos = corridor_mid + (t - 0.5) * SWEEP_LENGTH_M * sweep_dir
        sync_ok = wait_obstacle_pose(
            node, scene_client, "dyn_sweep", sweep_pos, republish=republish_sweep
        )
        for key, planner_id in PLANNERS:
            ok, dt, path_states = plan_with_states(moveit, planner_id, goal)
            record = RunRecord(
                scenario="moving_sweep",
                obstacle_id="dyn_sweep",
                planner_id=planner_id,
                success=ok,
                plan_time_s=dt,
                num_path_points=len(path_states),
                scene_sync_ok=sync_ok,
                obstacle_position=tuple(float(v) for v in sweep_pos),
                sweep_t=round(t, 4),
            )
            # Validate a successful path against the current scene.
            if ok:
                all_s, all_e, contacts = check_path_validity(
                    node, validity_client, path_states, rfb.JOINTS
                )
                record.all_states_valid = all_s
                record.all_edges_valid = all_e
                if contacts:
                    record.notes = "invalid=" + ";".join(contacts)
            records.append(record)
            print(
                f"  sweep_t={t:.2f} pos={tuple(round(float(v), 3) for v in sweep_pos)} "
                f"{key} {'OK' if ok else 'FAIL'} {dt:.3f}s pts={len(path_states)} sync={sync_ok}"
            )
        # Append incrementally so a crash keeps prior data.
        json_path.write_text(
            json.dumps([record_to_dict(r) for r in records], indent=2), encoding="utf-8"
        )

    # ------------------------------------------------------------------
    #  Scenario B — suddenly-appearing obstacle
    # ------------------------------------------------------------------
    node.get_logger().info("=== Scenario B: suddenly-appearing obstacle ===")
    moveit.remove_collision_object("dyn_sweep")
    time.sleep(0.3)

    # Baseline (AEB-RRT* only): plan zero -> first_ik, keep path states.
    ok, dt, baseline_states = plan_with_states(moveit, PLANNERS[0][1], goal)
    if not ok:
        node.get_logger().error("Baseline plan failed; cannot run Scenario B")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    node.get_logger().info(f"Baseline plan OK {dt:.3f}s states={len(baseline_states)}")

    block = np.asarray(tool0_positions(node, moveit, start, goal, n_samples=17)[8], dtype=float)
    lateral_dir = np.array([-sweep_dir[1], sweep_dir[0], 0.0], dtype=float)
    lateral_dir /= np.linalg.norm(lateral_dir)

    for offset in LATERAL_OFFSETS_M:
        pos = block + offset * lateral_dir
        moveit.add_collision_sphere(
            "sudden_sphere", radius=SPHERE_RADIUS_M, position=tuple(float(v) for v in pos),
            quat_xyzw=(0.0, 0.0, 0.0, 1.0), frame_id="base_link",
        )
        sync_ok = wait_obstacle_pose(
            node, scene_client, "sudden_sphere", pos,
            republish=lambda: moveit.add_collision_sphere(
                "sudden_sphere", radius=SPHERE_RADIUS_M,
                position=tuple(float(v) for v in pos),
                quat_xyzw=(0.0, 0.0, 0.0, 1.0), frame_id="base_link",
            ),
        )
        for key, planner_id in PLANNERS:
            ok, dt, path_states = plan_with_states(moveit, planner_id, goal)
            record = RunRecord(
                scenario="sudden_appear",
                obstacle_id="sudden_sphere",
                planner_id=planner_id,
                success=ok,
                plan_time_s=dt,
                num_path_points=len(path_states),
                scene_sync_ok=sync_ok,
                obstacle_position=tuple(float(v) for v in pos),
                lateral_offset_m=float(offset),
            )
            # Does the OLD baseline path now collide with the new scene?
            all_s_old, _, contacts_old = check_path_validity(
                node, validity_client, baseline_states, rfb.JOINTS
            )
            record.old_path_now_collides = all_s_old is False
            if all_s_old is False:
                record.notes += "baseline_invalid;"
            elif all_s_old is None:
                record.notes += "baseline_unknown;"
            # Does the replanned path (if any) avoid the obstacle?
            if ok:
                all_s, all_e, contacts = check_path_validity(
                    node, validity_client, path_states, rfb.JOINTS
                )
                record.all_states_valid = all_s
                record.all_edges_valid = all_e
                if contacts:
                    record.notes += "new_invalid=" + ";".join(contacts)
            records.append(record)
            print(
                f"  offset={offset:.2f} pos={tuple(round(float(v), 3) for v in pos)} "
                f"{key} {'OK' if ok else 'FAIL'} {dt:.3f}s pts={len(path_states)} "
                f"old_collides={record.old_path_now_collides} sync={sync_ok}"
            )
        json_path.write_text(
            json.dumps([record_to_dict(r) for r in records], indent=2), encoding="utf-8"
        )

    moveit.remove_collision_object("sudden_sphere")
    time.sleep(0.3)

    summarize(records)
    json_path.write_text(
        json.dumps([record_to_dict(r) for r in records], indent=2), encoding="utf-8"
    )
    node.get_logger().info(f"Saved {json_path}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
