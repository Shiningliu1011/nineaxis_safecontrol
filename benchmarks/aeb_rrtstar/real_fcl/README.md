# Real MoveIt2 FCL validation — AEB-RRT*

Date: 2026-08-01
Commit: `60e2f4c` + local fixes (see below)

## Background

The earlier C++ test suite (`test_aeb_full.cpp`, `ninezzhou_collision.h`) validates
the AEB-RRT* algorithm against a **simplified** link-geometry collision model.
This directory records the validation against the **real MoveIt2 FCL** collision
environment (the `aeb_rrtstar_ompl/AEBRRTstarPlannerManager` plugin running inside
`move_group`, with the actual URDF links for collision checking).

## Environment

- ROS2 Humble + MoveIt2 + pymoveit2
- `ros2 launch models/ninezzhou_moveit_config/launch/demo.launch.py`
  (move_group, mock ros2_control, joint_state_broadcaster, arm_trajectory_controller)
- `planning_plugin: aeb_rrtstar_ompl/AEBRRTstarPlannerManager`

## End-to-end pipeline (real config, planning only)

`config/plan_transition.yaml` (removed in the 2026-08 C1 refactor; see docs/adr/0001) with `execute_transition:=false`, `max_points=50`,
`align_tool_x_to_surface_normal:=true`, `planner_id:=AEBRRTstarFaithfulConfigDefault`.

Raw log: `validate_final.log`

```
Computed 50 surface-normal-aligned orientation(s); cylinder fitted on 14992 full trajectory samples
Continuous IK succeeded for 50 waypoint(s).
MoveIt planned a transition with 64 trajectory point(s).   <- AEB-RRT* via real FCL
```

## IK interior check

The boundary IK solution that motivated the fix
(`J6=1.483` at its `±1.48353` limit) is an **artifact of the synthetic test offset**
`trajectory_offset_m=[0,0,0.3]`, which pushed targets to the workspace edge.

With the **real** offset `[0, 0.343, 1.587]` and surface-normal alignment, the
first trajectory waypoint solves to an **interior** configuration
(`J1=0.2096, J7=0.2944, J9=-0.2944`, others ≈ 0) — all joints well inside limits.

| offset | J1 | J6 | result |
|---|---|---|---|
| `[0,0,0.3]` (test) | 0.48/0.58 | 0.91/1.48 | boundary / limit or IK fail |
| `[0,0.343,1.587]` (real) | 0.2096 | 0.0001 | interior |

## Benchmark: AEB-RRT* vs RRTConnect (real FCL)

Script: `../run_real_fcl_benchmark.py` (6 interior IK goals × 3 trials, zero start).
Raw JSON: `bench_real_fcl2.json`.

| planner | success | mean best-time |
|---|---|---|
| AEB-RRT* (`AEBRRTstarFaithfulConfigDefault`) | 6/6 | 0.0197 s |
| RRTConnect (`RRTConnectkConfigDefault`) | 6/6 | 0.0154 s |

Both plan the zero→first-IK transition collision-free through real FCL. AEB-RRT*
is ~30% slower on this easy transition but succeeds reliably; RRTConnect remains
a valid fallback.

## Fixes landed during this validation

1. **Segfault on invalid start states** — `solve()` called
   `si->cloneState(input_states.nextStart())` without checking for `nullptr`.
   MoveIt2 hands a colliding start to the planner (the `FixStartStateCollision`
   adapter could not repair it); `PlannerInputStates::nextStart()` skips invalid
   starts and returns `nullptr`, so `cloneState(nullptr)` segfaulted `move_group`.
   Fixed: guard the null return and return `INVALID_START`.

   Before: `move_group` SIGSEGV (exit -11). After: graceful `START_STATE_INVALID`
   (`AEB_FAIL: NO VALID START STATES` in the log), `move_group` stays alive.

2. **Anytime-mode connection-index swap** — in `extendTree()`, when the **goal**
   tree performed the cross-tree connection, the stored indices were
   `connect_start_idx_ = new_idx` (goal tree) and `connect_goal_idx_ = ci`
   (start tree), but `buildPath()` indexes `tree_start_` with
   `connect_start_idx_` and `tree_goal_` with `connect_goal_idx_`. Faithful mode
   never hits it (breaks on first solution from the start tree), but Anytime mode
   grew the goal tree and read out-of-bounds → segfault in `buildPath()`.
   Fixed: index swap when `is_goal_tree`.

   The full C++ suite (`test_aeb_full.cpp`) now passes **19/19** including the
   previously-crashing `Anytime solve (WITH collision)` test.
