# 动态移动 + 突发障碍物可行性基准（真实 MoveIt2 FCL）

Date: 2026-08-01（修正版：障碍物放在**连杆高度**）
Planner: AEB-RRT\* (`AEBRRTstarFaithfulConfigDefault`) vs RRTConnect baseline
Environment: 真实 MoveIt2 FCL（move_group + `aeb_rrtstar_ompl` 插件 + 4 固定障碍）
Raw data: `run_20260801_175632.json`（26 条记录）, `run_20260801_175632.log`

## 目的

现有测试只有固定障碍物。本基准验证 **动态移动障碍物** 和 **突然出现的障碍物**
两类场景下，零位→首 IK 内部解的过渡规划是否仍然可行。规划是快照式的（每次
规划只看到当前场景），因此移动障碍用「MOVE → 场景同步 → 重规划」的离散循环抽象。

## 障碍物放置：连杆高度（关键修正）

机械臂在 base_link 是**沿 Z 的竖直柱**：所有关节连杆都在 x≈0, y≈0.343 的窄带上
（Link3 z≈0.23–0.44, Link4 z≈0.45–0.66, Link5/6 z≈0.79–1.00, Link7 z≈0.93–1.14），
只有 tool0 末端到达 z≈1.5。固定障碍物（`obstacles.yaml`）都在 **z=0.4–0.9 的连杆
高度**。

因此动态障碍也放在连杆高度 **z=0.80**（Link5/6 区域），**横向扫过臂柱**，而不是
放在 tool0 末端轨迹高度。实测确定：障碍球（r=0.06m）离臂柱 |x|≥0.15m 或 dy≥0.15m
时 start/goal 保持有效（可规划）；进入 |x|≤0.09m 范围时压住连杆 → start/goal 无效。

## 场景 A — 移动障碍扫描（连杆高度）

半径 0.06m 球体 `dyn_sweep` 在连杆高度 z=0.80 沿 x 横向扫过臂柱（10 位置，
跨度 ±0.25m）。t=0.5 恰好在臂柱上（必堵连杆），边缘离柱 0.25m（畅通）。

| sweep_t | x位置 | AEB-RRT\* | RRTConnect | start/goal |
|---------|-------|-----------|------------|------------|
| 0.00 | -0.250 | OK | OK | 有效 |
| 0.11 | -0.194 | OK | OK | 有效 |
| 0.22 | -0.139 | OK | OK | 有效 |
| 0.33 | -0.083 | **FAIL** | **FAIL** | 被堵 |
| 0.44 | -0.028 | FAIL | FAIL | 被堵 |
| 0.56 | +0.028 | FAIL | FAIL | 被堵 |
| 0.67 | +0.083 | FAIL | FAIL | 被堵 |
| 0.78 | +0.139 | OK | OK | 有效 |
| 0.89 | +0.194 | OK | OK | 有效 |
| 1.00 | +0.250 | OK | OK | 有效 |

**结论**：两规划器在障碍压住连杆（|x|≤0.09）时正确失败——障碍使 start/goal 配置
本身进入碰撞，规划器如实报告无解，不是崩溃或假成功。障碍离开连杆后立即恢复成功。
所有成功路径经 `/check_state_validity` 验证全部有效。AEB-RRT\* 在边界处（t=0.22，
x=-0.139）找到绕障路径用时 0.043s，RRTConnect 用 0.254s——AEB-RRT\* 更快找到
贴近连杆的绕行路径。

## 场景 B — 突然出现的障碍物（连杆高度）

先规划基线路径，然后在连杆高度 (x=0, y=0.343, z=0.80) 沿 y 横向偏移 ADD 球体。

| 偏移(m) | y位置 | AEB-RRT\* | RRTConnect | start/goal | 旧路径碰撞 |
|---------|-------|-----------|------------|------------|------------|
| 0.00 | 0.343 | FAIL | FAIL | 被堵 | True |
| 0.10 | 0.443 | FAIL | FAIL | 被堵 | True |
| 0.20 | 0.543 | OK | OK | 有效 | False |

**结论**：突发障碍在连杆高度偏移 0/0.10m 时压住连杆 → 两规划器正确失败，且
`old_path_now_collides=True` 证明 ADD 确实改变了场景（旧路径不再有效）。偏移 0.20m
时障碍完全离开臂柱 → 两规划器都成功（此时旧路径也未碰到障碍，符合预期）。

## 关键机制

- **基准脚本** `run_dynamic_obstacles.py`：`move_collision()`（MOVE）/ `add_collision_sphere()`
  （ADD）发布到 `/collision_object`（项目此前未用过的 pymoveit2 接口）；`wait_obstacle_pose()`
  pose 级场景同步；`check_path_validity()` 用 `/check_state_validity` 验证路径；
  每次扫掠记录 `start_valid`/`goal_valid` 以区分「障碍压住连杆」与「规划器超时」。
- **MuJoCo 实时可视化**（`mujoco_viewer_with_cylinder.py`）：16 个 free-joint 障碍槽
  （8 球/4 盒/4 柱）订阅 `/collision_object` 实时渲染 ADD/MOVE/REMOVE；free joint 在
  worldbody 顶层，用 `_yup_to_zup_world()` 手动应用 Y-up→Z-up 旋转。动态障碍橙色，
  与静态蓝色区分。

## 复现

```bash
ros2 launch models/ninezzhou_moveit_config/launch/demo.launch.py   # MoveIt2
ros2 run robot_safecontrol_moveit mujoco_viewer                    # 可选：实时可视化
source install/setup.bash
python3 benchmarks/aeb_rrtstar/run_dynamic_obstacles.py            # 真实 FCL，~3 min
```

## 局限与后续

- 规划是快照式的：移动障碍用「逐位置重规划」抽象，未做时间参数化。真·动态（障碍
  扫过时预测轨迹）需时间状态空间。
- 零位→首 IK 过渡很小（J1 只动 0.21m），机械臂几乎保持竖直柱，所以连杆高度障碍的
  影响是「堵/通」的陡峭二值而非渐进绕行；若要观察渐进绕障，需用更远的目标（如
  idx2000 处 tool0 z≈1.64）。
- 突发障碍只测「能否判定/绕障」，未接入真实执行中的重规划循环（reactive replan）。
