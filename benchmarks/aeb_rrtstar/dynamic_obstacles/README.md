# 动态移动 + 突发障碍物可行性基准（真实 MoveIt2 FCL）

Date: 2026-08-01
Planner: AEB-RRT\* (`AEBRRTstarFaithfulConfigDefault`) vs RRTConnect baseline
Environment: 真实 MoveIt2 FCL（move_group + `aeb_rrtstar_ompl` 插件 + 4 固定障碍）
Raw data: `run_20260801_174605.json`（26 条记录）, `run_20260801_174605.log`

## 目的

现有测试只有固定障碍物。本基准验证 **动态移动障碍物** 和 **突然出现的障碍物**
两类场景下，零位→首 IK 内部解的过渡规划是否仍然可行。规划是快照式的（每次
规划只看到当前场景），因此移动障碍用「MOVE → 场景同步 → 重规划」的离散循环抽象。

## 场景 A — 移动障碍扫描

一个半径 0.06 m 的球体（`dyn_sweep`）沿过渡走廊横向扫过（10 个位置，
走廊中点 tool0 ≈ `[0, 0.378, 1.496]`，扫掠方向 `[0,1,0]`，跨度 0.30 m）。
t=0.5 恰好在 tool0 路径上（必堵），两端离路径 0.15 m（预计畅通）。

| sweep_t | AEB-RRT\* | RRTConnect | 说明 |
|---------|-----------|------------|------|
| 0.00 | OK | OK | 边缘畅通 |
| 0.11 | OK | OK | |
| 0.22 | OK | OK | |
| 0.33 | **FAIL** | OK | AEB-RRT\* 开始被堵 |
| 0.44 | FAIL | FAIL | 走廊中点，球体正挡路径 |
| 0.56 | FAIL | FAIL | 走廊中点 |
| 0.67 | FAIL | FAIL | |
| 0.78 | FAIL | FAIL | |
| 0.89 | OK | OK | 边缘畅通 |
| 1.00 | OK | OK | 边缘畅通 |

**结论**：两规划器都在走廊被球体占据时正确失败（不是崩溃/卡死，而是返回无解），
边缘恢复成功。所有成功路径经 `/check_state_validity` 对当前场景验证全部有效
（`all_states_valid=True`）。AEB-RRT\* 在 t=0.33 比 RRTConnect 更早失败——
在该样本下 5s 预算内未能绕障，属规划器差异而非系统性问题。

## 场景 B — 突然出现的障碍物

先在干净场景用 AEB-RRT\* 规划基线路径，然后在走廊中点 ADD 一个球体
（横向偏移扫描 0 / 0.03 / 0.06 m），验证：
1. 旧基线路径是否被新场景判定碰撞（`old_path_now_collides`）
2. 两规划器能否重规划绕障

| 偏移(m) | AEB-RRT\* | RRTConnect | 旧路径碰撞 | 新路径有效 |
|---------|-----------|------------|------------|------------|
| 0.00 | FAIL | FAIL | True | n/a（无路径）|
| 0.03 | FAIL | FAIL | True | n/a（无路径）|
| 0.06 | OK | OK | True | True |

**结论**：球体在偏移 0/0.03 m 时完全挡住走廊 → 两规划器都正确判定不可行
（无解，非崩溃）。偏移 0.06 m 时路径侧移绕开 → 两规划器都成功，且新路径
对当前场景验证有效。`old_path_now_collides=True` 全部成立，证明 ADD 确实
改变了场景并影响了基线路径的有效性。

## 关键机制（本次新增）

- **基准脚本** `run_dynamic_obstacles.py`：
  - `move_collision()`（MOVE op）/ `add_collision_sphere()`（ADD op）发布到
    `/collision_object` —— 项目此前从未用过这两个 pymoveit2 接口。
  - `wait_obstacle_pose()`：**pose 级**场景同步（ID 存在不够，MOVE 必须核对
    位置），轮询 `/get_planning_scene`，找不到时重发。
  - `check_path_validity()`：用 `/check_state_validity` 对当前场景逐 waypoint
    + 边插值点验证，区分「服务失败(None)/真无效(False)」。
  - `tool0_positions()`：`compute_fk` 沿零位→目标直线插值采样工具位姿，
    用于把移动障碍路径对准走廊。
- **MuJoCo 实时可视化**（`mujoco_viewer_with_cylinder.py`）：
  - 预分配 16 个 free-joint 障碍槽（8 球 / 4 盒 / 4 柱），订阅 `/collision_object`
    实时渲染 ADD/MOVE/REMOVE。free joint 必须在 worldbody 顶层，故用
    `_yup_to_zup_world()` 手动应用 Y-up→Z-up 旋转。
  - 动态障碍橙色，与静态蓝色区分。基准运行时可在查看器内看到扫描球步进、
    突发球在走廊中点弹出。

## 复现

```bash
# 1. 启动 MoveIt2（已在后台）
ros2 launch models/ninezzhou_moveit_config/launch/demo.launch.py

# 2. （可选）启动 MuJoCo 查看器实时可视化
ros2 run robot_safecontrol_moveit mujoco_viewer

# 3. 跑基准（真实 FCL，~3 min）
source install/setup.bash
python3 benchmarks/aeb_rrtstar/run_dynamic_obstacles.py
```

## 局限与后续

- 规划是快照式的：移动障碍用「逐位置重规划」抽象，未做时间参数化（obstacle
  扫过路径的轨迹预测规划）。若需真·动态（路径上球体持续移动），需时间状态空间。
- AEB-RRT\* 在 t=0.33 的一个样本下 5s 未绕障成功，可增大 `PLANNING_TIME_S` 或
  `planning_attempts` 复测。
- 突发障碍只测了「能否判定/绕障」，未接入真实执行中的重规划循环（reactive
  replan），那是生产管线改造，不在本次范围。
