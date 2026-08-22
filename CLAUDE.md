# robot_safecontrol

9-DOF 冗余机械臂（J1 棱柱 + J2-J9 旋转）安全控制：MuJoCo 仿真 + ROS 2 Humble /
MoveIt 2。随机位姿 → AEB-RRT* 无碰撞过渡 → OSCBF 安全跟踪蝴蝶轨迹，一键闭环
（`bash run_demo.sh`）。

## Tech Stack

- ROS 2 Humble（ament_python 主包 + 嵌套 ament_cmake OMPL 插件）
- Python 3：rclpy、numpy、scipy、mujoco、jax、qpax、pymoveit2
- C++：OMPL / MoveIt2 插件 `aeb_rrtstar_ompl`
- 控制内核：纯 JAX OSCBF（Morton & Pavone IROS 2025），位于 `portable_oscbf/work`

## Build & Run

```bash
bash build_aeb_moveit.sh   # 必须用此脚本：普通 colcon build 找不到嵌套 C++ 包
source install/setup.bash
bash run_demo.sh           # 全自动演示，无需键盘
bash run_all_tests.sh      # 全量测试入口（主包 + 控制内核）
pytest                     # 主包测试（setup.cfg: testpaths = tests）
pytest portable_oscbf/tests  # 控制核心测试
```

改 AEB C++ 代码后必须重新构建并重启整个 launch（move_group 不会热加载 `.so`）。

## Code Conventions

- 每个 ROS 节点一个模块，小写下划线命名，`main()` 注册为 console_scripts
  （见 setup.py `entry_points`）
- 模块级 docstring 写明职责与话题约定（如 oscbf_controller.py 的 M10 注释）
- 测试 `test_*.py`；launch 集成测试用 launch_testing；对必需配置文件做启动校验
- 提交信息中英混合，多为 feat:/fix: 前缀

## Key Entry Points

- `src/robot_safecontrol_moveit/oscbf_controller.py` — OSCBF 控制器节点，发布 `/oscbf_command`
- `src/robot_safecontrol_moveit/oscbf_plant.py` — jerk 限幅执行器仿真，发布 `/mujoco_joint_states`
- `src/robot_safecontrol_moveit/transition_planning_server.py` — 过渡规划服务器（薄 ROS 壳）
- `src/robot_safecontrol_moveit/transition_executor.py` — 过渡管线相位机（纯逻辑、无 ROS，经 ports 注入副作用）
- `src/robot_safecontrol_moveit/mujoco_viewer_with_cylinder.py` — MuJoCo 仿真/查看器
- `src/robot_safecontrol_moveit/oscbf_trajectory.py` — 三端共享的轨迹变换
- `portable_oscbf/work/jax_control_facade.py` — JAX 控制内核主机端入口
- `src/aeb_rrtstar_ompl/src/AEBRRTstar.cpp` — C++ AEB-RRT* MoveIt 插件
- `launch/mujoco_transition_final.launch.py` — 完整闭环 launch

## Data Flow & Conventions

`/mujoco_joint_states`（植物状态）→ oscbf_controller → `/oscbf_command` →
oscbf_plant（S 曲线 jerk 限幅）→ 状态发回。控制器独立于 MoveIt。

- 话题名/QoS/关节名等共享约定统一在 `src/robot_safecontrol_moveit/ros_conventions.py` 与 `robot_spec.py`，节点间禁止互相 import 私有符号
- 坐标系：URDF Y-up ↔ MuJoCo Z-up，经 `display_frame` euler 旋转转换
- 圆柱轴心拟合口径三端（轨迹/过渡/控制器）必须一致，默认最小二乘圆拟合
- 旋转误差用精确旋转向量 `-log(R_des·R_eeᵀ)`，不用一阶叉积（180° 盲区）
- 内核（portable_oscbf/work）零 ROS 依赖、零裸名同级 import（统一 `from work.X`），节点经 `oscbf_trajectory.bootstrap_portable` 引导

## Key Documents

- `README.md` — 概览、运行方式、关节配置
- `LESSONS_LEARNED.md` — 已踩坑教训（改参数/接口前先读）
- `OSCBF_PORTING_GUIDE.md` — OSCBF 移植架构与验收门
- `OSCBF_EXECUTION_PLAN.md` — M6-M12 执行计划
- `docs/ONBOARDING.md` — 完整入门指南

## Notes

- `src/aeb_rrtstar/` 为独立 Python 规划器（无 ROS），被 C++ 插件包参考
- `models/ninezzhou*` 为机械臂 URDF/MoveIt 配置；`data/nurbs/ik_input.mat` 为 IK 输入
- `config/` 下 YAML 为节点参数；`output/` 为生成文件（gitignore）
