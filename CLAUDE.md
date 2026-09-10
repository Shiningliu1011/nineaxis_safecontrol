# robot_safecontrol

## Tech Stack

- ROS 2 Humble（ament_python 主包 + 嵌套 ament_cmake OMPL 插件）
- Python 3：rclpy、numpy、scipy、mujoco、jax 0.6.2、qpax、pymoveit2
- C++：OMPL / MoveIt2 插件 `aeb_rrtstar_ompl`
- 控制内核：纯 JAX OSCBF（Morton & Pavone IROS 2025），位于 `portable_oscbf/work`
- 真机通信：python-can / SocketCAN，DrEmpower 协议

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

## Environment Setup（Ubuntu 22.04 / ROS Humble）

```bash
# 1) ROS 系统包（MoveIt2 2.5.9 + 控制器/参数化工具）
sudo apt-get install -y ros-humble-moveit ros-humble-moveit-configs-utils \
  ros-humble-controller-manager ros-humble-controller-manager-msgs \
  ros-humble-xacro ros-humble-joint-state-publisher

# 2) Python 依赖（用户态安装到 ~/.local；版本约束重要！）
python3 -m pip install --user "jax==0.6.2" "jaxlib==0.6.2" "cbfpy==0.0.1" \
  qpax python-fcl trimesh mujoco matplotlib
# 注意：cbfpy 必须锁 0.0.1 —— 0.0.3+ 将 relax_cbf 改名为 relax_qp，
# 会使 JAX 内核构造抛 TypeError（CBFConfig unexpected keyword 'relax_cbf'）。
# cbfpy 0.0.1 带 numpy<2 约束，会自动回退 numpy 1.26.x。

# 3) pymoveit2（非 PyPI，需 colcon 构建进本仓库 install/ 前缀）
git clone --branch 3.2.0 --depth 1 https://github.com/AndrejOrsula/pymoveit2 \
  ~/robot/pymoveit2_ws/src/pymoveit2
source /opt/ros/humble/setup.bash
cd ~/robot/pymoveit2_ws && colcon build --symlink-install \
  --install-base /home/lsn/robot/robot_safecontrol/install
```

已知未实施事项（均在 tracker，详见 `.scratch/oscbf-wayfinder/issues/`）：
`run_all_tests.sh` 的 `set -u` 与 ROS setup.bash 冲突（issue #10）；
主包 4 个必现失败：settle harness 缺方法 ×2、perf p95 预算、e2e 步数
（issue #9，pytest tests/ 需绕过脚本另行运行）；
`test_tool_axis_path_kernel_ignores_roll_only_reference_at_path_start` 已在工作区修复：
构造前固定并恢复 x64，区分起点与执行后误差并加入倾斜对照；内核套件
148 passed、34 skipped，修订已归档，详见 [实施记录](docs/planning/oscbf-reuse/handoffs/11-roll-only-tolerance.md)。

## Code Conventions

- 每个 ROS 节点一个模块，小写下划线命名，`main()` 注册为 console_scripts
  （见 setup.py `entry_points`）
- 模块级 docstring 写明职责与话题约定（如 oscbf_controller.py 的 M10 注释）
- 话题名/QoS/关节名等共享约定统一在 `ros_conventions.py` 与 `robot_spec.py`，
  节点间禁止互相 import 私有符号
- 控制内核（portable_oscbf/work）零 ROS 依赖、零裸名同级 import（统一 `from work.X`），
  节点经 `oscbf_trajectory.bootstrap_portable` 引导
- 测试 `test_*.py`；launch 集成测试用 launch_testing；对必需配置文件做启动校验
- 提交信息中英混合，多为 feat:/fix:/perf: 前缀

## Key Entry Points

- `src/robot_safecontrol_moveit/oscbf_controller.py` — OSCBF 控制器节点，发布 `/oscbf_command`
- `src/robot_safecontrol_moveit/oscbf_plant.py` — jerk 限幅执行器仿真，发布 `/mujoco_joint_states`
- `src/robot_safecontrol_moveit/transition_planning_server.py` — 过渡规划服务器（薄 ROS 壳）
- `src/robot_safecontrol_moveit/transition_executor.py` — 过渡管线相位机（纯逻辑、无 ROS，经 ports 注入副作用）
- `src/robot_safecontrol_moveit/mujoco_viewer_with_cylinder.py` — MuJoCo 仿真/查看器
- `src/robot_safecontrol_moveit/oscbf_trajectory.py` — 三端共享的轨迹变换
- `src/robot_safecontrol_moveit/hardware_bridge.py` — 真机执行端（CAN + 安全网关）
- `src/robot_safecontrol_moveit/perception_bridge.py` — 感知桥接节点
- `src/robot_safecontrol_moveit/tracking_evaluator.py` — 跟踪评价指标
- `portable_oscbf/work/jax_control_facade.py` — JAX 控制内核主机端入口
- `src/aeb_rrtstar_ompl/src/AEBRRTstar.cpp` — C++ AEB-RRT* MoveIt 插件
- `launch/mujoco_transition_final.launch.py` — 完整闭环 launch

## Data Flow & Conventions

`/mujoco_joint_states`（植物状态）→ oscbf_controller → `/oscbf_command` →
oscbf_plant（S 曲线 jerk 限幅）→ 状态发回。控制器独立于 MoveIt。

- 坐标系：URDF Y-up ↔ MuJoCo Z-up，经 `display_frame` euler 旋转转换
- 圆柱轴心拟合口径三端（轨迹/过渡/控制器）必须一致，默认最小二乘圆拟合
- 旋转误差用精确旋转向量 `-log(R_des·R_eeᵀ)`，不用一阶叉积（180° 盲区）
- 真机三种模式：sim（MuJoCo 仿真）、shadow（记录不发送）、live（CAN 发送）

## Key Documents

- `README.md` — 概览、运行方式、关节配置、真机模式
- `CONTEXT.md` — 领域词汇表
- `LESSONS_LEARNED.md` — 已踩坑教训（改参数/接口前先读）
- `OSCBF_PORTING_GUIDE.md` — OSCBF 移植架构与验收门
- `docs/real_robot_runbook.md` — 真机操作手册
- `docs/ONBOARDING.md` — 完整入门指南

## Agent skills

### Issue tracker

Issues live as GitHub issues on `Shiningliu1011/nineaxis_safecontrol`; use the `gh` CLI. See `docs/agents/issue-tracker.md`.

### Domain docs

Single-context: one `CONTEXT.md` + `docs/adr/` at the repo root. See `docs/agents/domain.md`.

## Notes

- `src/aeb_rrtstar/` 为独立 Python 规划器（无 ROS），被 C++ 插件包参考
- `models/ninezzhou*` 为机械臂 URDF/MoveIt 配置；`data/nurbs/ik_input.mat` 为 IK 输入
- `config/` 下 YAML 为节点参数；`output/` 为生成文件（gitignore）
