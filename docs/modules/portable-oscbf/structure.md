## 目录结构

```
portable_oscbf/
├── work/                          # 核心计算模块 (Python package)
│   ├── __init__.py
│   │
│   │  ── JAX 编译内核层 (热路径) ──
│   ├── jax_kernel_factory.py      # OSC+CBF+QP+积分 编译入口
│   ├── control_step_record.py     # 一步输出的具名结构 (含"是否量测"状态位)
│   ├── cross_step_state.py        # 跨步记忆 (上一步命令/CBF 遥测, 显式结构)
│   ├── jax_barrier_terms.py       # 固定 shape 障碍物几何/CBF RHS
│   ├── jax_path_following.py      # JAX 弧长路径跟踪状态机
│   ├── jax_posture_reference.py   # JAX 姿态参考插值
│   ├── jax_control_facade.py      # 主机端 facade, 输入归一化, JIT warmup
│   │
│   │  ── 运动学与碰撞几何 ──
│   ├── nineaxis_manipulator_jax.py # 9-DOF POE FK/Jacobian (JAX)
│   ├── nineaxis_kinematics.py     # POE FK/Jacobian/IK (NumPy+SciPy)
│   ├── obb_collision_model.py      # OBB 包络与 32 个环境采样球 (UPAKI)
│   ├── dpax_collision.py           # DCOL OBB 距离内核 (可微碰撞)
│   ├── fcl_collision.py           # FCL 基元自碰撞检测
│   ├── fcl_collision_mesh.py      # FCL BVHModel 网格碰撞
│   ├── point_cloud_obstacles.py   # FCL 点云环境碰撞
│   ├── point_cloud_obstacles_dynamic.py # 动态障碍物类型
│   ├── dynamic_obstacles.py       # DynamicObstacleManager
│   ├── dynamic_obstacle_placement.py    # FCL 辅助障碍物放置
│   ├── controller_step_cache.py   # 每步 FK 缓存
│   │
│   │  ── QP 求解器层 ──
│   ├── qp_solver_health.py        # QP 健康检查
│   ├── oscbf_qp_solver.py         # OSQP QP 求解器 (legacy 排除项)
│   │
│   │  ── OSCBF 配置层 (cbfpy 框架) ──
│   ├── oscbf_velocity_config.py   # 速度级 OSCBF 配置
│   ├── oscbf_collision_config.py  # 碰撞约束配置
│   ├── tool_axis_task.py          # 5D/6D 任务误差/Jacobian
│   ├── task_mode_contract.py      # 任务模式常量/验证
│   ├── safety_snapshot.py         # 固定 shape 距离场快照
│   ├── joint_limit_contract.py    # 关节限位 CBF 边距
│   ├── manipulability_metric.py   # 可操作度指标
│   ├── nullspace_policy.py        # 零空间策略 (关节中点/可操作度)
│   ├── cbf_types.py               # CbfConstraint 数据类
│   │
│   │  ── 感知 ──
│   ├── perception_config.py       # 感知配置加载 (YAML → 冻结 dataclass)
│   ├── static_occupancy.py        # 静态占有率跟踪 (点云持久/瞬态分离)
│   ├── dynamic_clustering.py      # 世界系点云聚类 → 8 槽动态 track
│   │
│   │  ── 路径/轨迹 ──
│   ├── ik_data_loader.py          # .mat 轨迹加载, Frenet-Serret 帧
│   ├── path_following.py          # 弧长路径几何/状态机 (NumPy)
│   ├── path_posture_reference.py  # 姿态参考 (NumPy)
│   ├── driver_simulator.py        # S 曲线驱动模拟器
│   │
│   │  ── 工具 ──
│   ├── actuator_limits.py         # YAML 执行器限位 (仿真/硬件边界)
│   ├── safe_posture.py            # 离线安全姿态验证
│   ├── joint_trajectory_logging.py # 关节时序记录
│   └── joint_velocity_limits.py   # 速度/加速度边界
│
├── config/                        # 机器人配置 (YAML)
│   ├── nineaxis.yaml              # 机器人参数
│   ├── actuator_modules.yaml      # 执行器限位
│   ├── controller_params.yaml     # 遗留 torque 配置（生产 ROS 不消费）
│   ├── fcl_params.yaml            # FCL 碰撞参数
│   ├── obb_model.yaml             # OBB 包络数据
│   ├── dcol_alpha.yaml            # DCOL alpha 校准
│   ├── obstacle_params.yaml       # 障碍物参数
│   ├── ompl_params.yaml           # OMPL 规划参数
│   └── robot_params.yaml          # 机器人参数
│
├── data/                          # 轨迹数据 (.mat)
│   ├── ik_input.mat               # 主轨迹数据
│   ├── workspace_ik_input.mat     # 工作空间轨迹
│   ├── nurbs_blocks.mat           # NURBS 块数据
│   ├── workspace_nurbs_blocks.mat # 工作空间 NURBS 块
│   ├── realtime_interpolation_results.mat
│   └── workspace_realtime_results.mat
│
├── scripts/                       # 校准/维护脚本
│   ├── generate_kinematics_data.py # 从 models/ninezzhou 生成运动学常量
│   ├── generate_obb_calibration.py
│   ├── calibrate_dcol_alpha.py
│   └── check_dependencies.py
│
├── tests/                         # 单元测试 (36 个)
│   ├── test_jax_tracking_step.py
│   ├── test_jax_path_following.py
│   ├── test_jax_qp_health.py
│   ├── test_cbfpy_migration.py
│   ├── test_safety_snapshot.py
│   ├── test_modular_jit_equivalence.py
│   ├── test_perception_pipeline.py
│   └── ...
│
```
