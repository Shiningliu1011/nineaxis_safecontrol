# robot_safecontrol

9 自由度机械臂安全控制项目，基于 MuJoCo 物理仿真 + ROS2/MoveIt2。

## 目录结构

```
robot_safecontrol/
├── src/
│   └── robot_safecontrol_moveit/       # ROS2 Python 包
│       ├── transition_planning_server.py #   持久化规划服务器（薄 ROS 壳）
│       ├── transition_executor.py      #   过渡管线相位机（纯逻辑，无 ROS）
│       ├── continuous_ik.py            #   连续 IK 求解
│       ├── motion_planning.py          #   MoveIt 运动规划
│       ├── cylinder_geometry.py        #   圆柱拟合/表面法向（单一实现）
│       ├── robot_spec.py               #   共享机器人常量（关节名等）
│       ├── mujoco_viewer_with_cylinder.py  # MuJoCo 可视化节点
│       └── trajectory_execution.py     #   轨迹执行
├── models/
│   ├── ninezzhou/                      # 9 轴机械臂 URDF 包
│   │   ├── urdf/                       #   URDF 定义
│   │   ├── meshes/                     #   STL 网格 (Link1~9 + base_link)
│   │   └── config/                     #   关节名称
│   └── ninezzhou_moveit_config/        # MoveIt2 配置包
│       ├── config/                     #   SRDF、控制器、运动学配置
│       └── launch/                     #   demo.launch.py
├── config/
│   └── mujoco_transition_runtime.yaml  # 过渡服务器运行时参数
├── launch/
│   ├── mujoco_transition_final.launch.py  # 完整闭环 launch
│   └── mujoco_viewer.launch.py         # ROS2 launch: MuJoCo 可视化
├── data/
│   └── nurbs/                          # NURBS 轨迹数据
│       ├── ik_input.mat                #   逆运动学输入 (末端轨迹)
│       └── *.m                         #   MATLAB 脚本
├── output/                             # 生成文件 (已 gitignore)
│   ├── ninezzhou_env.xml               #   MuJoCo 环境文件
│   └── transition_path.npy             #   过渡路径
├── package.xml                         # ROS2 包清单
├── setup.py / setup.cfg                # Python 包配置
├── .gitignore
└── README.md
```

## 运行

### ROS2 节点

```bash
# 编译 Python 闭环包和 C++ AEB-RRT* MoveIt 插件。
# 注意：这里不能只运行普通的 `colcon build`，因为插件位于嵌套包。
bash build_aeb_moveit.sh
source install/setup.bash

# 启动全自动闭环：机械臂每次从随机工作位姿出发，自动规划无碰撞过渡到
# 轨迹起点，回放结束后 OSCBF 控制器接管 /oscbf_command 并跟踪蝴蝶轨迹到
# 终点。全程无需键盘。
bash run_demo.sh

# 全量测试（主包 + portable 内核）
bash run_all_tests.sh
```

依赖: ROS2 Humble, MoveIt2, pymoveit2

最终闭环默认使用 `AEBRRTstarFaithfulConfigDefault`。AEB-RRT* 由 MoveIt
PlanningScene/FCL 做状态与路径碰撞检查。

### 真机运行（shadow/live 模式）

```bash
# shadow 模式：记录命令与状态，不发送 CAN 帧
ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
    hardware_mode:=shadow start_oscbf_plant:=false

# live 模式：真机发送 CAN 帧
ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
    hardware_mode:=live start_oscbf_plant:=false

# 带感知的真机模式
ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
    hardware_mode:=live start_oscbf_plant:=false start_perception:=true
```

零位标定：`python3 scripts/calibrate_zero.py --interface can0`（详见 `docs/real_robot_runbook.md`）。

话题约定：OSCBF 控制器订阅植物状态 `/mujoco_joint_states`，把安全命令发布到
`/oscbf_command`；`oscbf_plant` 节点把命令经 jerk 限幅积分后作为植物状态发回
`/mujoco_joint_states`。查看器、过渡服务器、控制器共用同一套校准轨迹变换
（`oscbf_trajectory.py`），保证 tool0 与显示的蝴蝶曲线重合。

每次修改 AEB 的 C++ 代码后，都要重新执行 `bash build_aeb_moveit.sh`，再重启
整个 launch（直接运行 `bash run_demo.sh` 即可）。运行中的 `move_group` 不会自动
加载新编译的 `.so` 插件。

## 坐标系

- URDF 使用 Y-up
- MuJoCo 使用 Z-up
- 程序通过 `display_frame` body 的 euler 旋转自动转换

## 关节配置

| 关节 | 类型 | 范围 | 说明 |
|------|------|------|------|
| J1 | prismatic | [0, 0.585] m | 升降 |
| J2-J4 | revolute | [-π/2, π/2] | 肩/肘 |
| J5 | revolute | [-π, π] | 腕旋转 |
| J6-J9 | revolute | [-1.48, 1.48] | 末端 |
