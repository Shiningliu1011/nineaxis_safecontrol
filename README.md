# robot_safecontrol

9 自由度机械臂安全控制项目，基于 MuJoCo 物理仿真 + ROS2/MoveIt2。

## 目录结构

```
robot_safecontrol/
├── src/
│   ├── plan_transition.py              # 独立脚本：OMPL 过渡路径规划 (无 ROS)
│   ├── view_arm.py                     # 独立脚本：MuJoCo 机械臂可视化 (无 ROS)
│   └── robot_safecontrol_moveit/       # ROS2 Python 包
│       ├── plan_transition.py          #   ROS2 节点：过渡路径规划
│       ├── continuous_ik.py            #   连续 IK 求解
│       ├── motion_planning.py          #   MoveIt 运动规划
│       ├── mujoco_viewer.py            #   MuJoCo 可视化节点
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
│   ├── plan_transition.yaml            # ROS2 节点参数
│   └── obstacles.yaml                  # 障碍物定义
├── launch/
│   ├── plan_transition.launch.py       # ROS2 launch: 过渡路径规划
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

### 独立脚本（无需 ROS）

```bash
# MuJoCo 可视化机械臂 + 末端轨迹
python3 src/view_arm.py

# OMPL 过渡路径规划 (零位→轨迹起始点)
python3 src/plan_transition.py
```

依赖: `mujoco`, `numpy`, `scipy`, `ompl`

### ROS2 节点

```bash
# 编译 Python 闭环包和 C++ AEB-RRT* MoveIt 插件。
# 注意：这里不能只运行普通的 `colcon build`，因为插件位于嵌套包。
bash build_aeb_moveit.sh
source install/setup.bash

# 启动完整闭环（M 进入手动模式，调整关节后按 T）。
bash run_demo.sh
```

依赖: ROS2 Humble, MoveIt2, pymoveit2

最终闭环默认使用 `AEBRRTstarFaithfulConfigDefault`。AEB-RRT* 由 MoveIt
PlanningScene/FCL 做状态与路径碰撞检查，因此 `/collision_object` 上的动态
障碍物会同时影响 IK、状态合法性与规划，而不仅是 MuJoCo 中的可视化对象。

每次修改 AEB 的 C++ 代码后，都要重新执行 `bash build_aeb_moveit.sh`，再重启
整个 launch（直接运行 `bash run_demo.sh` 即可）。运行中的 `move_group` 不会自动
加载新编译的 `.so` 插件。

完成构建和最终 launch 后，可在另一终端用下面的自检命令验证动态障碍物链路：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_safecontrol_moveit dynamic_obstacle_probe
```

它会依次发布动态障碍物的 ADD、MOVE、REMOVE，查询 `/get_planning_scene`，并
确认 `/check_state_validity` 从有效变为无效、再恢复有效；无论成功或失败都会
清理自己的测试物体。完整无 GUI 回归测试仍可运行
`launch_test tests/test_aeb_dynamic_obstacle_runtime.py`。

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
