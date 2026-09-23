## 目录结构

```
robot_safecontrol/
├── src/
│   ├── aeb_rrtstar_ompl/               # C++ AEB-RRT* MoveIt 插件
│   └── robot_safecontrol_moveit/       # ROS2 Python 包
│       ├── transition_planning_server.py #   持久化规划服务器（薄 ROS 壳）
│       ├── transition_executor.py      #   过渡管线相位机（纯逻辑，无 ROS）
│       ├── continuous_ik.py            #   连续 IK 求解
│       ├── motion_planning.py          #   MoveIt 运动规划
│       ├── cylinder_geometry.py        #   圆柱拟合/表面法向（单一实现）
│       ├── robot_spec.py               #   共享机器人常量（关节名等）
│       ├── mujoco_viewer_with_cylinder.py  # MuJoCo 可视化节点
│       └── trajectory_execution.py     #   轨迹执行
├── portable_oscbf/                    # JAX 控制内核与独立测试
├── xy-mid-360-s/                      # 独立 Python 雷达驱动
├── docs/                              # 架构、运行手册、研究与实施交接
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
│       └── ik_input.mat                #   逆运动学输入 (末端轨迹)
├── output/                             # 生成文件 (已 gitignore)
│   ├── ninezzhou_env.xml               #   MuJoCo 环境文件
│   └── transition_path.npy             #   过渡路径
├── package.xml                         # ROS2 包清单
├── setup.py / setup.cfg                # Python 包配置
├── .gitignore
└── README.md
```
