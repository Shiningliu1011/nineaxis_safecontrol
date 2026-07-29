# robot_safecontrol

9 自由度机械臂安全控制项目，基于 MuJoCo 物理仿真。

## 目录结构

```
robot_safecontrol/
├── src/
│   └── view_arm.py              # 主程序：URDF→MJCF 转换 + MuJoCo 可视化
├── models/
│   └── ninezzhou/               # 9 轴机械臂模型
│       ├── urdf/                # URDF 定义
│       ├── meshes/              # STL 网格文件 (Link1~9 + base_link)
│       ├── config/              # 关节名称配置
│       └── launch/              # ROS launch 文件
├── data/
│   └── nurbs/                   # NURBS 轨迹规划数据
│       ├── ik_input.mat         # 逆运动学输入 (末端轨迹)
│       ├── control_points.txt   # NURBS 控制点
│       └── *.m                  # MATLAB 离线/实时插补脚本
├── output/                      # 生成文件 (已 gitignore)
│   └── ninezzhou_env.xml        # MuJoCo 环境文件
├── .gitignore
└── README.md
```

## 运行

```bash
python3 src/view_arm.py
```

需要安装: `mujoco`, `numpy`, `scipy`

## 坐标系

- URDF 使用 Y-up
- MuJoCo 使用 Z-up
- 程序自动通过 `display_frame` body 的 euler 旋转转换

## 功能

- 加载 9 轴机械臂 URDF 并在 MuJoCo viewer 中可视化
- 注入地面、灯光、障碍物环境
- 从 `.mat` 文件加载末端轨迹并可视化
- 纯运动学模式 (无重力)
