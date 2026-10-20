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
```

MuJoCo 视图采用蓝色渐变天空和蓝灰色棋盘地面，显示机械臂、圆柱曲面及目标轨迹。

`config/oscbf_controller.yaml` 是 OSCBF 控制器的必需生产配置。launch 将其
作为独立的 `production_config_yaml` 来源加载；文件缺失、字段不全、值非法、
资源不可读或启动快照无法落盘时，控制器会在建立命令 publisher 前退出。直接
启动节点使用同一契约，例如：

```bash
ros2 run robot_safecontrol_moveit oscbf_controller --ros-args \
  -p production_config_yaml:=$(ros2 pkg prefix robot_safecontrol_moveit)/share/robot_safecontrol_moveit/config/oscbf_controller.yaml
```

显式 `-p` 覆盖优先于已验证的 YAML 基础值；所有生产参数在启动后不可修改。
每次成功启动的最终值、来源链、资源路径、配置/软件哈希及拟合几何会原子写入
性能报告同目录下的 `runtime_snapshots/`。

跟踪评价输出逐项结论、真实工具轴夹角、弧长覆盖与证据来源，并保存 Markdown、JSON 和样本记录。离线子区间须在采样前声明；当前节点报告的边界为控制内核模型输出。指标定义、阈值来源及使用示例见[跟踪评价说明](../tracking_evaluation.md)。

控制器采用 50 Hz 调度与 20 ms 路径推进步长，QP 积分步长为 10 ms。
仿真执行器以 500 Hz 推进 Ruckig 位置轨迹，整个跟踪与最终保持过程持续限制
速度、加速度和 jerk。速度和加速度上限来自 `actuator_modules.yaml`；
`config/oscbf_plant.yaml` 的 `jerk_time` 决定 jerk 上限（加速度上限除以该时间）。
`minimum_duration_s` 为每次运动规划规定最短持续时间，当前为 80 ms。
目标更新保留执行器当前速度与加速度。`JointState.velocity` 发布仿真执行器速度，
绘图时应优先使用该记录；位置差分得到的速度与加速度需要标注为估计值。
这些运动学限制属于仿真参数，实机准入仍按[硬件模式](hardware-modes.md)执行。

完整路径的平顺性验证使用两个独立 ROS 进程，记录位置、速度、加速度、命令、
仿真时间和接收时间；输出目录必须尚不存在：

```bash
python3 scripts/validate_tracking_smoothness.py --output output/tracking-smoothness
```

该验证从路径起点开始，检查跟踪与最终保持，不执行 MoveIt 过渡规划。
周期试验、指标定义及图片说明见[跟踪平顺性](../tracking_smoothness.md)。

依赖包含 ROS 2 Humble、MoveIt2、pymoveit2、MuJoCo，以及 JAX/CBFpy/qpax 等控制内核依赖。环境与构建说明见 [项目入门](../ONBOARDING.md)和 [控制内核说明](../modules/portable_oscbf.md)。

最终闭环默认使用 `AEBRRTstarFaithfulConfigDefault`。AEB-RRT* 由 MoveIt
PlanningScene/FCL 做状态与路径碰撞检查。
