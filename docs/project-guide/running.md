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

依赖包含 ROS 2 Humble、MoveIt2、pymoveit2、MuJoCo，以及 JAX/CBFpy/qpax 等控制内核依赖。环境与构建说明见 [项目入门](../ONBOARDING.md)和 [控制内核说明](../modules/portable_oscbf.md)。

最终闭环默认使用 `AEBRRTstarFaithfulConfigDefault`。AEB-RRT* 由 MoveIt
PlanningScene/FCL 做状态与路径碰撞检查。
