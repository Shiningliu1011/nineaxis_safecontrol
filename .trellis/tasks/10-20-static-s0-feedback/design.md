# S0 技术设计

## 当前行为与目标

`scripts/validate_tracking_smoothness.py` 已使用独立 ROS 进程驱动
`OscbfController` 与 `OscbfPlant`，记录 500 Hz 位置、速度及加速度。
当前参数分散于生产 YAML 与脚本，反馈评价依赖工作目录中的分析程序。
本任务提供固定实验配置、可安装的运行入口、持续记录及可重放评价。

## 所有权与数据流

- 新增 `static_experiment_config.py`：验证实验配置，解析路径与资源，保存不可变场景身份。
- 新增 `static_experiment_recording.py`：保存独立反馈、命令和控制参考数据；记录时间和运行结束原因。
- 新增 `static_experiment_evaluation.py`：使用反馈 FK、连续局部投影与真实夹角独立评价。
- 新增 `static_experiment.py` 与脚本入口：管理本任务创建的 ROS 进程、运行输出、重放及显示。
- 复用 `oscbf_trajectory.py` 的路径变换、`robot_spec.py` 的关节身份和 `ros_conventions.py` 的 QoS。
- 复用 `OscbfPlant`、`SCurveDriverSimulator` 与 `OscbfController`；控制参考由实际控制记录提供。
- MuJoCo 显示通过现有 viewer 加载同一实验几何；显示数据不能影响控制状态。

## 实验与证据

新输出目录必须不存在。运行开始即保存配置、资源 hash、代码 hash、模型及路径。
反馈以独立记录持续写入，避免异常退出丢失整个运行。
仿真时间依据 plant 的实际积分次数，单调时间使用 monotonic；
消息时间与命令接收时间分别保存。
异常处理只负责记录、关闭本任务进程及释放资源，随后传播异常。

评价包含明确的测量边界、有效样本数量、未测项与实际结束原因。
路径交叉采用连续局部投影；终点保持使用反馈误差及持续时间。
配置和记录缺失不能生成通过结论。

## 修改范围

实验模块、相关脚本和配置、MuJoCo 场景注入、相关测试、安装资源及
`docs/static_obstacle_experiments.md`。现有生产默认行为通过相关回归检查。
本子任务的碰撞结果维持未测状态，后续 #174、#175 补充真实查询。
