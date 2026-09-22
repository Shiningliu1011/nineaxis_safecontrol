# 日志与运行证据

## 日志入口

ROS node 使用 `self.get_logger()` 或通过 port 注入的 logger。纯控制内核不创建 ROS logger；它通过返回值、诊断结构和报告数据向 adapter 提供信息。

## 级别

- `info`：生命周期节点、配置来源、资源路径、启动完成、阶段完成、报告路径和明确的运行模式。
- `warning` / `warn`：当前输入被拒绝或丢弃、可继续的单次运行问题、频率受控的安全提醒。
- `error`：当前任务无法完成、后台报告失败、自动规划达到尝试上限或启动准入失败。

高频控制循环不能逐步打印普通状态。周期统计使用 `telemetry_period_s` 控制频率，完整数据写入跟踪报告或运行快照。

## 稳定事件与状态码

跨进程检查和测试依赖稳定标记，例如：

```python
ports.log.info("START_STATE_RECEIVED")
ports.log.info("GOAL_IK_SUCCEEDED")
ports.log.info("TRANSITION_REPLAYED")
```

自动规划使用 `AUTO_PLAN_SUCCEEDED`、`AUTO_PLAN_RETRY` 和 `AUTO_PLAN_FAILED`。感知标定使用 `CALIBRATION_WARN`、`CALIBRATION_NOTE` 和 `CALIBRATION_REFUSED`。改变这些标记前，先搜索测试、脚本和 runbook 中的消费者。

可供程序读取的结果使用稳定字段名和明确分隔符：

```text
error_code=TRANSITION_PLANNED|trajectory_points=42|planning_time=0.137
```

自然语言可以补充 `detail`，错误分类仍由状态码负责。

## 运行快照与报告

`src/robot_safecontrol_moveit/runtime_snapshot.py` 为每次生产启动创建唯一 JSON 文件，并以同目录临时文件、`fsync` 和 `os.replace` 完成原子替换。快照应包含：

- 生效参数及来源；
- override 链；
- 资源原始值和解析路径；
- raw topic 与 resolved topic；
- 软件身份和启动时间；
- 与本次运行直接相关的几何或控制标识。

JSON 写入使用 `allow_nan=False`，非有限数值必须在更早边界被拒绝。报告成功日志包含最终路径，失败日志包含错误原因。

测试或评估结论需要同时记录 commit、日期、环境、执行命令和退出码。报告中的通过状态只说明对应检查通过，不能扩展为真机准入或物理停车能力。

## 禁止模式

- 使用 `print()` 代替 ROS node logger。
- 在 100 Hz 回调中持续打印每个样本。
- 只记录“失败”，没有阶段码、资源路径或输入来源。
- 把未知反馈、未测距离或无效标定记录成正常零值。
- 日志宣称已经发送 CAN、已经获得反馈或已经开放 `live`，而当前代码没有对应能力。
- 用日志文本承担控制状态；控制状态必须保存在明确的数据对象或状态机中。

## 参考文件

- `src/robot_safecontrol_moveit/transition_executor.py`
- `src/robot_safecontrol_moveit/oscbf_controller.py`
- `src/robot_safecontrol_moveit/perception_bridge.py`
- `src/robot_safecontrol_moveit/runtime_snapshot.py`
- `src/robot_safecontrol_moveit/tracking_report_writer.py`
