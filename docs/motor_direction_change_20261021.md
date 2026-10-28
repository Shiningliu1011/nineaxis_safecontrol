# 电机方向配置与恢复记录

## 依据与范围

2026-10-21，用户报告 J2–J9 中只有 J3、J5 的电机正角度运动方向符合仓库模型，并授权修改及保存恢复记录。本配置按其余六个关节方向相反处理。当前库函数按节点 ID 2–9 分别控制 J2–J9；本轮每个节点的反馈与实际发送 ID 已核对。

模型零位下，从基座朝末端观察：J2、J3、J4、J6、J8 正角度向右，J7、J9 正角度向上，J5 正角度顺时针。前级关节姿态改变时，后级关节的空间轴方向随之改变。

## 配置记录

修改文件：[hardware_joint_zero.yaml](../config/hardware_joint_zero.yaml)。

| 关节 | 修改前 sign | 当前 sign |
|---|---:|---:|
| J1 | 1 | -1 |
| J2 | 1 | -1 |
| J3 | 1 | 1 |
| J4 | 1 | -1 |
| J5 | 1 | 1 |
| J6 | 1 | -1 |
| J7 | 1 | -1 |
| J8 | 1 | -1 |
| J9 | 1 | -1 |

J2–J9 的 `zero_offset_deg` 已按用户确认 URDF 零位后的静止编码器读数中位数保存；原值全部为 `0.0`。采集来源与实机执行结果见[单关节执行记录](single_joint_execution.md)。J1 的正方向、零位、5 mm/rev 导程和输出轴直连关系已由用户确认，详见[J1 运动规格](planning/nineaxis-full-project/j1-motion-spec.md)。

## 生效条件

带限位、反馈时效、到位判定和异常处理的执行入口见[单关节低速执行](single_joint_execution.md)。本页的 `DrEmpowerJointControl` 是底层方向转换接口。

[UnitConverter](../src/robot_safecontrol_moveit/unit_conversion.py) 已提供绝对位置的双向换算：

```text
电机目标（deg） = zero_offset_deg + sign × 模型目标（rad） × 180/π
模型反馈（rad） = sign × (电机反馈（deg）− zero_offset_deg) × π/180
```

项目提供 [DrEmpowerJointControl](../src/robot_safecontrol_moveit/drempower_joint_control.py)，读取完整配置并复用 `UnitConverter`。接口的角度和增量单位为模型方向的 degree，速度反馈为模型方向的 rpm。调用接口接收已连接的厂家 Python 模块 `vendor`，不自行导入厂家模块或打开串口。

- `set_angle(vendor, joint, angle_deg, speed_rpm=..., accel_rpm_s=...)`：转换绝对目标，调用厂家 `set_angle(mode=1)`。速度和加速度为正的大小值。
- `get_state(vendor, joint)`：读取厂家位置和速度，检查读取状态及 `axis_error`，转换后返回 `JointFeedback`。
- `step_angle(vendor, joint, delta_deg, speed_rpm=..., accel_rpm_s=...)`：读取当前位置，加上模型方向的增量，再调用单节点绝对位置命令。
- `position_arguments(...)`、`decode_state(...)`：纯计算接口，用于不连接设备的核对。

以下示例只计算参数，不连接设备、不发送命令。`node_map` 必须来自实际安装对应关系；这里展示当前配置中的编号顺序：

```python
from pathlib import Path
from robot_safecontrol_moveit.drempower_joint_control import DrEmpowerJointControl
from robot_safecontrol_moveit.unit_conversion import JointNodeMap

node_map = JointNodeMap.from_joint_list(tuple(f"J{i}" for i in range(1, 10)))
control = DrEmpowerJointControl(Path("config/hardware_joint_zero.yaml"), node_map)
arguments = control.position_arguments("J2", 5.0, speed_rpm=0.1, accel_rpm_s=0.1)
# arguments 中 angle 为 zero_offset_deg - 5.0，id_num 为 2。
```

实际项目调用须使用上述 `control` 接口，直接调用 `vendor.set_angle()` 仍采用电机方向。厂家库串口连接、到位监控、反馈身份与时效验证、限位和独立停车由执行程序负责。函数返回只表示厂家库调用成功，不表示运动到位。本接口仅支持 J2–J9，拒绝 J1、无效配置、无效数值及零速度。

用户确认通过厂家 `DR Actuator` 调试界面控制电机。该界面不读取仓库方向配置，界面角度使用电机自身的方向定义。项目终端入口已接入双向换算，ROS `live` 继续关闭。J2–J9 已逐个完成模型方向相对 +5° 的位置反馈验证；J2 实际方向已由用户确认。其他关节的空间方向仍需现场观察确认。未保存电机参数。

## 手动恢复

1. 在停止发送运动指令的状态下，用文件编辑器打开 `config/hardware_joint_zero.yaml`。
2. 将 J2、J4、J6、J7、J8、J9 的 `sign` 从 `-1` 改回 `1`；其他数值保持原样。
3. 配置注释同步说明全部方向和零点待验证。若以后控制程序已读取该配置，重新加载配置后核对转换结果，再决定是否恢复运行。

恢复最初的软件零位配置时，将 J2–J9 的 `zero_offset_deg` 改回 `0.0`。保留之后其他任务的修改。当前没有设备端参数需要恢复。

若需要撤销调用接口接入：停止使用 `DrEmpowerJointControl`，用文件编辑器恢复调用位置，并移除本次新增的 `src/robot_safecontrol_moveit/drempower_joint_control.py` 和 `tests/test_drempower_joint_control.py`。现有 `unit_conversion.py`、厂家库、URDF 和硬件 bridge 均未修改。

## 验证范围

### 零位与实机记录

用户确认机械臂处于 URDF 零位后，通过转换器 `87949380174B` 读取 J2–J9 的实际位置、速度及独立属性。静止位置范围小于 0.007°，瞬时速度读数存在波动。执行程序同时检查时间平均速度与独立位置变化速度，保留原始读数。当前软件零位取自 `.scratch/motor_tools/diagnostic_final_preflight.json` 的静止位置中位数，未发送设备清零或参数保存命令。

完整 +5° 执行记录与恢复文件见[单关节执行记录](single_joint_execution.md)。J7、J8 使用原绝对目标完成复验，未重复叠加增量。八个关节的最终目标误差均小于 0.017°，完成记录包含持续静止与 `traj_done` 检查。用户确认 J2 实际转动方向正确，其余空间方向尚未全部确认。

### 本地软件检查

当前五组相关测试共 152 项通过，完整命令见[单关节执行记录](single_joint_execution.md)。覆盖实际方向与零位配置、转换、CAN 帧、URDF 运动限位、±10° 请求、2.5°/s 速度边界、2.46°/s 指令参数与协议编码、按行程计算的运动时限、到位观察时限、停止锁定、绝对目标、协议参数精度及参数预览与发送编码的一致性。测试无 mock，无设备 I/O；实机证据单独保存在上述日志中。

用户追加 J4–J9 到模型绝对角度 +10° 的观察测试尚未完成。当前运动锁定，最新现场状态与停止记录见上述执行记录。
