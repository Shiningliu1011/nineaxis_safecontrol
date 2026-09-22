# 公共入口与代码复用

## 编码前检查

1. 使用 `rg` 搜索已有 identifier、topic、参数名、frame、错误码和相近转换。
2. 确认数据的所有者和公共入口。
3. 查看现有消费者与测试，理解输入、输出、单位、dtype 和失败行为。
4. 扩展现有模块，或在明确的新职责边界创建模块。
5. 添加能够发现消费者分叉的测试。

## 当前公共入口

- `robot_spec.py`：九轴身份和顺序。
- `ros_conventions.py`：公共 topic 和 state stream QoS。
- `oscbf_trajectory.py`：轨迹变换、表面投影和采样顺序。
- `task_target.py`：首个任务点和姿态。
- `production_config.py`：控制器参数模式、验证、来源和资源解析。
- `hardware_contract.py`：硬件状态、位置命令、watchdog 和控制安全门。
- `tracking_contract.py`：跟踪阶段与数据规则。
- `portable_oscbf/work/kinematics_data.py`：生成的运动学常量。
- `portable_oscbf/work/robot_geometry.py`：碰撞模块共享的机器人几何。
- `portable_oscbf/work/control_step_record.py`：JIT 控制步骤输出字段。

## 应当复用的模式

多个消费者需要相同轨迹时，调用 `load_calibrated_path()` 或 `load_calibrated_path_with_times()`。不要分别执行旋转、缩放、平移、圆柱拟合和采样。

ROS node 需要状态流 QoS 时，调用：

```python
from .ros_conventions import state_stream_qos

self.create_subscription(JointState, topic, callback, state_stream_qos())
```

配置字段进入 controller 前，经过 `load_production_profile()` 与 `build_effective_configuration()`。消费者读取已经验证的最终值，不能自行解释同一 YAML 字段。

生成的运动学和 OBB 数据由脚本维护。需要改变来源数据或生成规则时，修改来源或 generator，重新生成并运行 `--check`。

## 新建公共模块的条件

满足以下任一情况时，可以增加公共模块：

- 同一领域规则已有多个真实消费者；
- 规则需要统一校验、错误码或版本标识；
- 数据跨越 JAX、ROS 或文件边界，需要一个明确的转换所有者；
- 当前模块职责已经清楚表明新逻辑属于独立领域。

只有一个简单调用点时，逻辑通常留在拥有该行为的模块。公共模块的名称应描述领域含义，不能使用含糊的 `utils.py` 或 `helpers.py`。

## 完成检查

- 搜索过重复常量和转换。
- 新代码调用已有公共入口。
- 所有消费者保持一致的名称、顺序、frame、单位和 dtype。
- 测试覆盖至少两个消费者或一个生成来源与其结果。
- 文档引用所有者文件，避免复制容易变化的数值。
