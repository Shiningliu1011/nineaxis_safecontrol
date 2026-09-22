# 错误处理

## 基本规则

输入、配置和安全前置条件应在最接近来源的位置完成验证。无效配置、资源缺失、非有限数值、形状错误和不支持的模式应立即中止当前操作，并保留字段名、资源路径或阶段码。

所需依赖使用直接 import。新增代码不得用 `try-except` 隐藏依赖缺失。仅在已有的安装路径解析边界处理明确的可选环境，例如 `ament_index_python` 在源码检出目录中不可用的场景。

## 错误类型

- `ValueError`：字段类型、数值范围、数组形状、mode、topic 或组合规则无效。
- `FileNotFoundError`：配置、轨迹、控制内核目录或其他必需资源不存在。
- `RuntimeError`：当前运行状态无法继续，例如 `live` 尚未开放、后台报告仍在写入或控制资源创建失败。
- 领域错误：MoveIt 过渡流程使用 `IKError`、`IKServiceUnavailable`、`PlanningError`、`StateValidityError` 和 `ExecutionError` 区分阶段。
- 标定和跟踪模块使用各自的命名错误类型与稳定状态码，调用者不得把不同原因合并为普通失败。

数据对象在构造时校验自身不变量：

```python
def __post_init__(self) -> None:
    object.__setattr__(self, "q", np.asarray(self.q, dtype=float).reshape(-1))
    object.__setattr__(self, "qdot", np.asarray(self.qdot, dtype=float).reshape(-1))
    if self.q.shape != self.qdot.shape:
        raise ValueError("q 与 qdot 形状必须一致")
```

来源：`src/robot_safecontrol_moveit/hardware_contract.py`。

## 边界转换

纯逻辑层可以抛出明确异常。面向 ROS service 的执行器在阶段边界捕获已知异常，并转换为可稳定解析的结果：

```python
return _format_result(
    "IK_SERVICE_UNAVAILABLE", 0, monotonic() - plan_start, f"detail={error}"
)
```

`TransitionExecutor` 的结果以 `error_code=...|trajectory_points=...|planning_time=...` 开头。错误码与真实阶段保持一致；没有 MoveIt 响应时不能报告为 IK 解失败。

node 构造失败时应释放已创建资源后重新抛出异常。`main()` 负责销毁 node 并调用 `rclpy.shutdown()`。资源清理不能掩盖原始错误。

## 可继续处理的运行输入

持续输入流中的单帧解码错误可以记录 warning 并丢弃该帧，前提是：

- 下一帧独立有效；
- 失败不会创建伪造状态；
- 安全状态保持关闭或不发送；
- 日志包含来源与原因，并有频率限制。

感知点云解码和发布边界采用该模式，见 `src/robot_safecontrol_moveit/perception_bridge.py`。配置、标定准入、反馈 freshness 和控制安全门不能采用自动 fallback。

## 异常捕获规则

- 捕获能够在当前边界处理的明确异常类型。
- 保留原始原因时使用 `raise ... from exc`。
- 广泛捕获仅用于 node 构造清理、进程顶层清理或将第三方边界错误转换为带上下文的领域错误。
- 不能返回空数组、齐零状态或成功码掩盖安全输入缺失。
- 不能降低阈值、删除断言或扩大成功码集合来通过测试。

## 验证入口

- `tests/test_oscbf_production_config.py`：配置和资源错误。
- `tests/test_transition_executor.py`：MoveIt 各阶段错误码与执行顺序。
- `tests/test_hardware_contract.py`：状态、命令、watchdog 和锁存停止原因。
- `tests/test_perception_bridge_points.py` 与 `tests/test_perception_bridge_startup.py`：输入帧和标定拒绝。
