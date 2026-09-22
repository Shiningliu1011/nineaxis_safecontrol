# 目录与模块职责

## 主要目录

- `src/robot_safecontrol_moveit/`：ROS 2 node、MoveIt2 适配、MuJoCo plant/viewer、感知适配、配置读取与纯 Python 协调逻辑。
- `portable_oscbf/work/`：可移植控制内核，包含运动学、OSCBF/JAX 控制、QP 健康判定、碰撞几何与控制结果类型。该目录不能依赖 ROS。
- `portable_oscbf/config/`：控制内核使用的运动学、执行器、碰撞和标定配置。
- `portable_oscbf/scripts/`：从 URDF、mesh 或标定数据生成受管文件的脚本。
- `launch/`：ROS 进程拓扑和参数连接。最终入口为 `mujoco_transition_final.launch.py`，仿真演示入口由 `run_demo.sh` 启动。
- `config/`：ROS node 的生产配置和 launch 运行配置。`config/oscbf_controller.yaml` 是控制器生产配置入口。
- `tests/`：主包的纯逻辑、ROS 接口、launch 结构和运行测试。
- `portable_oscbf/tests/`：控制内核、JAX、生成文件和几何测试。
- `docs/adr/`：稳定设计决定；`docs/ONBOARDING.md` 记录入口与所有权关系。

## 依赖方向

`portable_oscbf/work` 保持 ROS 无关。ROS node 可以调用控制内核，控制内核不能导入 `rclpy`、ROS message 或 launch API。`scripts/agent_check.sh` 会检查一组纯逻辑模块在导入后没有加载 `rclpy`。

带副作用的 ROS 调用留在 node 或 adapter 中，流程状态放入可独立测试的纯逻辑对象。`TransitionExecutor` 使用 `TransitionPorts` 接收 MoveIt 服务、重放、执行和日志能力：

```python
@dataclass(frozen=True)
class TransitionPorts:
    log: Any
    get_parameter: Callable[[str], Any]
    check_moveit_services: Callable[[], tuple[bool, str]]
    wait_for_joint_state: Callable[..., Any]
```

来源：`src/robot_safecontrol_moveit/transition_executor.py`。

JAX 编译边界使用固定字段和固定形状的数据。`ControlStepRecord` 采用 `NamedTuple`，并以布尔字段说明距离是否完成测量：

```python
class ControlStepRecord(NamedTuple):
    q_next: jax.Array
    min_obs_dist: jax.Array
    min_obs_dist_measured: jax.Array
```

来源：`portable_oscbf/work/control_step_record.py`。

## 公共规则的归属

- 九轴关节名称和顺序：`src/robot_safecontrol_moveit/robot_spec.py`。
- ROS topic 与 state stream QoS：`src/robot_safecontrol_moveit/ros_conventions.py`。
- 轨迹到 `base_link` 的公共变换和圆柱表面投影：`src/robot_safecontrol_moveit/oscbf_trajectory.py`。
- 首个任务点和姿态：`src/robot_safecontrol_moveit/task_target.py`。
- 控制器生产参数模式、校验和来源记录：`src/robot_safecontrol_moveit/production_config.py`。
- 运行快照文件：`src/robot_safecontrol_moveit/runtime_snapshot.py`。
- 真机状态、位置命令和控制安全门：`src/robot_safecontrol_moveit/hardware_contract.py`。
- 运动学常量：由 `portable_oscbf/scripts/generate_kinematics_data.py` 生成到 `portable_oscbf/work/kinematics_data.py`。

## 新文件位置

- 与 ROS 无关并可复用的状态机、校验器和数值转换放在主包纯逻辑模块或 `portable_oscbf/work`，根据其是否属于控制内核选择位置。
- ROS subscription、publisher、service、timer 和 parameter 声明留在对应 node。
- 多个 node 共享的 topic、身份或坐标转换应进入现有公共入口，不能在消费者中复制常量。
- 行为测试放在对应测试目录，并按模块命名为 `test_<module>.py`。
- 长期设计决定写入 `docs/adr/`；运行说明写入现有 README 或 runbook。

## 禁止模式

- 在 `portable_oscbf/work` 中导入 ROS。
- 在 node 内重新声明 `J1` 到 `J9`、公共 topic 或轨迹变换。
- 让控制内核直接创建 publisher、读取 ROS parameter 或管理 ROS 生命周期。
- 把显示坐标转换带入控制、碰撞、IK 或规划计算。
- 为一个调用点创建与现有公共模块含义相同的新 helper。
