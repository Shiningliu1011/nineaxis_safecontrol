# ROS 接口、坐标与时间

## 关节身份与顺序

九轴顺序固定为：

```python
DEFAULT_JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9")
```

来源：`src/robot_safecontrol_moveit/robot_spec.py`。

生产配置中的 `joint_names` 必须与该顺序完全一致。外部 `JointState` 需要按名称映射，不能假定消息数组已经采用控制内核顺序。新增消费者时，应复用公共常量并为缺失关节、重复关节和顺序变化补充测试。

## Topic 与 QoS

公共 topic 由 `src/robot_safecontrol_moveit/ros_conventions.py` 提供：

```python
JOINT_STATE_TOPIC = "/mujoco_joint_states"
OSCBF_COMMAND_TOPIC = "/oscbf_command"
PERCEPTION_TRACKS_TOPIC = "/perception/tracks"
```

高频 state stream 使用 `state_stream_qos()`：`KEEP_LAST`、depth 20、`BEST_EFFORT`、`VOLATILE`。publisher 与 subscriber 必须采用兼容 QoS。改变 depth 或 reliability 前，要核对所有生产者、消费者和长时间延迟测试。

控制器应同时保存配置中的原始 topic 和 ROS 解析后的 topic。创建实体后必须检查最终 state topic 与 command topic 不同；链式 remap 只按 ROS 的实际解析结果处理一次。对应证据位于 `tests/test_oscbf_production_config.py`。

## 所有权与回调

- MuJoCo plant 负责仿真状态发布，并消费安全命令。
- OSCBF controller 订阅状态、计算安全命令并发布到 command stream。
- Viewer 只负责显示和运行模式切换，不能成为控制状态真源。
- Transition server 负责 MoveIt 服务调用、过渡轨迹和向 OSCBF 的交接。
- Hardware bridge 在当前 containment 中不发布真实硬件状态。

新增 subscription、publisher、service 或 timer 时，要说明其 node、callback group、输入真源、输出消费者和停止时的行为。`CODING_STANDARDS.md` 要求在代码审查时逐项核对这些所有权信息。

## 坐标语义

`base_link` 是固定的规范世界坐标系，Y 轴向上，J1 沿 Z 轴运动。控制、碰撞、IK、规划、感知输出和轨迹都使用该语义。Viewer 可以在显示边界转换到 Z 轴向上的画面坐标，转换不能传播回控制数据。

轨迹消费者统一调用 `src/robot_safecontrol_moveit/oscbf_trajectory.py`：

```python
transform = trajectory_to_base_transform(mat_path)
calibrated = apply_trajectory_transform(raw, transform)
positions, _, _ = snap_path_to_cylindrical_surface(
    calibrated, cylinder_axis_direction
)
```

投影必须在选点之前完成，使 controller、viewer 和 transition target 使用相同圆柱与相同路径。对应回归位于 `tests/test_trajectory_transform_unity.py`。

## 时间与 freshness

- ROS message 时间戳表示产生该测量的时刻，接收时刻不能替代测量时刻。
- freshness 使用同一时间基准比较。新增 watchdog 前，要明确时间源、允许时长和恢复条件。
- 没有真实反馈时，不能用命令、齐零数组或新时间戳生成看似健康的状态。
- transition 读取起始状态时必须检查等待时长和最大消息年龄；启动等待不能消耗规划重试次数。
- 控制循环中的路径时间、ROS 时间和 monotonic watchdog 时间要在边界明确转换，不能混为一个数值。

## 验证入口

- `tests/test_oscbf_production_config.py`：topic 校验、remap 后名称和运行快照。
- `tests/test_launch_structure.py`：launch node 拓扑、remap 和默认模式。
- `tests/test_final_launch_runtime.py`：最终 launch 的运行行为。
- `tests/test_trajectory_transform_unity.py`：controller、viewer 和 transition 的轨迹一致性。
- `tests/test_transition_pipeline_flow.py`：关节状态读取、阶段顺序与 freshness。
