# 真机落地执行文档（H0–H7）

> 日期：2026-08-22
> 基线：仿真闭环（M0–M12 主干已完成，见 `OSCBF_EXECUTION_PLAN.md`）
> 模板与验收规则沿用 OSCBF 执行文档：阶段严格串行，每个阶段全部 AC 通过才算完成；未通过记录偏差，不跳过。

## 1. 目的与范围

把 `robot_safecontrol` 从 MuJoCo 仿真闭环扩展到真实九轴机械臂：真实编码器反馈 → 安全
网关 → DrEmpower CAN 命令 → 低速轨迹跟踪 → 实机软障碍物动态避障。

范围（2026-08-22 访谈共识，14 项决策基线，与用户逐项确认）：

| # | 决策 | 结论 |
|---|------|------|
| 1 | 落地定义 | 真机部署，仿真闭环为基线 |
| 2 | 真机现状 | 已全部装配，控制通道未通 |
| 3 | 主仓 | `robot_safecontrol`；参考项目 `robot_oscbf/newaxis` 仅作移植来源与设计参照 |
| 4 | 里程碑 | 覆盖参考项目 7 阶段，含实机动态避障（软障碍物） |
| 5 | 电机命令模式 | DrEmpower 位置模式 `0x19`（目标角 deg + 速度 + 滤波/加速度字段，电机内置插补） |
| 6 | CAN 通道 | 先 `vcan0` 虚拟 CAN 验证，后购 CANable 2.0（candleLight/candleusb 固件，内核 `gs_usb` 驱动） |
| 7 | 运行载体 | 本 VMware VM（USB 透传 CANable 与 Orbbec），实测 p99 进阶段门 |
| 8 | 失效模式 | 超时/故障/限幅 → 零速度帧（保持位置）+ 锁存 stop_reason + 人工确认恢复；物理急停为独立链路 |
| 9 | J1 传动 | 待实测；未标定前 J1 禁止运动（fail-closed，网关 `require_hardware_executable=True` 语义） |
| 10 | 零位标定 | 机械零位标记法：手动摆零 → `set_zero`（0x08 order 0x05）→ ±5° 正方向验证，偏移存 `hardware_joint_zero.yaml` |
| 11 | 感知传感器 | Orbbec Gemini 335L（PID 0x0804，SN CP28563000GD），官方 `orbbec_camera` ROS2 驱动 |
| 12 | 感知管线 | 固定安装 + 手动测量外参 → 静态 TF（替换占位 `camera_to_world_static`/`use_tf:true`）→ 聚类 → 解析障碍物（球/圆柱）→ 控制器 `obs_*`；`sdf_*` 不启用；不把稠密点云直接塞进 CBF |
| 13 | 控制频率 | 保持 100 Hz（与仿真一致）；过渡 replay 与 OSCBF 命令共用 `/oscbf_command` 位置流，watchdog 全程生效；launch 增 `hardware_mode:=sim|shadow|live` |
| 14 | 工程化 | 本地自洽档：无 CI/无远程仓；清死配置、更新过时文档、pytest 全量入口、验收证据归档 |

## 2. 前置清单（现场事项，用户侧）

这些事项阻塞对应阶段，不阻塞 H0/H1 开发：

| # | 事项 | 阻塞 | 说明 |
|---|------|------|------|
| P1 | 获取 DrEmpower 厂商包（`DrEmpower_socketcan.py`、`interface_enums.py`、协议 PDF）| H1 后半 | 本机磁盘上没有；参考项目文档有协议摘要（`real_robot_hardware_source_review.md`），但帧字段单位（0x19 速度字段 rpm 还是 deg/s）需以厂商库为准确认 |
| P2 | 实测 J1 丝杆导程/二级传动比/效率/行程/正方向/回零方式 | H2(J1) / H4 | 未标定前 J1 保持 fail-closed；测量公式见参考项目 `docs/实机计划.md` §3 |
| P3 | 采购 CANable 2.0 | H2 | H1 全部可在 vcan0 完成 |
| P4 | 确认物理急停按钮/断电链路与制动器现状 | H2 / H4 | 阶段门要求：任何运动命令前完成急停触发试验；软件零速≠物理停止 |
| P5 | 确认 9 个电机已配置的 node_id 与波特率（推荐 J1–J9=1–9，1Mbps）| H2 | 用厂商工具或实读 property 31001 |
| P6 | 相机固定安装并手动测量外参（x/y/z + roll/pitch/yaw）| H6 | 外参参数化，测量值写入 `config/sensor_extrinsics.yaml` |

## 3. 命令路径蓝图（自顶向下）

```text
仿真模式（现状，不可破坏）：
  transition replay ──┐
                       ├─> /oscbf_command (JointState.position @ 100Hz) ─> oscbf_plant ─> /mujoco_joint_states
  oscbf_controller ────┘

真机模式（H1 起新增）：
  transition replay ──┐
                       ├─> /oscbf_command ─> CommandSafetyGate ─> 单位换算 ─> 0x19 CAN 帧 ─> CANable ─> 电机
  oscbf_controller ────┘                              │
                                                      └─(超时/故障/限幅)─> 零速帧 + 锁存
  电机反馈帧 ─> CAN 收包 ─> 解码 ─> 单位换算 ─> /mujoco_joint_states
```

要点：
- 执行端只有一个新节点 `hardware_bridge`（`live` 模式），`shadow` 模式只记录不发送
  （录制命令/状态/停车事件，供 vcan 与实机对比）；`sim` 模式 = 现在的 plant。
- 过渡 replay 与 OSCBF 命令共用 `/oscbf_command`，与仿真一致；replay 期间 watchdog
  同步生效（命令超时=replay 停顿即停车）。
- 安全网关在命令单位（rad/s/rad）层校验：速度上限、相邻命令变化上限、反馈超时、
  命令超时、关节数/有限性/故障码/急停，任何违例 → 零速帧 + 锁存 `stop_reason`；
  人工确认（外部监控节点，不自动清除）后恢复。
- MoveIt 的 controller_manager / ros2_control 闭环不进入本路线（最终 launch 无这些
  节点）；move_group 仅用于规划与 PlanningScene 碰撞检查。
- 零位偏移在单位换算层扣除：`q_urdf = sign × (reading_deg − zero_offset_deg) × scale`
  （J2–J9 的 scale=π/180 rad/deg；J1 由丝杆换算 m/deg；
  直接换算；J1 需 P2 的传动比 ε）。

## 4. 阶段拆分

### H0 仓库欠账清理与仿真基线（无功能变更）

- 删除死配置/悬空文件：`portable_oscbf/config/fcl_params.yaml` 的 `fcl_todo` 节、
  `robot_params.yaml` 的 `real_model_todo` 节、`launch/mujoco_transition_test.launch.py`
  （引用不存在的 `config/obstacles.yaml`）、`portable_oscbf/config/obstacle_params.yaml`
  中过时注释视情况更新。
- 更新过时文档计数：`portable_oscbf/README.md`（模块与测试数）、`docs/ONBOARDING.md`
  （"15+ 单测"→实际）、`OSCBF_PORTING_GUIDE.md` §9 过时段落。
- 新增 `make test-all` 形式的统一测试入口脚本（主包 + portable），README/CLAUDE.md
  补一行。
- 验收证据归档策略：`output/` 关键产物（M6/M10 性能报告、验收报告、基线 npy）移入
  `docs/evidence/` 或 `.gitattributes` 单独归类；README 注明出处。
- AC：`pytest`（主包）与 `pytest portable_oscbf/tests` 全绿；无任何文件引用已删除
  配置；文档计数与 `ls tests` 一致。

### H1 DrEmpower 协议栈 + vcan 验证（参考阶段 2）

- 新增模块：
  - `hardware_contract.py`：移植参考项目 `newaxis/hardware_contract.py`，改写为
    位置契约（`HardwareState`、`PositionCommand`、`WatchdogConfig`、`CommandSafetyGate`、
    每关节限幅、锁存/人工恢复；`require_hardware_executable()` 对 J1 fail-closed）。
  - `drempower_can.py`：纯函数帧编解码（无 I/O）：`encode_position`（0x19，
    字段单位以 P1 厂商库为准）、`decode_feedback`（pos/vel/torque/flags）、
    `encode_system`（0x08：clear_error/set_zero/estop/sync_start）、使能/失能流
    （clear_error → set requested_state=8 → 闭环；IDLE=1）。
  - `socketcan_backend.py`：python-can socketcan 后端（接口名 `vcan0`/`can0` 参数化），
    轮询 9 节点、丢帧统计、时间戳、重连；无 ROS 依赖。
  - `unit_conversion.py`：joint↔node_id 表、rad↔deg、单位换算与零位偏移；J1 未标定
    时断言禁止换算。
- 新增配置：`config/drempower.yaml`（波特率、node_id、命令参数）、
  `config/hardware_joint_zero.yaml`（占位：zero_offset=0，注：待机械零位标定）。
- 测试（vcan 集成，无真卡）：`test_drempower_frame_codec.py`（帧编解码 round-trip、
  错误注入）、`test_hardware_safety_gate.py`（超时/限幅/锁存/人工恢复参数化）、
  `test_vcan_backend.py`（vcan0 上模拟 9 个虚拟节点：轮询丢帧注入、watchdog 触发、
  重连）。
- AC：
  - H1.1 帧编解码单测 100%（含厂家示例帧对照，若 P1 到位）。
  - H1.2 vcan0 上 9 虚拟节点轮询：往返延迟中位数 <2ms、95% <5ms（单机脚本度量）。
  - H1.3 丢帧/错码/非有限值注入 → 网关 100% 输出零速并锁存原因。
  - H1.4 shadow 记录器可复现命令流（与 sim 同输入）。
- 前置：无（P1 只影响 H1.1 的厂家对照样例）。

### H2 单关节台架（参考阶段 3）

- 形式：整臂停在安全姿态，只使能并驱动一个轴低速（空载），其余轴断使能/锁定。
- 首选轴 J1（P2 到位后）；若 P2 未就绪先用 J2 作为首轴验证链路，J1 待 P2 后专项补验。
- 新建 `hardware_bridge.py`（ROS 2 节点）：订阅 `/oscbf_command` → 网关 → 后端；
  发布 `/mujoco_joint_states`（BEST_EFFORT、深度 20，与 `ros_conventions.py` 一致）；
  `hardware_mode` 参数（sim 不参与 launch / shadow=记录 / live=发送）。
- 单关节验收脚本：低速正弦参考（参考计划：首轴 0.05 m/s 等价 rpm、目标跟踪
  <1mm 或 <0.1°）。
- AC：
  - H2.1 使能/读状态链路 100% 通（反馈健康、单位/正方向与 URDF 一致——±5° 命令
    验证，对照仿真 FK）。
  - H2.2 编码器零位已 set_zero 固化，`hardware_joint_zero.yaml` 已填。
  - H2.3 低速正弦跟踪：误差 <1mm（J1）/ <0.1°（旋转轴），无丢帧、无故障码。
  - H2.4 急停与恢复演练一次：触发 → 停止且锁存 → 人工恢复 → 复位成功。
  - H2.5 网关对 J1（未标定）运动命令 100% 拒绝（fail-closed 验证）。
- 前置：P3（CANable）、P4（急停验证）、P5（node_id/波特率确认）、P2（若用 J1）。

### H3 全臂 shadow（参考阶段 4）

- 9 路同时只读反馈（不发运动命令）；`hardware_bridge` shadow+live 只读。
- AC：
  - H3.1 9 路反馈轮询延迟 <5ms（P99），丢帧率 <0.1%。
  - H3.2 任一节点离线/重启 → 网关按失败语义停与诊断，重连后恢复流程走通。
  - H3.3 反馈位置/速度与手动盘车对照（各轴方向、减速比、单位二次确认）。
  - H3.4 连续运行 10 分钟无异常（作为 H4 前置）。
- 前置：H2。

### H4 低速无障碍闭环（参考阶段 5）

- 全链：随机位姿 → AEB-RRT* 过渡（move_group 规划 + replay 下发）→ OSCBF 跟踪蝴蝶
  轨迹，全部经真机执行；速度取仿真 10%。
- launch 集成：`hardware_mode:=live` 分支；`start_oscbf_plant:=false`；
  J1 若未标定 → 过渡目标/轨迹的 J1 关节固定于标定点并按差集排除（或 P2 已完成则正常）。
- AC：
  - H4.1 端到端一次：过渡无碰撞、跟踪至终点、无 qp_fail、无急停触发。
  - H4.2 末端跟踪误差 <2mm（10% 速度、无动态障碍物）。
  - H4.3 软件限幅（速度/相邻命令变化/加速度项）全部生效，日志可审计。
  - H4.4 制动距离实测记录（急停命令 → 完全停止的最坏距离），对比 CBF 裕度。
- 前置：H3、P2（J1 参与运动时）。

### H5 虚拟动态障碍物（参考阶段 6）

- 控制器运行在实机反馈上，障碍物为程序生成（复用 bench 的动态障碍物能力），
  不接传感器。
- AC：H5.1 `dyn_min>0`、`qp_fail=0`；H5.2 反应可观察（日志/遥测字段）；
  H5.3 控制器障碍物路径（`obs_*` 启用，人工注入）与仿真行为一致。
- 前置：H4。

### H6 实机软障碍物避障（参考阶段 7）

- 感知实现：
  - 新增 `obstacle_extractor.py`：PointCloud2 → 背景/机械臂自过滤 → 聚类 →
    球/圆柱参数 → `obs_*`（复用 M9 接口路径；`perception_bridge.py` 占位逻辑移除）。
  - `sensor_extrinsics.yaml`（P6 测量值）+ 静态 TF 节点（替换 camera_to_world_static
    占位，`use_tf:true`）。
  - launch 增加 `start_perception:=true` 分支与 `orbbec_camera` 驱动段。
- 实验条件：软质泡沫球/柱，低速（≤0.05 m/s 等效），滑轨或手持，隔离区。
- AC：
  - H6.1 外参验收：已知尺寸物体验证点云位置误差 <20mm（参考项目 §7.4 流程）。
  - H6.2 障碍物从点云到 `obs_*` 的端到端延迟与频率记录（目标 ≥10Hz 输出）。
  - H6.3 实机避障：无碰撞、可观察反应、可安全急停（全程至少 3 次反复）。
  - H6.4 点云失效/老化 → 平滑降级（超时停车，不误判、不失控）。
- 前置：H5、P6。

### H7 验收收尾与文档

- runbook（`docs/real_robot_runbook.md`）：上电前检查、急停试验、只读反馈、单轴低速、
  九轴低速、传感器、故障恢复，每步含停止条件与回滚。
- 证据归档：各阶段日志/报告（H2–H6）入 `docs/evidence/`；README/CLAUDE.md 增
  「真机运行」小节。
- 更新 `CONTEXT.md` 词汇表（真机执行端、硬件桥接、安全网关词条）。
- AC：H7.1 全部 H1–H6 证据齐备且一致；H7.2 `perftest`/`pytest` 全绿；
  H7.3 文档无过时计数与悬空引用。

## 5. 批次与提交建议

每个 H 阶段 2–4 个提交，小步提交，提交信息沿用 feat:/fix:/refactor: 前缀：

1. H0：清理式提交（死配置删除、文档计数、测试入口）。
2. H1a：`hardware_contract` + `drempower_can` + 单测（无 I/O，纯函数）。
3. H1b：`socketcan_backend` + vcan 测试 + 配置。
4. H2—H7 按 AC 逐个提交；含硬件/感知配置与 launch 的合并成一个「feature」提交。

## 6. 风险与未决点

- 0x19 速度/滤波字段单位与取值范围：以 P1 厂商库/协议 PDF 为准，H1.1 锁定后不再变动。
- CANable 2.0 在 1Mbps + 9 节点轮询下的实际负荷：H3 实测，超压则上调轮询周期或
  采用 feedback 广播（厂商支持 `0x1E` 快读，轮询优先级字段可后续调整）。
- VMware 透传下 CAN 时间戳无硬件时钟：H3 以 P99 门衡量，不达标降级方案=裸机运行
  （决策 7 预留迁移路径）。
- 可操作度零空间策略（M8）在真机仍默认关闭（`use_nullspace_policy=false`），待限位
  感知正则化后再评估，不是本计划内容。
- 阶段 6 的「机械臂自过滤」需要点云含机械臂本体：以 Gemini 335L 视场与固定位置先
  做静态背景剔除 + 已知域（工作空间外）过滤，实机试点后迭代。

## 7. 变更记录

| 日期 | 内容 |
|------|------|
| 2026-08-22 | 初版：H0–H7 阶段拆分，14 项决策基线来自 grilling 访谈共识；模板与验收规则沿用 OSCBF 执行文档 |
