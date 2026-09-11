# [11] 真机执行链路:SocketCAN 后端与参数注入

- Tracker：https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13
- 日期：2026-09-09；执行者 Codex；已认领给 Shiningliu1011。
- 当前状态（2026-09-15 复核）：containment 已进入当前 main；本分支包含通信/诊断准备及其 freshness、响应隔离和运动许可修复，PR 仍为草稿、尚未合入。完整执行会话及实机验收未完成，原票保持 OPEN。

## 2026-09-12 与几何票联合对齐

用户本轮明确按 HTML 标题票号处理「11 真机执行链路」与「12 自碰撞 OBB 与环境几何模型核对」，使用 grill-with-docs；不是重新处理已经关闭的测试容差与配置一致性票。本轮只读核对当前代码、正式票据和 PR，未重跑历史测试、未操作 CAN 或设备。

- [准备 SocketCAN 传输与笔记本非运动诊断入口](https://github.com/Shiningliu1011/nineaxis_safecontrol/pull/43)仍为 OPEN / draft，分支 `codex/ticket13-socketcan-review`；协议修正、python-can 薄适配、doctor/probe 与本轮 freshness、响应隔离、运动许可修复属于该分支，尚未进入当前 main。
- 当前 main 的 `hardware_bridge` 明确拒绝 live；sim 不创建硬件 I/O，shadow 不收发 CAN。PR 分支实现了 `SocketCANBus._create_backend` 与非运动诊断，但位置/使能/失能仍要求安全 owner 注入的 motion permit。
- 已有架构方向继续沿用 python-can、既有编解码/总线边界及 CommandSafetyGate，不重新投票。可独立推进的工程缺口是完整执行会话：真实反馈身份与时间、命令产生时间和独立断流检查、全轴映射/标定拒绝门、发送失败锁存与人工恢复。
- Linux vcan 通过证据、实际电机/适配器身份、J1 传动与全轴标定、设备独立看门狗及承重停车能力仍缺。协议读取成功和软件保持命令均不等于物理停止；这些事实不能由访谈投票代替实测。
- [选定不可行/裕度/降级策略](18-infeasibility-margin-degradation.md)的失败锁存、人工恢复及低通预算决议作为后续接线输入；控制器到执行端的连接缺口见[命令链事实核验](../research/18-recovery-command-chain-20260912.md)。该连接未因单模块网关测试通过而完成。

2026-09-12，用户已接受首版限定在经认证的任务关节范围内，覆盖过渡与停车所需范围，范围外拒绝运行；具体含义及待验证的确定方法见[几何票联合对齐](8-obb-environment-geometry.md#本轮已接受决策首版认证的有效范围)。执行端后续必须保留真实位置/速度与时间依据，使准入和停止范围检查能够成立；该范围尚未计算或获证，不意味着只检查位置上下限即可准入。本票不关闭、草稿未合入、live 限制未解除；实际反应与制动证据仍是明确缺口。

## 2026-09-11 实施接续：笔记本连接准备

用户目标：出差回去后，笔记本不需要大量调整即可连接控制机械臂电机。
本轮已接续认领，基线 `73b99ef`，起始工作区 clean。目标笔记本暂按当前 Ubuntu 22.04；
已询问适配器、电机型号/固件与节点/标定信息，本轮尚未收到补充，未假定设备已到货。

### 已落地

- 复用 python-can 4.6.1（LGPL-3.0-only）的 SocketCAN/virtual 与错误处理，增加同步薄适配；
  禁止隐式环境配置覆盖，不管理网络，不自动使能/恢复。`setup.py[hardware]` 声明可选依赖。
  当前用户环境已安装 python-can 4.6.1、wrapt 1.17.3（BSD），保留 PyYAML 5.4.1（MIT）。
- 保留现有 codec / backend seam / SocketCANBus；修正属性类型码、requested_state=30003/u32、
  mode0 绝对值截断和带宽边界；clear-error 发送失败不继续使能；释放旧连接并清空旧节点缓存。
  离线/坏反馈返回 NaN 与无效时间，拒绝错节点/属性响应；轮询 latency 改为请求调用实耗时，
  不再把两次轮询间隔称为延迟。旧串行 quick-state 层仍非经资格验证的 live loop。
- 修复轮询路径丢弃底层接收时间的问题：反馈保留 python-can 的原始时间，并拒绝超时、未来、
  轮询前及类型化属性响应帧；位置、使能、失能接口默认需要安全 owner 注入的 motion permit，
  缺少许可时不发送。
- 新增 `hardware_probe`：doctor 仅检查依赖、接口类型/UP/bitrate；probe 显式选节点，
  只发类型化属性读。每个属性采用所有节点共享期限，检查响应节点/地址/类型/长度及原始时间，
  保留实际 RX 时间与缺失项，不发布控制反馈。CAN 无事务 ID，不宣称设备采样年龄已知。
- 新增 `scripts/hardware_check.sh`，复用 `config/drempower.yaml` 的 can/node_ids，
  不依赖 ROS source 或重编译；安全/命令/轮询候选字段仍未接入 live。
- 修正 vcan 脚本仓库根目录及假通过问题：仅在实际 UP 的 Linux vcan0 上运行双端测试，
  缺依赖/接口 exit 2，绝不回退 Fake。无自动 sudo、无物理 CAN 测试。
- [笔记本快速入口](../../../hardware_laptop_quickstart.md)集中依赖、现场配置和待确认事实；
  修正标定表 J1 offset 的注释（仍为输出轴 deg，随后才转换成 m），标明旧 runbook 不适用于当前 live。

### 协议独立证据

除下方历史源码指纹外，新核验本地《DrEmpower电机CAN通讯协议说明 v2.0.pdf》印刷页 7–9：
属性响应沿用请求 ID；布局为地址 u16、类型 u16、值；快速反馈 bit0=1；输出轴位置示例是度。
PDF SHA256：`464e65de8a4c6fdd00d3609e5052782777170da5a382946a88b7f14562f7ab98`。
独立 golden test 使用 PDF 例子：节点4 `0x9E / 7794000038f0b8c2` → 约 -92.47 度，
以及厂家枚举推导的 `3375030008000000` 使能 payload。未把厂家库导入运行链。
本地资料不能替代实际固件与已知角度回读。

### 本轮验证

证据目录 `output/ticket13-implementation/` 被 gitignore，仅本机可访问。

- 原 PR focused：source ROS Humble/install 后，以 localhost/domain217 运行 codec、CAN、probe、
  conversion、hardware bridge/contract/stack、launch structure：**140 passed / 1 skipped**，exit 0；
  该历史结果未覆盖本轮修复。
- 本轮修复定向测试：`drempower_can`、`socketcan_backend`、`python_can_backend`、`hardware_probe`
  **70 passed / 1 skipped**，exit 0；新增覆盖真实时间戳、过期/属性混入拒绝及 motion permit 默认拒绝。
- 另运行 hardware bridge、contract、launch structure、final launch runtime：**51 passed**，exit 0；
  其中 final launch runtime 证明 live 仍在 Node 启动前拒绝。
- `bash scripts/agent_check.sh`：exit 0，pure contracts **69 passed**。
- Python virtual 双端：九节点逆序、错地址/节点、过期队列、缺帧、错误帧、发送失败、关闭与缺接口均有验证。
- Linux vcan：**NOT RUN**，vcan0 不存在；模块文件存在，但 `sudo -n true` 提示需要密码；
  未请求或使用密码、未改网络。脚本 exit 2 如实报告。
- `bash scripts/hardware_check.sh`：exit 2，python-can 4.6.1 已就绪，can0 缺失，0发送/未开CAN；
  该失败是当前设备环境事实，不是转为 shadow 的隐式成功。
- `git diff --check` 通过。本轮未重复与驱动无关的完整 portable/JAX 回归。

### 未完成与下一步

本票不关闭，不向地图 Decisions so far 写入完成结论。
`hardware_bridge` 与 final launch 的 sim/shadow/live containment 保持原有运行边界。
**当前不能直接控制实机**；本轮没有引入 motion-ready 开关来绕过资格验证。

1. Linux vcan 实收发待管理员创建虚拟接口后运行固定脚本。
2. 设备型号/固件、适配器驱动、真实接口/bitrate、节点物理身份和全轴标定待现场事实。
   无设备身份时无法可靠生成自动联网规则；不默认刷固件或把占位表当标定完成。
3. 完整执行会话仍需实现/接线并验证：真实反馈 freshness、命令产生时间/断流定时器、
   全轴映射与标定门、发送失败后的锁存和人工恢复；单纯诊断读取不能代替此链路。
4. 独立硬件看门狗和承重/制动策略，以及受控低速运动准入，仍接续原有执行保护与实机验收票。
   没有已验证的物理停止策略前，不将软件 estop/disable 视为受控制动。

回退：移除本轮软件提交/变更即可恢复之前 containment；现场配置和设备未被修改。
当前机器新增的 python-can/wrapt 是用户级可选依赖，可独立卸载；不修改系统 ROS 包。

## 起点（2026-09-09 历史记录）

继承 [可复现基线与证据目录](30-baseline.md)。本机 main 与本次远端 main 查询均为 `659da6c6db598abddc272ad0941b72f77793211b`。原有 MAP.md 修改、handoffs、.scratch/oscbf-reuse-wayfinder 和大然电机资料保留，未提交或覆盖。原始状态见 `output/audit-13/status-before.txt`。

用户原始指令：“读取交接文档继续处理下一个ticket”。本窗口按 wayfinder 默认研究/决策范围完成接口核验和隔离测量，未改产品代码、安装依赖或运行硬件。

## 证据与复现

本地证据目录 `/home/lsn/robot/robot_safecontrol/output/audit-13/` 被 gitignore，跨机器须另行复制。`probe.py` 仅导入厂家枚举和项目代码，使用 FakeCANBackend；不实例化厂家驱动、不打开 CAN。复现：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 output/audit-13/probe.py
python3 -m pytest tests/test_drempower_can.py tests/test_socketcan_backend.py tests/test_hardware_bridge.py tests/test_hardware_contract.py tests/test_hardware_stack_integration.py tests/test_unit_conversion.py -q
```

Python 3.10 环境继承基线。本次相关测试 **103 passed in 0.42s**，exit 0（tests.log/tests.exit）；探针最终 exit 0（probe.json/probe.stderr/probe.exit）。最初探针误写厂家变量名 property_address，修正为原始 property_addresss 后执行；未修改厂家文件。探针是事实采集，不是缺陷已修复的回归测试。

`interfaces.txt` 未见 can0/vcan0；`python-can.txt` 确认 python-can 未安装。没有执行 vcan 脚本、sudo 或实际收发；不能从接口缺失推断硬件是否已到货。

## 协议核验

来源为用户本地厂家资料 `大然电机/大然电机所有资料/drempower-wiki-master/4-库函数/python/linux-socketcan/` 下 DrEmpower_socketcan.py 与 interface_enums.py。前者头部版本 v1.0、日期 2023-06-28；根 LICENSE 为 MIT，Copyright (c) 2021 唐昭。本地副本没有可核对的上游提交身份，不能宣称适配当前设备固件。

- 驱动 SHA256：`d6c4b9babefe868475a3be6499fda959937f3872329d1efc5495f6403d992d6f`
- 枚举 SHA256：`b9e17127ba7a916e2c67a863cc746c08db17e5584bf989b6e2608e60b587fcaa`
- LICENSE SHA256：`a027be09a86974c22239e22fad36b0962f566ee7819a887addee46b61a92d0ec`

| 项目 | 厂家本地源码事实 | 当前项目差异 |
|---|---|---|
| 属性写 | write_property：u16 地址 + u16 类型码 + 值；u32 类型码为 3 | encode_property_write 缺类型码，值提前两字节 |
| 状态地址 | 执行原始枚举：current_state=30002，requested_state=30003，均 u32 | requested_state 假定为30002，使用 u16；check_node_status 从 data[2:4] 读取类型区 |
| 使能数据 | 按厂家布局，requested_state=8 应为 `3375030008000000` | 当前为 `3275080000000000` |
| 快速状态读取 | get_state 使用 0x1E，地址0/类型0；反馈 f32+s16+s16 | 现有全零请求可保留，不能误判成无效查询；需补节点/帧种类/新鲜度核对 |
| 模式0位置指令 | set_angle：0x19，输出轴角度deg、速度rpm、输入滤波带宽；速度/带宽乘100后取整 | 布局基本一致；现有 round 与厂家截断、符号/范围规则不同，默认 filter_accel=1.0 未证明适合100Hz |
| 反馈单位 | 构造函数明确 degree/rpm/Nm；get_state docstring却称位置为转 | 源码内说明有冲突，设备固件/已知角度回读仍须台架确认 |
| 急停 | estop 文档：进入IDLE、卸载、置错；恢复需清错并重新闭环 | 不能把总线estop等同于承重轴受控制动；真实停止能力未证实 |

## 执行链与测试缺口

1. SocketCANBus 是管理层，真实工厂仍抛 NotImplementedError；厂家资料已包含基于 python-can 的 SocketCAN 实现。既有测试只注入 Fake，不覆盖 Linux socket。
2. HardwareBridge 只接受 Python 构造参数，无 ROS declare/get 参数。实测 `--ros-args -p hardware_mode:=live` 后实际仍为 shadow；launch/YAML 未打通。
3. 全九轴 Fake 反馈离线后，_tick_feedback 发布九个零；随后一条零位置命令在 J1 未标定时仍向其余八轴发送，锁存原因为空。_on_command 将反馈标成当下时间、feedback_ok=True、qdot=0，丢失真实年龄和健康；缺首帧也用目标代替反馈。
4. 命令时间采用到达时刻而非消息产生时刻；无独立命令断流检查，只有新命令才执行安全门。节点错误、发送失败和停止原因未形成可靠的传输停止通路；人工解除锁存也构造健康状态。
5. 9轴串行 recv 各10ms，单轮超时等待预算可达约90ms（源码预算推导，未实测），不满足宣称的100Hz；CANBusMetrics 的 latency 是反馈轮间隔，不能作为请求响应耗时。
6. scripts/vcan_integration_test.sh 根目录实际落在 scripts/，且最终运行的仍是 Fake 测试；ROS source 也存在 nounset 风险。不能沿用其“vcan验收完成”输出。
7. 现有测试有意接受未标定J1只跳过该轴、缺真实反馈即发送等行为，因此103通过不能充当安全验收。

## 复用结论与编码门

无新增用户取舍；继承九轴/5D、复用择优、关键无效停止锁存及实机独立准入原则。

可保留 CANBusBackend seam、总线管理/编解码分层、JointNodeMap/UnitConverter 和纯逻辑安全门，但修正协议和端到端健康接线。优先通过 python-can 直接复用 Linux SocketCAN；尚未安装，具体版本及依赖许可证冻结在实施阶段核验，不虚构已经通过的版本。厂家 MIT 代码用作协议参考和 golden-frame 来源；其初始化自行 sudo/修改接口、共享状态与宽泛异常处理不直接带入 ROS 节点。需要摘录代码时保留许可。收益是避免自写 CAN socket 与继承隐式系统副作用，性能收益仍待测量。

**可以开始离线编码**，无需再选择总体架构，具体范围：

- 修正厂家资料对应协议 profile（u16类型码/u32状态、读响应校验、模式0字段明确命名），固定独立 golden frames；实际固件未知时不得据此开启live。
- python-can 薄适配与 ROS 参数注入，缺依赖/接口/非法模式显式失败；shadow 不得发生运动命令发送，sim 不占用执行链。工厂不隐式配置网络或自动使能。
- 保留逐节点真实时间/速度/错误，缺失映射、标定、反馈须整机拒绝运动；处理乱序、错节点、属性响应混入、发送失败、资源关闭；独立期限检查与失败锁存接线。实际硬件看门狗/制动能力继续交后续执行保护与验收票。
- 修正 vcan 入口并建立真正经 Linux vcan 的双端收发测试，分开 Fake、python-can virtual 与 vcan 证据。

验收必须覆盖上述探针从错误行为转为拒绝、CLI/YAML生效、无后端显式失败、9轴反馈/故障注入、断流无新命令也触发保护、未知状态不发位置、重连不自动恢复、关闭释放资源。协议验收不能只让同一错误编码器与解码器互测。变更后运行相关单测与ROS桥接测试，再执行可用的vcan；记录跳过，不能算通过。

**尚不能启动真实硬件**：仍缺精确电机型号/固件、实际节点映射与单位回读、J1传动及各轴零位/方向、总线适配器配置、独立通信超时行为和承重/制动能力。厂商枚举有 watchdog 错误标志不等于已配置独立看门狗。100Hz与滤波参数仅是候选，须实测。实机操作留给受控低速实机验收。

## 改动、回退与下一窗口

本票只新增本交接和 ignored 审计证据，向原票发布进展；未改运行代码、未提交/合入/部署。地图 Decisions so far 不追加未解决票；不另建同义票。

本票仍开放，下一步优先在本票实施上述离线范围并验收。若继续规划队列，下一个为 [spec 与参数文件坐标系/术语修订](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/20)，它不能将本票执行链视为通过。保留原基线归档；撤销本窗口只移除本交接和 audit-13 资产，不清理用户原资料。本窗口不自动启动下一票。
