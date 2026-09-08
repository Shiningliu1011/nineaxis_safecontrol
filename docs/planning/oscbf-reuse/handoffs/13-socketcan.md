# [11] 真机执行链路:SocketCAN 后端与参数注入

- Tracker：https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13
- 日期：2026-09-09；执行者 Codex；已认领给 Shiningliu1011。
- 状态：离线核验完成；实施与验收未完成，issue 保持 OPEN。

## 起点

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
