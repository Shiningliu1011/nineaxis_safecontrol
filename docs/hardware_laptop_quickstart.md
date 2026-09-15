# 笔记本连接电机：准备与现场入口

当前提供环境检查和不发运动命令的属性读取。`hardware_bridge` 和 final launch 的
`live` 入口仍拒绝启动；本页不是可运动验收。不要通过删除 live 拒绝条件接线。

## 出差期间可完成

当前目标是这台 Ubuntu 22.04 / Python 3.10 笔记本。驱动复用
[python-can 4.6.1 SocketCAN](https://python-can.readthedocs.io/en/4.6.1/interfaces/socketcan.html)，
固定版本声明在 `setup.py` 的 `hardware` extra；python-can 为 LGPL-3.0-only，
wrapt 为 BSD，PyYAML 为 MIT。通过安装的库调用，未复制库源码。

首次准备依赖（当前机器已安装 python-can 4.6.1，PyYAML 5.4.1）：

```bash
python3 -m pip install --user 'python-can==4.6.1' 'PyYAML>=5.4.1,<7'
```

在仓库根目录运行，或用脚本绝对路径从任何目录运行；不用重新编译 MoveIt 或 source ROS：

```bash
bash scripts/hardware_check.sh
```

默认 `doctor` 不打开 CAN、不发帧、不改网络。输出 JSON，接口/依赖不满足时 exit 2；
即使 `transport_ready=true`，也始终有 `motion_ready=false`。
连接配置只有 `config/drempower.yaml` 中 `can.interface`、`can.bitrate`、`node_ids`
这一个入口；检查器从文件读取，不在脚本中复制接口名或波特率。
该文件的命令/安全/轮询字段仍是未接入 live 的候选值，不因检查通过就生效。

软件离线验收：

```bash
python3 -m pytest tests/test_drempower_can.py tests/test_socketcan_backend.py \
  tests/test_python_can_backend.py tests/test_hardware_probe.py -q
bash scripts/vcan_integration_test.sh
```

第二条要求真实 Linux `vcan0`。缺接口时 exit 2，不会偷偷换成 Fake 测试。
一次性创建测试接口需要管理员执行以下命令；仅创建虚拟接口，不操作 `can0`：

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

测试仅向确认类型为 vcan 的接口发帧。Python `virtual` 是进程内双端测试，
与 Linux vcan、USB 适配器、电机验收分别记录。

## 回来后先补齐这些事实

| 项目 | 当前事实 / 待确认内容 | 落点 |
|---|---|---|
| 笔记本 | 默认当前 Ubuntu 22.04 笔记本 | 换机器时重做环境检查 |
| USB-CAN | 型号、固件、序列号和驱动未知；不能假定 candleLight 或 SLCAN | 先确认系统呈现的接口，再填 `can.interface` |
| CAN 波特率 | 1 Mbps 是候选，须与设备核对 | `can.bitrate` |
| 电机 | 精确型号/固件、是否适用本地厂家 v1.0 协议未知 | 记录设备清单与协议对照 |
| 节点身份 | J1–J9 = 1–9 是推荐分配，尚未实读 | `node_ids`，逐台确认，不自动改设备 ID |
| 零位、方向 | YAML 中 +1/0 是占位，未获得标定证据 | `config/hardware_joint_zero.yaml` |
| J1 | 已记录导程 5 mm/rev，传动比、方向/行程/回零待确认 | 实测后交传动配置接入；不假定传动比为 1 |
| 执行保护 | 独立断流看门狗、承重轴停车/制动未验收 | 最终命令与执行保护、受控低速实机验收票 |

USB 适配器接入方式必须据实确定：原生 SocketCAN 和串口 SLCAN 的准备不同，
因此此时不刷固件、不写自动联网规则、不假定设备序列号。
确认是原生 SocketCAN、实际接口确为 can0、实际波特率确为 1 Mbps 后，
现场可用 `sudo ip link set can0 up type can bitrate 1000000` 配置接口。
驱动和检查器均不执行这条管理命令；已连接的接口不会被脚本 down/up 或自动重启。

## 现场不发运动命令的读取

在设备型号/协议确认、现场上电条件满足后，显式选择待查 ID，例如只查已确认的节点 1：

```bash
bash scripts/hardware_check.sh probe --nodes 1
```

此入口仅发送 0x1E 类型化属性读取：current_state、axis_error、output_position、
output_velocity。不会广播扫描，也不会使能、清错、失能、归零或写位置。
读取节点 ID 应与配置中的映射做人工物理核对，软件看到 ID 并不能识别它安装在哪个关节。

每一属性采用全节点共享的超时期限；响应必须匹配节点、地址、类型、长度和原始接收时间。
输出 `communication_complete` 只表示请求属性齐全；电机错误码、闭环状态和读数仍需查看。
原始输出轴单位按本地源码暂按 deg/rpm 解释，尚须已知角度/转速对照；工具不转换成 URDF 状态。
CAN 无事务 ID，延迟到达的同型响应及设备采样年龄不能仅凭接收时间证明，
因此诊断报告不能作为控制反馈或独立看门狗证据。运行时应只有此工具作为属性请求者。

## 接下来才能进入运动

本次软件改动完成的是驱动与诊断准备。仍需接通并独立验证执行会话、真实反馈新鲜度、
命令断流检查、全轴映射/标定门、失败锁存/恢复及停车策略，再做受控低速实机验收。
`shadow` 仅记录命令与拒绝，不会读取 CAN 或发布硬件关节状态。
“插线后少调软件”的可交付目标是依赖预装、连接参数集中、检查/读取命令固定；
当前还不能承诺回来后直接运动。现场实测的标定和物理停车证据不能用默认参数替代。
