# 三路九个 DrEmpower 电机通信记录

2026-10-16，三路电机分别为 1、2、3；4、5、6；7、8、9。三路通过转接盒同时连接，电机之间连接线为 50 cm。九个电机与转换器的 CAN 速率均已设为 **500 kbps** 并保存，重新通电后的参数回读通过。

## 当前设置

| 项目 | 当前值 |
|---|---|
| 电机 `can.config.baud_rate` | 节点 1–9 均为 `500000`，属性地址 `21001`，类型 `u32` |
| 转换器 | DR-Tech DR-USB_CAN，序列号 `87949380174B` |
| 转换器 CAN 速率 | `UCONFIGR40` 返回 `value = 6 `，对应 500 kbps |
| 转换器模式 | 厂家标准模式，`UCONFIGR30` 返回 `value = 0 ` |
| 串口 | `/dev/serial/by-id/usb-Dr-Tech_DR-USB_CAN_87949380174B-if00`，Python 串口参数 `115200` |
| `axis0.config.extra_setting.enable_120r` | 节点 5 为 `0`；节点 1、2、3、4、6、7、8、9 为 `1`，与试验前一致 |
| 项目配置 | [config/drempower.yaml](../config/drempower.yaml) 的 `can.bitrate = 500000` |

后续使用上位机或其他 CAN 适配器时，CAN 速率须设为 500 kbps。当前接入使用厂家串口协议；项目配置中的 `can0` 尚未作为本次设备连接入口验证。电机编号与实际关节 J1–J9 的安装对应关系仍需人工核对。

## 实际验证

每轮逐个读取九个节点的 ID，校验回复中的节点、命令、属性地址、数据类型和 ID 值。每秒开始一轮，单次等待上限 0.6 秒；没有请求重试。每轮均保存原始收发字节和主机响应时间。

| 条件 | 轮数 | 有效回复 | 最大响应时间 | 结果 |
|---|---:|---:|---:|---|
| 三路全部接回并通电，500 kbps | 120 | 1080/1080 | 2.766 ms | 通过，无超时 |
| 三路保持连接，再次关闭电机电源、拔掉 USB 后重新通电 | 120 | 1080/1080 | 0.943 ms | 通过，无超时 |

两组检查分别持续 119.114 秒、119.119 秒，共 240 轮、2160 次有效回复；两组结束后九个 `can.error` 均为 `0`。再次重新通电后，九个电机速率仍均为 `500000`，转换器速率仍为 index `6`，终端参数保持原值。两次电机供电属性回读范围分别为 23.99～24.26 V、24.02～24.25 V。

原接线在 1 Mbps 下三路全部连接时为 0/9 回复，包括将每节点等待时间延长到 2 秒的检查。相同三路在 500 kbps 下恢复通信。当前证据确认了可用速率；尚无信号波形测量，具体电气原因仍未确认。

本次验证范围为静止状态的属性通信及断电后的参数保持。未进行运动测试，也未验证 100 Hz 控制通信。项目 `live` 入口仍保持关闭。

## 修改审计

用户授权通信参数试验、保存及必要恢复，并要求每次修改保留记录。记录目录为本机 `.scratch/motor_check_20261016/`，由 Git 忽略。所有记录均包含请求时间、目标、原始请求及回读结果；发送成功和实际回读成功分别记录。

| 时间（Asia/Shanghai） | 操作与结果 | 原始记录 |
|---|---|---|
| 12:06:04 | 三路连接时尝试将 1、2 号 `enable_120r` 写为 0；无回读，未保存 | [写入记录](../.scratch/motor_check_20261016/termination_20261016T120604_846728.json) |
| 12:06:36 | 尝试将 1、2 号恢复为 1；无回读，未保存；随后断电改为两路后读到 1 | [恢复请求](../.scratch/motor_check_20261016/termination_20261016T120636_712701.json) |
| 12:08:27、12:08:41 | 两路连接时 1、2 号完成 1 → 0 → 1，回读通过，未保存 | [设为 0](../.scratch/motor_check_20261016/termination_20261016T120827_758182.json)、[恢复为 1](../.scratch/motor_check_20261016/termination_20261016T120841_449229.json) |
| 12:10:41、12:10:42 | 两路连接时将 1、2 号设为 0 并保存；断电接三路后仍无通信 | [写入](../.scratch/motor_check_20261016/termination_20261016T121041_588552.json)、[保存](../.scratch/motor_check_20261016/save_20261016T121042_000204.json) |
| 12:12:59 | 重新连接两路后确认 0 已保持，再恢复为 1 并保存；后续重新通电回读为 1 | [恢复](../.scratch/motor_check_20261016/termination_20261016T121259_347949.json)、[保存](../.scratch/motor_check_20261016/save_20261016T121259_755840.json) |
| 12:16:55 | 节点 1、2、3、7、8、9 及转换器从 1 Mbps 改为 500 kbps；回读通过，逐节点保存 | [六个节点修改](../.scratch/motor_check_20261016/bitrate_20261016T121655_419196.json) |
| 12:19:15 | 仅连接 4、5、6，转换器临时恢复 1 Mbps，与这三个节点的原始速率一致 | [转换器修改](../.scratch/motor_check_20261016/bitrate_20261016T121915_029825.json) |
| 12:19:15 | 节点 4、5、6 及转换器改为 500 kbps；回读通过，逐节点保存 | [三个节点修改](../.scratch/motor_check_20261016/bitrate_20261016T121915_343237.json) |

最终保留的设备改动为 CAN 速率：九个电机从 `1000000` 变为 `500000`，转换器从 index `8` 变为 `6`。1、2 号终端参数已恢复；其他电机的终端参数未写入。厂家 `save_config` 保存节点当时的当前配置，本次主动写入的属性仅为上述通信参数。

## 本机复查入口

脚本位于 `/home/lsn/robot/robot_safecontrol/.scratch/motor_check_20261016/`。运行时保持只有一个程序占用串口。

```bash
cd /home/lsn/robot/robot_safecontrol
export PYTHONPATH="$PWD/src:$PWD/.scratch/motor_connection_20261004/deps"
python3 .scratch/motor_check_20261016/read_nodes.py --property can_bitrate --expect-value 500000
python3 .scratch/motor_check_20261016/read_nodes.py --property can_error --expect-value 0
python3 .scratch/motor_check_20261016/verify_stability.py --rounds 120 --label all_three_500k_recheck
```

这些命令仅发送属性读取。修改工具 `bitrate_trial.py` 必须带有 `--execute` 才会写入设备；省略该参数仅打印计划。速率写入使用广播，因此执行时必须只连接所声明的组别。

## 恢复原始 1 Mbps 的方法

当前保留已验证的 500 kbps。以下仅供需要恢复原始参数时使用；原三路接线在 1 Mbps 下的通信故障可能再次出现。恢复操作须使用上述运行环境，所有接线变化均在关闭电机电源、拔掉 USB 后进行，机械臂保持固定。

1. 仅连接 1、2、3 与 7、8、9，恢复供电和 USB，执行：

   ```bash
   python3 .scratch/motor_check_20261016/bitrate_trial.py group --group ac --from-rate 500000 --to-rate 1000000 --execute
   ```

2. 关闭电源并拔掉 USB，仅连接 4、5、6，再恢复供电和 USB。转换器此时为 1 Mbps，这三个电机仍为 500 kbps，按顺序执行：

   ```bash
   python3 .scratch/motor_check_20261016/bitrate_trial.py adapter --from-rate 1000000 --to-rate 500000 --execute
   python3 .scratch/motor_check_20261016/bitrate_trial.py group --group b --from-rate 500000 --to-rate 1000000 --execute
   ```

3. 各组重新通电后，分别读取本组全部节点的 `can_bitrate`，期望值为 `1000000`。不要将不同速率的组接在一起进行正常查询。用文件编辑工具将项目 `can.bitrate` 同步为 `1000000`，并更新本记录。

每次修改工具均检查原始速率、修改后的速率及 ID，全部回读通过才逐节点保存。任一环节失败时保留该次 JSON，依据回读结果确定当前状态，不连续重复写入。

## 验证证据

- [1 Mbps、三路、每节点等待 2 秒仍无回复](../.scratch/motor_check_20261016/nodes_20261016T120238_671592.json)
- [500 kbps、三路首次九个节点全部回复](../.scratch/motor_check_20261016/nodes_20261016T122008_746618.json)
- [第一组 120 轮汇总](../.scratch/motor_check_20261016/stability_20261016T122305_449305/summary.json)
- [第一组结束后的 CAN 错误码](../.scratch/motor_check_20261016/nodes_20261016T122514_636379.json)
- [再次重新通电后的转换器配置](../.scratch/motor_check_20261008/adapter_20261016T122657_145383.json)
- [再次重新通电后的九个节点速率](../.scratch/motor_check_20261016/nodes_20261016T122657_297900.json)
- [再次重新通电后的终端参数](../.scratch/motor_check_20261016/nodes_20261016T122657_449010.json)
- [再次重新通电后的 120 轮汇总](../.scratch/motor_check_20261016/stability_20261016T122657_839629/summary.json)
- [最终 CAN 错误码](../.scratch/motor_check_20261016/nodes_20261016T122930_520364.json)

原始日志中的 `persistence_verified: false` 表示写入当时尚未重新通电；保持能力由上述独立的断电后读取记录证明，原始写入记录保留原样。
