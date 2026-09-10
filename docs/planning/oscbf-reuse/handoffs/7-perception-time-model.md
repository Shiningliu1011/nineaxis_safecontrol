# [10] 感知时间同步与延迟模型（点云融合 → CBF 输入）

- Tracker：https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/7
- 日期：2026-09-09；已认领：Shiningliu1011。
- 阶段记录：[GitHub 进度评论](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/7#issuecomment-5595568265)，非 resolution，不追加地图已完成索引。
- 状态：本地审计与测量契约已整理；整票保持 OPEN，尚未得到最终 age_warn / age_stop 或完成诊断接线。
- 起点：main `f3cc336b13f4435580200f54dd00b83197dca3cf`。保留已有驱动交接修改及未跟踪标定、日志、电机资料。

## 范围与前置

用户原话：“继续下一个ticket”。沿用[官方驱动交接](31-official-sensor-drivers.md)指定的下一窗口和[推荐窗口顺序](../DECISION-WINDOW-ORDER.md)。官方驱动票仍是本票的原生阻塞；仓库顺序规则允许先推进不依赖缺失证据的研究，不撤销依赖、不伪称前置通过。GitHub 本票旧正文的 `Blocked by: None` 已过时，实际关系以原生依赖为准。

本轮只分析已有采样、执行隔离离线审计、核验上游和整理交接；没有启动设备、修改运行代码/参数或授权真机运动。本轮没有新增用户取舍；双源逐源有效性、覆盖失效停车锁存、覆盖恢复后人工确认均继承 ADR 0003/0006。

## 本地时间流转与复现

源码起点与上述 commit 对齐；下列相对链接均相对于本交接文件。隔离脚本和原始结果在 [audit_time.py](../../../../.scratch/oscbf-reuse-wayfinder/7/audit_time.py) / [audit-results.json](../../../../.scratch/oscbf-reuse-wayfinder/7/audit-results.json)。脚本只导入纯 Python 融合引擎和读取历史 JSON，不构建或发送 ROS 消息。

1. [perception_bridge.py](../../../../src/robot_safecontrol_moveit/perception_bridge.py) 的 `_sensor_callback` 将 header 的 sec/nanosec 转为 float64 秒；只保留 xyz，丢弃 LiDAR 逐点时间。空输入或过滤后为空直接返回，不记录此次消息到达与失败原因。外参变换按配置帧及 header 时刻请求；静态外参不能修正动态环境在扫描期间的运动。
2. `_fusion_callback` 在处理前读取 ROS now，交给 [FusionEngine](../../../../portable_oscbf/work/fusion_engine.py)。默认相机/LiDAR 最大年龄各 0.5 s，配对差上限 0.1 s，占据过期 0.3 s，perception_timeout 1.0 s；这些是当前配置事实，不是已验收的时间预算。
3. 引擎以最新 LiDAR 选最近相机，超过配对差上限丢弃较旧一路；`camera_age` 报告最新相机而非实际选中的相机。复现 now=100，camera=99.91/99.99，lidar=99.92：实际使用相机 99.91（90 ms），报告 camera_age=10 ms。fusion_stamp=99.92，fusion_age=80 ms，低估最旧贡献的90 ms。
4. 年龄只检查 `< max_age`，没有负年龄拒绝。复现 now=100，camera stamp=101：age=-1 s，camera_alive=1，perception_valid=1。不能将负值钳为0然后继续使用；应先判定时钟映射可信性。
5. 重复配对调用返回 `_empty_result`：两源仍 alive，但 perception_valid=0，fusion_stamp=0。应区分“没有新帧”和“旧观测已无效”，否则直接接健康门会产生错误失效或错误缓存语义。
6. `fusion_stamp` 通过 Float32MultiArray status 发布。对历史样本 epoch 秒，float32 相邻可表示值相差128 s；它不能承担毫秒级绝对时间。短 duration 可以使用浮点，但绝对时间必须保留 sec/nanosec 或 int64 ns。
7. `_publish_tracks(tracks, now_s)` 的 now_s 未编码进消息；8×10 slots 只有几何、速度、enabled、d_safe、alpha。[oscbf_controller.py](../../../../src/robot_safecontrol_moveit/oscbf_controller.py) 的 `_tracks_callback` 缓存这些字段，未保存采集或接收时刻，也未订阅 perception/status。融合无源后不发布 tracks，因此旧 `_obs_state` 不会因时间过期而主动失效。
8. 融合引擎在计算聚类 dt 之前更新 `_prev_fusion_stamps`，随后减去当前组合的最早 stamp：单源会得到0后取1/30 s，双源可能得到源间时间差，而非上一融合周期的时间差。这是速度增强可信性缺口，后续 tracks 契约实施需一并核验；不能将当前估计速度当作可信最大相对速度。

复现命令：`python3 .scratch/oscbf-reuse-wayfinder/7/audit_time.py`。本轮执行成功，退出码0；解释器、平台和相关代码 SHA256 见 [environment.json](../../../../.scratch/oscbf-reuse-wayfinder/7/environment.json)。这是隔离审计，不是产品修复验收。没有修改代码，未运行全量测试。

## 历史实测能支持什么

输入来自[官方驱动在线采样目录](../../../../.scratch/oscbf-reuse-wayfinder/31/online-2026-09-09/)，每份输入 SHA256 已写入审计 JSON。旧 probe 仅记录 `[time.time(), header_stamp, point_count]`，没有单调时钟、设备原始时间、曝光/扫描区间和驱动发布时刻。

| 采样 | 帧数 / 订阅端 Hz | callback wall − header 中位数 / 最大值 | header 相邻间隔最大值 |
|---|---:|---:|---:|
| 相机单源 | 313 / 26.263 | 43.738 / 46.730 ms | 333.453 ms |
| 相机并发 | 244 / 20.628 | 43.567 / 46.397 ms | 500.585 ms |
| LiDAR 单源 | 121 / 10.000 | 100.511 / 101.947 ms | 101.329 ms |
| LiDAR 并发 | 121 / 10.001 | 101.098 / 101.991 ms | 100.511 ms |

这些差值是时间域尚未独立校验的残差，不是已证明的采集至回调延迟。采样约12秒，不能给长期上界。相机间隔异常可能来自发布、注册/同步、调度或传输丢帧，旧 probe 无法定位。0.5 s 间隔也不能单独证明在线年龄门已经触发，因为缺少门执行日志。LiDAR约100 ms残差不能据此断定网络传输耗时100 ms。

## 上游复用结论

固定源码核验见[官方时间语义研究](../research/perception-time-model.md)。研究在隔离分支 `codex/research/perception-time-model` 提交 `fe81eaf`，未推送；文档已复制至当前主工作区，未提交/合入产品分支。

继续复用 Livox driver2 1.2.6 / SDK2 1.3.1（自有部分 MIT，保留第三方通知）与 Orbbec ROS2 v2.9.3（wrapper Apache-2.0，SDK另核条款），许可证证据见[驱动研究](../research/official-sensor-drivers.md#license与验收边界)。保留现有融合管线作为对照；必要改动为时间/provenance/诊断薄适配，不重写驱动、配对或通用时钟设施。message_filters只是配对工具，本票没有证据要求替换现有配对算法。

Livox未同步时用主机 `high_resolution_clock` 处理包时间替代设备时间；该时钟的epoch与count单位须核本机构建。PointCloud2逐点timestamp是FLOAT64绝对ns，header取批首点，不能误读为逐点相对秒。Orbbec注册彩色点云取depth_frame时间；global是映射到主机域的采集时间，仍须实测误差及具体时钟选择。这些差异由适配记录并判有效性，无法证明采集年龄的源不得仅凭“时间接近当前”取得准入。

## 建议时间契约（后续实施输入）

本节是基于既有逐源/覆盖决议的工程细化建议；尚未实现或完成设备验收。

每路保留 source_id、设备身份、boot/session epoch、frame sequence、原始设备时间及单位、header时间和其语义、采集区间 start/end、时钟域、映射版本与误差上界、回调 ROS时间与单调时间、frame_id、标定ID和明确无效原因。没有可靠 frame sequence 时记录其缺失，不能用订阅计数冒充设备计数。相机注册点云应记录真正几何来源（深度）与配对彩色帧，不能由彩色帧刷新深度年龄。LiDAR 保留逐点时间或可信扫描区间；不能把整帧当作瞬时采样。

处理时刻另记录：预处理始末、选中各源帧、融合始末、观测发布、控制接收、QP开始/结束、最终命令校验与实际执行反馈。源时间不因重发、融合或预测而刷新；预测估计时刻与最后测量时刻分开。耗时/无新消息超时用单调时钟；ROS /clock 暂停或回跳时不能让真机看门狗停止计时。仿真时间和真实运行时间不能混用。

记映射后的最早有效采集时刻为 t_start,i，时钟与采集语义误差上界为 epsilon_t,i。在控制消费时刻，同一可信时间域内的保守年龄为：

`A_i = max(0, t_control - t_start,i) + epsilon_t,i`

此公式只用于已验证的时钟映射；未来时间超出映射误差界、零/非有限时间、会话变化或映射失锁先置无效，不通过 max(0, ...) 掩盖错误。只有 header 而无扫描区间时，须先证实 header 意义并补扫描/曝光宽度界，不能假设其为采集起点。

使用每项必要覆盖和几何实际贡献源的年龄；诊断 `max_obstacle_age_ms` 可汇总最大值，但不能代替逐源字段。融合内部仍保留来源及最旧有效贡献。跨源配对差只约束时间错位，不证明同步，也不独立决定覆盖是否足够。

用于推导的空间预算写为：

`B_i >= d0 + epsilon_geom,i + v_rel_bound,i * (A_i + T_remaining_bound) + D_stop_bound`

其中 B_i 是该动作及必要覆盖下可用的保守间距预算；epsilon_geom 包括以米表示的标定、深度/体素/几何/跟踪误差界；T_remaining 是消费后至停止反应起始的控制/命令时延界；D_stop 是此后机器人制动位移与障碍接近位移的保守合计。同一部分不可重复计入时间项与制动项。已对未来位置膨胀/预测的几何需声明基准，避免重复补偿。

当 v_rel_bound>0 且适用域明确，可推导 `A_stop <= (B_i-d0-epsilon_geom,i-D_stop_bound)/v_rel_bound,i - T_remaining_bound`。非正余量表示该工况不能准入；速度为0的退化情形单独评估，速度未知不等于0。有加速度界时改用相应可达位移界。此式只是预算核算框架，不是 CBF 安全性证明或完整人机协作标准验收。

`A_warn < A_stop`，两者的差必须容纳检测与已验证降进给响应；测量分位数用于性能比较，不能冒充故障时间上界。尚缺速度工况、几何误差、剩余控制时延及实际停止证据，故本票不提供虚构最终秒数，0.5/1.0 s 不升级为安全合同。超过使用域或必要覆盖失效且无有效保守替代时停止锁存，不删除过期约束后继续运行。

## 测量和编码交接

可以开始编码的范围：独立只读记录器、上述时间字段的离线校验/报告、合成时间故障回放。需要用户后续实施指令才进入产品代码；本轮没有新增运行程序。

正式诊断及控制接线尚不能按旧8×10 slots直接实现并宣称完成。应由本票提供字段/计时要求，与[观测契约升级](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/21)落实消息、[感知健康接线](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/23)落实有效性与锁存、[控制率与延迟预算](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/17)及[最终命令与执行保护](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/33)补消费后时间和执行证据。不要反向增加循环阻塞。

验收至少覆盖：两路独立/同时采样；预热与稳定阶段；CPU/GPU及录包组合负载；注册点云发布间隔异常定位；原始与映射时间同时记录；时钟偏移/漂移/跳变、设备重启、乱序、重复、空帧、停流；证明新源不能刷新旧源年龄，发布/处理不能刷新采集时间，控制端断流仍会自行过期。离线故障注入不得修改主机系统时钟或访问命令流。随后再在明确设备操作范围内补长期实测，保留样本量/时长/负载及分位数，不把一次通过当全工况上界。

回退：保留当前源码、配置及驱动版本快照；诊断先独立部署，停止诊断即可撤回。未来消息迁移须显式区分新旧版本并恢复匹配的生产者/消费者，禁止用旧无时间字段消息绕过准入。

下一推荐研究窗口：[自体过滤 bridge 接线：逐源 MoveIt 几何过滤](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/22)。其前置未闭合时仅推进独立研究；本窗口不自动启动下一票。
