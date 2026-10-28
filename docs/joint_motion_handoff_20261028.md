# 关节动作实验问题与跨设备交接

## 接续位置

实验日期为 2026-10-28，设备为 `lsn`，仓库为 `Shiningliu1011/nineaxis_safecontrol`，工作分支为 `main`。代码与真实回放数据基点为 `4c98a64d9bfc474e0e2efd253df642a4bc48b4f4`。本次同步保存现场问题、用户确认的模型坐标定义与修复验收要求；生产代码和模型保持该基点的内容。

相关任务为[实现 M11 真实设备协议与执行保护](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/164)和[实现 M02/M07 共用模型、导数与约束](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/153)。完整现场记录见[单关节执行说明](single_joint_execution.md)，方向与传动定义见[J1 运动规格](planning/nineaxis-full-project/j1-motion-spec.md)。

另一台设备按[双设备工作流程](two_machine_workflow.md)检查已有改动、获取远程状态并接续 `main`。保持用户未提交改动，使用当前设备的实际依赖环境，不复制其他会话的进程或硬件授权状态。

## 用户确认的 J1 模型坐标

用户原话：“从原先 585 mm 的地方往回倒 500 mm 才是关节1起点”。

- 新起点位于原 URDF 坐标 `85 mm`，新行程为 `0–500 mm`，对应原坐标 `85–585 mm`。
- 坐标关系为 `q_old_m = 0.085 + q_new_m`。新坐标 `0`、`250`、`500 mm` 分别对应原坐标 `85`、`335`、`585 mm`。
- 当前 J1 轴为 `base_link` 的 `+Z`，`rpy="0 0 0"`；模型修改目标为 `origin xyz="0 0 0.085"`、`axis xyz="0 0 1"`、`lower="0"`、`upper="0.5"`。
- 当前源码中的 URDF 仍为 `origin xyz="0 0 0"`、`upper="0.585"`。此次交接没有修改模型。
- `85 mm` 是模型固定位置修正。现场电机零位、`sign = -1`、输出轴直连与 `5 mm/rev` 的传动定义继续有效；电机相对零位 `−360°` 对应向前 `5 mm`，设备限位为相对零位 `−36000°–0°`。

修改时核对全部模型消费者、派生文件、模型 hash、标定适用身份与已有参考资产。以相同物理位置验证新旧 FK、TCP 与碰撞几何，保存原始实验的代码及坐标来源。

## 今日实验问题

1. **J1 向前 50 mm 中止，停车观察超时。** 指令为 `0.25 mm/s`、`0.05 mm/s²`，保护上限为 `0.3 mm/s`。最后编码器换算位移为 `27.685030495 mm`。触发时两帧变化 `0.526245117°`，接收间隔 `21.854477 ms`，允许变化 `0.482056703°`；请求间隔为 `27.140787 ms`，两次事务耗时分别为 `6.034760 ms` 和 `0.748450 ms`。相邻反馈计算速度为 `4.013252 rpm`，同期位置窗口速度为 `3.216849 rpm`、平均反馈速度为 `3.113891 rpm`，后两项小于保护上限 `3.6 rpm`。设备没有采样时间戳，原因尚未确定。

2. **J1 保持观察中的速度与位置证据需要核查。** 保持阶段约 `7.908 s`、共 `360` 条反馈，位置容差与 `traj_done` 均满足要求，速度条件有 `268` 次不满足。全部静止条件最长连续满足约 `0.330 s`，要求为 `0.5 s`。末次平均速度为 `0.050444 rpm`，静止阈值为 `0.02 rpm`；最后一秒编码器位置范围为 `0.005371°`。最终记录为 `external_stop_required / hold_observation_timeout`，用户随后确认已操作急停且 J1 已停止。

3. **J2 首次速度保护及异常保持失败。** 指令为 `0.3°/s`，约 `0.178 s` 的位置变化计算速度为 `1.210°/s`，超过当时 `1.2°/s` 的保护上限，终止原因为 `velocity_limit`；异常保持同样被速度检查拒绝。后来用户指定保护上限为 `2.5°/s`，后续绝对目标 `+10°` 已完成。最初速度差异的原因仍未查明，原始记录按原参数保存。

4. **J5 的 +90° 目标未完成。** 指令从 `0.3°/s` 提高到协议可表示的 `2.46°/s` 后，触发 `feedback_position_jump`。两帧间隔 `0.022020839 s`，位置变化 `0.06581878662109375°`，允许变化 `0.06505209750026551°`；位置窗口速度为 `2.4456135624°/s`。保持观察通过，独立复测为 `+68.476000°`，故障码均为零。

5. **J7 的 +60° 目标未完成。** 指令为 `2.46°/s`，触发 `feedback_position_jump`。两帧间隔 `0.022115486 s`，位置变化 `0.06929636001586914°`，允许变化 `0.06528871500018794°`；位置窗口速度为 `1.5484792657°/s`。保持观察通过，独立复测为 `+2.993411°`，故障码均为零。

6. **J1 断电恢复尚未验证。** 限位 `−36000°–0°` 与开关 `1` 已写入、回读并发送保存命令；重新上电后的限位、零点和多圈计数仍需按设备版本验证。编码器换算位移尚无独立位移测量证据。

J2 后续复测以及 J3、J4、J6、J8、J9 的 `+10°` 目标均已完成，并通过持续静止及独立反馈检查。J4 在 2026-10-21 发生的通信中断今天没有重现，原因仍未查明。

## 修复入口与验收

- 模型与传动：[URDF](../models/ninezzhou/urdf/ninezzhou.urdf)、[J1 传动配置](../config/j1_transmission.yaml)、[零位配置](../config/hardware_joint_zero.yaml)、[J1 执行接口](../src/robot_safecontrol_moveit/j1_execution.py)。按已确认的坐标映射完成模型修改，并验证模型、规划与执行使用的行程一致。
- 反馈时序：[SingleJointMonitor](../src/robot_safecontrol_moveit/single_joint_execution.py) 当前按主机接收间隔计算相邻位置变化上限。核实设备数据更新方式、采样年龄及通信延迟，在有测量依据的时间定义下处理位置与速度。原始记录目前只能确认保护触发条件。
- 速度与停车：[VelocityWindow、checked_speed、ArrivalWindow](../src/robot_safecontrol_moveit/single_joint_execution.py) 共同参与运动和静止判断。采集并比较真实静止噪声、驱动速度估计、位置变化和独立物理测量，分别确定 J1 与转动关节的可验证判据；位置稳定与 `traj_done` 不能单独证明物理停车。
- 异常处理：核查正常运动准入与受控停止请求各自需要的反馈条件，复现 J2 保持请求被拒绝的路径。停止方式、减速度及失联保护需要厂家依据和现场测量；主机停止发送不能保证电机停止。
- 参数与通信：[运动参数校验](../src/robot_safecontrol_moveit/motor_motion_limits.py)、[串口实现](../src/robot_safecontrol_moveit/drempower_serial.py)、[CLI](../src/robot_safecontrol_moveit/motor_cli.py)。保留 `0.01 rpm` 协议精度及参数预览与实际编码一致性。核查驱动内部轨迹目标与真实反馈，按测量结果确定指令速度余量。J4 的历史通信问题继续按原始收发记录核查。
- 所有修复保留实际超速、反馈中断、错误身份、过期数据、运动越界与无法确认停车时的拒绝检查。旧记录按当次参数回放，阈值和时限的改变需要明确依据与新验证。

模型验收至少覆盖新坐标 `0`、`250`、`500 mm` 与原模型对应位置的一致性、共同模型消费者、安装资源和模型身份。反馈修复需要覆盖上述真实故障、真实正常动作、真实静止记录及拒绝条件，并区分离线软件验证与实机验收。

## 已有验证与数据位置

基点 `4c98a64` 在设备 `lsn` 执行以下检查，结果为 `168 passed in 3.95s`，退出码为 `0`。JUnit 保存在本机 `.scratch/motor-submit-20261028/pytest.xml`，该报告没有随 Git 同步。

```bash
PYTHONPATH=src:.scratch/motor_tools/deps python3 -m pytest \
  tests/test_j1_execution.py tests/test_single_joint_execution.py \
  tests/test_motor_connection.py tests/test_drempower_joint_control.py \
  tests/test_unit_conversion.py tests/test_drempower_can.py \
  -q -p no:cacheprovider \
  --basetemp=.scratch/motor-submit-20261028/tests \
  --junitxml=.scratch/motor-submit-20261028/pytest.xml
```

另一台设备使用本地依赖及独立的输出目录运行，核实测试不连接设备。当前通过结果证明既有软件检查与真实中止回放保持预期结果。

已随 Git 同步的 J1 真实数据共两份，合计 `265187` 字节：

- [J1 中止与保持观察数据](../tests/data/j1_motion_abort_20261028.json)，SHA-256 为 `6726465bf5b9dc4d31d38a45fbfd4e61a472f43748f5cb9b442cbe68f511475c`。包含两条触发反馈、对应原始帧和全部保持观察记录。
- [J1 静止数据](../tests/data/j1_static_feedback_20261028.json)，SHA-256 为 `40291220f253482525320b5aeff3726ce3cb04410b90207b124bebe9deba6dc4`。

完整原始日志与收发帧仍保存在设备 `lsn`，由 Git 忽略。继续精确核查 J2、J5、J7 或 J1 完整通信时，需要单独取得对应文件并核对 SHA-256：

- `.scratch/j1-motion/forward50_20261028T193051.jsonl` 及对应 `.frames.json`、`.abort_audit.json`。JSONL SHA-256 为 `1dcbf39d406c8da20a19c08276b68ace4d54989bfec0c01764e6db3b3ba587f4`。
- `.scratch/motor_tools/J2_direction_target4_20261028.jsonl` 及对应 `.frames.json`，JSONL SHA-256 为 `169fece121f67e93e3635b297f0c118c304e9d121f5d0f1da2965fb5d0e6ce8f`。
- `.scratch/motor_tools/J5_absolute90_speed246_20261028.jsonl` 及对应 `.frames.json`、`J5_speed246_audit_20261028.json`，JSONL SHA-256 为 `43caa241ddc5d15cdc3a547ce8cf047815aad990626c81c54a365922aa33effb`。
- `.scratch/motor_tools/J7_absolute60_speed246_20261028.jsonl` 及对应 `.frames.json`、`J7_speed246_audit_20261028.json`，JSONL SHA-256 为 `d52da81766d9cad62805deb9ac17a8f35d4fe2d41b112e87d2feb41bcb71627a`。

厂家资料与本地 RAG 位于设备 `lsn` 的 `大然电机/`，同样由 Git 忽略。另一台设备缺少原件时应使用可核实的厂家原始资料，并记录版本及引用。

## 恢复边界

现场已确认 J1 急停后停止；本机 `.scratch/motor_control/stop.json` 保存 `feedback_position_jump` 和人工审查要求，该锁定文件不会随 Git 迁移。另一台设备缺少该文件不代表获得运动许可。接续工作处理本地模型、代码、测试与文档；实机动作、限位写入、零位设置、清错及解除运动锁定均按新的具体授权执行，ROS `live` 保持关闭。
