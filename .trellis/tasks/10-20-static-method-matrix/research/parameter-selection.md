# 开发参数选择

参数只由开发场景确定，正式实验尚未开始。
各次运行的最终值及来源保存在对应 `manifest.json`。

## 已测量范围

- 任务误差增益：位置 40、方向 20 s⁻¹，以及五个分量统一 8 s⁻¹。
- 控制周期：已完成 20 ms 与 100 ms 的开发运行；完整方法矩阵同时测量
  20、40、50、100 ms，选择依据为全部方法完整周期 P99.9 ≤ 周期 80%。
- 名义进给：共同开发倍率为生产配置进给的 0.05；正式组合另乘以 0.5、1.0、1.5。
- 位置与方向权重分别为 10,000 与 1,000；构型辅助权重 0.01，
  平滑权重 0.02，进给权重 100。这些权重保持各组一致，未进行额外搜索。
- 共同控制间距余量测试 0 与 0.1 mm；其约束始终包含原始间距要求。
  约束使用 `-gradient @ velocity <= alpha * (distance - required - reserve)`，
  低于控制余量时要求恢复距离，处理后的连续运动仍须满足物理间距要求。
- 关节余量使用连续有界的对数型梯度，`posture_joint_margin_fraction=0.05`，
  所有方法共同使用。该参数的完整场景验证正在进行。
- 共同构型辅助速度上限测量 0.1、0.05、0.02、0.01、0.005。
  在 `s4-joint-development-09` 的实际受阻位置，0.02 配合 0.1 mm 控制余量，
  正向候选及零进给候选分别通过 42、12 个区间的连续证明。
  单独控制计算为 33.594 ms，完整场景与周期测量仍须验证。

## 当前开发配置

五维增益统一为 8 s⁻¹，在 100 ms 开发周期下每周期的误差校正系数为 0.8。
实际 D/S0 和 E/S0 已完成完整路径，路径距离 RMS 分别为
0.058782 mm 和 0.059183 mm，真实工具轴角度 RMS 分别为
0.008290° 和 0.008357°。几何、执行器限制与参考结算同时通过。
这两项证据用于选择当前增益；共同周期仍依据完整测量确定。

构型辅助通过相同五维零空间，归一化关节行程余量及距要求间距 80 mm
以内的对象共同形成辅助速度。单轴辅助速度上限为 0.02，并受实际执行器上限限制。
归一化位置 z 与范围宽度 w 对应的关节项为
`-z / (w * (1 - z² + 0.05²))`。

暂停阈值为位置 1 mm、工具轴 0.2°；恢复阈值为位置 0.3 mm、
工具轴 0.04°，持续确认时间 0.2 s，连续暂停上限 30 s。
共同进给增长率为 0.03 m/s²。Ruckig 最短运动时间与 jerk 时间均为 80 ms，
500 Hz 仿真积分及全部执行器限制来自实际 profile。

## 来源

- `output/static-experiments/s0-common-control-01/manifest.json`
- `output/static-experiments/s0-common-control-02/manifest.json`
- `output/static-experiments/s0-common-control-03/manifest.json`
- `output/static-experiments/s0-joint-full-geometry-01/feedback_report.json`
- `output/static-experiments/s0-sns-full-geometry-01/feedback_report.json`
- `output/static-experiments/s4-joint-development-06/feedback_report.json`：
  93,447 条反馈，路径距离 RMS 0.048498 mm、最大 0.183984 mm，
  外部及自碰撞检查通过；路径完成 66.9% 后持续暂停终止。
  末态 J1 达到上限、J9 达到下限。该运行用于判断关节余量作用的适用性，
  保留为未完成运行。

正式清单必须保存本说明、完整周期报告、最终配置及源文件身份。
开发配置中的 100 ms 不构成正式周期已经选定的结论。
