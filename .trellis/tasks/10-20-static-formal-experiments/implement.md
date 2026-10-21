# 实施与验证

`implement-spec` 管理整个 #171 的实现、子任务、集成及最终审查；Trellis 保存
规划、验证证据与接手记录。集成目录 `/home/lsn/robot/robot_safecontrol`，分支
`codex/static-obstacle-experiments`，环境入口 `.scratch/host-recovery/env.sh`。
全部实验限定 sim，用户已授权相关修改、验证、子 agent、commit 和 push。

## 当前证据

- #172–#181 已关闭，#182、#183 和 #171 保持开放，正式采集0/1080。
- 完整计时修复 `8d022236` 与前置检查 `a6a422a5` 已审查并集成至 `3771bc88`。
  定向验证、来源归档与校验值见 `research/complete-cycle-timing-implementation.md`。
- 集成完整回归04正在执行；主包已出现一项既有50 Hz性能测试失败，尚待最终结果
  与独占复验。日志在 `output/static-capacity-20261020/final-regression-04.log`，
  产物在 `.scratch/regression-xiLHz4Py/`。不能记为通过。
- 源码 `40d43bb` 的回归03通过：主包732 passed/1 skipped，控制内核
  486 passed/34 skipped，CTest4/4。身份见 `research/full-regression-20261021.md`。
- 相同旧源码三卷工程容量通过，需求214189355008字节，可用284842614784字节，
  `formal_bound=false`。当前计时文件额度需要重新核查和绑定。
- S4九组D方法完整运行与独立连续见证通过，新S2三个初态完整ROS运行通过。
  36份完整来源支持39/72物理配置证明，见 `research/formal-proof-index-results.md`。
- 旧周期队列在20 ms的A、B结束后停止，原件及SHA-256见
  `research/period-study-records-01.md`。新边界需要测量全部20组。

## 接续顺序

1. 集成容量公共参数比较和保存数据播放入口，完成相关测试及独立审查。
2. 在最终源码上执行 `research/complete-cycle-preflight.py`，验证三卷真实记录、
   随机读取、中断恢复、原子JSON、D/S5、S0、S4、420秒timeout及预算耗尽。
3. 使用新原件、当前来源和完整统计额度重建三卷工程容量，要求全部检查通过。
4. 新目录测量20/40/50/100 ms × A–E全部20组，按完整周期P99.9选择最短合格周期。
   没有合格候选时保持运行时间验收失败，修复有证据的问题并复验。
5. 固定参数、源码、环境和1080项正式清单，核查登记的72配置身份与39/72证明；
   容量绑定最终清单与周期，全部前置通过后串行采集，保留全部失败原件。
6. 完成配对统计、六类图表、重算和播放验证；执行最终规范与规格独立审查，
   修复并复验，关闭工单，更新接手记录并提交推送。

## 数据位置

主仓库 `output/` 和两个NTFS任务目录全部保留：
`/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/`、
`/media/lsn/Windows/robot_safecontrol_static_experiments_20261021/`。
UUID分别为 `083CBB753CBB5BFE`、`6C001EAA001E7B72`。
数据来源与校验值见 `docs/static_obstacle_experiments_handoff.md` 及所引研究文件。
当前任务保持 `in_progress`，正式采集等待全部前置通过。
