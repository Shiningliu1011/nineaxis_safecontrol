# B/S4 完整运行与独立核查

原件位于主仓库
`output/static-method-evidence-20261020/b-s4-current-01/`。
运行来源为 `e6fe012ea3eb77ad1adaae5865a4d0d365e522af`，
使用 `development-scenes-06/s4.yaml` 和
`s4-paired-independent-witness-01/witness`，共同构型辅助上限为0.02。
运行使用 CPU 8–15、ROS domain 200，存在其他开发计算；
周期记录用于开发观察。

## 完整结果

controller/plant 退出码为 `[0,0]`，完整跟随、S4构型调整、
功能、运行时间和总体验收均通过。88,062 个有效反馈覆盖全部路径，
40个外部组合及36对自碰撞均通过。终点保持3.584秒，
位置误差0.001961 mm，工具轴角度误差 `2.657994e-6 rad`。
路径距离 RMS/P95/最大值为0.054661/0.116056/0.174758 mm。

1,739个周期的 P99.9 为61.641395 ms，最大为72.781219 ms；
超期和准入拒绝均为0。一次周期因 `no_certified_candidate` 未生成命令，
完整数据保留。八份原件的来源、身份和数据库完整性核查结果保存在
`commands/b-s4-current-01-audit.json`。

## 真实 QP 核查

在 CPU 16–25 使用实际记录的 P/c/G/h 和 solution，
逐项重算全部1,775个 QP 的限制，其中325个问题含负上界；
最大限制超差为 `5.651403824080692e-18`。

选择开始、中段、终点、最大KKT残差和首次负上界对应的5个代表问题，
使用独立 SciPy SLSQP 核查相同目标与全部限制。
全部求解成功，独立限制超差和目标值差均为0。
实际输入、原始解与独立解保存于
`commands/b-s4-current-01-solver.json`，脚本为
`commands/verify_b_s4_qp.py`。该脚本SHA为
`7f98bc9a2201a1e05b6e5350a87e311048d5f28a3b48e46732010cca2a1db391`。
核查前后 controller 数据库 SHA 相同。
该项读取既有数据，没有重新执行 ROS。

## 恢复事件

原始 transition 在116.5秒记录
`RUNNING → REFERENCE_PAUSED`，原因为 `candidate_rejected`。
116.6秒首次得到有效正向候选并恢复，基础恢复延迟为0秒。
173.3秒进入 `ENDPOINT_HOLD`。

实际反馈中的暂停区间为116.502至116.600秒，持续0.098秒。
参考进度保持在1.218374590 m，变化范围为0；
J1位置变化0.061730 mm，J8角度变化0.002210460 rad。
恢复后的最大参考位置误差为0.496407 mm，
最大真实工具轴角度误差为 `0.001122085 rad`。
本次实际恢复次数为1，没有FAULT。
`commands/b-s4-current-01-events.json` 保存原始转换和被拒周期，
反馈报告保存全部暂停区间、关节变化和恢复后误差。

## 身份与范围

本次独立检查在集成 `3b3b216` 后执行，核查对象仍为上述 e6fe012 原始数据。
源码与环境归档复用主仓库
`output/static-capacity-20261020/ntfs-s5-E-02-source/sources.zip`，
SHA-256 为 `d9965035536bb13a8561f1fa7e9ad6fdbe0ab5d178cc9250df702e175fda9c28`。
本次证据支持B的固定场景基础恢复与完整跟随；最终共同周期另行测量。

| 文件 | SHA-256 |
| --- | --- |
| `controller.sqlite3` | `f6f889ee4466b8ddb2eba5ac151f5797ee74a712d770c258015fe2752c6731df` |
| `feedback_report.json` | `7510bec365dfa5160c57ee15bc073d81a940b90ed6903d351485f632d7fdc570` |
| `commands/b-s4-current-01-audit.json` | `2e1b314f5d455e00807c055640aa190444d677dd6b06d4584fbe12334460cb97` |
| `commands/b-s4-current-01-solver.json` | `832db0c35e063b5a9d72d11475e1fe75407de787b7db9530531c9ad2021b6f9d` |
| `commands/b-s4-current-01-events.json` | `fb66374c4f09a4d85bd424e576033d6e6a2210b47048b0dafc03854f057f21a8` |
