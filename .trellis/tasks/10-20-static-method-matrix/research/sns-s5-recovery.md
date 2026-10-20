# E/S5 非零恢复速度验证

## 原始运行与独立可行性

原始运行位于
`/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/lifecycle-s5-E-01/run`。
controller sequence 155 保存实际 `sns_failure` 的完整 J、desired、G、lower、h，
该次零进给求解在 `generalized_sns` 最终限制检查抛出异常。
原始结果为 FAULT、worker exit codes `[0, 1]`，失败记录完整保留。

诊断使用 CPU 16–25，全部输出位于
`/home/lsn/robot/robot_safecontrol/output/static-method-evidence-20261020/` 下的
`e-s5-sns-diagnosis-01/`、`e-s5-sns-diagnosis-02/` 和 `commands/`。
两个诊断目录保存独立数据库副本、实际 `problem.npz`、HiGHS 解及逐次饱和记录。
原始 manifest、两个数据库、反馈报告和结果文件在检查前后 SHA-256 相同。

HiGHS 使用原始上下界与 `Jv = s·desired`，求得最大缩放 `s=1`，
全部限制超差为 0，完整任务残差 `3.292863762404205e-15`。
独立零任务与仅限制问题也可行。

首次未饱和解对应第 65 行的允许缩放区间为
`[4.215076738022197, 42777.10391144128]`，与算法允许的 `[0, 1]` 没有交集。
该行上界为 `-0.0002565176663531783`，要求主动增加间距。
缩放区间检查需要保留这条限制最小缩放量的约束，供下一次饱和计算使用。

## 作者语义与实现

已重新完整读取固定版本作者 `GSNS_VL.m`、`FindScaleFactor.m` 和 `damped_pinv.m`。
作者实现对缩放为 1 但仍违反广义限制的对象设置缩放 0，
继续将该对象加入饱和矩阵。
Python 实现同时记录缩放区间的最小值约束；没有最大值约束可选时，
返回该最小值约束的编号，继续原有饱和过程。
任务整体缩放、SVD 阻尼、秩检查及全部最终限制保持相同。

真实输入完成两次迭代，饱和索引为 `(65,)`，任务缩放为 1，
全部限制超差为 0。饱和后最小奇异值为 `0.056497490158179384`，
作者参数得到阻尼平方 `6.808033605826424e-7`。
完整任务残差为 `4.8333549882726293e-8`。
使用独立的受约束阻尼正规方程求解，九轴速度与 SNS 的最大差为
`3.0135896761981223e-13`，并核对完整任务残差。

## 验证与剩余工作

真实输入加入 `tests/data/static_sns_s5_recovery.npz`。
回归通过公开 `generalized_sns` 检查原始约束及其等价反向表示，
与 HiGHS 最大缩放和独立阻尼方程比较。
既有解析缩放、S1 实际恢复、零任务、完全阻塞、不可行限制拒绝、
构型辅助及 E 组实际控制推进和终点区间均通过。
共 14 passed，26.15 s；命令为：

```bash
python -m pytest tests/test_static_sns.py \
  'tests/test_static_control.py::test_method_matrix_uses_common_processed_motion_and_advances_from_feedback[E]' \
  'tests/test_static_control.py::test_last_reference_interval_is_executed_before_endpoint_hold[E]' \
  -q --basetemp=.scratch/sns-s5-regression
```

TDD 失败日志为 `commands/sns-s5-red.log`，通过日志为
`commands/sns-s5-regression.log`。重放脚本保存为
`commands/diagnose_sns_failure.py`；输出目录参数必须尚不存在。
完整 E/S5 闭环复验及其全部实际 SNS 数值核查通过，详见下一节；
最终集成全量检查由主会话统一执行。

## 完整 E/S5 闭环

主会话使用修复来源 `e6fe012ea3eb77ad1adaae5865a4d0d365e522af`
完成完整运行，原件位于
`/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/lifecycle-s5-E-02/run`。
CPU 8–15、ROS domain200、周期100 ms、共同构型辅助上限0.02，
同期存在其他开发验证。

controller/plant 退出码为 `[0,0]`，生命周期、功能、运行时间及总体验收通过，
按 S5 的持续阻塞要求以 `pause_timeout` 安全终止。
347个周期的 P99.9 为55.320026 ms，最大58.329657 ms；
没有超期或准入拒绝。来源和原件八项SHA核查通过。

独立核对全部2,826个实际SNS结果，其中2,745个含负上界、2,787个发生饱和。
最大限制超差为 `6.106226635438361e-16`，
最大完整任务残差为 `2.316538233458993e-5`，
最小任务缩放为1。6个代表问题的独立 HiGHS 最大缩放均为1；
独立 null-space 基底和阻尼正规方程与实际任务的最大差为
`1.5954692804723914e-15`。全部原始限制和作者DLS参数保持相同。

核查结果位于主仓库
`output/static-method-evidence-20261020/commands/`。
`e-s5-current-02-audit.json` SHA为
`b21247183ba66cf84ee4bfd694102bc059ffd453cfd92874ac8d9183924d6775`，
`e-s5-current-02-solver-dls.json` SHA为
`4142b7068b236a2366dce13574bbbf420cada9ae73319f9dbcf8392a5678bfb7`。
原始controller数据库SHA为
`93b0e99e433f928fac2d6cd84c06e9bc065d216bce3e0ffed3368af44a469a87`，
反馈报告SHA为
`f6bd5d181b27df3520fbe1b2167a52df0d0c068e2867a2e3ddc7a7b75cde98ee`。
核查前后原件保持相同；原 `lifecycle-s5-E-01` 失败记录继续保留。

## 身份

| 文件 | SHA-256 |
|---|---|
| 原始 `manifest.json` | `425ecae6a10f33ade021b58891658891e2bd33b42b382280cf0098ea1c5909ee` |
| 原始 `controller.sqlite3` | `f836185f55a43fb1f948c48336048c08a883b2548f0ccbce58bff346e3bca84b` |
| 原始 `plant.sqlite3` | `f9b2003572d560cff0e53a1975193a3c264b39db81a48c5918d98e8665877d93` |
| 原始 `feedback_report.json` | `b1630f4c6b32e186eee1ef6fdb6d082d2a95621ef6a29a5788e53a5fb9bb62f7` |
| 原始 `result.json` | `d37c37a5ff8b60e394473b5647521df2b7f64afefc90a90857bf28ec67196c3e` |
| `e-s5-sns-diagnosis-01/diagnosis.json` | `30f067366b5b2ba96d033947a3187ac441454fbdfd942f1e243ad8f7edb84932` |
| `e-s5-sns-diagnosis-02/diagnosis.json` | `7541bdb9c7490dc390e8944b9f6d709f8a99f3fc0529c6873090d9a7ff333b7c` |
| `tests/data/static_sns_s5_recovery.npz` | `2f883c669432e31f7dc20fed6a7c57fc8d9b3947deda109210cfef68f241de29` |
| `commands/sns-s5-regression.log` | `a01c393249c3ebe17ec6c096b333c5cd731e912309bb99bae6ea1a996b2af4b0` |
