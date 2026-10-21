# 完整周期计时实现与定向验证

范围为 #182 的完整周期边界、统一读取检查及统计文件容量。实现基于
`b028962`，使用独立 `codex/static-capacity` worktree；原始数据库、历史报告、
19 组编码审计和三卷验证原件均保留。

## 记录与评价

SQLite cycle 保留基础 `duration_s`。controller 使用只读
`RunRecorder.last_sequence` 登记该条 cycle，完成时间覆盖同步 cycle 提交及
该次准入拒绝后的恢复处理、transition 提交。正常准入、无候选和 FAULT
在对应分支完成后登记；计算异常、计算中终止及等待准入终止的待完成记录，
在 worker 的故障处理和 terminal event 提交后统一完成。

内存表以现有 `maximum_candidate_records` 限制数量，原子写入
`controller.recording.json` 的 `cycle_timing` 字段；版本为 1，边界为
`controller_after_synchronous_records_and_recovery_v1`。manifest 声明边界，
result 绑定统计文件 SHA-256。退出统计序列化和 checkpoint 使用独立存储额度。

共享 `load_cycle_timing` 检查版本、边界、SHA、实际 SQLite sequence 的一一
对应、有限数值、起止时间与 duration，以及完整耗时不小于基础耗时。
evaluator、plot、共同周期及 freeze 使用相同来源。周期恢复和正式冻结均
拒绝历史边界，冻结时重新核对全部 20 项摘要与原件。

历史格式 0、2、3 保持读取能力。基础计时数值继续可见；重新评价的 runtime
为 `insufficient_evidence`，物理和任务指标独立保留，历史文件不改写。
已声明完整边界的缺项、非法表或不匹配身份直接报错。真实 SIGKILL 验证保留
已提交 cycle，同时拒绝将缺少完成表的运行用于当前完整周期准入。

## 统计文件容量

每条完成记录含一个 SQLite 正整数 sequence（最多 19 位）和三个有限
float64，JSON 数值文本每项不超过 25 字节。固定键名、分隔符及嵌套缩进
纳入每条 256 字节额度；其余固定统计字段和容器预留 65,536 字节。
序列化后、写入前执行实际 UTF-8 字节和文件系统分配单位检查，超过范围
即失败并保留原始数据库。pending 文件写入完成后检查实际 allocation，
原子发布后检查包含该文件的收尾增加量。

| 周期 | 最大完成记录数 | controller 字节额度 | plant 字节额度 |
| --- | ---: | ---: | ---: |
| 100 ms | 4,732 | 1,277,952 | 65,536 |
| 20 ms | 23,652 | 6,123,520 | 65,536 |

工程模型将完整额度显式加入 `other_retained_bytes` 和 writer 的
`completion_base_bytes`。100 ms 下固定 1,080 项的新统计额度合计
1,450,967,040 字节，已有 5% 编码裕量、WAL、checkpoint 和每卷保留空间
保持。20 ms 最大表实际序列化 4,343,574 字节，ext4 分配 4,345,856 字节，
预留 6,123,520 字节。内存表数量与相同生命周期上限一致。

## 实际验证

环境入口为主仓库 `.scratch/host-recovery/env.sh`，PYTHONPATH 指向本 worktree
的 `src`、`portable_oscbf` 和主仓库既有 native 依赖。纯 SQLite/逻辑检查
使用 CPU 16–21，真实 ROS 定向检查使用 CPU 0–15、domain 231；旧共同周期
服务已退出。未执行完整性能或正式矩阵。

日志共同前缀：
`/home/lsn/robot/robot_safecontrol/.scratch/worktrees/static-capacity/.scratch/`。

| 日志 | 实际结果 |
| --- | --- |
| `cycle-timing-red-01.log` | 新共享计时模块尚不存在时失败 |
| `cycle-timing-red-02.log` | 真实失败 future 缺少完成表时失败 |
| `cycle-timing-red-03.log` | 评价缺少显式计时边界时失败 |
| `cycle-timing-red-04.log` | 历史周期数据进入新准入时失败 |
| `cycle-timing-red-05.log` | 统计容量接口尚不存在时失败 |
| `cycle-timing-red-07.log` | 旧周期报告未在冻结入口拒绝时失败 |
| `cycle-timing-red-08.log` | 历史报告按需重算不能保持读取时失败 |
| `cycle-timing-red-12.log` | 统计发布异常覆盖原始错误时失败 |
| `cycle-timing-green-13.log` | 50 项通过：逻辑、容量、严格完成表、周期、冻结和记录检查 |
| `cycle-timing-ros-09.log` | 3 项通过：真实计算中终止、等待准入终止、实际 plant late_command 恢复记录 |
| `cycle-timing-bounds-10.log` | 2 项通过：23,652 项实际文件、超出预留空间时保留数据库 |
| `cycle-timing-consumers-11.log` | 1 项通过：实际运动评价、历史重算、完整计时身份、格式转换及逐值一致 |
| `cycle-timing-sigkill-14.log` | 1 项通过：实际 SIGKILL、WAL 恢复、缺少完成表拒绝准入 |
| `cycle-timing-cleanup-17.log` | 8 项通过：既有 worker 异常收尾和本次完整计时终止路径 |
| `cycle-timing-budget-16.log` | 复用 19 组审计，当前源码生成全部 20 项预算，命令退出 0 |

预算原件在 `.scratch/cycle-timing-budget-16/`，属于开发验证；正式源码集成后
需要用公共 `scripts/prepare_static_storage_budget.py` 重新绑定身份。
13 个受影响 Python 模块 `py_compile` 及 `git diff --check` 通过。

## 来源与接续

交付证据目录：
`/home/lsn/robot/robot_safecontrol/output/static-capacity-20261020/complete-cycle-timing-02/`。

- `verification.json`：124 项实际运行来源、日志 SHA-256、20 项预算身份和
  统计/收尾额度一致检查；SHA-256
  `dfee810b915d0cd8325aff37e656ab148f88c0663771c2508ba92f2044e71c6b`。
- `sources.zip`：对应运行源码和 native 文件，内部路径与来源映射保存在上述
  JSON；SHA-256 `97a1f2bbe398ffc876d5be07fd9ddf38df4972546bd55a904c512276b39254f1`。

本次改变 recording 模块和执行节点来源，原三卷 FS 与生命周期证据保持原始
身份；未扩大 `execution_source_compatibility` 白名单。集成后由主会话完成
三卷记录/恢复验证、各卷完整生命周期、S4 和 420 秒写入、预算耗尽，以及
启用当前守卫和完整计时的共同周期测量。工程模型和现场余量据新来源重新
生成，当前定向验证不解除正式准入。
