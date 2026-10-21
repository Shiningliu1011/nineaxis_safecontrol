# 完整周期计时边界

2026-10-21，适用 #182 与 #171 的执行、周期和同步记录要求。
本文件提供实现设计及验证要求；源码实施由 storage_capacity agent 负责。
研究分支只增加文档，当前没有执行新的性能测试。

## 判断与测量定义

规格要求完整周期包含几何、恢复判断、求解、命令检查及同步记录。
源码 `40d43bb` 的 `_record_cycle()` 在 cycle 自身 append 前取得终点；
`_on_admission()` 在该调用之后执行拒绝恢复与 transition 写入。
这两部分都应进入完整周期，当前基础 duration 尚未完整覆盖规格要求。
现有运行及基础 duration 全部保留，并明确标注对应的计时边界。

完整周期起点沿用该次 controller callback 的 `started`；终点取在该周期的
状态处理和全部同步记录完成后。终点时间与完整 duration 先保存在 controller
内存中，退出时写入已有 `controller.recording.json`。统计文件的序列化与写入
属于运行收尾，不再次进入已经完成的每个周期，从而保持有限的测量边界。

## 数据与完成位置

保留 SQLite cycle 行的现有值；每条补充记录包含真实 SQLite cycle sequence、
完整完成 monotonic 时间、从原始 started 计算的完整 duration。
sidecar 顶层声明 `schema_version` 和固定 `boundary` 标识。
cycle sequence 必须来自实际 INSERT；可为 RunRecorder 增加只读
`last_sequence` 接口。不能使用可为空的 `command_id` 作为唯一键。
RunRecorder.append 的现有返回值可辅助核对同步写入耗时，最终 duration
直接使用实际终点减去原始起点，保持所有中间工作和等待时间。

| 分支 | 完成位置 |
| --- | --- |
| plant 接受命令 | controller 写完 admission、cycle 后 |
| plant 拒绝命令 | controller 写完 admission、cycle，完成 `admission_rejected()`、`_record_transitions()` 后 |
| 无合格候选 | candidate、transition、rejection、cycle 全部写完后 |
| recovery 进入 FAULT | candidate、transition、cycle 写完且执行状态更新后 |
| `_finish_calculation` 收到 stop_reason | candidate、transition、rejection、cycle 写完；如果随后需要收尾 FAULT，保持 pending |
| SolverFailure 或其他 future 异常 | 保持 pending，直到异常处理的 FAULT transition 和最终 event 完成 |
| 计算中终止、等待 admission 时终止 | 记录真实已开始工作和 cycle，保持 pending，直到收尾 FAULT transition 与最终 event 完成 |

`finish_worker_recording()` 已负责 node 收尾和最终 event。它应在
`recorder.append("event", ...)` 完成后，将仍 pending 的 cycle 登记完成，
再写 sidecar。正常已经完成的周期不重复延长。清理过程中出现新异常时继续保留
真实错误；没有成功写入 cycle 的计算不能生成虚构 sequence。
若无法取得完整补充记录，则完整计时证据不足。

sidecar 通过现有 `result.json.recording_statistics_sha256` 绑定。
采用同目录 pending 文件及原子 rename，完整写入后才生成结果身份。
SIGKILL、写入异常或部分完成不会产生可用于新版周期准入的完整计时结论。

## 统一读取与来源核查

增加独立的共享读取入口，返回样本与明确的计时版本/边界，集中执行以下检查：

- 核对 manifest 声明、sidecar schema/boundary 和 result 中的 SHA-256。
- 保留 `iter_records` 的真实 sequence；当前 evaluator 的 `_records` 仅保留
  values，不能据此建立补充记录关联。
- 每条 SQLite cycle 恰好有一条补充记录；拒绝缺失、重复、多余和无对应 cycle
  的 sequence。检查 SQLite cycle 数量与 sidecar 数量相同。
- 时间必须有限，duration 非负，完成时间不得早于基础 cycle 时间，完整 duration
  不得短于基础 duration；结合原始起点检查两者计算一致。
- 旧记录保留基础测量与边界说明。声明新版且证据损坏时直接报告错误；缺少完整
  测量时不得把基础 duration 充当新版完整 duration。

evaluator、图表与周期选择均使用这个入口。具体消费者包括：

- `static_execution_evaluation.evaluate_execution()` 的全部 timing 指标。
- `static_experiment_reporting` 的 cycle duration 图。
- `static_period_measurement` 的新增运行、恢复已有运行、已有结果完整性检查和
  最终 select_period 输入。已保存结果也必须具有要求的 boundary。
- `static_experiment_provenance.validate_run_report()` 对实际 sidecar 身份和
  报告计时边界的核查。仅核对 result 文件 SHA-256 无法验证被引用的 sidecar。
- compact、重放和来源归档的完整身份传递。当前 compact 保留记录 sequence
  并复制 sidecar，应验证转换后得到相同完整 duration。

报告分别显示计时边界和来源身份；缺少新版完整计时时，runtime 使用
`insufficient_evidence`，功能指标仍按已有真实证据独立评价。
共同周期准入只接收同一明确完整边界的测量。
历史物理与功能证据继续可读。原始数据库、sidecar、报告及其 SHA-256 保持原状；
新口径的重算结果保存到独立输出，旧基础样本和统计使用明确的 legacy 边界。
兼容读取不能把不完整计时转换成新版准入证据。

## 容量影响

补充数组数量与最大 cycle 数使用同一生命周期上限。420 秒 timeout、2 秒
settle 对应 100 ms 下最多 4,732 项、20 ms 下最多 23,652 项。
内存数组应同样受这个上限约束。

工程容量当前 `other_retained_bytes` 来自历史实际观测及裕量，需要显式增加
补充数组的序列化与文件分配预算，并同步更新 `other_bytes`、收尾留量和
`retained_run_bytes`。使用最终 JSON schema、有限 float64 表示和有界序号
长度计算字段容量，按目标文件系统实际分配单位计入；不得按短 S5 样本外推。
数据库/WAL 的记录流与字段保持原有数量；sidecar 不追加 SQLite 计时记录。

现有 `RecordingStorageGuard.measure()` 排除 `.pending` 文件。因此必须在
原子写入前为完整 sidecar 预留空间，并在写完后核查实际文件分配与可用空间。
如存在旧文件替换，还要覆盖新旧文件同时存在期间的空间。周期测量和正式运行
使用重新绑定源码、配置与卷身份的预算，既有数据和预算证据保留。

## 必要验证

使用真实 RunRecorder、SQLite、controller/plant、Ruckig 与现有数值控制；
不使用 mock 或人为等待替代实际写入。

1. 真实周期完成后，补充 duration 包含自身 cycle append；对应的
   sequence、起点、终点和基础 duration 关系成立。
2. 实际 plant 通过已过期 start_step 产生 `late_command`，controller 完成
   拒绝恢复和 transition 后才登记终点。检查真实 transition 及关联。
3. 真实 future 异常、计算中停止、等待 admission 时停止，分别覆盖
   `finish_recording()` 和 `finish_worker_recording()`，每个实际 cycle
   恰好对应一条完整补充记录，FAULT transition 与终止 event 计入未完成周期。
4. 对已经生成的真实记录副本删除补充项、重复项、修改 SHA、制造不一致时间，
   验证严格读取拒绝；SIGKILL 保留数据库前缀但缺少完整 sidecar 时不能通过新版准入。
5. evaluator、图表输入及 period 的样本数、P99.9、最大值和超期次数一致；
   使用真实 compact/重放核查记录 sequence、完整 duration 与身份传递。
6. 序列化最大项数的 schema 边界测试，并用实际 sidecar 核对占用空间；真实
   容量守卫终止仍能在预留空间内写出补充记录、错误原因及最终身份。
7. 相关测试通过后运行完整 D/S0、D/S5，核查功能及完整生命周期；使用完成后的
   同一源码和计时边界重新执行共同周期测量。全部失败原件保持可查。

本设计只修订测量及其完整性、容量和消费入口，不修改求解、几何或安全参数。
真实 seq179 的性能原因仍待完成统一计时后的实际复现证据判定。
