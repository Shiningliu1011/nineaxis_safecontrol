# 运行期间记录空间检查

本项延续工程容量与多卷准入，所有运行限定 sim。原始实验目录保持完整，
新增验证保存到主仓库 `output/static-capacity-20261020/runtime-storage-guards-03/`
和 NTFS 专用目录的同名目录。

## 执行边界

`run_formal` 依据已经验证的工程分组、输出卷及固定运行编号生成
`--storage-budget`。运行入口核对配置、源码和文件系统 UUID，在启动 worker
之前把完整预算写入 manifest。worker 的 ready 和结束统计继续绑定 manifest
及源码身份。

`RunRecorder.check_capacity()` 检查数据库逻辑页空间、WAL 实际分配、
其他保留文件及文件系统可用空间。`RecordingBudget` 在开始新任务前调用该
接口；controller 根据实际待完成周期数量核对收尾额度。每条记录完成后保存
实际原始编码尺寸与 WAL 分配增长，超过配置包络时停止后续新任务。
已开始的候选、失败求解、待完成周期、终止事件和退出统计完整保存。

数据库和 WAL 阈值来自固定次数及逐流编码工程模型。5% 编码裕量保持。
各数据库先按 4096 字节分配单位取整，再计入逐运行总额度。
每卷保留空间扣除两个完整数据库的 checkpoint 空间及其他保留文件额度，
其余分别分配给两个 writer 收尾。每个 writer 的任务收尾计算包含：

- plant：一条反馈、写入延迟、收到命令、两条 admission 和终止事件。
- controller：一项候选计算及其 cycle、command、admission、rejection、
  四条 transition、两类 solver failure 和终止事件。
- 已发布命令对应的待完成 cycle 按实际数量逐项增加所需空间。

单条记录收尾空间使用固定生产 schema：九个关节、五维任务、最多九个
trial，基础约束数量为 `36 + pairs`。QP 最大变量数量为十个，进给上下界
增加两项约束；SNS 的 `G` 为 `(36 + pairs) × 9`。全部数组数字按每项
九个 MessagePack 字节计入，字段名、容器头、certificate、来源信息、
几何接近点和梯度分别计入。名称采用 256 个 UTF-8 字节上限，工程审计
核查全部几何对象名称，正式配置再次检查障碍物名称。

每条记录的 B-tree 路径、相邻页、overflow pages、WAL 页头和 checkpoint
页空间均计入。新任务开始前保留一项在途任务和全部待收尾 cycle 的完整
额度。实际记录超过工程编码范围时，触发记录继续保存，后续任务停止。
该模型声明工程验证范围；无法承诺外部进程同时耗尽整个卷时仍可完成写入。

controller 的新计算起点位于 `_tick` 入口，覆盖容量检查及状态准备；
已经开始的 future 完成时保留其原始起点。反馈维持同步逐条提交。
唯一终止 event 写入前检查其完整编码和写入额度；容量终止时 event 与
worker result 均记录 `budget_exhausted`，已有异常字段保持。

共同周期入口 `benchmark --storage-budget-index` 要求完整二十项预算
文件及其 SHA-256，覆盖四个周期和五种方法。每项实际 run 使用容量守卫，
测量结果绑定 worker 统计身份。正式清单创建时核查该统计，缺少守卫
或预算触发的测量不能进入正式共同周期证据。

## 验证入口

`tests/test_static_recording_storage_budget.py` 使用真实 SQLite 和真实 QP
输入，覆盖 WAL、数据库、文件系统余量、记录结构尺寸及待完成周期空间。
预算触发后检查完整候选、cycle、终止事件与 float64 位模式。
真实 ROS plant 验证空间不足时尚未开始新反馈。

`scripts/validate_static_storage_guard.py` 使用最新 E/S5 的全部真实记录，
在两个目标文件系统分别完整重写和逐值核查；随后降低本次隔离验证的 WAL
预算，保存触发记录和终止事件，执行 SIGKILL 并核查全部已提交记录。
原始数据库 SHA-256 前后相同。完整 sim、最长写入及共同周期需要使用最终
源码与启用守卫的运行配置继续验证。

## 本机结果

CPU affinity 为 26–27；I/O 与测试时间为
`2026-10-20T23:46:56.740328996+08:00` 至
`2026-10-20T23:48:52.916099575+08:00`。本轮没有其他 sim。
两个文件系统各自精确重写并核查 19,578 条最新 E/S5 记录，
再完成 WAL 预算触发、完整终止事件与 SIGKILL 恢复检查；
全部已提交记录及原始数据库身份保持。收尾实际增长均在预留额度内。

相关七个测试文件共 46 项通过，耗时 97.60 秒。日志：
`output/static-capacity-20261020/runtime-storage-guards-03/tests-green.log`，
SHA-256 为
`dab245a552de9817c60349d1f12e115afe23a426498927dea8057c95c16f958e`。

文件系统验证报告：

- 主 ext4：同目录 `ext4/validation.json`，
  SHA-256 `c3ed61e2bec4800fd82046eb4de607dba4d7ef8a2de107ff8f2b9708b30e8937`。
- NTFS：`/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/runtime-storage-guards-03/validation.json`，
  SHA-256 `b2b2789e791e39e57f84a23399a6d31e8c31da156eecded163bb64b66899aeb5`。

完整来源、预算、原始数据库及恢复结果身份保存于主目录同级
`delivery-summary.json`。十九组审计配置和固定 OBB 名称的实际最大
UTF-8 长度为 10 字节。56、66、76 个几何组合对应的完整九个 trial
candidate 尺寸界限分别为 258,048、286,720、311,296 字节。
真实九个 trial fixture 来自最新 E/S5 的 candidate sequence 823，
保留全部 121,955 字节 MessagePack 值及来源身份。

主目录 `.scratch/static-implementation/prepare_guard_budget.py` 根据指定
配置、方法、周期及几何组合数量生成开发验证预算；已实际生成全部二十项
共同周期预算并检查各项单次空间需求。生成时重新绑定当前源码及输出卷，
支持 `--study-index` 和隔离验证使用的 `--force-wal`。该接口只准备开发
验证，正式容量计划仍须绑定完整最终证据并重新检查两个卷的可用空间。
