# 2026-10-20 主机恢复与紧凑存储审查

审查基准为 `6afe1d34a0d1d33b85f91c53dbe103023ce718ec`，
范围包括本次工作区的记录格式、公共读取、迁移、评价、图表、测试和接手文档。
在 `trellis-check` 完成构建、快速检查、相关测试及回归问题定向复核后，
依照项目 `code-review` skill 安排两个独立审查者并行只读检查。

## 规范与代码质量

`storage_quality_review` 核查项目编码、依赖、错误处理、日志及来源记录规范。
记录编码、stream 字典、随机访问、事务提交顺序、原始文件保护和依赖声明
未发现需要继续处理的问题。

评价来源记录由 `evaluation_samples()` 返回实际使用的完整评价报告，
`figures.json.evaluation_provenance` 保存相应 evaluator 和源码身份。
按需生成使用本次重算身份；使用历史数组时采用其原报告提供的身份。
原始报告保持不变。审查者对修正及全部调用点完成只读复核，通过。

## 需求、验收与范围

`storage_acceptance_review` 核查完整真实记录数量、内容、恢复、重放、
耗时、容量、原始资料保护和 sim 范围。
四组共 331,573 条记录、8 次 SIGKILL 和完整 S0 sim 证据与记录一致；
S4/S5 及共同周期的未完成状态保持明确。

`validate_storage_migration()` 在重放和报告验证中统一核对转换数据库集合、
数据库 SHA-256、源文件清单、保留的 manifest/result/path 以及来源 manifest。
身份检查发生于重写报告之前。审查者对修正及新增拒绝检查完成只读复核，通过。

完整历史 S0 的隔离副本删除了 1,712 条 `received_command`：
重放与报告读取均因转换数据库身份变化而拒绝，来源目录及已有报告保持不变。
实际结果为 `output/static-storage-migration-identity-01.json`，命令退出 0；
入口 `.scratch/host-recovery/check_migration_identity.py`，脚本身份写入结果。

## 最终复核

完整回归通过：主包 678 passed、1 skipped，控制内核 486 passed、
34 skipped，C++ CTest 4 passed。来源检查修正的存储、评价和实验管理
25 项测试通过；六类图表重新生成，实际评价源码、字体、PNG 与原始报告身份检查通过。

回归入口为每次运行创建独立产物目录。baseline、M6、M8、perception
共享指定的测试目录；单独读取基准前检查当前语义，必要时实际生成基准。
ROS 测试的报告使用 pytest 目录，辅助节点在 executor 和线程退出后销毁。
ROS 14 项和控制内核 10 项定向测试通过；ROS 完整流程的线程异常严格检查
1 项通过且无警告。M8 与 perception 另从各自空目录运行，
分别 5 passed（111.12 s）和 3 passed（62.38 s），8 份历史文件 SHA 保持一致。
`storage_quality_review` 已完成上述实现与运行证据的只读复核，无待处理发现。

`storage_acceptance_review` 核查最终资料清单：
14,284 个文件、23,143,586,250 字节全部匹配来源身份，
三份笔记本基准原件的 SHA 已独立核对。
108 份迁移来源资料均有逐文件一致的保留副本。
正式采集停止状态、剩余容量和 S4/S5、共同周期的待办与用户要求一致。
需求与验收复核无待处理发现。

## 工作流和交付边界

本次用户授权恢复、安装、修改、验证、更新接手记录及提交、推送，
并补充授权删除已经完成恢复且不再需要的迁移目录。
正式采集保持停止；原始实验及标定记录保留。

格式及来源检查的使用说明保存在 `docs/static_experiment_storage.md`。
这些知识可作为后续规范更新的候选，目标为
`.trellis/spec/backend/logging-guidelines.md`；规范升级仍按 Phase 3.3
单独确认。本次保留现有规范、CONTEXT 和 ADR。
当前任务保持 `in_progress`，本次工作不归档正式实验任务。
