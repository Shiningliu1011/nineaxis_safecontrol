# 两个 S2 初态的正式证据接续入口

本轮普通合并至 `dda176d`，只准备研究脚本和执行说明。
#177–#181 的关闭状态及 `formal-scene-input.json` 中登记的 33/72
历史快照保持相同。没有启动 sim、读取大型数据库、重新运行求解器或 FCL 审计。

入口为 [prepare_formal_proof_index.py](prepare_formal_proof_index.py)，
SHA-256 为 `ce2915969765ba1bd3732c2395c37f40d0a4a6d95e0cf196afb62501037333eb`。
该脚本使用显式 `--resource-root`，不根据归档副本所在目录推导仓库。

## 输入和检查范围

主会话 `.scratch/static-implementation/run_guarded_s2_initials.py`
完成两项 D/S2 后，控制目录应包含：

- `output/static-capacity-20261020/guarded-s2-initials-01/completed.json`；
- 同目录的 `execution.json`、`status.json` 与 `source/snapshot.json`；
- 每项实际原始目录中的完整报告、两份数据库、生命周期报告及容量统计。

脚本在加载验证依赖和读取大型原件前，要求两项完成记录齐备、
`running=null`、执行与完成源码身份相同、必需原件全部存在。
缺少实际结果时立即抛出异常，输出目录仍保持不存在。

物理输入继续使用已登记的 dev07、72 配置及 33/72 矩阵。
登记 SHA-256 为
`0f27070df3e6bd707426db43b85ff61d442e93f89aa55af3fdf12a319e7f3c9e`；
两项计划 SHA-256 为
`172575eb2694e2d57e7214fc3a7d1e30a8a38e6c4c324b3bb08d1392f4a60935`。
脚本核对所有登记资源、72 份配置文件、两项计划及有效配置身份。

现有矩阵中的全部 `complete_run_evidence` 指向 34 个不同运行目录。
与两项新增结果按 `Path.resolve()` 后的实际目录去重，每个目录仅执行
一次 `validate_run_report`。验证保留配置、路径、结果、报告、两份
数据库及派生数据的完整校验，并要求完整跟随、任务、外部间距和自碰撞通过。

历史源码身份绑定其已登记 `manifest_sha256`，原始 manifest 中的
`source_sha256` 完整保留。新增两项额外要求 `execution.json`、
`completed.json`、实际 manifest 的源码相同，并执行 `verify_snapshot`
核对整个源码归档、逐文件 SHA、环境文件及来源映射。
历史来源不要求其原源码路径上的当前文件仍为旧版本。

新增来源还必须与计划输出目录、配置身份、全部原件校验值、生命周期
报告完全对应；workers 为 `[0,0]`、终止为 `completed`。
容量预算必须与 manifest 一致，生命周期的容量检查为通过，
`validate_period_storage` 再核对两份实际容量统计与结果中的校验值。

所有来源与 72 配置通过现有 `_physical_identity` 入口重定位资源并匹配。
两个新增来源分别只允许匹配 S2 初态 2、3 的三档进给。
全部检查通过后，新矩阵应为 39/72；其他未证明状态保持明确。

## 输出和执行命令

主会话已在两项实际来源完成后执行下列命令，结果见
[formal-proof-index-results.md](formal-proof-index-results.md)：

```bash
source /home/lsn/robot/robot_safecontrol/.scratch/host-recovery/env.sh
taskset -c 22-27 python /home/lsn/robot/robot_safecontrol/.trellis/tasks/10-20-static-formal-experiments/research/prepare_formal_proof_index.py \
  --resource-root /home/lsn/robot/robot_safecontrol \
  --output /home/lsn/robot/robot_safecontrol/output/static-scene-validation-20261020/formal-feasibility-01
```

`--output` 必须不存在，全部来源检查完成后才创建目录，输出四个文件：

- `feasibility_matrix.json`：72 配置的新增证明状态和全部匹配来源；
- `verified_sources.json`：按实际目录去重的完整验证清单；
- `evidence_index.json`：生产 `freeze_formal` 使用的四字段绝对路径索引；
- `summary.json`：输入、源码归档、计划、参数说明、脚本及输出校验值。

索引中的 `s4_witness` 指向
`output/static-scene-validation-20261020/s4-paired-independent-witness-01/witness`
目录，`s5_blockage` 指向 dev07 的 `s5_blockage.json` 文件。
只核对既有独立见证文件及状态数组身份，本轮不重算 solver/FCL。
纯数值记录与 S5 暂停超时记录继续保留原用途，不加入完整跟随来源列表。

`parameter_rationale` 指向现有 `parameter-selection.md`。
共同周期确定后，主会话维护最终选择依据；正式 freeze 读取该路径当时
的内容。该脚本不执行 freeze、不启动正式采集，容量和最终审查仍独立验收。

## 本轮实际验证

使用 `.scratch/host-recovery/env.sh` 中的 Python 完成语法编译和
`--help`，两项均退出 0。
在实际 `completed.json` 尚不存在时调用入口，得到 `FileNotFoundError`
并退出 1；未创建 `formal-feasibility-01` 输出目录，未进入大型原件校验。
记录保存在本 worktree 的 `.scratch/proof-index-missing-results.log`。
完整来源执行阶段已由主会话通过；结果、来源和校验值见上述实际结果记录。
