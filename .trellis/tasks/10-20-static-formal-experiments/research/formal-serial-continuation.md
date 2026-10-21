# 正式冻结、三卷容量绑定与 1080 项串行执行

本记录基于 `3771bc8` 的完整周期计时源码，只核查现有 CLI、调用关系和小型研究文件。
没有运行 sim、性能测量、数据库校验或正式冻结。`complete-cycle-preflight-01`
和 `guarded-period-study-02` 在本次检查时尚不存在；以下命令依赖其真实结果齐备。

## 启动前置

1. 完整计时 preflight、相关回归、最终审查完成，三卷生命周期与文件系统证据通过。
2. study02 完成 20/40/50/100 ms × A–E 全部 20 项。`period_report.json` 为 pass，
   边界为 `controller_after_synchronous_records_and_recovery_v1`，最短合格周期的
   全部方法 P99.9 ≤ 周期的 80%。`freeze` 会重算选择，并校验完整 sidecar 与原件。
3. 参数选择说明更新至最终共同周期及公共控制参数。S4/S5、420 秒写入及三卷
   生命周期的 `RecordingLimits`、共同参数必须与选定周期相符；若选中周期与
   preflight 的 100 ms 不同，须取得对应生命周期证据，不能沿用不匹配参数。
4. 下述容量公共参数比较缺口完成修复及相关验证，再固定最终源码身份。

当前明确缺口：`plan_static_engineering_capacity.py` 使用 `method="D"` 加载配置，
其 `common_control` 包含 `joint/improved`；`freeze_formal` 将正式配置保存为
`separated/basic`，各次运行再按 A–E 加载方法。`require_engineering_evidence`
只移除 `nominal_feed_multiplier` 后完整比较 control，因此正式绑定会因方法字段
不同而抛出“工程容量公共参数与正式配置不一致”。现有未绑定计划没有经过该路径。
该发现已报告主会话，本轮没有修改生产代码。

## 固定来源和环境

运行入口使用主仓库，不使用本工作树：

```bash
set -eo pipefail
project=/home/lsn/robot/robot_safecontrol
capacity="$project/output/static-capacity-20261020"
scene="$project/output/static-scene-validation-20261020"
volume2=/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020
volume3=/media/lsn/Windows/robot_safecontrol_static_experiments_20261021
preflight="$capacity/complete-cycle-preflight-01"
study="$volume2/guarded-period-study-02"
formal="$project/output/static-experiments/formal-01"
bound="$capacity/formal-capacity-01"
cd "$project"
source .scratch/host-recovery/env.sh
export PYTHONPATH="$project/src:$project/portable_oscbf${PYTHONPATH:+:$PYTHONPATH}"
```

`environment_identity` 的 SHA 包含 `cpu_affinity`、Python、已安装包、native 依赖和
线程环境。study02、freeze、batch 使用相同环境与 `taskset -c 0-15`；容量计算可使用
CPU 16–21。冻结前核对 study.json 实际 affinity 确为 0–15。
source_identity 包括全部项目 Python、portable Python/native 源码及 FCL `.so`。
study02 开始后保持这些文件、依赖和环境不变；源码修复必须在 study02 固定身份前完成。
仅保存旧源码 zip 不能让变更后的当前源码继续同一批次。

物理登记与新增证明继续保留：

| 输入 | SHA-256 |
| --- | --- |
| research/formal-scene-input.json（scene-witnesses 任务） | `0f27070df3e6bd707426db43b85ff61d442e93f89aa55af3fdf12a319e7f3c9e` |
| development-scenes-07/catalog.json | `d4f03cb3d47cd76d8d11a44dfe54557e9b112af05d18a3a55d1772d10ac2ced0` |
| formal-feasibility-01/evidence_index.json | `69e724c3cc392dc4980739c586bef85c2b6029b0822de9dcb3e09de2cdc38f16` |
| formal-feasibility-01/feasibility_matrix.json | `75494befed0744a911c8e5c55ad6e5f1af94c368ba731658110dadbcee42cf49` |

登记中的历史 33/72 快照保持原件；新增矩阵为 39/72，索引含 36 个不同完整来源。
S4 指向 `s4-paired-independent-witness-01/witness` 目录，S5 指向
`development-scenes-07/s5_blockage.json`。这些相对路径均位于 `$scene`。

## 冻结与容量绑定

所有前置通过后，使用尚不存在的正式目录执行：

```bash
taskset -c 0-15 python scripts/static_obstacle_experiment.py freeze \
  --catalog "$scene/development-scenes-07/catalog.json" \
  --period-study "$study" \
  --evidence-index "$scene/formal-feasibility-01/evidence_index.json" \
  --output "$formal"
```

产物为 `formal_manifest.json`、`formal_identity.json`、72 份配置和 `provenance/`。
实际验收要求：1080 个唯一编号、三次重复、A–E 各 216 项；每个配置的 physical_identity
与登记资源重定位后的同名配置一致，证明状态仍为 39/72，S4/S5 scene_evidence 均通过。
`freeze` 校验几何初态与资源，但没有读取 registration 文件；上述登记身份及 72 项逐项
比较由主会话在冻结验收中完成。控制周期与方法字段变化不应改变物理身份。

以下容量命令使用 preflight 生成的六份正常生命周期、三份 filesystem、当前源码归档，
以及三份已有逐流审计。以 preflight 的 `capacity-inputs.json` 核对这些路径；
正式配置提供最终周期与公共参数。输出目录必须尚不存在。

```bash
taskset -c 16-21 python scripts/plan_static_engineering_capacity.py \
  --audit "$capacity/record-envelopes-02/envelopes.json" \
  --audit "$capacity/record-envelopes-03/envelopes.json" \
  --audit "$capacity/record-envelopes-04/envelopes.json" \
  --lifecycle "$preflight/d-s5/lifecycle-validation.json" \
  --lifecycle "$volume2/complete-cycle-preflight-01/d-s5/lifecycle-validation.json" \
  --lifecycle "$volume3/complete-cycle-preflight-01/d-s5/lifecycle-validation.json" \
  --lifecycle "$volume2/complete-cycle-preflight-01/d-s0/lifecycle-validation.json" \
  --lifecycle "$volume2/complete-cycle-preflight-01/d-s4-initial3-feed0.5/lifecycle-validation.json" \
  --lifecycle "$volume2/complete-cycle-preflight-01/d-timeout/lifecycle-validation.json" \
  --filesystem "$preflight/filesystem/validation.json" \
  --filesystem "$volume2/complete-cycle-preflight-01/filesystem/validation.json" \
  --filesystem "$volume3/complete-cycle-preflight-01/filesystem/validation.json" \
  --runtime-archive "$preflight/source/sources.zip" \
  --config "$formal/configs/s0-initial-1-feed-1.yaml" \
  --resource-root "$project" --formal "$formal" --retry-runs 0 \
  --volume "$capacity/formal-runs-ext4" \
  --volume "$volume2/formal-runs" --volume "$volume3/formal-runs" \
  --output "$bound"
```

三个输出目录须已存在，对应 UUID 依次为 `03b3e264-cb91-440b-b594-0bdc7c9d190c`、
`083CBB753CBB5BFE`、`6C001EAA001E7B72`。计划必须 `status=pass`、
`generation.formal_bound=true`、`is_validated_engineering_budget=true`，绑定本次
formal_manifest SHA 和完整 source mapping；1080 个编号每个仅分配至一个卷。
`--formal` 会进一步调用真实工程证据重建和当前空间准入，失败时保留输出供检查。

通过后将计划复制至固定入口，工程证据仍引用 `$bound` 原始路径：

```bash
test ! -e "$formal/capacity_plan.json"
cp --no-clobber "$bound/capacity_plan.json" "$formal/capacity_plan.json"
sha256sum "$formal/formal_manifest.json" "$formal/capacity_plan.json"
```

## 串行执行、恢复与交付判定

以下为现有执行入口，由主会话的持续进程管理方式托管，并保留外层日志：

```bash
taskset -c 0-15 python scripts/static_obstacle_experiment.py batch \
  --output "$formal" --domain 200
taskset -c 0-15 python scripts/static_obstacle_experiment.py paired-report \
  --output "$formal"
```

`batch` 顺序执行固定编号。每项在 `logs/` 保存进程命令、退出码和逐 writer 预算；
`runs/<id>` 链接到计划指定的实际卷。每次启动前后重新检查证据、额度和剩余空间。
`batch_progress.json` 保存进度；容量耗尽或已有运行超额会停止后续批次。

重复执行相同 `batch` 命令只启动尚不存在目录的编号。已存在的成功、失败及 partial
目录全部保留，不会自动重试；`--limit N` 可限制本次新启动数量，部分批次通常退出 1。
启动失败且只有 `logs/<id>.log`、没有运行目录时，下次独占创建日志会拒绝覆盖；
须由主会话保留该尝试并处理明确故障后继续。不得删除失败原件取得完整覆盖。
额外重试需要独立身份和新的容量准入，当前 `--retry-runs 0` 未提供额外预算。

最终保留 `paired_report.json/.md`、全部 process/worker/容量统计与原件身份。
`collection_complete` 仅表示全部编号存在记录；partial 继续为 insufficient_evidence。
交付还需核对 D 的 144 项功能验收、72 项预先证明不足单列、全部方法失败状态与
运行时间结果，并完成 #183 的配对统计、图表及报告。CLI 退出码不能单独替代这些验收。

本轮验证限于源码调用关系、参数名称和路径格式检查；没有将上述待执行命令记为通过。
