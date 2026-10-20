# #171 接续执行

## 执行入口

集成分支为 `codex/static-obstacle-experiments`，恢复起点 `35a1a61`。
`implement-spec` 管理全部 #172–#183 的实现、调度、集成和最终 code-review。
Trellis 管理既有任务、规划、证据与接手记录。
本机环境入口为 `.scratch/host-recovery/env.sh`；所有实验限定 sim。

## 当前安排

1. #172–#176 已完成逐项验收并关闭，证据保存于 `research/ticket-acceptance.md`。
2. 接续 #177 的 S4 独立见证、完整暂停恢复、S5 终止及当前参数验证。
3. 核查 #183 所需容量上界、失败重试及临时文件空间，保存计算来源。
4. 按依赖补齐场景和 A/B/C/D/E 开发验证，完成 #182 的二十组周期测量。
5. 全部前置条件通过后固定正式清单，执行 1,080 次实验及独立分析。
6. 完成回归、规范与需求审查，修复发现并复验，按 tracker 规则关闭有充分证据的任务。
7. 更新接手记录、数据身份与校验清单，提交并推送相关工作。

## 子任务与计算资源

实现工作使用基于集成分支的独立 worktree，原始大型数据由主目录只读提供。
子任务提交前合并集成最新提交，集成由 merger 子 agent 执行。
ROS sim 与周期测量由主会话协调；正式周期测量期间停止其他计算密集验证。
容量与已有证据检查可独立推进，正式采集等待所有前置证据通过。

当前集成为 `659bf7c`，A/S0 方法证据为 `062cd52`。
容量 worktree 已完成 D/S0、D/S5、420 秒 timeout 三项真实记录验证；
D/S5 的 P99.9 为 82.480 ms，周期要求尚未通过。
下一步由容量任务补充 worker 记录额度、异常退出验证和容量预算；
场景任务继续 C/S4 实际来源、独立构型见证及 S5 性能验证。
完整测量目录、来源和 SHA-256 见 `docs/static_obstacle_experiments_handoff.md`。

原始实验为 `output/static-experiments/`，紧凑验证为
`output/static-storage-validation-05/`，恢复来源和逐文件身份见
`docs/host_recovery_20261020.md`。新数据使用单独目录，原件保留。
