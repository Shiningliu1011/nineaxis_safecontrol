# A/B/C/E 方法验收证据

## 当前状态与来源

2026-10-20 读取 #178–#181 最新正文和 comments，四项均为 OPEN、无评论；
GitHub 原生依赖为 `#177 → #178 → #179/#180`、`#180 → #181`。
本记录供 implement-spec 继续执行与逐项验收。#172–#176 已由主会话关闭，
对应完整证据见父任务 `research/ticket-acceptance.md`。

工作目录为 `/home/lsn/robot/robot_safecontrol/.scratch/worktrees/static-evidence`，
分支为 `codex/static-method-evidence`，建立于集成 `5d52885`，旧分支保留。
环境使用 `/home/lsn/robot/robot_safecontrol/.scratch/host-recovery/env.sh`，
`PYTHONPATH` 明确使用本 worktree 的 `src` 与 `portable_oscbf`。
本轮只运行 CPU 16–25 的离线数值和控制验证；没有启动 ROS 闭环。

大型原始记录继续保存在主仓库 `output/`。
本轮命令、上游原件、数值输入输出、清单与日志位于
`/home/lsn/robot/robot_safecontrol/output/static-method-evidence-20261020/`。
`inventory.json` 检查该次扫描中的全部 78 个 manifest，其中 42 个含实验控制配置。
它保存每份配置、反馈报告和终止记录的 SHA-256，包含失败和证据不足的记录。
当前扫描中 B/C 没有完整运行，A/E 的已有记录如下。

| 方法/场景 | 原始目录（`output/static-experiments/` 下） | 实际结果与适用范围 |
|---|---|---|
| A/S0 | `s0-common-control-03` | completed=true、任务通过；36 个自碰撞组合缺少测量，综合 insufficient_evidence |
| A/S0 | `s0-common-control-01`、`02` | 未完成，原始失败保留 |
| A/S1 | `s1-common-control-01`、`s1-full-geometry-01` | 未完成，原始失败保留 |
| E/S0 | `s0-sns-full-geometry-01` | 旧参数完整通过；86469 个反馈、1709 次 SNS，全程 task_scale=1 |

E/S0 历史报告的路径距离 RMS 为 0.059183 mm，工具轴 RMS 为 0.008357°，
外部/自碰撞最小间距为 40.625016/16.999884 mm，20+36 个组合全部测量；
终点保持 3.664 s，完整有序覆盖通过。QP 状态明确为 `not_applicable`。
控制周期 100 ms，P99.9 46.353000 ms，最大 50.047918 ms。
该配置使用 `posture_speed_limit=0.1`，当前开发配置采用 0.02；
当前开发配置的 E/S0 完整验证仍需运行。历史最小 task_scale=1 仅证明基础跟随。

## 独立数值与公共控制验证

本次 `tests/test_static_sns.py` 为 10 passed，日志
`commands/sns-final-tests.log`，CPU 16–25，1.35 s。
未添加 mock。既有解析实例的缩放为 0.5 和 1.1/3，
与 SciPy 1.15.3 HiGHS 的独立最大缩放一致，完整输入输出保存为
`sns-numerical.json`。SVD 参数、零任务、完全阻塞、构型辅助仍通过。
E 组公共控制推进、末端完整区间及共同控制余量的三个受影响集成用例
为 3 passed，40.13 s，日志 `commands/sns-integration.log`；
命令保存为 `commands/run_integration.sh`。

父任务本机执行的 84 passed 包含：

- `test_static_velocity_solver.py`：真实九轴 QP 与独立 SLSQP 的约束、目标值和 KKT；
  另含 `static_qp_feed_boundary.npz` 的 S4 实际 QP 边界输入。
- `test_static_control.py`：A–E 通过同一 Ruckig/几何/反馈接口推进、实际终点区间完成。
- `test_static_experiment.py`：方法选择、显式参数冲突和配置快照。

父任务另一次 15 passed 包含 `test_static_recovery.py` 的持续确认、中断重计、
角度超限、准入拒绝、30 s 持续暂停和终点处理。对应代码与本轮起点一致；
命令和完整日志保存在 `output/static-ticket-evidence-20261020/commands/`。
上述纯逻辑与短控制验证不能替代下列完整场景运行。

## E 组广义上下界

作者来源：
[论文全文](https://arxiv.org/html/2202.13184)、
[作者实现](https://github.com/amrkzp/Generalized-SNS/tree/0d4df38bc735e137a69796dc3ca3396c9de3a616)。
已核查 README、LICENSE、`GSNS_VL.m`、`FindScaleFactor.m`、`damped_pinv.m`
和 `computeVelocityBounds.m` 全文，版本为
`0d4df38bc735e137a69796dc3ca3396c9de3a616`，许可 CC BY 4.0。
`damped_pinv.m` 署名 Fabrizio Flacco，修改者 Amirhossein Kazemipour；
面向使用者的文档保存作者、版本、许可、免责与 Python 移植说明。
本机没有运行 MATLAB/Octave，数值对照采用独立解析结果和 HiGHS。

论文及作者接口使用有序广义速度上下界；它们可以要求某对象主动增加距离。
Python 入口接受有限、有序上下界，保持逐项饱和、任务整体缩放、作者 SVD 参数、
秩判定和最终全部限制检查。原始限制、共同控制余量和构型辅助权重保持相同。

数值输入源自本机已通过的当前 D/S1：
`output/static-ticket-evidence-20261020/s1-current-01/controller.sqlite3`，
SHA-256 `335b3915bc6b6934b83141ae6667915f7fa3c88e4a881f9365bef094a874c7e4`。
sequence=1627、参考进度 0.40490084790027453 m，
Link1/obstacle_1 的 `h=-0.00025650841840563167`。
使用该真实关节位置、参考进度和相同场景重新计算五维任务和全部几何限制。
独立 HiGHS 找到 task_scale=1 的可行结果。

测试输入 `tests/data/static_sns_clearance_reserve.npz` 的 SHA-256 为
`4c1386270958c2bd0a0e94abc3dfc6a3be3209432740dbba5939412ddd4c8eaf`。
测试覆盖实际前进任务、零任务保持、等价反向限制、不可行速度限制必须抛出异常。
`commands/sns-reserve-red.log` 保存 TDD 失败结果；
`commands/sns-final-tests.log` 保存最终通过结果。

完整 S1 记录中 71 个候选要求非零恢复速度。本轮逐个重算前进任务与暂停时的误差校正，
并加入共同构型辅助，共 142 个数值实例全部通过：

- 最大五维任务残差 `3.656318114310038e-16`。
- 最大广义限制超差 `4.414195539796022e-17`。
- 与独立 HiGHS 的最大任务缩放差 `0`。
- 同时核对来源记录的实际 D/QP 约束与 KKT；原始数据库 SHA-256 前后相同。

证据为 `sns-reserve-verification.json`、`commands/verify_sns_reserve.py` 和运行日志。
这些结果是 D 状态上的 E 数值验证；E 的完整闭环仍须按下列清单执行。

## #178：A 组

| 验收项 | 已有证据 | 尚需执行 |
|---|---|---|
| 五个状态及转换原因 | `RecoveryState`、真实状态测试、transition 记录 | A/S4、A/S5 完整运行 |
| 暂停参考保持且允许关节调整 | 公共控制入口在零进给仍计算误差与构型辅助 | A/S4 反馈测得固定参考、关节变化及恢复 |
| 障碍间距与归一化行程构型辅助 | `common_posture_velocity`、真实 Jacobian 零空间及边界测试 | 当前配置完整 A 运行 |
| 基础恢复、几何/求解故障处理 | 公共状态/控制检查、实际无解 QP 拒绝 | 场景中基础恢复过程 |
| 共同进给渐增 | RecoveryState 测试、manifest 保存 0.03 m/s² | A/S4 实际进给与准入 |
| 固定障碍 S4 暂停调整恢复 | 等待 #177 当前可行性见证 | A/S4 完整反馈证据 |
| S5 持续暂停超过 30 s 终止 | timeout 纯逻辑验证 | A/S5 停止、范围、对象与原因 |
| 终点精度、0.5 s 保持与完整覆盖 | 公共终点测试、历史 A/S0 任务通过 | 当前 A/S0 完整几何与执行通过 |
| 入口选择与共同接口 | A–E 真实短控制验证、配置测试 | 完整 A 开发运行 |
| 行为说明 | `docs/static_obstacle_experiments.md` | 随闭环结果补充验收证据 |

## #179：B 组

| 验收项 | 已有证据 | 尚需执行 |
|---|---|---|
| 五维联合目标含误差、进给、平滑与构型 | `solve_velocity` 数学对应及实际 QP/SLSQP 测试 | B 完整开发运行 |
| 全部限制与共同执行检查 | 公共约束、Ruckig、准入测试 | B/S1–S3 的实际限制触发 |
| 原始问题、结果、残差、独立求解 | 真实问题和 S4 QP 数据与独立 SLSQP 一致 | B 运行记录中的代表性问题核查 |
| A/B 单一因素比较 | 方法配置映射和冲突测试 | 相同开发配置 A/B 配对 |
| S0–S4 开发场景 | 当前仅有 D 相关场景证据 | A/B 各自 S0、S1、S2、S3 三档、S4 |
| 零进给、终点、求解失败、超期 | 公共数值/终点/失败周期验证 | B 完整结果中保留这些实际事件 |
| 配对误差、间距、进给和完成结果 | 统一反馈与配对报告入口 | A/B 完整配对报告 |
| 与 M09 关系、数学与运行说明 | 文档说明公共五维定义及 #157 单独验收内容 | #179 完整证据发布 |

## #180：C 组

| 验收项 | 已有证据 | 尚需执行 |
|---|---|---|
| 位置、真实角度与正向候选共同检查 | RecoveryState 与 StaticControl 真实测试 | C/S4 实际恢复 |
| 严格恢复阈值及选择依据 | `parameter-selection.md`，1/0.3 mm、0.2/0.04° | 在 A/C 相同固定配置中保存 |
| 连续至少 0.2 s，中断重新计时 | 真正状态对象的持续确认及中断测试 | C 实际恢复事件时间 |
| 共同渐增与逐周期检查 | 公共控制及状态测试 | C/S4 进给与连续准入记录 |
| A/C 同一 S4 配对 | 报告支持首个正向候选、实际恢复与延迟 | 见证通过后运行 A/C 并生成配对结果 |
| 短暂满足、角度超限、拒绝、阻塞、终点 | 纯逻辑和末端控制测试 | 当前配置实际运行仍需覆盖和逐项索引 |
| 规则独立于求解方法 | C/D/E 参数化真实控制测试 | 各组完整场景验证 |
| 参数、事件、延迟说明 | 使用文档及反馈区间统计说明 | 配对结果写入验收记录 |

## #181：E 组

| 验收项 | 已有证据 | 尚需执行 |
|---|---|---|
| 作者算法、版本与许可 | 上游全文、固定 SHA、CC BY 4.0 及作者说明 | 无来源读取缺口 |
| 可对照数值输入与输出 | 解析/HiGHS、10 tests、142 个真实状态实例 | 随完整运行核查实际 SNS 记录 |
| 原任务缩放语义 | desired 包含误差校正与路径速度，整体缩放 | 受限开发运行中 task_scale<1 的实际样本 |
| 九轴五维、全部限制、构型、恢复 | 真实公共入口测试及非零恢复约束验证 | 当前配置完整 E 运行 |
| 跟随、受限减速、恢复、持续阻塞 | 历史 E/S0 基础跟随 | 当前 E/S0、E/S1、E/S4、E/S5 |
| 反馈与失败进入共同格式 | 历史 E/S0 原始记录及统一反馈报告 | 当前运行完整记录与失败保留 |
| QP N/A 与真实 SNS 有效性 | 历史 E/S0 QP=not_applicable，1709 次 SNS | 当前受限运行的残差及饱和对象核查 |
| 安装、来源与运行说明 | 固定依赖环境、共同入口、移植说明 | 闭环结果写入验收记录 |

## 需要补充的开发运行

以下配置根目录为
`/home/lsn/robot/robot_safecontrol/output/static-experiments/development-scenes-05`。
所有运行仍为开发验证。正式采集需要容量、S4/S5、全部前置条件及共同周期通过。

| 组合 | 配置 | 验收重点 |
|---|---|---|
| A/B/C/E × S0 | `s0.yaml` | 当前共同参数完整跟随、终点、全部几何与执行限制 |
| A/B × S1；E × S1 | `s1.yaml` | 真实外部限制、A/B 配对；E 缩放与全部广义限制 |
| A/B × S2 | `s2.yaml` | 同时限制不同连杆，完整任务和误差 |
| A/B × S3 三档 | `s3-travel-100.yaml`、`70.yaml`、`40.yaml` | 生效行程比例及真实受限结果 |
| A/B/C/E × S4 | #177 最终通过的 S4 配置和见证 | 固定参考期间构型变化、恢复时刻、A/C 恢复差异 |
| A/C/E × S5 | #177 最终通过的 S5 配置和阻塞证据 | 持续暂停终止、实际停止、范围与限制对象 |

S0–S3 的当前文件 SHA-256：

| 文件 | SHA-256 |
|---|---|
| `s0.yaml` | `32c81c2f22c3ade49453ff52e2f9ba624973ee345b071d75110d4c8915b3c01b` |
| `s1.yaml` | `d72fb1bff41bbede1a2677ec4ed22df097c7384ed3fbc88bfcfd7748e9a98941` |
| `s2.yaml` | `7b84a9f36c6db64c86b690ca1c117e39ba7550ecafa6bad0ef79373decefff38` |
| `s3-travel-100.yaml` | `485f34c20536797c4d762b8b7d066cfed3cd5fe822e48ad0046a8e4779e9b10e` |
| `s3-travel-70.yaml` | `9e946a10499b87621af929f028d6ea2632ff65bebeb3144214b74773fe175c62` |
| `s3-travel-40.yaml` | `bf5ed6e1b835368ac19bb520b93fc1eda13e9e4ef41c46a46c075b46d05f09fa` |

获得独占 sim 窗口后，单个组合的完整命令如下。每次更换 `method`、`config`、
`output`；输出目录必须尚不存在。

```bash
source /home/lsn/robot/robot_safecontrol/.scratch/host-recovery/env.sh
cd /home/lsn/robot/robot_safecontrol/.scratch/worktrees/static-evidence
export PYTHONPATH="$PWD/src:$PWD/portable_oscbf:$PYTHONPATH"
taskset -c 0-15 python scripts/static_obstacle_experiment.py run \
  --config /home/lsn/robot/robot_safecontrol/output/static-experiments/development-scenes-05/s0.yaml \
  --method A --resource-root "$PWD" --domain 199 \
  --output /home/lsn/robot/robot_safecontrol/output/static-method-evidence-20261020/a-s0-current-01
```

S4 必须另加 `--scene-evidence <已通过的见证目录>`；
S5 必须另加 `--scene-evidence <已通过的s5_blockage.json>`。
这两项的最终配置与身份由 #177 交付后填写，禁止使用未通过的候选替代。
相同场景的成对运行保持全部非方法参数一致，运行完成立即核对 manifest、
完整覆盖、反馈精度、全部几何、执行限制、恢复、SNS/QP 和完整周期。
保留未完成与超期记录，不用单一几何通过结果替代整项验收。

## 数据身份

| 文件 | SHA-256 |
|---|---|
| `inventory.json` | `23873960cc88bee32d49be3e473654dcbb302b32ed48a48a4b82c86dc86db292` |
| `sns-numerical.json` | `5a5246c4edde876cf60257d9c02b717d85a2e6d431d7550af3f6068cd798d266` |
| `sns-reserve-verification.json` | `76194efd05f52498c37d4267560db528e1f8fd6fd7991a455469128a9c6b594b` |
| 历史 E/S0 `manifest.json` | `c346e4ba4da40161b9ee6777c26eb109ac0f2bb542579093c1787e41d143e3ef` |
| 历史 E/S0 `controller.sqlite3` | `7316555aef9c06fdb3364716d91a29ebba6e1e3f1d3efc7b5d3ec079ad53b6c3` |
| 历史 E/S0 `plant.sqlite3` | `703d732d3f9256946cdb91f12cb369d86dad7d2b5ae6a0f746d990df39130e3e` |
| 历史 E/S0 `feedback_report.json` | `b9e7de08e2bc019993e3ed4f4179deae530fe797a4331a820de81954cda2c85a` |
| 作者 `GSNS_VL.m` | `48d24ad3dca937c7bd42f4dcd2c80adca3c90dca8f30b5afd1bce79c0d408028` |
| 作者 `FindScaleFactor.m` | `3f46dc0882568b1b88ce5416292abd9995b0f414e7b31473ae45d861a9c63c70` |
| 作者 `damped_pinv.m` | `bf039fc0bc65a5eb349c72c7e36989b85375ebb2dbf3c18ab6108978f1bcf46c` |
| 作者 LICENSE | `7e7170e3cebf88a9f60c7b8421418323c09304da1af4d5e90f4da1dc1c8a2661` |
