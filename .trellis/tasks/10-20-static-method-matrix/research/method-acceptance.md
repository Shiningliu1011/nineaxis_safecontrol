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
数值验证使用 CPU 16–25。A/S0 使用 CPU 0–15；
后续 B/C/E S0、A/B S3-100/70 和 A/S3-40 使用 CPU 8–15、domain 199，
均完成完整 ROS 仿真。来源、参数分组与剩余组合见下文。

大型原始记录继续保存在主仓库 `output/`。
本轮命令、上游原件、数值输入输出、清单与日志位于
`/home/lsn/robot/robot_safecontrol/output/static-method-evidence-20261020/`。
`inventory.json` 检查该次扫描中的全部 78 个 manifest，其中 42 个含实验控制配置。
它保存每份配置、反馈报告和终止记录的 SHA-256，包含失败和证据不足的记录。
该次初始扫描中 B/C 没有完整运行，A/E 的历史记录如下；
本次新增的九份完整运行见后文开发批次。

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
0.02 参数组的 E/S0 完整验证已通过。历史最小 task_scale=1 仅证明基础跟随。

## 当前共同参数 A/S0

运行目录：
`/home/lsn/robot/robot_safecontrol/output/static-method-evidence-20261020/a-s0-current-01`。
代码来源为 `659bf7c1a14beffd2a435b23768adc615d896156`，
配置为 `development-scenes-05/s0.yaml`，SHA-256
`32c81c2f22c3ade49453ff52e2f9ba624973ee345b071d75110d4c8915b3c01b`。
`commands/a-s0-current-01.log` 保存实际 Python、模块路径、CPU affinity、
domain、来源提交与完整入口输出；执行脚本为 `commands/run_s0.sh`。

方法配置为 A＝separated/basic，保持共同 gains、构型辅助、100 ms 周期、
0.1 mm 控制间距余量及完整九轴执行限制。运行期间源码身份保持相同。
CPU 16–27 同时存在场景离线筛查和容量验证负载，本次耗时用于开发记录；
最终共同周期另在独占测量窗口选择。

| 验收对象 | 实际结果 |
|---|---|
| 完成与来源 | `completed=true`；functional/runtime/verdict 均为 pass；worker exit codes `[0, 0]` |
| 反馈与覆盖 | 90,622 个有效反馈，全部数值有效；参考及反馈有序完整覆盖通过 |
| 路径距离 RMS/P95/最大值 | 0.054778 / 0.119293 / 0.183777 mm |
| 真实工具轴角度 RMS/P95/最大值 | 0.008080 / 0.017736 / 0.079091° |
| 外部间距 | 全部 20 个组合通过，最小 35.598525 mm，unsafe_samples=0 |
| 自碰撞间距 | 全部 36 个组合通过，最小 15.800911 mm，unsafe_samples=0 |
| 终点 | 保持 3.570 s；位置误差 0.001874 mm，工具轴角度 0.000077° |
| QP 与执行 | 1,797 次 QP 全部成功，限制超差 0，最大 KKT 残差 `1.2619742389077326e-09` |
| 九轴与参考 | 位置、速度、加速度限制通过；参考结算缺失、不一致、未经准入样本均为 0 |
| 完整周期 | 1,790 次；P99.9 64.693851 ms，最大 71.483849 ms；超期、拒绝周期与准入拒绝均为 0 |
| 存储格式 | controller/plant 均为 3，SQLite integrity_check=ok；数据库合计 44,662,784 bytes |

原始 plant 数据库含 94,275 个 feedback 样本，评价区间采用其中 90,622 个，
覆盖开始运行至完成后的静止区间。初始参考暂停 0.092 s 后进入 RUNNING；
S0 没有障碍引起的暂停恢复事件，该运行完成基础跟随验收。
所有命令、候选、准入和 cycle 各为 1,790 条，transition 为 2 条。

`commands/audit_run.py` 调用共同 `validate_run_report`，核对 manifest、报告、
两份数据库、path、result 和原始配置的身份，检查数据库格式与完整性，
并验证 manifest 中全部 source_sha256。核查前后八份原件的 SHA-256 相同。
结果为 `commands/a-s0-current-01-audit.json`，SHA-256
`0299c8ab43ef9d5b65798b2141c10f151e56ab56b338d57eb4c1751507db8b3c`。
完整运行退出后已确认 supervisor 和两个 worker 均不存在，并释放 CPU 0–15/domain 199。

## `posture_speed_limit=0.02` 开发批次

除上述 A/S0 外，本批次八份运行固定执行来源
`1186d0c1a8e0ea1766a4efa9f4ed6c2a73f8a403`。该提交与 A/S0 的 `659bf7c`
仅有文档变化；配对时逐个比较 manifest 中的全部 source_sha256，结果一致。
所有配置继续使用 `development-scenes-05`，方法验证占用 CPU 8–15/domain 199，
scene 使用 CPU 0–7/domain 198，storage 使用 CPU 16–27。不同 CPU 分配和并行负载
均已记录；这些周期数据只用于观察，最终共同周期需单独测量。

以下九份记录的 functional/runtime/verdict 均为 pass，worker exit codes 均为 `[0, 0]`。
参考和反馈有序完整覆盖、终点精度、九轴限制及参考结算均通过。
S0 完整测量 20+36 个几何组合；S3 完整测量 30+36 个组合。
每个目录均采用格式 3，SQLite integrity_check=ok，核查前后原始文件 SHA-256 相同。
全部目录位于本记录的数据根目录，名称均以 `-current-01` 结尾。

| 目录前缀 | 有效反馈 / 周期 | 路径 RMS（mm） | 工具轴 RMS（°） | 外部 / 自碰撞最小间距（mm） | 终点保持（s） |
|---|---:|---:|---:|---:|---:|
| `a-s0` | 90,622 / 1,790 | 0.054778 | 0.008080 | 35.598525 / 15.800911 | 3.570 |
| `b-s0` | 87,210 / 1,717 | 0.056181 | 0.008907 | 35.617379 / 15.789116 | 4.094 |
| `c-s0` | 90,631 / 1,789 | 0.054792 | 0.008054 | 35.591198 / 15.800042 | 3.626 |
| `e-s0` | 86,545 / 1,706 | 0.057143 | 0.008190 | 35.637957 / 15.959460 | 3.818 |
| `a-s3-travel-100` | 98,445 / 1,945 | 0.049901 | 0.008007 | 30.015187 / 15.800003 | 3.698 |
| `b-s3-travel-100` | 87,330 / 1,719 | 0.055909 | 0.009154 | 30.013741 / 15.787306 | 4.114 |
| `a-s3-travel-70` | 104,048 / 2,056 | 0.047739 | 0.007773 | 30.051161 / 15.789150 | 3.778 |
| `b-s3-travel-70` | 87,514 / 1,724 | 0.055922 | 0.010099 | 30.033826 / 15.789339 | 4.016 |
| `a-s3-travel-40` | 115,441 / 2,285 | 0.044560 | 0.007501 | 30.006549 / 15.782086 | 3.738 |

| 目录前缀 | 周期 P99.9 / 最大值（ms） | 超期 / 拒绝周期 |
|---|---:|---:|
| `a-s0` | 64.693851 / 71.483849 | 0 / 0 |
| `b-s0` | 67.346954 / 337.291938 | 1 / 1 |
| `c-s0` | 53.894852 / 73.792095 | 0 / 0 |
| `e-s0` | 71.239214 / 76.871503 | 0 / 0 |
| `a-s3-travel-100` | 63.713718 / 70.159609 | 0 / 0 |
| `b-s3-travel-100` | 74.911537 / 360.195388 | 1 / 1 |
| `a-s3-travel-70` | 56.331587 / 67.537570 | 0 / 0 |
| `b-s3-travel-70` | 53.410347 / 330.361025 | 1 / 1 |
| `a-s3-travel-40` | 63.593138 / 79.461541 | 0 / 0 |

B/S0 的 command 1709 由 controller 与 plant 共同以 `late_command` 拒绝；
simulation 178.738 s 进入 REFERENCE_PAUSED，178.9 s 恢复，179.0 s 进入
ENDPOINT_HOLD。对应 cycle、准入和转换保存在
`commands/b-s0-current-01-events.json`。该记录支持实际超期处理和终点附近恢复核查。

每份 A/B/C 新记录的全部 QP 限制由原始输入与实际解独立重算，并选取开始、
中段、终点、最大 KKT 残差和首次负上界的代表问题，与 SciPy SLSQP
核查可行性及目标值；全部通过。完整问题、实际解与独立解保存在
`commands/<目录>-solver.json`。

E/S0 含 1,708 个 SNS 实际结果，最小 task_scale=1，QP 为 `not_applicable`。
全部广义上下界核查通过，最大完整任务残差为 `1.7484169697521853e-7`，
与原始记录一致。全部无饱和实例满足作者阻尼参数对应的独立正规方程
`J Jᵀ + λ² I`，任务差不超过 `1e-10`；四个代表问题的最大任务缩放
与独立 HiGHS 一致。原始 controller SHA-256 在核查前后相同。
证据为 `commands/e-s0-current-01-solver.json`，SHA-256
`3961058836ef74a4813c3e143a7893e3e318d6c50aade9f3d684bdef0169e984`。
`commands/verify_recorded_solver_damped.py` 保存完整残差、作者阻尼方程与
HiGHS 的核查方法。受限缩放及暂停恢复仍需要 E/S1、E/S4、E/S5 的真实运行。

### S1 开发证据映射与 S3 行程

A/B 的 S3-100 记录分别通过 S1 等价核查：
有效 scene（去除 config_sha）、初态、execution_limits、全部控制参数、
每个路径数组及实际评价分支均一致。两个场景均返回相同的
`complete_tracking` 实际结果。原始记录保持 `scenario=S3`、
`joint_travel_scale=1.0`，以身份映射支持 S1 开发验收。
正式矩阵仍分别保留 S1 与 S3-100 的独立运行。
依据及检查脚本为 `commands/*-s3-travel-100-current-01-s1-equivalence.json`
和 `commands/verify_s1_equivalence.py`。

A/B 的 100% 行程记录中分别有 72/214 条实际执行命令的
`Link1/obstacle_1` 约束活跃；获准运动均覆盖全部 66 个几何组合。
实际命令、plant 准入和反馈 active_command_id 已交叉核对，
记录为 `commands/*-s3-travel-100-current-01-records.json`。

五份 S3 记录的 J1/J4 上下限按初始构型和 100%、70%、40% 比例逐项核查通过，
其余七轴范围及九轴速度、加速度、jerk 限制保持一致。
证据 `travel-scope-verification-01.json` 的 SHA-256 为
`f407238301121a23b827cf22a3460f6c696aa660ae5ce2e9abf90415fa0900d1`。

### 记录索引与运行边界

`development-comparison-03.json` 包含九份完整记录的原始文件 SHA-256、
完成结果、误差、间距、实际进给、暂停时间和周期统计，以及四组真实配对：
A/B S0、A/C S0、A/B S3-100、A/B S3-70。配对已检查物理条件、
全部非方法控制参数与执行源码身份。该索引 SHA-256 为
`aa9b390dcb615cdebee9a530525c65b2c841f0bead2fb32ddb23f76033ab8fc7`。
执行、审计、独立求解与配对脚本均保存在数据根目录的 `commands/`。

本批次完成后已确认所属 supervisor、worker 和外层派发进程均退出，
CPU 8–15/domain 199 已交还主会话。共同构型辅助上限保持 0.02，
与本节九份数据一致。B/S3-40、A/B S2、E/S1 尚未启动；
新独立见证对应的 S4 方法运行、A/C/E S5 及最终共同周期仍待完成。
当前源码的 D/S5 完整复验结果由主会话记录。

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
| 障碍间距与归一化行程构型辅助 | `common_posture_velocity`、真实 Jacobian 零空间及边界测试，当前 A/S0 完整通过 | A/S4 实际暂停期间构型调整 |
| 基础恢复、几何/求解故障处理 | 公共状态/控制检查、实际无解 QP 拒绝 | 场景中基础恢复过程 |
| 共同进给渐增 | RecoveryState 测试、manifest 保存 0.03 m/s² | A/S4 实际进给与准入 |
| 固定障碍 S4 暂停调整恢复 | 等待 #177 当前可行性见证 | A/S4 完整反馈证据 |
| S5 持续暂停超过 30 s 终止 | timeout 纯逻辑验证 | A/S5 停止、范围、对象与原因 |
| 终点精度、0.5 s 保持与完整覆盖 | 当前 A/S0 完整通过、保持 3.570 s、全部 20+36 几何组合与执行通过 | 在受限开发场景继续核查 |
| 入口选择与共同接口 | A–E 真实短控制验证、配置测试、当前 A/S0 完整通过 | 受限 A 开发运行 |
| 行为说明 | `docs/static_obstacle_experiments.md` | 随闭环结果补充验收证据 |

## #179：B 组

| 验收项 | 已有证据 | 尚需执行 |
|---|---|---|
| 五维联合目标含误差、进给、平滑与构型 | `solve_velocity` 数学对应、实际 QP/SLSQP、B/S0 与 S3-100/70 完整通过 | B/S2、S3-40、S4 |
| 全部限制与共同执行检查 | B/S3-100 的 214 条实际命令外部约束活跃，完整几何及执行通过 | B/S2、S3-40 |
| 原始问题、结果、残差、独立求解 | 当前 A/B/C 完整记录的全量限制及代表性独立 SLSQP 通过 | 后续实际记录继续核查 |
| A/B 单一因素比较 | S0、S3-100/70 配对通过物理、非方法参数和源码身份检查 | S2、S3-40、S4 配对 |
| S0–S4 开发场景 | A/B S0、S1 等价映射、S3-100/70，A/S3-40 完整通过 | A/B S2，B/S3-40，A/B S4 |
| 零进给、终点、求解失败、超期 | 公共数值验证；B/S0 真实 late_command 拒绝、暂停恢复和终点保持 | 后续受限场景的实际事件 |
| 配对误差、间距、进给和完成结果 | `development-comparison-03.json` 保存逐项配对 | 补齐剩余场景的配对结果 |
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
| 九轴五维、全部限制、构型、恢复 | 当前 E/S0 完整通过，真实非零恢复约束数值通过 | E/S1、E/S4、E/S5 |
| 跟随、受限减速、恢复、持续阻塞 | 当前 E/S0 完整基础跟随 | E/S1、E/S4、E/S5 |
| 反馈与失败进入共同格式 | 当前 E/S0 格式 3 完整记录与反馈评价 | 后续受限运行及失败保留 |
| QP N/A 与真实 SNS 有效性 | 当前 E/S0 QP=N/A；1708 次 SNS 全量限制、残差、作者阻尼方程与独立 HiGHS 通过 | 受限运行的残差及饱和对象核查 |
| 安装、来源与运行说明 | 固定依赖环境、共同入口、移植说明 | 闭环结果写入验收记录 |

## #182：完整方法矩阵与共同周期

2026-10-20 再次读取 #182，状态为 OPEN、无评论，依赖 #179 与 #181。
方法映射、共同控制参数与入口选择已有真实测试证据。A/B/C/E S0 完整通过；
D/S0、D/S1、D/S2 既有完整证据见父任务与本任务实施记录。
#177 的新独立见证及对应 D/S4 由 scene agent 记录，
共同构型辅助上限保持 0.02，完整方法矩阵与共同周期仍需验证。

周期验收需要在目标机器、固定软件与参数、预热后测量全部 A–E，
候选为 20、40、50、100 ms；完整周期须包含几何、恢复判断、求解、
执行检查和同步记录。选择全部方法 P99.9 不超过周期 80% 的最短候选，
同时记录最大耗时与超期次数。没有候选满足条件时保留失败记录。
当前有并行计算的单次开发运行不能确定最终共同周期。
正式清单须在场景、容量及全部前置验证通过后生成。

## 需要补充的开发运行

以下配置根目录为
`/home/lsn/robot/robot_safecontrol/output/static-experiments/development-scenes-05`。
所有运行仍为开发验证。正式采集需要容量、S4/S5、全部前置条件及共同周期通过。

| 组合 | 配置 | 验收重点 |
|---|---|---|
| E × S1 | `s1.yaml` | E 缩放与全部广义限制；A/B S1 开发证据由已核查的 S3-100 实际记录映射 |
| A/B × S2 | `s2.yaml` | 同时限制不同连杆，完整任务和误差 |
| B × S3-40 | `s3-travel-40.yaml` | 生效行程比例及真实受限结果；A 三档及 B 100/70 已通过 |
| A/B/C/E × S4 | #177 最终通过的 S4 配置和见证 | 固定参考期间构型变化、恢复时刻、A/C 恢复差异 |
| A/C/E × S5 | #177 最终通过的 S5 配置和阻塞证据 | 持续暂停终止、实际停止、范围与限制对象 |

当前调度由主会话管理。方法批次已停止派发，CPU 8–15 用于主会话的
420 s 生命周期压力验证，CPU 0–7 用于 scene agent 的新见证 D/S4 运行。
收到新的 sim 窗口分配后继续剩余方法验证，共同构型辅助上限保持 0.02。
后续数据使用独立目录并记录实际源码身份，保留本节全部开发证据。

S0–S3 的当前文件 SHA-256：

| 文件 | SHA-256 |
|---|---|
| `s0.yaml` | `32c81c2f22c3ade49453ff52e2f9ba624973ee345b071d75110d4c8915b3c01b` |
| `s1.yaml` | `d72fb1bff41bbede1a2677ec4ed22df097c7384ed3fbc88bfcfd7748e9a98941` |
| `s2.yaml` | `7b84a9f36c6db64c86b690ca1c117e39ba7550ecafa6bad0ef79373decefff38` |
| `s3-travel-100.yaml` | `485f34c20536797c4d762b8b7d066cfed3cd5fe822e48ad0046a8e4779e9b10e` |
| `s3-travel-70.yaml` | `9e946a10499b87621af929f028d6ea2632ff65bebeb3144214b74773fe175c62` |
| `s3-travel-40.yaml` | `bf5ed6e1b835368ac19bb520b93fc1eda13e9e4ef41c46a46c075b46d05f09fa` |

获得主会话分配的 sim 窗口后，单个组合的完整命令如下。每次更换 `method`、`config`、
`output`；输出目录必须尚不存在。

```bash
source /home/lsn/robot/robot_safecontrol/.scratch/host-recovery/env.sh
cd /home/lsn/robot/robot_safecontrol/.scratch/worktrees/static-evidence
export PYTHONPATH="$PWD/src:$PWD/portable_oscbf:$PYTHONPATH"
taskset -c 8-15 python scripts/static_obstacle_experiment.py run \
  --config /home/lsn/robot/robot_safecontrol/output/static-experiments/development-scenes-05/s3-travel-40.yaml \
  --method B --resource-root "$PWD" --domain 199 \
  --output /home/lsn/robot/robot_safecontrol/output/static-method-evidence-20261020/b-s3-travel-40-current-01
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
| 当前 A/S0 `manifest.json` | `659644f40259547c9372f18b22d5765c772054cbb839c95326472c47ae84ca6f` |
| 当前 A/S0 `feedback_report.json` | `bee20a3b2bce1127044a4da87085c12efc75ddee3630554279cc83d469dbba90` |
| 当前 A/S0 `controller.sqlite3` | `26d91bdc1393dd86b587c4a5d0954aa317e1efb4aeceaa03f737c00490f9f16e` |
| 当前 A/S0 `plant.sqlite3` | `bf0de3db1b29aed617e40736982e9b0c4c47be37e2c09d61ad27e089e9620ddb` |
| 当前 A/S0 `result.json` | `1bfbf10242046eaa084e8adb39c1428f31a403dd7182cf6aa40c2b49285412f8` |
| 当前 A/S0 `path.npz` | `ed50f33cb09929285233bc4f87703112e8cfe7007cc958ea6e67b70631492705` |
| 当前 A/S0 `commands/a-s0-current-01-audit.json` | `0299c8ab43ef9d5b65798b2141c10f151e56ab56b338d57eb4c1751507db8b3c` |
| 历史 E/S0 `manifest.json` | `c346e4ba4da40161b9ee6777c26eb109ac0f2bb542579093c1787e41d143e3ef` |
| 历史 E/S0 `controller.sqlite3` | `7316555aef9c06fdb3364716d91a29ebba6e1e3f1d3efc7b5d3ec079ad53b6c3` |
| 历史 E/S0 `plant.sqlite3` | `703d732d3f9256946cdb91f12cb369d86dad7d2b5ae6a0f746d990df39130e3e` |
| 历史 E/S0 `feedback_report.json` | `b9e7de08e2bc019993e3ed4f4179deae530fe797a4331a820de81954cda2c85a` |
| 作者 `GSNS_VL.m` | `48d24ad3dca937c7bd42f4dcd2c80adca3c90dca8f30b5afd1bce79c0d408028` |
| 作者 `FindScaleFactor.m` | `3f46dc0882568b1b88ce5416292abd9995b0f414e7b31473ae45d861a9c63c70` |
| 作者 `damped_pinv.m` | `bf039fc0bc65a5eb349c72c7e36989b85375ebb2dbf3c18ab6108978f1bcf46c` |
| 作者 LICENSE | `7e7170e3cebf88a9f60c7b8421418323c09304da1af4d5e90f4da1dc1c8a2661` |
