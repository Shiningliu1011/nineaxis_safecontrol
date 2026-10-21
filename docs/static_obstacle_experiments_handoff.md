# 静态实验接手记录

## 当前状态

工作分支为 `codex/static-obstacle-experiments`，本次接续起点为 `35a1a61`，
当前代码集成为 `6aeb527`，运行源码身份基于 `40d43bb`，包含独立预热、
SNS 最小缩放限制处理、多卷工程预算及运行期间的容量保护。
容量与场景验收记录已集成至当前分支。
#172–#181 已发布逐项证据并关闭；
#171、#182–#183 尚未完成验收。项目已在当前主机恢复到
`/home/lsn/robot/robot_safecontrol`。紧凑存储已完成四组完整真实记录验证，
原始数据保留；正式 1,080 次采集继续停止，验证限定 sim。

`implement-spec` 负责整个 #171 的执行、子任务、集成与最终审查。
Trellis 接手入口是 `.trellis/tasks/10-20-static-obstacle-experiments/`，
正式实验资料位于 `.trellis/tasks/10-20-static-formal-experiments/`；
总任务及子任务对应关系见
`.trellis/tasks/10-20-static-obstacle-experiments/prd.md`。
实现、参数和已有验证记录随本分支保存。

本次完成的 S0–S1、共享参考、全部非相邻 mesh、自碰撞及 Ruckig 准入
逐项证据见
[`research/ticket-acceptance.md`](../.trellis/tasks/10-20-static-obstacle-experiments/research/ticket-acceptance.md)。
当前 S1 原始数据为 `output/static-ticket-evidence-20261020/s1-current-01/`，
运行源码基于 `b6000e8`，87,255 条有效反馈及 1,721 个命令通过评价，
完整周期 P99.9 为 58.207 ms。原始文件 SHA-256 和来源资源路径已随证据保存。

正式容量准入仍未通过。多卷工程预算、运行前后空间检查已集成；
19 组真实记录的编码审计及目标文件系统验证已保存。
运行中的字节额度、空间检查及收尾余量已经完成11项完整验证。
最新两卷规划 `guarded-capacity-review-01/engineering-plan` 的需求为
209,226,444,800 字节，可用 208,146,608,128 字节，缺口为
1,079,836,672 字节；正式启动前继续核查现场空间。
工程预算保持 `formal_bound=false`。计算依据见
[`research/engineering-capacity-validation.md`](../.trellis/tasks/10-20-static-formal-experiments/research/engineering-capacity-validation.md)
及
[`research/capacity-validation.md`](../.trellis/tasks/10-20-static-formal-experiments/research/capacity-validation.md)。
格式 3、S5 计算复用和 Generalized-SNS 广义区间支持已经集成，
58 项组合测试通过，原始记录身份与跨段读取检查通过。
记录生命周期预算及异常收尾已经集成。S4 独立见证与九组数值预核通过；
S4 九组 D 方法完整 ROS 运行均已通过；新 S2 的 D/A/B 完整运行、
E/S4 及 D/E/B S5 验证已经完成。新 S2 的 initial 2、3 完整 ROS
运行待执行。正式采集等待全部前置条件通过。

S0 initial 2、3 的完整 D 组记录及生命周期验证均通过，来源和 SHA-256 见
[`research/s0-initial-configurations.md`](../.trellis/tasks/10-20-static-formal-experiments/research/s0-initial-configurations.md)。
同场景 A/C 的 S4 配对记录分别取得九次、六次恢复，随后均因暂停超时终止；
完整失败记录和配对身份见
[`research/s4-ac-development-pair.md`](../.trellis/tasks/10-20-static-formal-experiments/research/s4-ac-development-pair.md)。

## 实验需要回答的问题

- S0–S5 检查九轴运动的路径覆盖、位置和工具轴精度、外部间距、
  自碰撞间距、关节限制、暂停恢复与持续阻塞后的终止。
- A/B/C/D 对比联合进给求解与改进恢复各自的效果；E 提供 Generalized-SNS 参考。
- 完整周期测量检查求解、运动检查和准入能否满足共同控制周期。

正式矩阵为 8 个场景配置 × 3 个初态 × 3 档进给 × 5 个方法 × 3 次重复，
共 1,080 次。正式数据尚未取得，现有开发结果不构成总体效果或真机能力结论。

## 存储验证与容量

格式 3 使用 SQLite 逐条事务和 MessagePack float64；
逐条比较 zlib level 6 与 zstandard level 3 的无损结果并保存较短项。
zlib 使用 32 KiB 字典，zstandard 使用 128 KiB 字典；
每个 stream 每 128 条独立开始。原始字段、事件和数值精度完整保留，
格式 0/2 数据库继续可读。汇总评价完整计算，逐反馈派生数组按需生成。
转换只写入新目录，报告同时核对来源说明和转换后的数据库身份。
使用方法见[静态实验紧凑存储](static_experiment_storage.md)。

| 完整来源目录 | 原始 MB | 格式 2 MB | 存储验证 | 实验结论 |
|---|---:|---:|---|---|
| `s0-joint-full-geometry-01` | 187.03 | 43.26 | pass | pass |
| `s2-joint-development-01` | 218.38 | 49.32 | pass | pass |
| `s4-joint-development-10` | 206.87 | 46.44 | pass | fail |
| `s5-joint-development-01` | 84.77 | 12.01 | pass | fail |

四组共 331,573 条记录的数量、时间戳、数值位模式、状态和全部字段一致。
原格式与紧凑格式分别重算的指标及派生数组一致；8 次实际 SIGKILL
检查恢复全部已确认提交的记录。源目录前后 SHA-256 一致。
图表按需生成、六类图表导出及来源身份检查通过。

格式 2 另有本机完整 S0 sim 通过：`output/static-storage-control-01`，
有效反馈 87,116 条；1,719 个完整周期的 P99.9 为 68.333 ms，
最大 70.513 ms，100 ms 周期内没有超时。
该目录最终为 43.36 MB，包含 WAL 的运行容量观测峰值为 998.50 MB。
完整命令、环境及文件身份见上述存储验证记录。

按最大完整样本直接乘以 1,080，预计 53.26 GB；按三档进给时长反比外推，
预计 65.10 GB。该外推没有覆盖全部场景与失败时长，不能作为容量上界。
验证观测磁盘分配峰值 1.13 GB，进程 RSS 峰值 0.99 GB；
清理重复迁移资料后，可用空间约 80.99 GB。
直接外推后剩余约 27.72 GB，进给外推后剩余约 15.89 GB；
当前空间可以容纳该样本外推，全部场景的正式容量上界仍需验证。
容量快照见 `output/host-recovery-capacity.json`。

完整读写耗时、各流计数、恢复步骤及原始数据库 SHA-256 见
[本次存储验证记录](../.trellis/tasks/10-20-static-formal-experiments/research/storage-validation.md)。
测量文件为 `output/static-storage-validation-05/validation.json`，
SHA-256：`740eafa49b709b5b70d2d744b854f1cbe1af45d1fc4612181011af78a9005209`。

格式 3 的同四组完整升级共 138,312,784 字节；331,573 条记录、
全部重算指标及三类派生数组位模式一致，八个数据库各 1,200 条
已提交记录的 SIGKILL 恢复通过。数据目录为
`output/static-capacity-20261020/format3-validation-01/`，
`validation.json` SHA-256 为
`5a9dae049d72c8afdcdbc3a93482d83e9b32b04721e4e41e313f9412abb3906c`。
全部编码、重放、随机读取及恢复证据见
[逐条编码验证](../.trellis/tasks/10-20-static-formal-experiments/research/codec-validation.md)。
完整 sim 写入、最新控制字段、WAL 和记录生命周期已完成三项测量；
可执行记录额度和异常退出判定已经集成，完整矩阵容量继续验证。

三项测量使用 `.scratch/worktrees/static-capacity` 的固定源码，
每项 `run/manifest.json` 保存完整 `source_sha256`，数据均在
`output/static-capacity-20261020/`。controller 完成预热后启动 plant，
每个未来执行步最多计算一次，候选目标均严格递增。

| 测量目录 | 反馈 / 候选数量 | 实际分配字节 | WAL 字节合计 | 结果 |
|---|---:|---:|---:|---|
| `lifecycle-s0-01` | 87,209 / 1,721 | 41,381,888 | 965,410,824 | 完整运行通过；P99.9 71.289 ms |
| `lifecycle-s5-01` | 18,384 / 346 | 5,103,616 | 175,977,624 | 按暂停超时终止；P99.9 82.480 ms，周期检查未通过 |
| `lifecycle-timeout-01` | 209,340 / 4,185 | 99,287,040 | 2,327,833,024 | 420 秒压力记录完成，按 timeout 退出 |

三个运行的 worker 退出码均为 `[0, 0]`。压力项采用 0.01 倍进给，
用于验证最长记录过程；其功能结果保留为失败。压力项记录时长为
420.432 秒，进程 RSS 观测峰值为 1,319,043,072 字节。
WAL 数字取两个数据库关闭 checkpoint 前的各自最大值之和。
三项生命周期检查通过，仅证明本次记录过程符合当次检查项，
完整容量准入仍需记录额度执行保证及全部方法、场景的预算证据。

汇总文件为 `output/static-capacity-20261020/lifecycle-series-01-summary.json`，
SHA-256 为 `8b4d32da29a397b5d0eb46b7af77dac008f48800c22098468f282608747f83c6`。
该文件包含每项 validation、manifest、result、两个数据库及反馈报告的
SHA-256、完整命令和资源测量。原始运行与失败结果均保留。

`fa18bea` 集成 worker 记录预算：100 ms 配置下允许开始新记录工作的
最长时段为 473 秒，最多 236,500 条反馈和 4,732 个候选。
运行在额度耗尽时保存 `budget_exhausted` 并停止正式批次；
已经开始的计算完成候选、拒绝、周期和终止记录。实际操作系统调度或正在进行的
写入可能在该时段后结束，该时段约束新工作的开始时间。
ready、记录统计、manifest 和源码身份相互绑定。32 项异常收尾及记录检查通过，
合并检查另有 15 项通过，证据见
[生命周期验证](../.trellis/tasks/10-20-static-formal-experiments/research/lifecycle-capacity-validation.md)。
上述三项完整测量对应各自 manifest 中保存的源码；最新集成源码的完整测量另行保存。

第二个文件系统已验证为可写的 ntfs3，UUID 为 `083CBB753CBB5BFE`，
挂载点为 `/media/lsn/宇宙小机器人`。本任务仅在新目录
`/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/`
保存新数据。`filesystem-validation-02/` 的四组八个数据库共 459,371 条
记录逐字段及 float64 位模式一致；199 次随机读取、八次 SIGKILL 后的
9,493 条已提交记录恢复和 JSON 原子替换均通过。
`validation.json` SHA-256 为
`bbf65a6cba5ea22f466c3759e4a2f6e1eb1f2555d271f87e7d622e93ec10e072`。
本机摘要为 `output/static-capacity-20261020/ntfs-validation-02-summary.json`，
验证结束时该文件系统可用 133,453,279,232 字节。
完整 sim 写入耗时、最长生命周期与两卷容量准入继续验证。

集成 `7b643c4` 在该卷的 `lifecycle-s5-02/` 完整通过：
18,362 条反馈、346 个唯一控制目标，完整周期 P99.9 为 73.627538 ms，
最大 75.175110 ms，超期及命令准入拒绝均为零。持续阻塞安全终止，
worker 退出码为 `[0, 0]`，逐流额度及 ready、统计来源绑定检查通过。
实际分配为 5,193,728 字节，两个数据库 WAL 峰值之和为 175,977,624 字节。
`lifecycle-validation.json` SHA-256 为
`5ef9e26bb342bc44a413e8d92dd98b9c65750ad2473a2eaf976b475ee82bb621`。
源码归档为 `output/static-capacity-20261020/ntfs-lifecycle-source-02-source/sources.zip`，
SHA-256 为 `25f3f268ba1858a554a33e5277e2f1375776a5c92a641e7b008166db87115148`。
相同源码的 `lifecycle-timeout-02/` 已通过 420 秒最长记录检查：
209,690 条反馈覆盖 420.406598 秒，4,192 个控制目标均唯一且递增，
两个 worker 正常退出，记录额度及来源绑定通过。按预期 timeout 结束，
功能结果保留为失败。实际分配 99,647,488 字节，两个数据库 WAL 峰值之和
为 2,331,438,024 字节，进程 RSS 观测峰值为 1,285,644,288 字节。
本次 CPU 8–15 与其他开发计算并行，耗时数据只作为本次观测。
`lifecycle-validation.json` SHA-256 为
`6280ccea9a3a8e59a2d043ba1a2950c86bd51f54a5222d9fe2eebec027cdc80b`。
两个 NTFS 运行的汇总为 `output/static-capacity-20261020/ntfs-lifecycle-series-02.json`，
SHA-256 为 `97c57d5a3b12c7a6788088adb6da7d7fcb1b3db866778a0bcc22897759980046`。

同一运行源码的 A/C/E S5 原始记录位于第二卷的
`lifecycle-s5-{A,C,E}-01/`。A/C 完整通过，分别取得 327/331 个周期，
P99.9 为 51.141/58.287 ms，无超期，按持续暂停超时正常终止。
E 保存为失败：第 155 条 controller 记录中的 `sns_failure` 保存完整问题，
零进给候选的 Generalized-SNS 广义限制检查失败，worker 退出码为 `[0, 1]`。
该真实输入的独立 HiGHS 与饱和算法检查保存于方法任务的
`research/sns-s5-recovery.md`，全部原件保留。
汇总为 `output/static-capacity-20261020/ntfs-s5-methods-01.json`，
SHA-256 为 `cfa4ee54dd16503b88eda6e7dd742851fa0a33de2585841288b60777ac3aab5f`。

源码 `e6fe012` 的 E/S5 完整复验位于同一卷的 `lifecycle-s5-E-02/`，
生命周期、功能及耗时检查均通过。347 个唯一递增候选的周期 P99.9 为
55.320026 ms、最大 58.329657 ms，没有超期或准入拒绝，按暂停超时安全终止。
`lifecycle-validation.json` SHA-256 为
`95723b80bbc7027c8207bfadc5787e9fc8d8e7ac55eb13fedad9796e4e0e2e88`。
汇总为 `output/static-capacity-20261020/ntfs-s5-E-02.json`，
SHA-256 为 `ed5970639054c56c6749d5070beaa7471f1dcaebd0a003d808dc1c8f30b3db62`。
源码归档为该目录下 `ntfs-s5-E-02-source/sources.zip`，
SHA-256 为 `d9965035536bb13a8561f1fa7e9ad6fdbe0ab5d178cc9250df702e175fda9c28`。
本次使用 CPU 8–15，与其他开发验证并行；共同周期仍待最终测量。

## 已有运行与未完成项

| 运行目录（位于 `output/static-experiments/`） | 当前证据 |
|---|---|
| `s0-joint-full-geometry-01` | D/S0 完整跟随通过；路径 RMS 0.058782 mm |
| `s0-sns-full-geometry-01` | E/S0 完整跟随通过；路径 RMS 0.059183 mm |
| `s2-joint-development-01` | 当前参数下 D/S2 综合通过；87,294 条反馈；路径 RMS/P95/最大值为 0.060204/0.123024/0.244570 mm；完整周期 P99.9 为 53.162420 ms |
| `s4-joint-development-10` | 当前保存结果为失败；没有所要求的实际暂停恢复事件 |
| `s5-joint-development-01` | 持续暂停后终止及几何检查通过，完整周期检查失败；原始失败保留 |
| `s4-witness-01` | 已有独立连续构型调整见证；需要结合当前场景和实际运行重新验证 |

`development-scenes-05` 已生成 8 个配置及三组初态。S1 使用候选 0，
S2 使用候选 15、31，S4 使用候选 0、2。当前 S4 完整 sim 在
约 66.9% 进度处持续暂停后终止；正在验证带有来源记录的新候选。
候选编号对应该目录的 `obstacle_candidates.json`，其来源身份保存在选择记录中。

S4 位移组合及来源现保存于
`output/static-scene-validation-20261020/development-scenes-06/`。
`s4-paired-independent-witness-01/witness/` 在该固定几何下重新生成并通过
现有独立核查入口：两段连续调整共 0.32 秒，位置变化上界最大 48.0255 μm、
工具轴变化上界最大 65.6837 μrad，全部关节限制与 76 个碰撞组合通过，
终态正向进给为 0.008879531 m/s。来源是历史真实候选的 q/v/a，
旧数据库、旧场景、旧见证和新重核分别保留身份；当前 D 组实际暂停恢复单独验收。
新 `witness.json` SHA-256 为
`53b77f141b84b4fa889a545d3595da5e9678e56108c2fae35bd4807a360db9ce`，
`verification.json` SHA-256 为
`fd0c98cfe9f948b48487bb351008cae303b347188b9fadc3aa5c403647b52998`。
当前源码的 `s4-d-paired-witness-01/` 完整 sim 已通过，覆盖完整路径与终点，
实际出现三次暂停恢复。三个区间的参考进度范围均为零，最大构型变化均超过
0.0011 rad；1,821 次 QP 检查全部通过。报告 SHA-256 为
`284692060ca10b3e60e2c6eb0cfdfda865faf0407051f1909935654bde1714ae`。
九个初态、进给组合的数值预核全部到达终点并具有途中暂停恢复；
`development-scenes-06/s4-screen-summary.json` SHA-256 为
`80065e82c57c0baf1b0a9ef0ce92b87d7bbbc17a3dfed8186bad43fa67fab579`。
完整 ROS 验证目前覆盖 initial 1/feed 1，其余八组继续运行。
initial 2、3 的 S0 反事实基线同步验证，输出为本数据根目录的
`s0-initial-2-01/run/` 和 `s0-initial-3-01/run/`。

本次 D/S5 完整 sim `output/static-scene-validation-20261020/s5-current-03/`
通过安全终止、几何和运行时间检查。949 个完整周期的 P99.9 为
65.689 ms，最大 66.090 ms，超期次数为零；8,235 次 QP 检查全部通过。
该结果对应当次固定源码；节点调度与全部方法集成后的共同周期另行验证。
来源身份和原始数据库校验值见
[本机场景验证](../.trellis/tasks/10-20-static-scene-witnesses/research/host-scene-validation.md)。

Generalized-SNS 的广义区间与 142 个真实 S1 状态实例通过独立 HiGHS
核查。A/S0 的当前完整运行
`output/static-method-evidence-20261020/a-s0-current-01/` 已通过：
90,622 个有效反馈、1,790 个完整周期、20 个外部对象组合与 36 个自碰撞组合，
P99.9 为 64.694 ms，最大 71.484 ms，超期次数为零。
来源 `659bf7c` 与全部原始文件身份已经核对。
B/C/E 及各方法的其他场景继续验证，具体组合与来源见
[方法验收证据](../.trellis/tasks/10-20-static-method-matrix/research/method-acceptance.md)。

共同构型辅助速度上限为 0.02，开发控制间距余量为 0.1 mm；
其余共同参数与测量范围见
`.trellis/tasks/10-20-static-method-matrix/research/parameter-selection.md`。

后续尚需完成：

1. 完成第三卷的真实记录、随机读取、中断恢复、原子 JSON 与完整 S5
   sim 验证，结合共同周期和现场空间重新确认三卷工程容量准入。
2. 执行新 S2 的 initial 2、3 完整 D 方法 ROS 运行，补充物理可行性证据。
3. S0–S4 对应初态与行程范围的可行性证据。
4. 五种方法、四种候选周期的 20 次完整测量。开发配置的 100 ms 尚未被选为正式周期。
5. 固定正式清单，执行 1,080 次实验，完成配对统计与六类图表。
6. 完成当前源码的本机完整回归、控制周期及规范、需求审查；处理验收问题。

第三卷为 `/dev/nvme0n1p3`，UUID `6C001EAA001E7B72`，挂载目录为
`/media/lsn/Windows`；本任务只使用其子目录
`robot_safecontrol_static_experiments_20261021/`。恢复挂载时可用空间为
77,810,987,008 字节，现有原始数据保留。第三卷的验证和容量准入尚未完成。
已有实验卷 UUID 为 `083CBB753CBB5BFE`，目录为
`/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/`。

`final-regression-01.log`、`final-regression-02.log` 均保留于
`output/static-capacity-20261020/`，两次运行均未生成最终汇总。
当前接续入口为 `.scratch/static-implementation/run_final_regression.sh`，
新日志使用 `final-regression-03.log`。完整通过以三部分最终退出码为准。

最新方法运行与独立 SNS 核查见
[E 方法 S1/S4 开发运行](../.trellis/tasks/10-20-static-method-matrix/research/current-sns-s1-s4.md)。
其中包含实际数据库、报告及来源归档的完整 SHA-256。
两组开发 systemd 队列均已结束。启用容量保护的
`static-guarded-preflight-01.service` 的11项验证已经全部结束；
D/E/B S5、新 D/A/B S2、两项 D/S4 复验及 E/S4 已通过。
420秒写入和预算耗尽检查达到各自预定终止，收尾空间检查通过。
ext4 D/S5 的功能与生命周期通过，时间验收失败。
运行来源、数据位置、
校验值及接续入口见
[容量保护完整仿真](../.trellis/tasks/10-20-static-formal-experiments/research/guarded-preflight.md)。
当前正式实验仍为 `0/1080`。
子任务 #172–#181 已关闭，#182–#183 保持开放。
本轮评论链接为
[#177](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/177#issuecomment-6306806376)、
[#178](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/178#issuecomment-6306812755)、
[#179](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/179#issuecomment-6306820342)、
[#180](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/180#issuecomment-6306825233)、
[#181](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/181#issuecomment-6306830304)。
正式物理输入登记在场景任务的 `research/formal-scene-input.json`，
SHA-256 为 `0f27070df3e6bd707426db43b85ff61d442e93f89aa55af3fdf12a319e7f3c9e`。
当前源码的主包回归为732 passed、1 skipped；控制内核与CTest继续执行。

## 验证状态

- 集成 `371e063` 的格式 0/2/3、迁移、恢复、控制、KKT、SNS、
  场景见证及执行评价共 58 项通过。日志为
  `.scratch/integration-frontier/pytest-format3-scene-sns.log`，
  SHA-256 为 `12d9d66fea71757fd5d7cdd207646f0d59c81e4a981ae6fe8f6d48fe5d324472`。
- 本次四组存储源文件 SHA-256 保持一致，193 次跨段随机读取一致。
  独立核查记录为 `.scratch/integration-frontier/format3-evidence-verification.json`，
  SHA-256 为 `c712efeae280d8f05dac6b81eeb4909dab622baa1800d380f8ee9f6f4e5ef585`。
- 本机 `bash scripts/agent_check.sh` 通过：Python 编译、纯模块导入、
  shell 语法、Git diff 检查及 69 项纯逻辑测试通过。
- 本机 `robot_safecontrol_moveit`、`aeb_rrtstar_ompl`、`pymoveit2` 构建通过，
  原生模块导入和 ROS CLI 入口通过。
- 本机存储、评价、实验管理的 25 项针对性测试通过。
- 本机完整回归通过：主包 678 passed、1 skipped，控制内核 486 passed、
  34 skipped，C++ CTest 4 passed，三个部分退出码均为 0。
- 本机完整 S0 sim 的实际周期和两方面独立审查已经记录；
  正式场景与共同周期的待办保持开放。
- 测试产物隔离的 ROS 用例 14 项、控制内核用例 10 项通过；
  ROS 完整流程另外启用线程异常严格检查，1 项通过且无警告。
  8 份历史基准与报告的 SHA-256 保持一致。
- 本机环境固定 `OPENBLAS_CORETYPE=Haswell`；十个闭合 STL、URDF、
  来源记录和几何生成物的 10 项检查通过。该设置参与环境身份计算。

## 接手所需文件与环境

代码、配置、测试、图表及 Trellis 记录由 Git 分支同步。
当前全部实验记录在笔记本和本次恢复的主机目录
`/home/lsn/robot/robot_safecontrol/output/static-experiments/`，约 3.5 GiB，
由 `.gitignore` 排除，Git 拉取不会传输这些记录。

本机迁移资料已经完成恢复，重复分卷按用户授权删除，释放 26.77 GB。
来源说明、`project-files.json`、`PACKAGE-INFO.json`、`SHA256SUMS`、
环境清单、skills、字体及安装资料保存在
`output/host-recovery-migration-evidence/`。项目、Git 历史、实验记录、
场景选择、S4 见证请求、标定及研究数据保留在恢复后的目录。
三份历史基准产物 `output/baseline_tracking.npz`、`output/oscbf_m6_perf.md`、
`output/oscbf_m8_nullspace.md` 与笔记本原件的 SHA-256 一致；
本机重测结果保存在独立目录。文件原始 SHA-256 与来源位置见
[主机恢复记录](host_recovery_20261020.md#数据身份)。
文件范围和原始恢复方法见[Linux 主机迁移](project_migration.md)。
既有记录中的原始绝对路径和来源身份必须保留；涉及目录移动的评价使用项目的资源重定位接口。

笔记本和台式机通过[双设备工作流程](two_machine_workflow.md)同步当前分支、
Trellis 进度和接手记录；大型原始数据按需要另行传输并验证校验值。

当前运行环境为 Ubuntu 22.04、ROS 2 Humble、Python 3.10，原生几何模块依赖
FCL 与 pybind11。Python 控制依赖见 `portable_oscbf/requirements.txt`；
记录依赖见 `requirements-static-recording.txt`，
绘图环境见 `requirements-static-report.txt`。新机器须重新构建原生模块。
构建与运行入口见[实验说明](static_obstacle_experiments.md)及
[项目运行指南](PROJECT_GUIDE.md)。

新机器的设备身份和运行时间必须重新测量。共同周期或正式清单不能沿用其他机器的
性能结论。所有工作继续限定在 sim。

本机安装、环境入口、字体及资料校验记录见
[2026-10-20 主机恢复](host_recovery_20261020.md)。
