# #177–#183 验收状态与正式证据入口

2026-10-21 核对 GitHub 最新正文、comments 和原生依赖。
#177–#183 均为 OPEN、无评论。工作树普通合并至 `567ad0f`，
其中 `6aeb527` 为本轮场景和容量证据集成点；后两项提交仅维护任务与接手资料。
本轮只读取文档、源码及已有 JSON/小型见证文件，没有启动 sim 或重新求解。
提交前普通合并至 `d4938ba`，本轮只修改研究资料和物理输入登记。

## 逐项验收

| Issue | 已有实际证据 | 仍需完成的验收动作 |
| --- | --- | --- |
| #177 | 新 S2 的 D/B 各32条获准双活动约束；S3全部初态的三档双向行程边界；S4独立连续见证及D九组合完整通过；S5几何阻断；三初态合法、候选选位与72配置状态 | 当前物理快照已登记为正式场景输入；主会话审查并发布条款证据。两个S2初态完整ROS用于增加物理证明覆盖 |
| #178 | A/S0、S1等价映射、S3三档及新S2完整通过；S4九次实际恢复、暂停参考保持和关节调整；S5暂停超时安全终止；共同状态与终点测试 | 主会话审查并发布逐项证据；当前没有必须新增的方法开发组合 |
| #179 | B/S0、S1等价映射、S3-100/70、S4及新S2完整通过；B/S5通过；保存全部QP和代表SLSQP；已有同身份A/B配对 | 发布真实问题、配对和边界处理证据；当前没有必须新增的方法开发组合 |
| #180 | C/S0终点、同身份A/C S4配对、六次0.2–0.3秒恢复、C/S5终止；真实RecoveryState的连续确认/角度/拒绝测试；D/E共用规则实际通过 | 审查完整条款索引并发布；当前没有必须新增的方法开发组合 |
| #181 | 作者来源与许可、解析缩放/HiGHS/DLS、E/S0/S1完整跟随、E/S4完整恢复26次、E/S5持续阻塞终止、实际SNS全量限制核查 | 审查来源与真实数值索引并发布；当前没有必须新增的方法开发组合 |
| #182 | A–E公共入口及参数、D完整跟随/受限/恢复、预热和复用来源修复、容量守卫计入周期 | 完成20/40/50/100 ms×A–E的20组共同周期测量，选择全部方法P99.9≤80%周期的最短周期；完成最终参数依据和formal freeze；配合完整回归及最终审查 |
| #183 | 正式1080行生成、记录/评价/图表入口及开发数据的重算验证 | 正式矩阵仍为0/1080；容量准入、冻结和前置验收后执行全部1080项，保留所有失败，完成配对统计、字体/导出检查、报告和交付 |

上述方法组合是各开发票正文要求的覆盖。#179 要求 S3 行程受限开发验证，
现有100%与70%配对满足该项；全部40%正式重复仍属于#183。
E实际已记录的最小 task_scale 为1，缩放小于1由解析和独立数值实例验证。
不能将实际进给变化表述为已经观察到 task_scale 小于1。

A/C 的 S4 最终 `pause_timeout` 保持完整跟随失败，
其9/6次恢复支持基础与改进恢复行为验收。S5的持续阻塞终止符合场景要求。
E/S4历史反馈时效故障、早期E/S5算法故障和B历史终点超期均保留；
当前guarded完整通过的来源另行登记。开发失败进入比较，完整正式结论等待#183。

## 固定的正式场景输入

[formal-scene-input.json](../../10-20-static-scene-witnesses/research/formal-scene-input.json)
登记 dev07 开发清单和 `configuration-feasibility-02/configurations.json`
作为正式场景的物理输入。72项包含每个场景、初态和进给标识的原始配置路径、
配置hash、完整manifest及physical_identity。坐标、初态、S3行程、模型和路径
均沿用该快照。后续正式控制周期、共同参数、方法和重复编号由#182的
`freeze_formal` 写入独立正式目录。物理场景登记不启动正式运行。

本轮重新计算上述两个JSON、当前可行性矩阵、S2选位、S4见证及状态数组、
S5阻断文件的SHA-256，与登记值一致。没有改写原始配置或记录。
登记文件SHA-256为
`0f27070df3e6bd707426db43b85ff61d442e93f89aa55af3fdf12a319e7f3c9e`。
全部72个配置文件hash、manifest引用的原文件hash及有效配置hash也已逐项通过。
独立worktree的 `.scratch/acceptance-frontier/registration-verification.json`
保存本次7份来源和72项配置核查，SHA-256为
`51244b6bf1344a461690f51d4ba208fe600022036c5b63ea0c59d10b488184cb`。
历史 `fixed-scene-geometry-01` 的旧S2不作为当前正式S2来源；
其余合法初态和S3边界证据继续通过当前配置快照关联。

## 33/72 的证明范围

| 场景 | 完整ROS物理路径已证明 | 尚无完整ROS物理路径来源 |
| --- | ---: | ---: |
| S0 | 9 | 0 |
| S1 | 3 | 6 |
| S2 | 3 | 6 |
| S3-100 / S3-70 / S3-40 | 各3 | 各6 |
| S4 | 9 | 0 |
| S5 | 0 | 9；使用专门阻断及安全终止验收 |

72项分成21个不同的已保存physical_identity。S1与S3-100在三个初态下
分别具有相同的物理身份，全部三档进给共享物理路径身份。
这一映射支持物理路径来源复用；各方法、进给和正式重复的功能仍单独测量。

新S2初态2、3各一份完整ROS来源通过并纳入后，可增加6项，成为39/72。
剩余24项S0–S4配置对应六个不同物理条件：初态2、3各自的
S3-100、S3-70、S3-40；100%来源同时匹配S1。
若要让全部S0–S4正式配置预先具备完整路径证明，最少还需这六份完整来源。
规范允许未证明配置明确保持 `insufficient_evidence`；本轮没有把这六份
补充运行列为#177–#181关闭的新增前置。

`paired_report` 对 D 的功能验收包含已证明S0–S4和全部S5。
当前33项对应126次D正式验收，90次D因预先证据不足单列；
S2两项补充通过后为144次与72次。正式矩阵始终保留1080项。
只有预先取得并写入正式清单的证据可以决定该范围。

本轮轻量分组结果保存在独立worktree的
`.scratch/acceptance-frontier/physical-groups.json`，SHA-256
`20454e884aee5052ecb6237c920908ac41cf4447f0a34cd3ca4d430eabea1ba8`；
对应脚本SHA-256为
`a0f2de63c0c4606064b6aa132fab60f6b0b613c00eff94d66d1cd0eddf669922`。
只读取已保存JSON的物理身份，没有重新执行几何或求解器审计。

## formal proof index 的补充方式

生产入口 `src/robot_safecontrol_moveit/static_formal_experiments.py`
的 `freeze_formal` 读取证据索引中的四个字段：

| 字段 | 当前确定的来源 |
| --- | --- |
| `parameter_rationale` | 当前仓库 `.trellis/tasks/10-20-static-method-matrix/research/parameter-selection.md`；共同周期确定后补充最终选择依据 |
| `feasibility_runs` | 当前可行性矩阵中 `complete_run_evidence` 指向的完整ROS运行目录；加入后续新S2通过来源后按目录去重 |
| `s4_witness` | 主仓库 `output/static-scene-validation-20261020/s4-paired-independent-witness-01/witness` 目录 |
| `s5_blockage` | 主仓库 `output/static-scene-validation-20261020/development-scenes-07/s5_blockage.json` 文件 |

该索引使用绝对路径。S5暂停超时记录和纯数值全路径记录分别保留各自用途，
不能作为要求 `completed=true` 的 `feasibility_runs` 项。
完整ROS来源须已退出，`task_verdict`、外部间距和自碰撞间距均通过，
并具有匹配的配置、报告、两份数据库及来源身份。`freeze_formal` 会再次执行
这些身份检查，并按资源重定位后的physical_identity匹配72配置。

现有脚本的衔接如下：

1. 主仓库 `.scratch/static-implementation/run_guarded_s2_initials.py`
   读取 `guarded-scene-evidence-01/pending-s2-initials-plan.json`，
   执行两项有容量保护的D完整运行并写入status/complete和来源归档。
   它没有更新可行性矩阵或生成证据索引。
2. static-scenes worktree的 `.scratch/collect_scene_feasibility.py`
   可生成配置/运行来源/可行性清单，但只扫描主仓库output。
   NTFS上的guarded和新S2来源需要显式加入；仅重新扫描主output会遗漏这些来源。
3. 已归档 `guarded-scene-evidence-01/audit_guarded_scene_evidence.py`
   展示如何验证外卷完整来源并加入矩阵，但输入SHA及输出目录已经固定，
   且包含已完成的独立约束审计。补充两项时采用同样的来源检查步骤写入新目录，
   复用既有审计身份即可；不覆盖旧目录或重复运行全部求解器审计。
4. 将最新矩阵的完整运行目录去重，形成上述四字段证据索引，保存其SHA；
   保留完整inventory、每个原件SHA和数值见证作为研究附件。
   两个既有研究脚本均依赖自身位置推导worktree根目录，归档副本直接运行前
   必须核对其根目录；不能将归档路径作为原执行环境。
5. 共同周期通过后，使用 `static_obstacle_experiment.py freeze`，
   `--catalog` 指向登记的dev07/catalog.json，`--period-study` 指向完整共同周期目录，
   `--evidence-index` 指向新索引，`--output` 使用尚不存在的正式目录。
   核对最终72配置的物理身份与本次登记一致、1080行齐全、各配置证明状态明确。

截至本轮读取时，两项S2及共同周期service均为 `LoadState=not-found`，
对应输出目录尚不存在。入口准备充分；没有把未执行计划计为完成证据。

## 最短依赖顺序

GitHub原生依赖为 `#177 → #178 → (#179；#180 → #181) → #182 → #183`，
#182同时依赖#179和#181，#177的前置#176已经关闭。

先审查正式物理输入登记和#177–#181评论草稿，按原生依赖发布验收。
在root安排的独占窗口完成S2两个初态和共同周期；完整回归03的结果由root登记。
共同周期通过后补充参数依据和证据索引，生成正式清单，按其精确身份重新核查容量。
最终审查与容量准入通过后启动#183，完成全部运行、分析、图表及交付。

最新已提交容量报告的需求为209,226,444,800字节，当时可用为
208,146,608,128字节，差额1,079,836,672字节，状态仍为fail。
十项工程证据通过且 `formal_bound=false`。这份现场快照不能证明之后空间充足；
共同周期和开发记录增加后须重新计算，保留原始数据。

完整运行与独立核查见 [guarded-preflight.md](guarded-preflight.md)，
场景见 [guarded-scene-acceptance.md](../../10-20-static-scene-witnesses/research/guarded-scene-acceptance.md)，
容量见 [guarded-engineering-capacity.md](guarded-engineering-capacity.md)。
逐票评论草稿位于本目录 `issue-177-acceptance.md` 至 `issue-181-acceptance.md`，
本轮没有发布评论或关闭issue。
