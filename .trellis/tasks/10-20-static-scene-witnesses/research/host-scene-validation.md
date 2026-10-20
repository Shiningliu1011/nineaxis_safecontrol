# 本机 S4/S5 前置验证

## 执行身份

- 工作目录：`/home/lsn/robot/robot_safecontrol/.scratch/worktrees/static-scenes`。
- 初始提交：`35a1a61022d451ba79c5f222d7aef09f8f72ab0c`。
- 环境入口：主仓库 `.scratch/host-recovery/env.sh`。
- Python 控制与 ROS 模块从本 worktree 加载；原生 FCL 扩展使用主仓库
  `build/robot_safecontrol_moveit/src/robot_static_fcl.cpython-310-x86_64-linux-gnu.so`。
- 新实验数据目录：主仓库 `output/static-scene-validation-20261020/`。
- 原始实验和见证保留于主仓库 `output/static-experiments/`。

## 测试边界与身份迁移

测试沿用规格确认的公开边界：场景配置、独立几何见证重核、完整 sim、
原始记录与独立评价。数值与身份回归调用实际资源及公共接口。

历史 `s4-witness-01` 对当前 `development-scenes-05/s4.yaml` 的重核，
在 `verify_reconfiguration` 的场景比较阶段报告
`构型见证的场景或执行器限制不一致`。逐字段检查确认两份场景的障碍坐标、
全部资源 SHA-256、碰撞组合和执行器限制一致；配置身份变化来自开发参数，
资源路径变化来自独立 worktree。

身份迁移复用 `relocate_manifest`。对于未记录 `resource_root` 的历史记录，
依据 collision manifest 的配置相对路径与资源绝对路径识别原始根目录。
迁移保持文件角色、SHA-256、障碍坐标和实际限制；身份重核继续严格比较。
修改范围为资源迁移、见证重核、对应公开接口测试和实验说明。

迁移测试首次运行 `1 failed`（4.65 s），修改后两个迁移测试
`2 passed`（3.96 s）。历史见证的当前参数重核输出为
`s4-witness-current-01/verification.json`，结果 `proven`，SHA-256：
`a47f40ff1e6b9f9f8c38099505343bc2b770978f3ee3c1895d6253cfc80202ba`。
该见证来源进度为 0.8252114850232904 m；本次实际暂停位置另行取证。

## S4 当前参数的完整 sim

`s4-current-01` 使用 `development-scenes-05/s4.yaml` 与 D 方法，
运行退出码为 1，两个 worker 均正常退出。
已经观测六次恢复，前四段暂停的构型变化达到 `1e-4`，
全部暂停区间参考进度范围为零。最终进度为 1.2149316955583627 m
（完整路径约 66.9%），持续暂停后按 30 s 上限终止。
因此完整 S4 验收保持失败。

最终 J1 达到 0.585 m 上限，J9 达到 -1.48353 rad 下限；
Link3、Link4 与 `obstacle_2` 的距离均为 30.1 mm。
独立线性规划在八档进给下均未取得正向解。
固定五维任务的局部可行方向主要通过 J8 运动，仅产生很小的 J1/J9
向范围内部移动量；共同构型辅助的约束解接近零。
原始候选、公开接口探测输出和独立规划结果保存在
`s4-current-blockage-probe.json`。

完整周期 1,547 项，P99.9 为 57.278100 ms，最大 59.281412 ms，
没有超期。QP 1,984 次全部通过最终状态检查；关节限制通过。

## S5 当前参数的完整 sim

`s5-current-01` 使用 `development-scenes-05/s5.yaml` 与 D 方法，
运行退出码为 1，两个 worker 均正常退出。
独立阻塞证明、`task_verdict`、安全终止判定均通过；
实际进度在 0.0037895319972561063 m 后保持，持续暂停超过 30 s 终止。
终止状态速度和加速度为零，阻塞对象为 `Link9/path_block`。

完整周期 639 项，P99.9 为 101.907148 ms，最大 114.028606 ms，
有一次超过 100 ms，运行时间验收失败。该结果作为后续性能修复的
完整仿真失败证据。

最后真实候选的 9 次求解中，第 4、5、7、8 项（从零开始编号）的
九轴速度逐项完全相同。每次求解仍保存独立 QP 输入和最终状态检查；
同一次控制计算可复用完全相同输入对应的 Ruckig 运动及连续证书。
独立 profile、全部慢周期和对应候选编号分别保存于
`s5-profile-01.log`、`s5-current-01-slow-cycles.json`。
新增行为回归针对该真实停止状态，逐次独立重建运动证书并核查
每份 QP 原始约束，`1 passed`（14.85 s）。

性能修改位于 `StaticControl.propose`：在单次计算中使用
`np.array_equal` 比较九轴速度；初态、周期和几何相同且速度完全相同时，
复用已经生成的 Ruckig 运动及连续证书。每次 QP 继续独立求解，
保存全部输入、最终状态检查与 trial。复用范围不跨控制周期，
安全距离、证书与运行时间要求保持原值。

完整仿真前已经合入集成提交 `f3b3658`。运行期间源码保持固定。

`s5-current-02` 完成同配置的第二次完整 sim：885 个周期，
P99.9 为 86.068305 ms，最大 86.145064 ms，超期次数为零；
安全终止与任务判定通过，80 ms 预算尚未通过。

进一步优化保留实际求解得到的 primal/dual。对于零进给，
九轴 P、c、G、h 必须逐值完全相同；新完整问题的进给边界 dual
由 stationarity 计算，使用原 `terminal_qp_health` 重新检查
stationarity、complementarity、primal、dual 非负与有限性。
只有全部 KKT 条件通过才复用；原始 G/h 的 `1e-9` 限制继续核查。
每条 trial 保存自己的问题、完整解、dual、KKT 与复用来源，
复用的迭代次数记为零。完整迭代与验证复用分支在启动时共同预热。
对应真实 S5 状态使用独立 SLSQP 核查目标值、原始约束与运动证书；
增大名义进给后重新取得正向解，核查零进给解没有错误沿用。
控制与求解器相关回归 `24 passed`（105.09 s）。

`s5-current-03` 的完整 sim 退出码为零，两个 worker 正常结束，
`verdict`、`task_verdict`、S5 安全终止判定全部通过。
949 个完整周期的 P99.9 为 65.689195 ms，最大 66.089532 ms，
超期次数为零。8,235 次 QP 检查全部通过，原始约束违例为零，
最大 KKT 残差为 `1.94821183197174e-9`。
完整来源及文件身份保存于 `validation-summary.json`：

- `manifest.json`：`913c16ff782bd0b6d76d587dd468c7bf7af566c7803647ea28788adee0d6441c`
- `feedback_report.json`：`6156b28d1ed0d28f3d4f673d80dd87948f2f3445d6477c0763603edd04d01d9d`
- `controller.sqlite3`：`7252696bbab5d41da7611d32c783346f30f027de3587c26ca1f742c6f1a8e2f8`
- `plant.sqlite3`：`b0cf93e5ef2061550799a9c6314a19d8dfbfd76bbeb44c3c9894adf4d8492669`

该证据覆盖本 worktree 的当前 D/S5 配置。后续调度与 SNS 集成后的
共同周期仍需按最终源码统一验证。

## 开发场景筛查

`screen-*` 是基于原始 `obstacle_candidates.json` 的离线数值仿真，
调用真实控制器、QP、FCL 和 Ruckig，保存每次候选的输入、求解与证书。
此筛查用于选择下一次完整 ROS sim 的开发配置，不能代替反馈验收、
运行时间测量及实际暂停位置的独立连续见证。

`screen-2-xm015` 基于原候选 2，将球心 x 减少 0.015 m；
半径、其他坐标、控制参数保持相同。该数值运行完成全路径，
117.9 s 发生真实候选拒绝，118.3 s 恢复，175.1 s 到达终点；
暂停期间参考保持在 1.218160813622271 m，最大关节变化约 0.003212 rad。
来源候选、位移、配置、脚本与源码 SHA-256 均保存在该目录。
完整 ROS 反馈及实际暂停状态独立见证仍须验证。

`s4-shifted-01` 完整 ROS sim 已完成全路径和终点保持。
实际暂停为 125.602–126.000 s，参考保持在
1.2181602108105354 m，最大关节调整约 0.002347 rad。
1,821 次 QP 检查全部通过。独立构型见证尚不足；
1,759 项完整周期的 P99.9 为 86.613880 ms，最大 124.040000 ms，
有一次超期。最慢候选为 sequence 5521、进度 1.4452153227570903 m，
其连续证书验证 146 个区间。后续最终源码的独占周期测量仍须完成。

位移后场景的三组初态间距余量均超过 7 mm，完整工具通道下界为
297.432953 mm。原始 S0 的 24%–26% 区间共核查 1,603 条真实反馈，
最小 Link5 间距为 26.000725 mm；最接近 25% 且违反 30 mm 要求的
真实反馈为 sequence 27631、进度 0.454311036926233 m，
距离 29.998136 mm。原始反馈身份、逐条结果与位置保存在
`screen-2-xm015/counterfactual-neighborhood.json`。

对应实际暂停候选为 sequence 4723，恢复前同参考进度的目标构型
来自 sequence 4740。直接投影的第一段 Ruckig 运动未通过固定五维任务
证书：位置变化最大 0.177802 mm，角度变化最大 `9.958e-5` rad。
来源状态的最大关节速度为 0.061746 rad/s，TCP 任务速度约 11.66 mm/s。
失败见证和采样诊断完整保存在 `s4-shifted-witness-01` 与
`s4-shifted-witness-start-diagnosis.json`。继续使用原
50 μm、`1e-4` rad 的连续见证要求。

以该真实 sequence 4723 为来源的独立目标搜索保留于
`s4-shifted-witness-search-01/search.json`。最佳首段采样位置变化为
108.117868 μm、轴向变化为 144.288123 μrad，尚未满足连续见证要求。
目标搜索仅用于开发筛查，完整尝试与来源 SHA-256 均保留。

相邻位移 `screen-2-xm014`、`screen-2-xm013`、`screen-2-xm012`
以及保留候选 0 的 `screen-0-2-xm015`、`screen-0-2-xm014`、
`screen-0-2-xm013` 均完成数值路径。进入暂停时仍有实际速度，
下一周期静止状态已经具有正向解。对其中首段采样最接近阈值的
`screen-2-xm014` sequence 1082 生成完整见证，固定轴向条件未通过；
采样位置变化为 35.1067 μm、轴向变化为 130.741 μrad。
全部排名及逐状态来源保存在 `screen-witness-ranking.json`。

后续同物理场景的 C 组可提供实际阻塞状态，作为场景独立见证来源。
该见证须保存真实源状态、独立正向尝试及拒绝原因，并证明连续调整满足
全部限制、终态允许正向进给。D 组全路径、实际暂停与调整恢复保留单独验收；
记录分别列明来源方法及状态，核查场景、初态、执行限制和资源内容一致。

开发脚本与下一轮 C 组候选计划保存于数据根目录下的
`development-scripts-01/`，逐文件校验值见 `SHA256SUMS`，
该清单 SHA-256 为
`2cc3b6bea236432b9614107537dac6ecdc985d1bba1820e423480f59cc15e3df`。
该目录保存当前脚本版本；各次历史运行的实际脚本身份仍以其 manifest
为准。后续 C 组筛查保持候选坐标与限制不变，选择方法写入来源记录。

## 后续 S5 完整周期验证

集成生命周期调度后的 `output/static-capacity-20261020/lifecycle-s5-01/run`
使用真实独占测量，346 个唯一计算目标，P99.9 为 82.480 ms，
最大 83.526 ms，没有超过 100 ms，80 ms 预算仍未满足。
该运行原始 controller SHA-256 为
`e860b84e96da3eff84656340bb2450f8c411956634976d1b78f67cac3f43ac0c`。

真实 candidate 836、839、842 的位置、速度、加速度、参考进度以及
各自九个 trial 的完整问题、解、dual、目标速度、进给上限与运动证书
逐值完全相等。核查结果保存于本任务数据根目录的
`run-repeated-inputs-836-839-842.json`，原始运行 manifest SHA-256 为
`5ac19186aa2c31ee0147138a72759d9d6904b0de41316c2fca21350af96c2c60`。
此证据支持后续评估相邻周期的精确输入复用；每次完整 QP 的 KKT
检查、全部 trial 记录、复用来源周期和恢复计时仍须保留。

跨周期复用已在 `StaticControl.propose` 与 `solve_velocity` 实现。
完整 P、c、G、h 与 variable_scale 逐值相同才提供前次解；
当前完整 KKT 与原始限制每次重新检查。相同真实 q/v/a、周期、
执行限制、静态几何和目标速度允许共享真实 Ruckig 运动与证书。
每条 trial 保留当前计算序号、仿真时间与原始计算来源。
改变参考进度会重新求解，改变加速度会重新生成运动；暂停计时继续执行。

真实 S5 状态的测试 RED 为 `1 failed in 8.62s`，第二周期仍执行
100 次迭代；GREEN 为 `1 passed in 8.81s`。完整受影响回归：
`tests/test_static_velocity_solver.py tests/test_static_control.py`，
`23 passed in 100.89s`。日志保存于数据根目录的
`s5-exact-reuse-{red,green,regression}.log`。
真实 lifecycle S5 最终状态的 20 次控制计算平均为 11.488836 ms；
5 次 cProfile 合计 94 ms，45 个 trial 的求解入口合计 53 ms，
几何查询 23 ms。该开发测量使用 CPU 4–7，其他任务并行；
最终完整周期须在集成源码和独占环境中重新测量。

## 组合场景与见证来源核查

`s4-c-shifted-01` 使用单候选 2 的 x−15 mm 场景，
`s4-c-paired-01` 同时保留原候选 0。两个 C 组完整 sim 均正常结束，
分别约在 57.2% 与 66.1% 持续暂停超时。每组按任务空间速度排序的
前 12 个真实暂停来源，独立 HiGHS/SLSQP 正向解从真实 q/v/a
生成的 Ruckig 运动均通过连续证书，因此这些来源未选为独立调整见证。
实际逐候选来源和全部正向结果保存在各运行目录的
`blocked-source-ranking.json` 与 `actual-source-forward-probe.json`。

`s4-d-paired-01` 使用同一组合场景，已完成完整路径与终点保持，
两个 worker 正常退出。真实暂停来源为 sequence 4675 和 4709，
进度分别为 1.2176743021310366 m 与 1.2205102106525336 m；
实际恢复发生在 125.1 s 和 125.9 s。1,803 次 QP 检查全部通过，
最大 KKT 残差 `7.628431947791194e-10`。缺少对应独立连续见证，
场景验收仍为证据不足。1,750 项完整周期 P99.9 为 78.517197 ms，
最大 83.169468 ms，超期零次；并行负载存在，该结果仅作开发观察。

该 D 运行使用固定 checkout
`/home/lsn/robot/robot_safecontrol/.scratch/worktrees/static-scene-runs`，
提交 `062cd52`，CPU 0–7。原始数据位于主仓库 output 下，
resource_root、源码及文件身份保存于 manifest 和 validation-summary。
native 依赖来自主仓库 build；运行及评价期间源文件保持相同。

旧 `s4-witness-01` 来源为 `s4-joint-development-01` sequence 3893，
进度 0.8252114850232904 m。原 controller SHA-256 为
`dfd7c1b2969cec6547e73ffaf6aab40ecf74cf3f58357f3a8e185f381e616d58`。
该来源 J4 距上限约 `4.00124e-13` rad，速度约 `9.21147e-12` rad/s。
在原场景与当前组合场景中，独立 LP 都得到 0.008879531 m/s 正进给；
从真实 q/v/a 生成对应 Ruckig 运动时，两者均被 `joint_position` 拒绝。
该结果只说明已计算运动的执行限制，未声称所有连续控制全局无解。
结果与来源身份保存在 `s4-old-source-physical-probe-01/report.json`。

当前组合 D 在相同进度附近的 sequence 3071 与旧来源构型不同，
全程 J4 距上限至少 0.031947726 rad。
`old-witness-neighbor-comparison.json` 保存逐关节差异和两个数据库身份。
已核对配置参数差异为 `posture_speed_limit` 的 0.1 与 0.02；
来源源码身份亦不同，参数差异未被认定为唯一原因。
后续共同 0.1 参数开发验证用于核查当前源码行为。
候选确定后仍需三初态与三档进给的完整功能验证，以及每组实际初态的
S0 反事实连杆干涉、工具通道和初态合法性证据。
