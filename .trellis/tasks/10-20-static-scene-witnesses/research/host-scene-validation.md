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
