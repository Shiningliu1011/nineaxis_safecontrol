# 三初态场景证据

数据根目录为
`/home/lsn/robot/robot_safecontrol/output/static-scene-validation-20261020/`。
本轮使用 `static-scenes@78b5626`，生产源码对应 `7b643c4`；
控制参数 `posture_speed_limit=0.02`、周期 100 ms、外部间距 30 mm
和自碰撞间距 10 mm 保持固定。生产源码在完整运行期间保持相同。

## 完整 S0 反事实几何

三份完整 D/S0 基线分别为主仓库的
`output/static-capacity-20261020/lifecycle-s0-01/run`、
本数据根目录的 `s0-initial-2-01/run` 与 `s0-initial-3-01/run`。
各自报告、manifest、两个原始数据库和资源身份已经核查。
初态必须与 `development-scenes-06/catalog.json` 逐项相同。

对每条真实反馈计算 FK，再独立使用 FCL 检查目标障碍物。
`counterfactual-full-initial-{1,2,3}-01/measurements.npz` 保存
全部反馈序号、时间、参考进度、q、各连杆距离与碰撞状态。
`summary.json` 保存基线身份、源码、资源、列标签和每个组合最小值。
逐障碍物的验收使用同目录 `assessment.json`。

| 初态 | 完整反馈条数 | S1 触及要求间距的反馈 | S4 障碍物 1 / 2 触及要求间距的反馈 |
| --- | ---: | ---: | ---: |
| 1 | 87,209 | 9,369 | 9,369 / 46,431 |
| 2 | 87,301 | 9,563 | 9,563 / 46,552 |
| 3 | 87,217 | 9,427 | 9,427 / 46,464 |

三初态的 S1、S4 每个所选障碍物均会与未经避障调整的中间连杆
触及要求间距。零距离使用 FCL 的 collision 状态记录。
该证明只检查固定障碍物作用，不将反事实碰撞当作实际获准运动。

| 初态 | `summary.json` SHA-256 | `assessment.json` SHA-256 |
| --- | --- | --- |
| 1 | `320aa08465a4e5fa9e709604b9680997ddaa8a4e6d38a3ba5324a514e45c15b5` | `e72c1e37d414c305e8a34b6431e202a9b34f9290a07e54ef0eaa75f8eeeda24e` |
| 2 | `5805eab28149ecc93afa7c9b8769f614888a5fa90562416040117a24f97f94a7` | `02d0d5364e7b69c591aebf468ee74c39a3ad603d51ca14c848de807f25c13c38` |
| 3 | `f0a7241643ca641d7ecdf4345af51898b1a9a5f7c2c40a685b39101c0742f3b5` | `c59df631856a74d0d001eebc9edc4fbce85fc217935cafa08af4435a4d70c317` |

原候选锚点附近的局部检查保存在
`development-scenes-06/counterfactual-initial-{2,3}/`；
完整轨迹证据覆盖后续经过障碍物附近的区间。

## S2 用途核查

`development-scenes-06` 的 S2 中，三份当前 S0 基线对障碍物 1
的中间连杆最小距离分别为 53.574383、55.171049、51.521237 mm。
障碍物 2 在三份基线中均触及要求间距。逐障碍物反事实验收仍为
`insufficient_evidence`。

旧完整 D/S2 的全部 Link1–Link9、不同障碍物组合中，
同一候选最接近的最大距离为 78.419063 mm，另一距离为 34.064463 mm。
两条约束同时距离不超过 31 mm 的样本为零。对应状态的 FCL
独立重算与原记录一致，证据为 `s2-simultaneous-alllinks-01/evidence.json`，
SHA-256 为
`bc2d2ee1b7ff51bb0cc0b90ca49eefd666f056b25a913e753945d2a34327b330`。

1,724 个获准运动 trial 的完整 QP 约束按各自实际缩放重新核对；
两条不同障碍物、不同连杆约束同时生效的数量为零，
最小双约束最大 slack 为 0.240552420。
原始问题、解和记录仍保持完整，结果为 `s2-active-constraints-01.json`。
ticket_evidence 对 A/旧 S2 的 2,546 个获准 trial 也取得零共同生效，
其来源记录另见该任务的
`commands/a-s2-current-01-simultaneous.json`。

候选筛选使用原始 25%、50%、75% 候选池，
`s2-candidate-ranking-01/ranking.json` 保存三初态合法性和真实 S0
采样的不同连杆同时干涉证据。候选 7+10、24+28 已完成真实数值全路径，
候选 5+8 因持续暂停超时结束。全部输入、QP、FCL、Ruckig、
状态转换及失败记录均保留。

## 新 S2 开发场景

`development-scenes-07` 采用原 50% 候选 7、10，S4 和其余场景、
控制参数保持相同。三初态的全部 76 组静止连续证书通过，
最小间距余量分别为 7.028814、7.041281、7.019709 mm。
完整工具通道的保守距离下界均满足要求。

| 文件 | SHA-256 |
| --- | --- |
| `development-scenes-07/catalog.json` | `d4f03cb3d47cd76d8d11a44dfe54557e9b112af05d18a3a55d1772d10ac2ced0` |
| `development-scenes-07/s2.yaml` | `2abe4bd8cd8ce09d39f11bfbce76b792b5f11276ba3b7780c864003cbee5d576` |
| `development-scenes-07/s2-selection.json` | `85cd986b5bea1ad0d436ed053b82b76a9307d3f4e56a0302020819377fa8d4f9` |

`s2-screen-7-10-01` 完整数值跟随到达终点。
sequence 1163、116.2 s、参考进度 1.192822215 m，
Link2/obstacle_1、Link5/obstacle_2 的距离分别为
30.043353、30.043717 mm。全过程有 42 个同时距离不超过 31 mm
的样本和 32 个获准 trial 的共同生效约束。独立 FCL 重算与记录一致。
`simultaneous-evidence.json` SHA-256 为
`75540bb9b7e4370b619fe01c9c3046c5bf540f15a6c1d3b9d9a070c5027e3a97`。
完整 ROS 反馈验收仍单独执行。

新 S2 对三份完整 S0 基线重新核查，每个障碍物均有中间连杆触及
要求间距，且存在不同连杆同时触及不同障碍物的实际反事实状态。
所有真实反馈序号、q、进度、逐对象距离与原始身份保存于
`s2-counterfactual-full-initial-{1,2,3}-01/`。

| 初态 | `summary.json` SHA-256 |
| --- | --- |
| 1 | `6a910cd2512dfae054de17716402638b2ff0904b34efbfb0a1de5330da03b5f1` |
| 2 | `794e70962123e04629373efcbafd3ba25eb410790e02b2e2883fb9dbb6bb62d9` |
| 3 | `2f4074d465aef21249413734377651a627c10de0572a9374310a67b5b222152a` |

三初态与三档进给的数值验证计划保存在
`s2-numerical-matrix-01/plan.json`，SHA-256 为
`83f078bef3d11009e20ddc8b1b2f9455ad31656ea78e477ad4446fc656614715`。
九组全部到达终点，每组均出现不同障碍物、不同连杆共同生效的获准
QP trial。各项数值结果和完整 ROS 结果分别登记。
每组的最接近状态均经独立 FCL 重算，与原始记录一致。

| 初态 | 进给倍率 | 同时距离不超过 31 mm 的候选 | 共同生效的获准 trial |
| --- | ---: | ---: | ---: |
| 1 | 0.5 | 91 | 70 |
| 1 | 1.0 | 42 | 32 |
| 1 | 1.5 | 13 | 10 |
| 2 | 0.5 | 91 | 70 |
| 2 | 1.0 | 41 | 32 |
| 2 | 1.5 | 12 | 9 |
| 3 | 0.5 | 91 | 70 |
| 3 | 1.0 | 42 | 32 |
| 3 | 1.5 | 15 | 10 |

共同生效按原始完整 QP 的 `h - G @ solution` 检查，
两条约束均要求 slack 小于 `1e-7`。
`s2-numerical-matrix-01/summary.json` 汇总各组配置、源码、数据库、
候选池、逐对象检查和结果校验值，SHA-256 为
`bc8a5b0f2438bc3a4da698f3adfcd59df48fa0de84b0de70eb7b1aa419f22792`。

## S4 完整运行管理

initial 1/feed 1 已有 `s4-d-paired-witness-01` 完整通过记录和三次恢复。
`s4-ros-matrix-01/initial-1-feed-0.5/run` 完整通过并记录六次恢复。
原队列在 initial 1/feed 1.5 的准备阶段结束，原目录仅保存
`path.npz` 和 `source_configuration.yaml`，结束原因没有充分证据。
原始目录及状态文件保留。

剩余七组由 systemd 用户 service `static-s4-matrix-02.service`
串行运行，CPU 0–7、ROS domain 198，现已全部结束。
最终状态为 `inactive`、`MainPID=0`、`ExecMainStatus=0`；
service 正常结束表示完成队列调度，各实验结果分别保留。检查入口为：

```bash
systemctl --user show static-s4-matrix-02.service -p ActiveState -p MainPID -p ExecMainStatus
```

`s4-ros-matrix-02/queue_manifest.json` 保存原队列身份、完成项、
中断项、固定源码和全部配置。每组均使用实际配置独立重核 S4 witness。
`queue_status.json` 保存各组完整报告身份与实际恢复事件。
运行状态必须结合进程、日志和最终报告核查。

新队列的 initial 1/feed 1.5 在早期触发 200 ms 反馈时效限制。
plant 的 step 1716–1717 存在一次 800.802031 ms 的真实反馈记录间隔，
controller 使用的过期快照为 step 1716。
controller 和 plant ready 时间分别为 22782.518867、22789.255875 s；
首完整周期 52.974901 ms，之前两个周期为 27.430630、41.209577 ms；
最后一个周期因 fault 结束，为 231.512509 ms。
plant 的 `recording_delay` 流为空；源码在同步 append 超过 20 ms
时才写入该流，因此没有记录证据将此长间隔定位到同步记录阶段。
缺少 controller 每条接收时间，现有证据未确定发送与接收调度的具体原因。
原 200 ms 阈值保持固定，后续使用完整预热源码重新验证。

包含记录耗时流的诊断保存在 `s4-fast-feed-age-02.json`，
SHA-256 为 `55619cb618dbd538a12d50ac97df3c455587519c393fdefb04c64df6c7f4c80a`。
原诊断 `s4-fast-feed-age-01.json` 保持原件。

initial 3/feed 0.5 在接近终点时，controller 使用 step 171156
的反馈快照，age 为 318.486835 ms。plant 的 step 171157
同步 append 耗时为 313.047070 ms，记录结束时间
24018.955326660 s；step 171157–171158 的记录时间间隔为
315.165473 ms。该项已有同步记录阶段的直接测量，具体 I/O 原因
尚未确定。全程共有 12 次反馈记录间隔超过 200 ms。
`s4-initial3-slow-feed-age-01.json` SHA-256 为
`412786cdcedb1a286e3f34e6647a01120c158c146e4384b0bff608b5d898c09a`。

这项失败运行在 231.602–235.300 s 期间已观察到六次实际恢复；
每次保持参考进度并出现构型调整。`scenario` 因完整跟随失败直接
返回失败，原报告的 `execution.recovery` 保存了已发生事件。
九组汇总逐项保存完整结果和真实事件，二者分别验收。

| 初态 | 进给倍率 | 完整功能 | 已观察到的暂停、调整和恢复次数 |
| --- | ---: | --- | ---: |
| 1 | 0.5 | pass | 6 |
| 1 | 1.0 | pass | 3 |
| 1 | 1.5 | fail，反馈时效 | 0 |
| 2 | 0.5 | pass | 6 |
| 2 | 1.0 | pass | 1 |
| 2 | 1.5 | pass | 1 |
| 3 | 0.5 | fail，反馈时效 | 6 |
| 3 | 1.0 | pass | 1 |
| 3 | 1.5 | pass | 1 |

`s4-ros-matrix-summary-01.json` SHA-256 为
`5f3f6564169e62b1d5b5544b115eb52c5528aa9a61493159e7cc9fd943859539`。
每组原始 manifest 均保存实际配置下的独立 witness 验证；
队列还单独保存 `witness-recheck.json` 及其校验值。
运行期间其他 agent 在不同 CPU 上执行任务，周期统计作为本轮观测值；
最终共同周期使用集成源码和无并行计算的专用窗口验收。

## 三初态和逐配置可行性

`fixed-scene-geometry-01/geometry.json` 保存全部八个场景条目、
三初态的静止运动、关节范围和逐对象几何，共 24 组。
S3 的三个行程比例均按各自初态计算 J1、J4 上下界，逐项核对
`q + scale * (limit - q)`。S1、S4 全路径工具通道证据与初态合法性
均通过。该目录中的 S2 为 development-scenes-06；
当前 S2 使用 development-scenes-07 的三初态合法性和通道证明。

| 几何证据 | SHA-256 |
| --- | --- |
| `fixed-scene-geometry-01/geometry.json` | `8057074934f28df3507daddfe978f0968fd3a44ec93bdb9d421e1964dc49ae72` |
| `fixed-scene-geometry-01/physical_catalog.json` | `62d25413e30c1bb70696f6c2c345bb88c302fdad7efc981be5529d5c58900007` |
| `development-scenes-07/s2-selection.json` | `85cd986b5bea1ad0d436ed053b82b76a9307d3f4e56a0302020819377fa8d4f9` |

当前 72 个物理配置定义在 `configuration-feasibility-02/configurations.json`，
包含 S0、S1、S2、三种 S3 行程、S4、S5，各自三初态及三档进给。
来源检索覆盖主数据目录全部完整反馈报告，按场景、初态、执行器限制、
路径、运动学身份匹配；完整任务和几何通过且来源校验值一致时纳入。
控制方法和进给各自的功能验收仍单独执行。

最新清单纳入 28 份完整来源，形成 30 个配置的已知完整物理路径证据：

- S0 的三初态、三档进给。
- S1 初态 1、三档进给。
- S3 的三个行程比例、初态 1、三档进给。
- S4 的三初态、三档进给。

A、B、C、D、E 已存在的完整来源均按相同标准检查；
其中 A/S3-40、A/B/S3-70 的已证明可行性完整保留。
新 S2 尚无完整 ROS 反馈证明，S1/S3 的初态 2、3 尚缺完整路径来源。
S5 依据其安全阻塞终止要求验收，完整路径清单中的
`insufficient_evidence` 不替代 S5 的专门结果。

| 最新配置与来源清单 | SHA-256 |
| --- | --- |
| `configuration-feasibility-02/configurations.json` | `18174f4272abe57fc9ca7889503eeaf07a224ef194085cdb17f795cdccec88c6` |
| `configuration-feasibility-02/run_inventory.json` | `a078aae8753f2e6458b52dc3db5ea3f72e3cebce6d45e0321c4ebe611dcd2136` |
| `configuration-feasibility-02/feasibility_matrix.json` | `5df63866c191adde31954fa73d9df400eeeb6bea1b4d7da0329178f9d571fa3c` |

## 后续验收

#177 当前仍需新 S2 的真实 D/S2 完整反馈和 A/B 配对验证，
以及 S4 两项反馈时效失败配置的完整复验。
完整源码和容量守卫通过后，由主会话安排无并行计算的仿真窗口。
当前几何、初态、进给和行程定义均有明确身份；正式运行清单继续等待
共同周期、容量和相关功能要求通过，正式采集保持停止。

本轮检查与运行脚本保存在 `development-scripts-04/`；
`SHA256SUMS` 文件的 SHA-256 为
`3d358d56d97e5f43ef77fa13541dbabdcba060ed0ef369d478636b8f06a354fd`。
最终检查脚本补充归档在 `development-scripts-05/`，23 个文件的
`sha256sum -c SHA256SUMS` 全部通过；清单 SHA-256 为
`f63009e89834c6fe4f5a85fc98367dda563df1e8c6cd3ef51afbd8f012493f70`。
所有历史候选、数值失败、部分目录和完整 ROS 原件保持原位置。
