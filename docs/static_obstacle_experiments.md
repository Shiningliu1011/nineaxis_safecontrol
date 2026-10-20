# 静态障碍物仿真实验

实验五维任务使用同一弧长进度 s 的位置和
工具 X 轴方向；roll 自由。误差导数为 `J_e u - xi_e nu`，包括参考方向和
参考坐标基随 s 的变化。工具轴方向使用球面最短旋转向量，评价使用 0–π
真实夹角；完全反向时方向导数无唯一值，任务无效并终止命令发布。
控制使用本周期参考弧长增量除以周期得到的实际进给，记录中的进给与参考推进一致。

[规格说明书](specs/static-obstacle-experiments.md)定义九轴机械臂在固定障碍物环境中的路径跟随、臂身避让、进给协调和构型调整后恢复。对应的 [GitHub 规格为 #171](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/171)。

## 实施任务

当前接手状态、存储测量与剩余验收见[接手记录](static_obstacle_experiments_handoff.md)。

[任务编号与前置关系](planning/nineaxis-full-project/dependencies.md#静态障碍物仿真实验)列出 61–72 号任务及各自的直接前置，GitHub issue 为 #172–#183。每项任务保存交付行为及验收条件。

可以从 [61 可重复的 S0 仿真与反馈评价](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/172)开始。共同入口完成后，五维参考与外部障碍物接入可以分别推进；A 组完成后，联合求解与改进恢复可以分别推进。

## 实验要求

- 场景包含 S0 基础跟随、S1 单障碍、S2 多连杆受限、S3 行程受限、S4 构型调整后恢复、S5 持续阻塞。
- 方法包含 A/B/C/D 两因素对照和 E 组 Generalized-SNS，全部方法共享场景、限制、构型辅助和评价。
- 位置与工具轴使用同一个路径参考；TCP 作业距离为 50 mm，外部间距为 30 mm，非相邻连杆间距为 10 mm。
- 正式实验采用三组初态、三档名义进给，每个组合重复三次；完整周期测量确定共同控制周期。
- 路径距离 RMS/P95/最大值要求为 0.15/0.5/2 mm；真实工具轴夹角 RMS/最大值要求为 0.05°/0.5°。

## 方法与数学对应

`work.moving_reference_task` 提供同一进度处的五维误差 `e`、Jacobian `J_e`
和参考变化率 `xi_e`。`work.static_velocity_solver.solve_velocity` 为静态实验的
共同速度求解入口；B/D 使用九维速度 `u` 与进给 `nu` 的联合目标：

```math
\|J_e u-\xi_e\nu+K e\|_W^2
+w_p\|u-u_{\mathrm{posture}}\|^2
+w_s\|u-u_{\mathrm{previous}}\|^2
+w_f(\nu-\nu_{\mathrm{cap}})^2.
```

约束包含 `G u <= h` 及 `0 <= nu <= nu_cap`。`nu_cap` 同时受路径给定进给、
共同进给增长率和剩余路径长度限制；求解器使用 `nu / nu_cap` 作为第十个变量。
A/C 用相同的任务和全部限制确定进给，再求解九维关节速度。零进给与最后一个
参考区间按确定的进给重新求解。QP 保存原始目标矩阵、经过行缩放的约束、
变量缩放、结果和 KKT 残差；运动仍经过 Ruckig 连续检查及 plant 准入。

[M09 联合 QP 项目 #157](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/157)
与本实验共享五维误差和联合变量的数学定义。本实验通过
`StaticControl`、`VelocitySolution` 和实验入口验收 A/B/C/D；
M09 的 `LowerResult`、独立及协同模式接口、名义速度策略仍由 #157 单独验收。

## S0 运行与重放

`config/static_experiment_s0.yaml` 统一声明固定初态、路径半径、工件半径、
台面尺寸、运行时限与资源路径。场景使用 `base_link` 坐标，圆柱轴为 +Y；
工件沿轴方向覆盖完整路径并各延长 100 mm，台面位于工件底面。
工件表面与 TCP 路径分别显示。
`external_clearance_m` 和 `self_clearance_m` 分别规定 30 mm、10 mm，
`collision_mesh_manifest` 指定闭合自碰撞模型来源。

```bash
source install/setup.bash
python3 scripts/static_obstacle_experiment.py run \
  --output output/static-experiments/s0-run --domain 198
python3 scripts/static_obstacle_experiment.py replay \
  --output output/static-experiments/s0-run
python3 scripts/static_obstacle_experiment.py run \
  --config config/static_experiment_s1.yaml --solver-mode joint --recovery-mode improved \
  --output output/static-experiments/s1-joint-run
```

输出目录必须尚不存在。运行入口创建独立 ROS controller 与 Ruckig plant，
通过 localhost 内的指定 ROS domain 通信。配置的 `control` 保存 solver_mode、
recovery.mode、控制周期、任务权重、构型辅助和恢复阈值。
`--solver-mode` 和 `--recovery-mode` 显式覆盖计算与恢复方式，
最终值、覆盖参数及相应配置身份进入运行记录，并传递给两个进程。
`--method A|B|C|D|E` 选择规格中的方法组合；与显式计算或恢复方式冲突时拒绝启动。
`--period-s` 选择 20、40、50、100 ms，`--feed-factor` 乘以开发配置的名义进给。
S3 的 `--joint-travel-scale` 选择 1.0、0.7、0.4，实际关节边界写入 manifest。
`--clearance-reserve-m` 为控制约束增加共同的间距余量，几何验收仍采用
30 mm 外部间距与 10 mm 自碰撞间距。控制距离低于该余量时，
CBF 要求正向增加距离；全部方法采用相同的要求。
停止时只处理本次创建的进程，并让已获准运动完成停止过程。
过渡路径只有单个轨迹点时，回放在规定时长内持续发布该点。
`TMPDIR` 与 ROS 日志均指向输出目录。安装后可使用
`ros2 run robot_safecontrol_moveit static_obstacle_experiment` 调用相同入口。

MuJoCo 显示使用同一配置和状态 topic：

```bash
ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 \
  python3 scripts/static_obstacle_experiment.py show
```

运行参数 `path_radius_m=0` 使用生产路径的拟合半径；S0 通过显式 override
采用 270.6 mm。运行快照保存最终参数及其来源。

## 数据与评价

- `manifest.json`：实验配置、完整场景、资源及源文件 SHA-256、执行命令和 Python 身份。
- `path.npz`：控制使用的完整路径、方向、弧长和名义进给数组。
- `plant.sqlite3`：每个 2 ms 仿真积分步的关节位置、速度、加速度，以及实际收到的命令。
- `controller.sqlite3`：全部 QP 输入、候选速度、逐对象查询、运动检查、
  定时命令、准入回复、状态转换和完整周期耗时。
- `result.json`：完成、超时或异常终止原因，以及本次进程退出码。
- `feedback_report.json`、`feedback_report.md`：基于执行反馈的独立评价。
- `feedback_metrics.npz`：逐样本位置、工具轴、参考与投影进度、阶段和误差。
- `feedback_arc_metrics.npz`：按实际有序进度等距采样的误差，未观测区间不补充数据。

新 SQLite 记录使用 MessagePack 双精度数值与无损字典压缩，旧 JSON 数据库
继续可读。格式、转换及验证入口见[紧凑存储](static_experiment_storage.md)。
SQLite 在采样期间持续提交记录，运行异常保留已经写入的数据。
运行期间关闭自动 checkpoint，停止采样后由记录连接执行完整 checkpoint。
监督进程确认所属进程全部退出后，核查数据库完整性并完成剩余 checkpoint。
单次写入超过 20 ms 时另行保存 `recording_delay`，并继续保持原有反馈时效检查。
WAL 文件与数据库共同保存运行中的已提交记录；运行结束前不能单独移动数据库文件。
反馈记录同时保存仿真时间、单调时钟时间和 ROS 消息时间；
仿真时间严格采用 plant 的积分次数，500 Hz 表示仿真采样率。
报告另行列出墙钟采样率和仿真时长。

评价使用每个反馈状态的正运动学。连续局部投影按前一次弧长限制候选区间，
保持路径交叉处的顺序。路径距离、当前控制参考误差、0–π 真实工具轴角度、
TCP 到工件表面的距离分别统计。命令参考与反馈按同机单调时钟关联。
终点位置与方向要求连续保持至少 0.5 s，并单独检查完整有序覆盖。
参考进度使用 plant 结算值，按同一弧长独立插值参考位置和工具轴。
独立执行评价逐积分步关联获准 command_id，并重算有效执行时间对应的进度。
历史记录缺少执行器限制快照时，关节限制及末次限制对象评价明确返回“证据不足”；
已有反馈、几何和进度仍分别评价。
最后一个参考区间使用到达路径终点所需的进给重新求解关节运动，并完成
连续检查和实际执行；参考结算至终点后进入 `ENDPOINT_HOLD`。
完整覆盖要求参考确实结算至末端，实际运动连续经过声明区间，首末位置满足
0.5 mm 终点范围；实际投影距终点的剩余长度单独报告。
关节范围、QP 成功率、完整周期 P99.9 和超期次数参与综合判定。
周期统计包含计算失败和终止阶段。报告分别列出全部拒绝周期及 plant 实际准入拒绝数量。
反馈报告保存 manifest、终止记录、路径、原始数据库及已保存派生数组的 SHA-256。
逐反馈派生数组默认按需生成；`replay --save-samples` 显式保存，绘图入口在
独立中间目录重算并验证指标后使用。转换记录另行绑定来源说明的 SHA-256。
共同周期与正式配对入口核查这些身份后使用报告。

外部间距根据逐条反馈的独立 FCL 查询评价，结果保存为
`clearance_metrics.npz` 和 `clearance_report.json`。自碰撞测量须覆盖全部
36 个组合，覆盖不完整时报告“证据不足”。完整 #171 的结果还包括共享参考、执行检查、
恢复、方法矩阵、场景见证和正式配对实验。
命令退出状态依据任务精度及全部几何检查的共同结果；证据不足返回非零状态。

当前仿真执行器说明见[跟踪平顺性](tracking_smoothness.md)，
已有控制器评价接口见[跟踪评价](tracking_evaluation.md)。

## 固定环境几何

`work.static_collision.StaticGeometry` 使用生成的机器人连杆 OBB、
圆柱工件和台面 box；球形障碍物使用外接 OBB。每个对象组合独立返回名称、
距离、要求间距、最近点和关节梯度。接触、相交和实体包含均明确报告碰撞，
此时不生成距离梯度。

原生模块 `robot_static_fcl` 使用系统 FCL 与 Eigen，查询容差为 `1e-12 m`。
构建依赖由 `package.xml` 声明，包括 `libfcl-dev` 与 `pybind11-dev`。
标准 ROS package 构建会安装该扩展；当前工作区也可以单独构建：

```bash
source install/setup.bash
TMPDIR="$PWD/.scratch/static-experiments/tmp" python3 setup.py build_ext \
  --build-lib install/robot_safecontrol_moveit/lib/python3.10/site-packages \
  --build-temp build/static_fcl
```

几何测试使用独立圆柱距离优化与有限差分检查查询值和梯度。静态实验的控制
限制、处理后的运动检查和反馈距离评价使用这个公共查询接口。

自碰撞采用 `models/ninezzhou/collision_meshes/` 中现有的闭合 STL，
通过 `collision_geometry_sources.json` 和 `provenance.json` 核查文件身份。
运行记录保存清单、URDF、原 mesh、闭合 mesh 和生成器的 SHA256。
trimesh 验证闭合性、方向一致性及凸性，FCL Convex 直接使用已有顶点和三角形。
全部 36 个非相邻组合独立参与计算，最近点梯度使用两个连杆点 Jacobian 的差。
反馈评价用独立 FK 逐条计算这 36 个组合，报告每个组合的最小距离和无效样本数。

## 定时执行与暂停

controller 和 plant 共用 Ruckig 运动处理参数。候选从指定积分步的位置、
速度和加速度开始，包含一个控制周期的运动及之后的停止过程。
检查器利用各关节运动引起的刚体距离变化上界检查连续时间区间；
尚未证明满足限制的区间继续划分，达到检查预算时拒绝候选。
区间按待证明对象分组，批量计算变换与 FCL 距离；父区间中已经证明的对象
在子区间保持有效。接近几何边界时，使用最近点定义的分离平面，
结合刚体一、二阶时间导数及三阶导数余项证明整个区间的间距。
关节范围由 Ruckig 实际分段多项式及其导数根计算，包含转向时刻、
控制周期截取位置和完整停止过程；速度、加速度及 jerk 均核对连续范围。
实际中点违反关节或几何要求时立即拒绝候选；其余候选继续完成全部连续范围检查。
各关节共享 Ruckig 临界时刻的状态计算。
同次控制计算中的九轴速度完全相同时，共享对应的 Ruckig 运动与连续证书；
每次进给试算保存独立 QP 输入、最终状态检查与证书。零进给解的九轴
P、c、G、h 完全一致时，可以重建新问题的进给边界 dual；
完整 KKT 与原始不等式均通过检查后复用该解，记录 dual 与来源 trial。
其余问题执行完整迭代求解。两种分支在启动预热时共同编译。
相邻控制周期的完整 P、c、G、h 与变量比例逐值相同时，复用前次真实解并重新
检查当前完整 KKT 和原始不等式。位置、速度、加速度、周期、执行器限制、
静态几何及目标速度均完全相同时，可共享已验证的 Ruckig 运动与连续证书。
每条 trial 保存本次控制计算序号、仿真时间以及复用来源的序号、时间和 trial。
缓存只保留前次控制计算的数据；参考条件、恢复确认和暂停超时每次照常检查。

controller 根据当前已获准运动预测下个周期起点，plant 独立检查候选，
并在指定的 2 ms 积分步激活。过期、状态不一致、参考不一致及检查失败均拒绝。
plant 在有效执行区间按获准进给结算参考进度；自动停止期间进给为零。
关节位置约束采用不超过 `cbf_alpha` 的接近边界速度比例，并由完整停止运动
检查决定准入。controller 在独立计算线程处理候选，ROS 接收线程持续读取反馈。

基础恢复检查正向候选；改进恢复还检查更严格的位置、真实工具轴角度，
并连续确认至少 0.2 s。暂停时继续计算误差校正及共同构型辅助，
所有方法使用相同的进给渐增参数。持续暂停超过 30 s 时保存终止原因。
当前开发配置的暂停阈值为位置 1 mm、角度 0.2°，恢复阈值为位置 0.3 mm、
角度 0.04°，进给增长率为 0.03 m/s²。任一恢复条件失效都会清除连续确认起点。
`transition` 保存状态转换的仿真时刻与原因；反馈报告在初始启动之后的每次暂停中，
列出首个通过连续检查的正向候选时刻、实际反馈恢复时刻、两者的时间差和随后运行区间。
恢复后的误差按该实际区间统计，初始启动单独识别。
准入拒绝会重新开始恢复确认，持续暂停计时仍然保留。
几何、求解或反馈故障记录 FAULT 状态及具体原因。
QP 进给处于边界的数值误差范围时，固定边界进给重新计算九轴速度，
并重新检查原始完整 QP 的 KKT 与全部不等式。暂停阶段的进给严格为零。

共同构型辅助使用五维任务零空间。归一化关节位置为 z，关节范围宽度为 w，
关节余量项为 `-z / (w * (1 - z² + epsilon²))`；
`posture_joint_margin_fraction` 指定 epsilon，默认 0.05。
该项接近关节边界时增强，最终辅助速度仍遵守共同速度上限。
障碍物间距项与关节余量项共同参与所有方法的构型辅助。

## Generalized-SNS

E 组依据 Amirhossein Kazemipour 等人的
[论文](https://arxiv.org/html/2202.13184)及
[作者公开实现](https://github.com/amrkzp/Generalized-SNS/tree/0d4df38bc735e137a69796dc3ca3396c9de3a616)，
版本为 `0d4df38bc735e137a69796dc3ca3396c9de3a616`，许可为
[CC BY 4.0](https://github.com/amrkzp/Generalized-SNS/blob/0d4df38bc735e137a69796dc3ca3396c9de3a616/LICENSE)。
作者实现中的 `damped_pinv.m` 由 Fabrizio Flacco 编写、Amirhossein Kazemipour 修改。
本项目将 MATLAB 单任务速度算法移植为 NumPy，增加输入和最终限制检查，
接入共同五维任务、构型辅助、运动准入和记录接口；上述来源的作者署名与许可保留。
来源材料按其 LICENSE 中的担保免责条款提供。
Python 实现保留广义速度限制、逐项饱和和最大有效任务缩放语义；
误差校正与路径速度共同乘以任务缩放量。SVD 阻尼参数为
epsilon=0.1、lambda=0.001，秩检查阈值为 1e-6。
广义上下界要求有限且顺序有效。当共同控制余量要求主动增加间距时，
广义速度区间可以完全位于零点的一侧；SNS 按同一饱和过程求解该区间。
没有满足全部限制的结果时抛出求解错误。

共同构型辅助和命令平滑的加权速度在主任务零空间内按允许比例加入，
每个结果仍通过全部限制与连续运动检查。记录包含原始输入、任务缩放量、
任务残差、饱和对象索引及算法迭代次数；QP 指标标记为不适用。
具有独立解析结果的广义限制实例同时使用 SciPy HiGHS 线性规划核查任务缩放。

## 开发场景证据

```bash
python3 scripts/static_obstacle_experiment.py prepare \
  --baseline output/static-experiments/s0-joint-full-geometry-01 \
  --output output/static-experiments/development-scenes
```

入口要求完整 S0 通过任务和全部几何评价，并核查原始记录身份。
输出包含 25%、50%、75% 进度处的障碍候选、三组独立求得的初态、
S0–S5 配置、S3 的三个行程比例和 S5 阻塞证明。
候选分别核查合法初态、完整工具通道和未经避让的连杆距离；
S2 的两个障碍在相同 S0 进度处限制不同连杆。
目录保留每项可行性状态；正式实验须取得对应见证后固定清单。
`prepare --scene-selection <JSON>` 可以分别固定 S1、S2、S4 的候选。
文件记录 `candidate_source_sha256`（`obstacle_candidates.json` 的身份）及
`scenes`，其中 S1/S2/S4 各自包含 `candidate_indices` 和 `reason`。
编号采用候选数组的零起始索引。入口重新核查三组初态；
S2 的两个候选须在相同进度处限制不同连杆。选择理由与结果进入开发清单。

S4 见证请求包含实际运行目录 `source_run`、被拒绝的候选
`candidate_sequence` 与独立调整目标 `goal`。入口核查来源配置和原始记录，
使用固定末端投影及 Ruckig 连续运动证明构型调整，独立求解终态正向候选：

```bash
python3 scripts/static_obstacle_experiment.py witness \
  --config output/static-experiments/development-scenes/s4.yaml --method D \
  --witness-request output/static-experiments/s4-request.json \
  --output output/static-experiments/s4-witness
python3 scripts/static_obstacle_experiment.py witness-replay \
  --config output/static-experiments/development-scenes/s4.yaml --method D \
  --output output/static-experiments/s4-witness
```

运行 S4 时用 `--scene-evidence` 指定见证目录；运行 S5 时指定
`s5_blockage.json`。运行前重新核查见证，验证结果写入 manifest。
S4 还要求实际暂停期间参考保持、关节构型变化及随后恢复。
独立见证可在相同资源的其他检出目录重核。重核按资源角色迁移路径，
继续比较各文件 SHA-256、固定障碍物、碰撞组合和执行器限制；
历史记录缺少 `resource_root` 时，由 collision manifest 的配置相对路径
及其资源绝对路径确定原始目录。原始见证与来源候选保持原有身份。
S5 的阻塞证明允许 2 mm 位置误差和 0.5° 工具轴误差，验收要求持续阻塞后
按规定终止、反馈确认停止、全部间距与关节限制有效，并报告未完成范围。
未提供独立证据时，相应场景返回“证据不足”。

## 共同周期

```bash
python3 scripts/static_obstacle_experiment.py benchmark \
  --config output/static-experiments/development-scenes/s4.yaml \
  --output output/static-experiments/period-study
```

入口串行运行四个候选周期与 A–E 方法的 20 个完整 ROS 仿真，保存每次原始记录、
退出状态及反馈评价。重复调用会检查已有配置、代码和设备环境身份，并继续尚未开始的组合。
`period_report.json` 记录每组 P99.9、最大耗时和超期次数，只有全部方法均
不超过周期 80% 的候选可用；测量缺失和全部候选超时分别报告。
周期通过与功能通过分别保存。

## 正式清单与配对报告

`freeze` 使用开发场景清单、完整周期报告及独立证据索引。
证据索引为 JSON，字段包含 `parameter_rationale`（开发参数选择说明文件）、
`feasibility_runs`（完整跟随的实际运行目录列表）、`s4_witness`
和 `s5_blockage`。参数说明记录搜索范围、选择依据及最终参数。
可行性记录来自其他 worktree 时，依据来源 `resource_root` 重定位到本次资源目录，
并逐项核查碰撞来源文件、全部 mesh、轨迹和运动学配置的 SHA-256。
场景参数、初态、执行器限制和全部自碰撞组合继续参与物理身份比较；
来源 manifest 与原始记录保持原内容。

```bash
python3 scripts/static_obstacle_experiment.py freeze \
  --catalog output/static-experiments/development-scenes/catalog.json \
  --period-study output/static-experiments/period-study \
  --evidence-index output/static-experiments/evidence-index.json \
  --output output/static-experiments/formal
python3 scripts/static_obstacle_experiment.py batch \
  --output output/static-experiments/formal
python3 scripts/static_obstacle_experiment.py paired-report \
  --output output/static-experiments/formal
```

正式清单包含 8 个场景配置、3 个初态、3 档进给、5 个方法和3次重复，
共 1,080 个固定运行编号。启动前核查共同周期、三组初态的实际几何、
软件与设备身份以及源文件。`provenance/sources.zip` 保留代码和模型原文，
`environment.json` 保存软件版本、CPU、内存及线程环境；
归档逐文件校验 SHA-256，并核对 ZIP 内容完整性。

批量运行串行使用同一 ROS 入口，保留每项退出状态和原始数据。
重复调用继续尚未开始的编号，已有运行保持原记录。
`--limit` 限制本次启动数量，剩余组合继续标记为尚未运行。
运行代码、配置或环境改变时拒绝继续同一正式清单。

配对报告分别列出 A/B、C/D、A/C、B/D，差值为后一个方法减前一个方法。
每项连续指标记录实际成对观测数量，全部失败与未完成状态参与方法统计，
完成时间仅用于已经完整完成的运行。D 组功能判定同时检查正式组合覆盖
及预先保存的可行性证据，列出参加功能评价的场景和数量；
缺少证明的配置单独列出。功能与运行时间分别判定，综合结果同时要求两项通过。

## 图表与字体

绘图依赖使用 `requirements-static-report.txt` 中的固定版本。
在包含这些依赖的 Python 环境中运行：

```bash
PYTHONPATH="$PWD/src" python -m robot_safecontrol_moveit.static_experiment_reporting \
  --run output/static-experiments/s0-run --resource-root "$PWD"
```

统一入口也提供 `report --output <运行目录>`，可通过
`--report-python <Python 路径>` 选择已经安装绘图依赖的环境，
通过 `--figures-output <picture/ 内的新目录>` 指定图片位置。

入口生成代表构型、误差、进给与阶段、间距、关节余量和完整周期六类图表，
保存至 `picture/static-obstacle-experiments/`。图表使用 SimSun 和
Times New Roman，检查实际字体名称、字符覆盖和 PNG 文件完整性；
`figures.json` 保存输入记录、字体和图片身份，重复生成时使用新的图片目录。

## 回归验证

`bash run_all_tests.sh` 检查主包、控制内核和 AEB C++ 插件。
控制内核按测试文件启动独立 pytest 进程，以释放 JAX 编译缓存；
全部测试文件和失败状态均进入最终结果。
运行前将 `TMPDIR` 指向仓库内已忽略的中间目录。
