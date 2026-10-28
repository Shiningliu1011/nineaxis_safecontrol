# M08 联合 NMPC、导数接口与求解器调查

核验日期：2026-10-27。对应 #150。采用当前九轴、原蝴蝶圆柱、M02/M07、共享 CollisionSafety，以及已经确认的 [M01](decision-137-path-reference.md)、[执行结算](decision-141-progress.md)、[共同数据与时间](decision-142-data-time.md)、[联合 QP](decision-143-joint-qp.md)、[Runtime](decision-151-runtime.md)和[正式验收](decision-144-acceptance.md)。本页记录数学问题、当前接口证据和候选测量；#150 的选型、周期与预算仍待确认。

## 本机来源与可复现输入

主仓库为 `/home/lsn/robot/robot_safecontrol`，Git HEAD 为 `9c354d5505e26aa28b3e6fc323c661bd7542b105`；模型、限制和连续参考为主工作区未提交源码。探针使用 main 的实际公开入口，没有替换模块或连接设备。

| 输入 | 本轮身份与范围 |
|---|---|
| M02 | `NineaxisModel.load`；model hash `4e845d44abb632786cc4f0226d431fd9c97be1abfd5e98721bc17560cf592f0c`。TCP 为 `tool0`，工具局部 `+X`；真实安装验证状态保持原值。 |
| M01 | main 生成的 S0 包 `data/nurbs/butterfly_reference_v1/s0`，identity `58ff1e18b3c7f874f9ec2a3557785d6316bd4a2136bb40b25014d089b91f877e`，总长约 1.816762 m；`load_reference_bundle` 校验来源和连续表达。 |
| 初态 | `config/static_experiment_s0.yaml` 的九轴 `initial_position`；时域探针配置初始速度、加速度和参考导数为零，作为明确的数值问题输入。现场反馈与执行 ledger 尚未由这些探针取得。 |
| M07 | `ProcessLimits.load/bind` 的共同研究配置，保留来源、单位和适用阶段；几何配置身份用于数值绑定，完整场景证明另行取得。 |
| 碰撞 | 当前 `collision_geometry_artifact.json` 的全部 45 个 self link pair。独立导数探针使用明确的解析空环境与数值接口政策，保留查询失败状态；该构型的 `state_valid=false`。 |
| 运行依赖 | 独立 `.scratch/map147-150/venv`：Python 3.10.12，CasADi 3.8.2、其 wheel 中的 IPOPT 3.14.19、JAX/JAXlib 0.6.2、NumPy 1.26.4、SciPy 1.15.3、Ruckig 0.12.2；SRDF/URDF parser 来自 #153 已核验的独立安装。 |
| 计算设备 | Intel Core i7-14700，Linux 6.8.0-138，CPU affinity 0–27，CPU JAX、float64，OMP/OpenBLAS/MKL 线程数分别为 1。没有测量真实设备的运动能力或延迟。 |

源码入口：[nineaxis_model.py](../../../portable_oscbf/work/nineaxis_model.py)、[process_limits.py](../../../portable_oscbf/work/process_limits.py)、[continuous_path_reference.py](../../../portable_oscbf/work/continuous_path_reference.py)、[moving_reference_task.py](../../../portable_oscbf/work/moving_reference_task.py)、[collision_safety.py](../../../portable_oscbf/work/collision_safety.py)。本机原始 JSON、日志、安装记录、探针和 SHA-256 清单位于 `.scratch/map147-150/`；[复现命令](../../../.scratch/map147-150/run_probes.sh)依次运行全部探针，[来源清单](../../../.scratch/map147-150/evidence-manifest.json)绑定报告、源码、依赖和数据。

## 状态、控制与精确区间表达

建议使用运动学 NMPC；状态与控制分别为

\[
x=(q,v,a,s,\nu,b)\in\mathbb R^{30},\qquad
u=(j,w)\in\mathbb R^{10},\quad b=\dot\nu,\quad w=\ddot\nu.
\]

`q/v/a/j` 的第一分量使用 m，其余使用 rad，并分别包含对应时间量纲；`s` 为最终真实弧长的归一化进度。`nu/b/w` 的单位分别为 s⁻¹、s⁻²、s⁻³。模型假设区间内的关节 jerk 与参考 jerk 为常值。在任意区间内部的相对时间 `tau`，使用同一表达重建：

\[
\begin{aligned}
q(\tau)&=q_k+v_k\tau+\tfrac12a_k\tau^2+\tfrac16j_k\tau^3,&
v(\tau)&=v_k+a_k\tau+\tfrac12j_k\tau^2,& a(\tau)&=a_k+j_k\tau,\\
s(\tau)&=s_k+\nu_k\tau+\tfrac12b_k\tau^2+\tfrac16w_k\tau^3,&
\nu(\tau)&=\nu_k+b_k\tau+\tfrac12w_k\tau^2,& b(\tau)&=b_k+w_k\tau.
\end{aligned}
\]

在 `tau=dt_k` 处取得精确递推；这是所选 jerk 模型的解析积分。相邻区间共享 `q/v/a/s/nu/b`，保证 C2。N+1 个节点和 N 个区间完整保存，`UpperPlan` 可从这些分段重建。模型的输入 jerk 与 M09 的目标速度分别具有语义，M10 生成的 Ruckig 运动保留自己的实际分段。

采用 multiple shooting：各节点状态和各区间 jerk 为变量，上述递推为稀疏等式。按共同关节能力、参考进给与预测总时长归一化变量；以 M07 容差归一化任务残差。每一节点分别使用 `s_k` 和 `t_k_ns`，场景、路径和模型身份在一次求解中不可变。绝对时间保存 int64 ns，求解器仅计算相对秒；输出的区间时长转换为正整数 ns，量化后重新核验状态及原始约束。

## 五维任务、工具量与约束

五维误差沿用 M09：三维位置加工具轴球面 log 的两个分量，保留自由滚转，真实轴角另行复核。各节点调用共同模型和 `moving_reference_terms_jax`，连续参考在该节点自己的 `s_k` 求值。任务坐标基随参考变化的导数一并保留。

| 条件 | 数学问题与最终复核 |
|---|---|
| 初始条件 | 从同一有效时刻的 State 固定 `q/v/a/s_ref/nu/b`，保存反馈、估计误差和执行来源；不能以计划起点覆盖实际状态。 |
| 关节与进度 | 共同位置余量、逐轴速度/加速度/jerk，`0<=s<=1`、`nu>=0`；参考 `L*nu`、`L*b`、`L*w` 分别满足 M07 的进给、加速度和 jerk 上限。 |
| 五维任务 | 本阶段的位置与轴向边界分别约束，保存原始 m/rad 残差；恢复、入场、加工及端点使用相应 M07 条件。任务秩采用 #143 的尺度与规则。 |
| 实际工具 | 使用 `tool_motion_jax` 的实际 TCP 速度、加速度与 jerk，包含关节运动引起的 Jacobian 时间变化；完整限制由 M07 根据阶段核验。 |
| 切向工艺 | 路径切向来自实际 TCP 的有效有序投影 `s_tool`。`v_t=T·v_tcp`，其一二阶时间导数包含 `T_dot/T_ddot`。投影的分支、唯一性、事件和有效范围必须明确；预测参考切向与实际投影切向分别保存。 |
| 碰撞节点 | 对共享 CollisionSafety 的每条 primitive 使用 `h_i(q_k,t_k)>=0` 和其 `grad_h_q`，固定身份、容量和活动行。M08 不重新编写几何或碰撞公式。 |
| 节点之间 | 对完整多项式运动、参考、路径事件及停止使用共同区间证明。节点通过仅提供节点证据；未证明区间不得发布为可采用计划。 |
| 未配置参数 | 保留 M07 当前未配置项及取得任务，按实际必需字段拒绝对应模式，不能以零值表示已取得物理证据。 |

M08 的节点碰撞行采用 `h>=0`，其一阶 Jacobian 已由接口提供。若把 `grad_h_q*v+partial_h_partial_t` 再写成 NLP 约束，该约束的一阶导数还需要碰撞二阶导数；当前 self 内核未提供经过核验的完整二阶接口。M09 的瞬时 CBF 行继续使用其已经确认的定义。

M01 的位置为 C2、三阶分段连续。NURBS 节点、轴向事件及停止接续按左右规则核验；第三阶工艺约束可能在事件处改变表达。求解中记录分段与局部坐标，查询跨越事件时重新取得相应导数；事件两侧及区间证明全部保留。当前资料不足以给出跨全部事件的全局光滑 NLP 收敛保证。

## 目标、终点与运行接续

目标由按 `dt_k/T` 加权的归一化五维误差、构型引导、关节运动和 jerk 变化，以及有效进度组成。构型引导保留来自 #140 的明确来源；没有引导时以实际初态和共同关节余量构造研究代价。进度奖励可写为 `-lambda_P*(s_N-s_0)/(nu_scale*T)`；任务、构型与变化项按 #143 的物理尺度建立新配置身份。研究参数必须随目标完整保存，权重数值由 #150/#144 确认，原有未归一化数值不自动取得等价性。

建议将时域末端的 `v_N/a_N/nu_N/b_N` 约束为零，并对到达后的有限保持区间取得有效证据；本轮数值探针采用这个终端条件。若加工终点进入时域，额外约束 `s_N=1`，核验实际覆盖、端点误差和 #138 的保持条件。安全保持需要覆盖当前场景的未来有效区间；动态场景中的无限期保持能力尚无证明。

M08 只发布具有当前来源、约束复核与有效区间的候选。M09 按实际生效时间取得相应区间，使用最新 State 修正；M10 对最终 Ruckig 运动重新协调同区间 C2 参考并检查全部工具、关节、碰撞与停止。M11 的实际采用和执行证据供 M06 结算。参考与关节运动改变后，原计划的未来证明不能直接沿用。

滚动初值按实际物理时间截取上一计划，查询区间内部的状态与 jerk，从最新 State 重新推进剩余控制，并补充经过检查的终端停止或保持。记录实时修正、拒绝、部分执行和三类进度。乘子只有在行身份、尺度及网格关系相容时才重用；数值初值与安全证明分别保存。计划重启、路径/模型/时钟改变或状态不相容时，按共同规则重新初始化。

终端停止、通过的节点及一次优化成功尚不能给出完整闭环的稳定性、递归可行性或全局最优性结论。这些论证需要有效终端集合、误差范围、动态障碍假设及实际执行接续；本轮仅核验明确的数值问题和接口。

## 导数与库能力核验

CasADi 官方 Callback 示例支持提供完整 Jacobian，并要求保持导数 Callback 的对象引用；本轮将 JAX 数值与一阶 Jacobian 经该接口实际传入求解器。[官方 3.8.2 示例](https://raw.githubusercontent.com/casadi/casadi/3.8.2/docs/examples/python/callback.py)

IPOPT 支持 `hessian_approximation=limited-memory`，此时无需提供 Lagrangian Hessian；官方同时说明精确二阶导数与 quasi-Newton 的成本需要依问题判断，warm start 也受活动约束改变影响。[IPOPT Special Features](https://coin-or.github.io/Ipopt/SPECIALS.html) CasADi 官方另有显式 `hess_lag` 入口示例；本轮在真实 FK 小问题中实际接入 JAX Hessian。[官方 sysid 示例](https://raw.githubusercontent.com/casadi/casadi/3.8.2/docs/examples/python/sysid.py)

| 核验 | 实际结果 |
|---|---|
| 九轴加连续蝴蝶的五维任务 | 归一化变量中心差分的 Jacobian 最大差 `1.31e-9`；任务代价 Hessian 最大差 `4.97e-8`（步长 `1e-3`）。CasADi 传递前后的 Jacobian/Hessian 差均为 0。该点在一个连续分段内部。 |
| 45 个真实 self pair | 全部 solver row 收敛；一阶梯度中心差分最大差 `5.35e-10`（步长 `1e-5`）。 |
| self 的直接二次 AD | 与重新求解后的一阶梯度中心差分最大差约 `1.40557`，三个步长保持该量级；当前直接二次 AD 不能作为精确 Hessian。 |
| public query 期限 | 冻结探针 100 次 warm 调用的 host 总耗时 p50/p95/max 为 `5.403/6.726/7.784 ms`；cold 为 3188 ms，完整 156 次查询中有 1 次 `DEADLINE_MISSED`，保留原 9 ms deadline。没有将该场景接纳为安全运动。 |

另一次本机运行的 100 次 warm 调用 p50/p95/max 为 `9.202/14.583/16.293 ms`，完整 156 次查询有 36 次 `DEADLINE_MISSED`。其[工具返回汇总](../../../.scratch/map147-150/initial-query-observation.json)保留该次结果及来源限制；冻结探针的完整来源由上述清单记录。各次本机测量分别保存，没有据此取得完整实时周期的通过结论。

self 内核对内部最优参数调用 `stop_gradient`，其 envelope 一阶导数已经独立检查；完整二阶导数还需要最优参数的变化。JAX 明确说明该操作会阻断嵌套 AD 的梯度。[JAX stop_gradient](https://docs.jax.dev/en/latest/_autosummary/jax.lax.stop_gradient.html) 因此当前完整碰撞问题采用解析一阶导数配合 quasi-Newton 的方向具有接口依据，精确 Hessian 需要独立的数学实现和验收。

## 本机候选测量

时域探针使用同一九轴、S0 连续蝴蝶、五维 log 任务、共同关节/进给上限、实际 TCP 速度节点约束、精确常值 jerk 递推和末端停止。`N=4/8`、`dt=100 ms` 对应 0.4/0.8 s 时域；这些是数值探针规模。全部求解从配置的同一静止初态开始。**完整工具高阶限制、未来碰撞、连续区间证明、真实反馈和最终参考协调尚未包含。**

探针的任务尺度为位置 1 mm、工具轴 0.2°；代价为五维归一化误差平方的节点均值，加 `0.01*mean(z_jerk²)`、`0.02*mean(delta(z_jerk)²)`，减去 `100*(s_N-s_0)/(nu_scale*T)`。这些数值是本轮算法研究设置，完整 M08 权重仍需确认。SciPy 与 CasADi 使用同一数值问题的不同表达，分别复核原始约束，不以 solver 布尔值替代运动准入。

| 形式与后端 | N=4 | N=8 | 检查范围 |
|---|---:|---:|---|
| condensed、JAX 解析一阶、SciPy SLSQP | 15 次；75.0 ms | 23 次；235.8 ms | 原始节点约束最大违反量分别约 `2.84e-13/1.04e-10`；均报告成功。 |
| condensed、CasADi/IPOPT limited-memory | 150 次；1291–1331 ms | 150 次；3426–3794 ms | 达到迭代上限，未报告成功。 |
| multiple shooting、逐节点 Callback、IPOPT limited-memory | 17 次；164–183 ms | 47 次；1176–1221 ms | 160/320 个变量，152/284 个约束；均报告成功，约束违反量约 `1.11e-16`。 |
| multiple shooting、批量 JAX/稀疏 Callback、IPOPT limited-memory | 17 次；71.0–73.4 ms | 150 次；972–1017 ms | N=4 成功；N=8 达到迭代上限。两种规模的 Callback Jacobian 传递差均为 0，约束 Jacobian 分别为 800/1640 个结构非零元素。 |

完整数值结果保存在 [condensed 数据](../../../.scratch/map147-150/nmpc-probe.json)、[multiple shooting 数据](../../../.scratch/map147-150/sparse-nmpc-probe.json)和[批量稀疏数据](../../../.scratch/map147-150/batched-sparse-nmpc-probe.json)。JAX 编译成本和 solver 构建成本分别保存；这些计时范围包含本机 Python Callback、同步和 solver 调用，尚未包含下述全部准入过程。

另一个真实 FK 局部问题中，SciPy SLSQP 通过；IPOPT 的两种 Hessian 形式在所测配置中达到 150 次上限。原始报告保留该结果。该测量说明需要根据实际时域、尺度、图结构、初值和约束判断方法，不能从单次通用 FK 求解推断完整 NMPC 的能力。SciPy SLSQP 的算法和 `ftol/maxiter` 语义采用对应安装版本的官方说明。[SciPy 1.15.3 SLSQP](https://docs.scipy.org/doc/scipy-1.15.3/reference/optimize.minimize-slsqp.html)

数值时域得到的第一周期末速度作为真实 `MotionPlanner/Ruckig` 的目标后，各轴 SI 速度分量差最大约 `1e-17`，位置和加速度仍有可测差异；分量最大差及量纲在 `nmpc-probe.json` 的字段中明确保存。Ruckig velocity interface 的输入包含当前位置/速度/加速度和目标速度/加速度，与预测 jerk 控制分别具有语义。[官方 0.12.2 velocity 示例](https://raw.githubusercontent.com/pantor/ruckig/v0.12.2/examples/05_velocity.py) 该证据支持在 M10 中重新生成与复核实际运动及变速参考。

JAX 测量区分编译、数据准备和同步完成后的求值。官方说明异步 dispatch、首次编译和精度会影响计时；本轮调用 `block_until_ready` 或同步转为 NumPy后记录结果。[JAX benchmarking](https://docs.jax.dev/en/latest/benchmarking.html) 两次重复数据用于核验本机行为，没有从这些次数估计 p99.9 或最坏情况。

## 选型范围与完整预算

数学模型可采用上述 30 维状态、10 维 jerk、C2 分段、multiple shooting、共同五维与原始约束、终端停止及实际执行重新准入。当前导数证据支持以解析一阶导数为共同入口。正式后端和周期保持待测，候选实现必须保存批量评估、稀疏结构、原始残差、迭代、数值初值与失败，不把本轮部分问题的耗时登记为完整周期。

正式冻结需要以下证据：

1. #101/#112/#113/#154 提供逐状态时间、固定行容量及遗漏检查、完整运动和停止证明、逐请求覆盖；当前 `QueryBatch` 和 `certify` 尚不具备全部所需行为。
2. 核验全部实际工具高阶约束的数值/Jacobian、投影导数和事件两侧，接入共同 M07；当前 `ProcessLimits.measure/check` 为 host 接口，完整预测装配尚未测量。
3. #155/#156/#157/#161/#162 核验 State、执行 ledger、参考协调、实际采用、异步更新和停止接续。初值、求解结果、证明与已执行区间分别记录。
4. 在共同正式输入下测量 N、网格和时域组合：预热与首次编译、冷/滚动初值、实际修正、等待/恢复、临近事件、终点、静态/动态障碍、活动行变化、容量和失败。三种方法的准备和重算成本均计入 #144/#169。
5. 对完整链分别记录状态/场景获取、复制/device transfer、排队、预测、实时 QP、Ruckig 与参考、M07、query/certify、发布/下发及预计生效；按实际采用时间检查物理有效区间与来源期限。所选预测/实时周期同时满足整个过程及其争用条件，不能只比较 solver 时间。
6. #119/#169 在目标设备按已确认 #144 每方法保存至少 10000 个完整实时周期，核验 `p99.9_complete<=0.8*T_lower`，以及开发最大耗时、时钟/调度误差和物理有效范围。上层同时保存完整状态装配、导数、求解、原约束复核与发布成本，按 TimingPolicy 和实际生效时间核验。停止和设备时延由已有现场任务测量；solver 超时或失败的结果不可采用，Runtime 按当前有效运动及保护证据处置。

本轮约束问题、导数及 Ruckig 接续测量不包含全项目质量、完整停止、安全闭环或现场能力结论。#150 继续保持开放，待数学问题、实际选型与完整预算的依据充分后，由用户按票面条件确认。
