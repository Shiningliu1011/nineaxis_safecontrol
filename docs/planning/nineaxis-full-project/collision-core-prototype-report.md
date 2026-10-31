# 碰撞核心算法原型试验

本机测量支持将当前碰撞凸体与 FCL/GJK 距离查询作为后续实现的优先选择，OBB 用于具有后续运动上界的快速排除。该选择涉及现有原生 JAX 几何接口的后端适配；完整任务、完整周期和正式切换仍需要对应验收。

本页属于算法试验记录。需求依据为总体项目 #145/#147、[共同场景决定](decision-139-scene.md)、[公共限制](decision-138-limits.md)、[执行与时间](decision-142-data-time.md)和[验收规则](decision-144-acceptance.md)。本次读取了 GitHub 当前 #147、#145、#139 及其确认评论、#112、#113、#119，当前会话明确排除九组直接相邻 link。

## 复现与原始证据

在仓库根目录运行：

```bash
bash portable_oscbf/scripts/prototype_collision_core.sh --stage all --repeats 2000
```

入口为[试验程序](../../../portable_oscbf/scripts/prototype_collision_core.py)、[本机启动脚本](../../../portable_oscbf/scripts/prototype_collision_core.sh)、[FCL 原型绑定](../../../portable_oscbf/scripts/prototype_collision_core_native.cpp)和[单文件交互报告](../../../portable_oscbf/scripts/prototype_collision_core.html)。程序独立执行数值查询，使用本机已经存在的 Python 和官方 Julia 环境。Clarabel 0.11.1 安装到本原型的 `.scratch/collision-core-prototype/deps`。

原始结果保存在 [`.scratch/collision-core-prototype`](../../../.scratch/collision-core-prototype/)：

| 证据 | 内容 |
| --- | --- |
| [manifest_all.json](../../../.scratch/collision-core-prototype/manifest_all.json) | 源文件、模型及数据 SHA-256、设备、float64、CPU 范围和版本 |
| [inputs.npz](../../../.scratch/collision-core-prototype/inputs.npz) | 68 个关节构型、对应 FK、OBB 位姿和全部 36 组配对 |
| [correctness.json](../../../.scratch/collision-core-prototype/correctness.json) | 包络和碰撞判定比较 |
| [timing.json](../../../.scratch/collision-core-prototype/timing.json) | 每次计时原值、中位数、高分位数、最大值和超时数量 |
| [boundary_cases.json](../../../.scratch/collision-core-prototype/boundary_cases.json) | 接触、相交、包含及毫米间距边界 |
| [distance_oracle.json](../../../.scratch/collision-core-prototype/distance_oracle.json) | 独立凸 QP 的欧氏距离核验 |
| [gradient_checks.json](../../../.scratch/collision-core-prototype/gradient_checks.json) | 关节梯度、左右方向差异及具体对象 |
| [external.json](../../../.scratch/collision-core-prototype/external.json) | 当前工件、工作台、移动球体及时间导数 |
| [continuous.json](../../../.scratch/collision-core-prototype/continuous.json) | 真实 Ruckig 运动、停车、二分和预算结果 |
| [support_capacity.json](../../../.scratch/collision-core-prototype/support_capacity.json) | support 容量和有运动下界的筛选 |
| [raw_stl_coverage.json](../../../.scratch/collision-core-prototype/raw_stl_coverage.json) | 原始 STL 全部三角面顶点覆盖 |
| [official_reference.json](../../../.scratch/collision-core-prototype/official_reference.json) | 官方 DCOL 参照、运行状态及日志 |
| [jax_dcol_qpax_convex.json](../../../.scratch/collision-core-prototype/jax_dcol_qpax_convex.json) | 第二种原生 JAX 求解后端的状态与耗时 |

完整试验入口退出码为 0，包含两种 JAX 后端、全部几何及连续案例。官方完整凸体探针的失败由主程序保留为候选实现的负结果，其子进程退出码为 1。

## 输入与计时条件

- 仓库基线：`a95831b`，测量来自 `/home/lsn/robot-safecontrol` 当前源码与资产。
- CPU：Intel Core i7-14700，进程允许使用 CPU `0,2,4,6`。各计时任务顺序执行。
- Python 3.10、JAX/JAXlib 0.6.2、NumPy 1.26.4、SciPy 1.15.3、FCL 0.7.0、Clarabel 0.11.1；碰撞计算使用 float64。
- 官方 Julia 1.10.12，DifferentiableCollisions 0.1.5，revision `3d29f760a295dbf97ba3c151a8f26bed317e70ce`。
- qpax 包版本 0.1.4，实际加载的 solver 源文件 SHA-256 写入对应结果，避免只凭包版本推断代码。
- 68 个构型包括原配置初态、零位、既有蝴蝶首点 IK 候选、已知相交构型及固定 seed 的 64 个关节范围样本。全部属于数字模型计算构型。
- 36 组非相邻 self pair 全部检查；9 组直接相邻 pair 排除。Self 要求 10 mm，外部要求 30 mm，路径中已经应用的 50 mm 工作距离保留其含义。
- 主查询计时采用 2,000 次预热后调用；耗时较高的 dpax 凸体为 30 次、qpax 凸体和 Clarabel 为 100 次。这些较小样本的高分位数仅用于原型比较。
- JAX 调用在计时结束前同步完成，初次编译单独记录。官方 DCOL 的位姿查询不包含 FK 和关节梯度转换，其计时范围单独标注。

## 几何包络

加载当前 `ninezzhou_collision.urdf` 对应的十个闭合凸体，同时保留现有 OBB。原始 STL 使用 `process=False` 读取，核对全部 2,088,990 个三角面顶点及来源 hash。

凸体平面覆盖的最大顶点超出量为 `4.16e-16 m`，OBB 为 `8.52e-13 m`，均处于本次浮点核验精度范围。凸体顶点也全部包含在 OBB 内，可用于构建保守筛选下界。

OBB 体积为相应凸体的 1.13–2.17 倍。当前完整凸体的 DCOL 半空间表示每个 link 包含 86–716 条面约束；本次没有改变凸体形状或减少其面约束。

| 同一组 68 个构型 | 当前 OBB | 当前凸体 |
| --- | ---: | ---: |
| 满足全部 36 组 self 的 10 mm 间距 | 1 | 65 |
| 同时满足 self 和工件、工作台的 10/30 mm 间距 | 0 | 58 |

OBB 额外判为相交的配对共 137 次；凸体达到 10 mm 而 OBB 没有达到的配对共 213 次，集中于 Link3/Link5、Link5/Link7、Link6/Link8 和 Link7/Link9。这些均属于必须检查的非相邻对象。

原配置初态中，OBB 的 Link5/Link7 与 Link6/Link8 相交；相同构型的凸体最小自碰撞间距为 17.0288 mm。已知真实凸体相交构型中，两种包络均拒绝，凸体命中 Link1/Link9。

这些数量说明当前包络对同一输入的影响，不能解释为完整蝴蝶任务的完成率。

## 碰撞判定与毫米距离

DCOL 采用最小共同缩放量，`alpha > 1` 表示分离。多面体使用 `Ax <= b`，两物体对应四个原始变量及各自平面约束。算法与接口依据为 [DCOL 官方说明](https://raw.githubusercontent.com/kevin-tracy/DifferentiableCollisions.jl/master/README.md)。本次同时使用官方实现、仓库 dpax、qpax 及 [Clarabel](https://clarabel.org/stable/python/getting_started_py/) 核对求解行为。

Clarabel DCOL 与 FCL 在每种几何各 2,448 次 self 配对查询中，相交判定差异均为 0。对当前圆柱和工作台，各 160 次额外查询也没有相交判定差异。凸体外部 SOCP 有 5 次 `AlmostSolved`，其状态必须使相应准入保持无效。

OBB SAT 的 2,448 次相交判定也与 FCL 一致。SAT 的最大投影间隙提供分离与距离下界；精确欧氏距离及 DCOL scale 具有各自的定义。

采用独立六变量凸 QP 最小化两个几何内点的欧氏距离，核对 FCL 的分离距离：

| 几何 | 查询数量 | 最大距离差异 / mm |
| --- | ---: | ---: |
| OBB | 206 | 0.0002093 |
| 凸体 | 216 | 0.000000451 |

全部差异低于本次检查阈值 0.001 mm。该核对使用真实几何约束与成熟求解器，没有替换碰撞结果。

DCOL 的 scale 独立于毫米距离。Self 试验对单位法向平面外扩 5 mm 后检查两个扩展集合分离，形成 10 mm 间距的保守充分条件；OBB 有 13 次额外拒绝，凸体在这些样本上为 0。球体外部 support 通过将球半径增加 30 mm 表达欧氏间距。

## 边界精度与导数

现有 dpax solver 的默认停止条件在 24 个解析接触边界案例中出现 3 次误放行：

| 输入 | 解析 scale | dpax 返回 scale |
| --- | ---: | ---: |
| 相交深度 0.001 mm | 0.9999937500 | 1.0000043466 |
| 相交深度 0.00001 mm | 0.9999999375 | 1.0000105340 |
| 间距为 9.99999 mm | 0.9999999412 | 1.0000014773 |

此处的“误放行”明确指忽略 health 后直接使用 `alpha > 1 + 1e-7`。这些案例用于检验数值准入规则，不表示实体测量精度。Solver 收敛、残差及数值安全界需要共同参与判定。

第二种 JAX 后端 qpax 使用原 DCOL 线性目标、80 次迭代上限及 `solver_tol=1e-7`。OBB 的 2,448 对中有 2,270 对报告收敛；凸体 576 对中只有 62 对报告收敛。其余查询必须保持无效。该后端仍未形成可采用的共同碰撞来源。

FCL 最近点经共同空间 Jacobian 计算关节梯度。在两种步长和左右方向检查后，387 组 OBB、426 组凸体的光滑位置均满足本次 `1e-4` 的混合 SI 坐标导数差异阈值。另有 19 组 OBB、5 组凸体被标记为方向或步长不一致；完整对象和数值保存在结果内。接触特征变化时，单个 witness 的梯度不能直接解释为唯一导数。

移动球体案例明确规定名义速度 20 mm/s、半径增长 1 mm/s，在相同目标时间检查中心和半径。距离时间导数与有限差分的最大差异为 `0.00004584 mm/s`，DCOL scale 时间导数最大差异为 `4.47e-9 s^-1`。该输入没有真实观测资格。

## 查询耗时

单位为 ms。每行计时范围保持其实际含义。

| 实现与工作量 | 中位数 | p99.9 | 最大值 |
| --- | ---: | ---: | ---: |
| OBB NumPy SAT，36 组相交判断，已有位姿 | 0.0900 | 0.0980 | 0.0986 |
| OBB FCL，36 组相交判断，已有位姿 | 0.0064 | 0.0116 | 0.0163 |
| 凸体 FCL，36 组相交判断，已有位姿 | 0.0335 | 0.0491 | 0.0492 |
| 凸体 FCL，36 组距离，已有位姿 | 0.1948 | 0.2347 | 0.2366 |
| 凸体 FCL，FK、36 组距离及关节梯度 | 0.4452 | 0.4985 | 0.6961 |
| 凸体 FCL，36 self + 20 固定外部距离，已有位姿 | 0.2691 | 0.3218 | 0.3244 |
| OBB 官方 DCOL，36 组 scale 及位姿梯度 | 0.2088 | 0.2224 | 0.2251 |
| OBB JAX/dpax，FK、36 组 scale 及关节梯度 | 0.7976 | 1.4019 | 3.0085 |
| 凸体 JAX/dpax，FK、36 组 scale 及关节梯度 | 741.511 | 837.416 | 837.612 |
| 凸体 JAX/qpax，FK、36 组 scale 及关节梯度 | 224.608 | 236.321 | 236.756 |
| 凸体 DCOL/Clarabel，36 组 scale，已有位姿 | 113.277 | 117.786 | 117.793 |

JAX 凸体试验按固定布局重复已有面约束，填充到每个 link 716 条，保留原几何集合。结果受该布局和现有 solver 实现影响；不能推断成所有 DCOL 实现的固有耗时。

官方 OBB DCOL 与独立参考的最大 scale 差异低于 `4e-9`。官方完整 Link3/Link5 凸体共 1,272 条面约束，在本机依赖版本的 StaticArrays 生成函数中出现 `expected call to *`，尚无该组合的有效官方计时。

## 连续运动与停车

使用共享九轴 Ruckig `MotionPlanner` 和 M07 研究速度、加速度及 jerk 限制。对每个区间中点取得真实 q、速度和加速度，通过 Taylor 余项和最大速度分别计算关节变化上界并取交集。Self 的共同上游运动相消，OBB 外包半径形成对当前凸体同样有效的相对运动上界。

FCL 检查“中心距离 − 10 mm − 相对运动上界”。DCOL 对下游面约束增加 10 mm 及相对运动范围后检查扩展集合分离。只有全部配对和全部区间均通过，数字模型证书才返回 `certified`。

| 凸体案例 | FCL | DCOL/Clarabel |
| --- | --- | --- |
| 10 ms 运动 + 30 ms 停车 | 2 个区间通过，最小下界余量 7.006 mm；0.880 ms | 2 个区间通过；227.5 ms |
| 500 ms 运动 + 400 ms 停车 | 36 次区间检查、19 个证明区间通过；12.60 ms | 同样通过；4063.8 ms |
| 仅允许检查一个区间 | `unproven / interval_budget` | `unproven / interval_budget` |
| 前 10 ms 间距有效，随后 126.49 ms 停车违反间距 | 拒绝 | 拒绝 |

连续计时为这些具体轨迹的单次测量，包含区间逻辑，不能用于高分位数资格。900 ms 运动的证明用时已经超过 10 ms，说明 `certify` 的长度、预算和完整周期安排需要独立测量。提前采样用于寻找停车反例，最终通过依据为区间下界。

以上连续试验只覆盖 36 组 self。外部静态与动态几何在本次完成点态和时间导数检查；完整外部 reachable tube 的连续证明仍由相应实现及验收任务完成。

## Support 容量与有证明的省略

容量输入分别来自当前工件尺寸内的规定体积采样，以及 Link9 附近规定范围的密集球体。它们用于几何压力试验，不代表扫描数据或工件完整覆盖。

OBB 到球的距离是凸体到球距离的下界。仅在减去 30 mm、机器人后续 10 ms 的共同最大速度运动界、support 名义运动和半径增长界之后仍为正时，才省略对应组合。

| 256 个 support 场景 | 全部组合 | 保留组合 | 全部查询中位数 / ms | 筛选及查询中位数 / ms |
| --- | ---: | ---: | ---: | ---: |
| 工件体积输入 | 2560 | 36 | 3.695 | 0.278 |
| Link9 附近密集输入 | 2560 | 530 | 5.379 | 1.867 |

16、64、256 个 support 的两类输入共六组，独立逐对距离核验未发现不满足所声明下界的遗漏对象。530 个保留组合仍然需要真实固定容量及 overflow 规则，不能凭筛选降低必须检查的对象数量。

## 对完整任务的选择依据与接入要求

1. 当前凸体覆盖原始 STL，能保留明显更多满足间距的构型，适合作为共同几何来源。使用单个现有 OBB 直接拒绝运动，会显著限制当前机构的可用空间。
2. FCL/GJK 在当前完整凸体上提供足够快的点态欧氏距离；最近点及共同 Jacobian 可以形成 CBF/MPC 所需的一阶信息。逐对象行、状态和身份仍通过统一模块提供。
3. OBB 的角色是带有运动下界证明的快速排除。无法排除的组合继续查询凸体，筛选结果保持原对象身份及有效范围。
4. 实际 Ruckig 和停车段的区间检查是最终运动采用的条件。较长区间、外部 support tube、递归预算及实际完整周期需要继续按 #113/#119/#169 验收。
5. 现有设计包含原生 JAX 查询及 NumPy/JAX 共同消费者。FCL 属于 CPU 原生库，接入涉及后端和 derivative interface 的明确调整；本次没有证明端到端 JIT、二阶导数或完整 MPC 的开销。`proximity_scale` 保持其缩放含义，欧氏距离使用独立字段，barrier 定义与数值设置需要新的 kernel/policy 身份。若原生 JAX 几何计算保持为必须条件，当前测试的实现均没有完整满足该条件及所有数值、周期要求。
6. 本次没有形成完整蝴蝶九轴运动见证，也没有取得实际附件、标定、真实反馈误差、物理停止或目标设备正式资格。#119 与 #169 仍要求完整计时边界、至少 10,000 周期、20% 计算余量及相应原始数据。
