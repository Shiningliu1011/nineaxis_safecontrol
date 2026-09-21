# 统一 scale 碰撞查询研究记录

- 日期：2026-09-21
- 范围：MoveIt/FCL 规划、轨迹回放、JAX OSCBF、LiDAR 点云
- 状态：研究与接口原型；未修改生产控制代码

## 建议采用的统一含义

三阶段共享一个无量纲线性缩放量、同一组机器人包围体、同一份带版本号的场景快照和同一条安全边界：

\[
\texttt{proximity\_scale}(q,t)\ge\texttt{scale\_margin}
\]

- `proximity_scale > 1`：两个未加安全余量的几何体分离。
- `proximity_scale = 1`：几何体接触。
- `proximity_scale < 1`：几何体相交，数值仍保留穿透方向信息。
- `scale_margin >= 1`：安全边界。它表示几何缩放比例，不表示米制距离。
- `h`：内核按其已论证的 barrier 形式返回，安全集合必须与上述 scale 边界相同。
- `cbf_gain > 0`：class-K 增益，进入 `hdot + cbf_gain * h >= 0`。
- `metric_inflation_m >= 0`：传感器误差、体素范围、标定误差、通信延迟和期望米制间距形成的几何外扩量。

当前配置中的 `alpha` 必须改名后才能接入这一接口。[`portable_oscbf/config/dcol_alpha.yaml`](../../../../portable_oscbf/config/dcol_alpha.yaml) 第 8 行用
`alpha = clip(safety_factor * v_95 / d_safe, 5, 30)` 生成 5 至 30 的 CBF 增益。DCOL 论文中的 \(\alpha^*\) 是最小缩放量，两个量仅有名称相同。

建议的生产接口返回：

```text
ScaleQueryResult(
  proximity_scale,
  scale_margin,
  h,
  gradient_h_q,
  time_derivative,
  witness_pair,
  solver_status,
  scene_revision,
  source_stamp,
  geometry_hash,
)
```

同一接口可以有 primitive 专用内核。自碰撞使用 ellipsoid–ellipsoid DCOL；LiDAR 环境使用 ellipsoid–point scale 与保守 `softmin`；规划状态有效性调用同一批函数或数值一致的 C++ 对应函数。统一对象是 `proximity_scale` 的含义和准入判据，不要求所有 primitive 都进入同一个 LP。

这里存在一个必须显式处理的指数差异：DCOL 的 \(\alpha^*\) 是线性统一缩放量；Vessel 论文把 \(s_j^2=p_j^TPp_j\) 记为 \(\alpha_j\)，属于缩放量平方。建议对外返回线性的 `proximity_scale`，并保留各论文已经分析的 barrier：

\[
\begin{aligned}
\text{DCOL:}\quad &h=\alpha^*-\texttt{scale\_margin} \\
\text{Vessel:}\quad &h=s^2-\texttt{scale\_margin}^2
\end{aligned}
\]

两者的安全集合完全相同，QP 使用各自的 `gradient_h_q`。若强制把 Vessel 改成线性 `h = s - scale_margin`，需要重新推导 CBF 条件，不能直接引用原论文证明。HTML 原型明确采用线性归一化形式，只验证状态接口，不代表最终 QP 公式。

## 推荐的几何表示

### 机器人连杆

每个连杆使用数量受控的 ellipsoid 外包：

1. 直杆形状通常使用一个 ellipsoid。
2. 存在明显转折的连杆可以使用两个或三个 ellipsoid。
3. 每个 ellipsoid 都要通过网格顶点、三角面采样和关节极限范围检查，确认外包关系。
4. pair list、primitive slot 数量和参数顺序固定，使 JAX 编译形状稳定。

这条路线减少球体数量，也避开 OBB 面切换对梯度造成的主要问题。Dai 等人对 scaling-function CBF 的连续可微证明要求双方 scaling function 具有强凸性以及连续的一、二阶导数；ellipsoid 和 sphere 满足该条件。polytope 与 OBB 只获得一般连续性结论；“一方强凸、另一方一般凸”的连续可微性在论文中仍是 Conjecture 1。[Safe Navigation and Obstacle Avoidance Using Differentiable Optimization Based Control Barrier Functions](https://arxiv.org/html/2304.08586)

自碰撞使用每个允许 pair 的 ellipsoid–ellipsoid `proximity_scale`，每个 pair 保留独立 CBF 行。相邻连杆和允许接触关系继续由 SRDF/allowed-collision 规则筛选，不能用全局最小值替代 pair 身份。

### LiDAR 环境

建议使用 Vessel 的直接 point scale，并把点云体素化后形成数量上限明确的 support points：

\[
s_{ij}^2=p_{ij}^{B_i\top}P_i p_{ij}^{B_i},\qquad
\alpha_{ij}=s_{ij}^2,\qquad
h_i=\operatorname{softmin}_j(\alpha_{ij}-\beta_i)
\]

这里 \(p_{ij}^{B_i}\) 是第 \(j\) 个环境点在第 \(i\) 个连杆包围体坐标系中的位置，\(\beta_i=\texttt{scale\_margin}_i^2\)。对外的线性 `proximity_scale` 为 \(\min_j s_{ij}\)。每个连杆输出一条聚合 CBF 行；点数影响内核计算量，不增加 QP 行数。`softmin` 取小于或等于普通最小值的形式，使安全判断保持保守。

Vessel 论文用点云直接构造 scaling-factor CBF，给出对 body pose 的连续可微推导，并在 `xdot = u` 速度控制模型下证明 CBF 条件；论文展示了 16-channel 3D LiDAR、10 Hz 感知和四足机器人试验。[Sailing Through Point Clouds](https://arxiv.org/abs/2403.18206)；[官方代码](https://github.com/BolunDai0216/SailingThroughPointClouds)

迁移到多连杆机械臂时需要增加以下条件：

- 每个连杆独立变换点集并计算 `gradient_h_q = gradient_h_pose * pose_jacobian`。
- 自体点必须由当前关节状态与经验证的机器人包围体滤除。
- 点云覆盖范围、遮挡区域和地图 unknown 区域进入准入判断。
- 对称包围点可能使聚合梯度互相抵消；靠近边界时若梯度不可用，OSCBF 停止放行命令。
- 原论文使用整体机器人包围体，无法直接给出机械臂各连杆与自碰撞保证。

体素 centroid 代表一个有体积的 cell。要保持外包关系，可以把体素半对角线、测距误差、标定误差和移动量上界加入机器人 ellipsoid 的保守统一缩放，或用每个 voxel 的完整 cell 做支持函数查询。只检查 centroid 且不增加外扩量会漏掉跨入 ellipsoid 的 cell 边缘。

点云预处理继续保留 occupied、free、unknown 语义。OctoMap 明确区分这三种区域，并提供 `insertPointCloud`、`castRay` 与占据查询；PCL `VoxelGrid` 提供固定 leaf size 的降采样。[OctoMap 文档](https://octomap.github.io/octomap/doc/)；[PCL VoxelGrid](https://pointclouds.org/documentation/classpcl_1_1_voxel_grid.html)

support point 容量超限时采用保守合并、扩大 voxel size 后重新构造或拒绝运动。直接丢弃超出容量的 occupied cells 会破坏安全含义。

### 固定面数 outer polytope 的用途

如果项目要求规划和 OSCBF 调用完全相同的通用凸体优化器，可以把每个 occupied voxel cluster 转成固定法向集合的 outer H-polytope：

\[
b_k=\max_{p\in\text{voxel corners}} n_k^T p+\varepsilon,
\qquad n_k^T x\le b_k
\]

只要法向集合正向覆盖三维空间，该构造形成有界外包凸体；固定面数便于 JAX 固定形状。MoveIt 可以接收同一 polytope 的闭合三角网格，JAX/DCOL 接收 `(A,b)`。

这条路线有三项代价：

- cluster 的合并、分开和身份变化会让几何参数跨帧跳变。
- 凹形障碍物的单个凸外包会占据可通行区域。
- ellipsoid–polytope 的连续可微性证明仍处在 Conjecture 1 范围。

因此，在线 OSCBF 建议采用 ellipsoid–point scale；固定面数 outer polytope 用于冻结的规划场景、回放区间检查和独立验证。两条内核共享 scale 含义、机器人 ellipsoid、占据证据、版本号与准入状态。若“单一方法”要求相同优化器，需接受 polytope 的可微证明边界，并增加梯度连续性试验。

## DCOL `proximity_scale` 的数学含义

DCOL 求解两个凸体在各自 body origin 周围进行统一缩放后首次相交所需的最小比例：

\[
\min_{x,\alpha}\ \alpha
\quad\text{s.t.}\quad
x\in S_1(\alpha),\;x\in S_2(\alpha),\;\alpha\ge0
\]

论文覆盖 polytope、capsule、cylinder、cone、ellipsoid 和 padded polygon。`proximity_scale` 在穿透时仍小于 1，并能从 primal-dual 解通过 Lagrangian/implicit differentiation 得到几何参数梯度。[Differentiable Collision Detection for a Set of Convex Primitives](https://arxiv.org/html/2207.00669)；[作者代码](https://github.com/kevin-tracy/DifferentiableCollisions.jl)

`proximity_scale` 无量纲。同样的 30 mm 间距，在不同尺寸的物体上会给出不同变化量。米制安全量应在几何构造阶段保守外扩，或为每种 shape pair 标定 `scale_margin`。DCOL 论文的 ellipsoid 约束是 `||U Q^T (x-r)||_2 <= alpha`，因此它的 `alpha` 直接线性放大各半轴；`scale_margin = 1.03` 对 DCOL ellipsoid 表示线性尺寸扩大 3%，不能等同于 30 mm。Vessel 的 `alpha_j = s_j^2` 使用平方形式，对应的线性 margin 是 `sqrt(beta)`。

DCOL barrier 的速度级 OSCBF 使用：

\[
\nabla_q\alpha^*(q,t)\dot q+\partial_t\alpha^*(q,t)
\ge -\texttt{cbf\_gain}\,h(q,t)
\]

- 静止障碍物可令 `time_derivative = 0`。
- 有持续身份和速度估计的动态障碍物，把障碍 pose 对 `proximity_scale` 的导数加入 `time_derivative`。
- 原始扫描点没有跨帧身份时，不能把 `time_derivative` 默认为已经覆盖障碍物运动。应使用 `v_max * cloud_age` 外扩并设置 freshness 上限。
- CBF 的正向不变性从 `h >= 0` 的初始状态开始。`h < 0` 的恢复动作需要独立策略。

Vessel 路径在同一个 QP 接口中使用 `h = point_scale_sq - scale_margin**2` 及其导数。规划与回放仍比较线性的 `proximity_scale` 和 `scale_margin`，从而避免把 DCOL 线性 \(\alpha^*\) 与 Vessel 平方 \(\alpha_j\) 当成相同数值。

DCOL 的连续性结论适用于一般凸体；连续可微结论需要更强条件。interior-point solver 在有限容差下产生平滑数值结果属于工程现象，不能扩大论文定理范围。

## 三阶段准入

### 规划

规划请求冻结一个 `scene_revision`，保存 `source_stamp`、`geometry_hash`、occupied/free/unknown 覆盖状态和所有 primitive 参数。状态有效性使用 `proximity_scale >= scale_margin`。起点、终点和中间状态都必须引用同一 revision。

MoveIt 当前仍通过 `/check_state_validity` 与标准 PlanningScene 检查起点和终点，[`motion_planning.py`](../../../../src/robot_safecontrol_moveit/motion_planning.py) 第 56 至 171 行可见该流程。标准 MoveIt FCL adapter 把 `MESH` 变为 BVH mesh，并直接支持 BOX、SPHERE、CYLINDER、CONE、PLANE、OCTREE；其 shape switch 没有暴露 FCL `Convex`、ellipsoid 和 capsule。[MoveIt FCL `collision_common.cpp`](https://raw.githubusercontent.com/moveit/moveit2/main/moveit_core/collision_detection_fcl/src/collision_common.cpp)

FCL 本身支持 ellipsoid、capsule、convex、mesh 与 octree，并提供离散、距离和连续查询。[FCL README](https://github.com/flexible-collision-library/fcl)；[`fcl::Convex`](https://github.com/flexible-collision-library/fcl/blob/master/include/fcl/geometry/shape/convex.h)

要让规划阶段使用相同 `proximity_scale`，需要编写 MoveIt collision/state-validity plugin，把 FCL broad phase 留作候选 pair 生成，narrow phase 调用 scale query。把 ellipsoid 生成三角网格交给当前 FCL 可以共享几何外形，但碰撞量和边界行为仍与 DCOL/Vessel 不同。

点态有效性无法证明采样状态之间的整段关节运动安全。continuous DCOL 的公开形式为固定 orientation 下沿线段平移的凸体增加时间变量 \(\tau\)，没有覆盖关节运动产生的连杆旋转。[Differentiable Continuous Collision Detection for Convex Sets](https://continuous-collisions.github.io/)

本项目需要以下一种经验证的边检查：

- 基于每个连杆最大点速度与 `proximity_scale` Lipschitz 上界的自适应区间验证；或
- 支持刚体旋转和关节插值的连续 scale query。

单纯增加固定采样密度没有可证明的遗漏上界。

### 轨迹回放

当前 [`trajectory_execution.py`](../../../../src/robot_safecontrol_moveit/trajectory_execution.py) 第 65 至 147 行按时间插值并发布轨迹；第 273 行之后只检查 joint 顺序、数值和时间字段，没有读取当前场景并重新检查碰撞。

统一准入要求：

1. 开始回放前，当前 `scene_revision` 与规划 revision 一致，或在当前 revision 上完整复查整条轨迹。
2. 回放期间每个短时间区间重新检查前视 horizon。
3. 新扫描产生 revision 后，旧检查结果失效；完成当前 horizon 复查前发送保持命令。
4. 感知过期、unknown 覆盖进入工作区、scale solver 状态异常或 `h < 0` 时停止放行命令。

### OSCBF

每个控制周期必须同时满足：

- `scene_revision` 与查询输入一致；
- `cloud_age_ms <= freshness_limit_ms`；
- 工作空间 coverage 状态允许运动；
- `proximity_scale`、`h`、`gradient_h_q` 和 `time_derivative` 都为有限值；
- 近边界时 `gradient_h_q` 有可用方向；
- QP solver 通过残差与迭代状态检查；
- collision slack 不超过接近零的审计容差。

当前 [`jax_kernel_factory.py`](../../../../portable_oscbf/work/jax_kernel_factory.py) 第 313 至 352 行在 `relax_cbf` 模式下只用 finiteness 与 solver accepted 判定命令，`delta_slack` 仅进入诊断。统一碰撞 CBF 需要不可放松的 collision rows，或在 collision slack 大于容差时拒绝命令。

## 当前仓库与目标接口的差距

### JAX OBB

[`dpax_collision.py`](../../../../portable_oscbf/work/dpax_collision.py) 第 11 至 14 行明确记录当前内核没有采用 polytope DCOL scale；它计算米制 edge/face distance。第 168 至 180 行仅对分离的 OBB 给出距离，第 296 至 324 行通过 `maximum(..., 0)` 把相交状态饱和到零。结果适合分离区域的距离 CBF，穿透后不再提供有符号深度或 DCOL `proximity_scale`。

当前 self-collision pair 数量和固定八个障碍 slot 也属于现有内核形状约束，不能直接当作全身 pair 完整性证明。

### vendored dpax

[`vendor/dpax/dpax/polytopes.py`](../../../../portable_oscbf/vendor/dpax/dpax/polytopes.py) 第 55 至 96 行实现真正的 polytope `proximity_scale` 及 custom JVP。其 [`pdip_solver.py`](../../../../portable_oscbf/vendor/dpax/dpax/pdip_solver.py) 第 94 至 115 行最多运行 20 次，阈值为 `1e-5`，返回 `(x,s,z)`，没有返回终止原因或残差状态。生产准入还需要显式 solver health。

[`vendor/dpax/dpax/ellipsoids.py`](../../../../portable_oscbf/vendor/dpax/dpax/ellipsoids.py) 第 22 至 48 行用 42 个 Fibonacci normals 把 ellipsoid 转成 polytope，第 71 行之后调用同一 LP；它是 42 面外包近似。仓库检索未找到该文件的调用点或测试。upstream `kevin-tracy/dpax` 公开内容只包含 capsule 与 polytope 路径，因此该文件不能作为 upstream exact ellipsoid DCOL 的证据。[dpax upstream](https://github.com/kevin-tracy/dpax)

### 感知

[`fusion_engine.py`](../../../../portable_oscbf/work/fusion_engine.py) 第 242 至 279 行已经具备 voxel downsample、自体球裁剪、三层占据分类、ESDF 和动态聚类；第 281 至 306 行计算 freshness/status。[`perception_bridge.py`](../../../../src/robot_safecontrol_moveit/perception_bridge.py) 第 676 至 727 行把最多八个动态 track 作为 sphere 同时发布给 OSCBF 与 MoveIt，其中 track slot 的 `alpha = 1.5` 仍是 CBF 增益。

已有感知流程适合继续提供时间戳、占据证据和 track identity。需要补充的统一数据包括 `scene_revision`、coverage、外扩误差分量、primitive 类型、参数、active mask 和 `geometry_hash`。

## Vessel 与点云凸拟合的选择

Vessel ellipsoid–point scale 的特征：

- 场景更新直接使用当前 support points。
- QP 可以聚合为每个连杆一行。
- JAX 输入使用固定点数与 mask。
- ellipsoid point scale 有解析导数。
- 保守性依赖 voxel 范围与误差外扩。
- 长墙会增加点数，同时保留外形。
- 动态物体需要另外构造时间项。
- MoveIt 接入需要自定义 validity checker。
- 主要风险来自遮挡、稀疏点、自体点和梯度抵消。

点云 cluster 转换为 convex primitive 的特征：

- 场景在聚类、拟合和身份维护后更新。
- QP 行数通常为 `link × active primitive`。
- JAX 输入使用固定 slot、固定面数与 mask。
- ellipsoid–polytope 仍有连续可微定理边界。
- enclosing primitive 可以直接外包 occupied cells。
- 长墙形成单个凸体时会覆盖较大空间。
- 持续 ID 可以携带 pose 与 velocity。
- 当前 PlanningScene 可以接收生成的 mesh。
- 主要风险来自 cluster 跳变、凸外包保守和数量超限。

建议把 Vessel 路线作为 OSCBF 环境主查询，把固定面数 outer polytope 作为冻结规划快照与独立验证表示。两者从同一 occupied voxel set 生成，并保留相同 revision。自碰撞统一使用 ellipsoid–ellipsoid DCOL。

## vendored `polytope_proximity` 数值探针

探针输入为两个 axis-aligned box：每个 box 的 half extent 均为 1 m，第一个中心在原点，第二个中心分别位于 `x = 3, 2, 1 m`。运行环境为 JAX 0.6.2、CPU backend、x64。命令：

```bash
PYTHONPATH=portable_oscbf/vendor/dpax python3 .scratch/dcol_alpha_probe.py
```

- 分离状态：中心距 `3.0 m`，`proximity_scale = 1.5000018532`，`dscale/dx = 0.4999995425`，evaluate 稳态中位数 `0.0339 ms`，gradient 稳态中位数 `0.0340 ms`。
- 接触状态：中心距 `2.0 m`，`proximity_scale = 1.0000099556`，`dscale/dx = 0.4999930670`，evaluate 稳态中位数 `0.0287 ms`，gradient 稳态中位数 `0.0289 ms`。
- 穿透状态：中心距 `1.0 m`，`proximity_scale = 0.5000009101`，`dscale/dx = 0.4999912128`，evaluate 稳态中位数 `0.0292 ms`，gradient 稳态中位数 `0.0292 ms`。

三组 value 与 gradient 都是有限值，方向符合 `>1 / ≈1 / <1`。首次 evaluate JIT 为 232.47 ms，首次 gradient JIT 为 133.85 ms。稳态数据是同一 shape 的 100 次调用中位数，不能外推到多连杆、多面数、多 pair 或 GPU 控制周期。

## 必须通过的验证项目

1. 机器人 envelope：每个 link ellipsoid 在整个关节范围内包含目标 collision mesh；统计体积膨胀率和最窄通道损失。
2. scale conformance：JAX 与 MoveIt plugin 对同一几何 corpus 的 `proximity_scale`、contact boundary、pair identity 和梯度方向一致。
3. gradient：有限差分覆盖分离、接触、穿透、近对称、多个最小点和 primitive 切换。
4. perception：自体滤除、稀疏扫描、镜面缺点、遮挡、unknown、延迟、乱序和 sensor frame 变化。
5. dynamic term：已知速度障碍与最坏速度外扩分别验证，禁止把 stale 点当作静止物体。
6. planning edge：验证关节插值整段，并给出可审计的最大遗漏间距。
7. QP admission：正 collision slack、solver residual 超限、非有限 gradient、revision 变化和 freshness 超限都产生保持命令。
8. 计时：记录首次 JIT、稳态 P50/P95/P99、最坏 active point 数、最坏 pair 数和 scene revision 更新耗时。

## 逐项来源

1. [Tracy, Howell, Manchester, DCOL](https://arxiv.org/html/2207.00669)：最小统一缩放优化、支持的六类 convex primitives、穿透状态、contact points 与 differentiable optimization。
2. [DCOL 作者代码](https://github.com/kevin-tracy/DifferentiableCollisions.jl)：完整 primitive 参考实现。
3. [Dai et al., differentiable optimization CBF](https://arxiv.org/html/2304.08586)：`h = alpha* - beta`、pose Jacobian 到 joint gradient、FR3 多连杆 CBF、连续性与连续可微边界。
4. [Sailing Through Point Clouds](https://arxiv.org/abs/2403.18206)：Vessel point scale、`softmin`、连续可微与 velocity-control CBF 论证、LiDAR 试验范围。
5. [Vessel 官方代码](https://github.com/BolunDai0216/SailingThroughPointClouds)：point scale 和聚合 CBF 的参考代码；源码未提供机械臂 per-link、自体滤除、freshness gate 或显式动态点时间项。
6. [Differentiable Continuous Collision Detection](https://continuous-collisions.github.io/)：在线段平移参数 \(\tau\) 上最小化 scale 的公开形式。
7. [FCL](https://github.com/flexible-collision-library/fcl) 与 [`fcl::Convex`](https://github.com/flexible-collision-library/fcl/blob/master/include/fcl/geometry/shape/convex.h)：FCL 支持的几何类型与 proximity query 能力。
8. [MoveIt FCL adapter](https://raw.githubusercontent.com/moveit/moveit2/main/moveit_core/collision_detection_fcl/src/collision_common.cpp)：MoveIt shape 到 FCL geometry 的当前映射。
9. [OctoMap](https://octomap.github.io/octomap/doc/)：occupied、free、unknown 表示和 point-cloud/raycast 接口。
10. [PCL VoxelGrid](https://pointclouds.org/documentation/classpcl_1_1_voxel_grid.html)：体素降采样接口与 leaf size 控制。
11. [dpax upstream](https://github.com/kevin-tracy/dpax)：公开 JAX 版本的范围；仓库 vendored 文件通过本地源码逐行核对。
