# OBB 自碰撞与环境实体几何：上游接口核验

日期：2026-09-08。对应「自碰撞 OBB 与环境几何模型核对（研究）」；本文件是上游源码研究，不是生产替换决议或本机测试报告。复用[环境点云研究](environment-pointcloud-collision.md)中的双源、未知区和动态地图结论，这里只补充几何接口、误差预算和导数边界。工作树：`.scratch/oscbf-reuse-wayfinder/8/research-worktree`；研究分支：`codex/research/obb-environment-geometry`。

## 候选与版本

保留自碰撞 OBB 这一表示选择；推荐对照 FCL 实体 Box–Box 内核与当前距离实现。环境优先验证同一机器人 OBB 对局部占据实体盒的距离，可用 Box–OcTree 加速，也可用逐叶 Box–Box 作可解释参考。改变距离内核不等于改变机器人包络；是否外包围真实连杆、夹具和 TCP 需要本地模型审计。

复核版本是 FCL **0.7.0**、PCL **pcl-1.12.1**、OctoMap **v1.9.7**；不是声称系统已经安装或适配。FCL/PCL/OctoMap 库许可证为 BSD 三条款，发布时保留版权、条件及免责声明，不能用贡献者名义背书；完整传递依赖和实际构建版本仍需冻结清单。[FCL LICENSE](https://raw.githubusercontent.com/flexible-collision-library/fcl/0.7.0/LICENSE)、[PCL LICENSE](https://raw.githubusercontent.com/PointCloudLibrary/pcl/pcl-1.12.1/LICENSE.txt)、[OctoMap 库 LICENSE](https://raw.githubusercontent.com/OctoMap/octomap/v1.9.7/octomap/LICENSE.txt)。

## FCL 距离、接触与符号必须分开验收

`DistanceRequest` 默认不求最近点、不启用符号距离，采用 `GST_LIBCCD`，`distance_tolerance=1e-6`。分离态提供正距离；相交而未启用符号时结果由实现决定，不能把负值直接解释为米制穿透深度。官方表区分 primitive＋LIBCCD 的表面点保证，以及 INDEP、mesh/octree 的 NP_X 限制。[DistanceRequest 0.7.0](https://raw.githubusercontent.com/flexible-collision-library/fcl/0.7.0/include/fcl/narrowphase/distance_request.h)。

Box–Box 是实体盒查询，不是只比较两盒角点。Box–Box 碰撞有专门实现；距离路径与碰撞路径不应混为一谈。分别记录 solver 类型、标量类型和容差，先以 double＋LIBCCD 为候选基线，再决定是否需要 INDEP 对照。[LIBCCD solver](https://raw.githubusercontent.com/flexible-collision-library/fcl/0.7.0/include/fcl/narrowphase/detail/gjk_solver_libccd-inl.h)、[形状距离 traversal](https://raw.githubusercontent.com/flexible-collision-library/fcl/0.7.0/include/fcl/narrowphase/detail/traversal/distance/shape_distance_traversal_node-inl.h)。

Box–OcTree 遍历占据叶子，以叶子真实边界构造 Box 并调用 shapeDistance；非占据分支不会成为障碍距离。它既不是 ESDF，也不证明未知区可通行。相交查询可返回接触点、法向和深度，但结果受接触容量限制。[OcTree solver，特别是 OcTreeShapeDistanceRecurse / OcTreeShapeIntersectRecurse](https://raw.githubusercontent.com/flexible-collision-library/fcl/0.7.0/include/fcl/narrowphase/detail/traversal/octree/octree_solver-inl.h)。

**源码暴露的适配风险：** Octree 符号距离 fallback 额外调用碰撞查询，取返回 contacts 中最大深度的负值写入 `result.min_distance`，随后仍返回此前 `res`；不能假设函数标量返回值和 result 字段一致。开启最近点后，fallback 将两点都写成同一个 contact.pos，不能据此归一化点差。这里“最大”仅针对实际返回 contacts，不能解释为完整占据并集的全局最小脱离平移量。[distance-inl.h L129–178](https://raw.githubusercontent.com/flexible-collision-library/fcl/0.7.0/include/fcl/narrowphase/distance-inl.h)。

该 fallback 使用默认 CollisionRequest，只开启 enable_contact；默认 `num_max_contacts=1`。因此多体素穿透需专门实验，不能直接援引 API 文字中的 maximum penetration 当作完整全局保证。接触 normal 的文档方向为 o1 到 o2，几何适配须以返回对象身份和换序实验核对。[CollisionRequest](https://raw.githubusercontent.com/flexible-collision-library/fcl/0.7.0/include/fcl/narrowphase/collision_request.h)、[Contact](https://raw.githubusercontent.com/flexible-collision-library/fcl/0.7.0/include/fcl/narrowphase/contact.h)。

建议控制适配只在已验证的正间隙区使用距离及最近点导数；接触、穿透、无效点对、空树和查询失败进入明确状态，不能默认为一个可微负距离脱困模型。此建议不预先规定停车执行机制，沿用后续控制失败契约。

## PCL 与占据体素的边界

PCL VoxelGrid 输出格内质心，不能把质心当作该格中心，也不能把质心零半径点等同于实体占据盒。建议保留原始格索引和格尺寸，后续生成实体盒；最小点数过滤的少点剔除必须单独检查细杆召回。[PCL VoxelGrid 1.12.1](https://raw.githubusercontent.com/PointCloudLibrary/pcl/pcl-1.12.1/filters/include/pcl/filters/impl/voxel_grid.hpp)。

PCL 处理与 OctoMap 占据状态是两个层次：PCL 不自动提供传感器时效、未知掩码或有效空闲射线。叶节点可能压缩为更大盒，查询应使用实际叶尺寸，而非总假定最细分辨率。占据/空闲更新与动态失效策略沿用既有[环境点云研究](environment-pointcloud-collision.md)，不在此重新决策。

## 可审核的误差预算（数学推导，尚无数值承诺）

设当前真实实体为 R、O，建模实体为 R̂、Ô。若有覆盖证明 `R ⊆ R̂ ⊕ B(ε_R)` 与 `O ⊆ Ô ⊕ B(ε_O)`，且分离态数值误差满足 `|d_num − dist(R̂,Ô)| ≤ ε_num`，则：

`dist(R,O) ≥ max(0, d_num − ε_R − ε_O − ε_num)`。

若 R̂ 本来就是完整外包络，可令这一项 ε_R=0；但标定、装配和运动估计误差仍需单列。观测年龄 τ 内相对速度有可信上界 V_rel 时，增加 `V_rel τ`；如只给初始速度及加速度界，则用 `V₀ τ + ½ A_rel τ²`。这些是集合包含与三角不等式推论，不是库的安全认证。没有覆盖条件时，有限膨胀不能修复完全漏检障碍。

边长 s 的立方格：格中心代表完整格需要半径 `√3 s/2`；**格内任意质心**代表完整格最坏需要 `√3 s`，不能把中心的半对角线界直接套到 PCL 质心。若直接查询完整命中实体格，相对所含测量点已经保守，不应无条件重复扣除半对角线；量测偏差及未观测表面仍另算。采用非立方体素时以上分别换为半对角线和全对角线。

建议账本分别登记：包络欠覆盖 ε_R、传感量测/外参 ε_obs、离散表示 ε_repr、算法 ε_num、时间运动 ε_age。不要在几何膨胀和 h 阈值两处重复收费。`distance_tolerance` 是迭代终止参数，不能未经对照测试就当作 ε_num 的全局上界。

## 九轴导数接入

以下是分离、局部最近特征稳定时的运动学推导。取机器人点 p_R、环境点 p_O，`n=(p_R−p_O)/d`。静态环境下 `∂d/∂q = nᵀ J_R`；自碰撞两个运动点则为 `nᵀ(J_A−J_B)`。点 Jacobian 必须包括连杆旋转引起的偏置点速度以及全部相关九轴运动，不能只取 TCP 或盒中心平移 Jacobian。

动态环境显式项为 `−nᵀv_O`；若双方有额外时间项则同样计入。最近面/边/角、最近体素或最近连杆切换处，最小距离可能不可微；接近并列的候选不能凭单个 point pair 承诺全局梯度。上述表达仅是一阶局部结果，不能自动提供加速度型 CBF 所需的二阶项。FCL C++ 返回值不被 JAX 自动微分；传入常数 d 也不会产生这些 Jacobian，接入路线见既有[环境研究中的控制边界](environment-pointcloud-collision.md#距离怎样进入-oscbf哪些事情库不会代办)。

## 后续验收与本次证据边界

本次只浏览标签源码并提交研究文件；没有运行 FCL、PCL、性能或硬件测试。父任务的本地模型审计及隔离实验应另行链接，不将其结果预写在这里。

1. Box–Box：轴对齐解析间隙、旋转分离、面/边/角接触、浅穿透、完全包含、近共面；记录 scalar、min_distance、两点表面残差和点差长度。
2. Box–OcTree：单叶与等价 Box 对照、多叶并列/穿透、压缩叶、交换输入顺序、空/未知/仅空闲树；检查返回对象身份和树叶身份，不用输入顺序猜点对。
3. signed 开关、solver、容差/坐标尺度矩阵：碰撞状态对照、有限值、点对有效性；穿透不要用正间隙公式硬算梯度。
4. 导数：非切换点多步长有限差分与九轴 Jacobian 对照，另测特征切换与多约束选择，不以非光滑点误差简单宣判几何内核错误。
5. 全链路：真实 OBB 覆盖、附件、每对误差账本、传感未知/年龄，以及候选数量和并发下端到端尾延迟；本研究不承诺频率或毫米精度。
