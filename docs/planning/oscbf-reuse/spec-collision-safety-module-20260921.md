# CollisionSafety 统一碰撞安全模块规格说明书

**状态**：已确认（2026-09-21）。本规格汇集 ADR 0010 至 ADR 0046 的 accepted 决定，术语以
`CONTEXT.md` 为准。坐标语义沿用 ADR 0002，感知失效、标定真源与标定准入沿用 ADR 0003、
ADR 0004、ADR 0006 和 ADR 0009。

## Problem Statement

当前系统在规划与控制阶段使用不同的碰撞方法。规划依赖 MoveIt/FCL mesh 查询，OSCBF 使用
JAX OBB 自碰撞与球体环境约束。两条路径的机器人包络、安全阈值、距离含义、候选碰撞关系和
失败语义不同，因此同一个状态可能在规划阶段通过，在控制阶段得到不同结论。

现有 OBB 距离在相交后饱和到零，无法提供 DCOL 的连续缩放语义；球体环境表示需要较多 primitive，
也会占用机器人本体之外的空间。聚合 barrier 还可能在多个相反危险方向之间产生梯度抵消。
当前允许松弛的命令 QP 不能把碰撞约束作为不可放宽的授权条件。

LiDAR 单帧点集没有表达 `occupied`、`free` 和 `unknown`，也没有完整保存射线证据、遮挡、数据
年龄、逐点采集时间与运动不确定性。机器人自身点的过滤、固定环境模型、移动障碍物跟踪和
必要空间覆盖尚未形成一个带版本的共同场景。规划结果、前视证明与控制查询也缺少统一的
scene identity。

规划阶段还依赖 MoveIt/OMPL/FCL 运行路径，无法直接复用 OSCBF 所需的 JAX 梯度与 point-scale
barrier。用户需要自研规划覆盖过渡段和完整任务名义轨迹，并让规划、轨迹验证与 100 Hz OSCBF
共享相同的几何、场景、查询 kernel、毫米间距规则和失败状态。

系统还缺少以下完整依据：机器人组合 ellipsoid 对 collision mesh 的体积覆盖证明、完整
self collision 候选集合、连续运动区间证明、固定容量溢出规则、明确 solver health、共享场景
发布规则、目标计算设备 deadline、参数测量来源以及从现有实现切换到新模块的独立验收证据。

## Solution

建立无 ROS 的 JAX `CollisionSafety` Module，作为规划、轨迹执行与 OSCBF 的唯一碰撞判定来源。
Module 只公开 `prepare_scene`、`query` 和 `certify` 三个操作，内部统一管理机器人组合
ellipsoid、ellipsoid–ellipsoid DCOL、ellipsoid–point scale、独立毫米距离查询、活动约束集合、
单位转换、solver health 与连续区间证明。

机器人每个 link、滑台和当前末端工具使用固定容量组合 ellipsoid，并通过离线四面体体积覆盖
证明生成 `Collision geometry artifact`。自碰撞默认检查完整 link pair 集合，只有经过验证并
绑定 `geometry_hash` 的允许接触 pair 可以排除。每个有效 ellipsoid pair 保留独立 barrier 与
梯度。

环境信息由 LiDAR 射线更新的局部占据场景提供，并与经过验证的固定环境模型取并集。
`collision_scene_server` 是 `CollisionScene`、`scene_epoch`、`scene_revision` 和双缓冲共享内存
的唯一写入者。它负责 mesh ray self-filter、`occupied/free/unknown`、运动跟踪、support
生成、容量管理、覆盖检查和场景发布。

自研规划由 JAX multi-start IK、AEB-RRT* 全局搜索和 JAX trajectory optimization 组成，覆盖
当前位置到任务起点的过渡段以及完整任务名义轨迹。所有状态、规划边、优化轨迹、执行前视和
受限恢复轨迹都调用同一 `CollisionSafety` Module，并通过自适应区间证明。

OSCBF 使用独立且不可松弛的 self 与 environment primitive CBF 行。动态 support 的名义速度
进入 `partial_h_partial_t`，误差、加速度范围与延迟进入 `beta` 和 `beta_dot`。命令 QP 只接受
identity 一致、状态为 `OK`、solver health 有效且未超过 deadline 的固定结构结果。

所有安全数值由版本化 `CollisionParameterArtifact` 提供。新模块经过独立几何参考、真实 LiDAR
回放、MuJoCo、shadow、故障注入和目标计算设备运行时间验收后，成为唯一运行路径；随后移除
MoveIt/FCL 规划碰撞、OBB barrier、球体环境约束和运行期间 fallback。

## User Stories

1. 作为任务操作者，我希望规划与控制使用同一个碰撞判定来源，从而避免同一状态得到两种安全结论。
2. 作为任务操作者，我希望环境变化后只执行当前场景已经证明安全的轨迹区间，从而避免沿用旧场景证明。
3. 作为任务操作者，我希望场景 revision 正在准备或验证时机器人保持命令，从而等待完整的新场景结果。
4. 作为任务操作者，我希望真正破坏安全依据的故障被锁存，从而避免条件波动引起意外恢复运动。
5. 作为任务操作者，我希望故障恢复同时要求条件恢复、重新验证和人工确认，从而使恢复过程可以审计。
6. 作为任务操作者，我希望安全间距越界但仍有几何间距时可以准备受限恢复轨迹，从而在受控条件下回到安全集合。
7. 作为任务操作者，我希望物理接触、unknown 或 solver 异常时禁止机器人恢复命令，从而避免缺少安全依据的运动。
8. 作为规划使用者，我希望规划阶段不依赖 MoveIt/OMPL/FCL 运行路径，从而直接使用统一 JAX 碰撞语义。
9. 作为规划使用者，我希望 AEB-RRT* 搜索九轴关节空间的全局连通路径，从而处理需要大范围构型变化的任务。
10. 作为规划使用者，我希望 JAX trajectory optimization 使用碰撞梯度改善初始路径，从而兼顾路径、时间、平滑程度和安全余量。
11. 作为规划使用者，我希望规划覆盖过渡段和完整任务名义轨迹，从而提前安排冗余关节构型。
12. 作为规划使用者，我希望任务名义轨迹保持末端位置与工具轴要求，从而维持蝴蝶轨迹的任务语义。
13. 作为规划使用者，我希望任务名义轨迹只提供任务参考与零空间参考，从而让 OSCBF 保留最终命令授权能力。
14. 作为规划使用者，我希望 JAX multi-start IK 生成多个有效冗余目标，从而避免单一 IK 构型限制全局搜索。
15. 作为规划使用者，我希望所有 IK 候选满足关节、任务和当前场景碰撞条件，从而拒绝无效任务起点。
16. 作为规划使用者，我希望每条规划边都经过连续区间证明，从而避免端点采样遗漏区间内碰撞。
17. 作为规划使用者，我希望优化后的轨迹重新接受完整证明，从而防止优化过程改变原有安全依据。
18. 作为规划使用者，我希望规划使用冻结场景并在完成后接受最新场景复查，从而兼顾稳定规划输入与当前环境安全。
19. 作为规划使用者，我希望无关 scene revision 只更新准入 revision，从而避免持续 LiDAR 更新反复触发全局规划。
20. 作为规划使用者，我希望剩余轨迹出现未通过区间、覆盖不足、动态冲突或余量不足时重新规划，从而在风险变化时更新路径。
21. 作为控制维护者，我希望每个有效 self ellipsoid pair 都保留独立 CBF 行，从而避免相反梯度互相抵消。
22. 作为控制维护者，我希望每个活动 environment ellipsoid–support pair 都保留独立 CBF 行，从而保持危险方向信息。
23. 作为控制维护者，我希望所有碰撞 CBF 行禁止松弛，从而让任务目标在安全约束冲突时让步。
24. 作为控制维护者，我希望诊断松弛计算与命令 QP 隔离，从而防止诊断结果获得命令权限。
25. 作为控制维护者，我希望 self 与 environment 使用各自固定的 `cbf_rate`，从而分别验证两类响应行为。
26. 作为控制维护者，我希望 DCOL 几何缩放量与 CBF 响应率使用不同名称，从而避免 `alpha` 含义混用。
27. 作为控制维护者，我希望动态障碍物速度进入 barrier 时间导数，从而让接近速度直接影响当前约束。
28. 作为控制维护者，我希望跟踪误差、加速度范围和延迟进入 `beta` 与 `beta_dot`，从而覆盖运动估计不确定性。
29. 作为控制维护者，我希望 100 Hz 线程只读取完整 `PreparedScene` 并执行查询与 QP，从而避免感知处理占用控制期限。
30. 作为控制维护者，我希望 collision query 与 QP 超过 deadline 时进入故障锁存，从而拒绝未经本周期检查的命令。
31. 作为感知维护者，我希望 LiDAR 射线明确更新 `occupied`、`free` 和 `unknown`，从而表达遮挡、无返回与未观测空间。
32. 作为感知维护者，我希望一次有效 hit 立即建立 occupied，从而及时保留新障碍物。
33. 作为感知维护者，我希望 occupied 只能由连续有效 free 射线清除，从而避免单次噪声删除障碍物。
34. 作为感知维护者，我希望陈旧 occupied 转为 unknown，从而避免仅凭时间经过声明自由空间。
35. 作为感知维护者，我希望固定环境模型与 LiDAR occupied 取并集，从而同时约束工作台、夹具、地面和新障碍物。
36. 作为感知维护者，我希望 LiDAR free 不能清除固定环境模型，从而保持经过验证的固定几何。
37. 作为感知维护者，我希望 self-filter 使用逐点采集时间对应的机器人 mesh 首次交点，从而正确处理扫描期间的机器人运动。
38. 作为感知维护者，我希望机器人表面之前的返回保留为 environment occupied，从而识别贴近机器人的外部物体。
39. 作为感知维护者，我希望机器人遮挡后的空间保持 unknown，从而避免把不可见区域声明为 free。
40. 作为感知维护者，我希望歧义观测阻止覆盖准入，从而显式处理测量、标定和时间误差。
41. 作为感知维护者，我希望组合 ellipsoid 不参与删除 LiDAR point，从而避免删除包络内的外部障碍物。
42. 作为感知维护者，我希望 occupancy 决定障碍物是否存在，从而避免 tracking 关联中断删除障碍物。
43. 作为感知维护者，我希望可信 tracking 只提供速度和误差范围，从而增强动态约束并保持占据证据权威。
44. 作为感知维护者，我希望未跟踪 occupied 使用已验证的速度与加速度上界，从而覆盖未知运动。
45. 作为感知维护者，我希望未来规划区间使用 time-indexed reachable support tube，从而验证障碍物在整个区间的可达范围。
46. 作为场景服务维护者，我希望 `collision_scene_server` 独占场景状态与 revision，从而防止多个进程生成不同环境事实。
47. 作为场景服务维护者，我希望每次进程启动生成新的 `scene_epoch`，从而识别重启后的 revision 重用。
48. 作为场景服务维护者，我希望使用固定布局双缓冲共享内存发布场景，从而减少大数组重复序列化。
49. 作为场景服务维护者，我希望发布前检查 identity、checksum、时间、覆盖和容量，从而只公开完整场景。
50. 作为场景读取者，我希望每个 revision 只向本地 JAX device buffer 复制一次，从而让控制查询期间不访问共享内存。
51. 作为场景读取者，我希望 revision 变化、checksum 不一致或布局身份不符时拒绝副本，从而避免读取部分更新或错误布局。
52. 作为机器人几何维护者，我希望每个 link 使用固定容量组合 ellipsoid，从而兼顾包络质量与 JAX shape 稳定性。
53. 作为机器人几何维护者，我希望机器人本体、滑台和当前工具共用同一表示，从而形成完整机器人碰撞包络。
54. 作为机器人几何维护者，我希望 collision mesh 经过四面体体积覆盖证明，从而证明 ellipsoid 并集覆盖完整体积。
55. 作为机器人几何维护者，我希望 mesh 或工具变化产生新的 `geometry_hash`，从而使旧规划与证明自动失效。
56. 作为机器人几何维护者，我希望 self collision 默认检查完整 link pair 集合，从而避免固定少量 pair 遗漏碰撞关系。
57. 作为机器人几何维护者，我希望允许接触 pair 具有原因、证据和适用几何身份，从而让每项排除可以审查。
58. 作为碰撞 kernel 维护者，我希望 self query 使用原生 JAX ellipsoid DCOL，从而获得分离、接触和相交状态下的一致缩放语义。
59. 作为碰撞 kernel 维护者，我希望 DCOL 返回 residual、iteration 和 solver health，从而拒绝未收敛结果。
60. 作为碰撞 kernel 维护者，我希望环境约束使用 ellipsoid–point scale，从而直接利用解析 barrier 与梯度。
61. 作为碰撞 kernel 维护者，我希望公开配置和诊断中的间距统一使用毫米，从而明确单位并保留现场可读性。
62. 作为碰撞 kernel 维护者，我希望单位转换集中在 Module Interface 边界，从而防止调用方重复转换。
63. 作为碰撞 kernel 维护者，我希望所有碰撞计算统一使用 `float64`，从而保持毫米边界、梯度和区间下界一致。
64. 作为诊断使用者，我希望独立查询机器人到外部障碍物的有符号毫米距离，从而查看最近 pair、最近点和剩余余量。
65. 作为诊断使用者，我希望毫米距离只用于诊断和规划代价，从而保持 point-scale barrier 的 OSCBF 授权职责。
66. 作为诊断使用者，我希望每项结果携带 scene、geometry、kernel 和 policy identity，从而知道该结论依赖哪些输入。
67. 作为诊断使用者，我希望等待、场景无效、unknown、容量、solver、证明和 deadline 状态彼此区分，从而定位主原因。
68. 作为接口维护者，我希望所有 query 与 certify 结果具有固定结构和 `valid_mask`，从而保持 JAX shape 与调用方式稳定。
69. 作为接口维护者，我希望只有 `status == OK` 的有效结果进入命令或路径准入，从而防止特殊数值被解释为安全。
70. 作为参数维护者，我希望安全参数来自版本化 `CollisionParameterArtifact`，从而追踪数据来源、单位、适用设备和验证结果。
71. 作为参数维护者，我希望运行期间禁止覆盖 artifact 中的安全字段，从而保持规划、执行和 OSCBF 使用同一策略身份。
72. 作为验收人员，我希望切换证据 manifest 绑定代码、配置、几何、kernel、数据和设备身份，从而重现每次验收结论。
73. 作为验收人员，我希望几何解析用例与独立高精度参考作为正确性依据，从而避免把现有 OBB 或球体结果当作参考答案。
74. 作为验收人员，我希望使用真实 LiDAR 回放覆盖遮挡、运动、tracking 中断和数据过期，从而验证场景语义。
75. 作为验收人员，我希望使用 MuJoCo 验证规划、OSCBF、恢复和故障流程，从而观察完整闭环行为。
76. 作为验收人员，我希望 shadow 比较期间新模块没有命令权限，从而在切换前收集差异证据。
77. 作为验收人员，我希望目标计算设备记录每个阶段的最大运行时间与 `p99.9`，从而验证 100 Hz deadline。
78. 作为验收人员，我希望参数开发数据与最终验收数据分离，从而防止验收期间继续调整阈值。
79. 作为维护仓库的人，我希望新模块通过证据门后成为唯一运行来源，从而结束两套碰撞语义并存。
80. 作为维护仓库的人，我希望切换后移除 MoveIt/FCL 规划碰撞、OBB barrier、球体环境约束和 fallback，从而保持单一命令授权路径。

## Implementation Decisions

1. `CollisionSafety` 是规划、轨迹执行与 OSCBF 共同加载的无 ROS JAX Module。ROS 组件只负责
   传感器接入、状态传输、共享场景通知和诊断，不能复制碰撞公式。
2. Module 只公开三个操作：`prepare_scene(scene, identities)`、
   `query(query_batch, prepared_scene, query_mode)` 和
   `certify(segment_batch, prepared_scene)`。state validity、OSCBF barrier 与毫米距离通过
   固定 `query_mode` 表达。
3. `prepare_scene` 验证场景、时间、覆盖、容量、shape、`scene_epoch`、`scene_revision`、
   `geometry_hash`、`kernel_version` 和 `collision_policy_hash`，成功后返回不可变
   `PreparedScene`。
4. 机器人每个 link 使用固定容量组合 ellipsoid 与 `active_mask`。机器人本体、J1 滑台和
   当前末端工具均纳入同一 `geometry_hash`。
5. `Collision geometry artifact` 由经过检查的 closed collision mesh 离线生成。mesh volume
   经过四面体化，每个四面体全部顶点必须被至少一个 outer ellipsoid 包含。artifact 保存
   mesh hash、槽位、覆盖证明、最大包络范围、运行时间证据和 `geometry_hash`。
6. 自碰撞候选从完整 link 集合生成。允许接触列表是唯一排除来源，每项排除绑定 link identity、
   原因、验证依据和 `geometry_hash`。几何变化后重新生成候选与允许接触列表。
7. self collision 使用原生 JAX ellipsoid–ellipsoid DCOL，返回线性 `proximity_scale`、
   关节梯度、primal/dual residual、iteration 与 solver status。42 面 polytope ellipsoid 近似
   不进入生产查询。
8. self pair 的公开间距使用 `clearance_mm_ij`。Interface 转换为米，并根据两个 ellipsoid
   最短半轴计算 pair-specific `scale_margin_ij`。self barrier 使用
   `h_ij = proximity_scale_ij - scale_margin_ij`。
9. 环境 occupied voxel 转换为 `support_point + rho_mm`。`rho_mm` 覆盖 voxel 半对角线、
   LiDAR 误差、外参误差、时间误差和障碍物运动范围；`required_clearance_mm` 单独保存。
10. environment pair 先计算
    `barrier_radius_m = rho_m + required_clearance_m`，再根据机器人 ellipsoid 最短半轴转换为
    `scale_margin_ij = 1 + barrier_radius_m_j / a_min_i`，barrier 使用
    `h_ij = point_scale_sq_ij - scale_margin_ij^2`。
11. Module 保留独立环境毫米距离查询。它返回 `distance_mm`、`required_clearance_mm`、
    `remaining_margin_mm`、最近 link、ellipsoid slot、support identity、双方最近点、track
    状态与 scene identity。该查询没有 OSCBF 授权能力。
12. 每个有效 self ellipsoid pair 保留独立 barrier、梯度和 CBF 行。link pair 最小值只用于
    诊断。Q4 原型已证明聚合梯度会在对称危险方向发生抵消。
13. JAX 批量计算全部有效 environment barrier。每个 robot ellipsoid 选择固定容量 `K` 个
    最危险 support pair，每个 pair 保留独立行。只有具有下一次检查前安全下界证明的 pair
    可以省略；需求超过 `K` 时返回 `CONSTRAINT_OVERFLOW`。
14. `CollisionScene` 使用固定 `MAX_SUPPORT_POINTS`。超出容量时只能按
    `rho_merged = max(norm(p_j - c) + rho_j)` 进行可证明覆盖的合并；覆盖无法证明时返回
    `CAPACITY_OVERFLOW`。
15. 环境活动集合使用不同进入与退出阈值并保留 witness identity。机器人 ellipsoid、support、
    track 与 QP row 均采用固定容量和 mask，运行期间不改变 JAX shape。
16. LiDAR 场景通过射线更新 `occupied/free/unknown`。有效 hit 立即产生 occupied；连续、独立且
    时间有效的 free 证据才能清除 occupied；陈旧 occupied 转为 unknown。
17. 经验证固定环境模型转换为版本化 `support_point + rho_mm` 并与 LiDAR occupied 取并集。
    LiDAR free 不能清除固定模型，冲突时采用 occupied；模型 revision 改变会撤销旧证明。
18. LiDAR self-filter 使用 collision mesh、共享运动学、逐点采集时间和关节状态历史执行射线
    首次交点查询。表面前的返回保留为 environment occupied，表面后的区域保持 unknown，
    ambiguous observation 阻止覆盖准入。
19. occupancy 决定障碍物是否存在。可选 tracking 只为已存在 support 提供名义速度、加速度
    范围、关联时间和估计误差。track 失效后 occupied 保留，并使用未跟踪运动上界。
20. 当前控制 barrier 写为 `h(q,t) = s_squared(q,p(t)) - beta(t)^2`。`partial_h_partial_t`
    同时包含 support 名义速度项与 `beta_dot`。规划与 certify 使用 time-indexed reachable
    support tube。
21. 规划器由 JAX multi-start IK、AEB-RRT* 与 JAX trajectory optimization 组成，不使用
    MoveIt/OMPL/FCL 规划运行路径。规划输出是带时间且经过证明的九轴轨迹。
22. AEB-RRT* 负责全局连通搜索；trajectory optimization 同时优化过渡段与完整任务名义轨迹。
    IK、全局搜索、优化和区间证明共用相同关节顺序、限制、scene 与 geometry identity。
23. `certify` 对关节运动区间计算机器人 ellipsoid 最大运动范围和所有有效 pair 的
    `proximity_scale` 下界。无法直接证明时递归二分；深度或时间预算耗尽返回未通过。
24. 一次规划使用冻结 `PreparedScene`。规划完成后在最新 scene 上复查完整路径并记录
    `admitted_revision`。新 revision 会使旧未来区间证明失效。
25. scene 更新后立即验证执行前视区间，完成前输出保持命令。前视通过后可以继续执行，并在
    独立 worker 验证剩余轨迹。风险条件满足时重新运行 AEB-RRT* 与 trajectory optimization。
26. 命令 QP 中全部 self 与 environment collision 行不可松弛。约束冲突、QP 失败、无效梯度、
    solver health 异常或非有限结果均不能生成准入命令。
27. self collision 与 environment collision 分别使用固定的
    `self_collision_cbf_rate_s_inv` 和 `environment_cbf_rate_s_inv`。运行期间不按 pair、barrier
    或接近速度改变响应率。
28. `proximity_scale` 只表示 DCOL 或 point-scale 几何结果；`cbf_rate` 只表示一阶 CBF 时间
    响应参数。新配置和诊断不使用 `alpha` 指代 CBF 响应率。
29. `collision_scene_server` 是 CollisionScene、scene identity 和共享内存的唯一写入者。
    它拥有 mesh self-filter、射线占据、固定模型合并、tracking、support、覆盖、容量与场景健康。
30. 场景跨进程发布使用固定布局双缓冲共享内存。场景服务只写未启用 slot，全部验证成功后
    原子切换 active slot；启用 slot 在替换前保持不可变。
31. ROS 只发布 revision、采集时间、准备时间、active slot、布局身份和健康状态。每个读取进程
    检查 header、identity、checksum 与时间，并在每个 revision 复制一次到本地 JAX device。
32. 感知准备、100 Hz query/QP、完整剩余轨迹验证和重新规划使用独立 worker。控制线程不能
    等待锁、文件、ROS 调用、scene preparation 或动态容量扩展。
33. LiDAR 原始坐标可以使用 `float32`。`prepare_scene` 一次转换为 `float64`；geometry、DCOL、
    point-scale、barrier、gradient、区间证明、距离查询和 QP 输入统一使用 `float64`。
34. `query` 与 `certify` 返回固定 shape typed result。公共 header 包含 status、scene identity、
    geometry identity、kernel identity、policy identity、运行时间和 deadline 状态。
35. OSCBF payload 包含 `valid_mask`、barrier、`grad_h_q`、`partial_h_partial_t`、
    `proximity_scale`、primitive pair identity、solver residual、iteration 与 health。
36. 运行状态至少区分 `OK`、`REVISION_PENDING`、`INVALID_SCENE`、
    `UNKNOWN_REQUIRED_SPACE`、`CAPACITY_OVERFLOW`、`CONSTRAINT_OVERFLOW`、
    `SOLVER_UNHEALTHY`、`CERTIFICATE_FAILED` 和 `DEADLINE_MISSED`。
37. 错误 shape、非法配置、未知 query mode 和程序调用错误在进入 JAX kernel 前立即抛出异常。
    运行期非 `OK` 结果只允许进入诊断。
38. 运行状态区分碰撞临时等待与碰撞故障锁存。revision 准备、前视验证和重新规划可以在当前
    准入完成后自动继续；标定、时间、覆盖、容量、solver、QP 与 self-filter 时间错误进入锁存。
39. 受限恢复只适用于 barrier 已越界且所有相关几何仍然分离的状态。恢复计划使用低速限制，
    每个区间让全部已违反 barrier 严格增加，并维持其余碰撞约束。
40. 恢复确认绑定 scene、geometry、kernel、policy 和轨迹 hash。identity 或 revision 变化立即
    撤销确认。回到恢复阈值后，原任务仍需重新规划和准入。
41. `CollisionParameterArtifact` 保存误差范围、延迟、间距、两类 cbf rate、固定容量、solver
    限制、区间二分限制和各阶段 deadline。每个参数记录单位、数据来源、方法、适用设备、适用
    场景、置信要求和验证结果。
42. `parameter_hash` 与允许接触列表、geometry identity、kernel identity 共同生成
    `collision_policy_hash`。任何字段或生成方法变化都会撤销旧规划准入、恢复确认和验收证据。
43. 当前生产路径继续运行到新模块完成全部证据门。并行验证期间新模块只输出诊断，没有命令
    权限。现有输出只用于差异定位。
44. 证据门全部通过后，新模块一次成为规划、轨迹执行与 OSCBF 的唯一碰撞来源。旧规划碰撞、
    OBB barrier、球体环境约束、旧配置入口和 fallback 随切换移除。

## Testing Decisions

- 好测试只验证外部行为：给定场景、identity、机器人状态、轨迹区间与时间，断言公开结果、
  status、witness、准入结论和状态变化。测试不依赖内部函数组织、临时变量或 solver 内部步骤。
- 主要测试 seam 只有 `CollisionSafety` 的三个公开操作。kernel 的解析参考与数值参考通过同一
  seam 输入，避免建立只供测试调用的公共入口。
- `prepare_scene` 测试覆盖有效 scene、frame/单位/shape、非有限值、scene epoch、revision、
  checksum、geometry/kernel/policy identity、时间、required-space coverage、capacity、固定
  环境 revision、tracking 有效期和 `float64` 转换。
- `query` 测试覆盖 self DCOL、environment point-scale、独立毫米距离、单状态、批量状态、
  固定 mask、接触附近、相交、极端 ellipsoid 长宽比、动态 `partial_h_partial_t`、`beta_dot`、
  最近 witness、solver residual 与全部 status。
- self 和 environment primitive 行必须逐行检查。Q4 原型的对称危险方向作为常驻回归：独立行
  必须限制危险速度，任何聚合展示值不能参与命令授权。
- `certify` 测试覆盖整段可直接证明、递归二分后证明、碰撞区间、计算预算耗尽、动态 reachable
  support tube、scene revision 变化与 geometry identity 变化。
- geometry artifact 测试使用经过检查的 closed mesh，验证全部四面体覆盖、mesh hash、slot、
  `active_mask`、允许接触列表和 `geometry_hash`。非 closed、拓扑无效或单位缺失必须拒绝。
- DCOL 数值测试使用解析球体与轴对齐 ellipsoid、独立高精度参考和官方 DCOL 离线参考，覆盖
  分离、接触、相交、尺寸差异、接近退化、梯度、residual 和 iteration 上限。
- 感知测试使用保留逐点采集时间、LiDAR 原点、标定身份与关节历史的录制数据，覆盖正常表面、
  表面前障碍、表面后遮挡、ambiguous observation、hit、连续 free、陈旧 occupied、unknown、
  固定模型冲突、tracking 中断与 support 合并。
- `collision_scene_server` 集成测试启动真实进程并使用真实共享内存布局，验证唯一写入、双缓冲
  原子发布、进程重启后的 scene epoch、revision 跳跃、checksum 损坏、slot 超时和 reader
  每 revision 单次 device 复制。测试不使用 mock scene server。
- 控制集成测试在 100 Hz seam 输入 typed result，验证只有 `OK` 与有效 mask 能进入 QP；碰撞行
  无松弛；task objective 可以让步；QP 不可行、solver 异常、非有限值和 deadline miss 进入锁存。
- 规划集成测试从 multi-start IK 目标集合经过 AEB-RRT*、trajectory optimization、最新场景
  复查和执行前视，断言所有被接受轨迹都带完整 identity 和 segment certificate。
- 重新规划测试覆盖无关 revision、前视失败、剩余轨迹失败、动态冲突、余量阈值、后台验证和
  重新规划期间的当前区间执行规则。
- 恢复测试覆盖允许准备、等待人工确认、确认身份绑定、revision 撤销、barrier 单调增加、回到
  恢复阈值、无法生成恢复轨迹、物理接触和 unknown 禁止恢复。
- 故障注入使用真实输入与真实进程边界，覆盖 calibration identity、数据年龄、coverage、shape、
  checksum、revision、shared memory、capacity、constraint、solver、QP、certificate 和 deadline。
- MuJoCo 端到端测试覆盖过渡段、完整蝴蝶轨迹、冗余避障、新障碍物、动态障碍物、保持、锁存、
  人工确认与重新规划。测试断言命令与状态，不读取内部 worker 状态。
- shadow 测试让新旧实现消费同一份录制输入。新结果没有命令权限；差异只用于定位。解析几何、
  高精度参考、覆盖证明与区间证明决定正确性。
- 目标计算设备性能测试分别记录 scene preparation、host-to-device transfer、collision query、
  QP 和完整 100 Hz 路径的最大值与 `p99.9`。任何 required deadline 未满足都会阻止切换。
- 参数开发数据与独立验收数据分离；验收规范在运行前固定容差、deadline、数据 identity、通过
  条件和随机种子，验收期间禁止修改阈值。
- 证据 manifest 保存代码 revision、CollisionParameterArtifact、geometry、kernel、policy、
  数据和目标计算设备 identity，以及每项命令、结果与证据位置。
- 现有先例包括 portable JAX kernel 测试、动态障碍物时间项测试、点云安全快照测试、感知管线
  回放测试、控制节点冒烟测试和完整闭环测试。新测试沿用这些最高层入口，并用新 Module seam
  替换旧 OBB、球体和 FCL 查询入口。

## Out of Scope

- 力矩级 OSCBF、动力学参数辨识、高阶 CBF 与力矩命令语义。
- 深度相机启用、相机与 LiDAR 联合标定以及多传感器融合扩展。
- 修改 `base_link` 规范坐标系、标定真源、标定记录 schema 或逐源标定准入规则。
- 开发新的 LiDAR 驱动；传感器消息规范化继续使用现有驱动接入边界。
- 通过外部毫米距离查询授权 OSCBF 命令；该查询只服务诊断、规划代价与显示。
- 为物理接触、unknown、无效 solver 或无效 scene 生成机器人恢复命令。
- 运行期间调整 clearance、cbf rate、capacity、误差范围或 deadline。
- 长期维护 MoveIt/FCL、OBB 或球体运行 fallback。
- 解除 live 模式限制、发送真实机器人命令或安排现场实验时间。
- 本规格发布过程中的生产代码修改。

## Further Notes

- ADR 0010 至 ADR 0046 是本规格的决定来源；领域名称采用 `CONTEXT.md` 的定义。
- 当前生产代码仍使用 MoveIt/FCL、OBB 与球体路径。本规格描述目标系统，不能解释为当前功能
  已经实现或通过验收。
- 本规格替代既有 OSCBF 架构规格中与规划碰撞、机器人碰撞包络、环境约束、self-filter、
  碰撞松弛、场景发布和旧实现退出方式冲突的目标条款。既有实现地图中的冲突票需要依据本规格
  重新组织后才能继续。
- Q4 throwaway prototype 提供独立 primitive 行的决策证据；该 HTML 制品不进入生产运行路径。
- safety clearance、cbf rate、capacity、solver limit 与 deadline 的具体数值没有在规格中编造，
  它们由 CollisionParameterArtifact 测量计划和独立验收生成。
- 环境距离查询只使用 occupied 与误差范围 `rho_mm` 计算有符号距离，并单独返回
  `required_clearance_mm`；point-scale barrier 使用两者之和，避免遗漏或重复计入要求间距。
- 生产切换前仍需实施方案、依赖关系、验收规范、parameter artifact 实验计划和新的执行地图。
- 项目 tracker 已停用 `ready-for-agent` 与 `ready-for-human` 标签。本规格按项目的 14 标签规则
  发布，不新增 tracker 标签。
