# 环境点云如何进入九轴碰撞检测与 OSCBF

- 日期：2026-09-07。
- 状态：研究建议，供「确定上游控制接入方式与九轴扩展边界」继续讨论；未接受的新方案不能当作实施决议。
- 条件：Ubuntu 22.04 / ROS 2 Humble；CPU/PCL 为已接受的首版点云查询基线；Mid-360S 与 Gemini 335L 持续互补，每源按采样时刻过滤后融合；自碰撞继续现有 OBB，替换以现有与上游实测择优为准。
- 范围：核验官方文档与源码，不安装依赖、不改运行代码、不运行硬件或性能测试。PCL 核验标签 1.12.1，FCL 核验标签 0.7.0，MoveIt 核验 humble 分支；具体生产依赖还须锁定和构建验证。

## 推荐结论

后续状态：用户已接受优先验证 CPU/PCL＋局部体素＋FCL 及动态回放到低速实机的路线，完整边界见[控制接入阶段决议](../issues/05-control-adoption.md#阶段决议动态环境碰撞候选与验证路线已接受)。下文保留研究时的候选措辞；环境侧优先复用 OBB 的验证路线也已获接受，见同票 Resolution comment；实际生产替换仍待验证。

**首版优先评估“PCL 整理点云＋局部占据体素＋FCL 计算机器人包络到体素的距离”，与现有实现对照后择优。** PCL 仍是点云处理/空间索引基础；增加 FCL 是复用几何距离能力，不是推翻 CPU/PCL 路线，也不是要求重新写一个距离求解器。MoveIt/OctoMap 用于规划场景复用；控制侧需要明确的时效、覆盖和距离适配，不能直接把默认规划地图当安全观测。

这里有两种完全不同的“盒子”：机器人 OBB 随连杆移动，描述机器人外形；环境体素是空间小格子，记录哪里观测到障碍。环境用小格子不改变已接受的 OBB 自碰撞。**环境侧机器人外包络尚未决定**：建议先比较复用现有 OBB 与现有环境包络；若 OBB 的外包围关系、全臂/TCP/附件覆盖及误差验证通过，可以统一复用 OBB。不能仅凭自碰撞选择就宣称环境几何已定。

上述为工程推荐，不是库提供的安全保证。FCL、PCL、OctoMap 都不会自动补出传感器没有看到的真实物体。

## 用一个靠近机械臂的纸箱说明链路

```text
LiDAR 的点 ─→ 本源时间/TF/质量检查 ─→ 可信本体点剔除 ─┐
                                                       ├→ 统一坐标中的观测
相机的点 ──→ 本源时间/TF/质量检查 ─→ 可信本体点剔除 ─┘     保留来源和年龄
                                                            │
固定工作台/夹具模型 ─────────────────────────────────────────┤
                                                            ↓
                   障碍占据几何 ＋ 已观测空闲/未知覆盖状态
                                                            ↓
                    全部相关连杆包络 ↔ 附近障碍体素
                                                            ↓
              距离、最近点对、涉及连杆、时间、有效性与误差
                                                            ↓
                   控制适配层 → OSCBF 约束 → 九轴指令
```

纸箱不必先被识别成“纸箱”，也不必连续跟踪成功才成为障碍。点落进哪些格子，就有相应占据证据；盒后没有看到的区域仍是未知。全臂每段连杆均要检查，不能只看末端或连杆中心。固定模型并行存在，不能用空帧清掉固定工作台。

## 开源库真实提供什么

| 候选 | 官方能力 | 本项目边界和建议 |
|---|---|---|
| PCL KD-tree＋点到包络计算 | `nearestKSearch` 找查询点的近邻，`radiusSearch` 找半径内点，输出点索引与平方距离。[1] | 不是 OBB 到点云的距离接口。仅找盒中心最近点会漏掉靠近长盒端部的危险点。可复用作候选筛选，但精确几何和候选完整性仍需适配。作为简单对照路线，不强迫新写几何库。 |
| FCL Box＋OcTree | 支持盒子、网格、OctoMap octree；提供碰撞/距离查询。[2] `OcTreeShapeDistance` 遍历已占据叶子，构造 Box 后调用形状距离求解。[3] | 机器人 OBB 可表示为 Box＋刚体变换；环境叶格是实体盒。可直接复用通用距离内核，优先评估。只检查占据结点，不自动阻止进入未知空间；需显式检查覆盖和空树状态。 |
| MoveIt Humble＋OctoMap | 点云 updater 用本源 TF 与射线更新 occupied/free；距离请求有最近点、阈值及多种结果模式。[4][5] | 优先复用规划场景与检查接口。控制循环是否直接调用、是否单独持有快照，由时效和尾延迟实测决定，不预先承诺控制频率。 |
| ESDF，如 voxblox/nvblox | voxblox `getDistanceAndGradientAtPosition` 返回某位置距离/梯度，并通过成功标志与 `isObserved` 表达是否观测；nvblox ESDF voxel 也有 observed 字段。[6][7] | 后续候选，不能把查几个位置等同整个 OBB 净空；仍需包络覆盖、插值误差、地图时效与动态清除。GPU 使用条件沿用既有 CPU/GPU 研究，不因提供梯度直接升级为首版。 |

FCL 的穿透距离和最近点有明确实现限制：`DistanceRequest` 说明未启用 signed distance 时相交结果可由实现定义；mesh/octree 最近点标为 `NP_X`，不保证所有情况下位于几何表面。[8] 因此先验证所用 Box–OcTree/Box–Box 组合在正间隙、接触和穿透下的距离、点对及法向；不把普通距离查询当作已验证的紧急脱困方向。

## 点少了，为什么反而可能“看起来更安全”

以下是集合距离的数学推论，不是 PCL 安全承诺。设真实障碍表面为 O，测到的点是其子集 P，则 `dist(R,P) ≥ dist(R,O)`：机器人到测量点的距离可能比到真实物体远。一个细杆只出现少量点，或者杆被遮住，最近点算法照样能正常返回结果，却不能证明真实净空。

体素化能把每个命中点变为有体积的小格子，补偿点的零体积表示；但没有命中点的细杆不会凭空出现。应分别建立量测/标定误差界、离散化误差界、传感器覆盖条件和运动/时延边界，不能用一个随意膨胀半径掩盖未知。

PCL `VoxelGrid` 计算每格点的质心，不保留该格完整外边界。[9] 所以推荐保留命中格的边界或实际格尺寸，必要时叠加有依据的量测包络；不要将质心当作零半径障碍。降噪也必须评估细杆、边缘和少点新障碍：点少不等于噪声。ROI 应包括各连杆运动和停止所需空间，不能只截取工具前方。

若备选采用机器人表面采样，不能只取 OBB 八个角点。若采样集合确实对整个相关表面形成半径 ε 的覆盖，在分离状态且距离函数为真实欧氏距离时，`最小采样距离 − ε` 可作为表面最小距离的下界；只测角点却没有 ε 证明不成立。该推导也不能检测“障碍已被完全包进盒内而表面采样仍有正距”等体积相交情形，必须另外做相交/包含检测。直接实体 OBB–体素距离可避免这一层机器人表面采样近似。

若保留 PCL 点候选路线，点到 OBB 的非负实体距离有简单解析式：令点在盒局部坐标为 x，盒半长为 e，则 `d = ||max(|x|−e, 0)||₂`；盒内为 0，不是穿透深度。此式为几何推导，仅对应观测点。以盒中心为圆心、半径“盒半对角线＋关注距离”的半径查询，可覆盖关注距离内的全部点；这只是宽阶段筛选，不改变机器人为 OBB。不能用固定最近 K 点或静默截断结果替代完整性证明。

## 动态障碍不能无限堆旧点，也不能过期就变空闲

Humble updater 对命中格增加占据证据，对该源有效射线经过格增加空闲证据，并在同次更新里让 occupied 优先于 free。[4] OctoMap 是累计更新占据证据的机制，标准 `OcTree` 插入过程不等同按业务有效期管理的动态障碍快照。[10] 人走开后，如果旧位置没有新的有效观测，不能假定旧占据马上消失；相反，简单滚动删点会把被遮挡仍存在的物体误删。

推荐项目策略（需实现/回放验证）：

- 控制侧保存有界年龄的局部观测，按源记录最后有效占据/空闲证据；固定模型另存。
- 旧占据仅在有效新证据及已验证策略下清除。超时或被遮挡可转为未知；删除数据不等于证明可通行。
- 双源分别用自己的射线原点更新；一源“没看到”不能删除另一源有效占据。不同时间的冲突还要考虑障碍运动，不能只靠更新顺序决定。
- 新障碍无需等待分类/跟踪，先进入几何约束；速度未知不能按静止处理，需已验证的运动上界或停止策略。
- 空点、空树、无有效距离、索引失败、覆盖不足均显式返回状态；不能默认为 10 m 或无穷远安全距离。

这些来源/时间/覆盖字段不是普通 XYZ 点云、默认 OctoMap 节点或 FCL 返回值自动具备的，需要项目的薄适配层维护。

## 距离怎样进入 OSCBF，哪些事情库不会代办

MoveIt 的 `compute_gradient` 注释实际指最近点连线的单位向量，是三维空间方向，并非九个关节的距离导数。[5] 对分离且最近特征局部有效的一对点，控制适配可利用机器人最近点的运动 Jacobian 将方向映射到关节方向；这是局部几何推导，须校验点所在坐标系、方向正负及旋转贡献。

最近体素、盒面、边、角可能切换，最小距离函数可能不可微；只保留某一时刻的单个最近点并贯穿整个控制周期，不会自动得到安全的下一步约束。候选接近并列时要评估多约束、保守近似或有证明的选取规则，同时避免容量溢出被静默截断。精确策略属于后续控制/几何验收，不能据本研究声称已解决。

CBFpy 的 `Lfh/Lgh` 对状态 z 做 JVP，附加参数不会自动成为随时间演化的障碍状态。[11] FCL C++ 查询也不自动变成 JAX 可微函数。接入时需要明确传递/构造距离导数与障碍时变项，或采用经过验证的可微局部表示；不能把返回的距离作为常数传进 h 后期待 JAX 自动求出九轴梯度。即使距离为正，未知覆盖、过期或无可信导数仍需按既有失败契约处理。

## 当前仓库与目标链路的差距

本轮只读核对：`src/robot_safecontrol_moveit/oscbf_controller.py:118` 附近订阅 tracks，`:374` 附近解析位置、半径、速度等固定槽；`:305` 附近构造 `JaxControlLoop` 未提供 `sdf_shape`，而 `portable_oscbf/work/jax_control_facade.py:125` 根据该参数启用距离场。`portable_oscbf/work/jax_barrier_terms.py:25` 附近已有 OBB 对障碍球的距离路径。不能将这些代码解释为连续环境点云已经完整接入控制。

`portable_oscbf/work/safety_snapshot.py:211` 的距离场由命中格和 SciPy 无符号 EDT 构造，空占据返回固定远距离；该函数没有独立未知区域掩码。其注释中的 conservative 不能替代对覆盖、离散误差及调用方有效性门的证明。这些是当前代码观察，不是本轮故障测试结果。

## 首版比较与准入证据（建议）

推荐先比较 FCL 实体包络–体素路线和现有环境几何能力，不先决定删除文件；PCL 候选点＋解析几何作必要的简单核对，ESDF 后置。至少需要：

1. 几何：长连杆端部、贴面/边角、细杆、盒内障碍、工具附件、多体素并列；和独立参考检查距离误差/相交判定。确认环境包络真实外包围机器人。
2. 观测：遮挡、双源冲突、机器人贴近外物、空帧、TF/时钟偏差、障碍出现/离开/再遮挡；误删率和覆盖失效响应。
3. 控制：有限差分核验局部关节导数、特征切换、动态时变项、QP 容量/不可行与停车路径。
4. 性能：本机真实点量/全臂包络、双源和规划并发时的端到端延迟、尾延迟、超期次数，包含建树/快照/跨语言拷贝，不能仅计单次距离内核。

尚待用户决策：是否将“FCL 对局部占据体素的实体距离”作为首版优先验证候选，以及环境侧机器人包络是否优先验证复用现有 OBB。该研究不替用户接受，也不改变自碰撞 OBB 决议。

## 一手来源

1. [PCL 1.12.1 KdTreeFLANN API](https://raw.githubusercontent.com/PointCloudLibrary/pcl/pcl-1.12.1/kdtree/include/pcl/kdtree/kdtree_flann.h)。
2. [FCL 官方 README：形状和距离能力](https://github.com/flexible-collision-library/fcl)。
3. [FCL 0.7.0 Octree solver：OcTreeShapeDistanceRecurse](https://raw.githubusercontent.com/flexible-collision-library/fcl/0.7.0/include/fcl/narrowphase/detail/traversal/octree/octree_solver-inl.h)。
4. [MoveIt Humble PointCloudOctomapUpdater](https://raw.githubusercontent.com/moveit/moveit2/humble/moveit_ros/perception/pointcloud_octomap_updater/src/pointcloud_octomap_updater.cpp)。
5. [MoveIt Humble DistanceRequest / DistanceResultsData](https://raw.githubusercontent.com/moveit/moveit2/humble/moveit_core/collision_detection/include/moveit/collision_detection/collision_common.h)。
6. [voxblox EsdfMap 查询接口](https://raw.githubusercontent.com/ethz-asl/voxblox/master/voxblox/include/voxblox/core/esdf_map.h)。
7. [nvblox voxel 数据结构](https://raw.githubusercontent.com/nvidia-isaac/nvblox/public/nvblox/include/nvblox/map/voxels.h)。
8. [FCL 0.7.0 DistanceRequest 的 signed distance / nearest point 限制](https://raw.githubusercontent.com/flexible-collision-library/fcl/0.7.0/include/fcl/narrowphase/distance_request.h)。
9. [PCL 1.12.1 VoxelGrid 实现](https://raw.githubusercontent.com/PointCloudLibrary/pcl/pcl-1.12.1/filters/include/pcl/filters/impl/voxel_grid.hpp)。
10. [OctoMap 1.9.7 OccupancyOcTreeBase 更新实现](https://raw.githubusercontent.com/OctoMap/octomap/v1.9.7/octomap/include/octomap/OccupancyOcTreeBase.hxx)。
11. [CBFpy 官方 CBF 实现：h_and_Lfh / Lgh](https://raw.githubusercontent.com/StanfordASL/cbfpy/main/cbfpy/cbfs/cbf.py)。

既有项目上下文：[双源协作研究](dual-sensor-cooperation.md)、[感知上游研究](perception-upstream.md)、[CPU/GPU 部署研究](cpu-gpu-deployment.md)、[控制接入决策](../issues/05-control-adoption.md)。
