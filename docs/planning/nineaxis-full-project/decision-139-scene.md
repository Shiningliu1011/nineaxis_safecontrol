# [08] M03/M04 场景与预测碰撞接口

## 背景

对应 [#139](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/139)。当前研究使用现有九轴机械臂、`tool0`、工具局部 `+X` 与原蝴蝶圆柱路径。依据为[总体规格](../../specs/predictive-coordination-control.md)及已确认的[模型](decision-136-model.md)、[路径](decision-137-path-reference.md)、[限制](decision-138-limits.md)、[执行进度](decision-141-progress.md)与[复用安排](decision-149-reuse.md)。

M03 提供有来源、空间、时间和有效范围的环境事实；M04 为当前实时构型、各个预测构型及实际执行运动提供共同碰撞计算。入场、加工、停止、暂停构型调整、恢复和退出使用相同对象、几何、间距与失败规则。静态场景及单个预定低速规则运动障碍属于总体需求。用户于2026-10-27确认本页完整方案，接口实现与独立验收由地图已有任务交付。

## 现状核验

本轮读取主仓库 `main`，Git HEAD 为 `9c354d5505e26aa28b3e6fc323c661bd7542b105`；#108 的已验证改动存在于该分支未提交工作区。核验对象包含 #139/#154 需求及其实现入口。

| 对象 | 当前事实与证据 |
|---|---|
| 公共操作 | [collision_safety.py:200](../../../portable_oscbf/work/collision_safety.py:200) 已公开 `prepare_scene/query/certify`。`QueryBatch` 当前只有 `q/active_mask`，尚无每个状态的预测时间和请求空间，见[第88行](../../../portable_oscbf/work/collision_safety.py:88)。 |
| 连续证明 | 当前 `SegmentBatch` 保存起终关节状态与起终时间，尚未表达实际非线性运动；`certify` 在场景有效时仍返回 `CERTIFICATE_FAILED`，见[第376行](../../../portable_oscbf/work/collision_safety.py:376)。 |
| 场景覆盖 | [场景准入:232](../../../portable_oscbf/work/_collision_scene.py:232) 核验 checksum、身份、场景时间、容量、tracking、`required_space_covered` 和 hash。当前 hash 指向场景发布时的网格与必要区域，查询尚未核验该范围是否包含本次 `q` 或完整 `q(t)` 所需空间。 |
| 动态导数 | [环境 kernel:96](../../../portable_oscbf/work/_ellipsoid_point.py:96) 已计算 support 名义速度的时间项，半径增长的 `beta_dot` 与未来 support tube 由 #101 接续。 |
| 占据及固定并集 | #108 已在 main 实际源码下完成272项受影响测试，35.32 s；[完成记录](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/108#issuecomment-6440120209)保存来源。`FixedEnvironmentModel.from_mesh` 当前按完整 mesh 的 AABB 生成覆盖 support，见[第242行](../../../portable_oscbf/work/lidar_occupancy.py:242)。本项验收证明占据语义、保守并集、身份与容量处理。 |
| 当前几何 | [scene:296](../../../src/robot_safecontrol_moveit/static_experiment_config.py:296) 使用 `base_link` 中 `+Y` 轴圆柱工件与工作台；轴向范围为路径坐标范围各扩展100 mm。S0路径半径270.6 mm、工件半径220.6 mm，外部间距30 mm、自碰撞10 mm，见[配置:27](../../../config/static_experiment_s0.yaml:27)。 |
| 机器人几何 | [geometry artifact说明](../../modules/portable-oscbf/collision-geometry-artifact.md)记录10个link、全部45个self link pair与当前空允许接触列表。`tool0` 为 Link9 局部 `+X` 方向235 mm处工作点，当前工具几何归属Link9；实际附属工具需有对应来源与覆盖证据。 |

本轮调用当前 `ExperimentConfig.load_path()`，独立计算14984个已保留路径样本到实体圆柱侧面和填满AABB的距离。原MAT含14992点、23块，原时长29.982 s；当前 `PathGeometry.from_samples` 会移除数值零长度段，见[path_following.py:151](../../../portable_oscbf/work/path_following.py:151)。

| 实际输入与结果 | 数值 |
|---|---:|
| S0圆柱中心 / m | `[-0.00007246874682720339, 0.3466393505935161, 1.6372593390674266]` |
| S0圆柱高度 / m | `0.386433305528315` |
| TCP到圆柱侧面距离 / mm | `49.99999999999985` 至 `50.000000000000185` |
| TCP到填满AABB的最小有符号距离 / mm | `-10.336563933028703` |
| 位于填满AABB内的TCP样本数 | `1027 / 14984` |
| TCP距填满AABB少于30 mm的样本数 | `6657 / 14984` |

探针命令在主仓库运行：

```bash
source .scratch/host-recovery/env.sh
python /home/lsn/robot/map147-exploration/scene_interface_probe.py
```

退出码0，完整[探针](/home/lsn/robot/map147-exploration/scene_interface_probe.py)和[输入hash及结果](/home/lsn/robot/map147-exploration/scene-interface-probe.json)已保存。圆柱侧面距离为径向模长减去工件半径；AABB有符号距离使用各坐标超出半尺寸的欧氏距离及内部最近面距离。MAT SHA-256为 `79868b03afd68f9d4cca1cba568320f0f48efd355d020a9372f2037549c3293f`，S0配置SHA-256为 `2b2859312fca886a235b30d20727665b841e2f26e826f86fd26cda79a724c08e`。此处仅证明实际TCP样本的几何关系，完整机器人、工具、连续区间与共同约束由对应验收取得证据。

## 实施安排

### 对象、坐标和固定几何

| 对象 | 共同安排 |
|---|---|
| 机器人与工具 | 使用M02同一九轴模型、关节顺序、每个link变换、TCP、工具轴及标定。机器人本体、滑台、Link9及具有来源的当前附属工具进入同一geometry identity；固定工具变换应用一次。 |
| 工件和工作台 | 从当前蝴蝶场景读取圆柱中心、轴、半径、有限高度与工作台实体及安装变换。各对象保存来源、单位、建模/安装误差、模型revision和覆盖证据，转换为版本化 `support_point + rho_mm`。 |
| 固定实体表示 | #154对有限圆柱与box采用离线体积划分，只有经解析几何检查证明位于实体外的单元才能排除；保留单元由support球完整包围。保存完整划分覆盖证据、每个球的来源单元和误差界，并核查球体额外占据范围、容量与全路径可行性。半径包含表示范围及有来源的建模误差，要求间距单独保存。 |
| LiDAR并集 | 固定实体与扫描occupied取并集。LiDAR free保留其观测意义，固定实体保持occupied。未知、遮挡、歧义及年龄保留原状态与证据，固定实体之外的自由空间需有效观测。 |
| 对象对 | 全部机器人/工具与工件、工作台、附加障碍及全部要求的self pair均检查。允许接触只来自绑定当前geometry/policy的既有证据，当前空允许接触列表继续生效。 |
| 距离用途 | S0名义TCP工作距离50 mm由M01/M07保存其已应用状态；外部30 mm与self10 mm用于完整几何对象避碰。实际工作距离容差按#138记录，由工艺及现场测量提供。 |

固定体积覆盖证明与可用性分别核验。#154须对当前路径取得有效共同模型下的完整运动见证，验证整机/工具间距、保守表示、连续区间与停止过程，分别记录参考几何距离与M04约束余量。表示额外占据范围、组合ellipsoid或容量使场景无法通过时，保留具体失败与对象证据，按源数据和既有几何/参数任务调整可证明表示。工件实体、50 mm工作距离及30/10 mm研究间距沿用已确认来源。

### 三个共同操作

`prepare_scene/query/certify`为三个公开操作，NumPy/JAX和各方法共同消费同一几何、时间模型、对象对及policy。类型和布局在对应实现任务中版本化，维度和mask固定。

| 操作或数据 | 输入及核验 | 必需输出 |
|---|---|---|
| `prepare_scene(scene, identities)` | 完整不可变副本、grid/coverage数据、occupied支持及逐来源时间、固定实体、track/tube参数、checksum、layout、frame/单位、身份、容量和时钟映射；保留逐区域/逐support有效范围。 | 不可变 `PreparedScene`、覆盖域和证据、时间范围、状态及错误原因。每revision向本地JAX device传输一次，由#104负责实际发布与读取。 |
| `QueryBatch` | 每个活动行包含九维 `q`、`query_time_ns`、运行/时钟身份及请求身份；预测节点分别使用自己的构型与目标时间。所需空间由共同geometry及policy从该行状态保守计算并校验。 | 每行目标时间、空间请求hash与coverage核验结果进入返回证据。实时查询使用该状态对应的检查/生效时间，状态估计有效性由M05/M10核验。 |
| `query(..., query_mode)` | 在同一预测时间求机器人变换、环境中心和不确定半径，校验整个所需空间处于已证明覆盖域；分别核验当下freshness和未来tube有效性。 | 三种mode、self/environment独立barrier行、`grad_h_q`、完整`partial_h_partial_t`、solver health、活动pair及状态mask；外部毫米距离、要求间距、剩余余量和最近点保持其明确诊断用途。 |
| `SegmentBatch` | 完整可重建的实际 `q(t)`、起止时间和时钟身份、每段系数/边界/有效mask、命令hash及模型/配置身份。实际Ruckig保存真实jerk分段、跟踪截断位置与停止后续运动；线性关节段明确声明线性表达。 | 同一表达用于状态查询、运动范围及起终状态与导数接续核验。 |
| `certify(..., prepared_scene)` | 对实际完整运动及停止区间，检查全部机器人实体运动范围、整个区间必要空间、环境可达tube和self pair下界；无法直接证明时按artifact预算二分。 | 固定结构有效mask、证明结论、下界及量纲、失败对象/区间、深度、耗时和完整输入身份。全部区间均已证明后返回通过。 |
| 结果身份 | 保留`scene_epoch/revision`、geometry、kernel、policy与固定环境revision；模型、TCP/工具、标定、工件/场景配置、运行及时间映射通过明确字段或受hash绑定的manifest核验。 | `request_hash`绑定实际`q`或`q(t)`、活动行、目标时间/区间、必要空间与身份；任一输入变化后重新检查。 |

`required_space_hash`继续标识场景发布时的网格、坐标、分辨率和必要空间配置。本次请求hash绑定当前查询。M04根据完整机器人geometry、要求间距、误差及整个运动范围保守生成空间需求；coverage payload须证明该需求包含于有效观测或固定实体证据的范围。所需范围跨越网格边界、未观测单元、遮挡或歧义区域时返回 `UNKNOWN_REQUIRED_SPACE`。与需求范围相交的外部support及其不确定范围全部计入，保留网格边界外来源的必要贡献。

### 动态、时间与失败

| 项目 | 规则 |
|---|---|
| 存在性 | occupancy保存障碍存在事实；可信track提供名义运动及误差，track中断后保持occupied并采用有证据的未跟踪上界。 |
| 时间表达 | 观测、状态、预测物理时间与命令区间使用同一运行时钟或经验证映射，映射误差进入范围。仿真使用共同仿真时间，真实设备使用明确单调时间与映射，耗时使用计算设备单调计时；具体映射及期限由#142/#151确认。 |
| future support | #101从原观测/关联时间构造`p(t)`和`rho(t)`，保留几何、测量、标定、时间与运动误差分项，连续覆盖至当前及未来。合并support包围全部成员及各自运动，已有年龄范围按来源计入一次。 |
| 当前导数 | `h=point_scale_sq-beta(t)^2`的时间项包括名义中心速度和`-2*beta*beta_dot`；关节运动由`grad_h_q`表达。独立毫米距离与barrier在相同目标时间使用相同support。 |
| 三类期限 | 当下freshness、tube/coverage物理有效区间、本次query/certify计算deadline分别记录并检查。 |
| 无效和容量 | 沿用`REVISION_PENDING/INVALID_SCENE/UNKNOWN_REQUIRED_SPACE/CAPACITY_OVERFLOW/CONSTRAINT_OVERFLOW/SOLVER_UNHEALTHY/CERTIFICATE_FAILED/DEADLINE_MISSED`。shape、配置和非有限调用输入立即拒绝，运行非`OK`清除相应准入mask。 |
| 版本变化 | 模型、工具、标定、policy、固定实体、时钟映射或scene revision变化后旧未来证明失效。按#151重新检查执行前视与停止，记录等待、重新准入或终止原因；运动与参考按#141共同区间结算。 |

静态场景需有效安装几何和覆盖身份。受控动态场景同时具备障碍物真实时间、观测条件与完整请求区间可达范围。未来节点、离散采样及完整连续运动分别保存各自证据。

### 验收与任务归属

| 验收内容 | 公开证据与责任 |
|---|---|
| 圆柱、工作台及九轴/工具 | #154保存实体来源及完整支持覆盖证明；核验端面、侧面、台边缘、接近/接触/相交、原路径样本及连续区间。独立解析圆柱/box和高精度参考核验距离、witness及梯度，既有FCL/OBB用于差异定位；附属工具缺失时明确适用范围。 |
| 逐请求覆盖 | #154通过实际三个操作，核验同scene不同q、同q不同预测时间、未来/停止越界、边界外support、空support与unknown，以及身份相同但内容改变。未覆盖部分全部拒绝。 |
| 感知及动态 | #105验证逐点时间mesh self-filter；#101验证track中断/恢复、未跟踪运动、时间导数及完整future tube；#104验证实际进程、双缓冲、唯一写入、checksum与reader复制；#122完成预先固定真实LiDAR回放验收。 |
| 查询和连续证明 | #112验证逐行barrier、固定K、活动集合、遗漏下界、overflow及health；#113验证真实Ruckig分段、停止、直接/二分证明、内部碰撞、动态tube、预算及identity变化；#125使用独立解析、高精度和官方参考验收正确性。 |
| 参数与性能 | #117取得误差、运动范围、间距、容量、solver及deadline生产来源；#119在目标设备记录prepare、device transfer、query、QP和规定完整路径的p99.9、最大值与超时。#142/#144/#151/#169确定实时/MPC周期和计时边界，本方案不增加数值。 |
| 闭环和生产 | #160/#166提供完整九轴仿真、停止与任务证据；#123在切换证据满足后接入单一CollisionSafety来源。sim、shadow、live分别遵守对应准入要求，本地探针和模型测试分别声明范围。 |

## 确认记录

用户于2026-10-27回复“确认推荐方案”，以下条目共同定案。完整方案及依据已发表在[#139方案评论](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/139#issuecomment-6440986386)。

| 确认事项 | 定案 | 后果与依据 |
|---|---|---|
| 场景及对象 | 当前九轴、原蝴蝶圆柱、工作台、完整机器人/工具及已有障碍任务；对象对与允许接触绑定geometry/policy来源。 | 与#136/#137及总体规格共同使用同一对象，#154取得独立证据，附属工具和安装资料由既有任务提供。 |
| 固定实体及可用性 | 有完整体积覆盖和误差界的局部support与LiDAR occupied取并集，同时核验全路径整机可行性；50 mm工作距离、30 mm外距、10 mm自碰撞分别检查。 | 原蝴蝶继续采用；14984样本的独立几何计算为表示质量验收提供依据，#108作为保守并集来源。 |
| 时间和覆盖 | 三个操作共用逐状态目标时间、完整身份及请求hash，从实际q或完整q(t)计算必要空间并验证包含关系。 | 预测、当前控制和停止分别取得证据，空间或未来时间超出范围时拒绝。 |
| 实际运动证明 | 可重建的实际Ruckig分段和停止后续运动，全部区间下界证明作为准入条件；参数及预算来自artifact。 | 运动生成或实时修正后重新检查，证明有效期及执行区间与#141一致。 |
| 实现和验收归属 | 保留#105/#101/#104/#112/#113/#125/#154/#117/#119等职责，分别保存模块、回放、设备及闭环证据。 | 决定票确认接口与证据要求，现场标定、物理范围、周期及生产切换由相应任务验证。 |

本票制品包括本页完整决定、仓库外的 `scene-interface-proposal.md`、`scene_interface_probe.py`、`scene-interface-probe.json` 和 `verify_scene_proposal.py`，以及对应 GitHub 方案与确认记录。探针和接口核验各自保留实际来源，完整实现、连续运动及设备能力按任务验收。

## 供 spec 采用的要点

- M03/M04共用当前九轴和蝴蝶圆柱场景，机器人、工具、工件、工作台、扫描障碍及self pair具有来源、模型与标定身份。
- 固定support完整覆盖原实体，保存表示误差及额外占据范围，验证整条路径与停止可行性；固定模型和LiDAR occupied取并集。
- `prepare_scene/query/certify`为共同接口，每个q与预测时间分别求值，实际q(t)和停止共用区间、命令身份及连续证明。
- 覆盖域身份与本次运动/空间请求身份分别保存，每次请求验证必要空间的包含关系和物理有效期。
- 动态support保存原观测与误差分项，统一计算名义速度、`beta_dot`及未来tube；freshness、预测有效期与计算deadline分别检查。
- 几何、导数、连续区间、真实感知、设备性能、闭环和生产接入按既有任务验收，缺少现场数据时保持明确未验证状态。
