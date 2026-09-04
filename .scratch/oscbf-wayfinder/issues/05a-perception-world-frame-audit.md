# 05A — 感知世界坐标系审计：base_link 与「固定环境系」冲突（research）

**What to build:** 现状：控制器与感知的 `world_frame` 均为 `base_link`，而
`docs/specs/dual_sensor_perception_fusion_spec.md` 要求固定环境坐标系。机器人在
执行过渡/跟踪时基座不动的假设下二者等价；一旦机器人基座移动（或后续任务含
移动），占据点云/障碍槽会跟着世界漂移。本票审计并产出影响面清单：

- 当前 `perception_bridge` → `/perception/tracks` → controller 各环节中
  `world_frame` 是被消费还是仅存在 YAML/备注里；
- 外参占位（LiDAR/depth extrinsics YAML placeholder）在代码里被谁消费、
  标定失败时的默认行为；
- 若要切到固定环境系，需要改哪几个接口（时间戳/变换随附、坐标系参数）；
- 给出「继续用 base_link」与「固定环境系」两种方案的代码影响面对比表。

**Blocked by:** None — 纯审计/文档产出。「选哪个坐标系」由 **05B（grilling 决策票）** 拍板。

**Type:** research（AFK）

**Queue:** wayfinder-core — 审计 AFK 可做；选系决策在 05B。
**Tracker:** #4 (https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/4)

**Status:** resolved (2026-09-04, AFK research — audit complete, decision delegated to 05B)

- [x] 追踪 world_frame 从 spec → yaml → 代码的完整链路（标注哪些是死配置）
- [x] 追踪外参占位在代码中的消费点与失败路径
- [x] 产出影响面清单（需要改动的文件/接口/测试）
- [x] 两种坐标系方案对比表写入报告

---

# Resolution — 感知世界坐标系审计（2026-09-04）

**结论（一句话）：** 当前整条感知→CBF 链在数学上是自洽的——点云、占据栅格、tracks、
控制器 FK 几何全部以 `base_link` 为共同坐标系；但 `world_frame` 参数本身在今天
**并不控制**任何坐标变换（它只是输出消息 frame_id + 休眠中的 TF 目标），真正绑定
frame 语义的是两个静态外参矩阵和控制器侧的隐式约定；`config/sensor_extrinsics.yaml`
是**完全未被消费的死配置**。同 frame 自洽成立的前提是「基座永远不动」；一旦基座
运动，三层占据模型/ESDF/track 速度估计全部出现语义破坏（详见 §5）。

本文档按【代码事实 → 引用】呈现，不做 05B 决策。所有行号以 2026-09-04 main 分支为准。

---

## 1. 当前真实 frame graph（逐行追踪）

```
Livox LiDAR frame (livox_frame, input_frame_lidar)
   │  当前未启用: source_topic_lidar="" 不创建订阅 (perception_bridge.py:160-165)
   │  [启用时] _lidar_callback → _sensor_callback(...) (perception_bridge.py:314-318)
   ▼
_sensor_to_world(input_frame, msg.header.stamp, static_param) (perception_bridge.py:250-281)
   │  · use_tf=True 时: lookup_transform(world_frame, sensor_frame, msg.stamp)
   │    → 失败 → 回落 latest (Time()) → 失败 → 静态矩阵 → identity (perception_bridge.py:258-281)
   │  · use_tf=False (当前配置): 直接用 static_param 参数; 空 → identity
   │  · TF 目标 frame = self._world_frame; source frame = sensor frame; 时间戳 = 消息 header.stamp
   ▼
preprocess_points: world = T @ [sensor_pts;1], 工作空间裁剪, 源体素降采样
   (safety_snapshot.py:177-208)
   ▼
deque 缓冲 (world_pts, stamp_s, s2w) (perception_bridge.py:306-312)
   │  ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ── ──
   │  注意: s2w 存进缓冲后即被丢弃!
   │  _fusion_callback: for pts, stamp, _s2w in ...: engine.feed_lidar(pts, stamp)
   │  (perception_bridge.py:354-358) → engine 只拿到「已变换到 world 的点」+ 时间戳
   ▼
FusionEngine (frame-agnostic, 无任何 frame 参数/字段; fusion_engine.py:60-138)
   │  · 时间戳配对 / 新鲜度 / 跨传感器 dt / 重复帧 / 合并 / 融合体素 (fusion_engine.py:139-307)
   │  · OccupancyTracker.update(points_world, fusion_stamp) (fusion_engine.py:264-265)
   │    体素索引 = floor((pts - workspace_min)/voxel) (static_occupancy.py:70-78)
   │    → 体素身份 = 「base_link 坐标系中的格」, 不是「环境中的格」
   │  · cluster_into_tracks(unconfirmed_pts, ...) (fusion_engine.py:271-278)
   │    → track pos/vel 在「世界系」中估计 (dynamic_clustering.py:6-8, 12, 22-24)
   ▼
/perception/tracks (Float32MultiArray, 8 槽 × 10 float) (perception_bridge.py:446-459)
   │  layout: px,py,pz | r | vx,vy,vz | enabled | d_safe | alpha (perception_bridge.py:64-65)
   │  ⚠ 消息无 header / 无 frame_id / 无 time 字段 —— frame 信息完全不随消息传递!
   │  frame 只是「约定」= bridge 的 world_frame (=base_link) + 静态外参链的目标
   ▼
OscbfController._tracks_callback → obs_* 缓存 (oscbf_controller.py:370-385)
   │  · 解码: obs_pos/obs_radii/obs_enabled/obs_d_safe/obs_vel/obs_alpha
   │    obs_radius_dot 未解码 → 内核默认全 0 (jax_control_facade.py:765-767)
   │  · 无 frame 校验/无 frame 假设检查
   ▼
JAX CBF 内核 build_constraint_terms (jax_kernel_factory.py:179-230)
   │  q → robot.environment_collision_data(q) = POE FK 输出
   │  h_obs = obb_sphere_clearance(q, obs_pos, obs_radii, obs_vel, ...) (jax_kernel_factory.py:208-213)
   │  → 距离 = 位置直接量算术 (jax_barrier_terms.py:19-22, dpax_collision.py obb_sphere_clearance)
   │  → 假设: 机器人几何与障碍几何「已在同一坐标系的同一单位 (m) 下」
   ▼
robot collision geometry (OBB / 碰撞球 / ee_pos) — POE 基座系
   JOINT_CHAIN 根: ("world","base_link","fixed", 0,0,0,...) 恒等 (kinematics_data.py:18)
   URDF 根 link = base_link, 无 world link (ninezzhou.urdf:8, 无 "<link name=world")
   → 机器人几何表达在 base_link
```

### 每一步的契约表

| 步骤 | 输入 frame_id | TF 查询目标 | TF 时间戳 | 输出 frame | frame 信息随消息传递? |
|---|---|---|---|---|---|
| Camera 回调 | `camera_color_optical_frame` (config/perception_runtime.yaml:24) | `world_frame`(=base_link), 仅 use_tf=True | msg.header.stamp; 失败回落 Time()=latest | 感知世界系 = base_link | 否(数组存点, 已变换) |
| LiDAR 回调 | `input_frame_lidar`(当前 "" 未启用) | 同上 | 同上 | 同上 | 否 |
| FusionEngine | 无帧概念(输入即世界系) | — | — | 无(输出同输入) | 否 |
| OccupancyTracker | 世界系点 | — | fusion_stamp (时间戳驱动占据) | 体素中心(世界系) | 体素索引隐含帧 |
| /perception/tracks | —(无 header) | — | 无时间字段 | world 系 | **否** — 纯 float 数组 |
| controller obs_* | 无 | — | — | 假设 = FK 系 (base_link) | 否 |
| CBF distance | base_link (FK 输出) | — | — | 同帧算术 | 不适用 |

### YAML 中存在但没有被真正消费的 frame 参数

| 参数 | 位置 | 消费状态 |
|---|---|---|
| `config/sensor_extrinsics.yaml` **整个文件** | 仓库根 config/ | **零代码引用**（grep 全仓仅 docs/planning/.scratch 提及）。占位矩阵、calibration_method/error_estimate、parent/child_frame、orbbec_launch 全部未加载 |
| `parent_frame / child_frame / child_frame_lidar` | sensor_extrinsics.yaml | 未消费 |
| `calibration_method* / calibration_error_estimate_*` | sensor_extrinsics.yaml | 未消费 |
| `world_frame` (portable_oscbf/config/obstacle_params.yaml:16-17) | 默认值源 | 只作为 ROS 参数**默认值**被感知桥 `_declare_parameters` 读取 (perception_bridge.py:211-214 → perception_config.py:116)，随后被 perception_runtime.yaml 的同值覆盖 |
| `input_frame_lidar: ""` / `source_topic_lidar: ""` | perception_runtime.yaml | LiDAR 未订阅 → 整条链死配置（启用时才有意义） |
| `use_tf: false` | perception_runtime.yaml | 整条 TF 查询路径休眠 |
| `voxel_size`（obstacle_params）| — | 被 `spec_of` 消费 (perception_config.py:145-151)，但与 `source_voxel_*`/`fusion_voxel_m` 中若干值存在重复定义（09 票范畴） |

---

## 2. world_frame = base_link 当前属于哪一类 → **B（输出 frame_id）+ 休眠的 A（TF 目标）;不是 C, 是 '只按约定' 的 A'**

**所有读取点（完整清单）：**
- `perception_bridge.py:86` — `self._world_frame = str(self.get_parameter("world_frame").value)`
- `perception_bridge.py:205` — 启动日志
- `perception_bridge.py:258-277` — `_sensor_to_world` 的 TF 目标 frame（**条件: use_tf=True**, 当前 use_tf=false → 代码路径不可达）
- `perception_bridge.py:399 / 411 / 466` — `_make_header` → frame_id: `/perception/instant_occupancy`、`/perception/cloud_world`、`/collision_object`
- `perception_config.py:36,116` — 配置字段定义/默认值
- `portable_oscbf/tests/test_perception_pipeline.py:44` — 测试断言 `world_frame == "base_link"`
- `config/perception_runtime.yaml:35` — 运行时值

**消费点（真正影响行为的）：**
1. **输出消息 frame_id**（3 个话题的头）— 纯元数据,repo 内无下游消费者对 frame_id 做校验（MoveIt planning scene 只读 /collision_object 位姿,不校验帧）
2. **TF 查询目标** — 休眠（use_tf=false）
3. **静态矩阵链的「语义命名」** — 这是决定性事实:今天真正把点云放进 base_link 的是 `camera_to_world_static` 矩阵的**定义语义**（child_frame→parent_frame,parent 被注释/约定为 base_link）+ 控制器对 FK 系=base_link 的隐式假定,而不是 `world_frame` 参数值本身。

**关键推论：** 如果把 `world_frame` 改为 `map` 而保持 use_tf=false、静态矩阵不变 → 数据仍全部在 base_link,只有消息 header 变成 map → **静默不一致,无任何校验**。`world_frame` 参数与静态矩阵的「目标帧」之间没有任何一致性检查（矩阵参数名称写死为 `*_to_world_static`,把语义烘焙进了名字）。

因此 05B 决策时应注意：「切到固定环境系」**不能只改 world_frame 参数**——必须同时改外参目标帧 / 引入 TF / 或加控制器侧变换,否则只是改了个字符串。

---

## 3. LiDAR / depth 外参当前真实状态

### 3.1 实际被加载的外参（vs 占位）

| 项 | 真实来源 | 值 | 状态 |
|---|---|---|---|
| Camera→world | ROS 参数 `camera_to_world_static`（默认=单位阵,perception_bridge.py:220-223;**被 config/perception_runtime.yaml:25 覆盖为一个非单位阵**） | R=[[0,0,1],[-1,0,0],[0,-1,0]], t=(-0.9, 0.4, 1.7)。注释 `bx=cz, by=-cx, bz=-cy, t=[-0.9, 0.4, 1.7]` | **被加载,非占位,但无标定来源**——是写死的「假定安装位姿」 |
| LiDAR→world | ROS 参数 `lidar_to_world_static`（perception_runtime.yaml:31） | **单位阵（占位）** | 加载但**未生效**——`source_topic_lidar: ""` 不订阅 LiDAR,整条链不运行 |
| sensor_extrinsics.yaml 中的两个占位 4×4 | — | 单位阵 | **未加载**（死文件）——与上文「实际加载值」是两个不同文件,勿混淆 |

### 3.2 如果 TF 缺失 / 外参缺失 → 行为链
`_sensor_to_world`（perception_bridge.py:258-281）:
1. 若 use_tf=True 且 frame 不同 → lookup_transform(目标=world_frame, 源=sensor_frame, msg.stamp);失败 → warn(逐帧!)
2. 回落 `Time()`(=0 → tf2 语义 = 最新可用变换);失败 → warn
3. 回落静态参数;空 → identity

**结论：不是 fail-fast、不是 drop frame。是「log 警告 + 回落到静态参数/单位阵」的 silent degradation**——点云照常进入管线,若落到单位阵则一切被解释为「传感器与 base_link 重合」。当前 use_tf=false,步骤 1/2 不可达,步骤 3 恒生效。

### 3.3 外参 timestamp / calibration provenance / 单位 / rotation convention
- **Timestamp:** 无。静态矩阵无时间戳;TF 路径有时间戳（msg.stamp + latest 回落）但当前未启用。time-varying 外参不存在。
- **Provenance:** 无。perception_runtime.yaml 只有一行注释;calibration_method/error_estimate 字段只存在于未被消费的 sensor_extrinsics.yaml（camera: "manual_measurement", error=null;LiDAR: 全 null）。
- **单位:** 米（safety_snapshot.py 直接按米处理;矩阵注释 "12 axes: 毫米?" 不成立——配置注释明确 m,ESDF/voxel 全部米）。
- **Rotation convention:** 4×4 齐次、行主序 16 float（sensor_extrinsics.yaml 注释）,`p_world = R @ p_sensor + t`（safety_snapshot.py:191-192）。感知侧矩阵列向量 = 传感器轴在世界系中的方向;不遵循 REP-103 标注但遵循光学坐标系（Z 前向、Y down、X right）的典型映射。
- **既有测试注:** `tests/test_perception_bridge_demo.py:94` 的注释承认「占位: world_frame==input 时 bridge 用 identity/camera_to_world_static」。

---

## 4. 方案 A（base_link）vs 方案 B（固定环境系）影响面对比矩阵

| 维度 | 方案 A: world_frame = base_link | 方案 B: 固定环境系 (map/workcell/world) |
|---|---|---|
| **固定基座机器人**（现状） | 数学等价,零改动,零风险 | 需要先定义环境帧（repo 中不存在）+ 基座→环境系标定;收益为零（等价） |
| **transition 阶段** | 无影响——transition planning（AEB-RRT*）是关节空间规划,不消费感知 frame | 无影响（同左,感知不参与 transition） |
| **tracking 阶段** | 现状自洽（FK 与 tracks 同系） | 控制器需把 tracks/ESDF 从环境系变换回 FK 系（或内核显式带 frame 输入）,否则距离/速度计算静默错误 |
| **以后基座移动**（移动平台/整机移位/升降台） | **语义崩溃**:占据/ESDF/tracks 整体错;环境随 base_link 平移,误差 = 基座位移 | 环境系稳定,感知不受基座运动影响;机器人几何按 base_link 动态经 TF 变换回环境系即可 |
| **静态障碍物**（ESDF/static 层） | 基座不动时 OK;基座动 → GRID 内两代快照错位积存（旧占据 0.3s 超时前 + 新帧并存 = 重影） | 环境帧中体素身份 = 环境格,持久化语义正确 |
| **动态障碍物**（tracks/instant） | 基座不动时 OK;基座动 → (a) 静态障碍在 base_link 中出现视速度 ≈ -v_base,可能被聚类成「移动障碍」;(b) 跨帧关联/速度估计被基座速度污染 | 环境帧中静态即静态,速度估计=真实环境速度 |
| **occupancy persistence** | 体素索引 = base_link 格（static_occupancy.py:70-78）;基座一动,持续占据窗口跨帧语义失效 | 体素索引 = 环境格,持续/静态确认语义稳定 |
| **LiDAR + depth 融合** | 两传感器各自经静态外参到 base_link,融合无帧概念,自洽 | 需要外参目标帧改为环境系（或传感器→base_link 之后再 base_link→env）,TF 链/消息时间戳查询复杂度上升 |
| **CBF obstacle velocity** | obs_vel 在 base_link;基座静止时 = 环境真实速度,相对速度项有效 | 环境系速度;控制器侧需 T_base_env 动态项修正为相对速度才能用（或把速度也变换） |
| **calibration complexity** | 只需 2 个传感器→base_link 外参（+ 其动态项无） | 需 2×传感器→基座 外参 + 1×基座→环境系位姿;若基座可动还需动态 T_base_env(t) |
| **真机部署** | 「落地即用」:装传感器→测外参→填矩阵;无环境系概念;与 spec「world_frame 必须是固定环境系」**直接冲突**（spec 草案,可由 05B 修改） | 需要环境系物理基准（地面/立柱/工装角点）;额外测量;但长期可迁移、可复用、符合 spec |

**05B 的两个候选的实现影响面（改动清单,供决策参考,本票不动手）:**

- **方案 B 最小改动（无 TF,静态）:** perception_bridge 的 `world_frame` 参数改环境帧名 + 两个静态矩阵改为 sensor→env（或把 base_link→env 常量乘进现有矩阵）+ 控制器侧与感知侧达成「tracks 在环境系」的约定并加**显式变换**（controller 内 FK 系=base_link → 需要 obs_pos/obs_vel 用 T_env_base(已知/静态) 转回 base_link 之后才能进内核）;测试更新（test_perception_pipeline.py:44 断言、test_perception_bridge_demo.py）。
- **方案 B 完整（动态基座/移动基座）:** 环境帧 TF（base_link→map/odom 动态发布）+ perception use_tf=true 按 msg.stamp 查询 sensor→env + 控制器侧查询 env→base_link @ 消息到达时刻 + ESDF 采样前变换（perception_demo 路径）+ track 速度的相对速度修正。涉及文件: perception_bridge.py、jax_control_facade.py（或新增 controller 适配层）、oscbf_controller.py、perception_demo.py、launch、3 个 YAML、相关测试。
- **方案 A 维持现状:** 无代码改动;需要归档「spec 与实现冲突」为已知偏差（或修订 spec,05B 拍板）。

---

## 5. 特别检查：occupancy persistence 与「环境跟着机器人移动」

**事实链：** `OccupancyTracker`（static_occupancy.py:25-113）的体素身份 = `floor((pts - workspace_min)/voxel)` 索引（:70-78）,`_first_seen/_last_seen/_prev_occupied`（:46-50）全部按体素索引存储,`static_confirm_s=0.5s`、`occupancy_timeout_s=0.3s`（config: perception_runtime.yaml:37-38）。ESDF 由 static 层体素构建（fusion_engine.py:268）;track 速度由跨帧关联位置差估计（dynamic_clustering.py:69-95, fusion_engine.py:269-278 用 fusion 时间戳差做 dt）。

**结论：如果 world_frame=base_link 而基座(或整机/升降台)在运动 → 三层占据模型出现确凿的语义问题：**

1. 同一环境点在**不同时刻**映射到**不同体素索引**（grid 随 base_link 平移）→ 持续占据被中断,静态确认重计时;
2. 旧位姿的静态体素（0.3s 超时前未清理）+ 新位姿的新帧**并存于同一 grid** → ESDF 出现「双影」,距离场给出错误（偏大/偏虚）的障碍位置;
3. 环境在 grid 中整体以 -v_base 视运动 → 静止环境点可能被 unconfirmed 聚类成「移动 track」并估计出 ≈ -v_base 的速度 → CBF 收到假的动态障碍;
4. 即使在**单帧**内点云几何仍自洽（外参=静态矩阵,传感器刚体安装于 base_link;每帧变换正确）,跨帧的 persistence/速度语义全部失效。

**当前项目基座固定吗? —— 代码证据显示: 仿真闭环中固定。**
- URDF 根 = base_link（ninezzhou.urdf:8,无 world link）;J1 是 prismatic 升降轴,**在链内**（base_link→Link1, kin data:19）——升降运动移动的是 Link1 及臂,不是 base_link;
- POE 根固定恒等（kinematics_data.py:18）;MuJoCo viewer 把 base 放在世界原点（mujoco_viewer_with_cylinder.py:588-593 注释）;`oscbf_plant` 的 randomize 只随机关节起点（oscbf_plant.py:84,129-137）,无基座位姿状态;
- 没有任何节点/launch 部件发布 base_link 的平移/旋转（grep 无 base 运动路径）。
- **与 spec 的口径差异:** spec 用户故事 19/「坐标系与 TF」note 声称 "base_link 随升降轴运动"— 与 URDF 不符（升降轴 J1 在 base_link 之下而非之上）。真机基座是否落地安装 / 是否有升降台 = **Not yet specified**（repo 无真机数据,README/CONTEXT 不算事实源）。

**在什么假设下 base_link 与固定环境系数学等价（形式化）:**

```
假设环境系 E 固连于某个物理基准（地面/立柱/工装）。
若对全程 t ∈ [0, T_run] 有
  (1) T_E^base(t) = T_const   （基座在环境系中零平移、零旋转——严格为常矩阵）
  (2) T_base^sensor(t) = T_const  （传感器刚体安装于基座,无姿态漂移）
  (3) 传感器消息时间戳可靠映射到变换生效时刻（当前 use_tf=false 下为常矩阵,自动满足; use_tf=true 时 TF 树里 base_link 有唯一祖先链）
则对每个测量点 p_sensor：
  p_world := T_base^sensor · p_sensor → 与 p_env := T_E^base · T_base^sensor · p_sensor 只差一个
  常量刚体变换 T_E^base。
  于是: 机器人↔障碍之间的欧氏距离、CBF 时间项（相对速度）、体素占据/静态确认/
  track 关联 —— 全部对这些量做刚性不变运算 —— 在 base_link 中计算与在环境系中
  计算给出完全相同的结果（距离=‖Δp‖, 相对速度投影=刚性不变）。
  等价成立所需的全部条件 = (1)(2)(3) + 两处使用同一米单位。
```

**关键界线:** 等价条件 (1) 破坏 ≠ 立即碰撞错误——**单帧**几何仍正确（传感器视角自洽）;破坏的是**跨帧持久化状态**（0.3s/0.5s 窗口）与 **速度估计**。连续运动下（例如基座以 0.1 m/s 移动,0.5s 内累计 50mm > 体素 30mm）静态确认在三个体素之外,ESDF 双影;track 速度包含 -v_base 分量。

---

## 6. CBF frame contract（当前隐含契约,形式化）

**现状全部代码事实:**

- `q →` POE FK / link transforms：输出在 POE 基座系 = base_link（dupax_collision.py:80-93 `link_transforms`;nineaxis_manipulator_jax.py:70-93, 211-221; kinematics_data.py:18 固定根恒等）。
- 机器人碰撞几何：`robot.environment_collision_data(q)` → (N,4) `[x,y,z,radius]`（kernelfactory.py:204-207）;OBB vs 球用 `obb_sphere_clearance(q, obs_pos, ...)`（jax_barrier_terms.py:25-37 → dpax_collision.py）。
- 障碍输入：obs_pos/obs_vel 直接来自 /perception/tracks（oscbf_controller.py:378-385）→ facade `_normalise_obstacle_inputs`（jax_control_facade.py:757-770,只做 None→零,无 frame 处理）。
- 距离 kernel：`compute_obstacle_clearance` = `‖center_deltas‖ - r_robot - r_obs - d_safe`（jax_barrier_terms.py:19-22）,`compute_obstacle_time_terms` = `-nᵀ v_obs - ṙ_obs`（:40-48）——**纯同系向量代数,零 frame 感知,零校验**。

**正式 frame contract（当前系统隐含唯一有效契约）:**

```
[C1] 设 F_b := base_link/POE 基座系（URDF 根）。
[C2] 机器人几何 G_robot(q) 必须且只能在 F_b 中计算（现状=POE FK,满足）。
[C3] 障碍几何 obs_pos / obs_radii / obs_vel / obs_radius_dot 必须表达在同一个
     右手系、同单位（m）、与 G_robot(q) 相同的坐标系 F_b 中。
[C4] /perception/tracks 不携带 frame 元数据 → 集成层必须保证感知外部约定
     （world_frame 参数 + 静态外参矩阵的目标帧）== F_b,并把它当作不可变的
     跨包契约;任何一端改动坐标系都必须同步另一端。
[C5] ESCDF（目前仅 perception_demo 消费;sdf_origin=spec.workspace_min,
     定义在 world_frame）的采样点 = 机器人碰撞球位置（在 F_b）,同样要求
     world_frame == F_b,或在采样前对 (origin, voxel) 做显式变换。
[C6] 无任何运行期校验手段验证 C3/C4 —— 违反时是「静默的几何/速度错误」,
     不是异常。
```

（可选补充: 若要打破 C4（例如切环境系）,唯一正确方式是在**自适应层**:
 每周期先算 `T_Fb^env(t)`（静态或 TF 查询）,把 obs_pos/obs_vel 与 sdf_origin
 变换回 F_b 再进 kernel;kernel 本体保持 frame-agnostic——这也是 05B 建议迁移面
 小的原因。）

---

## 7. 05B 决策输入（不代为选择）

### Option A — base_link
- **优点:** 与当前 100% 自洽;零代码改动;标定量最少（仅传感器→base_link 外参）;真机「装好即用」;ESDF/占据/tracks/速度全链路无新增变换;姿态/轨迹误差语义与 06B 验收口径一致（控制器内部量纲）。
- **风险:** 基座一旦运动（移动基座/升降台/整机重新定位）→ §5 描述的占据/ESDF/速度语义破坏;与 spec 用户故事 19 及「坐标系与 TF」节**明确冲突**（spec:275 "world_frame 必须是固定环境系,不是 base_link"）;未来任务与外部工装/场景坐标系对齐时需要额外变换;文档/契约中「world」一词在感知侧实指 base_link,容易造成集成误读。
- **前提:** 基座永久固定（落地安装并锁定）+ 传感器刚体安装;且明确接受「基座不可动」作为系统边界,写入 spec 偏差登记。
- **实现影响:** 无生产代码改动;仅文档（spec 修订/偏差登记 + 注释澄清 world_frame 语义）。

### Option B — fixed environment frame
- **优点:** 环境语义稳定（占据/ESDF/tracks 与基座运动解耦）;兼容未来移动/升降;与外部工装/多机对齐;符合 spec;为后续「任务级」（环境坐标系上的任务目标）留基础。
- **风险:** 引入新标定（基座→环境系位姿）与新的帧管理义务;控制器/ESDF 消费侧必须做**显式变换**,漏掉任何一处都是静默几何错误;若基座可动还需动态 TF 与逐周期查询,涉及延迟/时间戳一致性问题（与 ticket 10 交织）;测试断言需更新。
- **前提:** 环境系有唯一权威定义/物理基准;基座→环境系可测量;传感器外参目标帧=环境系（或维持 sensor→base_link 再加 base_link→env）。
- **实现影响:**（按改动从小到大)
  1. 感知侧不变（sensor→base_link 外参不变）,控制器侧加「env→base_link」变换(静态常量) — 最小;
  2. 感知侧改 world_frame=env + 两个静态矩阵改 sensor→env,控制器侧加「env→base_link」变换;
  3. 全 TF 化（use_tf=true + env TF 树 + 逐周期查询 + 速度相对化）;
  涉及文件: perception_bridge.py(可选)、oscbf_controller.py 或新适配层、jax_control_facade(变换接入点)、perception_demo.py、3 个 YAML、launch、tests/test_perception_pipeline.py:44、tests/test_perception_bridge_demo.py:94。

### Calibration choices（至少四项对比）

| 方案 | 说明 | repo 现状 | 误差水平与前提 |
|---|---|---|---|
| 机械测量/工装标定 | 卡尺/倾角仪/激光测距测传感器相对 base_link 的 x/y/z + rpy,填入矩阵 | sensor_extrinsics.yaml 声称为 camera 的方法（"manual_measurement",但文件未消费）;真实数据:只有 perception_runtime.yaml 的**假定矩阵**——无测量记录 | ±mm 级/±0.1-1° 量级;取决于工装与人员;成本低、可重复 |
| TF static transform | 把标定结果发布为 static_transform_publisher（sensor_frame→base_link）,bridge 用 use_tf=true 消费 | **无**——launch 无任何 static_transform_publisher;runbook 建议 "静态 TF 节点替换 camera_to_world_static"（docs/real_robot_execution_plan.md:172） | 与矩阵等价;额外收益 = frame 可观测、可被其他节点复用;需要 ROS TF 树（base_link 已存在） |
| 外部标定工具 | 激光跟踪仪/测量塔/全场测量;或 MATLAB/OpenCV 重建 | **无工具、无数据** | 最好(0.1mm/0.01°),成本高 |
| hand-eye / target-based (Apriltag / 球靶标定板;LiDAR 可 target-based) | 相机看 Apriltag/target 板;LiDAR 看球组/棋盘;求解 sensor↔base_link（相机固定、板在 robot 上或反之） | sensor_extrinsics.yaml 提及 "或 apriltag",但**无脚本、无板模型、无手眼数据** | 精度受视觉 pose 精度限制;需要机器人运动;实现成本中等 |

**目前 repository 中不存在的数据（Not yet specified / 缺失,不得假设）:**
1. 相机外参实测值（任何来源）— 仅假定矩阵(perception_runtime.yaml:25)与死文件占位(sensor_extrinsics.yaml);
2. LiDAR 外参 — 完全没有（单位阵占位;calibration_method_lidar=null);
3. 基座→环境系位姿 — 环境系本身在 repo 中**不存在**（URDF 根=base_link,无 world link,无 map/odom/workcell TF）;
4. 传感器真实安装几何（图纸/安装位）;
5. 传感器标定误差估计（error_estimate_mm/deg 全部 null）;
6. 传感器真实数据/回放 bag（data/ 仅 NURBS 轨迹文件）;
7. 时钟同步硬件方案（LiDAR PPS/CPU 时间同步 —— 现为软件时间戳配对, MAP §1.6 已记录）;
8. 标定脚本/工具（scripts/ 只有 calibrate_zero.py 关节零位标定,与传感外参无关）;
9. URDF 中的传感器 link（无 camera/lidar link 定义）。

---

## 附:05B 迁移如果发生,必须改动的接口清单（审计产物,05B 拍板后决定票面）

| # | 接口/文件 | 改动内容 | 触发条件 |
|---|---|---|---|
| 1 | `config/perception_runtime.yaml` | world_frame / camera_to_world_static / lidar_to_world_static / use_tf | 任何方案 B 路径 |
| 2 | `portable_oscbf/config/obstacle_params.yaml` | 注释/DEFAULT world_frame 同源 | 方案 B |
| 3 | `perception_bridge.py:_sensor_to_world` | TF 查询/静态矩阵语义（或取消透传 s2w 到 engine） | 方案 B + 动态 |
| 4 | `oscbf_controller.py`（或新适配层） | obs_pos/obs_vel 从感知系→FK 系(或反向),每周期/每消息 | 方案 B 一切形态（除非 FK 系也改成环境系,不现实） |
| 5 | `jax_control_facade.py` | 变换接入点 / frame 参数（可选,推荐保持 kernel frame-agnostic） | 方案 B |
| 6 | `perception_demo.py` | ESDF origin/采样系 | 方案 B |
| 7 | `launch/mujoco_transition_final.launch.py` | TF 节点/环境系参数 | 方案 B |
| 8 | `tests/test_perception_pipeline.py:44`、`tests/test_perception_bridge_demo.py:94` | 帧断言/占位注释 | 方案 B |
| 9 | spec 文档 | 若选 A,修订「world_frame 必须固定环境系」或登记偏差 | 方案 A |
| 10 | MAP §3 讨论项 D | 05B 定案后回填 | 两者 |
