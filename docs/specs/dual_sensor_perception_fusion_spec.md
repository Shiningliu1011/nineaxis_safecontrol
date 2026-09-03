# 规格：双传感器感知融合（Dual-Sensor Perception Fusion）

状态：draft（issue tracker 未配置，尚未发布）
来源：2026-09-03 方案评审（4 轮迭代，6 项修正后通过）
领域词汇遵循 `CONTEXT.md` 词汇表。

## Problem Statement

项目当前的感知桥接节点（`perception_bridge`）只订阅单个 Orbbec Gemini 335L
深度相机的点云，经预处理后输出静态 ESDF 距离场和动态障碍物 track 给 OSCBF
安全控制器。单一相机存在三个结构性问题：

1. **近场盲区**：深度相机在 <0.3m 距离内丢失深度数据，而机械臂工作空间
   正好覆盖这个范围。控制器在最近距离反而看不到障碍物。
2. **视野局限**：相机 FOV 有限，无法覆盖机械臂周围的全部避障区域。
   障碍物从侧面或后方进入时，感知层完全失明。
3. **单点故障**：相机掉线 → 感知层完全失效 → OSCBF 退化为无感知模式，
   失去所有避障能力。

用户引入 Livox Mid-360S 激光雷达作为主传感器（全域覆盖、10Hz），
保留 Orbbec Gemini 335L 深度相机作为补充（近场/盲区、30Hz），
需要把两路点云融合为统一的感知输出，同时满足 OSCBF 安全控制器对
ESDF、动态 track 和健康状态的需求。

## Solution

重构感知桥接节点为双传感器融合架构：Short History Buffer + Timestamp Matching
+ Fusion Timer。每路传感器的回调只做解码→坐标变换→ROI 裁剪→降采样→入缓冲；
20Hz 融合定时器在互斥回调组中执行时间戳配对、自体过滤、体素融合、三层分类、
ESDF 构建、动态聚类与发布。新增即时占据安全通道（`/perception/instant_occupancy`）
和感知健康状态话题（`/perception/status`），确保新障碍物在静态 ESDF 确认前
（0.5s 窗口内）仍能被 OSCBF 看到。

将占据跟踪器从帧计数泄漏计数器重构为时间戳驱动的三层模型（instant / unconfirmed
/ static），用 `prev_occupied` 连续性检测确保占据中断后重新计时。

## User Stories

1. 作为感知开发者，我想要 Livox Mid-360S 激光雷达点云经 TF 变换、ROI 裁剪、
   体素降采样后进入短缓冲队列，以便融合定时器能按时间戳选取最近帧参与融合。
2. 作为感知开发者，我想要 Orbbec Gemini 335L 深度相机点云以同样的预处理流程
   进入独立的短缓冲队列，以便两路数据互不干扰、各自按频率写入。
3. 作为感知开发者，我想要融合定时器以 LiDAR 帧时间戳为参考，在 Camera 缓冲中
   选最近时间戳的帧进行配对，以便两路数据在时间维度上对齐而非各自取最新。
4. 作为感知开发者，我想要系统在两路传感器时间戳差超过阈值时只采用更新的那路，
   以便避免用严重滞后的传感器数据污染融合结果。
5. 作为感知开发者，我想要系统跳过与上次融合完全相同的 stamp 组合，以便避免
   重复处理相同数据浪费计算资源。
6. 作为感知开发者，我想要每路传感器的自体过滤使用该传感器时间戳对应的关节角，
   以便运动中的机械臂不会因关节角不同步而误过滤或漏过滤点云。
7. 作为感知开发者，我想要两路点云经各自源体素降采样后合并为统一分辨率的融合
   体素，以便下游只有一个点云流进入分类和 ESDF 构建。
8. 作为感知开发者，我想要占据跟踪器用时间戳（而非帧计数）判断体素持续占据时长，
   以便 Camera 30Hz 和 LiDAR 10Hz 的频率差异不影响静态确认的语义。
9. 作为感知开发者，我想要占据跟踪器检测体素占据中断并重新计时（prev_occupied
   连续性），以便短暂消失再出现的体素不会被误判为"持续静态"。
10. 作为感知开发者，我想要三层感知输出：instant（当前帧全部占据）、unconfirmed
    （占据但未达静态阈值）、static（持续 ≥0.5s 的占据），以便下游根据安全等级
    选择不同策略。
11. 作为感知开发者，我想要 static 层点云构建 ESDF 距离场并发布 D(x) 和 ∇D(x)，
    以便 OSCBF 的 ESDF 约束使用稳定的环境距离信息。
12. 作为感知开发者，我想要 unconfirmed 层点云经连通域聚类和跨帧关联后输出动态
    track（位置、速度、尺寸），以便 OSCBF 的动态障碍物 CBF 约束使用运动目标信息。
13. 作为感知开发者，我想要 instant 层点云作为 `/perception/instant_occupancy` 发布，
    以便新障碍物在 0.5s 静态确认窗口内就能被 OSCBF 看到。
14. 作为安全集成者，我想要 `/perception/status` 话题区分 alive（传感器在线）
    和 used（本次融合实际采用），以便安全看门狗能精确判断感知降级程度。
15. 作为安全集成者，我想要 perception_valid 仅在有传感器被采用且融合结果超时内
    时为 true，以便无传感器数据时不会误报感知有效。
16. 作为安全集成者，我想要 `/perception/status` 包含 camera_age、lidar_age、
    fusion_age 等延迟指标，以便安全看门狗能基于延迟判断数据新鲜度。
17. 作为部署者，我想要 `source_topic_lidar` 参数默认为空（不订阅 LiDAR），
    以便单 Camera 场景下行为与原版完全一致，无需修改任何配置。
18. 作为部署者，我想要 LiDAR 和 Camera 各有独立的源体素尺寸参数，
    以便根据各自点云密度选择合适的降采样分辨率。
19. 作为部署者，我想要 world_frame 指向固定环境坐标系（而非 base_link），
    以便感知结果在机械臂升降运动中保持空间稳定。
20. 作为部署者，我想要 launch 文件新增 LiDAR 相关参数（topic、frame、
    extrinsics），以便双传感器部署只需修改 launch 参数和外参配置。
21. 作为感知开发者，我想要传感器回调使用 ReentrantCallbackGroup 而融合定时器
    使用 MutuallyExclusiveCallbackGroup，以便传感器回调可并行但融合处理串行
    且不与回调竞争共享状态。
22. 作为感知开发者，我想要缓冲队列加锁保护（deque + Lock），以便传感器回调
    写入和融合定时器读取之间不会发生数据竞争。
23. 作为感知开发者，我想要融合时间戳取参与融合的传感器时间戳的最大值（而非
    当前时间），以便下游 ESDF 和 track 的时间语义与实际数据一致。
24. 作为部署者，我想要旧 `StaticOccupancyTracker` 类名保留为兼容别名，
    以便已有代码和测试不需要立即修改。

## Implementation Decisions

### 模块变更

- **占据跟踪器**（`portable_oscbf/work/static_occupancy.py`）：重构为
  `OccupancyTracker`，从帧计数泄漏计数器改为时间戳驱动的三层模型。
  旧 `StaticOccupancyTracker` 保留为兼容别名。

- **感知配置**（`portable_oscbf/work/perception_config.py`）：
  `PointCloudCollisionConfig` 新增 LiDAR topic/frame、源体素尺寸、融合体素尺寸、
  跨传感器时间差阈值、传感器最大延迟、占据超时、静态确认时长等字段。

- **感知桥接节点**（`src/robot_safecontrol_moveit/perception_bridge.py`）：
  主要重构目标。新增双订阅、双缓冲、融合定时器、即时占据发布、健康状态发布。
  保留原有 ESDF、track、cloud_world 发布接口不变。

- **配置文件**（`portable_oscbf/config/obstacle_params.yaml`、
  `config/perception_runtime.yaml`、`config/sensor_extrinsics.yaml`）：
  新增 LiDAR 相关参数和融合参数。所有新参数有合理默认值，旧配置无需修改即可工作。

- **Launch 文件**（`launch/mujoco_transition_final.launch.py`）：
  新增 LiDAR 相关 launch arguments 并传递给 perception_bridge 节点。

### 接口契约

- **输入**：
  - `/livox/lidar`（sensor_msgs/PointCloud2，10Hz，Livox 坐标系）
  - `/camera/depth_registered/points`（sensor_msgs/PointCloud2，30Hz，Camera 坐标系）
  - `/mujoco_joint_states`（sensor_msgs/JointState，用于自体过滤的关节角）

- **输出**（与现有一致 + 新增）：
  - `/perception/esdf`（Float32MultiArray，ESDF 距离值数组）
  - `/perception/esdf_meta`（Float32MultiArray，ESDF 网格元信息）
  - `/perception/tracks`（Float32MultiArray，动态障碍物 track 数据）
  - `/perception/cloud_world`（PointCloud2，融合后世界系点云）
  - `/collision_object`（moveit_msgs/CollisionObject，用于 MoveIt 场景）
  - `/perception/instant_occupancy`（PointCloud2，**新增**，当前帧全部占据点）
  - `/perception/status`（Float32MultiArray，**新增**，10 元素健康状态）

### 占据跟踪器三层模型

`OccupancyTracker.update(points_world, stamp_s)` 返回三层：

- **instant_points**：当前帧全部点（即时安全通道）
- **unconfirmed_points**：占据但持续时长 < `static_confirm_s` 的体素内的点（动态聚类输入）
- **static_points**：持续占据 ≥ `static_confirm_s` 的体素中心（ESDF 构建输入）

内部状态：
- `_last_seen`：每体素最后被占据的时间戳（`-inf` = 从未占据）
- `_first_seen`：每体素连续占据起始时间戳（`inf` = 未在连续占据中）
- `_prev_occupied`：上一帧占据掩码（连续性检测）

连续性规则：`newly_occupied = occupied & ~prev_occupied` 时才记录 `first_seen`。
占据中断（上帧占据、本帧未占据）→ `first_seen` 重置为 `inf`，下次重新计时。
超时清理：`(stamp_s - last_seen) > occupancy_timeout_s` → `first_seen` 归 `inf`。

### 融合定时器数据流

```
fusion_timer (20Hz)
  ① 快照缓冲（加锁复制 deque）
  ② alive 检测（buffer 最新帧 age < max_age）
  ③ 时间戳配对（LiDAR 为参考，Camera 选最近帧）
  ④ 新鲜度检查（各自 max_age）
  ⑤ 跨传感器 dt 检查（> max_inter_sensor_dt → 只用更新的）
  ⑥ 重复帧检查（stamp 组合不变 → 跳过）
  ⑦ 自体过滤（各用各的 sensor stamp）
  ⑧ 合并 + 融合体素降采样
  ⑨ 三层分类（OccupancyTracker）
  ⑩ ESDF 构建（static 层）+ 动态聚类（unconfirmed 层）
  ⑪ 发布全部话题
```

### 感知健康状态

`/perception/status` 为 10 元素 Float32MultiArray：

| 索引 | 含义 | 类型 |
|------|------|------|
| 0 | camera_alive | bool→float |
| 1 | lidar_alive | bool→float |
| 2 | camera_age | 秒，-1=无数据 |
| 3 | lidar_age | 秒，-1=无数据 |
| 4 | camera_used | bool→float |
| 5 | lidar_used | bool→float |
| 6 | fusion_stamp | 秒 |
| 7 | fusion_age | 秒 |
| 8 | source_count | 被采用传感器数量 |
| 9 | perception_valid | bool→float |

`perception_valid = (camera_used or lidar_used) and fusion_age < perception_timeout`。
`alive` 仅表示传感器在线（buffer 中有新鲜数据），`used` 表示本次融合实际采用。

### 线程模型

- 传感器回调：`ReentrantCallbackGroup`，可并行执行，只写各自缓冲（加锁）
- 融合定时器：`MutuallyExclusiveCallbackGroup`，与传感器回调串行化，
  OccupancyTracker / TrackManager / ESDF 只在融合回调中访问

### 坐标系与 TF

- `world_frame`：固定环境坐标系（不是 base_link，base_link 随升降轴运动）
- 每路传感器有独立的静态外参（sensor_frame → world）
- TF 查询使用传感器自身的消息时间戳

### 体素尺寸层次

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `source_voxel_camera_m` | 0.02 | Camera 源体素，≤ fusion_voxel |
| `source_voxel_lidar_m` | 0.03 | LiDAR 源体素，≤ fusion_voxel |
| `fusion_voxel_m` | 0.03 | 融合后统一体素 |

### 向后兼容

- `source_topic_lidar` 默认空 → 不订阅 LiDAR → 只用 Camera，行为与原版一致
- `static_confirm_s` 默认 0.5s，`occupancy_timeout_s` 默认 0.3s
- 旧 `StaticOccupancyTracker` 名称保留为兼容别名
- 单 Camera 场景下三层模型退化为原行为（instant ≈ 原 dynamic，static ≈ 原 static）

## Testing Decisions

### 测试原则

- 只测外部行为（输入 → 输出），不测内部实现细节
- 占据跟踪器和感知桥接节点分别有独立的单元测试
- 集成测试在 ROS 环境中验证话题发布和消息内容
- 所有测试可在无硬件环境下运行（使用合成点云）

### 测试类别

1. **单 Camera 向后兼容**：`source_topic_lidar` 为空时，行为与原版
   `StaticOccupancyTracker` 一致——静态点进入 ESDF，动态点进入聚类，
   无 `/perception/instant_occupancy` 发布异常。

2. **单 LiDAR 降级**：Camera 缓冲为空时，系统仅用 LiDAR 数据维持 ESDF
   和 tracks，`/perception/status` 报 `camera_alive=0, lidar_alive=1`。

3. **双源静态重叠**：LiDAR 和 Camera 同时观测到同一个静止箱子，
   融合后不生成双层点云，ESDF 中只有一层障碍物表面，不产生重复 track。

4. **时间错位降级**：Camera 延迟 > `max_inter_sensor_dt`，
   系统只用 LiDAR；Camera 回到正常延迟后自动恢复双源融合。

5. **传感器掉线状态**：单路传感器停止发布 → `/perception/status`
   对应 `alive=0, used=0`，另一路正常时 `perception_valid` 仍为 1。
   两路都停止 → `perception_valid=0`。

6. **动态障碍物 track 速度连续性**：LiDAR 和 Camera 交替观测同一个
   运动障碍物时，track 速度估计不因传感器切换产生跳变（跨帧关联
   使用全局最近邻匹配，不区分传感器来源）。

### 占据跟踪器单元测试

- 连续占据 ≥ `static_confirm_s` → 体素升格为 static
- 占据中断后重新出现 → 重新计时，不继承之前的持续时长
- 超过 `occupancy_timeout_s` 未出现 → 清除 first_seen/last_seen
- `prev_occupied` 连续性：同一帧内连续出现不会重复设置 first_seen

### 感知桥接集成测试

- 融合定时器在无数据时不崩溃，发 status 报 valid=0
- 重复 stamp 组合被跳过（不重复处理）
- 融合时间戳 = max(参与传感器的 stamp)，不是 now
- `/perception/instant_occupancy` 在有数据时每帧发布

## Out of Scope

- **JointState 历史缓冲**：第一版自体过滤使用 latest q，关节角插值
  作为 TODO 标记，不在本规格范围内。
- **点云语义分割**：不区分障碍物类型（人、箱子、工具），只区分
  静态/动态/即时。
- **多机器人感知融合**：本规格只覆盖单臂单场景。
- **OSCBF 控制器侧的 instant_occupancy 消费**：控制器如何使用
  instant_occupancy 话题不在本规格范围内，由控制内核规格定义。
- **LiDAR 硬件驱动配置**：Livox SDK2 的安装、配置和 ROS 驱动不在
  本规格范围内。

## Further Notes

- 本规格经过 4 轮方案评审迭代，修正了 `_first_seen` 初始化（`inf`
  不是 `-inf`）、`perception_valid` 逻辑（无传感器时不应为 true）、
  alive/used 语义区分、`prev_occupied` 连续性、instant_occupancy
  安全通道、自体过滤移到融合定时器等 6 项关键问题。
- 向后兼容是硬约束：单 Camera 场景下不得有任何行为变化。
- world_frame 必须是固定环境坐标系，不是 base_link。
