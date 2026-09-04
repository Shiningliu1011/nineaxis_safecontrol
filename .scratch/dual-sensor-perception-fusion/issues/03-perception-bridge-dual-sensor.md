# 03 — 感知桥接节点双传感器重构

**What to build:** 重构 `perception_bridge` 节点为双传感器融合架构。传感器回调
（ReentrantCallbackGroup）只做解码→TF→ROI→源体素→入缓冲（deque + Lock）。
20Hz 融合定时器（MutuallyExclusiveCallbackGroup）执行时间戳配对、自体过滤、
融合体素降采样、三层分类、ESDF 构建、动态聚类，发布到现有话题（esdf、tracks、
cloud_world、collision_object）。

**Blocked by:** 01（OccupancyTracker 三层接口）、02（配置字段）

**Status:** ready-for-agent

- [ ] 订阅 `/livox/lidar`（PointCloud2）和 `/camera/depth_registered/points`
      （PointCloud2），各自 callback_group = ReentrantCallbackGroup
- [ ] 传感器回调：decode → TF(stamp) → ROI crop → source_voxel → 入 deque（加锁）
- [ ] LiDAR 缓冲 deque(maxlen=3)，Camera 缓冲 deque(maxlen=6)
- [ ] 融合定时器 20Hz，callback_group = MutuallyExclusiveCallbackGroup
- [ ] 时间戳配对：以 LiDAR 帧为参考，Camera 选最近 timestamp
- [ ] 新鲜度检查：camera_max_age / lidar_max_age
- [ ] 跨传感器 dt > max_inter_sensor_dt → 只用更新的那路
- [ ] 重复帧保护：stamp 组合与上次相同则跳过
- [ ] fusion_stamp = max(t_lidar, t_camera)，不用 now
- [ ] 自体过滤：各用各的 sensor stamp 对应的关节角
- [ ] 合并 + fusion_voxel 降采样
- [ ] 三层分类 → static → ESDF → 发布 /perception/esdf
- [ ] 三层分类 → unconfirmed → 聚类 → 发布 /perception/tracks
- [ ] 发布 /perception/cloud_world 和 /collision_object
- [ ] source_topic_lidar 为空时不订阅 LiDAR，行为与原版一致
- [ ] world_frame 使用固定环境坐标系（非 base_link）
