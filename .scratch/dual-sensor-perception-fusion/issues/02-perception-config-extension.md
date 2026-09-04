# 02 — 感知配置扩展：LiDAR + 融合 + 三层字段

**What to build:** 扩展 `PointCloudCollisionConfig` 数据类和 YAML 加载逻辑，新增 LiDAR
topic/frame、源体素尺寸（camera/lidar 各一）、融合体素尺寸、跨传感器时间差阈值、
各传感器最大延迟、占据超时、静态确认时长、感知有效超时等字段。所有新字段有合理
默认值，旧 YAML 无需修改即可加载。

**Blocked by:** None — 可立即开始。

**Status:** ready-for-agent

- [ ] `PointCloudCollisionConfig` 新增字段：`source_topic_lidar`, `input_frame_lidar`,
      `source_voxel_camera_m`, `source_voxel_lidar_m`, `fusion_voxel_m`,
      `max_inter_sensor_dt_s`, `camera_max_age_s`, `lidar_max_age_s`,
      `occupancy_timeout_s`, `static_confirm_s`, `perception_timeout_s`
- [ ] `source_topic_lidar` 默认空字符串（不订阅 LiDAR → 向后兼容）
- [ ] `source_voxel_camera_m` 默认 0.02，`source_voxel_lidar_m` 默认 0.03
- [ ] `fusion_voxel_m` 默认 0.03
- [ ] `static_confirm_s` 默认 0.5，`occupancy_timeout_s` 默认 0.3
- [ ] `load_point_cloud_collision()` 从 YAML 正确读取所有新字段
- [ ] 旧 YAML（无新字段）加载不报错、行为不变
- [ ] `obstacle_params.yaml` 新增 `point_cloud_collision` 段的 LiDAR + 融合参数
- [ ] `config/perception_runtime.yaml` 新增 ROS 参数覆盖
