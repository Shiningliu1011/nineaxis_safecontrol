# 05 — YAML 配置 + Launch 参数 + 传感器外参

**What to build:** 更新所有配置文件和 launch 文件，使双传感器部署只需修改 launch
参数和外参配置。LiDAR 外参 YAML 占位，launch 文件新增 LiDAR 相关 arguments
并传递给 perception_bridge 节点。

**Blocked by:** 02（配置字段定义）

**Status:** ready-for-agent

- [ ] `obstacle_params.yaml` 新增 LiDAR source_topic、input_frame、
      源体素/融合体素尺寸、跨传感器时间差、传感器最大延迟、
      占据超时/确认时长、感知有效超时
- [ ] `config/perception_runtime.yaml` 新增 ROS 参数覆盖（LiDAR 相关）
- [ ] `config/sensor_extrinsics.yaml` 新增 LiDAR 外参占位（4×4 矩阵）
- [ ] launch 文件新增 `source_topic_lidar`、`input_frame_lidar` 等 arguments
- [ ] launch 文件将 LiDAR 参数传递给 perception_bridge 节点
- [ ] 默认参数下单 Camera 启动行为不变
