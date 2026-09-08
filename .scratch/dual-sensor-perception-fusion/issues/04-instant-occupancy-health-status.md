# 04 — 即时占据安全通道 + 感知健康状态

**What to build:** 在融合定时器中新增两个发布通道。`/perception/instant_occupancy`：
当前帧全部占据点（PointCloud2），确保新障碍物在 0.5s 静态确认窗口内能被 OSCBF
看到。`/perception/status`：10 元素 Float32MultiArray，区分 alive（传感器在线）
和 used（本次融合采用），perception_valid = sources_used and fusion_age < timeout。

**Blocked by:** 03（融合定时器）

**Status:** ready-for-agent

- [ ] `/perception/instant_occupancy` 每帧发布当前全部占据点（PointCloud2 xyz）
- [ ] `/perception/status` 发布 10 元素 Float32MultiArray
- [ ] status[0-1] = camera_alive / lidar_alive（buffer 有新鲜数据）
- [ ] status[2-3] = camera_age / lidar_age（秒，-1=无数据）
- [ ] status[4-5] = camera_used / lidar_used（本次融合实际采用）
- [ ] status[6-7] = fusion_stamp / fusion_age
- [ ] status[8] = source_count（被采用传感器数量）
- [ ] status[9] = perception_valid（bool→float）
- [ ] perception_valid = (camera_used or lidar_used) and fusion_age < timeout
- [ ] 无传感器数据时 perception_valid = 0（不会误报 valid=1）
- [ ] alive 与 used 语义独立：alive=True 不保证 used=True
