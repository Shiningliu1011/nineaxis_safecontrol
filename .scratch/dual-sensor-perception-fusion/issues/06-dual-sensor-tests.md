# 06 — 双传感器感知融合测试（6 类）

**What to build:** 编写覆盖规格中全部 6 类测试场景的测试用例，验证双传感器融合的
正确性、向后兼容性和降级行为。所有测试在无硬件环境下运行（合成点云）。

**Blocked by:** 04（instant_occupancy + status）、05（YAML + launch）

**Status:** ready-for-agent

- [ ] 测试 1：单 Camera 向后兼容（source_topic_lidar 为空 → 行为与原版一致）
- [ ] 测试 2：单 LiDAR 降级（Camera 缓冲为空 → 仅用 LiDAR 维持 ESDF + tracks）
- [ ] 测试 3：双源静态重叠（同一箱子不生成双层点云/双 track）
- [ ] 测试 4：时间错位（Camera 延迟 > max_inter_sensor_dt → 只用 LiDAR；
      Camera 恢复后自动恢复双源）
- [ ] 测试 5：传感器掉线（单路停止 → status 报对应 alive=0/used=0；
      两路停止 → perception_valid=0）
- [ ] 测试 6：动态障碍物 track 速度连续性（LiDAR/Camera 交替观测 → velocity
      不跳变）
- [ ] OccupancyTracker 单元测试：连续占据升格、中断重计时、超时清理、
      prev_occupied 连续性
- [ ] `bash run_all_tests.sh` 全部通过
