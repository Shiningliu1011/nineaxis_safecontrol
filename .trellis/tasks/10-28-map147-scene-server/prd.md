# collision_scene_server 与共享场景

依据为「[24] 实现 collision_scene_server 与双缓冲共享场景」、其六项已关闭原生前置、ADR 0037/0038/0041，以及已确认场景、共同时间和 Runtime 规则。实现现有九轴与 LiDAR 场景的独立发布进程。

## 行为与验收

- `collision_scene_server` 独占关节历史、mesh self-filter、射线占据、固定模型合并、tracking、support、覆盖、容量和场景健康，并成为场景身份与共享内存的唯一写入者。
- 固定布局双缓冲只写未启用 slot，完成精度、容量、来源、checksum 与完整场景检查后原子发布；启用 slot 保持不可变。
- ROS 只发布 revision、采集/准备时间、slot、布局身份与健康信息。每次启动生成新 epoch，revision 在当前 epoch 中单调递增。
- 读取者检查完整 header、来源、checksum 与期限；每个有效 revision 只生成一次本地 JAX device 副本。异常状态明确且不授权查询。
- 使用真实场景服务进程及真实共享内存，核验唯一写入、重启、revision 跳跃、checksum 损坏、slot 超时、读取期间切换和单次 device 复制。
- 使用实际 mesh、真实公开占据/tracking/prepare 接口和独立进程；新增测试不使用 mock 或替代实现。保留现有硬件准入状态。
- 交付运行入口、配置、使用说明、隔离安装核验与实际测试证据。用户已授权将本次代码、测试与相关文档提交到 `main` 并推送到 `origin/main`。
