---
status: accepted
date: 2026-09-21
---

# 独立 collision_scene_server 负责碰撞场景

新增独立 ROS 进程 `collision_scene_server`，作为 CollisionScene、`scene_epoch`、
`scene_revision` 和双缓冲共享内存的唯一写入者。现有 `perception_bridge` 负责传感器消息
规范化、来源身份和时间信息检查，并向该进程提供保留逐点采集时间的 LiDAR 数据。

`collision_scene_server` 独占以下状态与处理：

- 关节状态时间历史和逐点姿态插值；
- robot mesh ray self-filter；
- occupied、free、unknown 射线占据更新；
- 固定环境模型合并；
- 可选运动跟踪与不确定性范围；
- support point、`rho_mm`、活动集合和容量管理；
- 覆盖、时间、identity、checksum 与场景健康检查；
- 不可变 `PreparedScene` host slot 的双缓冲发布。

控制器、规划器和轨迹验证器只读取已发布场景并构造本进程的 JAX device 引用，不能修改共享
slot，也不能产生各自的环境 revision。场景服务每次启动都生成新的 `scene_epoch`；场景身份由
`scene_epoch` 与该 epoch 内单调增加的 `scene_revision` 共同确定，防止进程重启后误用旧副本。

## Decision Basis

占据状态、tracking、support 合并和 revision 必须由单一所有者维护，才能让规划、执行与
OSCBF 使用同一环境事实。独立进程还把 LiDAR 处理与 100 Hz 控制运行资源和故障范围分开。

## Consequences

场景服务失联、epoch 改变或共享内存身份失效时，所有读取者立即撤销旧场景准入。新 epoch
完成当前状态与轨迹验证前保持 `REVISION_PENDING`。`perception_bridge` 不能发布已经完成
占据解释的第二套障碍物结果。
