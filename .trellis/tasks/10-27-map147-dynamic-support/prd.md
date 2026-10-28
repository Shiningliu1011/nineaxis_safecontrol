# 动态 tracking 与 reachable support tube

依据 #101、#109、ADR 0016/0039/0040，以及已确认 #139 的场景方案。

- occupancy 决定障碍存在性；tracking 仅补充已存在 support 的运动。
- 保留逐原观测的几何、时间、测量误差及来源身份；观测年龄范围计算一次。
- track 中断、过期与重新关联均保留 occupied；无有效 track 时使用 parameter artifact 的未跟踪范围。
- 当前与未来查询在各自 int64 ns 目标时间共用同一 support。barrier 时间项包含名义运动和半径增长，距离使用同一位置与范围。
- 合并后的 support 在完整有效时间范围包围全部来源；缺少范围、覆盖、身份或有效期时拒绝准入。
- 公开回放格式验证稳定关联、中断、恢复、接近、远离、横向运动、固定实体及异源合并。独立解析与数值差分验证导数及完整未来范围。
- 保持 OccupancySnapshot 和 CollisionPolicy 的字段。#105 的 batch/update 增量由集成人员保留。
- #113 完整运动证明尚未实现，certify 继续返回拒绝。物理设备参数与真实 LiDAR 验收由 #117/#122 提供。
