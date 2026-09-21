---
status: accepted
date: 2026-09-21
---

# CollisionSafety 使用三个操作组成的深 Module Interface

`CollisionSafety` Module 的外部 seam 只提供三个 Interface 操作：

- `prepare_scene(scene, identities)`：验证 scene、geometry、kernel identity、容量、时间和
  覆盖，返回不可变 `PreparedScene` 或明确 status。
- `query(query_batch, prepared_scene, query_mode)`：统一处理单状态与批量状态，按固定 mode
  返回规划状态、OSCBF barrier 或毫米距离结果。
- `certify(segment_batch, prepared_scene)`：对规划边与轨迹前视区间执行自适应证明，返回
  `SegmentCertificateBatch`。

ellipsoid DCOL、point-scale、活动集合选择、毫米距离、运动上界、mask、单位转换和 solver
health 都属于 Module implementation。ROS 只提供消息 Adapter，不能绕过 Interface 调用
内部 kernel。

## Considered Options

曾考虑把各内部查询作为独立公开函数，让规划器、轨迹执行器和 OSCBF 自行组合。该形状会
让调用方共同承担顺序、单位、mask 和失败状态知识，降低 Module Depth 与 Locality，因此
采用较小 Interface。

## Consequences

调用方与测试都通过同一个 seam。内部文件可以按数学职责组织，但不能形成第二套外部
Interface。新增能力应优先扩展 query result 或固定 `query_mode`，只有无法由现有三个操作
表达时才增加 Interface 操作。
