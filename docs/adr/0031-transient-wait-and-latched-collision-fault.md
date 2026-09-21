---
status: accepted
date: 2026-09-21
---

# 碰撞运行状态区分临时等待与故障锁存

`REVISION_PENDING`、前视区间正在验证和当前 revision 上正在重新规划属于碰撞临时等待，
期间输出保持命令；取得当前场景准入后可以自动继续。标定无效、LiDAR 超过停止年龄、必要
空间 unknown、`CAPACITY_OVERFLOW`、`CONSTRAINT_OVERFLOW`、DCOL solver health 无效、
collision QP 不可行或失败、mesh 自体过滤时间状态无效均进入碰撞故障锁存。

锁存后持续输出保持命令并保存唯一主原因。只有故障条件恢复、当前机器人状态重新验证、
场景有效、剩余轨迹重新取得准入，并收到人工确认后才能继续；人工确认不能替代任何健康
条件。

## Considered Options

曾考虑所有条件恢复后自动继续。传感器或求解状态反复变化时可能产生意外恢复运动，因此
只允许日常 revision 更新自动恢复，破坏安全依据的状态采用锁存。

## Consequences

状态机必须区分等待原因与锁存原因，诊断包含首次时间、最近时间、scene identity 和恢复
条件。新的 collision fault 需要明确归入其中一类，不能使用无原因的通用失败状态。
