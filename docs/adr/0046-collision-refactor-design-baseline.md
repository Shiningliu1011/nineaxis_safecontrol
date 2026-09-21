---
status: accepted
date: 2026-09-21
---

# CollisionSafety 重构设计依据已经确认

ADR 0010 至 ADR 0045 共同构成 CollisionSafety 重构的已接受设计依据。本轮决策问答结束，
后续设计和实施不能自行改变其中的几何语义、场景语义、Interface、安全状态、规划方法、
OSCBF 约束、恢复规则、切换证据或参数来源。

该设计依据描述目标系统。当前生产代码仍然使用现有 MoveIt/FCL、OBB 与球体路径，直到新模块
完成实现、分层验收和 ADR 0043 规定的单一切换。设计确认不授予生产代码修改、真实机器人
运动或生产切换权限。

下一项文档工作包括：

- 完整实施方案与模块依赖关系；
- 按可独立验证结果组织的任务划分；
- 分层验收规范与证据 manifest schema；
- `CollisionParameterArtifact` 测量和原型实验计划。

生产代码只能在用户明确要求开始实施后修改。

## Consequences

本目录中较早记录的 MoveIt/FCL 规划、OBB 自碰撞、球体环境约束或长期 fallback 选择只保留为
当前实现说明与历史研究材料，不能继续作为目标设计依据。若需要改变 accepted ADR，必须明确
指定对应 ADR 或主题并重新记录决定。
