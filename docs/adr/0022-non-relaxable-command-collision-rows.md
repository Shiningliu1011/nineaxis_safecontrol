---
status: accepted
date: 2026-09-21
---

# 命令 QP 中的碰撞 CBF 行全部不可松弛

命令 QP 中全部 self collision 与 environment collision CBF 行不可使用 slack 放宽。
存在可行速度时正常求解；约束冲突、求解失败、梯度无效或结果包含非有限值时，候选命令
不能取得准入，系统进入保持与锁存处理。允许松弛的计算只能作为独立诊断，诊断结果没有
命令授权能力。

## Considered Options

曾考虑继续使用允许松弛的命令 QP，并在求解后检查每条 collision slack。该方案依赖
slack 单位、准入阈值和所有调用路径都正确执行检查。直接在命令 QP 中禁止碰撞松弛能够
让求解问题本身表达安全要求，因此采用该方法。

## Consequences

当前 `relax_cbf=True` 且 collision slack 仅用于诊断的命令路径不能继续使用。任务跟踪
目标可以让步，碰撞行不能让步；诊断求解器必须与命令输出接口隔离。
