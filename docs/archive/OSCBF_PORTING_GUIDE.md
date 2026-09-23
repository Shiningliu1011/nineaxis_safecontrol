# OSCBF 框架移植指南

> 历史设计资料。当前接口、参数和验收范围以现有源码、测试及[文档总目录](../README.md)中的现行说明为准。

> **读者对象**：从零编写 OSCBF 安全控制的新项目。
> **用途**：说明目标结构、模块接口、参数和逐项验收要求。
> **生成日期**：2026-08-06

目标是九轴机械臂的 JAX OSCBF 控制器，参考本仓库 `portable_oscbf/` 与 OSCBF 源码。各章节描述目标设计；当前完成情况需结合源码和测试核对。

## 章节

1. [路线总览](oscbf-porting/01-overview.md)
2. [技术选择](oscbf-porting/02-decisions.md)
3. [控制管线](oscbf-porting/03-pipeline.md)
4. [模块与接口](oscbf-porting/04-modules.md)
5. [数学模型](oscbf-porting/05-math.md)
6. [关键参数](oscbf-porting/06-parameters.md)
7. [碰撞模型](oscbf-porting/07-collision.md)
8. [配置体系](oscbf-porting/08-configuration.md)
9. [实施步骤](oscbf-porting/09-steps.md)
10. [验证标准](oscbf-porting/10-validation.md)
11. [相关经验](oscbf-porting/11-lessons.md)
12. [目标文件清单](oscbf-porting/12-files.md)
13. [依赖清单](oscbf-porting/13-dependencies.md)
