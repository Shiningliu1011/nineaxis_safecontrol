# Journal - Lsn (Part 1)

> AI development session journal
> Started: 2026-09-21

---


## Session 1: 固定 OSCBF 运行依赖并清理未接线 QP 模块
<!-- trellis-session: v=2 fp=94a31a1f002b3937 -->

**Date**: 2026-09-23
**Task**: 固定 OSCBF 运行依赖并清理未接线 QP 模块
**Branch**: `main`

### Summary

完成 I5：固定 cbfpy 0.0.1、jax 和 jaxlib 0.6.2、qpax 0.1.4，删除未接线的 warm-start 模块与兼容分支，补充依赖接口测试及 Trellis code-spec；完整测试通过并推送 main。

### Git Commits

| Hash | Message |
|------|---------|
| `5c4c59b` | refactor: pin OSCBF dependencies and remove unused QP modules |

### Status

[OK] **Completed**


## Session 2: 恢复主机环境并验证紧凑记录
<!-- trellis-session: v=2 fp=bb0ff615f5513ed7 -->

**Date**: 2026-10-20
**Task**: 恢复主机环境并验证紧凑记录
**Branch**: `codex/static-obstacle-experiments`

### Summary

完成主机恢复、skills 与依赖及字体安装、紧凑存储真实验证、测试产物隔离和接手记录；原始资料逐文件校验一致，正式采集继续停止。

### Main Changes

- 主项目位于 /home/lsn/robot/robot_safecontrol，工作分支 codex/static-obstacle-experiments；环境入口 .scratch/host-recovery/env.sh。
- MessagePack float64 与 zlib 字典记录、旧 JSON 兼容、独立转换、按需数组、重放和图表来源校验。
- 迁移来源108份文件保存在 output/host-recovery-migration-evidence/；14,284个原始资料文件共23,143,586,250字节全部匹配SHA-256，清理重复迁移内容。

### Git Commits

| Hash | Message |
|------|---------|
| `e0d5ba2826c9ecdd14b6cbe189931dffa9fb9bab` | feat(storage): 完成紧凑记录与主机恢复验证 |

### Testing

- [OK] 三个ROS package构建通过，完整回归主包678 passed/1 skipped、内核486 passed/34 skipped、CTest4 passed，快速检查69 passed。
- [OK] S0/S2/S4/S5共331,573条真实记录逐字段及数值位模式一致，独立重算一致，8次SIGKILL恢复通过；容量减少76.87%–85.83%。
- [OK] 完整S0 sim通过，1,719个周期最大70.513 ms；本机空间约80.99 GB，1,080次样本外推53.26–65.10 GB，容量上界尚未确认。
- [OK] 来源检查25项、ROS产物隔离14项、内核10项通过；严格线程检查1项、空目录M8/感知8项通过；六类图表身份和完整性通过。
- [OK] 规范与质量、需求与验收两方面独立审查通过；详情见当前任务 research/review.md。

### Status

[OK] **Completed**

### Next Steps

- 继续限定sim；完成S4暂停/构型调整/恢复、S5当前参数复核及20组方法/周期测量，确认正式容量上界后再评估正式采集。
- 跨设备开始工作获取远程更新；原始实验及标定数据按接手记录单独传输并核验，任务保持in_progress。
