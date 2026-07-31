# AEB-RRT* 评估状态

## 阶段进度

| 阶段 | 状态 | 完成时间 | 备注 |
|------|------|----------|------|
| A: 仓库侦察 | ✅ 完成 | 2026-07-31 | 见 reconnaissance.md |
| B: 冻结基线 | ✅ 完成 | 2026-07-31 | 分支 feature/aeb-rrtstar-evaluation |
| C: 算法规格 | ✅ 完成 | 2026-07-31 | 见 benchmark_report.md 工程适配差异 |
| D: 实现 | ✅ 完成 | 2026-07-31 | AEBRRTstar (ompl.base.Planner) |
| E: 测试 | ✅ 完成 | 2026-07-31 | 16/16 纯逻辑测试通过，集成测试功能性通过 |
| F: 基准 | ✅ 完成 | 2026-07-31 | 3 场景 × 4 规划器，见 benchmark_report.md |
| G: 决策 | ✅ 完成 | 2026-07-31 | 推荐替换但先灰度 |

## 基线信息

- Git commit: `fc36d3f`
- 分支: `feature/aeb-rrtstar-evaluation`
- OMPL Python 版本: 2.0.1
- Python: 3.10
- OS: Ubuntu 22.04 (Linux 6.8.0-136-generic)

## 新增文件

```
src/aeb_rrtstar/__init__.py
src/aeb_rrtstar/aeb_rrtstar_planner.py      # AEB-RRT* ompl.base.Planner 实现
src/aeb_rrtstar/collision_checker.py         # OMPL StateValidityChecker + MotionValidator
src/aeb_rrtstar/robot_model.py              # 9-DOF 关节限制 + 简化 FK
src/aeb_rrtstar/scenarios.py                # 8 个固定测试场景
src/aeb_rrtstar/benchmark_runner.py         # Python 批量基准框架
src/aeb_rrtstar/single_run.py               # 单次运行（子进程隔离）
src/aeb_rrtstar/test_aeb_rrtstar.py         # 单元测试 (16 项)
benchmarks/aeb_rrtstar/raw_runs.csv         # 基准原始数据
benchmarks/aeb_rrtstar/summary.csv          # 汇总统计
benchmarks/aeb_rrtstar/environment.json     # 环境快照
docs/planning/aeb_rrtstar/reconnaissance.md # 仓库侦察报告
docs/planning/aeb_rrtstar/status.md         # 本文件
docs/planning/aeb_rrtstar/benchmark_report.md # 基准报告和决策
```

## 不修改的文件

- 所有现有 ROS 2 节点、配置和启动文件
- MoveIt 2 配置
- URDF 模型
- 当前默认规划器 (RRTConnectkConfigDefault)

## 最终决策

**推荐替换，但先灰度并保留回退**

详见 benchmark_report.md。
