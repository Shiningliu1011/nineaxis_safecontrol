# 过渡管线以 transition_planning_server 为唯一语义，删除 plan_transition CLI 路线

2026-08 重构（C1）：仓库曾并存两套等价的过渡编排实现——一次性 CLI 节点
`plan_transition`（750 行）与持久化服务器 `transition_planning_server`
（724 行），各自携带状态机与圆柱拟合/轨迹加载的知识副本。我们决定以
server 的语义为唯一规范（生产闭环 `run_demo.sh` 只走 server，且只有它有
测试覆盖），整体删除 CLI 节点、其 launch 与 config，把相位机抽成纯逻辑模块
`transition_executor`（无 ROS import，副作用经注入的 ports）。

**Considered Options**：保留 CLI 作为薄 adapter（被否：CLI 独有能力——任务轨迹
回放、dry-run、指定起始位姿——无任何调用方，其回放路径调用已删除的参数、
运行即 TypeError，属于腐烂代码）；以 CLI 为规范（被否：与生产路径相反）；
从零重写第三套（被否：丢弃已测试的交接/随机化语义）。

**Consequences**：丢失的能力仅限 CLI 独有且无人使用的特性（任务路径回放调用方、
dry-run、无关节流的起始位姿指定）；`make_task_trajectory` 随之删除（git 历史可
恢复）。旧文档与 `benchmarks/aeb_rrtstar/real_fcl/` 日志中仍会出现
`plan_transition` 字样，属历史记录。
