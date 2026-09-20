# 架构审查与保行为整理（2026-09-14）

本次从近期变更、架构入口和实际调用链检查模块职责，实施有明确等价验证的去重。
基线为 `258b789`，结果对应本次工作区改动。用户已有的未跟踪 `AGENTS.md` 保留。
这不是全仓逐行审计，也不表示完成全量仿真或真机验收。

## 架构边界与处置

| 边界 | 代码证据 | 结论 |
| --- | --- | --- |
| 过渡编排与 ROS 副作用 | `transition_planning_server.py` → `TransitionPorts` → `transition_executor.py` | 保留服务器与纯逻辑执行器的分工，符合 ADR 0001；不恢复另一套 CLI 编排。 |
| ROS 控制与可移植内核 | `oscbf_controller.py` → `work/jax_control_facade.py` | 保留单向依赖；不让独立内核反向依赖 ROS 主包。 |
| 生产与离线配置 | `production_config.py` 的必填字段、来源记录、legacy 参数拒绝 | 两类配置不是可互换副本；不合并默认值或放松生产配置校验。 |
| 跟踪评价与持久化 | `tracking_contract.py`、`tracking_evaluator.py`、`tracking_report_writer.py` | 近期 OFF-15 已分离契约、评价和后台写入。保留 ADR 0008 的真实夹角、声明区间和逐项结论，不按文件长度拆分。 |
| 轨迹加载与几何 | `oscbf_trajectory.py`、`cylinder_geometry.py` → 查看器及 `task_target.py` | 存在真实计算副本，本次合并。 |
| 状态流传输 | `ros_conventions.py` 与查看器、控制器、被控对象中的 QoS 使用 | depth 5 与 depth 20 差异有运行意义，不能作为文字重复直接统一。 |
| 真机执行端 | 当前 AGENTS、ONBOARDING 中 sim/shadow/live containment 契约 | 不将多入口拒绝检查当作冗余；本次未修改硬件实现，也未做硬件能力审计。 |

上述路径除 `work/` 外均位于 `src/robot_safecontrol_moveit/`；控制内核位于
`portable_oscbf/work/`。现有目录分层已能表达节点、共享逻辑、控制内核和插件，
没有足够证据支持整仓搬目录或删除研究实现。ADR 0005 也要求在退役前具备比较验证，
而非仅凭本地实现与上游能力相似就删除。

## 按收益和风险排序的候选

1. **轨迹加载计算去重：已实施。** 两个公开加载函数此前分别维护 MAT 读取、
   mm→m、齐次变换、全量圆柱投影和抽样。现在由 `_load_calibrated_samples`
   持有共同算法，复用 `apply_trajectory_transform`；两个公开入口仅选择位置或
   位置与时间输出。维护者只需修改一处标定流程。保留函数签名、返回类型、
   完整路径拟合后再抽样的顺序，以及位置输入无需 `time_series` 的契约。
   通过真实轨迹基线对照和合成输入边界测试验证。
2. **圆柱轴心表达去重：已实施。** 投影、工具姿态和查看器分别展开拟合坐标到
   三维轴心的表达式。`CylinderFit.axis_point` 统一该表达式，各调用方仍选择
   轴向位置；查看器负责地面延伸与高度边距，几何算法负责路径范围中点。
   收益是拟合结果的坐标解释集中在其所属类型中，避免把显示规则混入轨迹几何。
   用倾斜轴测试、已有姿态测试和查看器新旧规格对照验证。
3. **内核与主包的圆柱算法共享：保留为后续候选，未实施。**
   `portable_oscbf/work/ik_data_loader.py` 的 `_fit_surface_axis_point`、
   `_snap_path_onto_cylinder` 与主包几何存在相似计算，但内核还负责参考姿态、
   弧长路径及重复采样处理。若继续整理，应由独立的数值模块提供共同几何能力，
   两侧保留数据适配，不能直接从内核 import ROS 主包。预期减少两侧几何漂移；
   需要先建立退化输入、显式轴心、完整姿态与采样契约的逐项对照，再决定归属。
4. **QoS 约定收敛：不纳入本次保行为修改。** 统一 topic 常量和统一队列深度是
   两件事。队列深度会影响突发状态处理与延迟；应以实体使用表和仿真时延测试
   决定是否调整，不能用文本去重替代行为评估。

本次两个实现项与现有 ADR 无冲突；后续共享内核几何必须维持可移植边界，
不能借整理架构改变 ADR 0008 的评价定义或硬件准入规则。

## 验证记录

- 修改前：`python3 -m pytest tests/test_task_target.py tests/test_trajectory_transform_unity.py -q`
  → **15 passed**。
- 修改后：上述两文件加 `tests/test_calibrated_path_loading.py` → **22 passed**。
  新增覆盖完整拟合后抽样、位置与原始时间同步、大步长、负步长既有空输出、
  无时间字段的位置输入，以及倾斜圆柱轴心。
- 加载 ROS 与项目环境后：`python3 -m pytest tests/test_transition_executor.py tests/test_transition_pipeline_flow.py tests/test_tracking_evaluator.py tests/test_tracking_report_writer.py -q`
  → **96 passed**。过渡管线测试使用替身，不执行真实 MoveIt 或硬件动作。
- `python3 -m pytest portable_oscbf/tests/test_trajectory_loading.py -q`
  → **5 passed**，覆盖内核轨迹加载与 facade 路径状态契约。
- 临时只读对照：通过 `git show HEAD:<模块路径>` 加载修改前的加载器，对真实
  `data/nurbs/ik_input.mat` 使用两种轴向和三种抽样组合；**6 组**位置及原始时间
  均通过 `assert_array_equal`，无容差放宽。
- 查看器只调用静态圆柱拟合方法，不创建节点或窗口：两种轴向 × 地面延伸开关，
  **4 组**新旧 `TrackingCylinderSpec` 全字段完全相等。初次对照命令覆盖了
  `PYTHONPATH` 导致找不到 rclpy；保留 ROS 路径后重跑通过。
- `git diff --check` 通过。

修改后相关测试合计 **123 passed**。没有运行 `run_all_tests.sh`、完整 launch、
MuJoCo 图形闭环或真实硬件；上述结论限于所列行为与数值对照，不作全功能绝对保证。
