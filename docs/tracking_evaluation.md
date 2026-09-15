# 跟踪评价：离线指标、声明区间与证据来源

OFF-15 的评价器是纯 Python / NumPy 模块，不发布控制命令。主入口为 `robot_safecontrol_moveit.tracking_evaluator.TrackingEvaluator`；声明范围、证据身份和阈值类型位于同包的 `tracking_contract`。决定及历史阈值来源见 [ADR 0008](adr/0008-tracking-evaluation-scope-and-angle.md)和 [OFF-15 交接](planning/oscbf-reuse/handoffs/44-offline-tracking-evaluation.md)。

## 结果如何读

- `task_verdict`：声明区间的任务质量，使用已接受的 cross-track、工具轴、QP 比例要求，并检查范围完成、数据身份、时间及故障事件。
- `online_verdict`：所记录的准入与重叠判据。任一拒绝/重叠/关键故障会保留失败；缺少准入或重叠记录时为证据不足，不能从 `qp_ok` 推断。
- `verdict`：上述两项与全部所需阈值的合并结论。所有必需项通过才为 `pass`，有失败为 `fail`，其余为 `insufficient_evidence`。结论只覆盖报告声明的模型、数据、场景与边界。
- `completed`：显式正常结束、声明区间覆盖完整且主指标与样本时间有效；它不独立表示验收通过。`full_path_covered` 表示整条路径的样本进度覆盖；`full_path_verified` 还要求全部所需判据通过。

门槽（`qp_ok`、`admission_ok`、`overlap`）只接受布尔值。收到非布尔值时该样本按未测计入（仍是证据不足，不会变成拒绝），并在报告的 `gate_format_counts` 里按槽位计数、在 `issues` 中逐槽列明，例如 `admission_ok=3`。这条计数是纯加法，不参与任何 verdict 的计算；它存在的唯一目的是让「一次真实测量因为格式不对而悄悄丢失」变得可见——这是本评价器里唯一原本无从察觉的故障模式。

默认沿用 #14 的任务要求：cross-track RMS ≤ 0.15 mm、p95 ≤ 0.5 mm、max ≤ 2 mm；真实工具轴角 RMS ≤ 0.05°、max ≤ 0.5°；QP 成功率 ≥ 99.9%。本票没有修改这些任务数值。净空非负的既有下界作为 accepted 判据 `obstacle_clearance_nonnegative` 单独保留，负净空直接判失败；非负值仍不能替代独立重叠判据。历史 30 mm 净空和 1% 超期率仍为 provisional，因此即使解析实验的任务指标通过，默认的全部判据结论仍可能是证据不足。

阈值包含 `metric/limit/comparison/unit/category/status/source`。任务、数值、安全和时间判据分别列出；未定值可用 `limit=None, status="unknown"`。通过构造参数提供阈值时按指标覆盖默认值或追加新指标，不能通过只传一项来隐去其余默认必需项。accepted 需要明确值与来源；该标记由调用者提供，不能替代对来源及适用工况的审查。

## 指标定义与输入

每个 `update()` 对应一个测量边界的样本，调用方提供同一状态/参考时刻的量。需要多个边界时，各自建立评价器，通过相同 `run_id` 关联；显式标有不同 `measurement_boundary` 或 `evidence_kind` 的样本不能混入同一评价器。

控制节点走的是类型化的那个入口：`update_from_step_record(record, ...)`。它在记录的固定槽位清单（`STEP_RECORD_SLOTS`）里按名字取值，缺哪个槽就保持未测，不再靠 `getattr` 去猜对象形状；老的字典型入口 `update(dict)` 原样保留，服务历史脚本与「扔字典进去」的调用方。`step_latency_ms`（调用方实测的一步耗时，含记录转换为宿主数组的时间）和 `projection_before_m`（执行前一步时的实际投影）不是内核输出，由调用方通过 `update()` / `update_from_step_record()` 的参数单独提供；两者为 `None` 时沿用样本里已有的同名值。

| 指标 | 输入与定义 | 用途 |
|---|---|---|
| `cross_track_m` | `ee_pos - reference_position_m` 剔除沿归一化 `reference_tangent` 的分量后取范数；`reference_at_endpoint=True` 时使用全 3D 范数。也可显式输入已按同一口径计算的 `measured_cross_track_error_m`。 | 主位置验收 |
| `tool_axis_error_rad` | `ee_rot` 和 `reference_rotation` 的工具 X 轴夹角：`atan2(norm(cross), dot)`；也可显式输入真实夹角。 | 主姿态验收；0–π，roll 自由 |
| `pos_error_m` | 同一状态/参考的 3D 距离；仅有旧 `err_6d` 时保留其位置范数 | 诊断，不作为主验收 |
| `online_cross_track_m` | 旧 `cross_track_error_m`，路径调度器自己的周期初状态/参考横偏 | 在线调度诊断，与执行一步后的误差分开 |
| `legacy_orientation_error` | 旧 `err_6d[3:]` 范数；5D 时为 sin(θ) 型控制量 | 历史对照，不能恢复完整 0–π 角度 |
| `completion_fraction` | 最终 `projected_progress_m / total_length_m`；内核对象使用 `path_state[1]` | 绝对弧长位置，不是本次覆盖比例 |
| `scope_completion_fraction` | 本次初始到最终投影弧长对声明区间的覆盖比例 | 必须同时核对起点与结束状态 |
| `feedrate_m_s` / `actual_tangent_speed_m_s` | 调度进给与该边界的切向速度，分别记录 | 降速不自动作为误差；模型雅可比速度不冒充反馈测量 |
| `obstacle_margin_m` | 旧 `min_obs_dist`，模型障碍距离已减安全阈值 | 模型裕度，不等于物理净空 |
| `obstacle_clearance_m` | 调用方显式提供、已注明几何度量的净空 | 没有量测时保持缺失，不能从裕度或 inactive sentinel 反推 |
| `step_latency_ms` / `deadline_miss_rate` | 实测边界耗时；仅 `latency > deadline_ms` 计超期 | 绑定当前配置与时间边界，不代表端到端/物理停止时延 |

统计按样本等权，输出均值、RMS、线性插值 p95、min/max 及有效/缺失/无效计数。部分有效样本可以输出诊断统计，但不能通过要求完整样本的对应判据。缺失、NaN、Infinity 和空样本不会补零或补成 QP 成功。若记录越出声明区间，统计仍保留全部样本供诊断，但声明区间不能完成或通过；不自动过滤越界的低误差或高误差记录。

`constraint_metrics` 接受按约束类别命名的字典，每项包含 `value, quantity, unit, source`，可附 `active`。例如 `joint_linear.residual` 与 `joint_angular.residual` 分别是 m/s、rad/s，不能混加；对应阈值键为 `joint_linear.residual.maximum`。跨 tick 改变量纲/来源会使该序列无效。

节点透传的残差是求解时刻、动态修正后 `max(0, G u_candidate - h)` 的逐类值，使用未经健康门替换的原始候选。`static_margin` 明确是静态 CBF 右端除以基准增益，处于动态项修正前；它不是物理 clearance。奇异性指标保留模型可操作度量纲。障碍或 ESDF 未启用时保留未测和 inactive 计数，不把内核的占位距离当观测。`delta_slack`、全局 `qp_primal_residual` 仍可供求解诊断，跨类最大值不能解释为米。

一步记录对「本来就可能不适用」的槽位带显式状态位：`min_obs_dist_measured`、`min_esdf_dist_measured`。障碍或 ESDF 未启用时内核仍然填占位距离，状态位是内核侧「这不是观测」的表示。节点在交给评价器的样本里据状态位把 `min_obs_dist` 置为缺失（`None`），所以占位值不会进入统计；`min_esdf_dist` 目前没有消费方，节点不在返回样本里做同样的置空，将来读它的人必须自己先查 `min_esdf_dist_measured`。其余槽位（末端位姿、任务误差、耗时）每一步都有值，不加状态位。

## 预先声明子区间的例子

下面声明只验证 1 米路径中的 0.6–1 米。区间对象在采样后不可改写；第一条样本必须覆盖声明起点。若输入是执行一步后的状态，可附 `projection_before_m` 记录第一步的实际初始投影。比较只允许浮点舍入量 `64 * ulp(max(1, total_length_m))`，不使用新增任务误差来掩盖跳过的路径。

输入采用严格区间契约：每条观测投影及所有显式提供的 `projection_before_m` 都须位于声明范围内。调用方负责边界采样；直接跨过终点的记录不能自动插值为实测边界，也不能继续追加低误差尾段稀释验收值。参考点可以领先观测投影，因此 `reference_progress_m` 不用于筛选区间。覆盖还要求观测进度增量超过浮点比较分辨率，零进度及小于该分辨率的变化不能借助舍入余量伪装完成。

```python
import numpy as np
from robot_safecontrol_moveit.tracking_contract import EvidenceContext, EvaluationScope
from robot_safecontrol_moveit.tracking_evaluator import TrackingEvaluator

ev = TrackingEvaluator(
    scope=EvaluationScope(total_length_m=1.0, start_m=0.6, end_m=1.0),
    evidence=EvidenceContext(
        run_id="line-subinterval-1", kind="analytic", boundary="kernel_candidate",
        model_id="analytic-point-v1", config_id="example-v1", trajectory_id="line-1m",
        data_id="three-analytic-samples", scenario="straight-line subinterval",
        measurement="analytic point and same-time reference", time_basis="test seconds",
    ),
    deadline_ms=20.0,
)
for t, s in enumerate((0.6, 0.8, 1.0)):
    ev.update(dict(
        ee_pos=np.array([s, 0., 0.]), reference_position_m=np.array([s, 0., 0.]),
        reference_tangent=np.array([1., 0., 0.]),
        ee_rot=np.eye(3), reference_rotation=np.eye(3),
        reference_at_endpoint=(s == 1.0), projected_progress_m=s,
        qp_ok=True, admission_ok=True, overlap=False, step_latency_ms=1.0,
    ), wall_time_s=float(t))
ev.finish("completed")
report = ev.report()
assert report.task_verdict == "pass"
assert report.completed and not report.full_path_covered
assert report.verdict == "insufficient_evidence"  # 默认 provisional 判据仍未定
```

提前保持、取消、故障分别用 `finish("held"/"cancelled"/"fault")`；任务冲突、异常中断分别为 `task_conflict`、`interrupted`。结束后不能追加样本或改写终止原因。`record_event()` 可追加具名事件；准入拒绝、命令拒绝、任务冲突和故障不能被后续正常样本覆盖。零长度任务不属于本路径跟随评价契约，构造时拒绝。

## 控制节点输出与迁移

控制节点按配置路径生成固定全路径声明，复用启动快照，额外记录实际变换后轨迹数组哈希和评价器源码身份。当前默认报告是 `model/kernel_candidate`：误差取内核积分后的模型状态，尚未经过发布低通或实际执行。尚未取得的最终命令、模拟状态、真实反馈和准入/重叠证据明确未测；OFF-09/OFF-13 的后续结果可经上述输入契约接入。

节点在创建路径数组前应用 `enable_x64`，报告范围及轨迹哈希绑定内核实际使用的全部路径数组；运行快照记录 `path_array_dtype`。显式使用 float32 时也按实际量化后的路径总长评价，不拿原始 float64 总长比较。初始化修复可能使首次启动的浮点输出不同于旧记录，控制公式与验收阈值保持。

节点在已知保持/参考端点终止时结束采样，把已封存的评价器交给单个后台写盘线程；控制回调不遍历、复制或序列化历史样本，保持命令继续发布。后台工作不操作 ROS 节点；节点遥测轮询保存状态，`progress_snapshot()["report_status"]` 提供 `idle/writing/saved/failed`、目标路径和错误。仅 `saved` 表示本次报告文件已全部写完；`writing` 期间不要把同名旧文件当作本次结果，`failed` 必须视为本次保存未成功。

正常销毁节点时，未结束采样的报告标记为 `interrupted`，在停止执行回调后等待后台任务完成，再销毁 ROS 实体。异常强制结束进程不保证保留报告。后台线程消除同步写盘停顿，但仍与控制线程共享 CPU；它不是硬实时或物理停车保证。`write_tracking_report()` 保留为回调外的显式同步导出接口；后台正在写入时拒绝并发导出。

报告先写入同目录临时文件，再逐个替换目标文件，含样本哈希的汇总 JSON 最后替换；这不是三文件同时切换的事务，读取者应等待 `saved` 并校验哈希。默认文件与性能报告同目录：

- `tracking_report.md`：人读指标、判据、事件与证据限制。
- `tracking_report.json`：结构化汇总，严格 JSON，不输出 NaN/Infinity 字面量。
- `tracking_report.samples.json`：全部规范化样本、时间、终止状态、区间和阈值；非有限样本采用显式标记保留。文件内容的 SHA-256 与汇总报告中的 `sample_data_sha256` 对应。

离线调用者可用 `report.markdown()`、`report.json()`、`ev.trace_json()` 自行存储，不引入 ROS。现有调用方需将 `tracking_score` 改为明确的 verdict，将扁平误差字段改为 `report.metrics["cross_track_m"].rms` 等结构化访问。`summary()`、`completed`、`completion_fraction` 和 QP 比例仍保留，但完成度与完成事件已按上述新定义纠正；仅给 `trajectory_duration_s` 的旧输入可出诊断，无法通过范围验收。
