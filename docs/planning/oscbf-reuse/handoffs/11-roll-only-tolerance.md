# [04] portable 测试容差回归修复（roll-only 参考起点）

- Tracker：https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/11
- 日期：2026-09-11
- 状态：测试修订及本机验收完成；修复票已完成，测试修订及验收日志随本提交归档。
- 起点：沿用当前工作区及前一窗口 HEAD `786e0aec61b7abbf8e8c0a5c5aa99d20c3e54f21`，保留已有未提交修改。
- 前轮范围：研究并交接，不修改正式测试或控制代码。
- 本轮用户指令：调用 wayfinder 处理本票；按 ready-for-agent 交接修订测试并验收。

## 结论

不采用原票“只把 err_6d 的 atol 改成 1e-6”方案。独立运行暴露两件不同的事：构造参考时的 JAX 精度与初始化后精度不一致；路径 err_6d 是执行一步后的误差，不是起点误差。应明确测试精度、分别检验起点纯 roll 语义与执行后遥测语义。

## 复现与证据

命令：`python3 -m pytest portable_oscbf/tests/test_jax_tool_axis_tracking.py::test_tool_axis_path_kernel_ignores_roll_only_reference_at_path_start -q`。
结果：1 failed in 22.75s，err_6d 最大绝对值 8.5742118e-8，超过原 atol=1e-8。

隔离探针从原测试提取同一初始化和路径调用，只把断言换为数值输出，保留 qp_ok 断言。文件已发布到 [隔离探针](../research/11-roll-only-evidence/probe.py)（非正式测试）。

使用与 portable_oscbf/conftest.py 相同的本地依赖路径：

```bash
PYTHONPATH=portable_oscbf:portable_oscbf/vendor/dpax JAX_ENABLE_X64=0 python3 docs/planning/oscbf-reuse/research/11-roll-only-evidence/probe.py
PYTHONPATH=portable_oscbf:portable_oscbf/vendor/dpax JAX_ENABLE_X64=1 python3 docs/planning/oscbf-reuse/research/11-roll-only-evidence/probe.py
```

| 启动精度 | init_cbf 前路径 dtype | max abs(err_6d) | max abs(u_nom) |
|---|---|---|---|
| 默认 32 位 | float32 | 8.5742118e-8 | 1.5069893e-5 |
| 构造前启用 64 位 | float64 | 2.7295380e-8 | 1.5473833e-15 |

64 位详细探针还测得：起点 PRE_ERROR 全零，max abs(u_safe)=0.0185421，max abs(q_next-q)=0.000185421。这说明零名义命令与实际安全命令不同；本票未归因其具体活跃约束，不能称为单纯浮点噪声或认定 QP 故障。

日志：[默认精度](../research/11-roll-only-evidence/default.txt)、[预启用 x64](../research/11-roll-only-evidence/x64.txt)、[x64 详细输出](../research/11-roll-only-evidence/x64-detail.txt)。首次直接运行探针遗漏 vendored dpax 路径而失败，补齐 pytest 原有路径后获得上述数据；没有安装或替换依赖。

源码因果链：

- `JaxControlLoop` 默认 enable_x64=True，但直到 init_cbf 才更新全局 JAX 精度。
- 构造机器人已把运动学常量转成 JAX 数组；configure_path 同样使用 jnp.asarray。测试在 init_cbf 之前取得 FK 并创建路径，默认独立进程会先落成 float32；后续启用 x64 不会恢复已损失精度。
- jax_kernel_factory 的 `_path_finalize` 在 q_next 上重算位姿，再产生 err_report_next。纯 roll 的零名义命令不能推出求解和积分后的误差严格为零。
- tool_axis_task 的第六遥测分量是补零，不是受控 roll 误差。

语义验证：`python3 -m pytest portable_oscbf/tests/test_tool_axis_task.py -q` 得到 4 passed（包含纯 roll、倾斜反馈方向及雅可比检查）。额外调用 JAX task_error_5d_jax：0.7 rad 纯 roll 得到全零；0.001 rad 工具轴倾斜得到约 -0.001 的非零受控分量。因此当前证据支持几何层保留 roll 自由，但不把这些局部测试解释为完整控制或安全验收。

## 可以开始编码：推荐范围

仅修订本票测试及必要说明，不修改生产精度、求解器容差或控制参数：

1. 在构造 loop/机器人/路径之前显式固定该测试所需的 x64；测试结束恢复原全局设置，避免依赖其他测试的执行顺序。仅给构造函数传 enable_x64=True 不足以提前生效。
2. 起点纯 roll：直接断言起点工具轴误差与 u_nom 接近零，保留现有严格量级。增加同一入口的小幅工具轴倾斜对照，防止忽略全部姿态也通过。
3. 执行后 err_6d：与 q_next 的独立 NumPy FK/工具轴计算结果核对，而非把它当起点误差要求严格等于零。若保留“执行后偏移很小”的额外断言，需单独说明其测量对象和依据，不能将测试容差当作实机安全阈值。
4. 不将整个文件的容差统一放宽；原票提及的“与同文件其他断言一致”并不准确，相邻名义命令断言仍为 1e-8。

复用：保留现有 JAX、NumPy/SciPy、pytest 与仓库 vendored dpax；使用已有 NumPy 运动学/工具轴实现作独立参照。不新增算法、依赖、fork 或上游补丁。

## 实施验收与回退

- 在默认精度独立进程、预启用 x64 的独立进程、整个测试模块中验证同一断言。
- 确认纯 roll 的名义命令接近零，工具轴倾斜对照不能被忽略，执行后报告符合 q_next 的独立计算。
- 运行完整 portable_oscbf/tests，记录真实结果；本轮未运行全量，不宣称 0 failed。
- 回退只撤销本票的测试补丁。全局精度初始化的生产接口问题记录给控制模块复用与九轴适配票核验，本票不扩大为生产改动。

## 下一步

本票测试修订与全量验收已完成；发布 resolution 并关闭原实施票，本次按用户指令提交至仓库。本轮不自动启动下一票。


## 实施记录（2026-09-11）

基于 HEAD `3ec19a41aebfa1567d55caa851feed3bf4d9cbee` 的现有工作区实施，保留其他未提交修改。正式代码变更仅在 `portable_oscbf/tests/test_jax_tool_axis_tracking.py`，未修改生产控制代码、求解器或生产精度设置。

- pytest fixture 在机器人与路径构造前启用 x64，并在 finally 中恢复原设置。
- 同一路径入口参数化覆盖纯 roll 和 0.001 rad 工具轴倾斜：纯 roll 起点误差与名义命令保留 1e-8 绝对容差；倾斜时核对非零工具轴误差和名义命令。
- 零进给参考仍位于路径起点；执行后报告用已有 NumPy `NineaxisKinematics.ee_pose(q_next)` 与 `task_error_5d` 独立计算，以 1e-12 绝对容差核对。该容差用于双精度实现对照，不是实机安全阈值。
- 原测试独立复现：1 failed in 21.62s，最大误差 8.5742118e-8。

验收日志已归档至 [实施验收证据](../research/11-roll-only-evidence/implementation/)，本机原始目录为 `output/ticket-11-implementation/`。

| 验证命令 | 结果 | 日志 |
|---|---|---|
| `python3 -m pytest portable_oscbf/tests/test_jax_tool_axis_tracking.py::test_tool_axis_path_kernel_ignores_roll_only_reference_at_path_start -q` | 2 passed in 43.03s | [default.txt](../research/11-roll-only-evidence/implementation/default.txt) |
| 上述命令前加 `JAX_ENABLE_X64=1` | 2 passed in 30.60s | [x64.txt](../research/11-roll-only-evidence/implementation/x64.txt) |
| `python3 -m pytest portable_oscbf/tests/test_jax_tool_axis_tracking.py -q` | 6 passed in 53.83s | [module.txt](../research/11-roll-only-evidence/implementation/module.txt) |
| `python3 -m pytest portable_oscbf/tests -q -rs` | 148 passed, 34 skipped in 469.64s；退出码 0 | [full.txt](../research/11-roll-only-evidence/implementation/full.txt) |

回退仅撤销本票测试修改及相应状态说明；测试修订及验收日志随本提交归档。

34 个跳过项来自既有 portable 范围排除和缺少 `newaxis`，不是通过项。本轮未执行主包套件或真机验收；没有新增待决取舍或需要新增票据的范围。
