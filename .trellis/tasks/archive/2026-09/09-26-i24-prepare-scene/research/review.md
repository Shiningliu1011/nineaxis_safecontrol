# 独立审查

基准为 `dc0f0cc81f4064803f04bfc542d9ed8a90340610`，审查范围包括本任务新增文件及工作区变更。两个审查者分别执行规范与需求检查；主会话负责实现、行为验证和文档。

## 规范与代码质量

`i24_standards_review` 已完成复审，未发现仍需处理的问题。

- 准备期限检查位于成功结果构造、全部 device 复制和 `jax.block_until_ready` 完成之后。
- 完成时间采样后的成功路径只进行 Python 期限比较与返回。
- 真实 2 ms 期限实验共运行 100 次，98 次准入、2 次拒绝。函数外部计时包括调用、返回和局部对象释放；最大准入样本外部计时为 2,015,826 ns，不能据此推断内部数据完成时间超限。该实验不提供生产性能准入结论。

## 需求与验收

`i24_requirements_review` 已完成复审，未发现本票范围内仍需处理的问题。

- `_scene_status` 在 JAX 检查及期限读取完成后采样当前时间。
- query 使用 header 的同一个 `completed_ns` 检查场景、coverage 和 tracking 有效期，失败时清除准入 mask。
- `tracking_valid_until_ns` 保存全部活动 track 的最早期限，固定数组结构保持不变。
- `test_query_completion_cannot_outlive_any_admission_deadline` 对三项期限分别运行 80 个真实时钟窗口；所有 `OK` 结果的完成时间必须处于对应有效期内，同时观察准入与拒绝两类结果。

## 验证证据

- 时间边界与公开操作回归：124 passed，16.46 秒，日志 `.scratch/i24/time-boundaries.log`。
- 准备期限实验：`.scratch/i24/deadline_probe.py`、`.scratch/i24/deadline-probe.json`。
- 全范围回归命令和结果见同目录上级的 `implement.md`；最终代码摘要保存在 `.scratch/i24/code-sha256.txt`。
- 两个独立审查者只读检查当前代码与测试，未重复运行主会话已经完成的测试。
