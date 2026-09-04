# 04 — portable 测试容差回归修复（roll-only 参考起点）

**What to build:** `portable_oscbf/tests/test_jax_tool_axis_tracking.py::
test_tool_axis_path_kernel_ignores_roll_only_reference_at_path_start` 以
`rtol/atol=1e-8` 断言 `err_6d ≈ 0`，实测
`[8.57e-08, 1.22e-08, 2.08e-08, 6.77e-08, 2.52e-08, 0]`（JAX 数值精度级，
非逻辑失败）。核对语义无回归后，将容差放宽到与该文件其他断言一致的量级
（如 atol=1e-6），确保全量 portable 套件恢复 0 failed。

**Blocked by:** None.

**Queue:** implementation-backlog（已移出 Wayfinder）
**Tracker:** #11 (https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/11)

**Status:** ready-for-agent

- [ ] 确认 roll-only 参考在路径起点被正确忽略（语义核对，非只改容差）
- [ ] 容差放宽至与同文件其他断言一致（atol=1e-6 或更松）
- [ ] portable_oscbf/tests 全量恢复 0 failed
