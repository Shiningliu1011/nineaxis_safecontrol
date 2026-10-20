# QP 进给边界

`s4-joint-development-05` 使用 D 方法和 0.1 mm 共同控制余量。
在路径约 65.9% 处，`controller.sqlite3` 的 `qp_failure` sequence 5906
保存了原始 QP；进给为 −2.3773482560922366e-26 m/s。
QP 最终 KKT 检查通过，随后严格进给范围检查终止运行。

边界处理保持原始任务目标与所有不等式：当通过 KKT 检查的结果处于
进给边界外侧，将该边界作为固定进给重新求解九轴速度，并根据原始
完整 QP 重建边界乘子、检查 KKT 和不等式。候选运动仍须通过连续
Ruckig 几何与关节检查。

原始矩阵保存在 `tests/data/static_qp_feed_boundary.npz`。
回归测试使用实际 qpax 与独立 SciPy SLSQP 核查范围、约束与目标值。
