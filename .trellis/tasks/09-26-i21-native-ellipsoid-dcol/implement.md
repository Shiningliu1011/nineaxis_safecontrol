# 执行与验证

基准：`8a6a3ce`，初始分支 `main`。已有 T1 标定任务及草稿保持原状。

- [x] 核对票据、执行地图、规格与 ADR；领取票据。
- [x] 加载并校验 geometry、policy、primitive identity 和 margin。
- [x] 实现 JAX DCOL、梯度和完整健康判定；接入公开 query。
- [x] 通过解析、独立高精度、官方 DCOL、生产几何与失败路径测试。
- [x] 更新接口文档，运行相关回归与质量检查。
- [x] 独立规范与需求审查，处理发现并记录结果。
- [x] 发布实际验收证据、关闭票据并更新地图索引。

## 验证约定

测试在本机 CPU 离线执行；中间文件、第三方离线参考及 pytest 临时目录使用 `.scratch/i21/`。数值验收预设 scale 误差 `2e-8`、关节梯度误差 `2e-5`，接近退化参考 scale 误差 `2e-6`。solver 限制由解析测试自己的 parameter artifact 记录。测试不修改生产参数，不使用 mock。

## 已执行检查

- `TMPDIR=$PWD/.scratch/i21/tmp PYTHONPATH=$PWD/.scratch/i21/python-deps python3 -m pytest portable_oscbf/tests/test_ellipsoid_dcol.py -q --basetemp=.scratch/i21/final-dcol-pytest`：33 passed，65.07 s；日志 `.scratch/i21/final-dcol-tests.log`。
- 接近退化几何新增九轴高精度差分后，定向运行 `-k nearly_degenerate`：1 passed，32 deselected，5.24 s；日志 `.scratch/i21/degenerate-gradient.log`。
- `test_collision_safety.py`、`test_collision_parameters.py`、`test_collision_geometry_artifact.py`：132 passed，69.03 s；日志 `.scratch/i21/regression-with-deps.log`。
- `TMPDIR=$PWD/.scratch/i21/tmp PYTHONPATH=$PWD/.scratch/i21/python-deps bash scripts/agent_check.sh`：全部检查通过，含 69 项纯逻辑测试。
- `python3 -m compileall -q portable_oscbf/work portable_oscbf/tests/test_ellipsoid_dcol.py portable_oscbf/tests/reference/ellipsoid_dcol`、`git diff --check`：通过。
- 官方参考：Julia 1.10.12，DifferentiableCollisions.jl `3d29f760a295dbf97ba3c151a8f26bed317e70ce`，`pdip_tol=1e-12`；原版源码 checkout 未修改。六项公开 query 与官方参考/80 位 KKT 检查通过。生成命令见 `research/validation.md`。
- 数值证据见 `research/numerical-evidence.json`：官方 scale 最大绝对误差 `2.7711166694643907e-13`，旋转案例九轴梯度最大绝对误差 `1.0331944189090336e-9`。

完整回归的命令和结果由 `research/validation.md` 保存。

[票尾验收证据](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/107#issuecomment-5843138448)已发布。票据已关闭，统一执行地图已增加本票的索引。

## 交付边界

独立规范及需求审查见 `research/review.md`，无待处理发现。当前内容遵循既有 ADR，没有需要提升到 `.trellis/spec/`、`CONTEXT.md` 或新 ADR 的决定。用户授权仅提交和推送本票改动、维护议题与地图并归档本任务，分支仅保留 `main`。提交范围限定为本票实现、测试、接口文档及 Trellis 记录；已有 T1 文件保持原状。
