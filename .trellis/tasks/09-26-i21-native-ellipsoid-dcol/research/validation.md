# 验收证据

环境：Python 3.10.12，JAX/jaxlib 0.6.2，CPU；Linux 6.8.0-138-generic。验证覆盖基准 commit `8a6a3ce` 之上的本票实现、测试和文档，交付 commit 由任务 metadata 记录。

## 公开 query 数值验收

最终 DCOL 测试 33 项通过；接近退化九轴差分另行通过。覆盖分离、接触附近、相交、尺寸差异、旋转、接近退化、全部九轴梯度、残差、迭代上限、同心、非有限计算、间距换算溢出、批次及槽位 mask、相反危险方向、身份、容量、场景状态、deadline 和生产资产 45 个 pair。测试只使用公开操作与真实数值算法。

几何 identity：`add9743e201beff9ed1154af1ee2c7283d6f34533fde1b7b5bd169c30b7c5fc7`。解析测试参数仅用于离线数值验证，不代表测量参数资格。

数值记录：`numerical-evidence.json`。六组官方参考最大 scale 绝对误差 `2.7711166694643907e-13`；旋转案例九轴差分最大绝对误差 `1.0331944189090336e-9`。生产几何在本机 CPU 的 20 次预热查询均有 45 个健康 self pair，查询运行时间 `6.157035–7.682478 ms`。该采样用于本地诊断，目标设备 deadline 由独立验收处理。

## 官方参考复现

原版 DifferentiableCollisions.jl commit `3d29f760a295dbf97ba3c151a8f26bed317e70ce`，Julia 1.10.12；本地 checkout 在 `.scratch/i21/reference/DifferentiableCollisions.jl`，依赖由原项目的 `Pkg.instantiate()` 安装，源码未修改。

```bash
TMPDIR=$PWD/.scratch/i21/tmp JULIA_DEPOT_PATH=$PWD/.scratch/i21/reference/depot PYTHONPATH=$PWD/.scratch/i21/python-deps python3 portable_oscbf/tests/reference/ellipsoid_dcol/generate.py --julia .scratch/i21/reference/julia-1.10.12/bin/julia --project .scratch/i21/reference/DifferentiableCollisions.jl --work-dir .scratch/i21/reference/corpus --output portable_oscbf/tests/reference/ellipsoid_dcol/official.json
```

退出码 0。输出包含输入 SHA-256、参考程序 SHA-256、来源版本和六组 scale。对应 Python 测试同时核对官方结果与独立 80 位原始 KKT 解，全部通过。

## 回归

接口、参数、几何资产：132 passed。快速检查、compileall 与 `git diff --check` 通过。

完整入口：

```bash
TMPDIR=$PWD/.scratch/i21/tmp PYTHONPATH=$PWD/.scratch/i21/python-deps PYTEST_ADDOPTS='--basetemp=.scratch/i21/full-pytest' bash run_all_tests.sh
```

完整输出保存在 `.scratch/i21/full-tests.log`：主包 547 passed、1 skipped（188.25 s）；控制内核 342 passed、34 skipped（624.96 s）；AEB CTest 4/4 passed（3.94 s）。三个退出码均为 0。最终专项 suite 覆盖全部 33 项 DCOL 用例及间距换算溢出检查。

## 使用限制

query 当前完成 self DCOL。有效 environment support、距离模式和未绑定 geometry 的查询保持非 `OK`；`certify` 保持未通过。生产控制路径及真实设备安全限制保持当前状态。目标设备期限、完整场景能力和生产切换由对应执行票验收。
