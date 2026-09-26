# 实施与验证

- [x] 定义版本化 schema 与参数加载、不可变结构、parameter/policy hash。
- [x] 连接 CollisionSafety 配置，验证三个公开操作的 identity 和参数来源。
- [x] 验证必需 provenance、全部参数变化、独立进程 hash、旧 scene 拒绝和不可变行为。
- [x] 更新依赖、安装与模块说明，验证安装资源。
- [x] 运行受影响测试及适用回归；全部中间文件使用 `.scratch/i23/`。
- [x] 执行 trellis-check 与主会话 code-review，记录结果。
- [x] 在本票记录验收证据并更新地图。

## 检查入口

`portable_oscbf/tests/test_collision_parameters.py`、`portable_oscbf/tests/test_collision_safety.py`、`portable_oscbf/tests/test_cbfpy_dependency_contract.py`、`tests/test_model_installation.py`。根据实际修改范围追加回归。

## 规划复核

范围符合已接受的规格和 ADR；没有新增产品决定。Codex 使用 inline 执行，主会话完成实现和审查。

## 实际验证

环境为 Python 3.10.12、jax/jaxlib 0.6.2、cbfpy 0.0.1、qpax 0.1.4、jsonschema 4.23.0。JAX 使用 CPU。
代码基准为 `2440192f04b3b9d59b5d10666a589c1a2970eb1c`，分支为 `codex/i23-collision-parameter-artifact`。

- 首轮相关测试：`PYTHONPATH="$PWD/.scratch/i23/deps:$PWD/portable_oscbf" TMPDIR="$PWD/.scratch/i23/tmp" OPENBLAS_NUM_THREADS=1 python3 -m pytest -q portable_oscbf/tests/test_collision_parameters.py portable_oscbf/tests/test_collision_safety.py portable_oscbf/tests/test_cbfpy_dependency_contract.py --basetemp .scratch/i23/pytest`，122 passed。
- 最终完整回归：`PYTHONPATH="$PWD/.scratch/i23/deps:$PWD/.scratch/i20-tetgen:$PWD/src:$PWD/portable_oscbf" TMPDIR="$PWD/.scratch/i23/tmp" OPENBLAS_NUM_THREADS=1 PYTEST_ADDOPTS='--basetemp=.scratch/i23/full-pytest' bash run_all_tests.sh`，主包 547 passed、1 skipped；内核 310 passed、34 skipped；AEB CTest 4/4 passed；三个退出码均为 0。日志为 `.scratch/i23/full-tests.log`。
- `TMPDIR="$PWD/.scratch/i23/tmp" python3 setup.py egg_info --egg-base .scratch/i23 build --build-base .scratch/i23/build bdist_wheel --bdist-dir .scratch/i23/wheel-build --dist-dir .scratch/i23/dist`：成功，日志为 `.scratch/i23/build.log`。
- `PYTHONPATH="$PWD/.scratch/i23/deps" TMPDIR="$PWD/.scratch/i23/tmp" OPENBLAS_NUM_THREADS=1 python3 .scratch/i23/verify_installed.py`：从 wheel 独立目录成功加载 schema、artifact、policy 和 CollisionSafety；独立进程 identity 与源码一致。
- 默认 Python 环境已安装 `jsonschema==4.23.0`，`PYTHONPATH=portable_oscbf` 下导入 CollisionSafety 成功。
- 对修改的 Python 模块和测试执行 `python3 -m compileall -q`，检查新增文档链接目标，并执行 `git diff --check`：通过。
- 关键源文件 SHA-256 清单为 `.scratch/i23/source-sha256.txt`。

解析接口测试的 artifact identity：`b23f85df251845bb648b0f5613e51a04728fb5b45a9e2e71bafa3feb37b7e0b5`。
对应 policy identity：`77ddd9f2ec5ef710e72d653fd9d8ce7c92e2c0cf554601f3fe5e10a080086aab`。
这组输入的设备和场景分别为 `analytic-interface-test` 与 `json-and-identity-validation`。

## 工作流交接

- Phase 3.3：已有 ADR 0045 规定本次参数与 identity 规则，使用说明已经同步到模块文档；没有新增需要确认的通用决定，未修改 `.trellis/spec/`、CONTEXT 或 ADR。
- Phase 3.4：用户明确授权仅提交并推送本票改动，更新议题与执行地图，归档本地任务，并合并分支后只保留 `main`。
- 已按用户调用的 wayfinder 写入[票尾验收记录](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/102#issuecomment-5842877079)、关闭本票并更新统一执行地图。
- 用户已有的 LiDAR 标定任务和规划资料保持原状。
- 实现提交为 `772d6970bd9c75db639f0f22e318c84156a0c397`，已推送至远程仓库。提交前核对源文件 SHA-256 与完整回归所验证的版本一致。
- 真实安全数值、设备测量、独立验收、几何消费者的完整 pair 覆盖以及后续规划/恢复准入由地图中的相关任务继续处理。
