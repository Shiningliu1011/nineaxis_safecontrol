# main 集成验收

独立 merger 将审查通过的 #152 移植到 `/home/lsn/robot/robot_safecontrol`，保留 #108、#153 和主会话决定文档。源工作区为 `.scratch/worktrees/map147-path-reference`；源与 main HEAD 均为 `9c354d5505e26aa28b3e6fc323c661bd7542b105`。全部修改未提交。

## 文件与来源

冻结清单 `.scratch/map147-152/frozen-handoff.json` SHA256 为 `1559c4fc965328d1eaaa05fcf70bffe1285b1b0be188c2c1c1204915eecb44f4`。开始前核验全部 42 个来源文件、HEAD、分支、main 目标与既有修改，记录在 `integration/before.json`。源码、测试和文档逐项通过文件补丁移植。

`setup.py` 同时保留 #152 连续包安装、#105 的 rtree/urdf-parser-py 依赖及解析回放 JSON 安装。与来源 #152 的全部差异为 #105 的两项依赖和一项数据安装；移除这些已知新增文本的内存比较与源文件完全一致。共同模块索引保留 #108、#153，并新增连续路径和 self-filter 入口。

生成器的五份源码、公共关节身份来源和全部配置都指向 main，来源 hash 与已审清单相同；生成前证据为 `integration/generation-inputs.json`，命令为 `integration/generate.sh`。生成器执行两次真实 CLI，使用原 `data/nurbs` 输入，不复制工作区产物。

| 版本 | identity | main manifest SHA256 |
|---|---|---|
| production | `62049d74d7f44e85b7d70715105a7821dabca29bc0687c0b06c852739e4d9972` | `a46d39fe652327fbd9e5fd259d0fd941f88d47bfb349747a0190392d493c9261` |
| S0 | `58ff1e18b3c7f874f9ec2a3557785d6316bd4a2136bb40b25014d089b91f877e` | `671e65317f6a47cbd9b8237e1c5a703fec5a91f6dea052f73263c3af6b73d9d9` |

每份 `reference.npz`、`samples.npz`、`source_samples.npz` 的全部数组与字节 SHA256 都与冻结产物相同。两份 manifest 的命令和路径记录 main 生成来源。独立 SciPy BSpline、相似变换与两次圆柱投影重算每份 14992 个原样本：原曲线最大差异 `8.526512829121202e-14 mm`，位置最大差异 `2.498001805406602e-16 m`，轴向最大差异 `1.1102230246251565e-15`，全部有效。原始输入文件保留。初次资产结果为 `integration/artifacts.json`，组合 setup 完成后的复核为 `integration/artifacts-combined.json`。

## main 直接消费者与安装

环境入口为 `.scratch/map147-105/integration/env.sh`，使用主仓库 `src`、`portable_oscbf` 和任务依赖；Python 3.10.12、NumPy 1.26.4、SciPy 1.15.3、JAX 0.6.2。所有测试临时、字节缓存、ROS、Matplotlib 和数值产物使用被忽略的 `.scratch` 目录。

| 检查 | 实际结果 | 证据，以 `.scratch/map147-152` 为根 |
|---|---|---|
| 连续查询、资产与受影响消费者，含 #153 资源 | 首次 89 passed、1 failed；同一失败测试随后 1 passed | `integration/consumers.log`、`evaluation-retry.log` 与对应 JUnit |
| Ruff 新文件与既有消费者致命规则 | 通过 | `integration/ruff-new.log`、`ruff-consumers.log` |
| mypy 五份实现 | 通过 | `integration/mypy.log` |
| Trellis context | 5 条 implement、4 条 check 全部通过 | `integration/context.log` |
| main 组合隔离 colcon | 通过，包 9.25 s，总 9.89 s | `integration/build-install.log` |
| 安装源码、配置、数据字节 | #152 34 项、#105 7 项及 #153 资源相同 | `integration/installed-files.json` |
| Python `-I` 实际安装副本 | 两份包、连续消费者、模型/限制/FK、LiDAR 几何与解析射线通过 | `integration/installed-imports.json` |

首次消费者失败由既有 `.scratch/static-experiments` 链接指向未挂载的研究目录引起。可恢复目录准备运行同一公开入口后通过；原链接已恢复。完整诊断为 [评价临时目录检查](diagnosis.md)，原失败记录继续保留。

安装位于 `integration/install/robot_safecontrol_moveit`。安装探针明确检查模块文件位于该安装目录；模型、限制、连续查询和 LiDAR 几何加载期间没有导入 rclpy/rospy/ament。之后真实 ROS 消息 adapter 从该安装 lib 导入。原九轴 696330 个三角面与未验证标定状态完整保留，解析盒体首交点为 2.5 m，回放文件标识 `analytic_interface_replay`。

## 完整项目检查

已完整读取 `run_all_tests.sh`、pytest 配置、实际 runtime 进程管理、最终 launch 的 sim/live 边界及四份 AEB 测试入口。采用同样的 pytest/ctest 三层入口，主包及控制内核逐文件执行以释放 JAX 编译资源；环境加载本轮安装及 main 源码。完整收集为主包 854 项、控制内核 741 项。性能项独立运行，其余目标不删除。

运行入口为 `integration/run_full.sh`，完整计划、逐文件命令、退出码、收集 nodeid、setup/call/teardown、跳过原因和 JUnit 存放在 `integration/full/`。既有绘图环境 `.scratch/pipe-assets-runtime/venv` 用于四个绘图相关文件；实际 Matplotlib 与 Axes3D 路径由 `plot-environment.json` 核验。固定评价目录与两个固定输出路径采用可恢复本地目录或链接，运行结束恢复原对象。

四个绘图进程首次继承 ROS 路径，在收集前因插件所需 yaml 未提供而退出。独立绘图路径完整重跑同四份文件得到 35 passed，来源和断言保持原内容；`runs-plot-retry.json` 与阶段结果保留全部实际运行，首次日志继续保留。完整覆盖按 nodeid 采用真实成功结果并保存两次命令，详见 [环境检查](diagnosis.md)。

完整覆盖审计为 `integration/full/coverage.json`，逐项验证初次收集集合、全部计划文件、实际阶段结果与复验来源，无缺项。结果如下：

| 范围 | 完整收集 | 功能结果 | 独立计时 |
|---|---:|---|---|
| 主包 81 个文件 | 854 | 847 passed、6 skipped | controller 在下表所列四核心条件通过 |
| 控制内核 60 个文件 | 741 | 706 passed、34 skipped | modular 在下表所列八核心条件通过 |
| AEB ctest | 4 | 4 passed | 不含实时周期结论 |

真实 final launch 为 1 passed、transition runtime 为 14 passed，已经包含在主包数量内。主包首次批次退出 1 源于四份绘图进程启动失败；内核首次批次退出 1 源于 modular 相对计时。原命令、退出码和失败阶段均保存；同一 nodeid 的复验按实际结果和明确条件记录。

主包六项跳过分别为未配置 `HARDWARE_TEST_VCAN` 的一项 Linux vcan 验收，以及未提供 `STATIC_PLAYBACK_RUN` 原件的五项保存反馈重放。内核 34 项沿用当前可移植包未包含的 `newaxis`、`work.unified_qp_solver`、`work.jax_qp_benchmark` 相关原排除标记或 `importorskip`；每项原始原因位于覆盖清单。跳过项不计为通过。

功能批次结束、其他数值 worker 自然完成并暂停后，顺序测量以下五个条件。pytest 进程内记录实际 affinity，同期采样保存 CPU 拓扑、频率、负载、进程与源码 SHA256。所有源码、ROS executor 四个线程、控制参数和原断言保持一致。

| 计时入口与 CPU 集合 | 实际数值 | 原断言结果 | 原始目录，以 `integration/full/` 为根 |
|---|---|---|---|
| controller，`0-27` | 300 步，p95 28.082 ms，229 步超期（76.33%） | failed | `perf-all-cpus/` |
| controller，`0,2,4,6,8,10,12,14` | 300 步，p95 18.511 ms，4 步超期（1.33%） | failed | `perf-p-cores/` |
| controller，`0,2,4,6` | 303 步，p95 14.343 ms，1 步超期（0.33%） | passed | `perf-p-four/` |
| modular，`0-27` | 3000 步，单步平均 14.938 ms，相对原基准 6.452 ms | failed | `perf-modular-all-cpus/` |
| modular，`0,2,4,6,8,10,12,14` | 3000 步，单步平均 6.379 ms，初始化 27.36 s | passed | `perf-modular-p-cores/` |

controller 原门槛为 p95 < 20 ms、超期比例 ≤ 1%、至少 300 步；modular 原门槛为逐步数值与进度、QP 成功、单个 JIT 缓存、初始化 < 45 s 和相对单步耗时 ≤ 120%。modular 八核心报告的全部数值差异为零，缓存为 1。原基准 SHA256 始终为 `40bdab1e26b1594a227e851e93cacac1856591969cec704076456ccd415a916f`，复核只读取原文件，没有重新生成基准。

四核心 controller 是在八核心原测试失败后预先声明假设、仅改变 CPU 集合的一次对照，完整说明见 [计时诊断](diagnosis.md)。结果限定于各自记录的条件；完整项目共同周期尚待正式测量。原研究链接与两个固定 output 对象已恢复，数据产物保存在本次 `.scratch` 证据目录内。

实际运行入口如下，工作目录均为主仓库。每项 pytest 的完整参数、Python 解释器、输出目录及实际退出码另存 `full/plan-*.json`、`runs-*.json`；证据目录的存在性检查会拒绝覆盖已有记录，复验需选择独立输出目录。

```bash
bash .scratch/map147-152/integration/generate.sh
bash .scratch/map147-152/integration/run_consumers.sh
bash .scratch/map147-152/integration/run_with_local_evaluation.sh evaluation-retry bash .scratch/map147-152/integration/retry_evaluation.sh
bash .scratch/map147-105/integration/run_consumers.sh
bash .scratch/map147-152/integration/build_install.sh
bash .scratch/map147-152/integration/check_installed.sh
bash .scratch/map147-152/integration/run_full.sh collect
bash .scratch/map147-152/integration/run_with_local_evaluation.sh full-main bash .scratch/map147-152/integration/run_with_local_outputs.sh bash .scratch/map147-152/integration/run_full.sh main
bash .scratch/map147-152/integration/run_full.sh kernel
bash .scratch/map147-152/integration/run_full.sh cpp
source .scratch/map147-152/integration/full_env.sh
python .scratch/map147-152/integration/retry_plots.py
bash .scratch/map147-152/integration/run_performance.sh all-cpus 0-27 controller
bash .scratch/map147-152/integration/run_performance.sh p-cores 0,2,4,6,8,10,12,14 controller
bash .scratch/map147-152/integration/run_performance.sh all-cpus 0-27 modular
bash .scratch/map147-152/integration/run_performance.sh p-cores 0,2,4,6,8,10,12,14 modular
bash .scratch/map147-152/integration/run_performance.sh p-four 0,2,4,6 controller
python .scratch/map147-152/integration/audit_full_results.py
```

## 适用范围

main 在上述各自 CPU 条件下通过对应原计时门槛，全部 CPU 结果仍未通过。当前来源版本的旧对照独立保存在 `.scratch/map147-152/perf-diagnosis.md`，本次 main 结果以 `integration/full/` 为准。完整项目周期和完整计算组合由 #119、#144/#169 继续验收。最终来源与 main 文件 SHA256、安装及覆盖证据身份保存在 `integration/final-files.json`。本任务未执行规范提升、提交、推送、PR、issue 关闭或外部写入，完成当前集成后停止。
