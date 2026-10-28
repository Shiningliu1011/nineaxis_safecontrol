# 当前执行与验收

## 2026-10-28 路径参考修复

本次在 `main` 的 `930b582a447d29af8e289bdd24c6ae5ed255abe6` 上继续处理审查记录的两项 P2。开始时工作区干净。用户已明确授权将本次修复、测试和相关文档提交到 `main`，并推送到 `origin/main`；完成同步后停止。

测试边界采用既有公开接口 `generate_reference()`、`load_repository_trajectory()`、`set_surface_normal_orientation()`、`set_orientation_mode()`、`attach_continuous_reference()` 和 `path_geometry(require_continuous=True)`。使用正式生成器和真实蝴蝶来源文件，期望身份来自生成包的 manifest，位置、姿态和运动参考与相同参数的新加载结果核对。

- [x] 自动发现：实际生成默认版本及 `ee_center=[0.0,0.344,1.387]` 版本，验证相同来源与半径下选择匹配变换的版本，并保持直接连接的身份拒绝。
- [x] 姿态重新设置：验证 production/S0 重复设置、`surface_normal→fixed→surface_normal`，以及圆柱参数改变后的当前两次投影、连续身份及运动参考。
- [x] 修改 `portable_oscbf/work/ik_data_loader.py` 中的候选身份检查与当前圆柱投影状态；回归放在 `tests/test_butterfly_reference_assets.py`。
- [x] 运行 NumPy/JAX 连续参考、公共路径消费者和安装资源检查，更新 `docs/butterfly_reference.md` 与当前任务审查记录。

候选筛选与显式连接共用 `_continuous_reference_mismatch()`，共同检查模型、变换和全部投影参数；选定版本仍执行完整文件、来源及连续表达验证。姿态设置保存并复用首次投影完成的位置、速度、加速度与 jerk，重新构建当前第二次投影及旋转参考。连续包和路径缓存随姿态设置更新。

独立环境和所有新日志保存在已忽略的 `.scratch/reference-p2-fix-20261028/`。以下命令均从仓库根目录加载该目录的 `env.sh` 后执行。

TDD 的两个行为分别完成失败与通过验证：

- `python -m pytest -q tests/test_butterfly_reference_assets.py::test_reference_discovery_selects_the_matching_transform`：使用独立 `red-discovery-data` / `green-discovery-data` 作为 `--basetemp`，依次为 1 failed（退出 1，13.90 s）和 1 passed（退出 0，10.43 s）；日志为 `red-discovery.log` / `green-discovery.log`。
- `python -m pytest -q tests/test_butterfly_reference_assets.py -k surface_normal_reset`：使用独立 `red-orientation-data` / `green-orientation-data` 作为 `--basetemp`，依次为 8 failed 和 8 passed，各有 5 deselected；退出码依次为 1 / 0，耗时 38.62 s / 84.27 s；日志为 `red-orientation.log` / `green-orientation.log`。

最终相关回归命令：

```bash
python -m pytest -q tests/test_butterfly_reference_assets.py portable_oscbf/tests/test_continuous_path_reference.py --basetemp=.scratch/reference-p2-fix-20261028/reference-regression-data --junitxml=.scratch/reference-p2-fix-20261028/reference-regression.xml
python -m pytest -q tests/test_trajectory_transform_unity.py tests/test_static_experiment.py::test_s0_uses_declared_geometry_and_complete_path portable_oscbf/tests/test_trajectory_loading.py portable_oscbf/tests/test_jax_path_following.py portable_oscbf/tests/test_path_posture_reference.py portable_oscbf/tests/test_path_precision_initialization.py --basetemp=.scratch/reference-p2-fix-20261028/consumer-regression-data --junitxml=.scratch/reference-p2-fix-20261028/consumer-regression.xml
```

两组分别为 55 passed（184.18 s）和 26 passed（28.53 s），均退出 0，无失败、错误、跳过或未选中用例。真实生成的两个配置各验证 14992 个来源点。production/S0 的八种重复设置组合均连续重设三次，核验全部 14992 个位置、旋转、速度、加速度、角速度和 jerk；改变轴向、中心及半径后的恢复与全新加载结果一致。错误模型、缺少匹配配置与选定产物损坏的拒绝检查通过。

`python -m py_compile portable_oscbf/work/ik_data_loader.py tests/test_butterfly_reference_assets.py`、AST 与 `git diff --check` 通过。`python .scratch/reference-p2-fix-20261028/verify_results.py` 退出 0，核对两份 JUnit、生成版本来源验证及既有资源未改动，结果为 `verification.json`。文档已核对公开接口与当前行为，沿用既有单位与身份规则；本次没有新增规范决定。

本次没有重新运行全量测试、性能矩阵或设备测试。两项 P2 的修复与相关验收完成，修复代码、回归测试、使用说明及两份 Trellis 记录共同纳入 `main` 提交与 `origin/main` 同步范围。同步结果通过本地提交和远程分支身份核验，其余地图任务保持原状态；完成后停止。

## 执行规则

- 按原生未阻塞入口领取任务；决定先形成可审阅方案，满足票面确认条件后更新记录和任务图。
- 实现agent使用独立工作区和真实公开接口TDD，完成相关测试、文档、安装及来源核验后冻结。
- Spec与Standards分别独立检查；有问题返回实现修订，通过后由merger使用apply_patch集成。
- main运行受影响验证；广泛集成运行完整测试入口，保存所有失败、跳过及条件。
- 达到本票实际完成条件后关闭；全项目结束前保留#147和相应现场任务。未经授权不提交、推送、创建PR或执行设备动作。

## 本轮完成范围

用户要求完成当前并行工作后停止。#152/#105/#101的实现、独立审查、main集成与实际验收均已完成并关闭；#108和#153的完成状态继续保持。本轮#150研究已保存并发表，完整约束、选型及周期仍待测量，任务保持开放。后续任务未启动。此前并行交付已按其授权同步；两项路径参考的本地修复和验收见上方专节。

#136/#137/#138/#139/#140/#141/#142/#143/#144/#149/#151已确认并完成，完整规则及独立确认已发表。#144确认评论6443398104规定每效率场景每方法30次、保留全部失败及配对统计；每方法至少10000完整周期，p99.9使用20%余量，具体周期、容量与正式数据由#169提供。

## 实现与验收证据

- #108：main272项受影响回归及独立双线审查通过。
- #153：79项相关检查、隔离安装、7份资源hash、独立模型探针及双线审查通过；完成评论6442936431。冻结集成清单为`.scratch/map147-153/integration/final-files.json`。
- #152：main重新生成production和S0连续包，各14992原样本与数值产物独立核验通过，90项直接消费者均有实际通过记录；完成评论6445339517。最终集成清单45文件/17证据，SHA256为`86f28da54df3ccae0f4df1403d399026ddafc847b0421ed5ba91514d661e3857`。
- #105：main311项直接测试、真实解码错误的独立探针、组合构建及安装导入通过；完成评论6445352584。最终集成清单21文件/17证据，SHA256为`51c1f52af23ee08d07b908317e6fdf23782909aeeec0dc118dcb3573c3a44d57`。
- #152/#105阶段完整覆盖：main854收集项中847项功能通过、6项原条件跳过；kernel741项中706项功能通过、34项原条件跳过；另两项计时分别按下述CPU条件通过，AEB4项通过。全部原始失败、复验、nodeid和跳过原因保存在`.scratch/map147-152/integration/full`。主会话独立来源及覆盖核验为`.scratch/map147-decisions/parent-integration-verification.json`。
- #101：来源清单`.scratch/map147-101/review-fix3-handoff.json`包含22来源/25证据，SHA256为`4fee0f00584babac2628b516c16b96fa75fecdcede05dde8afeac82e81aa0206`。工作区278项实现检查及26项安装公开检查通过；独立Spec13项通过且无发现，Standards增量无发现。
- #101 main：456项相关消费者、30项独立及组合检查、43项实际安装接口全部通过，无skip/deselect/当前失败；36份实际安装文件与main一致。最终清单23份main文件、43份证据及76份保留身份，SHA256为`f73ac2871556604e70ab70c01f4af739de0bec7d1aebe7684ff41b2781979d42`。主会话独立核验全部原始JUnit、当前测试来源和安装文件，结果为`.scratch/map147-101/parent-final-check.json`。
- #101组合保留：#152的44份、#105的18份及#153的13份既有文件字节相同；四个LiDAR公共class和七个方法保持#105 AST，OccupancyGrid与已审#101来源一致。证据为`.scratch/map147-101/parent-preservation-audit.json`。
- #150研究：`docs/planning/nineaxis-full-project/mpc-solver-research.md`及评论6443986117；36个冻结文件与七份报告的生产/辅助源码来源经主会话核验，全文readback一致。研究结果限定于其记录的源版本和测量范围。

各完成评论均已全文readback，#101/#105/#152随后按completed关闭并读取状态确认。独立双线审查记录见`research/review.md`，各任务完整命令、日志及安装路径见对应Trellis的`research/integration.md`。

## 计时与验证范围

controller四个P-core条件通过原门槛：303步、p95=14.342699509870727ms、1/303超期（0.330033%）。modular八个P-core条件为3000步、6.379ms、初始化27.36s、数值差异为零、缓存1，通过原120%门槛。

全部CPU条件仍未通过：controller p95=28.082ms、229/300超期；modular14.938ms。controller八个P-core为p95=18.511ms、4/300超期，也未通过。所有条件及原失败保留，baseline SHA始终为`40bdab1e26b1594a227e851e93cacac1856591969cec704076456ccd415a916f`。

原9ms静态个案完整纳入#101 main456项检查并在四P-core条件通过。历史8.349961508s超期原因未确定，原始记录保留；没有取得普遍实时、完整项目周期、物理停止或现场准入结论。当前动态layout使用正数host monotonic ns，共同物理时间和完整连续证明仍由对应任务完成。

## 最终任务图与停止状态

真实原生快照为`.scratch/map147-decisions/frontier-after-101-105-152.json`：59项主线、179条依赖、41项开放、32项有开放前置；图无循环且原生摘要一致。#164/#104/#112/#113/#155/#156/#159均待领取；本轮没有启动这些任务。#95和#150保持现有指派，#145继续等待#150。

此前任务图检查入口为`python3 .scratch/map147-decisions/check_proposals.py`、`python3 .scratch/map147-decisions/check_frontier.py --snapshot .scratch/map147-decisions/frontier-after-101-105-152.json`及`git diff --check`。本次路径参考修复的授权、验证和停止条件见上方专节。

集成核验：15份文档结构、96个本地链接、九轴范围及21项GitHub完整正文/状态检查通过；Trellis implement/check各三条context全部通过，`git diff --check`退出0。集成及本轮审查基准为`9c354d5505e26aa28b3e6fc323c661bd7542b105`，全部子agent与数值进程已经结束。

## 审查与跨设备接手

2026-10-28在设备`lsn`完成两项路径参考 P2 的本地修复。自动发现与连接共用配置身份检查；姿态设置维护首次投影和当前圆柱投影。当前实际通过 81 项相关回归，命令、JUnit、来源身份与验证范围见上方专节及`research/review.md`。

本地原始审查数据保留在`/home/lsn/robot/robot_safecontrol/.scratch/pr-review-20261028/`，完整报告为`review.md`，冻结范围为`scope.json`。本次修复证据独立保存在`.scratch/reference-p2-fix-20261028/`。两个目录均按现有规则保留本地；跨设备接手使用仓库内本记录与`research/review.md`，需要原始证据时从该设备获取。修复基点为`930b582a447d29af8e289bdd24c6ae5ed255abe6`，本次修复、测试和相关文档共同纳入 `main` 提交与 `origin/main` 同步范围。
