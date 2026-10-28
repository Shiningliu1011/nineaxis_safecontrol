# 当前执行与验收

## 2026-10-28 圆柱拟合异常真实数据回归

本次基点为 `b77e83718f6f7baaf0f0ce030bc0bd812b26538f`，开始时工作区干净。用户授权补充圆柱拟合异常分支的真实数据回归，验证失败后轨迹、投影记录和连续缓存保持不变，并将本次测试及相关文档提交到 `main`、推送到 `origin/main`，完成同步后停止。本次范围为测试及对应说明和验收记录。

`test_cylinder_fit_failure_preserves_loaded_state` 在真实 MAT 的 `production` / `s0` 两种当前圆柱上建立显式连续参考及 geometry。只改变首次投影点集的重新拟合输入，构造空点集、有限坐标放大后的平方溢出、保留轴向数据的零半径退化，分别执行实际 NumPy 拟合或拟合结果检查。六种组合使用公开 `set_surface_normal_orientation()` 触发失败，检查全部14992个位置、速度、加速度、旋转、角速度和 jerk，以及圆柱字段、投影记录、显式目录、bundle 与 geometry。恢复原拟合点集后，检查合法重设的连续身份及几何。

- [x] 添加三种真实拟合失败条件和两种圆柱的回归用例。
- [x] 运行真实资产测试文件并核对全部结果。
- [x] 完成文档及工作区差异检查，保留本轮证据。

环境及本轮验证证据使用已忽略的 `.scratch/cylinder-fit-regression-20261028/`，既有审查与修复证据继续保留。实现与参考资产沿用当前版本，规范沿用既有失败状态保持要求。

在设备 `lsn` 的 Python 3.10.12 / NumPy 1.26.4 环境执行：

```bash
source .scratch/cylinder-fit-regression-20261028/env.sh
python -m pytest -q tests/test_butterfly_reference_assets.py --basetemp=.scratch/cylinder-fit-regression-20261028/regression-data --junitxml=.scratch/cylinder-fit-regression-20261028/regression.xml
python -m py_compile tests/test_butterfly_reference_assets.py .scratch/cylinder-fit-regression-20261028/verify_results.py
bash -n .scratch/cylinder-fit-regression-20261028/env.sh
python .scratch/cylinder-fit-regression-20261028/verify_results.py
git diff --check
```

真实资产测试为 28 passed（209.03 s），退出 0。JUnit 确认其中六项为新增拟合异常组合，全部28项无失败、错误或跳过。空点集在实际拟合运算中触发 `ValueError`；有限坐标乘 `1e160` 后，在 `np.errstate(over="raise", invalid="raise")` 下触发平方溢出的 `FloatingPointError`；保留真实轴向坐标的退化点集正常完成最小二乘计算，零半径结果由现有检查拒绝。六项均通过完整样本、数组对象、投影记录、连续身份与缓存保持检查，以及原拟合输入恢复后的合法重设检查。

编译、shell 语法、AST 和差异检查通过。验证脚本退出 0，核对两种真实资产各14992个来源点、MAT SHA256、28项JUnit与六项新增场景，并将代码及测试SHA256记录在 `verification.json`；完整测试日志为 `regression.log`。`git diff --exit-code HEAD -- portable_oscbf/work/ik_data_loader.py data/nurbs/butterfly_reference_v1` 退出 0。

本轮验证范围为真实资产模块及上述静态检查，未运行全量测试、性能矩阵或设备测试。规范沿用既有要求，没有新增规范决定。本次测试、说明与验收记录共同纳入 `main` 提交与 `origin/main` 同步范围，完成同步并核验分支身份后停止。

## 2026-10-28 参数检查与显式参考来源修复

本次基点为 `8089998d9c937694b39ef5a0653a9d58ccb6ef33`，开始时工作区干净。用户授权修复本次审查的两项 P2，补充真实数据回归和相关文档，并将本次改动提交到 `main`、推送到 `origin/main`，完成同步后停止。其余地图任务保持原状态。

测试继续使用既有公开接口 `load_repository_trajectory()`、`set_surface_normal_orientation()`、`set_orientation_mode()`、`attach_continuous_reference()`、`path_geometry(require_continuous=True)` 和全部原始任务参考。期望值采用成功加载后的真实 S0 参考及正式生成器产生的版本身份。

- [x] 输入参数检查和圆柱拟合在更新当前轨迹前完成；无效参数调用后，14992 个原始运动样本、连续身份及几何保持一致。
- [x] 保存显式连接的参考目录，重复设置或经过 `fixed` 恢复后重新验证并采用匹配版本。
- [x] 核验配置改变、恢复原配置及选定产物损坏时的行为，保持自动发现、production/S0 和安装资源验证。
- [x] 完成相关 NumPy/JAX 与公共消费者回归，更新 `docs/butterfly_reference.md` 及当前审查记录。

实现仅涉及 `portable_oscbf/work/ik_data_loader.py`，测试放在 `tests/test_butterfly_reference_assets.py`。候选版本采用同一完整加载检查，显式来源在自动发现其他配置时保留。当前环境、失败测试、通过测试及最终日志独立保存在已忽略的 `.scratch/reference-state-fix-20261028/`，保留已有审查与验证证据。

以下命令均在仓库根目录加载 `.scratch/reference-state-fix-20261028/env.sh` 后执行。TDD 两个行为的命令及结果：

- `python -m pytest -q tests/test_butterfly_reference_assets.py::test_invalid_cylinder_parameters_preserve_the_loaded_reference`：`--basetemp` 分别使用本轮目录的 `red-parameters-data` / `green-parameters-data`；结果为 1 failed（退出 1，7.79 s）及 1 passed（退出 0，7.02 s），日志为 `red-parameters.log` / `green-parameters.log`。
- `python -m pytest -q tests/test_butterfly_reference_assets.py -k explicit_reference_remains_available`：`--basetemp` 分别使用 `red-explicit-data` / `green-explicit-data`；结果为 2 failed（退出 1，16.72 s）及 2 passed（退出 0，18.41 s），均有 17 deselected，日志为 `red-explicit.log` / `green-explicit.log`。

最终相关回归命令：

```bash
python -m pytest -q tests/test_butterfly_reference_assets.py portable_oscbf/tests/test_continuous_path_reference.py --basetemp=.scratch/reference-state-fix-20261028/reference-regression-data --junitxml=.scratch/reference-state-fix-20261028/reference-regression.xml
python -m pytest -q tests/test_trajectory_transform_unity.py tests/test_static_experiment.py::test_s0_uses_declared_geometry_and_complete_path portable_oscbf/tests/test_trajectory_loading.py portable_oscbf/tests/test_jax_path_following.py portable_oscbf/tests/test_path_posture_reference.py portable_oscbf/tests/test_path_precision_initialization.py --basetemp=.scratch/reference-state-fix-20261028/consumer-regression-data --junitxml=.scratch/reference-state-fix-20261028/consumer-regression.xml
```

两组分别为 61 passed（219.59 s）和 26 passed（29.01 s），均退出 0，无失败、错误、跳过或未选中用例。检查包括十种无效输入后的完整样本保持、显式来源重复设置与 `fixed` 恢复、其他圆柱版本的自动选择及原显式版本恢复、来源 SHA256 损坏与 manifest 缺失拒绝。真实生成的默认与非默认变换版本各核验 14992 个来源点，全部运动样本及连续身份采用公开接口检查。

Standards 独立静态复核通过。Spec 独立真实探针执行 `source .scratch/reference-state-fix-20261028/review-spec/env.sh && python .scratch/reference-state-fix-20261028/review-spec/check_state_fix.py`，退出 0，八项检查通过，结果为 `review-spec/state-fix-results.json`。两方面未发现新的确认问题。

Python 编译、AST 与 `git diff --check` 通过。`python .scratch/reference-state-fix-20261028/verify_results.py` 退出 0，核对两份 JUnit、生成版本的来源验证和既有连续资产未改动，保存代码及测试 SHA256 至 `verification.json`。使用说明已核对当前接口与行为；规范沿用既有输入检查及身份规则，本次没有新增规范决定。

两项 P2 的修复、真实数据回归、文档与独立复核完成。本次没有运行全量测试、性能矩阵或设备测试。修复、测试和相关文档共同纳入 `main` 提交与 `origin/main` 同步范围，完成同步并核验分支身份后停止。

## 8089998 路径参考修复与同步

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

2026-10-28在设备`lsn`完成圆柱拟合异常分支的真实数据回归。三种失败条件与 `production` / `s0` 组成六项新增回归，检查失败后全部14992个运动样本、投影记录和连续缓存保持不变，并验证原拟合输入恢复后的合法重设。当前真实资产模块28项通过，命令、JUnit、来源身份与验证范围见上方专节及`research/review.md`。

既有审查、修复及验证证据继续保留，本轮证据独立保存在`.scratch/cylinder-fit-regression-20261028/`。各目录按现有规则保留在设备`lsn`；跨设备接手使用仓库内本记录与`research/review.md`，需要原始证据时从该设备获取。本次基点为`b77e83718f6f7baaf0f0ce030bc0bd812b26538f`，新增测试、说明与验收记录共同纳入 `main` 提交与 `origin/main` 同步范围，完成同步并核验分支身份后停止。
