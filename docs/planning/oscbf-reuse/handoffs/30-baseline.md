# [B0] 可复现基线与证据目录

- Tracker：https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/30
- 日期：2026-09-09；执行者：Codex（GitHub assignee 为当前认证用户）
- 工作目录：`/home/lsn/robot/robot_safecontrol`
- 状态：已解决（基线与证据交接完成；运行链路仍有已知缺口）。

## 起点与恢复

本机 branch 为 `main`，HEAD 与本次 `git ls-remote origin refs/heads/main` 均为 `659da6c6db598abddc272ad0941b72f77793211b`。采集时 tracked diff 为空；439 个未跟踪文件属于 `.scratch/oscbf-reuse-wayfinder/MAP.md` 和 `大然电机/`，并非远端提交内容。未提交、重置或丢弃既有文件。

证据根目录：`/home/lsn/robot/robot_safecontrol/output/baseline-30/`（gitignored，本机交接资产，尚未上传；跨主机交接必须另复制该目录）。

- `tracked.tar`：HEAD 的 tracked 文件。
- `working-tree.patch`：采集时相对 HEAD 的二进制 patch（本次为空）。
- `untracked.tar.gz`、`untracked.list`、`status-before.txt`：原有未跟踪文件与清单。
- `local-runtime-history.tar.gz`：本机 install/build 与当时 output 根目录历史资产。构建产物含原路径 symlink，不能作为跨主机可移植安装包。
- `archives.sha256`：归档 SHA256；`tracked-verify.json` 与 `untracked-verify.*`：归档与采集工作区比对结果。
- `inputs-history-manifest.json`：config/models/data/benchmarks 与 output 根目录文件的大小和 SHA256。

先在证据目录执行 `sha256sum -c archives.sha256`。恢复到新的独立 checkout：checkout 上述 SHA，在新 checkout 中解压 `untracked.tar.gz`；patch 非空时再 `git apply --binary`。也可向空目录解压 `tracked.tar` 获得无 Git 元数据的源文件副本。不要向活跃工作区覆盖解压。原机器测量复用现有 install；跨机器依照 CLAUDE.md 构建并核对实际依赖清单，不盲目沿用其中示例版本。配置和数据需与 manifest 一致。

归档指纹（用于交接时确认资产身份）：

```text
c826370d737960e24833bd4c89b147f888f6be2713214e85db948109c45f932c  local-runtime-history.tar.gz
bc1e9aa2cbc4c16834b6ac1ff44fbb6480c649b08d30abbc7df9e859f2908fa5  tracked.tar
3cc61512f5a1efc6568791c6920f34feb19be679768bb04036713449a22f78d5  untracked.tar.gz
```

## 环境与复用

以本次包元数据和 dpkg 查询为准（`python-dependencies.json`、`pip-freeze.txt`、`dpkg.txt`、`ros-python.txt`、`ros-licenses.txt`）：

| 组件 | 实际版本 | 本机元数据许可证 |
|---|---|---|
| Python | 3.10.12 /usr/bin/python3 | 系统包记录 |
| ROS rclpy | Humble / 3.3.21 | Apache-2.0 |
| pymoveit2 | 2.1.0 | BSD |
| JAX / jaxlib | 0.6.2 / 0.6.2 | Apache-2.0 |
| qpax | 0.1.4 | Apache-2.0 |
| CBFpy | 0.0.1 | MIT |
| numpy / scipy | 1.26.4 / 1.15.3 | BSD，附带组件条款见原始元数据 |
| MuJoCo | 3.12.0 | Apache-2.0 |
| python-fcl | 0.7.0.11 | BSD |
| trimesh | 5.1.0 | MIT |
| pytest | 6.2.5 | MIT |
| python-can / osqp | 当前解释器缺失 | 未安装，不作版本冻结 |

注意实际 pymoveit2 为 2.1.0，与 CLAUDE.md 安装示例中的 3.2.0 不同；以本机安装证据为测量基线，不在本票升级。完整系统/CPU/GPU 信息见 `os-release.txt`、`kernel.txt`、`cpu.txt`、`gpu.txt`。NVIDIA 驱动 610.43.02，RTX 3060 Laptop 6 GiB；nvidia-smi 成功不等于 JAX GPU 后端可用。

本票复用 git、tar、sha256sum、pip/dpkg 和既有 pytest 入口，无新增控制算法或产品依赖，无替换收益待决策。包版本清单是同机复现记录，不是包含 wheels、apt 镜像和驱动的离线环境镜像。

## 历史与输入证据

`data/nurbs/` 提供轨迹输入，`output/baseline_tracking.npz` 和 `output/transition_path.npy` 为已有回放/路径资产；`benchmarks/` 为已有规划测量资料。只登记其存在和 hash，未将其当作双传感器实采证据。

历史 `output/oscbf_m10_perf_isolated_final.md` p95=9.287ms、`output/oscbf_m10_perf_isolated_ac.md` p95=13.322ms 属于旧工况，**不是本次性能复测**，不能据此通过当前延迟预算。后续性能票应重新记录电源、热状态、并发负载、预热和数据搬运。

## 本次验证

- `bash run_all_tests.sh`：exit 1，`/opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable`。入口问题复现，归「run_all_tests.sh 在 set -euo pipefail 下直接退出」。
- 从普通 bash（未启用 nounset）执行以下命令，主包 **250 passed in 142.51s**，exit 0。旧票“241 passed / 4 failed”不是当前结果；当前 perf 测试已显式启动 tracking，源码已有修复。未在本票关闭其他修复票。

```bash
cd /home/lsn/robot/robot_safecontrol
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest tests/ -q
python3 -m pytest portable_oscbf/tests -q
```

主包本次产生的性能报告单独保存为 `main-suite-perf.md`：201 步，p95=8.611ms，最大9.531ms，>20ms为0。报告同时记录100Hz控制频率与50Hz/20ms预算，二者不可混为同一验收目标；这是测试上下文的局部计时，不是负载受控的端到端性能验收。该运行曾与快照归档短暂重叠，未测电源/热稳态，后续性能票必须独立复测。

portable：**146 passed, 34 skipped, 1 failed in 610.04s**，exit 1。失败为 `test_jax_tool_axis_tracking.py::test_tool_axis_path_kernel_ignores_roll_only_reference_at_path_start`，`err_6d` 最大绝对差 `2.72953804e-08`，超过 `atol=1e-8`；对应原「portable 测试容差回归修复」票。这里只复现，不据此独断语义正确或修改容差。

34 个跳过不是通过；源码跳过声明索引见 `skip-declarations.txt`，包括旧 newaxis 依赖用例；本次未逐项恢复这些覆盖。日志提示 CUDA-enabled jaxlib 未安装，回退 CPU。完整日志为 `tests-main.log`、`tests-portable.log`，对应 `.exit` 文件给出退出码。

归档验证：351 个 tracked 文件内容/链接与工作区一致；439 个原有 untracked 文件经 `tar -d` 比对 exit 0。测试会更新 ignored output 报告；旧副本在历史归档，新副本单列，差异见 `generated-output-changes.json`。

## 决定、改动与编码门

用户本轮指令：“开始第一个ticket”。没有新增架构取舍；继承地图已有九轴/5D、OBB、复用择优和故障行为决议。

新增本交接、`TEMPLATE.md` 和本地证据归档，向本地与正式地图追加本票结论索引；未修改运行代码或控制参数，未启动真机。后续可以使用这里固定的起点开展离线核验和测量。测试工具缺陷修复归原票，依赖安装及执行链路调整归对应票；不以本票证明生产链或实机准入。

## 回退与下一窗口

本票无运行行为变更，无需控制回退。保留本地证据目录直至交接完成；若要撤销文档，只处理本票新建 handoff 文件，不清理原有 untracked 资料。

下一窗口：[真机执行链路：SocketCAN 后端与参数注入](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13)。先核对 `python-can` 缺失和设备资料，按原票范围进行离线协议/接口核验。入口脚本与测试缺陷仍保留其原票身份，必要时先处理阻碍测量的部分。本窗口不自动开始下一票。
