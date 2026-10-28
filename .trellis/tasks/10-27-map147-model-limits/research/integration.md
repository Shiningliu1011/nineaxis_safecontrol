# #153 主工作区集成验收

## 来源与文件范围

集成工作区为 `/home/lsn/robot/robot_safecontrol`，分支 `main`，HEAD 为 `9c354d5505e26aa28b3e6fc323c661bd7542b105`。实现来源为 `.scratch/worktrees/map147-model-limits`，相同 HEAD，分支 `codex/map147-model-limits`。

独立 merger 在编辑前核验 `.scratch/map147-153/review-handoff-fixed.json` 与 `review-source-sha256-fixed.txt` 中全部 22 份文件，SHA-256 全部一致。Spec 复核和 Standards 审查均无发现，记录分别为 `.scratch/map147-153/review-spec-fixed.md` 和 `review-standards.md`。

源码、测试、两份配置、模块文档及八份 Trellis 记录全部通过 `apply_patch` 集成。`docs/modules/portable_oscbf.md` 增加本模块入口并保留现有 LiDAR 索引。`static_experiment_nodes.py` 仅在 `_resources` 中传递 `limits["max_jerk"]`。本任务 `implement.md` 登记审查与集成验收；其余交付文件与冻结清单一致。

开始时的 35 份既有修改及目标文件身份保存在 `.scratch/map147-153/integration/before.json`。集成后的全部文件身份、既有修改检查、环境、测试、安装与独立导入结果统一记录在 `integration/final-files.json`。源工作区保持冻结内容。

## 环境与真实测试

所有命令在 main 工作区执行；环境入口为 `.scratch/map147-153/integration/env.sh`，读取既有 `.scratch/host-recovery/env.sh`，将源码路径指向 main，将中间数据、缓存与日志指向本次 `integration` 目录。测试加载本次 main 构建的 `robot_static_fcl`，具体模块路径保存在 `integration/runtime-environment.json`。

实际环境为 Python 3.10.12、JAX/JAXLIB 0.6.2、NumPy 1.26.4、SciPy 1.15.3、MuJoCo 3.16.0、Ruckig 0.12.2、pytest 8.3.5、jsonschema 4.23.0，设备为 CPU。

`bash .scratch/map147-153/integration/run_tests.sh` 运行模型、限制、逐轴 Ruckig、实际资源消费者及既有运动/执行六份测试文件，结果为 **79 passed，52.28 s，退出码 0**。完整命令保存在脚本；原始输出为 `integration/pytest-main.log`，逐项结果为 `integration/pytest-main.xml`。没有跳过项目。现有环境发出一条 Matplotlib Axes3D 提示。

`bash .scratch/map147-153/integration/run_static_checks.sh` 运行新增模块和测试的 Ruff、资源文件的 F821、三个内核模块的 mypy、受影响执行文件语法及 `git diff --check`，全部退出码 0。日志分别为 `ruff.log`、`resource-names.log`、`mypy.log`、`compileall.log`、`diff-check.log`。

`python3 .scratch/map147-153/integration/check_documents.py` 使用 Markdown 解析器核对两个模块页面的 **18 个本地链接**；Trellis 的 implement/check context 全部指向现有文件，退出码 0。

## 隔离安装与实际导入

`bash .scratch/map147-153/integration/build_install.sh` 对 main 的 `robot_safecontrol_moveit` 执行独立 colcon build/install，路径为 `integration/build` 和 `integration/install`，退出码 0；构建记录为 8.67 s，colcon 总时间为 9.12 s。完整输出为 `integration/build-install.log`，构建产生现有 byte-compiling 和系统 pythran 提示。

`bash .scratch/map147-153/integration/run_installed_checks.sh` 检查三个内核模块、两份配置、package.xml 和资源消费者共 **7 份源码/安装副本 SHA-256**，全部一致。原始清单为 `integration/installed-files.json`，包含实际 native extension 路径与身份。

同一脚本分别对 main 和安装后的 package share 使用 `python -I -B` 真实导入。URDF/SRDF 解析库来自既有独立提取的官方发行包，搜索路径未加载系统 `/opt/ros`；`rclpy/rospy/ament` 均未导入。两次实际读取模型、SRDF 排除项、M07，启用 float64 并计算 FK。模型、资产、几何、配置、绑定、关节位置、FK 和验证范围等 **11 项字段完全一致**，记录为 `parser-isolation-source.json`、`parser-isolation-installed.json` 与 `isolated-comparison.json`。

共同模型 hash 为 `4e845d44abb632786cc4f0226d431fd9c97be1abfd5e98721bc17560cf592f0c`，未标定资产 hash 为 `176e05ee765c50ec97d81365e4034d0f3be45de46dc64e8ce9deea91116e3a57`，M07 hash 为 `7dc37cb81cd55dd48b531e8918cd1b49126a94d4318242e61276482d80e1179c`，与冻结清单相同。

## 验证范围

本次证据覆盖研究模型、实际工具数值导数、M07 样本和阶段检查、真实 Ruckig 完整关节运动及停止、实际消费者与安装入口。`hardware_validated=False`，真实工件、关节映射、TCP 和附加工具几何保持缺项。

全项目 `run_all_tests.sh` 由主会话在 #152/#105 等集成后统一安排。当前未运行设备动作或完整 Runtime，未修改安全门槛。main 保持未提交，没有 commit、push、PR、外部写入或任务关闭操作。
