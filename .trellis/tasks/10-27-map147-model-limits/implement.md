# 实施与验证

## 已审阅范围

已完整读取 #153 当前正文、总体规格与主仓库 #136/#138/#141 决定。用户已确认全部需求决定，主会话核验原生前置清零。开始前 branch 为 `codex/map147-model-limits`，HEAD/main/merge-base 为 `9c354d5505e26aa28b3e6fc323c661bd7542b105`，工作树干净。项目 TDD、Trellis、内核及质量规范已经读取。

## 公开受测接口

- `NineaxisModel.load`、`fk`、`link_poses`、`jacobian`、`tool_motion`、`source_pose_to_base`：模型、标定、单位与实际导数。
- `ProcessLimits.load`、`bind`、实际/参考工艺测量和阶段检查：有来源的限制及有效性。
- `MotionPlanner.from_limits`、`propose` 与真实 `CandidateMotion`：共同逐轴 jerk 的消费与完整停止。

## 检查清单

- [x] 审阅需求、来源和现有消费者；保存规划与 context。
- [x] 当前 `task.json` 为 `in_progress`；继续既有 Trellis 任务。
- [x] 模型身份/输入与真实 FK 公开接口 red/green。
- [x] 工具一至三阶运动及独立差分 red/green。
- [x] M07 参数、来源、单位、身份和缺项 red/green。
- [x] 实际工艺量及阶段限制 red/green。
- [x] 明确逐轴 jerk 的真实 Ruckig red/green。
- [x] 同步模块文档并执行受影响回归、lint、type、完整范围检查。
- [x] 按 trellis-check 审阅完整范围，记录误差、全部文件与最新 main 相容性。
- [x] 主会话独立 Spec/Standards 审查及 merger 集成，主工作区 79 项测试通过。

## 隔离环境

所有命令显式位于本 worktree。加载主仓库 `.scratch/host-recovery/env.sh` 后，将 PYTHONPATH 首部、TMPDIR、pytest basetemp、ROS 日志与缓存明确指向本 worktree或主仓库 `.scratch/map147-153`。不启动 ROS graph、viewer、硬件桥或真实 I/O。日志与命令脚本保存在主仓库 `.scratch/map147-153`。

## TDD 记录

所有下列 pytest 命令均在当前 worktree 执行，加载 `/home/lsn/robot/robot_safecontrol/.scratch/map147-153/env.sh`。每次使用独立的 `--basetemp`，完整日志保存在主仓库 `.scratch/map147-153/`。

| 公开行为 | red 证据 | green 证据 |
|---|---|---|
| 模型身份、九轴 FK/link/Jacobian | 既有 `model-red.log` 记录公开模块缺失 | `model-green-restored.log`：12 passed，9.27 s |
| 工具实际前三阶运动 | `tool-red.log`：2 failed，公开 `tool_motion` 缺失 | `tool-green.log`：14 passed，9.61 s；后续扩大为逐轴及联合运动 |
| M07 配置和实际/参考检查 | `limits-red.log`：公开模块缺失 | `limits-green.log`：7 passed，4.06 s |
| 逐轴 jerk 与完整停止 | `motion-red.log`：10 failed，公开逐轴接口缺失 | `motion-green.log`：10 passed，1.28 s |
| 单位和来源版本拒绝 | `identity-red.log`：明确单位字段缺失 | 后续组合验证通过 |
| 新提供的最低加工进给 | `optional-behavior-red.log`：30 mm/s 下限对 20 mm/s 输入的断言失败 | 后续组合验证通过，按阶段检查已配置下限 |
| 完整关节区间和停止 | `full-joint-red.log`：公开区间检查缺失 | `full-joint-green.log`：13 passed，13.21 s，包含真实静态资源消费者 |
| 变更参数的独立身份 | `replacement-red.log`：参数变更后的身份断言失败 | 后续组合验证通过，来源与数值生成新身份 |
| 独立 float64 入口和精度状态 | `precision-red.log`：2 failed，纯 JAX 精度标记及独立进程入口断言失败 | `final-precision-regression.log`：独立进程从关闭 x64 的状态建立共同 float64 参数，后续精度改变时拒绝求值 |
| 参数适用阶段及固定职责 | `review-phase-red.log`：16 failed | `review-phase-green.log`：25 passed，13.17 s；覆盖样本、质量汇总、间距缺项和未实现阶段组合的拒绝 |
| 暂停信号与阶段继续输出 | `review-pause-red.log`：2 failed | `review-pause-green.log`：28 passed，14.88 s；覆盖位置/轴向、配置阶段及入场/恢复/端点条件 |
| 标定适用模型资产身份 | `review-calibration-red.log`：3 failed | `review-calibration-green.log`：30 passed，12.24 s；`review-calibration-final.log` 再次3 passed，0.98 s，验证稳定身份及循环范围拒绝 |

## 最终受影响验证

`python -m pytest portable_oscbf/tests/test_nineaxis_model.py portable_oscbf/tests/test_process_limits.py portable_oscbf/tests/test_research_motion.py tests/test_model_limits_resources.py tests/test_static_motion.py tests/test_static_execution.py -q --basetemp /home/lsn/robot/robot_safecontrol/.scratch/map147-153/review-fixed-regression`

结果为 **79 passed**，44.81 s，退出码 0，日志 `review-fixed-regression.log`。其中新增模型 30 项、M07 28 项、逐轴 Ruckig/完整运动 11 项、真实资源消费者 2 项，共 71 项；既有运动/执行 8 项。

`python -m pytest tests/test_static_motion.py tests/test_static_execution.py tests/test_static_swept_geometry.py tests/test_static_execution_evaluation.py tests/test_static_control.py tests/test_static_recovery.py tests/test_static_velocity_solver.py tests/test_static_sns.py -q --basetemp /home/lsn/robot/robot_safecontrol/.scratch/map147-153/regression-main`

结果为 **57 passed**，132.30 s，退出码 0，日志 `regression-main.log`。该范围包含上述既有 8 项。完整停止和候选身份增加检查后，又由最终组合复核运动/执行入口。

`python -m pytest portable_oscbf/tests/test_fk_matches_urdf.py portable_oscbf/tests/test_tool_axis_task.py portable_oscbf/tests/test_generated_kinematics_data.py -q --basetemp /home/lsn/robot/robot_safecontrol/.scratch/map147-153/regression-kernel`

结果为 **13 passed**，2.26 s，退出码 0，日志 `regression-kernel.log`。`bash scripts/agent_check.sh` 通过编译、纯模块、shell 语法、diff 和 **69 项**纯逻辑检查，最新日志 `agent-check-review-fixed.log`，纯逻辑检查1.34 s。部分静态评价依赖发出已有 Matplotlib Axes3D 提示，相关测试正常通过。

本次按主会话边界没有启动 ROS graph、launch、viewer 或设备。包含 ROS launch 的全仓测试入口由主会话按集成范围执行；本任务不将纯模块证据声明为全项目或设备验收。

## 静态检查和安装

Ruff 检查新增模型、限制、Ruckig 模块及全部新增测试；mypy 使用 `--ignore-missing-imports --follow-imports=silent` 检查三个内核模块。最终日志为 `review-fix-ruff.log`、`review-fix-mypy.log`，均退出码 0。未使用类型忽略或压制注释。静态资源文件只保留 `_resources` 的参数修改，原有 import 布局保持原文件内容；该文件完成语法和未定义名称检查。

完整运动检查的类型标注完成后，`annotations-regression.log` 再次记录运动/执行 **8 passed**，7.24 s。文档使用 Markdown 解析器核对两个页面的 **17 个**本地链接，结果 `document-links.json`；Trellis context 的 implement 7 项、check 6 项验证通过，`git diff --check` 通过。

精度入口使用 JAX 0.6.2 的公开 `config.read("jax_enable_x64")`；最终精度专测 `precision-delivery.log` 为 **2 passed**，4.94 s，包含独立进程和纯 JAX 精度状态。

实际 colcon build/install 目标为 `robot_safecontrol_moveit`，使用专属 `.scratch/map147-153/build` 与 `install`，退出码 0，日志 `build-install.log`。安装后的 package share 使用 `python -I -B parser_isolation.py --root ... --output-name parser-isolation-installed` 真实加载模型/M07、读取 SRDF 排除项并计算 FK。两个解析库来自独立提取的正式 Debian 包；搜索路径和模块来源见 `parser-isolation-installed.json`，`rclpy/rospy/ament` 均未加载。

最终增量安装日志为 `build-install-review-fixed.log`，退出码 0，1.88 s。`installed-files.json` 逐项核对三个内核模块、两份配置、package.xml 和静态资源消费者的源文件/安装副本 SHA-256。独立安装导入同样从关闭 x64 的状态调用共同入口，并记录模型资产身份。

## 数值与来源证据

复现入口为主仓库 `.scratch/map147-153/numerical_evidence.py`，完整结果在 `numerical-evidence.json`，输出在 `numerical-evidence.log`。使用 Python 3.10.12、JAX 0.6.2、MuJoCo 3.16.0、NumPy 1.26.4 和 CPU。

- 27 组逐轴内部/边界附近状态：FK 位置最大分量差 `5.551115123125783e-13 mm`，旋转矩阵 `7.771561172376096e-16`，全部 link 变换 `8.881784197001252e-16`，SI Jacobian `2.3314683517128287e-15`。
- 10 组 TCP 修正后的联合/逐轴时间多项式：变换的一阶、二阶、三阶导数最大分量差分别为 `1.5662471319899396e-13`、`1.184023434852577e-10`、`8.059856110298469e-8`。完整旋转角导数和工具轴导数均在相同独立样本中验证。
- 共同模型 hash：`4e845d44abb632786cc4f0226d431fd9c97be1abfd5e98721bc17560cf592f0c`。
- 未标定模型资产 hash：`176e05ee765c50ec97d81365e4034d0f3be45de46dc64e8ce9deea91116e3a57`。
- 共同 M07 hash：`7dc37cb81cd55dd48b531e8918cd1b49126a94d4318242e61276482d80e1179c`。

## 审查与集成交接

当前 `HEAD` 和 `main` 均为 `9c354d5505e26aa28b3e6fc323c661bd7542b105`。主仓库的未提交依赖与决定通过已授权绝对路径读取；本 worktree 独立实现，没有复制或覆盖主仓库内容。没有 commit、push、PR 或外部写入。

全部22个交付文件及 SHA-256 见主仓库 `.scratch/map147-153/review-handoff-fixed.json` 和 `review-source-sha256-fixed.txt`；当前通用名称也保存相同清单。`docs/modules/portable_oscbf.md` 的集成只增加模块链接；保留主仓库既有 #108 等索引内容。`static_experiment_nodes.py` 只将 `_resources` 的 `jerk_time` 参数改为现有 `limits["max_jerk"]`，保持 #152 的所有文件。

独立 Spec/Standards 审查均无发现，merger 已在 main 完成集成及受影响验证，见 [主工作区集成记录](research/integration.md)。M07 样本数值检查、完整关节运动/停止检查、完整工具/碰撞区间证明及设备能力分别保存范围。后续正式方法接入、参考/执行协调和现场数据由原任务图继续完成。

规范提升审阅已完成：当前接口、参数和验证知识保存在模块文档与任务记录；没有修改 `.trellis/spec/`、CONTEXT 或 ADR。没有用户归档或提交请求，任务保持已验证的本地交付状态。
