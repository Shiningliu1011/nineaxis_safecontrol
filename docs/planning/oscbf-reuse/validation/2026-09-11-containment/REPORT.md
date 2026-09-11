# Containment 后续验证与文档验收报告

> 归档说明：本报告保留 2026-09-11 验收时点的 HEAD、未提交状态及结论。三份文档随后已在 `c3b3f186038ea2c8374f22959f1d3844eb3e8987` 提交并推送；本次按用户要求归档报告及相关测试日志。文中的临时目录是原始证据位置，随附文件已可跨机器读取。`skills-list.json` 仅保留 Retro 条目，其他运行时技能信息未归档。launch diff 仍为未应用提案。GitHub 评论前后快照未随附，相关进展可通过报告中的公开评论链接读取。

## 1. Repository state

仓库 `Shiningliu1011/nineaxis_safecontrol`，路径 `/home/lsn/robot/robot_safecontrol`；branch `main`；HEAD `e4d9a268c0dee49455c0e397c4910958cc166032`。起点 clean，无用户未提交修改。当前仅 CLAUDE.md、README.md、docs/ONBOARDING.md 修改，未暂存、未commit、未push；产品代码与测试未变。

已检查四项提交实际内容：ec8fd9e containment、9a483d7 Retro、cbcec40 agent_check、e4d9a26 reviewer standards。日期2026-09-11。临时证据目录 `/tmp/nineaxis-validation-20260911/` 未上传，可能被清理。

## 2. Validation

### fast-check

`bash scripts/agent_check.sh` 文档前后均exit0。compileall、6项shell语法、staged/unstaged diff和62项pure contracts全部PASS。日志：[修改前](fast-before.txt)、[修改后](fast-after.txt)。shell仅语法检查，没有执行vcan或标定脚本。

### focused hardware containment tests

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=217 python3 -m pytest -q \
  tests/test_hardware_bridge.py tests/test_launch_structure.py tests/test_hardware_contract.py
```

49 passed，exit0，[日志](focused.txt)。验证sim无硬件控制I/O、shadow不收发CAN且无健康反馈、live在硬件控制实体/backend前拒绝、ack不能制造healthy、ROS模式进入entrypoint且只读。

### full suite

`ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=217 bash run_all_tests.sh`，同一HEAD、文档修改前执行。

|套件|passed|failed|skipped|exit|
|---|---:|---:|---:|---:|
|主包|351|0|0|0|
|portable OSCBF|148|0|34|0|

总exit0。[完整日志](full.txt)。Linux6.8、ROS Humble、Python3.10.12、pytest6.2.5、JAX/jaxlib0.6.2、numpy1.26.4、cbfpy0.0.1、qpax0.1.4。CUDA-enabled jaxlib未安装，实际CPU；有既有禁用测试和optional dependency skip，newaxis不可用。没有逐条运行时skip原因列表，不能把34项skip计为通过或硬件验收。

### diff check

起点、完整测试后、文档修改后`git diff --check`均exit0；最终staged diff检查exit0。三文档本地Markdown链接检查PASS。仅文档变化，未重复完整JAX回归。

## 3. Documentation reconciliation

- CLAUDE.md：115行缩至39行；删除过期测试失败与live=CAN发送描述，保留构建/测试/SSOT/tracker指针及最小containment边界；不缓存passed数量，不复制reviewer表格。
- README.md：将测试修订“待实施”和传感器文档“尚未发布”改为当前已纳入仓库的事实；测试指向命令/#13最新验收；明确live仅bridge拒绝，不保证整个launch停止。
- docs/ONBOARDING.md：区分未来真机链路与当前sim inert/shadow noCAN/live disabled；说明run_demo由plant持续发布状态、viewer仅订阅，以及默认launch回放发布的区别；按实际实体记录QoS depth20/5，未进行QoS重构。

## 4. Issue #13

成功新增[进展评论](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13#issuecomment-5630688954)。回读确认正文匹配，issue仍**OPEN**。旧正文/评论保留；新评论明确supersede已被containment修复的旧缺陷，并列出后端、配置、映射、J1传动、标定、反馈、独立watchdog、发送失败、制动和受控真机验收待办。未新建或关闭issue。

## 5. Retro discovery

- `disable-model-invocation: true`、`allow_implicit_invocation: false`均按布尔值解析确认。
- 安装的codex-cli0.153.4实际app-server `skills/list`（cwd为仓库、forceReload）返回retro路径`.agents/skills/retro/SKILL.md`、scope=repo、enabled=true、errors=[]，见[响应节选（仅保留 Retro）](skills-list.json)。这证明loader实际发现，而非仅文件存在。
- 官方[Skills文档](https://learn.chatgpt.com/docs/build-skills)描述repo-local `.agents/skills`由cwd向仓库根发现；本机实际loader响应与之吻合。
- 本会话按用户明确指定路径读取并执行skill，首次报告完成；当前会话技能catalog未列retro，桌面proxy不可用，响应未给policy。因此**原生桌面`$retro`注入成功和普通请求不触发的端到端行为仍未验收**。metadata通过不等于行为通过；整体discovery验收为部分完成。
- 最小后续验证：在目标桌面的新仓库会话对照普通请求与显式`$retro`，记录实际加载路径/注入和report-only行为。未复制到全局目录或改skill配置。

## 6. First Retro result

完整证据与候选见[RETRO.md](RETRO.md)，只报告未apply。

- P0：未发现新的containment regression。
- P1：无足够证据提出新的P1；#13既有真机工作仍未完成。
- P2：RETRO-001 CONTEXT旧ownership/真机定义；RETRO-002 launch整体拒绝语义；RETRO-003桌面explicit-only行为验收缺证据。
- Do not change：保留live禁用、真实反馈门禁、fast-check的小型定位、reviewer/pointer分层；不新增重复CLAUDE规则，不改控制参数/坐标/感知SSOT。
- Remaining conflicts：CONTEXT.md仍把viewer写为publisher、CAN链路缺目标限定；LESSONS_LEARNED历史“实机”措辞未取得物理验收证据，不能证明或否定历史事件。三份本轮文档及#13当前进展已对齐。

## 7. Remaining risks

**Containment已完成并通过本次限定验证；real live implementation未完成；real hardware acceptance未完成。**真实SocketCAN资格、协议差异、设备/接口、映射、J1传动、各轴标定、真实反馈时效、独立watchdog、发送失败传播、真实停车/承重制动及低速准入均未完成。未触碰真实CAN、live、使能、恢复或标定，未配置接口或削弱测试/门禁。

## 8. Launch-level fail-fast recommendation

推荐**B**，只改善用户可见的启动语义。

|方案|优点|代价|
|---|---|---|
|A：bridge失败，其他节点可能继续|保留现有行为，仿真图可继续用于排查；现有bridge仍阻止硬件I/O|用户请求live却看到其他节点运行，整体启动结果不明确|
|B：live在任何进程启动前拒绝整个launch|明确错误、LaunchService返回1、零节点启动|无法用live参数保留其他仿真进程；未来解除containment时需独立审阅此入口|

已检查本机ROS Humble ExecuteLocal/LaunchService/OpaqueFunction源码：普通子进程退出只发ProcessExited，不默认全局shutdown；同步action异常令LaunchService shutdown并返回1。最小设计是在DeclareLaunchArgument之后、所有Node/TimerAction之前增加OpaqueFunction guard。保留bridge自身拒绝，避免直接节点入口绕过。

修改文件提案：`launch/mujoco_transition_final.launch.py`、`tests/test_launch_structure.py`。真实LaunchService下mock全部Node.execute，验证默认/sim/shadow继续调度，live返回1且零启动；临时拟议版本7 passed，exit0，见[日志](proposal-focused.txt)。不以此代替未来正式落地验收。

**以下diff未apply，仓库产品代码和测试不含这些修改。**

````diff
--- a/launch/mujoco_transition_final.launch.py
+++ b/launch/mujoco_transition_final.launch.py
@@ -11,12 +11,18 @@

 from ament_index_python.packages import get_package_share_directory
 from launch import LaunchDescription
-from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
+from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
 from launch.conditions import IfCondition
 from launch.substitutions import LaunchConfiguration
 from launch_ros.actions import Node

 from robot_safecontrol_moveit.moveit_runtime_config import build_moveit_params
+
+
+def _reject_contained_live(context):
+    if LaunchConfiguration("hardware_mode").perform(context) == "live":
+        raise RuntimeError("live disabled by containment; refusing entire launch")
+    return []


 def generate_launch_description() -> LaunchDescription:
@@ -81,6 +87,8 @@
             DeclareLaunchArgument("auto_plan_once", default_value="true"),
             # Show OBB collision envelopes in the MuJoCo viewer.
             DeclareLaunchArgument("show_obb", default_value="false"),
+            # Refuse before any Node or TimerAction can start a process.
+            OpaqueFunction(function=_reject_contained_live),
             # Log startup info.
             LogInfo(
                 msg=f"Starting unified MoveIt transition demo. "
--- a/tests/test_launch_structure.py
+++ b/tests/test_launch_structure.py
@@ -7,10 +7,11 @@
 import tempfile
 import unittest
 from pathlib import Path
+from unittest.mock import patch

 import yaml
-from launch import LaunchContext
-from launch.actions import TimerAction
+from launch import LaunchContext, LaunchDescription, LaunchService
+from launch.actions import SetLaunchConfiguration, TimerAction
 from launch_ros.actions import Node


@@ -104,6 +105,24 @@
             "robot_description": "<robot name='test_robot'/>"
         }
         return module.generate_launch_description()
+
+    def test_contained_live_fails_before_any_process_starts(self) -> None:
+        for mode in (None, "sim", "shadow", "live"):
+            with self.subTest(mode=mode):
+                actions = [SetLaunchConfiguration("start_mujoco_viewer", "false")]
+                if mode is not None:
+                    actions.append(SetLaunchConfiguration("hardware_mode", mode))
+                actions.append(self._description())
+                service = LaunchService()
+                service.include_launch_description(LaunchDescription(actions))
+                # Never start ROS processes or hardware, including on regression.
+                with patch.object(Node, "execute", return_value=[]) as start:
+                    result = service.run()
+                self.assertEqual(result, 1 if mode == "live" else 0)
+                if mode == "live":
+                    start.assert_not_called()
+                else:
+                    self.assertGreater(start.call_count, 0)

     def test_final_launch_creates_the_expected_node_topology(self) -> None:
         description = self._description()
````

## 9. Final diff

`git diff --stat`

```text
 CLAUDE.md          | 138 ++++++++++++-----------------------------------------
 README.md          |  18 +++----
 docs/ONBOARDING.md |  94 +++++++++++++++++++-----------------
 3 files changed, 89 insertions(+), 161 deletions(-)
```

`git diff`（仅本轮三文档，没有既有用户修改）

````diff
diff --git a/CLAUDE.md b/CLAUDE.md
index 0157fec..3d2b48b 100755
--- a/CLAUDE.md
+++ b/CLAUDE.md
@@ -1,115 +1,39 @@
 # robot_safecontrol

-## Tech Stack
+ROS 2 Humble / MoveIt2 / MuJoCo 的九轴仿真闭环，控制内核位于 `portable_oscbf/work`。

-- ROS 2 Humble（ament_python 主包 + 嵌套 ament_cmake OMPL 插件）
-- Python 3：rclpy、numpy、scipy、mujoco、jax 0.6.2、qpax、pymoveit2
-- C++：OMPL / MoveIt2 插件 `aeb_rrtstar_ompl`
-- 控制内核：纯 JAX OSCBF（Morton & Pavone IROS 2025），位于 `portable_oscbf/work`
-- 真机通信：python-can / SocketCAN，DrEmpower 协议
-
-## Build & Run
+## 构建与测试入口

 ```bash
-bash build_aeb_moveit.sh   # 必须用此脚本：普通 colcon build 找不到嵌套 C++ 包
+bash build_aeb_moveit.sh   # 嵌套 C++ 插件需由此脚本构建
 source install/setup.bash
-bash run_demo.sh           # 全自动演示，无需键盘
-bash run_all_tests.sh      # 全量测试入口（主包 + 控制内核）
-pytest                     # 主包测试（setup.cfg: testpaths = tests）
-pytest portable_oscbf/tests  # 控制核心测试
-```
-
-改 AEB C++ 代码后必须重新构建并重启整个 launch（move_group 不会热加载 `.so`）。
-
-## Environment Setup（Ubuntu 22.04 / ROS Humble）
-
-```bash
-# 1) ROS 系统包（MoveIt2 2.5.9 + 控制器/参数化工具）
-sudo apt-get install -y ros-humble-moveit ros-humble-moveit-configs-utils \
-  ros-humble-controller-manager ros-humble-controller-manager-msgs \
-  ros-humble-xacro ros-humble-joint-state-publisher
-
-# 2) Python 依赖（用户态安装到 ~/.local；版本约束重要！）
-python3 -m pip install --user "jax==0.6.2" "jaxlib==0.6.2" "cbfpy==0.0.1" \
-  qpax python-fcl trimesh mujoco matplotlib
-# 注意：cbfpy 必须锁 0.0.1 —— 0.0.3+ 将 relax_cbf 改名为 relax_qp，
-# 会使 JAX 内核构造抛 TypeError（CBFConfig unexpected keyword 'relax_cbf'）。
-# cbfpy 0.0.1 带 numpy<2 约束，会自动回退 numpy 1.26.x。
-
-# 3) pymoveit2（非 PyPI，需 colcon 构建进本仓库 install/ 前缀）
-git clone --branch 3.2.0 --depth 1 https://github.com/AndrejOrsula/pymoveit2 \
-  ~/robot/pymoveit2_ws/src/pymoveit2
-source /opt/ros/humble/setup.bash
-cd ~/robot/pymoveit2_ws && colcon build --symlink-install \
-  --install-base /home/lsn/robot/robot_safecontrol/install
+bash scripts/agent_check.sh  # 快速启发式检查，不替代完整测试
+bash run_all_tests.sh        # 主包 + portable OSCBF 完整测试
+bash run_demo.sh             # 仿真演示；会清理旧 demo 进程
 ```

-已知未实施事项（均在 tracker，详见 `.scratch/oscbf-wayfinder/issues/`）：
-`run_all_tests.sh` 的 `set -u` 与 ROS setup.bash 冲突（issue #10）；
-主包 4 个必现失败：settle harness 缺方法 ×2、perf p95 预算、e2e 步数
-（issue #9，pytest tests/ 需绕过脚本另行运行）；
-`test_tool_axis_path_kernel_ignores_roll_only_reference_at_path_start` 已在工作区修复：
-构造前固定并恢复 x64，区分起点与执行后误差并加入倾斜对照；内核套件
-148 passed、34 skipped，修订已归档，详见 [实施记录](docs/planning/oscbf-reuse/handoffs/11-roll-only-tolerance.md)。
-
-## Code Conventions
-
-- 每个 ROS 节点一个模块，小写下划线命名，`main()` 注册为 console_scripts
-  （见 setup.py `entry_points`）
-- 模块级 docstring 写明职责与话题约定（如 oscbf_controller.py 的 M10 注释）
-- 话题名/QoS/关节名等共享约定统一在 `ros_conventions.py` 与 `robot_spec.py`，
-  节点间禁止互相 import 私有符号
-- 控制内核（portable_oscbf/work）零 ROS 依赖、零裸名同级 import（统一 `from work.X`），
-  节点经 `oscbf_trajectory.bootstrap_portable` 引导
-- 测试 `test_*.py`；launch 集成测试用 launch_testing；对必需配置文件做启动校验
-- 提交信息中英混合，多为 feat:/fix:/perf: 前缀
-
-## Key Entry Points
-
-- `src/robot_safecontrol_moveit/oscbf_controller.py` — OSCBF 控制器节点，发布 `/oscbf_command`
-- `src/robot_safecontrol_moveit/oscbf_plant.py` — jerk 限幅执行器仿真，发布 `/mujoco_joint_states`
-- `src/robot_safecontrol_moveit/transition_planning_server.py` — 过渡规划服务器（薄 ROS 壳）
-- `src/robot_safecontrol_moveit/transition_executor.py` — 过渡管线相位机（纯逻辑、无 ROS，经 ports 注入副作用）
-- `src/robot_safecontrol_moveit/mujoco_viewer_with_cylinder.py` — MuJoCo 仿真/查看器
-- `src/robot_safecontrol_moveit/oscbf_trajectory.py` — 三端共享的轨迹变换
-- `src/robot_safecontrol_moveit/hardware_bridge.py` — 真机执行端（CAN + 安全网关）
-- `src/robot_safecontrol_moveit/perception_bridge.py` — 感知桥接节点
-- `src/robot_safecontrol_moveit/tracking_evaluator.py` — 跟踪评价指标
-- `portable_oscbf/work/jax_control_facade.py` — JAX 控制内核主机端入口
-- `src/aeb_rrtstar_ompl/src/AEBRRTstar.cpp` — C++ AEB-RRT* MoveIt 插件
-- `launch/mujoco_transition_final.launch.py` — 完整闭环 launch
-
-## Data Flow & Conventions
-
-`/mujoco_joint_states`（植物状态）→ oscbf_controller → `/oscbf_command` →
-oscbf_plant（S 曲线 jerk 限幅）→ 状态发回。控制器独立于 MoveIt。
-
-- 坐标系：URDF Y-up ↔ MuJoCo Z-up，经 `display_frame` euler 旋转转换
-- 圆柱轴心拟合口径三端（轨迹/过渡/控制器）必须一致，默认最小二乘圆拟合
-- 旋转误差用精确旋转向量 `-log(R_des·R_eeᵀ)`，不用一阶叉积（180° 盲区）
-- 真机三种模式：sim（MuJoCo 仿真）、shadow（记录不发送）、live（CAN 发送）
-
-## Key Documents
-
-- `README.md` — 概览、运行方式、关节配置、真机模式
-- `CONTEXT.md` — 领域词汇表
-- `LESSONS_LEARNED.md` — 已踩坑教训（改参数/接口前先读）
-- `OSCBF_PORTING_GUIDE.md` — OSCBF 移植架构与验收门
-- `docs/real_robot_runbook.md` — 真机操作手册
-- `docs/ONBOARDING.md` — 完整入门指南
-
-## Agent skills
-
-### Issue tracker
-
-Issues live as GitHub issues on `Shiningliu1011/nineaxis_safecontrol`; use the `gh` CLI. See `docs/agents/issue-tracker.md`.
-
-### Domain docs
-
-Single-context: one `CONTEXT.md` + `docs/adr/` at the repo root. See `docs/agents/domain.md`.
-
-## Notes
-
-- `src/aeb_rrtstar/` 为独立 Python 规划器（无 ROS），被 C++ 插件包参考
-- `models/ninezzhou*` 为机械臂 URDF/MoveIt 配置；`data/nurbs/ik_input.mat` 为 IK 输入
-- `config/` 下 YAML 为节点参数；`output/` 为生成文件（gitignore）
+修改 AEB C++ 后需重新构建并重启 launch，运行中的 move_group 不会热加载 `.so`。
+环境依赖见 [ONBOARDING](docs/ONBOARDING.md) 与 [控制内核 README](portable_oscbf/README.md)。
+当前测试真值由上述命令产生；历史测试数量和 issue body 不代表当前 HEAD。
+当前 tracker 见 [GitHub 地图](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/1)。
+
+## 当前硬件安全边界
+
+- sim：hardware bridge 不创建真实控制 I/O。
+- shadow：仅记录命令请求与安全拒绝，不收发 CAN；无真实反馈时不发布硬件状态或报告健康。
+- live：当前 containment 阶段禁用，无条件拒绝启动硬件执行路径，无参数可绕过。
+- command、补零、重打时间戳及人工 acknowledgement 均不能替代真实反馈。
+- containment、测试通过不等于真机准入或物理停车能力；硬件后续工作见 [#13](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13)。
+
+## Canonical sources 与导航
+
+- 架构、入口与当前状态流 ownership：[ONBOARDING](docs/ONBOARDING.md)。
+- 领域词汇与坐标语义：[CONTEXT](CONTEXT.md)、[ADR](docs/adr/)；ADR 目标与实现进度分开核对。
+- ROS topic/QoS：[ros_conventions.py](src/robot_safecontrol_moveit/ros_conventions.py)。
+- 关节身份/顺序：[robot_spec.py](src/robot_safecontrol_moveit/robot_spec.py)。
+- 共享轨迹变换：[oscbf_trajectory.py](src/robot_safecontrol_moveit/oscbf_trajectory.py)。
+- 生产 OSCBF 配置：[oscbf_controller.yaml](config/oscbf_controller.yaml)，入口契约见 [README](README.md)。
+- 参数/接口历史教训：[LESSONS_LEARNED](LESSONS_LEARNED.md)；复用时核对适用环境。
+- Reviewer 检查：[CODING_STANDARDS](CODING_STANDARDS.md)。
+- Tracker 操作：[issue-tracker](docs/agents/issue-tracker.md)；领域文档流程：[domain](docs/agents/domain.md)。
+- 真机目标与验收步骤：[runbook](docs/real_robot_runbook.md)；当前能力以 containment 实现和 #13 最新进展为准。
diff --git a/README.md b/README.md
index 959b12b..a6d1caf 100755
--- a/README.md
+++ b/README.md
@@ -11,7 +11,7 @@
 - [OSCBF 移植指南](OSCBF_PORTING_GUIDE.md)、[真机运行手册](docs/real_robot_runbook.md)
 - [独立 MID-360 / MID-360S 驱动](xy-mid-360-s/README.md)：安装、设备接口、来源许可及已验证范围

-最新发布的测试工具诊断见下方“测试与已知问题”；两项修复均待实施和完整验收。
+测试入口与 roll-only 测试修订已实施；当前验证方法与历史记录见下方“测试与已知问题”。

 ## 目录结构

@@ -99,16 +99,16 @@ PlanningScene/FCL 做状态与路径碰撞检查。
 bash run_all_tests.sh
 ```

-截至 2026-09-11 已发布的诊断：
+已实施的测试工具修订（链接保留各自历史验收记录）：

 | 项目 | 结论与状态 |
 |---|---|
 | [测试入口退出](docs/planning/oscbf-reuse/handoffs/10-test-entrypoint.md) | 已实施局部关闭 nounset 与双套件退出码汇总；验收记录见链接。 |
-| [roll-only 路径起点测试](docs/planning/oscbf-reuse/handoffs/11-roll-only-tolerance.md) | 已修订精度、纯 roll / 倾斜对照与执行后报告断言；内核 148 passed、34 skipped，修订已归档。 |
+| [roll-only 路径起点测试](docs/planning/oscbf-reuse/handoffs/11-roll-only-tolerance.md) | 已修订精度、纯 roll / 倾斜对照与执行后报告断言；修订已归档，历史验收环境见链接。 |

-入口修复不表示当前全量测试通过；既有测试失败继续由对应修复票处理。[可复现基线](docs/planning/oscbf-reuse/handoffs/30-baseline.md)保留此前运行结果及适用代码版本。
+当前测试真值请运行 `bash scripts/agent_check.sh`（快速反馈）与 `bash run_all_tests.sh`（完整回归）。当前 HEAD 的验收进展见 [#13 最新评论](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13)；记录应注明 commit、日期、环境和退出码。[可复现基线](docs/planning/oscbf-reuse/handoffs/30-baseline.md)仅保留历史结果，不表示当前仍有同样失败。

-## 真机运行（shadow/live 模式）
+## 硬件模式（当前 containment）

 当前处于 fail-closed containment：sim 不创建控制接口，shadow 只记录且不收发 CAN、不发布关节状态；live 无条件拒绝启动。真实 backend、配置、标定和真实反馈 freshness/watchdog 链路完成独立验收前，不提供启用 live 的参数。反馈不可用时不会报告 healthy，人工确认不能解除这一条件。禁止发送不等于物理制动，物理急停仍是独立链路。以下 live 命令仅用于说明参数形式，不是可用的真机运行入口；后续验收见[真机运行手册](docs/real_robot_runbook.md)。

@@ -117,16 +117,16 @@ bash run_all_tests.sh
 ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
     hardware_mode:=shadow start_oscbf_plant:=false

-# live 参数形式：当前将报错退出
+# live 参数形式：当前 hardware_bridge 将报错退出
 ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
     hardware_mode:=live start_oscbf_plant:=false

-# 即使启用感知，live 当前仍报错退出
+# 即使启用感知，live 当前 hardware_bridge 仍报错退出
 ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
     hardware_mode:=live start_oscbf_plant:=false start_perception:=true
 ```

-零位标定：`python3 scripts/calibrate_zero.py --interface can0`（详见[真机运行手册](docs/real_robot_runbook.md)）。
+这里的退出仅指 hardware_bridge；当前 launch 未配置联动 shutdown，其他仿真/控制节点可能继续运行。标定与实机操作属于后续独立准入流程，参见[真机运行手册](docs/real_robot_runbook.md)，不是当前 containment 的运行步骤。

 话题约定：OSCBF 控制器订阅植物状态 `/mujoco_joint_states`，把安全命令发布到
 `/oscbf_command`；`oscbf_plant` 节点把命令经 jerk 限幅积分后作为植物状态发回
@@ -154,4 +154,4 @@ ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \

 ## 传感器安装方案

-深度相机与雷达共用支架的布局、布线及标定方案已在本地记录，包含安装坐标、MuJoCo 图示、几何验证结果和实机待办。`docs/sensor_layout_and_calibration.md` 与 `docs/adr/0007-shared-rear-sensor-stand.md` 尚未随本次 README 更新发布到远端；远端已发布的驱动核验见[双传感器驱动交接](docs/planning/oscbf-reuse/handoffs/31-official-sensor-drivers.md)。
+深度相机与雷达共用支架的[布局与标定方案](docs/sensor_layout_and_calibration.md)及 [ADR 0007](docs/adr/0007-shared-rear-sensor-stand.md)已纳入仓库，包含安装坐标、MuJoCo 图示、几何验证结果和实机待办；文档存在不代表实机标定完成。驱动核验见[双传感器驱动交接](docs/planning/oscbf-reuse/handoffs/31-official-sensor-drivers.md)。
diff --git a/docs/ONBOARDING.md b/docs/ONBOARDING.md
index 721e050..92854c1 100755
--- a/docs/ONBOARDING.md
+++ b/docs/ONBOARDING.md
@@ -1,14 +1,12 @@
 # robot_safecontrol — Onboarding Guide
-**Generated:** 2026-09-03  **Stack:** ROS 2 Humble / MoveIt 2 / MuJoCo / JAX / C++ OMPL
+**Updated:** 2026-09-11  **Stack:** ROS 2 Humble / MoveIt 2 / MuJoCo / JAX / C++ OMPL

 ## Overview

-9-DOF 冗余机械臂（1 棱柱关节 J1 + 8 旋转关节 J2-J9）的安全控制项目：在 MuJoCo
-物理仿真或真机上，机械臂从任意随机工作位姿出发，自动规划无碰撞过渡（AEB-RRT*）
-到蝴蝶形参考轨迹起点，随后 OSCBF 安全控制器接管，在保证碰撞/关节限位安全的前提
-下完成末端轨迹跟踪（基于 Morton & Pavone, *Safe, Task-Consistent Manipulation
-with OSCBF*, IROS 2025）。全流程无需键盘，一键运行。支持仿真、shadow（影子记录）
-和 live（真机 CAN 发送）三种硬件模式。
+9-DOF 冗余机械臂（1 棱柱关节 J1 + 8 旋转关节 J2-J9）的安全控制项目。
+当前仿真演示从随机位姿经 AEB-RRT* 过渡到蝴蝶轨迹起点，再由 OSCBF 控制器跟踪。
+真机执行端处于 fail-closed containment：sim inert，shadow 仅记录且无 CAN I/O，
+live 无条件拒绝。仿真验证不代表真实硬件能力或实机验收完成。

 ## Tech Stack

@@ -30,18 +28,29 @@ with OSCBF*, IROS 2025）。全流程无需键盘，一键运行。支持仿真
 其 `src/aeb_rrtstar_ompl` 是嵌套的 ament_cmake 包（MoveIt2 OMPL 插件），普通
 `colcon build` 发现不了它，必须用 `build_aeb_moveit.sh` 指定 `--base-paths`。

-**控制闭环（单条链路）**：MuJoCo 查看器发布 `/mujoco_joint_states` →
-`oscbf_controller` 节点订阅植物状态、运行纯 JAX OSCBF 内核 → 安全命令发布到
-`/oscbf_command` → `oscbf_plant` 节点（带加速度/jerk 限幅的 S 曲线驱动仿真器）
-积分后把状态发回 `/mujoco_joint_states`。控制器独立于 MoveIt，不依赖 move_group。
+**仿真闭环（`run_demo.sh` 配置）**：脚本启用 `oscbf_plant`，它订阅
+`/oscbf_command`，通过带加速度/jerk 限幅的执行器仿真积分并持续发布
+`/mujoco_joint_states`。`oscbf_controller` 订阅该状态流，运行 JAX 内核后
+发布命令。viewer 只订阅状态做 MuJoCo 显示，不发布关节状态。
+过渡服务器经 `trajectory_execution.py` 将回放发送到 `/transition_replay_viz`，
+并向命令流发送过渡命令；回放及收敛完成后交接给控制器。
+
+**默认 launch 与 demo 的差别**：直接启动最终 launch 时 `start_oscbf_plant=false`，
+`transition_replay_topic=/mujoco_joint_states`；过渡回放会向此状态话题发布，
+但没有被控对象持续积分形成上述闭环。不能把 demo 的状态 ownership 套到所有配置。
+当前 containment 的 hardware bridge 在 sim/shadow 都不发布硬件状态。

 **自主过渡流程**：`transition_planning_server` 监听开始信号后，调用 move_group
 的 AEB-RRT* 规划器从随机位姿规划无碰撞过渡到轨迹起点，经 Ruckig 平滑后回放；
 回放结束 → OSCBF 控制器接管（`oscbf_wait_for_start`）。

-**真机部署链路**：`hardware_bridge` 节点订阅 `/oscbf_command`，经安全网关校验后
-换算为 DrEmpower CAN 帧发送给电机，反馈帧解码后发布到 `/mujoco_joint_states`。
-支持 shadow（仅记录）和 live（实际发送）两种模式。
+**真机目标与当前实现**：未来目标是 command → 安全网关 → CAN → 真实反馈 →
+硬件状态流。当前 `hardware_bridge.py` 未接通该链路：sim 不创建控制订阅、
+polling timer 或状态 publisher；shadow 订阅命令并记录请求/拒绝，不创建 CAN
+backend、不收发 CAN；live 在创建硬件控制实体之前无条件失败。
+无真实反馈时 `feedback_ok`、`watchdog_ok` 均为 false，acknowledge 不能制造健康状态。
+节点拒绝 live 尚不导致整个 launch 联动退出；其他节点可能继续运行。
+剩余实现及真机准入见 [GitHub #13](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13)。

 **感知管线**：`perception_bridge` 接收点云 → `obstacle_extractor` 聚类拟合球/
 圆柱几何 → 发布到 `/perception/tracks` → `oscbf_controller` 经 `obs_*` 接口注入
@@ -61,14 +70,14 @@ euler 旋转自动转换。圆柱轴心拟合口径在轨迹生成端、过渡
 | `portable_oscbf/` | 可移植 JAX OSCBF 控制核心（work/ 为 Python 包，随包分发） |
 | `portable_oscbf/work/` | 控制内核 Python 包（零 ROS 依赖，统一 `from work.X` import） |
 | `portable_oscbf/vendor/dpax/` | 内嵌的 DCOL 可微碰撞库 |
-| `portable_oscbf/tests/` | 控制核心独立测试（36+ 个单测文件） |
+| `portable_oscbf/tests/` | 控制核心独立测试 |
 | `portable_oscbf/scripts/` | 内核调试/标定脚本 |
 | `models/ninezzhou/` | 9 轴机械臂 URDF + STL 网格 |
 | `models/ninezzhou_moveit_config/` | MoveIt2 配置（SRDF、控制器、运动学） |
 | `config/` | 节点 YAML 参数（oscbf_controller.yaml、drempower.yaml 等） |
 | `launch/` | launch 文件（mujoco_transition_final.launch.py 为完整闭环） |
 | `data/nurbs/` | NURBS 轨迹数据（ik_input.mat 逆运动学输入） |
-| `tests/` | 主包 pytest 测试（24 个文件，含 launch 集成测试） |
+| `tests/` | 主包 pytest 测试（含 launch 集成测试） |
 | `scripts/` | 辅助脚本（零位标定、vcan 测试、清场启动） |
 | `docs/` | 文档（本指南、ADR、specs、runbook） |
 | `output/` | 生成文件（已 gitignore） |
@@ -76,20 +85,21 @@ euler 旋转自动转换。圆柱轴心拟合口径在轨迹生成端、过渡

 ## Request Lifecycle（闭环数据流）

-1. `mujoco_viewer_with_cylinder.py` 加载 `models/ninezzhou` URDF → MuJoCo 仿真，
-   发布关节状态到 `/mujoco_joint_states`（transient local QoS）。
-2. `oscbf_controller.py` 订阅状态（BEST_EFFORT），加载仓库蝴蝶轨迹
-   （`oscbf_trajectory.py` 统一变换），经 `portable_oscbf/work` 的
-   `JaxControlLoop` facade（输入归一化 → JIT 内核：OSC + CBF + QP + 积分）
-   计算安全关节速度/命令，发布到 `/oscbf_command`。
-3. `oscbf_plant.py` 的 `SCurveDriverSimulator` 对命令做位置环 + 加速度/jerk
-   限幅，持续发布状态回 `/mujoco_joint_states`（即使命令丢失也不会卡死，
-   模拟真实编码器行为）。
-4. `transition_planning_server.py` 在开始前用 move_group(AEB-RRT* 插件 +
-   FCL 碰撞检查) 规划任意位姿→轨迹起点的无碰撞过渡路径，经 Ruckig 平滑后回放。
-5. `tracking_evaluator.py` 订阅状态与命令，实时计算跟踪误差、完成度等指标。
-6. 轨迹变换统一：查看器、过渡服务器、控制器共用 `oscbf_trajectory.py`，
-   保证 tool0 与显示的蝴蝶曲线重合。
+1. `run_demo.sh` 启用被控对象并设置随机起始位姿；`oscbf_plant.py` 持续发布仿真状态。
+2. `transition_planning_server.py` 调用 MoveIt/AEB-RRT* 与 FCL 规划过渡，经 Ruckig
+   平滑并回放命令；回放后等待被控对象收敛，再交接给 OSCBF 控制器。
+3. `oscbf_controller.py` 订阅状态，经 `portable_oscbf/work` 的 JAX facade 计算并
+   发布 `/oscbf_command`；被控对象积分后继续发布 `/mujoco_joint_states`。
+4. `mujoco_viewer_with_cylinder.py` 订阅状态驱动显示；它不拥有控制状态 publisher。
+5. `tracking_evaluator.py` 可订阅状态与命令计算跟踪指标；查看器、过渡服务器和
+   控制器共用 `oscbf_trajectory.py` 的轨迹变换。
+
+**当前 QoS**：canonical [ros_conventions.py](../src/robot_safecontrol_moveit/ros_conventions.py)
+的 `state_stream_qos()` 是 KEEP_LAST / depth 20 / BEST_EFFORT / VOLATILE。
+被控对象的状态发布和命令订阅、控制器的状态订阅及 shadow bridge 的命令订阅使用它。
+viewer、过渡服务器的状态订阅、过渡回放发布和控制器的命令发布仍使用
+`qos_profile_sensor_data`（depth 5 / BEST_EFFORT / VOLATILE）。深度差异仍存在，
+这里记录当前实体，不代表全面 QoS 已统一；不存在 viewer 发布 transient-local 状态的链路。

 ## Conventions Detected

@@ -103,9 +113,9 @@ euler 旋转自动转换。圆柱轴心拟合口径在轨迹生成端、过渡
   （统一 `from work.X`），节点经 `oscbf_trajectory.bootstrap_portable` 引导。
 - **错误处理**：launch 文件对必需配置文件做启动时 `FileNotFoundError` 校验；
   内核侧有 QP 健康检查（`qp_solver_health.py`、`safety_snapshot.py`）；
-  真机侧有安全网关（超时/故障/限幅违例 → 零速保持 + 锁存停车原因）。
+  硬件合同网关可生成拒绝/保持结果并锁存原因；当前 bridge 无 CAN 发送能力，不能据此推断物理停车。
 - **测试**：pytest（`testpaths = tests`），主包含 launch 集成测试
-  （launch_testing）；`portable_oscbf/tests` 有独立 conftest 与 36+ 个单测文件；
+  （launch_testing）；`portable_oscbf/tests` 有独立 conftest；
   C++ 包有自测可执行文件（test_aeb_full.cpp 等）。全量入口：`bash run_all_tests.sh`。
 - **Git**：单 main 分支；提交信息中英混合、多为 feat:/fix:/perf: 前缀或中文摘要。

@@ -119,19 +129,13 @@ source install/setup.bash
 # 全自动演示：随机起始位姿 → 无碰撞过渡 → OSCBF 跟踪蝴蝶轨迹（无需键盘）
 bash run_demo.sh

-# 真机模式
-ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
-    hardware_mode:=shadow start_oscbf_plant:=false   # shadow 模式
+# shadow 记录模式：无 CAN I/O，不提供真实硬件反馈
 ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
-    hardware_mode:=live start_oscbf_plant:=false      # live 模式
-
-# 测试
-pytest                                   # 主包测试（24 个文件）
-pytest portable_oscbf/tests              # 控制核心测试（36+ 个文件）
-bash run_all_tests.sh                    # 全量入口
+    hardware_mode:=shadow start_oscbf_plant:=false

-# 零位标定
-python3 scripts/calibrate_zero.py --interface can0
+# 测试：结果以当前 checkout 实际执行为准
+bash scripts/agent_check.sh              # 快速启发式检查
+bash run_all_tests.sh                    # 完整主包 + 内核回归

 # 独立脚本（无 ROS）
 python3 src/aeb_rrtstar/single_run.py    # 查看 aeb_rrtstar 用法
@@ -148,12 +152,12 @@ launch——运行中的 move_group 不会自动加载新编译的 `.so` 插件
 - `src/robot_safecontrol_moveit/transition_executor.py` — 过渡管线相位机（纯逻辑，无 ROS）
 - `src/robot_safecontrol_moveit/mujoco_viewer_with_cylinder.py` — MuJoCo 查看器/仿真
 - `src/robot_safecontrol_moveit/oscbf_trajectory.py` — 统一轨迹变换（三端共享）
-- `src/robot_safecontrol_moveit/hardware_bridge.py` — 真机执行端（CAN 通信 + 安全网关）
+- `src/robot_safecontrol_moveit/hardware_bridge.py` — containment 入口（sim inert / shadow 记录 / live disabled）
 - `src/robot_safecontrol_moveit/perception_bridge.py` — 感知桥接节点
 - `src/robot_safecontrol_moveit/obstacle_extractor.py` — 点云→几何障碍物提取
 - `src/robot_safecontrol_moveit/tracking_evaluator.py` — 跟踪评价指标
 - `src/robot_safecontrol_moveit/drempower_can.py` — DrEmpower CAN 协议编解码
-- `src/robot_safecontrol_moveit/socketcan_backend.py` — SocketCAN 后端
+- `src/robot_safecontrol_moveit/socketcan_backend.py` — SocketCAN 抽象与替身测试入口，当前 bridge 不接入
 - `portable_oscbf/work/jax_control_facade.py` — JAX 控制内核主机端入口
 - `src/aeb_rrtstar_ompl/src/AEBRRTstar.cpp` — C++ AEB-RRT* 插件实现
 - `launch/mujoco_transition_final.launch.py` — 完整闭环 launch
````

## 10. Suggested next action

1. Review本轮三文档，并在目标桌面新会话补齐Retro显式/普通请求行为验收。
2. 单独批准CONTEXT两项定义修订；历史实机措辞需来源证据再处理。
3. 单独审阅launch方案B后再实施；真实硬件实现和准入继续由#13独立推进。
