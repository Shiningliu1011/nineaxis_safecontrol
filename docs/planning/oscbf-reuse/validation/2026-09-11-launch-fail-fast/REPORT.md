# Source-of-truth cleanup 与 launch-level live fail-fast

## 1. Repository state

- Branch：`main`。
- Starting HEAD：`f16cc35c43f4c3472adaf43b875d64d723460a3e`；干净，无用户未提交修改。
- 本轮产品代码验证：上述 HEAD 加 `tested.diff`。最终文档另经 fast-check。
- 实现 ending HEAD：`5c6b3b4b0854e1172c2131d665291f6dcf456d34`，干净（报告归档前）；source-of-truth 清理提交 `8283a28`，独立 launch/说明提交 `5c6b3b4`。本报告随其后的独立 archive commit 提交；最终提交 SHA 用 `git log -1 --format=%H -- docs/planning/oscbf-reuse/validation/2026-09-11-launch-fail-fast/REPORT.md` 查询，避免自引用 SHA。Git 以用户本条“执行完后提交到本地和远程仓库”为授权，覆盖附件的旧禁止提交/推送条件。
- 实施范围：README、CONTEXT、controller 注释；final launch 与结构测试；CLAUDE、ONBOARDING 最小同步。报告与日志另行归档，不修改旧证据。

## 2. Source-of-truth cleanup

README 不再把 e4d9a268 上的历史 #13 评论称作当前 HEAD 验收，明确旧完整验收版本与当前 checkout 的实际测试命令，不永久缓存测试数量。

CONTEXT 修正状态流 ownership：viewer 只订阅，run_demo 的被控对象持续发布，未启用被控对象的默认 launch 可由过渡回放发布；不再宣称所有实体 QoS 深度一致。真机执行端保留未来职责，同时明确当前 sim/shadow/live containment。

`oscbf_controller.py` 只把错误 viewer ownership 注释改成 canonical state-stream QoS 表述。AST 与 starting HEAD 完全一致，无运行行为变化。

## 3. Issue #13

[本轮短状态评论](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13#issuecomment-5630878593)，已重读确认 OPEN、正文匹配草稿。仅追加此评论，没有编辑历史、关闭或创建 issue。

评论是 Phase 3 快照，当时清理尚未提交、launch 修改尚未实施；后续实际提交以本报告与 Git 为准。旧完整 containment 产品代码验收绑定 `e4d9a268`；后续文档归档以及本轮均未改变 hardware_bridge 运行行为。本轮 launch 行为另以本轮测试证据说明。

真实 SocketCAN qualification、反馈 freshness/独立 watchdog、映射/标定、传输故障处理与物理停止仍未完成，保持 #13 开放。

## 4. Retro explicit-only acceptance

A 普通请求：无法观察隔离宿主会话中的 skill 注入记录，未验证“不触发”。B 显式 `$retro`：无法观察原生 dispatcher 注入路径，未验证端到端加载。可读取 repo skill 并按用户指令进行 report-only review，不等于原生调用验收。

repo skill：`/home/lsn/robot/robot_safecontrol/.agents/skills/retro/SKILL.md`。元数据 disable-model-invocation=true、allow_implicit_invocation=false 未修改；未安装全局副本，未以 skills/list 代替行为证据。

**runtime behavior acceptance not observable in this environment**。细节见 `retro-runtime.txt`。

## 5. Launch fail-fast implementation

在 DeclareLaunchArgument 之后、所有 Node/TimerAction 之前增加 OpaqueFunction；读取 hardware_mode=live 时抛出 `live disabled by containment; refusing entire launch`。

| 模式 | 真实 LaunchService、mock Node.execute 的结果 |
|---|---|
| default | 默认 sim，退出 0，Node.execute 被调用 |
| sim | 退出 0，Node.execute 被调用 |
| shadow | 退出 0，Node.execute 被调用 |
| live | 退出 1，Node.execute 0 次 |

测试只模拟进程启动，不能证明实际 MoveIt 图可达。保留 hardware_bridge 自身 direct node guard；未来解除 live containment 时两层必须独立审查。README、ONBOARDING、CLAUDE 仅同步上述已实现事实。旧归档 patch 是历史提案，本轮采用实际参数 `start_viewer` 修正其中无效的 `start_mujoco_viewer` 测试设置。

## 6. Tests

所有 ROS 检查使用 Humble 与 install/setup.bash；localhost-only、domain 217。环境详见 environment.txt：Python 3.10.12、pytest 6.2.5、JAX/jaxlib 0.6.2、numpy 1.26.4，CPU（无 CUDA-enabled jaxlib）。

### launch structure

`python3 -m pytest -q tests/test_launch_structure.py`：7 passed，exit 0。

### focused containment

`python3 -m pytest -q tests/test_hardware_bridge.py tests/test_launch_structure.py tests/test_hardware_contract.py`：50 passed，exit 0。

### fast-check

Phase 2、Phase 6 与最终文档阶段运行 `bash scripts/agent_check.sh`，各阶段均 exit 0，详情见对应日志。compileall、shell syntax（仅语法）、staged/unstaged diff、62 pure contracts。

### full regression

按用户“不得运行真实节点进程”约束，停止原封不动的完整套件步骤：tests/test_final_launch_runtime.py 实际启动 ROS/MoveIt 进程。未修改该文件、测试配置或降低其断言，而在本次命令中明确排除整个文件。

```bash
PYTEST_ADDOPTS='--ignore=tests/test_final_launch_runtime.py -ra' \
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=217 bash run_all_tests.sh
```

主包 **351 passed / 0 failed / 0 skipped，95.19s**；portable **148 passed / 0 failed / 34 skipped，451.61s**；两套退出码及总退出码均 **0**。

portable：15 项为既有显式 skip（unified_qp_solver、newaxis、benchmark/HARD_STOP 等历史语义），19 项因 newaxis 不可导入。它们不构成硬件验收。

此为受限回归，**不宣称无删减完整回归通过**。排除文件不计入 pytest skipped；另外的 portable skip 原因见 full.txt 的 -ra 输出。文档阶段不重复昂贵 JAX suite。

### diff check

Phase 0、2、6、8 及提交前 `git diff --check`；controller AST 等价检查。全部通过；实现提交后工作区干净。报告归档还将检查 staged diff。

## 7. Safety statement

No real CAN accessed.
No hardware enabled.
No calibration executed.
No live send capability added.

没有打开 can0/vcan0、sudo 配置接口、recovery/auto reconnect，也没有启动实际 ROS 节点进程。bridge 测试在 pytest 内创建隔离 ROS 对象，真实 CAN 构造禁止；其他设备协议测试只用 Fake/loopback。launch 测试 mock Node.execute，因此即使 guard 回归也不启动进程。未改 backend、控制算法/参数、坐标系、sensor extrinsics SSOT、QoS 或 reference_lead_m。

## 8. First-order remaining work

1. #13 真正 live 实现，继续保持 containment，独立审查两层 guard。
2. real hardware acceptance，包含真实反馈、独立 watchdog 与物理停止能力。
3. 在允许的环境补齐真实节点进程集成与宿主 Retro A/B 行为证据。

本轮 report-only Retro 详见 RETRO.md：No new P0/P1；没有新候选需要 apply，不自动开始上述工作。

## 9. Final diff

最终实施 diff 与 stat 随报告归档；测试时 diff 另存为 tested.diff，以区分后续纯文档同步。归档自身未递归包含进实现 diff。

```text
 CLAUDE.md                                        |  2 +-
 CONTEXT.md                                       |  4 ++--
 README.md                                        |  8 ++++----
 docs/ONBOARDING.md                               |  2 +-
 launch/mujoco_transition_final.launch.py         | 10 +++++++++-
 src/robot_safecontrol_moveit/oscbf_controller.py |  3 ++-
 tests/test_launch_structure.py                   | 23 +++++++++++++++++++++--
 7 files changed, 40 insertions(+), 12 deletions(-)
```

````diff
diff --git a/CLAUDE.md b/CLAUDE.md
index 3d2b48b..5e6c49b 100755
--- a/CLAUDE.md
+++ b/CLAUDE.md
@@ -21,7 +21,7 @@ bash run_demo.sh             # 仿真演示；会清理旧 demo 进程

 - sim：hardware bridge 不创建真实控制 I/O。
 - shadow：仅记录命令请求与安全拒绝，不收发 CAN；无真实反馈时不发布硬件状态或报告健康。
-- live：当前 containment 阶段禁用，无条件拒绝启动硬件执行路径，无参数可绕过。
+- live：final launch 与 direct hardware bridge 均 fail closed。
 - command、补零、重打时间戳及人工 acknowledgement 均不能替代真实反馈。
 - containment、测试通过不等于真机准入或物理停车能力；硬件后续工作见 [#13](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13)。

diff --git a/CONTEXT.md b/CONTEXT.md
index 2222cd2..6a38cc2 100755
--- a/CONTEXT.md
+++ b/CONTEXT.md
@@ -17,7 +17,7 @@ _Avoid_: 过渡状态机、pipeline orchestrator
 _Avoid_: handoff moment、接管点

 **状态流**:
-被控对象与查看器发布、控制器订阅的关节状态话题流（BEST_EFFORT、深度 20），约定统一在共享模块中。
+控制器及显示/评估消费者读取的关节状态话题流。`run_demo` 仿真闭环中由被控对象持续发布；默认最终 launch 未启用被控对象时，过渡回放可向该状态话题发布。viewer 只订阅状态，不拥有状态 publisher。具体实体和 QoS 见 [ONBOARDING](docs/ONBOARDING.md)，共享约定见 `ros_conventions.py`。
 _Avoid_: joint-state topic、传感器流

 **命令流**:
@@ -91,7 +91,7 @@ _Avoid_: cylinder fitting、轴线拟合
 ## 真机部署

 **真机执行端**:
-订阅命令流（/oscbf_command）、经安全网关校验后换算为 DrEmpower CAN 帧发送给电机，并把反馈帧解码换算后发布状态流（/mujoco_joint_states）的 ROS 2 节点。
+目标职责是订阅命令流，经安全网关和硬件传输发送真实执行命令，并将真实反馈转换成状态流。当前 `hardware_bridge` 仅提供 fail-closed containment：sim 不创建硬件控制 I/O；shadow 只记录请求/拒绝，不收发 CAN、不发布真实硬件状态；live 禁用。真实 SocketCAN、反馈 freshness/watchdog 和真机准入见 [GitHub #13](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13)。
 _Avoid_: hardware bridge、CAN bridge

 **安全网关**:
diff --git a/README.md b/README.md
index a6d1caf..9ba58f4 100755
--- a/README.md
+++ b/README.md
@@ -106,22 +106,22 @@ bash run_all_tests.sh
 | [测试入口退出](docs/planning/oscbf-reuse/handoffs/10-test-entrypoint.md) | 已实施局部关闭 nounset 与双套件退出码汇总；验收记录见链接。 |
 | [roll-only 路径起点测试](docs/planning/oscbf-reuse/handoffs/11-roll-only-tolerance.md) | 已修订精度、纯 roll / 倾斜对照与执行后报告断言；修订已归档，历史验收环境见链接。 |

-当前测试真值请运行 `bash scripts/agent_check.sh`（快速反馈）与 `bash run_all_tests.sh`（完整回归）。当前 HEAD 的验收进展见 [#13 最新评论](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13)；记录应注明 commit、日期、环境和退出码。[可复现基线](docs/planning/oscbf-reuse/handoffs/30-baseline.md)仅保留历史结果，不表示当前仍有同样失败。
+最近一次 containment 产品代码完整验收基于 `e4d9a268`，对应证据见[验证归档](docs/planning/oscbf-reuse/validation/2026-09-11-containment/REPORT.md)。当前 checkout 的测试真值仍以实际运行 `bash scripts/agent_check.sh`（快速反馈）与 `bash run_all_tests.sh`（完整回归）为准；验证记录应注明 commit、日期、环境和退出码。[可复现基线](docs/planning/oscbf-reuse/handoffs/30-baseline.md)仅保留历史结果，不表示当前仍有同样失败。

 ## 硬件模式（当前 containment）

-当前处于 fail-closed containment：sim 不创建控制接口，shadow 只记录且不收发 CAN、不发布关节状态；live 无条件拒绝启动。真实 backend、配置、标定和真实反馈 freshness/watchdog 链路完成独立验收前，不提供启用 live 的参数。反馈不可用时不会报告 healthy，人工确认不能解除这一条件。禁止发送不等于物理制动，物理急停仍是独立链路。以下 live 命令仅用于说明参数形式，不是可用的真机运行入口；后续验收见[真机运行手册](docs/real_robot_runbook.md)。
+当前处于 fail-closed containment：sim 不创建控制接口，shadow 只记录且不收发 CAN、不发布关节状态；final launch 在启动任何节点前拒绝 `hardware_mode=live`，`hardware_bridge` 自身也保留独立拒绝。真实 backend、配置、标定和真实反馈 freshness/watchdog 链路完成独立验收前，不提供启用 live 的参数。反馈不可用时不会报告 healthy，人工确认不能解除这一条件。禁止发送不等于物理制动，物理急停仍是独立链路。以下 live 命令仅用于说明参数形式，不是可用的真机运行入口；后续验收见[真机运行手册](docs/real_robot_runbook.md)。

 ```bash
 # shadow 模式：记录请求及拒绝结果；真实反馈 unavailable；不收发 CAN
 ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
     hardware_mode:=shadow start_oscbf_plant:=false

-# live 参数形式：当前 hardware_bridge 将报错退出
+# live 参数形式：final launch 在启动任何节点前报错退出
 ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
     hardware_mode:=live start_oscbf_plant:=false

-# 即使启用感知，live 当前 hardware_bridge 仍报错退出
+# 即使启用感知，final launch 仍在启动任何节点前拒绝 live
 ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
     hardware_mode:=live start_oscbf_plant:=false start_perception:=true
 ```
diff --git a/docs/ONBOARDING.md b/docs/ONBOARDING.md
index 92854c1..579d726 100755
--- a/docs/ONBOARDING.md
+++ b/docs/ONBOARDING.md
@@ -49,7 +49,7 @@ live 无条件拒绝。仿真验证不代表真实硬件能力或实机验收完
 polling timer 或状态 publisher；shadow 订阅命令并记录请求/拒绝，不创建 CAN
 backend、不收发 CAN；live 在创建硬件控制实体之前无条件失败。
 无真实反馈时 `feedback_ok`、`watchdog_ok` 均为 false，acknowledge 不能制造健康状态。
-节点拒绝 live 尚不导致整个 launch 联动退出；其他节点可能继续运行。
+final launch 在启动任何节点前拒绝 `hardware_mode=live`；`hardware_bridge` 自身也保留独立拒绝，保护 direct node 入口。未来解除 live containment 时，两层必须独立审查。
 剩余实现及真机准入见 [GitHub #13](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13)。

 **感知管线**：`perception_bridge` 接收点云 → `obstacle_extractor` 聚类拟合球/
diff --git a/launch/mujoco_transition_final.launch.py b/launch/mujoco_transition_final.launch.py
index ba16abb..ed32d1a 100755
--- a/launch/mujoco_transition_final.launch.py
+++ b/launch/mujoco_transition_final.launch.py
@@ -11,7 +11,7 @@ from pathlib import Path

 from ament_index_python.packages import get_package_share_directory
 from launch import LaunchDescription
-from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
+from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
 from launch.conditions import IfCondition
 from launch.substitutions import LaunchConfiguration
 from launch_ros.actions import Node
@@ -19,6 +19,12 @@ from launch_ros.actions import Node
 from robot_safecontrol_moveit.moveit_runtime_config import build_moveit_params


+def _reject_contained_live(context):
+    if LaunchConfiguration("hardware_mode").perform(context) == "live":
+        raise RuntimeError("live disabled by containment; refusing entire launch")
+    return []
+
+
 def generate_launch_description() -> LaunchDescription:
     share_dir = Path(get_package_share_directory("robot_safecontrol_moveit"))

@@ -81,6 +87,8 @@ def generate_launch_description() -> LaunchDescription:
             DeclareLaunchArgument("auto_plan_once", default_value="true"),
             # Show OBB collision envelopes in the MuJoCo viewer.
             DeclareLaunchArgument("show_obb", default_value="false"),
+            # Refuse before any Node or TimerAction can start a process.
+            OpaqueFunction(function=_reject_contained_live),
             # Log startup info.
             LogInfo(
                 msg=f"Starting unified MoveIt transition demo. "
diff --git a/src/robot_safecontrol_moveit/oscbf_controller.py b/src/robot_safecontrol_moveit/oscbf_controller.py
index b363dbf..9f69db7 100755
--- a/src/robot_safecontrol_moveit/oscbf_controller.py
+++ b/src/robot_safecontrol_moveit/oscbf_controller.py
@@ -131,7 +131,8 @@ class OscbfController(Node):

         joint_state_topic = str(self._runtime_config["joint_state_topic"])
         publish_topic = str(self._runtime_config["publish_joint_state_topic"])
-        # Same QoS as the MuJoCo viewer, which owns the joint-state stream.
+        # Subscribe to the configured joint-state stream using the canonical
+        # state-stream QoS.
         self.create_subscription(
             JointState,
             joint_state_topic,
diff --git a/tests/test_launch_structure.py b/tests/test_launch_structure.py
index c36df69..4303fd7 100755
--- a/tests/test_launch_structure.py
+++ b/tests/test_launch_structure.py
@@ -7,10 +7,11 @@ import sys
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


@@ -105,6 +106,24 @@ class TestFinalLaunchDescription(unittest.TestCase):
         }
         return module.generate_launch_description()

+    def test_contained_live_fails_before_any_process_starts(self) -> None:
+        for mode in (None, "sim", "shadow", "live"):
+            with self.subTest(mode=mode):
+                actions = [SetLaunchConfiguration("start_viewer", "false")]
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
+
     def test_final_launch_creates_the_expected_node_topology(self) -> None:
         description = self._description()
         nodes = _all_nodes(description.entities)
````
