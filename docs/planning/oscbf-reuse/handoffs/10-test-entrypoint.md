# [03] run_all_tests.sh 在 set -euo pipefail 下直接退出

- Tracker：https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/10
- 日期：2026-09-11
- 状态：已领取；诊断与实施范围明确，尚未实施，不关闭原实施票。
- 起点：HEAD `786e0aec61b7abbf8e8c0a5c5aa99d20c3e54f21`。工作区已有 CONTEXT.md、README.md、MAP.md 修改及传感器等未跟踪资产，本轮保留。

## 选择与授权范围

用户指令：“wayfinder 继续下一个ticket”。正式地图主队列中未领取的开放票均存在原生阻塞；依照 DECISION-WINDOW-ORDER.md 的基线修复顺序，跳过有阻塞的主包测试套件修复，领取本票。
本轮遵守该文档的窗口约定：研究和决策完成后交接，等待实施指令。未修改运行脚本、未运行硬件。

## 本机核验

1. `bash run_all_tests.sh` 退出 1，输出 `/opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable`；未进入 pytest。
2. 在 `bash --noprofile --norc -c` 中，`set -euo pipefail; export AMENT_TRACE_SETUP_FILES=; source /opt/ros/humble/setup.bash` 仍退出 1，下一处为 `AMENT_PYTHON_EXECUTABLE: unbound variable`。仅初始化一个变量不解决兼容问题。
3. 同类独立 shell 中，保持 `-e`/`pipefail`，仅在两个 source 期间 `set +u`，然后 `set -u`，两层实际 setup 均加载成功，退出 0；`[[ $- == *u* ]]` 确认 nounset 已恢复。
4. 最小 shell 探针证实：`set -euo pipefail` 下第一条测试命令退出 7 会跳过第二条；用 `command || status=$?` 分别捕获模拟套件退出值 7、3，可同时记录并最终退出 1。

上述是环境初始化与 shell 控制流证据，未执行完整 pytest，不代表两个真实套件通过。

## 可以开始编码

范围仅 `run_all_tests.sh`：

- 在加载 ROS 与工作区 setup 的窄范围关闭 nounset，随后恢复；保留环境加载失败时退出及缺少 install/setup.bash 的提示。
- 分别执行现有主包与内核 pytest 命令，捕获原始退出码；首套失败仍执行第二套。
- 末尾汇总两个退出码；仅二者均为 0 时打印全部通过并返回 0，否则返回非零。
- 不修改测试断言、控制参数、上游生成的 setup 文件或依赖。

复用结论：沿用现有 Bash、ROS/colcon setup 和 pytest；仓库 build_aeb_moveit.sh 已有 setup 不兼容 nounset 的说明。本票无新算法或第三方依赖选型，不需新增包或 fork。

## 实施验收与回退

- `bash -n run_all_tests.sh`。
- 独立进程验证两套退出组合 0/0、非零/0、0/非零、非零/非零，确保第二套执行且汇总真实；环境加载失败应在测试前结束。
- 实际运行 `bash run_all_tests.sh` 一次，确认两个套件均有真实结果；既有测试失败保留归属，不为入口验收放宽断言。
- 回退只撤销本票对入口脚本的补丁，保留已有用户改动。

## 下一步

收到实施指令后在本票完成脚本修改和上述验证，再发布 resolution、关闭原票并更新地图索引。本轮没有新增取舍或新增票，不启动下一票。
