# 03 — run_all_tests.sh 在 set -euo pipefail 下直接退出

**What to build:** `run_all_tests.sh` 在 `set -euo pipefail` 下 source
`/opt/ros/humble/setup.bash` 触发 `AMENT_TRACE_SETUP_FILES: unbound variable`
（setup.bash 第 8 行），导致脚本不复用即退出、主包测试根本没跑。
修复：source 前导出 `AMENT_TRACE_SETUP_FILES=1`（或临时 `set +u`），并让脚本
在失败时汇总两个套件的 exit code。

**Blocked by:** None — 单文件修复。

**Queue:** implementation-backlog（已移出 Wayfinder）
**Tracker:** #10 (https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/10)

**Status:** ready-for-agent

- [ ] 修复 source 处的 unbound variable
- [ ] 脚本对 main 与 portable 两个套件分别记录结果而非提前中止
- [ ] 验证 `bash run_all_tests.sh` 一次跑完两个套件
