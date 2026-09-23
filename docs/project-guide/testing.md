## 测试与已知问题

`run_all_tests.sh` 加载 ROS 与项目环境，依次运行主包 pytest、控制内核 pytest 和 AEB-RRT* 插件 ctest，分别汇总退出码。任一套件失败时返回失败。先执行 `bash build_aeb_moveit.sh`：

```bash
bash run_all_tests.sh
```

插件 ctest 登记 `test_aeb_rrtstar`、`test_aeb_full`、`test_plugin_load` 和 `test_plugin_init`，其中插件初始化测试从安装后的主包资源读取 URDF 与 SRDF。可以使用 `ctest --test-dir build/aeb_rrtstar_ompl -N` 检查登记结果。

已实施的测试工具修订（链接保留各自历史验收记录）：

| 项目 | 结论与状态 |
|---|---|
| [测试入口退出](../planning/oscbf-reuse/handoffs/10-test-entrypoint.md) | 已实施局部关闭 nounset 与双套件退出码汇总；验收记录见链接。 |
| [roll-only 路径起点测试](../planning/oscbf-reuse/handoffs/11-roll-only-tolerance.md) | 已修订精度、纯 roll / 倾斜对照与执行后报告断言；修订已归档，历史验收环境见链接。 |

最近一次 containment 产品代码完整验收基于 `e4d9a268`，对应证据见[验证归档](../planning/oscbf-reuse/validation/2026-09-11-containment/REPORT.md)。当前 checkout 的测试真值仍以实际运行 `bash scripts/agent_check.sh`（快速反馈）与 `bash run_all_tests.sh`（完整回归）为准；验证记录应注明 commit、日期、环境和退出码。[可复现基线](../planning/oscbf-reuse/handoffs/30-baseline.md)仅保留历史结果，不表示当前仍有同样失败。

最小纯软件 CI 入口：[Pure software checks](../../.github/workflows/pure-checks.yml)。
