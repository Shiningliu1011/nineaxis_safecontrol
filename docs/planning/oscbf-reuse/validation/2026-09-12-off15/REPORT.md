# OFF-15 本地离线验收

日期：2026-09-12。基线 HEAD：`744b44c82c17d668011afe3c462e0994c4da5c61`；在该工作区上实施，未提交/推送。开始时用户已有未跟踪 `AGENTS.md`，保留原状。实现后的文件身份见 [manifest.json](manifest.json)，运行环境见 [environment.json](environment.json)。

后续审查状态：以下为首次实现的历史验证记录。代码审查另复现区间外样本稀释误差、路径精度导致端点误判、报告同步生成阻塞控制回调三项问题；后续已按用户确认的决定修复，见[修复后的验证](../2026-09-12-off15-review-fixes/REPORT.md)及 [OFF-15 交接](../../handoffs/44-offline-tracking-evaluation.md#代码审查后的修复决策)。本页旧测试通过不能作为这些问题已解决的证据；本文件及相关决策文档的后续状态补充也不属于原 manifest 所绑定的版本。

## 结果

三项用户决定均已落地：取消综合分、真实工具轴夹角验收、允许预先声明子区间独立验收。任务质量、在线门控、范围完成及全部所需判据分别呈现；子区间通过不等于整条路径通过。默认 provisional 的安全和时延阈值仍给出证据不足。

| 检查 | 实际结果 | 日志 |
|---|---|---|
| 评价器、生产配置、逐类残差契约 | 92 passed，0.90 s | [contracts-final.txt](contracts-final.txt) |
| ROS 控制节点完整 smoke 文件 | 14 passed，76.01 s | [controller.txt](controller.txt) |
| 5D 工具轴、弹性 QP、弧长控制回归 | 14 passed，102.36 s | [portable.txt](portable.txt) |
| 报告样本哈希写盘、独立进度快照复核 | 2 passed，26.14 s | [report-writer.txt](report-writer.txt) |
| 六组解析场景与样本哈希 | 预期断言全部通过 | [examples-summary.json](examples-summary.json) |

前三组为 120 个不同用例；最后两项 ROS 测试是其中已有用例的复核，不重复计数。节点既有固定控制轨迹对照通过，未为本票放宽误差或安全断言。原来对未启用障碍返回的 `1.0` sentinel 断言已改为明确“未测”，避免把占位值当安全证据。

最终节点复核产生的[模型边界报告](controller-tracking.md)、[样本记录](controller-tracking.samples.json)和[运行快照](controller-runtime-snapshot.json)已一并保留，汇总与样本哈希匹配。报告内 `/tmp` 路径保留原采集位置，归档副本没有改写原记录。

## 实际命令

运行目录为仓库根目录。ROS 测试创建独立 DDS domain 中的短时测试节点，没有启动完整 demo、硬件 bridge、CAN 或真实机器人。相关日志和 JAX 缓存仅写入本次 `/tmp` 目录。

```bash
ROS_LOG_DIR=/tmp/off15-ros-log JAX_COMPILATION_CACHE_DIR=/tmp/off15-jax-cache python3 -m pytest tests/test_tracking_evaluator.py tests/test_oscbf_production_config.py portable_oscbf/tests/test_path_evaluation_diagnostics.py -q -x

ROS_LOG_DIR=/tmp/off15-ros-log JAX_COMPILATION_CACHE_DIR=/tmp/off15-jax-cache python3 -m pytest tests/test_oscbf_controller_smoke.py -q -x

JAX_COMPILATION_CACHE_DIR=/tmp/off15-jax-cache python3 -m pytest portable_oscbf/tests/test_jax_tool_axis_tracking.py portable_oscbf/tests/test_elastic_qp.py portable_oscbf/tests/test_jax_path_following.py -q -x

ROS_LOG_DIR=/tmp/off15-ros-log JAX_COMPILATION_CACHE_DIR=/tmp/off15-jax-cache python3 -m pytest tests/test_oscbf_controller_smoke.py::test_tracking_evaluator_integration tests/test_oscbf_controller_smoke.py::test_progress_snapshot_reports_tracking_state -q -x

PYTHONPATH=src python3 docs/planning/oscbf-reuse/validation/2026-09-12-off15/examples.py
git diff --check
```

上述最终检查均退出 0。完整 smoke 检查后补了样本记录写盘、独立运行的进度测试初始化、首样本实际起点提取及原 #14 净空非负下界，再对受影响的契约与两项节点检查复核通过；未机械重复无关套件。文档示例已实际执行，新增本地链接与 `git diff --check` 均通过。

## 中间失败与处理

1. 首次 ROS smoke 的日志目录是只读 `/home/lsn/.ros/log`，12 个 fixture 初始化错误；2 个不依赖该 fixture 的检查通过。JAX 缓存也提示默认目录只读。见 [controller-before-logdir.txt](controller-before-logdir.txt)。改用 `ROS_LOG_DIR` 和 `JAX_COMPILATION_CACHE_DIR` 指向本次 `/tmp` 路径后，14 个节点检查全部通过，没有修改产品日志目录或提高权限。
2. 单独复核进度快照时，旧测试依赖同文件的其他测试先发启动信号，因此出现 1 passed / 1 failed，见 [report-writer-order-dependency.txt](report-writer-order-dependency.txt)。已让该测试自行完成必要的启动和一次采样，独立重跑 2 passed；保留原有状态与数值断言。

## 解析证据

由 [examples.py](examples.py) 生成，每个场景均有 Markdown、汇总 JSON 与 samples JSON；脚本和样本身份在汇总中记录。

| 场景 | 任务结论 | 关键含义 |
|---|---|---|
| [完整路径](full-path.md) | 通过 | 路径已覆盖；默认安全/时延判据仍未定 |
| [0.6–1 m 子区间](subinterval.md) | 通过 | 区间完成，整条路径未覆盖/未验证 |
| [提前保持](held.md) | 不通过 | 仅推进到 50%，零进给不伪装完成 |
| [无效样本](invalid-sample.md) | 不通过 | 一个 NaN 样本被保留，不能删除后报告通过 |
| [准入拒绝](admission-rejected.md) | 不通过 | QP 全成功也不能抵消一次准入拒绝 |
| [工具轴反向](reversed-axis.md) | 不通过 | 180° 报真实夹角 π，不按 sin(π)≈0 判优良 |

纯逻辑测试另外覆盖空输入、缺失字段、正负 Infinity、零进度、没有正常完成事件、起点与声明不符、投影倒退、重复/倒退/缺失时间、来源边界混合、单位冲突、未定阈值、逐类残差和样本不可改写。

## 能力限制与回退

本票验证指标和离线报告能力。当前节点只产生内核模型边界的报告；滤波后命令、模拟状态、真实反馈及未接入的准入/重叠信息明确未测。逐类残差是原始候选在求解输入上的代数量，静态 RHS 派生裕度不是物理净空。实际执行链路、最终阈值、噪声长期分布、制动及实机准入继续由 OFF-09/OFF-13、原 #19 及现场票承接。

没有运行全仓 `run_all_tests.sh` 或启动实机；相关测试通过不扩大 sim/shadow/live 权限。回退本次改动时按 [OFF-15 交接](../../handoffs/44-offline-tracking-evaluation.md#回退与远端状态)成组撤销遥测接口与报告改动，保留用户文件。
