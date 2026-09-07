# OSCBF 延迟优化留存记录（2026-09-07）

研究按用户要求暂停；仅保留已有收益的实现与回归测试。完整系统的
20ms / miss≤1% 验收尚未达成。控制配置仍为 100Hz；20ms 是计算延迟预算。

## 保留的改动

- 障碍全禁用时跳过几何及导数；任意槽启用即恢复，保持 mask/soft-min 语义。
- OBB 标量距离用反向梯度构造 custom JVP，避免传播九份候选导数。
- 外层 JIT 合并路径模块，保留模块入口及真实缓存数。
- 缓存不可变默认输入；控制历史始终刷新，配置变化使缓存失效。
- 避免进入外层 JIT 前重复转换路径状态和标量增益。
- 线段距离用显式 2×2 Cholesky；保留 dpax 正则化、边界选择和梯度规则。
- 性能报告记录 20ms 超预算比例；隔离报告独立保存，失败也留证。
- perf 测试自给自足、e2e 按步数采样、settle 假对象与持久订阅实现同步。

## 已有证据与限制

接电、CPU 后端，无并发 demo/pytest。电池模式曾复现明显变慢，未调系统功耗。

| 测量阶段 | 结果 |
|---|---|
| 优化前接电完整 demo | 3,809 步，p95 24.814ms，miss 67.84%，QP 失败 0 |
| 默认输入缓存后完整 demo | 10,250 步，p50/p95/max 16.347/19.253/32.088ms，miss 2.43%，QP 失败 0 |
| 缓存后主包全量 | 250 passed；隔离 perf p95 9.287ms、miss 0% |
| 显式 2×2 求解后节点基准 | p50/p95 7.836/8.601ms；尚无此版本完整 demo 验收 |
| 2×2 数值、OBB 梯度和 FCL 回归 | 5 passed；512 组线段含平行、近平行及短线段，值/梯度容差 1e-10 |
| 最后一次 portable 全量 | 暂停时中断：76 passed、14 skipped；不是全量通过 |
| 清理后针对性回归 | 9 passed（20.50s），覆盖线段求解、OBB/FCL、默认缓存及障碍禁用分支 |

已有 #11 roll-only 严格容差失败未修复，定向复验仍出现。250 passed 在最终
2×2 求解加入之前；上述基准在清理恢复 SVD 之前，不能当成最终全量测试。

## 清理与复验

无运行时收益的 SVD→sqrt(det) 已恢复；device_get、遥测打包和线程数调整
均未保留。一次性探针与无效实验报告已删除。vendored dpax 未修改。

安装目录 portable 文件为源码符号链接，“未更新安装副本”的归因错误。
cProfile 数组同步包含等待内核的时间，不能当成纯转换成本。
demo 应核对报告步数和退出日志，重复 SIGINT 可能打断写报告。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH="src:$PYTHONPATH" python3 -m pytest tests/test_oscbf_controller_smoke.py::test_perf_report_p95_within_budget -q
# 测试退出后再跑 demo，限时只给 launch 发信号。
timeout --foreground --signal=INT --kill-after=15s 240s bash run_demo.sh
```

240 秒是采样窗口，不代表轨迹完成；此前 viewer/MoveIt 关停时出现过崩溃。
本地证据在 output/oscbf_main_final.log、output/oscbf_portable_final.log、
output/oscbf_m10_perf_demo_cached_pre_scalar.md 等；output 默认不进入 Git。
