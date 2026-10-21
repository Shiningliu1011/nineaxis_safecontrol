# 完整计时集成回归与性能复验

运行源码为 `3771bc88deb14e38bc9c24624b2c881717af7190`，后续至 `2abfea8`
仅修改文档。环境入口 `.scratch/host-recovery/env.sh`，CPU0–15、domain230，
命令为根目录 `bash run_all_tests.sh`，服务 `static-final-regression-04`。

## 完整回归实际结果

- 主包：752 passed、1 failed、1 skipped，1029.82秒，退出1。
- 控制内核：全部文件执行结束，退出0。
- CTest：4/4通过，退出0。
- 总服务退出1，完整回归保持未全部通过。

日志：`output/static-capacity-20261020/final-regression-04.log`，SHA-256：
`bbfb5aa789c806882d7426b2a6f707eb2766a9fbf1be0652d7a5b3073eb1b7ff`。
全部测试产物保留在 `.scratch/regression-xiLHz4Py/`。

唯一断言失败为既有 `test_perf_report_p95_within_budget`：301个采样，
P95为16.445 ms，最大53.134 ms，6次超期，比例1.99%超过1%，QP失败零次。
日志还保留两条 `cannot use Destroyable because destruction was requested`
未取出异常诊断，正在独立核查其来源；不能据此推断生产原因。

## 独占性能复验

完整回归、播放ROS验证及其他计算退出后，使用相同环境、CPU0–15和原测试、
原阈值单独执行该用例。服务 `static-regression04-perf-check-01`，
实际1 passed/27.67秒，退出0。301个采样，P95为15.232 ms，最大17.851 ms，
超期及QP失败均为零。

日志 `output/static-capacity-20261020/regression04-perf-check-01.log`，SHA-256：
`c5eb76bc33c4fa2a293031bbe22c4e0af241ee63d0df47efab8f2dc27a59a439`。
报告 `.scratch/regression04-perf-check-01/test_perf_report_p95_within_bu0/perf.md`，
SHA-256：`f679ec6ef943f0e9b22f915d68c35c26ad6703dc65dd141f71b8252ba7ca4518`。
复验结果与完整回归失败原件分别保存，没有修改测试或生产阈值。

容量公共参数修复与数据播放的后续集成采用各自相关验证及独立审查；
三卷前置、当前容量和共同周期仍需在最终源码上实际执行。
