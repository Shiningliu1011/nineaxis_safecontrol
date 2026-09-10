# roll-only 路径起点诊断证据

对应 [实施交接](../../handoffs/11-roll-only-tolerance.md)。采集于 2026-09-11，代码基线 `786e0aec61b7abbf8e8c0a5c5aa99d20c3e54f21`，CPU 后端。

`probe.py` 从原失败测试提取，仅做离线内核计算，不含 ROS/CAN 发布。默认精度和 x64 日志由初版探针产生；详细版增加 PRE_ERROR、SAFE_MAX 和 DQ_MAX 输出，对应 x64-detail.txt。没有将历史日志当作最终版探针重新运行的结果。

复现命令从仓库根目录执行，见交接。日志保留原始数值；这些证据不代表生产控制或实机验收。
