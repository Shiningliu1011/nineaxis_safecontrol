# 周期开发记录与接续状态

`guarded-period-study-01` 的20 ms/A和20 ms/B均完成原定运行与报告保存，
两个运行的worker退出码均为 `[0,0]`，最终均按timeout终止。
队列主进程停止后续分派，等待B完整结束后于2026-10-21正常结束服务；
没有启动C及后续组合，原始数据库、报告和源码身份全部保留。

原件目录为
`/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/guarded-period-study-01/`。
运行源码为 `40d43bb`，source mapping与已核查的S2源码归档相同。
原始duration的终点在cycle记录写入前；完整周期计时采用
[完整周期计时设计](complete-cycle-timing-design.md)，后续使用同一新版口径重新测量全部20组。
这两份基础计时开发记录不用于新版正式周期准入，正式采集保持0/1080。

归档清单为
`output/static-capacity-20261020/guarded-period-study-01-stopped.json`，
SHA-256 为 `967080684db0e65c79c7ff6f5e83e58741ead47c86c7ddc0062af22dc5a6f872`。
其中保存两份运行的完整文件校验值、原报告中的数据库校验值、终止状态、
基础计时及执行脚本归档身份，没有重新编码或改写数据库。

| 原始文件 | SHA-256 |
| --- | --- |
| A `feedback_report.json` | `26f991bedc3acd9ea8214d3c059299439f25a7fa0f8cb3e6adb3773456f0fd9d` |
| B `feedback_report.json` | `1aef5725297bfd65d3109dc07a2cf6244738856bbf0d46455a7217d497d6b1a7` |
| B `result.json` | `31b3e6adf81312c97d28efd78f8a6b94c4b0418f9e9dfe05f033ff5da50d627f` |

接续顺序为：在独立worktree完成完整计时及sidecar容量处理、真实定向测试，
集成后重核受影响的记录生命周期与容量来源，再使用新目录执行共同周期测量。
历史物理可行性证据保持原始来源；功能、时间及容量分别依据各自实际验证判定。
