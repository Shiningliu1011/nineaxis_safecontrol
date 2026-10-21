# 串行接续入口暂停记录

用户已明确暂停实验至开题后，当前工作转为整理开题材料。
本研究入口停止开发，不启动实验、服务、等待队列或自动接续。
再次开展实验需要用户后续明确恢复要求。

保存的草稿为 [formal-serial-driver.py](formal-serial-driver.py) 和
[test_formal_serial_driver.py](test_formal_serial_driver.py)。工作树为
`/home/lsn/robot/robot_safecontrol/.scratch/worktrees/static-scenes`，
分支为 `codex/static-scenes`，开始本轮时集成版本为 `67d1766`。
没有修改 `src/`、`portable_oscbf/` 或当前 study02 helper。

当前入口仅实现只读准备：检查部分源码和小型输入身份，输出计划及真实 study 状态，
保留 1080 项计划、39/72 证明与预先选定的 12 个代表编号。
它没有 `--execute` 参数，也没有等待、冻结、容量绑定、batch 或出图执行逻辑。
准备检查尚未完成全部准入要求，不能作为正式启动依据。

暂停前使用真实现有数据完成的 TDD 记录位于本工作树
`.scratch/formal-driver-validation/`：

- `red-01.log`、`green-01.log`：默认准备入口；GREEN 为 1 项通过，
  真实 study02 为 pending，未创建正式输出目录。
- `red-02.log`、`green-02.log`：旧 study01 的计时边界拒绝；GREEN 为 2 项通过。
- `red-03.log`：3 项中 2 项通过、1 项失败。新增测试要求显式执行请求在旧源码下
  拒绝；当前尚无 `--execute` 参数，该 TDD 步骤保持未完成状态。

以上测试没有使用 mock 或伪造成功数据，没有启动 sim、性能测量或正式批次。
暂停保存时只进行 Python 语法检查与 diff 检查，不继续该失败测试的实现。
本 agent 没有存活的 driver 或测试进程；周期服务由主会话处理暂停。

原始数据继续保留在主仓库 `output/` 和两个 NTFS 任务目录：

- `/media/lsn/宇宙小机器人/robot_safecontrol_static_experiments_20261020/`
- `/media/lsn/Windows/robot_safecontrol_static_experiments_20261021/`

本轮没有读取或改写大型数据库、源码 ZIP 或已有实验结果。
运行来源仍为 `504a274740a1e0c27c597c8d61a889fe39154aed` 的 125 项源码映射，
三卷前置、容量和 39/72 证明的身份分别由既有
`complete-cycle-preflight-results.md`、`formal-proof-index-results.md` 记录。
后续恢复时应重新读取实际 study02 状态，不沿用暂停前的 pending 数量。
