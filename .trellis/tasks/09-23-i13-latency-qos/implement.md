# 执行与验证记录

## 实施清单

- [x] 编写真实 ROS 2 双进程测量入口，记录状态发送、控制器完成、指令到达的关联事件。
- [x] 对状态订阅深度 5/20 与指令两端深度 5/20 运行四种组合，保存原始数据和运行条件。
- [x] 根据结果确定两条流的缓存条数，并用最终设置复测完整链路。
- [x] 在 `ros_conventions.py` 保存两条流的设置，更新仓库拥有的状态与指令端点及相关测试。
- [ ] 在项目文档和本票票尾评论记录测量结果、选值依据、证据位置与验收命令。
- [x] 运行受影响的 ROS 测试、`bash run_all_tests.sh`、`trellis-check` 和适用的代码审查。

## 计划运行的命令

- `python3 scripts/measure_qos_latency.py --base-domain 180`
- `python3 scripts/measure_qos_latency.py --reverse --base-domain 200`
- `python3 scripts/measure_qos_latency.py --production-qos --state-depths 20 --command-depths 5 --base-domain 220`
- `python3 -m pytest tests/test_oscbf_controller_smoke.py tests/test_oscbf_plant_smoke.py tests/test_oscbf_full_flow_e2e.py tests/test_hardware_bridge.py -q`
- `bash run_all_tests.sh`

## 实际结果

两轮四组对比和生产配置复测均完成。最终状态流 depth 20、指令流 depth 5。
完整测量方法、统计值和原始事件数据见
`docs/validation/qos-latency-2026-09-23.md` 与其链接的数据文件。
八组状态接收窗口均无状态未到达；生产配置复测得到 1,984 条完整关联样本，完整链路
p99 为 13.263 ms，状态未到达 0、指令未到达 1、时间逆序 0。`summary.json` 保留
运行时生效的源码与配置 SHA-256。

- 受影响测试：`37 passed in 74.68s`。
- 控制器与被控对象实际 ROS 端点 QoS：`2 passed, 19 deselected in 23.26s`。
- `python3 -m py_compile`（本次改动的 Python 文件）：通过。
- 生产 QoS 参数与常量不一致时，测量脚本拒绝运行，退出码 2。
- `git diff --check`：通过。
- `bash run_all_tests.sh`：主包 `544 passed, 1 skipped in 162.72s`；
  `portable_oscbf` 为 `178 passed, 34 skipped in 465.65s`；退出码 0。
- 规范与需求双线审查已完成，复核结果见 `research/review.md`，无待处理发现。

## 规范审查

状态流和指令流端点分别使用 `state_stream_qos()` 与 `command_stream_qos()` 是可复用的
项目约定，候选目标文件为 `.trellis/spec/backend/ros-interfaces.md`。用户尚未确认
规范内容和目标文件，本次保留规范文件原状。测量环境与取值依据已写入项目验证文档。
