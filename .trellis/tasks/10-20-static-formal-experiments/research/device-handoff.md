# 双设备接手记录

用户要求实验室笔记本与课题室工位台式机通过 GitHub 交替开发。
已授权本次项目资料整理与远程同步，当前继续停止新增实验采集。

远程地址：`https://github.com/Shiningliu1011/nineaxis_safecontrol.git`

工作分支：`codex/static-obstacle-experiments`

迁移资料位于原机器
`/home/lsn/robot/robot_safecontrol/output/robot-safecontrol-transfer/`。
9 个分卷共 26,774,385,704 字节，包含 86,374 个目录项，
普通文件为 42,737,916,946 字节。归档对应提交为
`68a52279423318d3773e8d6f61c21c10f1d53a24`；
恢复后获取远程工作分支，以读取后续迁移文档。

包内保留 `output/static-experiments/` 全部现有运行，以及场景选择、
见证请求、实验检查日志、项目相关目录、Git 历史、标定数据和本地资料。
支持资料包含 25 个个人 skills、原机器工作约定、绘图字体、厂商安装包；
项目内包含 20 个 skills。系统与 Python 依赖版本保存在 `environment/`。

所有归档普通文件与来源逐字节核查通过，归档包含范围与来源清单一致；
主项目及参考项目的 Git 连通性检查通过，支持资料与来源核查通过。
恢复脚本通过 Bash 语法检查。主机环境安装与构建须在目标设备上实际验证。

接手文档：

- `docs/project_migration.md`
- `docs/two_machine_workflow.md`
- `docs/prompts/static_experiments_host_handoff.md`
- `docs/static_obstacle_experiments_handoff.md`

下一步是主机恢复、环境与 skills 安装、真实记录的紧凑保存验证及容量测量。
正式 1,080 次采集尚未开始，当前任务保持 `in_progress`。
本次没有修改控制代码或运行新增实验，也没有执行任务归档或 spec promotion。
