# Linux 主机接手提示词

在主机 Codex 中打开复制好的 `robot-safecontrol-transfer` 目录，
发送以下内容：

```text
请在这台 Linux 主机上接手 robot_safecontrol 项目，完成项目恢复、环境与 skills 安装，并优先解决实验数据占用空间的问题。

我已把 robot-safecontrol-transfer 迁移资料文件夹复制到这台机器。读取 README.txt、SKILLS-INSTALL.md、ENVIRONMENT.md 和 PACKAGE-INFO.json，校验 SHA256SUMS。默认恢复到 ~/robot/robot_safecontrol；目标已有文件时检查内容并保留已有改动。资料包含完整项目、Git 历史、实验与标定记录、本地资料、个人 skills 和字体。恢复后以主项目目录作为工作目录。

远程仓库：https://github.com/Shiningliu1011/nineaxis_safecontrol.git
工作分支：codex/static-obstacle-experiments
归档对应提交：68a52279423318d3773e8d6f61c21c10f1d53a24

项目长期在实验室笔记本和课题室工位台式机之间交替开发。开始工作时获取远程更新；结束工作或切换设备前，更新接手记录并提交、推送本次相关改动。两台设备通过 GitHub 同步代码、文档、任务进度及接手记录。大型原始数据保存在设备本地，记录数据位置、来源身份和校验值，需要时另外传输。

恢复后检查 Git 状态、远程地址及分支，获取 origin，将工作分支快进到 origin/codex/static-obstacle-experiments。已有改动或两边都有新提交时，保留内容并检查差异，使用普通合并处理，不重写共享历史。读取最新 docs/project_migration.md 和 docs/two_machine_workflow.md。

按以下顺序实际执行：

1. 阅读本机及项目适用的 AGENTS.md、包内原机器工作约定，然后阅读 docs/static_obstacle_experiments_handoff.md、docs/static_obstacle_experiments.md、docs/specs/static-obstacle-experiments.md，以及 .trellis/tasks/10-20-static-formal-experiments/ 下的 prd.md、design.md、implement.md。沿用已有 Trellis 任务和 inline 工作方式。
2. 按 SKILLS-INSTALL.md 阅读并安装个人 skills，核查项目 .agents/skills/ 和 .codex/ 配置。相同内容复用，已有不同版本保留并说明差异。按项目需要安装 Trellis CLI，保留原有任务与规范。
3. 检查发行版、CPU、内存、磁盘、Python 和 ROS。结合 environment/ 清单、项目文档与固定依赖文件配置兼容 ROS 2 Humble 的环境，重新构建原生 FCL 模块和 ROS package，安装绘图依赖、字体及项目需要的本地资料检索环境。完成实际导入、CLI、构建和相关测试。需要管理员操作或登录时说明具体原因与操作。
4. 优先检查现有 SQLite、JSON 数值记录、重复状态及派生数组的空间占用，实施可验证的紧凑保存方式。完整保留原始反馈、控制决策、事件、失败记录、采样信息和数值精度；按需生成可重算数据。同步处理报告身份校验、重放、异常退出恢复及文档。
5. 使用已有真实 S0、S2、S4、S5 记录的副本验证内容：逐字段核查记录数量、时间戳、数值和状态，独立重算指标，核查中断恢复，测量读写开销及真实控制周期。原始记录继续保留。
6. 根据真实测量报告每次运行大小、1,080 次正式矩阵的预计容量、临时处理峰值、可用空间和余量。旧格式的未压缩估算约 202 GB，ZIP/LZMA 估算约 73 GB；新的空间需求必须实际验证。存储方案与实验前置条件通过之前，保持正式采集停止。

#171 尚未完成验收。D/S2 有完整通过记录；S4 实际暂停、构型调整与恢复，S5 当前参数复核、共同周期测量、正式实验和完整回归仍有未完成项。保持 sim，保留失败记录和全部验收条件，依据本机性能重新测量。

我授权项目恢复、相关 skills 和环境安装、本地存储代码修改、非破坏性验证，以及按上述流程提交和推送本次相关改动。实际执行并记录验证结果，本次优先交付可工作的主机环境和经过验证的存储方案。
```
