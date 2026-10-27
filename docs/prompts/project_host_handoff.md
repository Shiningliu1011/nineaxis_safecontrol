# Linux 主机接手提示词

在主机 Codex 中打开复制好的 `robot-safecontrol-transfer` 目录，
发送以下内容：

```text
请在这台 Linux 主机上接手 robot_safecontrol 项目，完成项目恢复、环境与 skills 核验，并依据完整项目地图继续主线工作。

我已把 robot-safecontrol-transfer 迁移资料文件夹复制到这台机器。读取 README.txt、SKILLS-INSTALL.md、ENVIRONMENT.md 和 PACKAGE-INFO.json，校验 SHA256SUMS。默认恢复到 ~/robot/robot_safecontrol；目标已有文件时检查内容并保留已有改动。资料包含完整项目、Git 历史、实验与标定记录、本地资料、个人 skills 和字体。恢复后以主项目目录作为工作目录。

远程仓库：https://github.com/Shiningliu1011/nineaxis_safecontrol.git
工作分支：main
归档对应提交：68a52279423318d3773e8d6f61c21c10f1d53a24

仓库由 lsn 一人在实验室笔记本和课题室工位台式机之间交替开发。开始工作时获取远程更新；结束工作或切换设备前，更新接手记录并提交、推送本次相关改动。两台设备通过 GitHub 同步代码、文档、任务进度及接手记录。大型原始数据保存在设备本地，记录数据位置、来源身份和校验值，需要时另外传输。

恢复后检查 Git 状态、远程地址及分支，获取 origin，将 main 快进到 origin/main。归档恢复到其他分支时，核对已有提交与改动，合并有效工作后使用 main。两边都有新提交时，检查差异并使用普通合并处理。读取最新 docs/project_migration.md 和 docs/two_machine_workflow.md。

按以下顺序实际执行：

1. 阅读本机及项目适用的 AGENTS.md，随后读取 docs/planning/nineaxis-full-project/README.md、dependencies.md 和 docs/specs/predictive-coordination-control.md。查询 GitHub #147 的当前子任务与原生前置，沿用对应 Trellis 任务。
2. 按 SKILLS-INSTALL.md 阅读并安装个人 skills，核查项目 .agents/skills/ 和 .codex/ 配置。相同内容复用，已有不同版本保留并说明差异。按项目需要安装 Trellis CLI，保留原有任务与规范。
3. 检查发行版、CPU、内存、磁盘、Python 和 ROS。结合 environment/ 清单、项目文档与固定依赖文件配置兼容 ROS 2 Humble 的环境，重新构建原生 FCL 模块和 ROS package，安装绘图依赖、字体及项目需要的本地资料检索环境。完成实际导入、CLI、构建和相关测试。需要管理员操作或登录时说明具体原因与操作。
4. 核对 NURBS、MATLAB、标定、模型及已有记录的保存位置和校验值，保留原始数据。前期证据入口为 docs/static_obstacle_experiments_handoff.md。
5. 根据任务实际前置继续工作；当前入口为 #148 基线与 M01–M12 资产核验、#135 工件路径资产核验，#95 标定由 lsn 跟进。使用实时 GitHub 状态确认可推进范围。
6. 记录本机环境、所用提交、实际命令、验证结果、数据位置和尚缺条件。项目输入与报告使用 mm/°，ROS、URDF 和控制内核使用 m/rad。

静态实验 #171、#182、#183 已取消。主线三方法共同条件由 #144 定义，联调、周期测量与正式统计由 #169 负责。真实设备动作按对应现场任务授权执行。

我授权项目恢复、相关 skills 和环境安装、主线任务所需的本地核验，以及按上述流程提交和推送本次相关改动。完成实际验证并保存跨设备接手记录。
```
