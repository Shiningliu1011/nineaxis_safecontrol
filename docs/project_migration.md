# 项目迁移到 Linux 主机

远程仓库：`https://github.com/Shiningliu1011/nineaxis_safecontrol.git`  
当前工作分支：`codex/static-obstacle-experiments`

在主机 Codex 中打开迁移资料文件夹，发送
[主机接手提示词](prompts/static_experiments_host_handoff.md)。
提示词要求完成文件校验、项目恢复、读取接手记录、安装 skills 和环境，
并优先验证实验数据的紧凑保存方式。
长期协作方式见[双设备工作流程](two_machine_workflow.md)。

## 本次迁移资料

原机器资料目录为
`/home/lsn/robot/robot_safecontrol/output/robot-safecontrol-transfer/`。
将整个目录复制到 U 盘，再复制到主机；保留所有分卷及其文件名。

项目归档共 9 个分卷，压缩数据为 26,774,385,704 字节，约 26.77 GB。
单个分卷最多 3,000 MiB，可用于 FAT32 文件系统。
连同说明、skills、字体和校验清单，资料目录约 26.91 GB。
U 盘需要至少 27 GB 可用空间。

归档普通文件合计约 42.74 GB。主机同时保存迁移包和恢复后的文件时，
需要约 70 GB 空间；环境安装、编译和后续实验另计。
恢复脚本也可以直接从 U 盘读取分卷，将项目写入主机的 Linux 文件系统。

归档包含以下工作目录及其内容：

- `robot_safecontrol`：源码、Git 历史、Trellis 任务、项目 skills、
  原始实验记录、标定 bag、图片、本地研究和电机资料。
- `robot_oscbf`、`pymoveit2_ws`：相关参考代码和工作目录。
- `xy-mid-360-s`、`lidar`：传感器相关资料。
- `交付`、`tmbs-main.zip`：原工作目录中的项目资料。

可重新生成的构建目录、Python 环境、缓存和重复备份按包内
`archive-excludes.txt` 排除。个人 skills、工作约定、绘图字体和已有厂商安装包
保存在 `support/`，软件版本保存在 `environment/`。

## 恢复与获取更新

包内 `README.txt`、`SKILLS-INSTALL.md`、`ENVIRONMENT.md`
分别说明恢复、skills 安装和环境配置。`PACKAGE-INFO.json` 记录实际文件清单、
归档提交、远程提交和验证结果。

安装 Git、zstd、GNU tar、Python 3 后，在迁移资料目录执行：

```bash
bash restore-project.sh "$HOME/robot"
cd "$HOME/robot/robot_safecontrol"
git status --short --branch
git remote -v
git fetch origin
git merge --ff-only origin/codex/static-obstacle-experiments
```

恢复脚本自动校验 `SHA256SUMS`，并检查目标中的同名项目尚不存在。
归档对应提交为 `68a52279423318d3773e8d6f61c21c10f1d53a24`。
本迁移说明与双设备流程由该提交之后的远程分支提供，恢复后获取最新分支即可读取。
如果主机已有工作区，按照[双设备流程](two_machine_workflow.md)检查并保留现有内容。

## 环境与实验

原机器为 Ubuntu 22.04、ROS 2 Humble、Python 3.10。
依据主机实际系统和项目固定依赖文件安装环境，重新构建原生 FCL 模块及 ROS package。
25 个个人 skills 和 20 个项目 skills 随资料提供；安装前阅读各自说明和项目路由。
主机已有不同版本配置时，保留现有内容并检查差异。

[实验接手记录](static_obstacle_experiments_handoff.md)说明现有验证和未完成项。
正式 1,080 次采集保持停止；必须完成存储内容一致性、容量、重放、
异常恢复及耗时验证，并满足实验前置条件后再评估是否开始。
实验继续限定在 sim，设备性能依据主机实际运行重新测量。

## 本机验证

已检查归档的 86,374 个目录项，全部普通文件与来源逐字节一致，
目录、链接及包含范围与来源清单一致。主项目与参考项目的 Git 连通性检查通过。
支持资料与来源内容一致，恢复脚本通过 Bash 语法检查。
主机的系统安装和构建需要在实际设备上完成验证。
