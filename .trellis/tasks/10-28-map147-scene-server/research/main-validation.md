# 主分支验收

2026-10-28 在 `/home/lsn/robot/robot_safecontrol` 的 `main` 工作区完成验收，验收基点为 `8d5e938d4fe0185e07f17bc2359881157967baac`。merger 使用 `apply_patch` 集成 22 份文件，主会话核验全部文件 SHA256 与两路独立审查的冻结清单相等。本次代码、测试与相关文档纳入 `main` 提交及 `origin/main` 同步范围。

## 命令与结果

实际执行 `bash .scratch/map147-104/parent/run_main_checks.sh`，退出码 0。环境入口为同目录 `env.sh`，加载 ROS 2 Humble 与现有 Python 3.10 venv，使用 CPU JAX x64。全部临时文件、缓存、构建和日志位于仓库 `.scratch/`；共享场景用例使用独立 `/dev/shm` 资源。

- 11 份受影响测试文件合计 **322 passed, 1 warning in 82.36s**，0 failure、error、skip；完整文件列表保存在验收脚本与 [实现记录](implementation-evidence.md)。JUnit 为 `.scratch/map147-104/parent/main-junit.xml`，SHA256 为 `c7e1587e368b2e75c60f2f12f61a81536d7059f18281880ba0fce4da48eec523`。
- compileall 与 `scripts/agent_check.sh` 通过；后者包含 **69 passed in 1.11s** 的纯逻辑测试、纯模块导入、shell 语法和差异检查。该入口仅提供快速检查，当前行为依据上述 322 项真实测试。
- `setup.py egg_info/build/install` 在 `.scratch/map147-104/parent/package-install` 隔离安装成功。真实安装后的 `perception_bridge` 与 `collision_scene_server` 默认 console 入口通过原始 PointCloud2 收发、逐点来源、共享场景、完整预期来源的 JAX reader、单 revision 准备及无效输入撤销验收，退出码 0。
- 13 份安装 Python 入口、共享模块、配置、launch、package.xml 与模块文档的 SHA256 与当前源码一致。
- `python .scratch/map147-104/parent/verify_delivery.py --integrated` 核验源码、实现者 18 份证据、两份 322 项 JUnit 和安装副本，返回 `checks_passed=true`。清理独立 worktree 后，增加 `--worker-archive .scratch/map147-104/worker-evidence` 实际重跑核验并通过；当前验收脚本采用该保存路径。机器结果保存于 `.scratch/map147-104/parent/main-verification.json`。

唯一测试 warning 来自现有 Matplotlib Axes3D 导入环境。源码与安装的 ROS 验收均未提供 `portable_oscbf_root` override；安装进程日志位于 `.scratch/map147-104/parent/installed-processes/`，完整输出为 `installed-processes.log`，其 SHA256 为 `0f06ef22bcf4cf47021b4b35ff1c6f354ed751a25f63da4ac8f69a52300c7762`。

## 需求与规范

独立 [Spec](review-spec.md) 的 41 项公开检查和来源探针通过；独立 [Standards](review-standards.md) 的 27 项回归与 pending/FIFO/源码和安装默认入口探针通过。当前两路均无确认发现。完整冻结交接 JSON 的 SHA256 为 `b9d8f33923d8e8b706ee59b8bfaac7d2245f9dac4f49f589ad05598b976db2db`。

本次实现遵循既有来源、时间、安全与资源规则，没有新增规范决定，未执行 spec promotion。用户已授权将本次代码、测试与相关文档提交到 `main` 并推送远程。同步前重新核验冻结源码、JUnit 与安装证据，并保留全部本地原始产物。

完整测试入口含旧有 mock 机制，本轮采用上述受影响真实集合。现场参数、真实 LiDAR 回放、目标设备 deadline、正式控制来源切换和物理能力分别需要对应任务证据。

## 工单

已发布[完成记录](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/104#issuecomment-6460673198)，并读取确认完整评论及 `closed / completed` 状态。M11 厂家资料核验见 [M11 记录](m11-context.md)，设备任务保持开放。

原实现 worktree 已清理，全部忽略产物保存在 `.scratch/map147-104/worker-evidence/`；交接 JSON 原绝对路径中的实现工作区 `.scratch/` 对应该目录。来源、JUnit、安装和日志文件的 SHA256 均已按当前路径重新核验。
