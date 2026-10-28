# Standards 审查记录

审查日期：2026-10-28。工作区为 `/home/lsn/robot/robot_safecontrol/.scratch/worktrees/map147-scene-server`，分支为 `codex/map147-scene-server`。固定基点与 HEAD 为 `8d5e938d4fe0185e07f17bc2359881157967baac`，工程变更未提交。

当前 Standards 复核未发现需要修复的问题。

## 来源与范围

审查覆盖全部 22 份工程变更，以及本次共享读取状态、文件类型检查和公共路径入口的增量。依据为项目工作约定、项目 `code-review` Skill、`docs/CODING_STANDARDS.md`、受影响的 backend 规范与 guides，以及 ADR 0037、0038、0041。

22 项来源校验清单为 [scene-source.sha256](/home/lsn/robot/robot_safecontrol/.scratch/map147-104/worker-evidence/scene-source.sha256)。审查时实际执行 `sha256sum -c .scratch/scene-source.sha256`，全部 22 项通过，退出码 0。工程现已集成 main，原工作区已清理；全部忽略产物保存在 `worker-evidence/`，当前验收入口见[主分支验收](main-validation.md)。清单本身的 SHA-256 为：

```text
379a08a0bc5919094cfcbb987f37d2cc81b03f75d1249e3bd3257d378868f263
```

这是当前 22 份文件内容的清单身份，校验输出位于 [source-check.log](/home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-standards/final/source-check.log)。

## 当前行为核验

- **复制与 device 准备期间的公开状态**：[shared_collision_scene.py:520](/home/lsn/robot/robot_safecontrol/portable_oscbf/work/shared_collision_scene.py:520) 在发现不同发布字后、读取 header 和复制 slot 前，把公开结果设为 `REVISION_PENDING` 并清除场景引用；第 575–579 行在真实 device 准备前保持 pending，最终检查成功后才在第 593 行交接新引用。公开探针实际得到 `writer_revision=2`、`reader_status_during_prepare=REVISION_PENDING`、`reader_revision_during_prepare=2`、`same_old_local_scene=false`，继续真实准备后返回 OK。回归同时覆盖准备期间 writer 失效后的引用撤销。符合 [ADR 0037:22](/home/lsn/robot/robot_safecontrol/docs/adr/0037-double-buffered-shared-scene.md:22)。证据为 [pending.log](/home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-standards/final/pending.log)，退出码 0。

- **FIFO 拒绝**：[shared_collision_scene.py:458](/home/lsn/robot/robot_safecontrol/portable_oscbf/work/shared_collision_scene.py:458) 采用 `O_NONBLOCK`，在 mmap 前检查 `stat.S_ISREG`，并在 `finally` 中关闭 fd。独立进程调用真实 reader 构造器后立即得到 `ValueError: 共享场景需要普通文件`，没有持续等待。符合 [错误处理:5](/home/lsn/robot/robot_safecontrol/.trellis/spec/backend/error-handling.md:5)。证据为 [fifo.log](/home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-standards/final/fifo.log)，退出码 0。

- **对应源码与安装资源**：[oscbf_trajectory.py:18](/home/lsn/robot/robot_safecontrol/src/robot_safecontrol_moveit/oscbf_trajectory.py:18) 的公共 `package_portable_root()` 根据当前 Python 包位置选择资源；[collision_scene_server.py:36](/home/lsn/robot/robot_safecontrol/src/robot_safecontrol_moveit/collision_scene_server.py:36)、bridge 与文档采用该入口。源码默认入口实际选择对应工作区的 `portable_oscbf`；隔离安装的真实 console 入口实际选择对应 `.scratch/package-install/share/robot_safecontrol_moveit/portable_oscbf`。两次启动均未提供 `portable_oscbf_root` override，均出现 `COLLISION_SCENE_SERVER_READY`，随后由验证程序发送 SIGINT，正常退出 0。符合 [配置与生成文件:59](/home/lsn/robot/robot_safecontrol/.trellis/spec/backend/configuration-and-generated-files.md:59)。证据为 [source-default.log](/home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-standards/final/source-default.log)、[installed-default.log](/home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-standards/final/installed-default.log)。日志中的原实现工作区 `.scratch/` 路径对应当前 `worker-evidence/`。

## 本次实际验证

环境为现有 ROS 2 Humble、项目 venv、CPU JAX 与 `JAX_ENABLE_X64=1`。环境脚本为 [env.sh](/home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-standards/env.sh)。缓存、临时文件、日志和产物均位于主仓库 `.scratch/map147-104/review-standards`；默认入口探针使用本机隔离 domain 189。

工作目录为 `/home/lsn/robot/robot_safecontrol/.scratch/worktrees/map147-scene-server`。实际命令如下，各命令均退出 0：

```bash
source /home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-standards/env.sh
export REVIEW_SCENE_ARTIFACT_DIR=/home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-standards/final
python /home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-standards/verify_boundaries.py pending
python /home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-standards/verify_boundaries.py fifo
python /home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-standards/verify_default_entry.py source
python /home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-standards/verify_default_entry.py installed
python -m pytest portable_oscbf/tests/test_shared_collision_scene.py tests/test_timed_lidar_transport.py tests/test_lidar_self_filter_adapter.py --basetemp /home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-standards/final/pytest -q
sha256sum -c .scratch/scene-source.sha256
sha256sum .scratch/scene-source.sha256
```

本次独立回归结果为 `27 passed, 1 warning in 6.56s`，原始输出位于 [pytest.log](/home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-standards/final/pytest.log)。警告为现有 Matplotlib 的 Axes3D 导入提示。验证使用真实共享资源、独立进程、系统原子操作、真实 ROS 消息与 JAX；方法入口的观察使用 `sys.setprofile`，未替换实现、返回值或数值计算。

本次还读取了实现者的 [scene-final-pytest.log](/home/lsn/robot/robot_safecontrol/.scratch/map147-104/worker-evidence/scene-final-pytest.log)，其中记录 `322 passed, 1 warning in 76.44s`；[installed-scene-final.log](/home/lsn/robot/robot_safecontrol/.scratch/map147-104/worker-evidence/installed-scene-final.log) 记录完整安装场景进程验收成功。这两项属于检查已有证据，本次没有重复运行完整集合或 domain 184 的桥接进程验收。

工程、工单、Git 历史和设备状态保持原状。原始审查日志保留于 `.scratch/map147-104/review-standards/`，本次复核证据保存在其 `final/` 目录。
