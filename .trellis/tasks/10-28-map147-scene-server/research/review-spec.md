# Spec 审查

审查日期：2026-10-28。工作区为 `/home/lsn/robot/robot_safecontrol/.scratch/worktrees/map147-scene-server`，分支为 `codex/map147-scene-server`，固定基点与 HEAD 均为 `8d5e938d4fe0185e07f17bc2359881157967baac`。审查对象为 11 份已跟踪文件变更及 11 份新文件；本轮复核最终来源身份、共享读取状态及安装资源入口增量，结合已完成的完整需求审查。工程文件保持原状。

源码已按相同 SHA256 集成至 main，原审查 worktree 已清理，全部忽略产物保存在 `.scratch/map147-104/worker-evidence/`。本报告记录审查时的实际命令；当前完整复现入口与 main 结果见[主分支验收](main-validation.md)。

## 当前结论

本轮未确认仍需修复的 Spec 缺陷。

`CollisionSceneSources.source_identity` 根据实际加载的标定、几何与验证、SelfFilterBounds、两种 ClockMapping、运行、反馈、零位、历史容量、测量范围、policy、grid 和固定模型计算。writer 将其绑定共享 header 与 SceneRevision；`sources.create_reader()` 提供同一来源身份，reader 在复制和 JAX 准备前核验。来源不一致返回 `INVALID_SCENE / source_identity_mismatch`，并且不提供 PreparedScene。

新 revision 在真实 device 准备前撤销公开旧引用并进入 `REVISION_PENDING`。共享唯一写入、双缓冲原子发布、完整时间模型及固定模型恢复、epoch 接受、异常清除准入和每 revision 单次 device 准备的实际测试通过。当前 Python 包对应的 `package_portable_root()` 由源码默认入口实际执行。

## 需求与核验范围

已核对任务 `prd.md/design.md/implement.md` 与 implement/check context、GitHub「[24] 实现 collision_scene_server 与双缓冲共享场景」正文、ADR 0037/0038/0041、规格决定 29–32 和 Testing Decisions、已确认的 decision-139 场景与 decision-142 时间要求，以及适用项目规范。

复核包含实际对象内容身份在同一路径标定变更、LiDAR/关节映射、run、SelfFilterBounds、ray 几何与验证变化时的拒绝，以及等价来源配置在不同 manifest 路径和 JSON 排列下的接受。公开测试同时核验未绑定来源的通用 reader 不能接受生产来源身份。

## 实际验证

工作目录为上述工作区。以下组合命令已经执行，退出码为 **0**：

```bash
source /home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-spec/env.sh
python /home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-spec/provenance_probe.py > /home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-spec/provenance-final-result.json 2> /home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-spec/provenance-final-stderr.log
python -m pytest portable_oscbf/tests/test_shared_collision_scene.py tests/test_collision_scene_server.py tests/test_timed_lidar_transport.py tests/test_lidar_self_filter_adapter.py --basetemp /home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-spec/pytest-final -q > /home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-spec/public-tests-final.log 2>&1
```

来源探针使用真实公开 pipeline、实际 mesh、自体过滤、占据、关节历史、`/dev/shm` 与 JAX。输出中 matching reader 为 `OK`，不同 CalibrationRecord、ClockMapping 和 run 的 reader 为 `INVALID_SCENE`，原因为 `source_identity_mismatch`。结果保存在 [provenance-final-result.json](/home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-spec/provenance-final-result.json)，stderr 为空。

四份公开接口测试结果为 **41 passed，12.13 s**，日志为 [public-tests-final.log](/home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-spec/public-tests-final.log)。它们使用真实独立 writer/bridge/server 进程、真实 ROS PointCloud2/JointState 消息、真实共享内存、系统原子操作和 JAX。覆盖唯一写入、FIFO 立即拒绝、重启、revision 跳跃、checksum、slot 超时、复制期间切换、准备期间撤销引用、正常与异常退出、完整来源恢复和每 revision 单次复制。输出包含一条 Matplotlib Axes3D 导入警告，全部测试通过。

## 当前文件身份

22 份工程文件的完整 SHA-256 保存于 [final-source-hashes.sha256](/home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-spec/final-source-hashes.sha256)。实际执行以下命令，退出码为 **0**，全部 22 份文件均为 `OK`：

```bash
sha256sum -c /home/lsn/robot/robot_safecontrol/.scratch/map147-104/review-spec/final-source-hashes.sha256
```

本轮主要增量的 SHA-256 为：

| 文件 | SHA-256 |
|---|---|
| `portable_oscbf/work/shared_collision_scene.py` | `fcb7a70794218523dd4da9b71f27abc96a83cfd3a7c49dc29fec731f6c5d2157` |
| `src/robot_safecontrol_moveit/collision_scene_sources.py` | `8dee195ccdaf7cb5213c5fd404151e36dcaf15ba7263a72536bf37603f8e2736` |
| `src/robot_safecontrol_moveit/collision_scene_server.py` | `9a08ad7143d2ac0f648772c4cf881a694e96b3ff18b8bdc77304359aef0186da` |
| `src/robot_safecontrol_moveit/oscbf_trajectory.py` | `444c46328753bfbe34310b99ed0cbaedcd80dbac0920017f1acbb3682e36e2c6` |
| `tests/test_collision_scene_server.py` | `4d6764c6c81d063bb96c026e0082bd4bda2cded151d2772c07633ac426001474` |
| `portable_oscbf/tests/test_shared_collision_scene.py` | `8ee8f0de606a2b497d2ab25826769cc84a8344c5d97d05d0c52c4b5f6d259808` |

原始审查报告与日志保留在 `.scratch/map147-104/review-spec/`。本报告的独立运行结果为上述 41 项测试与来源探针；安装制品验证由主会话及实现者承担。本次未执行设备 I/O、commit、push、PR 或工单写入。解析输入证明软件接口，不提供现场 LiDAR、设备参数或硬件准入结论。
