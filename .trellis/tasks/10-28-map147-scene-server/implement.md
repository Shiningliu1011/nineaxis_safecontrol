# 执行与验证

1. 在独立工作区确认基点与 main 一致；读取当前任务、相关规范及 ADR，使用 `tdd` 实现公开边界。
2. 完成固定布局、唯一写入、原子发布、完整身份/来源恢复和 reader；运行实际多进程测试。
3. 完成真实场景管线、ROS node、配置与安装入口，执行真实 mesh/占据/tracking/prepare 组合测试及 ROS 进程集成。
4. 运行受影响消费者、lint/类型检查、安装后公开接口及文档检查，记录命令、原始结果与来源 hash。
5. 按 `code-review` 分别进行 Spec 与 Standards 独立审查，修复确认问题，由 merger 使用文件编辑工具集成至 main。
6. 主会话在 main 运行受影响检查并独立核对证据；达到条件后记录任务结果，未取得的证据明确保留。

临时结果及全部缓存位于各自工作区 `.scratch/`。本次代码、测试与相关文档按用户授权提交到 `main`，并推送到 `origin/main`。

## 当前实现验证

22 份工程、测试与文档已集成至 `/home/lsn/robot/robot_safecontrol` 的 main 工作区，纳入本次提交与远程同步范围；实现、审查和 main 文件的 SHA256 全部相等。本轮验收基点为 `8d5e938d4fe0185e07f17bc2359881157967baac`。实现分支为 `codex/map147-scene-server`，原独立工作区已清理，其完整忽略产物保存于 `.scratch/map147-104/worker-evidence/`。

公开测试边界为真实 writer/server 进程、真实 `/dev/shm` slot、只读 reader 刷新、真实 JAX PreparedScene，以及 PointCloud2→mesh self-filter→ray occupancy→fixed model/tracking/support→共享发布的组合。普通控制查询仅消费已经准备的本地引用。

- 全部当前受影响真实用例：`322 passed, 1 warning in 76.44s`，退出码 0；JUnit、完整输出、环境脚本、JSON 与源码 SHA-256 清单见 [实现验证记录](research/implementation-evidence.md)。
- 隔离安装完成；安装后的 `perception_bridge` 和 `collision_scene_server` 可执行程序使用默认内核路径，通过真实 ROS 进程收发与共享场景验收，输出 `INSTALLED_COLLISION_SCENE_PROCESS_ACCEPTANCE_PASSED`，退出码 0。reader 使用 `sources.create_reader()` 提供完整的预期来源。
- 新增入口、传输模块、处理管线、配置来源与公共路径解析入口通过 compileall；`git diff --check` 通过。13 份安装副本与对应源码的 SHA-256 全部相等，比较结果保存在机器清单中。
- 真实测试包含完整模型与时间来源恢复、唯一 writer、只读映射、writer 生命周期、checksum、slot 超时、revision 跳跃、复制期间 300 次切换及同 slot 返回、正常与异常退出、epoch 接受、每 revision 单次 device 准备、tracking 中断保留 occupied、原始 PointCloud2 字节保留、明确输入拒绝与构造失败资源清理。

来源身份覆盖实际加载的 CalibrationRecord 内容、ray 几何及验证、SelfFilterBounds、两种 ClockMapping、run 与相关来源，并在 header、revision 通知及 reader 预期中严格核验；同一路径标定文件变化也会拒绝。新 revision 复制及真实 device 准备前清除公开旧引用，设置 `REVISION_PENDING`；准备期间健康改变后继续拒绝。reader 对真实 FIFO 及时拒绝。server、bridge 与消费者示例通过 `package_portable_root()` 使用当前 Python 包对应的源码或安装资源。

冻结交接 JSON：`/home/lsn/robot/robot_safecontrol/.scratch/map147-104/worker-evidence/scene-handoff.json`，SHA-256 为 `b9d8f33923d8e8b706ee59b8bfaac7d2245f9dac4f49f589ad05598b976db2db`，包含 22 份相对路径与 hash、原 worktree/base/head/main、全部 JUnit、安装证据与环境脚本。JSON 内的原工作区 `.scratch/` 绝对路径对应当前 `worker-evidence/` 保存目录，文件字节与 hash 保持一致。

## 最终交付

main 重新执行同一受影响集合：`322 passed, 1 warning in 82.36s`，0 failure、error、skip；compileall、快速检查中的 69 项纯逻辑测试、差异检查和隔离安装通过。两个安装后的默认 ROS console 入口与完整 expected 来源 reader 验收通过，13 份安装文件与源码身份相等。独立 Spec 41 项及来源探针、Standards 27 项及边界探针均通过，当前两路没有确认发现，详见[审查记录](research/review.md)与[主分支验收](research/main-validation.md)。

[完成评论](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/104#issuecomment-6460673198)已发布并读取确认，本票为 `closed / completed`。地图及项目文档已同步。主会话清理本次独立 worktree 前保存所有忽略产物，随后使用保存目录核验 22 份 main 文件、18 份证据、两份 322 项 JUnit 与 13 份安装文件，`checks_passed=true`。本次代码、测试与相关文档按用户授权提交到 `main` 并推送远程；规范沿用现有要求，没有执行 spec promotion。

全量入口未运行：其中包含旧有 mock 机制，当前验证采用受影响的真实生产接口集合。现场参数、几何、时钟与真实 LiDAR 回放资格保持对应任务的验证范围。
