# 执行与验证

1. 核验 Git 基准、#101/#109、ADR、已确认 #139 与 #108 当前依赖，记录来源 hash。
2. 真实公开接口 red/green：原观测与 untracked 未来增长；有效 track 与中断恢复；完整合并范围；逐行查询与时间导数。
3. 检查 checksum、身份、有效期、ns 边界、范围容量和无效输入，保持 certify 拒绝。
4. 运行受影响占据、准备、碰撞、参数与解析几何回归，检查 Ruff、类型及安装副本。
5. 更新模块文档，保存文件清单、SHA-256、命令、退出码及证据到主仓库 `.scratch/map147-101`。

测试边界：真实 LidarRayBatch → LidarOccupancyScene → CollisionScene → prepare_scene → query，以及公开 reachable_support_tube。使用有明确解析来源的参数和 JSON 回放，不使用 mock。独立公式与中心差分提供预期值。

修改为碰撞安全相关内容，交付后由主会话安排独立审查与集成。全部修改保持未提交；没有 commit、push、PR、归档或 spec promotion 授权。

## Red/green 证据

命令入口为主仓库 `.scratch/map147-101/run-tests.sh`，其明确使用当前工作区、JAX x64、项目 venv 及独立中间目录。

| 行为 | red 日志，退出码 1 | green 日志，退出码 0 |
|---|---|---|
| 原观测与未跟踪未来范围 | `red-untracked.log` | `green-untracked.log`：1 passed |
| tracking JSON 与三个运动方向 | `red-tracking.log` | `green-tracking.log`：4 passed |
| 每行时间、距离、barrier、完整导数 | `red-query-time.log` | `green-query-time.log`：5 passed |
| 原内容与合并包围的公开检查 | `red-identity-envelope.log` | `green-identity-envelope.log`：9 passed |
| 运动估计与物理范围相容 | `red-tracking-physical-range.log` | `dynamic-final.log`：53 passed |
| 同一 track 的多个成员保留声明速度 | `red-shared-velocity.log` | `dynamic-consumers-final.log`：312 passed，包含全部 54 项动态检查 |

## 本轮 Spec 与 Standards 修订

track 更新时间误差计入原观测至实际更新的完整历史年龄。未跟踪来源的历史半径、增长率、加速度和期限由真实 policy 核验；模块固定数组来自自身配置。直接 tube 显式接收实际 policy，缺少时有效 mask 为 False。固定来源绑定实际模型内容身份、版本、policy 和成员几何，全部模型成员均须由有效 source 与 group 完整表示。snapshot 在修改内部状态前完成外部 tracking 校验。

本轮完整命令保存在工作区 `.scratch/map147-101-review3/commands.md` 和 `commands.json`，日志与 JUnit 使用相同目录：

| 检查 | 结果 | 文件 |
|---|---|---|
| 未修改的独立 Spec 与 Standards 探针 | 修订前目标 1 failed；修订后完整 20 passed | `red-independent.log/xml`、`green-independent.log/xml` |
| 六项完整成员与合法合并测试 | 修订前 4 failed、2 passed；修订后 6 passed | `red-behavior.log/xml`、`green-behavior.log/xml` |
| 动态、准备、碰撞与距离消费者 | 235 passed、1 deselected，97.63 s | `consumers.log/xml` |
| #108 占据消费者 | 43 passed，3.52 s | `occupancy.log/xml` |
| Ruff、mypy、compileall、diff | 退出码 0 | `static-checks.log` |
| 隔离 colcon 构建与实际安装 | 退出码 0，1 package finished | `build-install.log` |
| 安装目录真实公开接口 26 项与八个资源身份 | 退出码 0 | `installed-check.log`、`installed-evidence.json` |

当前实现回归共 278 项通过，包含 80 项动态、105 项准备、22 项碰撞、28 项距离和 43 项 occupancy。前次 9 ms / 8.349961508 s 的原始失败与单项复测保留在前次目录；独立隔离复测通过但原因未确定，本轮没有重复运行或修改阈值，也不作性能修复结论。

新版安装公开接口脚本为工作区 `.scratch/map147-101-review3/check_installed.py` 与 `run-installed-check.sh`，实际运行通过。四个模块导入路径位于隔离安装目录；八个源码及配置 hash 与工作区相同。解析 barrier 最大误差 `3.552713678800501e-15`，时间导数最大误差 `2.220446049250313e-16`，毫米距离误差为 `2.842170943040401e-14`。这些数值仅用于当前解析软件接口验证。

## 交付与限制

初始证据、输入 JSON、tracking 回放与依赖来源位于主仓库 `.scratch/map147-101`。本轮日志与前后文件身份在工作区 `.scratch/map147-101-review3`，主仓库保存新的冻结入口；前次文件保持不变。模块文档为 `docs/modules/dynamic_support.md`，公共碰撞文档及导读已同步。

`OccupancySnapshot` 与 `CollisionPolicy` 的字段保持既有结构。集成时以主仓库最新内容为依据，保留 #105 的 LidarRayBatch、update/invalidate 与 snapshot 健康增量。只使用 host monotonic 的当前 layout；共同仿真物理时间接口由 #154 处理。完整机器人连续运动证明、设备参数及真实回放继续由 #113/#117/#122 提供。

没有进行 spec promotion、commit、push、PR 或归档。源码与证据冻结后停止编辑，等待独立复核与主会话集成验收，不领取后续任务。

## 与 main #105 的组合

由独立 merger 以 main 最新 lidar_occupancy 为依据组合，当前工作区没有覆盖 main 的修改。

- 保留 #105 的 LidarRayBatch 可见 free/unknown 区间、input_valid、filter_identity、invalid_reason 字段，以及 update/invalidate、_revoke_free 和 _identity_invalidated 的来源与行为。
- 保留 main snapshot 中 `_input_valid=False` 时返回 INVALID_SCENE、`covered=False` 的处理。加入 #101 tracking 校验时，其类型、身份、关联和内容验证须在 `_expire`、`_last_now_ns`、`_revision` 等状态修改以前完成。
- 使用共同 fixed_environment.FixedEnvironmentModel 并保留旧导入；加入原观测、tracking 与 SupportTimeModel 的来源数组、实际模型字段及全部成员覆盖检查。OccupancySnapshot 与 CollisionPolicy 字段结构保持原样。
- 组合后重新运行 #105 self-filter 与占据接口，以及本票动态/准备/直接 tube 和独立来源探针。当前工作区的通过结果不表示 main 已完成组合验收。
