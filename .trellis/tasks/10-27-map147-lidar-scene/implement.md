# #108 执行与验证

## 检查安排

- [x] 核对 worktree、分支、HEAD 与 main merge-base 均为 `9c354d55`。
- [x] 读取 issue、ADR 0006/0014–0018/0028/0045、既有 scene/参数接口及项目 backend 规范。
- [x] 主会话确认用户已定案 #149，#108 原生开放阻塞数为零，允许本地实施。
- [x] 审阅需求、设计与公开测试边界；范围符合 #108。
- [x] 激活 Trellis 任务。
- [x] TDD：hit 与未知覆盖；单个行为运行 red，再完成实现并运行 green。
- [x] TDD：连续独立 free、重复/中断/无返回/遮挡和过期。
- [x] TDD：误差与年龄范围、固定模型并集和版本。
- [x] TDD：完整覆盖合并、容量失败及真实 prepare_scene 复核。
- [x] 运行受影响 scene/参数接口测试、lint 和类型检查。
- [x] 按 trellis-check 完成本票全部范围检查，记录验证与剩余限制。
- [x] 检查 main 最新 tip，按相容性普通 fast-forward merge；交付未提交文件给主会话。
- [x] 完成两路独立审查要求的公开接口回归与修正。
- [ ] 主会话复核最终证据并组织文件集成，按 tracker 保存最终验收。

## 受测公开边界

`LidarRayBatch` → `LidarOccupancyScene.update/snapshot` → 真实 `CollisionSafety.prepare_scene`。固定模型通过真实 trimesh 对象进入公开转换接口，预期覆盖由独立解析几何检查。参数使用真实 `CollisionParameterArtifact` 公开签名/加载接口。测试不使用 mock。

## 隔离环境

加载主仓库 `.scratch/host-recovery/env.sh` 后，明确设置 `PYTHONPATH` 为本 worktree 的 `portable_oscbf`、其 tests 及 `src`；`TMPDIR`、`ROS_LOG_DIR`、pytest `--basetemp` 和缓存均写入主仓库 `.scratch/map147-108/`。使用现有 Python 3.10/JAX CPU 环境，不创建 ROS node，不运行 launch，不连接设备。

各次命令使用独立日志与 basetemp，使用同一受测源码路径。最终记录实际命令、退出码、red/green 结果与源文件身份。

## 执行记录

任务在 `TRELLIS_CONTEXT_ID=map147-108` 下完成 context validate 与 start，状态为 `in_progress`。

全部测试通过 `.scratch/map147-108/run_checks.sh <名称> <pytest目标>` 运行。脚本加载既有环境，显式 `JAX_ENABLE_X64=1`，优先从本 worktree 导入受测源码，按名称保存日志与 pytest basetemp；退出码由 `pipefail` 保留。

| 公开行为 | red 日志及退出码 | green 日志及实际结果 |
|---|---|---|
| hit、射线路径和必要 unknown | `hit-red.log`，2，缺少公开模块 | `hit-green-x64.log`，0，1 passed |
| 三次独立连续 free | `free-red.log`，1，第三次未清除 | `free-green.log`，0，2 passed |
| 无返回/遮挡/ambiguous 与陈旧 occupied | `expiry-red.log`，1，3 failed | `expiry-green.log`，0，5 passed |
| 测量误差、原始年龄和版本 | `age-radius-red.log`，1，包围范围断言失败 | `age-radius-green.log`，0，6 passed |
| 真实固定 mesh、free 冲突与模型版本 | `fixed-red.log`，4，缺少公开固定模型类型 | `fixed-green.log`，0，7 passed |
| 保守合并及容量失败 | `capacity-red.log`，1，2 failed | `capacity-green.log`，0，9 passed |
| 整数边界与同格逐点误差保留 | `input-bounds-red.log`，1，4 failed/1 passed | `input-bounds-green.log`，0，14 passed |

后续行为验证覆盖重复 scan、同 scan 多条 free 射线、证据中断与超时、再次 hit、较近 hit 的遮挡保留、free 过期、完全已观测空场景、合并范围超限、固定模型覆盖范围、无法表示的大模型、artifact 阈值及单点范围超限。新场景测试共 43 项，每种输出由真实 `prepare_scene` 校验。

## 独立审查覆盖

固定模型的完整内容、来源、几何、policy、验证说明、需求数量与 revision 共同生成 `model_hash`；构造和接收均重新校验，生产者持有自己的副本。名义 hit 在网格外、测量不确定范围进入网格时保留完整包围范围。ambiguous 对应的不确定范围逐格撤销覆盖，同批次 free 不能清除相应 occupied。free 证据保存逐条时间，只使用当前有效窗口内的独立连续证据完成清除。

| 公开行为 | red 日志及退出码 | green 日志及实际结果 |
|---|---|---|
| 固定模型内容、来源、policy 与身份 | `review-identity-red.log`，1，4 failed | `review-identity-green.log`，0，5 passed |
| 两侧边界 hit 不确定范围覆盖 | `review-boundary-red.log`，1，2 failed | `review-boundary-green.log`，0，3 passed |
| ambiguous 撤销 free/occupied 的覆盖与有效恢复 | `review-ambiguous-red.log`，1，2 failed | `review-ambiguous-green.log`，0，5 passed |
| free 确认窗口内的逐条时间有效性 | `review-free-age-red.log`，1，1 failed | `review-free-age-green.log`，0，5 passed |

全部新场景行为复核为 `43 passed in 3.20s`，日志为 `.scratch/map147-108/review-behavior-regression.log`。范围还包括构造后数组变化、自有副本、三个相交 voxel 的 ambiguous 范围、同批 free/ambiguous、几何 hash 和验证说明的内容绑定。

已读取并执行独立审查员的真实 probe，脚本为 `.scratch/map147-108/review-spec/run_probe.sh`，原始 probe 和日志保留。复核结果保存为 `probe-after-fixes.log`：过期 free 情形保持 occupied、当前计数 2；ambiguous 输出 unknown、coverage false，`prepare_scene` 状态为 3；边界 hit 输出 occupied 且存在活动 support，`prepare_scene` 状态为 0。

最终回归命令：

```bash
bash /home/lsn/robot/robot_safecontrol/.scratch/map147-108/run_checks.sh review-final-regression \
  portable_oscbf/tests/test_lidar_occupancy.py \
  portable_oscbf/tests/test_prepare_scene.py \
  portable_oscbf/tests/test_collision_parameters.py \
  portable_oscbf/tests/test_collision_safety.py
```

实际退出码 0，`272 passed in 35.57s`，直接日志为 `.scratch/map147-108/review-final-regression.log`。

`run_quality.sh lint` 实际退出码 0，Ruff `All checks passed!`；`run_quality.sh type` 实际退出码 0，mypy `Success: no issues found in 1 source file`。检查工具复用 `.scratch/pipe-geometry/check-tools`，缓存写入本任务目录。`git diff --check` 通过。安装声明 `setup.py:82` 的 `portable_oscbf/work/*.py` 已涵盖新增模块，配置 JSON 由同一现有安装入口包含；本轮没有重新安装工作区。

2026-10-27 读取集成分支 `main` 为 `9c354d5505e26aa28b3e6fc323c661bd7542b105`，执行 `git merge --ff-only main` 返回 `Already up to date.`，没有恢复或覆盖文件。

## 审查与交接

按 `trellis-check` 复核本票全部源码、公开行为、参数来源、错误传播、文档和安装声明。高风险范围为占据证据清除、未知空间准入、保守范围与容量失败；两路独立审查覆盖的行为均已有直接公开接口回归证据，最终由主会话复核集成。`CollisionScene/CollisionIdentities/SceneTracks` 的类型注解与既有 NumPy host 输入能力一致，数值准入检查保持原条件，显式非 OK 状态在覆盖拒绝前保留其错误身份。

本票交付纯场景对象和真实公开接口测试。没有运行 LiDAR 回放、ROS 节点、真实设备、生产切换或目标设备周期验收。外参、mesh 自体过滤、tracking/reachable support、server 和完整验收仍按 #95/#105/#101/#104/#122/#119/#123 执行。

规范审阅已完成：代码与模块文档可供后续接口消费；没有修改 `.trellis/spec/`、`CONTEXT.md` 或 ADR，没有执行 spec promotion。当前没有 commit/push/PR 授权，全部修改保持未提交，worktree 继续保留供独立审查与 merger 使用。

## main 工作区集成与验证

2026-10-27 在 `/home/lsn/robot/robot_safecontrol` 的 `main` 工作区使用 `apply_patch` 集成交接清单的十五个文件。`HEAD` 为 `9c354d5505e26aa28b3e6fc323c661bd7542b105`。集成前主仓库的五个已修改范围文档和五个 `decision-*.md` 已保存 hash；集成后逐项检查全部相同。`docs/modules/portable_oscbf.md` 仅增加 LiDAR 模块目录项，保留原有内容。主会话确认 Standards 与 Spec 两路独立复核通过。

主仓库专用脚本为 `.scratch/map147-108/integration/run_checks.sh`，实际测试入口为同目录 `run_regression.py`。脚本加载既有 `.scratch/host-recovery/env.sh`，显式设置主仓库 `cwd` 和 `PYTHONPATH`，启用 JAX x64，并把临时文件、pytest basetemp、缓存及日志全部写入 `.scratch/map147-108/integration/`。回归在同一 Python 进程中核对受测模块路径后调用真实 pytest，结束后再次核对测试模块路径；所有相关 `__file__` 均位于主仓库的 `portable_oscbf/`，没有导入源 worktree。

实际命令及结果：

| 命令 | 退出码 | 结果与日志 |
|---|---|---|
| `/bin/bash .scratch/map147-108/integration/run_checks.sh audit copied` | 0 | 十五个集成文件与已审源逐字相同，`copied-files.json` |
| `/bin/bash .scratch/map147-108/integration/run_checks.sh regression` | 0 | 四个相关测试文件 `272 passed in 35.32s`，包含 43 项 LiDAR 行为，`regression.log` 和 `regression.xml` |
| `/bin/bash .scratch/map147-108/integration/run_checks.sh lint` | 0 | Ruff `All checks passed!`，`lint.log` |
| `/bin/bash .scratch/map147-108/integration/run_checks.sh type` | 0 | mypy `Success: no issues found in 1 source file`，`type.log` |
| `/bin/bash .scratch/map147-108/integration/run_checks.sh context` | 0 | 8 项 implement context 和 5 项 check context 通过，`context.log` |
| `/bin/bash .scratch/map147-108/integration/run_checks.sh links` | 0 | 三篇模块文档的 17 项本地链接目标存在，`links.log` |
| `git diff --check` | 0 | 已修改文件空白检查通过，`diff-check.log` |
| `sha256sum -c .scratch/map147-108/integration/protected-before.sha256` | 0 | 主仓库原有十个范围文档与决定文件保持原内容，`protected-check.log` |
| `/bin/bash .scratch/map147-108/integration/run_checks.sh identity` | 0 | 九个源码、测试配置与模块文档符合已审 hash 清单，`identity.log` |
| `/bin/bash .scratch/map147-108/integration/run_checks.sh audit final` | 0 | 十五个文件空白检查、源身份、原有内容及 worktree 清单检查通过，`final-files.json` |

除本节追加内容外，十五项交付保持已审源内容。九个源码、测试配置与模块文档同时符合 `.scratch/map147-108/review-source-sha256.txt`；完整源文件身份、集成文件身份和逐文件空白检查保存于同目录 `final-files.json`。源 worktree 的十五个文件 hash 保持不变，其他 worktree 继续保留。没有执行 commit、push、PR、tracker 写入、spec promotion 或归档。本次仍为纯软件接口验收，没有执行 ROS node、真实 LiDAR 回放、设备操作或生产切换。
