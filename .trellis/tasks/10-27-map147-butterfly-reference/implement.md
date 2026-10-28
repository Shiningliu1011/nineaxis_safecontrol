# 实施与验证

实现来源为 `.scratch/worktrees/map147-path-reference`，冻结清单为 `.scratch/map147-152/frozen-handoff.json`，共同基点为 `9c354d5505e26aa28b3e6fc323c661bd7542b105`。主目录保留原蝴蝶输入，以 main 当前源码重新生成 production 与 S0 两份连续参考包。

- [x] 连续 NURBS、最终投影、真实弧长、NumPy/JAX 共用查询与有序投影完成。
- [x] 源节点左右侧三阶导数、公开姿态切换、连续记录恢复及样本筛选取得独立复核。
- [x] Spec 与 Standards 审查修订全部核验。
- [x] main 源码集成、两份包重新生成、全部原样本重算与直接消费者通过。
- [x] main 静态检查、组合隔离安装与安装消费者通过。
- [x] main 完整项目测试、CPU 条件性能验收与最终文件清单。

## 来源工作区的行为证据

全部 `logs/` 路径以主仓库 `.scratch/map147-152` 为根。源工作区同名记录保留完整命令，以下列出实际 red/green 证据。

| 行为 | red 证据 | green 证据 |
|---|---|---|
| 原蝴蝶、完整投影和最终弧长 | `logs/red1.log`，退出 2 | `logs/green1b.log`，1 passed |
| 版本文件生成、加载和身份 | `logs/red2.log`，退出 2 | `logs/green2b.log`，1 passed |
| NumPy/JAX 共用连续查询 | `logs/red3.log`，退出 1 | `logs/green3.log`，2 passed |
| 有序投影和节点侧别 | `logs/red4.log`，退出 1 | `logs/green45.log`，7 passed |
| 实际轨迹入口 | `logs/red5.log`，退出 1 | `logs/green45.log`，7 passed |
| 区间内部驻点拒绝 | `logs/red-stationary.log`，3 项拒绝断言失败 | `logs/green-stationary.log`，15 passed |
| 连续端点区间和终端预热 | `logs/recovery-main-test_static_control.log`、`logs/recovery-main-test_static_controller_warmup.log`，退出 1 | `logs/terminal-retry.log`，6 passed |
| 全部源节点左右侧三阶 JAX 自动微分 | `logs/review-side-red.log`，1 failed、1 passed | `logs/review-side-green2.log`，2 passed |
| fixed 姿态切换 | `logs/review-orientation-red.log`，1 failed | `logs/review-orientation-green2.log`，1 passed |
| NPZ 结构、身份与全部数组 | `logs/review-record-red.log`，13 failed、1 passed | `logs/review-record-green2.log`，15 passed |
| 公共关节身份和筛选边界 | `logs/standards-boundary-before.log`，1 failed、2 passed | `logs/standards-consumers.log`，88 passed |

测试使用真实 MAT/NPZ/JSON、SciPy BSpline/积分、NumPy、JAX `jacfwd` 和实际公开加载入口。两项筛选测试在整理前后均通过，验证全部保留数组、原重复终点、邻近源样本和事件边界。

## main 资产与消费者

正式生成器、参数、输入及环境记录在 `.scratch/map147-152/integration/generate.sh`、`generation-inputs.json` 和两个 manifest。原 MAT SHA256 为 `79868b03afd68f9d4cca1cba568320f0f48efd355d020a9372f2037549c3293f`，原始控制点、节点与规划 MAT 全部保留。

| 版本 | 参考身份 | 总长度 m | main manifest SHA256 |
|---|---|---|---|
| production | `62049d74d7f44e85b7d70715105a7821dabca29bc0687c0b06c852739e4d9972` | 1.8167657348637822 | `a46d39fe652327fbd9e5fd259d0fd941f88d47bfb349747a0190392d493c9261` |
| S0 | `58ff1e18b3c7f874f9ec2a3557785d6316bd4a2136bb40b25014d089b91f877e` | 1.8167619852376178 | `671e65317f6a47cbd9b8237e1c5a703fec5a91f6dea052f73263c3af6b73d9d9` |

两个 manifest 的生成源码及配置路径指向 main。每份三个 NPZ 的全部数组和文件 SHA256 与冻结产物相同；独立 SciPy 重算全部 14992 个原样本，原曲线差异最大 `8.526512829121202e-14 mm`，最终位置差异最大 `2.498001805406602e-16 m`，轴向差异最大 `1.1102230246251565e-15`，全部查询有效，见 `integration/artifacts.json`。

main 直接消费者覆盖连续查询、真实生成、JAX、原轨迹/姿态、移动参考、统一变换、静态执行评价、标定加载及 #153 资源加载共 90 项。首次为 89 passed、1 failed，评价临时路径指向未挂载的既有研究目录；本地可恢复目录准备后同一测试为 1 passed，原链接已经恢复，见 [评价临时目录检查](research/diagnosis.md)。首次失败日志继续保留。

Ruff 新文件与既有消费者致命规则、mypy 五个主要实现通过。组合 colcon 安装成功，安装文件字节与 main 一致，Python `-I` 下实际加载安装副本中的生产/S0、连续消费者、模型、限制和 FK，见 `integration/installed-files.json`、`installed-imports.json`。

## 完整验收与能力范围

main 完整项目测试和 CPU 条件性能结果记录在 [main 集成验收](research/integration.md)。完整 nodeid 无缺项：主包 847 项功能通过、6 项跳过；内核 706 项功能通过、34 项跳过；两项计时单独保留全部条件。controller 在四个物理 P-core 下为 303 步、p95 14.343 ms、1 步超期（0.33%），原门槛通过；modular 在八个 P-core 下为 3000 步、单步 6.379 ms 相对原基准 6.452 ms，原 120% 门槛通过。全部 CPU 与 controller 八个 P-core 的失败完整保留。

完整项目周期和完整计算组合由 #119、#144/#169 继续验收；本模块的连续参考与 CPU 条件结果不构成普遍 50 Hz 能力证明。真实标定、工艺和设备能力按对应任务取得。规范与 ADR 保持原内容，全部修改未提交。
