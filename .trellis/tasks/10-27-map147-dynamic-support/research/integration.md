# #101 main 集成验收

本记录对应 main 工作区 `/home/lsn/robot/robot_safecontrol`，Git HEAD 为
`9c354d5505e26aa28b3e6fc323c661bd7542b105`。来源为已通过 Spec 与 Standards 的 fix3：
`.scratch/map147-101/review-fix3-handoff.json`，SHA-256 为
`4fee0f00584babac2628b516c16b96fa75fecdcede05dde8afeac82e81aa0206`。
22 份来源、25 份证据与 15 份共享输入均已逐项核验。全部代码、测试、文档及 Trellis
使用 `apply_patch` 集成，工程保持未提交。

## 组合内容

五份独立生产文件与两份测试保持来源字节。`lidar_occupancy.py` 以 main 的 #105 内容组合，
保留 batch 的可见 free/unknown 区间、input/filter 身份、update/invalidate、free 撤销、
身份失效状态，以及输入失效时 `INVALID_SCENE` 和 `covered=False`。新增原观测、tracking
及时间来源数组；外部 tracking 校验位于过期处理、内部时间和 revision 修改以前。

共同 `FixedEnvironmentModel` 位于 `fixed_environment.py`，既有导入继续可用。
OccupancySnapshot 与 CollisionPolicy 字段保持原样。组合 occupancy 的 SHA-256 为
`ed8b70eb1b28e2154e71382545b821d75ef009802cfcd0fb768f5aa0d193c4f0`。
`combined-source-check.json` 保存逐方法 AST 对应关系，`combined-occupancy.diff` 保存
相对已审来源的 #105 增量。公共文档保留 #105 说明和 #108/#152/#153 目录内容。

主会话另以只读 stdlib 检查全部既有内容，证据为
`.scratch/map147-101/parent-preservation-audit.json`：#152 的44份、#105 的18份及 #153 的
13份保留文件字节一致；四个 LiDAR 公共 class 和七个状态方法保持 #105 AST，OccupancyGrid
与已审 #101 来源相同。旧集成清单与历史日志全部保留。

## main 实际检查

命令与环境入口位于 `.scratch/map147-101/integration`。测试使用项目 Python 3.10.12 venv、
JAX/jaxlib 0.6.2、x64 与 CPU，全部临时、pytest、Matplotlib、ROS 及编译缓存输出位于
被忽略的 `.scratch`。没有运行真实控制 I/O。

```bash
python3 .scratch/map147-101/integration/check_sources.py
bash .scratch/map147-101/integration/run_consumers.sh
bash .scratch/map147-101/integration/run_independent.sh
python3 .scratch/map147-101/integration/run_check.py static-checks bash .scratch/map147-101/integration/static_checks.sh
bash .scratch/map147-101/integration/build_install.sh
bash .scratch/map147-101/integration/run_installed.sh
python3 .scratch/map147-101/integration/audit_results.py
python3 .scratch/map147-101/integration/check_links.py
```

`run_check.py` 记录每条命令、环境、源码 hash、开始和结束时间、退出码及原始完整日志。
其命名结果文件采用独立证据身份；需要复测时使用新的名称和输出目录。JUnit 保存全部实际项，
`coverage.json` 保存逐项结果和模块数量。

| 集合 | 实际通过数量 |
|---|---:|
| dynamic support | 80 |
| prepare scene | 105 |
| CollisionSafety | 22 |
| ellipsoid point | 29 |
| ellipsoid DCOL | 33 |
| LiDAR occupancy | 43 |
| self-filter | 26 |
| ROS adapter | 10 |
| collision parameters | 105 |
| cbfpy dependency | 3 |
| 独立 Spec/Standards 探针及本次组合检查 | 30 |

相关消费者 **456 passed，236.73 s**，独立集合 **30 passed，12.43 s**；两次退出码均为0，
没有 skip、deselect 或失败。新增四项组合检查通过真实 mesh ray self-filter、tracking、
occupancy、prepare 和直接 tube 验证过滤输入、历史失效、过滤身份变化及拒绝请求后的状态。
Ruff、mypy 六份源码及未标注函数体、compileall、diff 检查均通过。

两个数值集合顺序使用 CPU affinity `0,2,4,6`。实际消费者进程记录为22线程，允许 CPU
集合与命令一致。原 `radii5-point5-witness5` 静态个案完整纳入消费者集合，并通过原9 ms
断言。原8.349961508 s失败、其他CPU条件与原baseline字节保留，原因仍未确定，本次没有
修改门槛，也不作性能修复或普遍50 Hz结论。

消费者出现一条已有 Matplotlib Axes3D 导入警告，本次没有生成图片。构建出现 pythran 的
NumPy FutureWarning 与可选 colorlog 提示；colcon 实际退出码为0。

## 实际安装

本次重新构建 main 到 `.scratch/map147-101/integration/install`，colcon 结果为1 package
finished，package 7.81 s、合计8.33 s。setup.py 使用既有 work 文件 glob 自动安装新增模块，
并保留 #105/#152/#153 的依赖和资源。

`python -I` 实际导入该安装目录的五份 kernel 模块与 adapter。安装公开接口 **43项通过**，
其中36项全程没有导入 ROS，另外7项使用真实 ROS 消息适配接口。36份安装源码与资源 hash
均与当前 main 相同，包含本次时间模型、occupancy、参数、self-filter、adapter、标定与回放
数据，以及既有共同模型限制和两份连续参考包。完整身份与导入路径见
`installed-evidence.json`，没有使用 worktree 包替代 main 安装证据。

独立解析公开检查的 barrier 最大误差为 `3.552713678800501e-15`，时间导数最大误差为
`2.220446049250313e-16`，中心差分误差为 `3.7476688419246784e-11`，距离误差为
`2.842170943040401e-14 mm`。

## 验证范围

生产调用点检查显示本次公共碰撞模块由 occupancy、self-filter 和 adapter 消费；其余控制器
源码保持既有字节。前次完整主包、kernel、AEB 与 CPU 条件证据保存在 #152 的
`integration/full/coverage.json` 和相应日志；本次根据新增公共边界运行受影响集合，没有
重复完整项目或 CPU 矩阵。

当前仅支持正数 host monotonic ns；共同物理时间由 #154 处理。`certify` 保持
`CERTIFICATE_FAILED`，完整机器人连续运动与停止证明、设备参数和真实 LiDAR 回放继续由
#113/#117/#122 验收。当前结果不提供真实硬件或正式完整周期准入结论。

最终集成清单为 `.scratch/map147-101/integration/final-files.json`，包含来源与 main 文件
SHA-256、组合关系、安装和测试证据及既有文件保留检查。本票交主会话登记，本轮结束后停止，
没有开始后续票，也没有 commit、push、PR、issue 写入或 spec promotion。
