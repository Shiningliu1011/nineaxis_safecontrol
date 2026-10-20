# 控制器预热与正式记录

## 真实输入与编译原因

本轮使用 `output/static-method-evidence-20261020/` 中已经关闭的真实 B 组记录，
在 CPU 16–25、JAX 0.6.2、x64 环境重放原始 QP。没有启动完整 sim。
所有输入、编译日志和输出保存在该目录的 `commands/`。

| 原始运行 | 初态 / 终点变量及限制数 | 首次终点求解 | 相同输入重复求解 |
|---|---|---:|---:|
| `b-s0-current-01` | 10/94 → 9/92 | 252.417 ms | 2.765 ms |
| `b-s3-travel-100-current-01` | 10/104 → 9/102 | 266.433 ms | 2.738 ms |

两份记录的首次终点调用均触发 `_solve_or_reuse` 九维形状的 JAX 编译。
B/S0 的 XLA 编译为 195.997 ms。B/S3 的首次和重复求解均为 100 次迭代，
数值结果与原始记录相同。重复调用没有新增编译。
联合十维问题内部使用的九维求解分支，没有准备正式固定进给入口的独立编译形状。

B/S0 使用 candidate sequence 6834、start_step 89250；其位置、速度、
加速度、参考进度及实际解保存为 `tests/data/static_b_s0_terminal.json`。
原始 controller SHA-256：

- B/S0：`3ec3345a0197c09d1ce8fa6ce230e7f6288963bce0c0a449835c0e983ff4f3fa`
- B/S3-100：`02c0fd3c44f90ca8af82243d292369401c6c4dc3c2fddb5c4e2700f703d6d1db`

重放前后原始数据库 SHA-256 相同。完整证据文件为
`commands/b-s0-terminal-compilation-before.{json,log}` 和
`commands/b-s3-travel-100-terminal-compilation-before.{json,log}`。

## 实现与回归入口

`StaticControl.warmup(initial_state)` 使用实际初态与相同设置，完成前进候选和
零进给求解。预热控制对象独立保存恢复状态、候选序号、QP 解与运动证书。
正式对象从初始恢复状态、空记录和序号开始。

任务、限制与构型输入由 `_problem_inputs` 共用，求解设置由 `_solve_velocity`
共用；原有 `propose` 的试算顺序、目标、限制、参数和最终检查保持相同。
ROS controller 在建立计算线程与订阅之前调用预热入口。

回归通过实际 node 初始化与公开 `propose` 接口验证：

- A–E 首条正式 candidate 的序号为 1，复用来源没有引用预热数据。
- 实际 B/S0 终点输入在初始化后不再触发 JAX 编译。
- 终点解与原始记录在 `1e-9` 绝对误差内一致，进给相同。

TDD 记录为 `commands/controller-warmup-history-red.log`、
`commands/controller-warmup-history-green.log`、
`commands/controller-warmup-terminal-red03.log`、
`commands/controller-warmup-green.log`。两项定向回归通过，耗时 28.00 s。

扩展回归为 39 passed，349.59 s，CPU 16–25：

```bash
python -m pytest tests/test_static_controller_warmup.py tests/test_static_control.py \
  tests/test_static_velocity_solver.py tests/test_static_experiment_nodes.py \
  tests/test_static_recovery.py -q --basetemp=.scratch/method-warmup-regression
```

包括 A–E 初始化、实际终点解、QP/SLSQP、相邻周期复用、Ruckig、
暂停恢复、节点失败记录和清理。日志
`commands/controller-warmup-regression.log` 的 SHA-256 为
`5c6e2a4149a625c4f3b30cbbfef685ef616dac5a7409a560bbbde0675596e945`。

## 修复后的实际重放

以各自原始配置初始化真实 controller，再调用相同 QP 输入。
B/S0 与 B/S3-100 的首次终点求解分别为 2.266 ms、2.501 ms；
十维和九维入口均没有新增编译，全部原始不等式与数值解检查通过。
两个重放均保持 100 次迭代，原始数据库 SHA-256 前后相同。
重放脚本为 `commands/diagnose_terminal_compilation.py`，
`--warmup-root` 指定本 worktree；JSON 保存执行来源提交与三个模块的内容 SHA-256。

| 文件 | SHA-256 |
|---|---|
| `commands/b-s0-terminal-compilation-before.json` | `25a162f122b2e83e7fa61fc4e3d6b00909b741f77f3baa97df5e45f1a7b35b6b` |
| `commands/b-s0-terminal-compilation-after.json` | `94b09455a89886341b795a69c36b55ee2c619ae0ff8a3b377821124a1b2ade65` |
| `commands/b-s3-travel-100-terminal-compilation-before.json` | `34781902b41637a0d2a021bc7f8ecee76d16b9433f53fedabe697a8c85f5136d` |
| `commands/b-s3-travel-100-terminal-compilation-after.json` | `d916ea9532f2cbe5b48b025bd23521e8ec6e6f073ec5d2307e1b0a515b0672ad` |
| `tests/data/static_b_s0_terminal.json` | `3afaa613933aa980c09a20ad8fbbded83ee47e9cef7da3259aab3097074b6474` |

完整 sim、共同周期及最终全量回归仍由集成阶段安排。
本轮离线结果只用于预热与记录来源验证。
