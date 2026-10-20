# 静态实验接手记录

## 当前状态

工作分支为 `codex/static-obstacle-experiments`，开发基准为 `34a2e34`。
#171 尚未完成验收，#172–#183 保持开放。用户要求空间不足时停止本机任务并同步远程，
随后补充主机也可能无法提供 200 GB 空间。当前已停止新增实验采集。

接手入口是 `.trellis/tasks/10-20-static-formal-experiments/`；
总任务及子任务对应关系见
`.trellis/tasks/10-20-static-obstacle-experiments/prd.md`。
实现、参数和已有验证记录随本分支保存。

## 实验需要回答的问题

- S0–S5 检查九轴运动的路径覆盖、位置和工具轴精度、外部间距、
  自碰撞间距、关节限制、暂停恢复与持续阻塞后的终止。
- A/B/C/D 对比联合进给求解与改进恢复各自的效果；E 提供 Generalized-SNS 参考。
- 完整周期测量检查求解、运动检查和准入能否满足共同控制周期。

正式矩阵为 8 个场景配置 × 3 个初态 × 3 档进给 × 5 个方法 × 3 次重复，
共 1,080 次。正式数据尚未取得，现有开发结果不构成总体效果或真机能力结论。

## 存储核查

当前 S0 完整运行的 15 个主要文件共 186,952,238 字节。ZIP/LZMA 无损压缩并逐文件
核对 SHA-256 后为 67,511,754 字节。按这个样本乘以 1,080，分别约为
202 GB 和 73 GB；实际体积随运行时长、暂停次数和场景对象数量变化。
S2 的当前完整运行目录约为 219 MB。

主要体积来自两个 SQLite 数据库中的 JSON 数值记录，以及逐反馈的派生评价数组。
`RunRecorder` 位于 `src/robot_safecontrol_moveit/static_experiment_recording.py`；
`static_clearance_evaluation.py` 和 `static_experiment_evaluation.py`
生成派生 NPZ。NPZ 已使用压缩保存，普通目录压缩仍有上述体积。

本机清理了 Git 识别出的 42 个遗留临时打包文件、经检查无活动进程使用且具有完整
PyInstaller 依赖标记的解压目录、pip 下载缓存及压缩测量副本。
释放约 30.98 GB，可用空间由 25.89 GB 增至 56.87 GB。
清理前后 Git 连通性检查均通过，实验原始记录仍保留。

**接手后的前置工作是核定存储方案与实测容量。**
数值记录可以研究无损的紧凑表示，重复状态可以通过身份关联，
可重算数组可以研究按需生成。尚未实施这些改动，也没有新的体积承诺。
修改时须同时处理报告身份校验、重放、异常退出恢复和完整周期测量，
并用真实记录逐字段核查。原始反馈、控制决策、失败原因和验收精度必须保持完整。

## 已有运行与未完成项

| 运行目录（位于 `output/static-experiments/`） | 当前证据 |
|---|---|
| `s0-joint-full-geometry-01` | D/S0 完整跟随通过；路径 RMS 0.058782 mm |
| `s0-sns-full-geometry-01` | E/S0 完整跟随通过；路径 RMS 0.059183 mm |
| `s2-joint-development-01` | 当前参数下 D/S2 综合通过；87,294 条反馈；路径 RMS/P95/最大值为 0.060204/0.123024/0.244570 mm；完整周期 P99.9 为 53.162420 ms |
| `s4-joint-development-10` | 当前保存结果为失败；没有所要求的实际暂停恢复事件 |
| `s5-joint-development-01` | 持续暂停后终止及几何检查通过，完整周期检查失败；当前参数尚未重跑 |
| `s4-witness-01` | 已有独立连续构型调整见证；需要结合当前场景和实际运行重新验证 |

`development-scenes-05` 已生成 8 个配置及三组初态。S1 使用候选 0，
S2 使用候选 15、31，S4 使用候选 0、2。S4 当前配置尚未执行完整验证。
候选编号对应该目录的 `obstacle_candidates.json`，其来源身份保存在选择记录中。

共同构型辅助速度上限为 0.02，开发控制间距余量为 0.1 mm；
其余共同参数与测量范围见
`.trellis/tasks/10-20-static-method-matrix/research/parameter-selection.md`。

后续尚需完成：

1. 存储改动的容量、内容一致性、异常恢复和重放验证。
2. 当前参数下 S4 的独立见证及实际暂停、构型调整、恢复全过程，S5 终止和耗时复核。
3. S0–S4 对应初态与行程范围的可行性证据。
4. 五种方法、四种候选周期的 20 次完整测量。开发配置的 100 ms 尚未被选为正式周期。
5. 固定正式清单，执行 1,080 次实验，完成配对统计与六类图表。
6. 当前版本的完整回归及项目规范、需求两方面的独立审查。

## 验证状态

- 本次接手检查 `bash scripts/agent_check.sh` 通过：Python 编译、纯模块导入、
  shell 语法、Git diff 检查及 69 项纯逻辑测试通过。
- 最近相关检查包括连续运动 6 项、终点与求解器 13 项、真实 ROS 节点与执行评价
  4 项通过；对应开发记录保存在各子任务中。
- 本次标准 package 构建通过，耗时 1.70 s；安装后的 CLI 入口及接手文档安装路径核查通过。
- 当前完整回归尚未完成。控制内核测试入口已采用按文件串行启动 pytest，
  用于释放各文件的 JAX 编译缓存。接手时执行 `bash run_all_tests.sh`。
- 项目规范与需求的独立审查尚未完成；本次同步用于继续开发。

## 接手所需文件与环境

代码、配置、测试、图表及 Trellis 记录由 Git 分支同步。
当前全部实验记录在原机器
`/home/lsn/robot/robot_safecontrol/output/static-experiments/`，约 3.5 GiB，
由 `.gitignore` 排除，Git 拉取不会传输这些记录。

本机另有完整数据包
`.scratch/static-experiments/handoff-data.tar.zst`，校验文件为
`.scratch/static-experiments/handoff-data.sha256`。压缩后为 1,292,058,284 字节，约 1.29 GB，
已使用 tar 逐项比较包内内容与来源文件。数据包还包含场景选择、
S4 见证请求、容量测量和接手检查日志。将数据包复制到目标机器的相同仓库相对路径，
在仓库根目录核对 SHA-256，再解压到尚无同名记录的工作区。
既有记录中的原始绝对路径和来源身份必须保留；涉及目录移动的评价使用项目的资源重定位接口。

当前运行环境为 Ubuntu 22.04、ROS 2 Humble、Python 3.10，原生几何模块依赖
FCL 与 pybind11。Python 控制依赖见 `portable_oscbf/requirements.txt`；
绘图环境见 `requirements-static-report.txt`。新机器须重新构建原生模块。
构建与运行入口见[实验说明](static_obstacle_experiments.md)及
[项目运行指南](PROJECT_GUIDE.md)。

新机器的设备身份和运行时间必须重新测量。共同周期或正式清单不能沿用其他机器的
性能结论。所有工作继续限定在 sim。
