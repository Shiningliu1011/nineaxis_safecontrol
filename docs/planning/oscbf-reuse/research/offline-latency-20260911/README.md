# 离线长程性能与组合负载补证

2026-09-11，继续[回归剖面与成本分布测量](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/3)及[选定控制率/延迟预算与实现策略](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/17)。仅研究原型，没有产品代码、控制参数或硬件链路修改。

## 对象与边界

本轮 HEAD `83545ea`；与首轮 `3ec19a41` 比较，`portable_oscbf/work`、`src/robot_safecontrol_moveit` 和控制 YAML 无版本差异。完整工作模块、节点、控制 YAML 的前后 SHA256 记录在各后端 JSON，输入 MAT 单独 SHA256。两后端使用相同 JAX 0.6.2、float64、相同固定输入和参数，实际设备写入 metadata。CUDA 仍使用独立 `.scratch/latency-3-gpu-venv`，没有改系统 Python。

`replay_prototype.py` 调用现有 elastic `path_tracking_step`，计时到主机可读结果返回（含必要同步和转换）；每场景先热身10步，初始化/预热独立记录。dt/dt_path均为0.01秒，保留首轮参数和初始q。CSV写入及额外统计在计时之外，可能影响下一步调度；不是实时周期调度验收。CPU/GPU顺序执行，无二者同时施压；没有控制桌面后台程序、温度或随机化试验顺序，因此小幅差异不能解释为确定因果收益。

- `full_disabled` / `full_far8`：原始14992点、1.8167634937米路径，理想 `q_next`反馈；到路径状态completed或20000步上限结束。远障为8个(10,10,10)米、半径0.1米球槽。
- `moving_near8`：初始TCP周围八个半径0.08米合成球，其中心在XY半径0.25米及Z±0.15米范围，并沿X以0.04sin(t)米移动，显式传入解析速度；3000步。**场景可能初始穿入机器人，故意作为不利计算负载，不代表可行的避障任务。**没有替换自碰撞OBB或删减约束。
- `far8_unloaded` / `far8_fusion_load`：同一远障序列各3000步，包括到达端点后的内核调用，用于匹配计算负载。组合组另起进程运行真实 `FusionEngine`，两路各12000合成点、8簇、51³网格、0.04米体素、目标10Hz。融合输出**未接入控制输入**；这是资源争用测量，不是真实双源端到端链路。融合单独记录实际周期数、有效周期、耗时。

继承首轮的q起点与路径存在约0.535米横向误差，最大值在首步。完整回放到达结束只说明这条理想模型运行结束，不是跟踪精度/真实任务验收。QP成功也不等于所有安全约束已满足，更不能用来掩盖初始穿入、弹性松弛、停滞或既有几何缺口。没有调整初始状态以美化本组性能。CPU/GPU移动回放的数值差异会导致完成步数不同，所以这不是逐步冻结输入的后端等价测试；固定输入的既有GPU诊断另见上一轮报告。

## 复现

仓库根目录运行；每个命令单独完成后再启动下一后端：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 JAX_PLATFORMS=cpu python3 docs/planning/oscbf-reuse/research/offline-latency-20260911/replay_prototype.py --name cpu
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 JAX_PLATFORMS=cuda XLA_PYTHON_CLIENT_PREALLOCATE=false .scratch/latency-3-gpu-venv/bin/python docs/planning/oscbf-reuse/research/offline-latency-20260911/replay_prototype.py --name gpu
python3 docs/planning/oscbf-reuse/research/offline-latency-20260911/analyze.py
```

所有模型数据、逐步耗时/QP/路径指标见CSV，汇总JSON保留完整最终q。编译缓存只在 `.scratch/offline-latency-cache`，不提交。第一次CPU组合组在保存负载JSON时遇到NumPy int64序列化错误；修复研究脚本、保留 `cpu-first-attempt.log` 与 `cpu-fusion-first-attempt.log`，重跑完整CPU矩阵成功后才使用其数据。

## 节点边界探针

`node_probe.py` 使用真实 `OscbfController._control_tick`，包含评价、故障/停滞分支、低通、消息构建及发布API返回。使用独立DDS domain182，输入/输出仅 `/wayfinder_offline3/input` 与 `/wayfinder_offline3/output`，不访问执行器主题。手动调用回调，不启动executor/timer，因此没有测定时器唤醒、DDS交付和CAN。反馈采用已平滑命令的理想下一状态，不能代表真实电机模型。

继承旧q起点时，CPU仅2步即触发 `TRACKING_STALLED`，日志描述“5s”，但实际源码条件仅检查历史长度≥2、进给/参考时间差/误差，未要求历史已覆盖5秒。保留原始 `cpu-node.json` 和日志；两点样本的分位数**不作为稳态尾延迟证据**。本票不修改这段产品逻辑，问题归控制/故障处理的后续验证。`--aligned` 使用仓库既有 `_work_start_configuration` IK方法求路径起点，独立记录初始q和结果；不会绕过节点保持判断。

```bash
source /opt/ros/humble/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 JAX_PLATFORMS=cpu python3 docs/planning/oscbf-reuse/research/offline-latency-20260911/node_probe.py --name cpu-aligned --aligned
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 JAX_PLATFORMS=cuda XLA_PYTHON_CLIENT_PREALLOCATE=false .scratch/latency-3-gpu-venv/bin/python docs/planning/oscbf-reuse/research/offline-latency-20260911/node_probe.py --name gpu-aligned --aligned
```

## 尚未闭合

本轮长程仅几十秒到数分钟级、有限场景，不是长期运行或最坏时延界。真实点云录包在本地检索未发现，本轮不伪造录包。传感器采集/驱动、真实ROS组合节点、调度交付、目标控制接入、近障可行任务与完整几何/任务正确性、设备反馈/停止仍需对应前置实现和验证。现有内核可保留，继续直接复用JAX/cbfpy/qpax和FusionEngine；新增仅计时包装、合成输入与记录器，没有证据支持更换求解器、删障碍或降低精度。
