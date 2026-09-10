# GPU 延迟诊断：是否缺 CUDA 组件（2026-09-11）

结论：**没有发现缺少 CUDA 核心依赖、CPU 回退、错误设备或运行中降频能够解释这次慢测的证据。GPU 加速功能正常；当前单步控制的工作形态与双精度成本是证据支持的解释。** 常见调度/分配选项逐项改变后，仍未消除差距。没有修改产品代码、系统依赖、生产精度或预算。

本报告是[当前CPU/GPU对照](../3-latency-evidence/GPU-COMPARISON.md)的进一步诊断，不是宣布所有GPU实现都慢，也不是完成GPU生产准入。

## 反馈环与最小化

原路径沿用当前 elastic、9-DOF/5D、x64、无外障、固定q/上次命令；只计预热后、输入已驻留设备且block_until_ready完成的内核。每轮100样本，计时与profile采样分开。

| 复现 | CPU resident p50 ms | GPU resident p50 ms | GPU/CPU |
|---|---:|---:|---:|
| 原14992点蝴蝶路径 | 6.561 | 13.539 | 2.064 |
| 缩成两点路径 | 7.076 | 13.289 | 1.878 |
| 原路径GPU再次运行，记录频率 | 同上CPU基线 | 13.564 | 2.067 |

两点路径仍保留真实路径控制、CBF约束与QP，外部轨迹规模已不是复现所必需；不删除约束来让测量变快。这里的RED仅表示复现GPU慢于CPU20%以上的现象，不是系统安全阈值，不把阈值放宽让诊断变绿。

已运行的检查命令：`python3 .scratch/gpu-latency-diagnosis/check.py full-cpu full-gpu`，输出 `GPU/CPU resident p50 ratio=2.064`、`RED: reproducible GPU slowdown >20%`，退出1。末次原场景再次得到RED。原型代码迁入本目录后可按下方命令重新产生数据，不依赖聊天结论。

## 核心组件与真实执行

[实际加载库](loaded-cuda-libraries.txt)记录11条CUDA相关动态库路径：驱动来自系统，其余主要来自独立`.scratch/latency-3-gpu-venv`。cuBLAS、cuSolver、cuDNN、cuFFT、cuSPARSE、CUPTI、nvJitLink均实际加载；`ptxas`和`libdevice.10.bc`也存在。

[版本查询](cuda-versions.txt)成功：运行时CUDA12.9、cuBLAS12.9.2、cuSolver11.7.5、cuDNN9.25.1等，JAX CUDA插件/PJRT均0.6.2。构建版本与运行版本并非逐项相同，已保留精确记录；没有声称核验了所有历史版本的性能兼容性。此前JAX初始化的兼容性检查与实际计算均成功。

[包检查](pip-check.txt)唯一报告为PyNaCl缺cffi，属于该共享环境的另一依赖链，当前GPU控制路径没有使用它；没有为了消除此提示更改环境。Nsight命令未安装，使用已有JAX/CUPTI profiler已成功得到GPU轨迹；缺Nsight不是运行依赖缺失。采集时出现TensorFlow Python profiler hook导入提示，但控制测量在采集前已经完成，而且实际GPU trace仍成功生成；该提示不是此次慢测的原因。

最后一次实际计时期间的[设备记录](full-gpu-repeat-power.csv)：P0，SM1425→1980MHz、显存7001MHz、温度64–68°C，throttle/event reason始终0。没有观察到这段测量处于P8闲置频率或温度/功耗限制事件。进程退出后回到P8是正常空闲状态，不能拿退出后截图解释运行速度。

## GPU 确实能加速：矩阵对照

[标准矩阵测试](matmul-results.json)使用同机JAX、设备驻留输入、同步完成、最高matmul精度配置；100个小矩阵样本或20个1024矩阵样本，随机种子固定，结果有限。仅用于区分GPU运行异常与工作规模/精度效应，不代表控制正确性。

| 矩阵乘法 | CPU p50 ms | GPU p50 ms |
|---|---:|---:|
| 9×9 float64 | 0.00564 | 0.04492 |
| 81×81 float64 | 0.04242 | 0.08948 |
| 1024×1024 float32 | 2.565 | 0.547 |
| 1024×1024 float64 | 10.548 | 11.395 |

同一GPU在较大的float32任务上约4.69倍加速，而小矩阵调用与本机float64任务没有同样收益。单精度结果支持“GPU功能正常”；双精度对照支持精度成本是相关因素，但不能把该比值直接套成完整控制链的预计提速。

## 工作形态：大量小操作与依赖

20步原路径设备时间线记录**52,220个GPU事件，即2,611个/步**。这是内核、复制等设备事件合计，不能全部称为kernel launches。每步包含约217次设备内复制、29次设备到主机的小额回传和57次cuGraphLaunch，说明默认图执行确实在工作。

回传中可定位到QP循环条件和SVD调用；例如条件分支传回4字节判定值，不能用“最终输出已留在GPU”推断内核内部没有主机交互。[trace摘要](trace-summary.json)保留具体事件与定位信息。设备事件累计约12.38ms/步，仪器化主机范围约16.20ms/步；profiler会扰动执行，不拿这些数替代普通计时。

HLO证据含440个静态cuBLAS GEMM custom-call位置、7个while、3个conditional，以及cuSolver QR/SVD/LU。静态位置数不等于每步动态调用次数。两处3×9 SVD custom-call可追溯至`oscbf_velocity_config.py`奇异性约束的SVD及其导数；QP while可追溯至qpax的elastic求解器。大量小规模、顺序相关的双精度运算和求解器调用，需要调度与同步，和一块足够大的并行矩阵工作不同。

## 单变量配置实验

全部使用同一个两点控制场景、x64、100样本；每次只改变表内一项，不改控制约束。

| GPU配置 | resident p50 ms | 相对默认变化 |
|---|---:|---:|
| 默认（之前关闭预分配） | 13.289 | — |
| command buffer增加WHILE/CONDITIONAL | 13.240 | -0.37% |
| 开启latency hiding scheduler | 13.177 | -0.84% |
| 开启内存预分配 | 13.055 | -1.76% |

三组固定输入控制输出与默认GPU相同。这些小差异未经多轮置信区间验证，不声称是可重复优化收益；可以确认它们都没有解决约2倍差距。安装版本的[XLA flags默认值](xla-flags.txt)显示，Triton GEMM和常见command buffer类别本来已经开启；不是缺一个单独Triton包。增加循环/条件类别后trace仍有29次D2H/步，未形成消除内部回传的证据。

此外尝试直接将当前控制原型切为float32，在构造CBF时出现f64→f32 reshape类型不一致，[错误记录](short-gpu-f32.log)。cbfpy导入会启用x64，现有模型/碰撞模块也包含双精度常量，不能把一个开关当作已支持的低精度控制实现。未修补或降低产品精度，因此**没有完整控制链float32速度结果**。

## 可保留结论与下一步

- 保留现有CPU生产候选和已接受的OBB/安全约束、JAX/cbfpy/qpax复用原则；不是采购或安装更多组件即可解决的已证实缺包问题。
- 若后续要让GPU获益，应研究减少重复的小矩阵分解与导数、降低顺序调度/回传、适当合并或批处理独立计算。先证明数学与几何语义保持，再对同工况比较；不为提速删除约束、替换OBB或直接降精度。
- 原场景固定输入的CPU/GPU安全命令最大分量差约1.98e-13；不扩张为全轨迹/近障/不可行情况下的等价证明。
- 未证明GPU端每一微秒的唯一根因，也未穷尽驱动/运行库/编译器版本；本轮已排除常见缺依赖、错误设备、简单调度/分配开关、运行降频和长轨迹规模的解释。
- 因没有发现需要修补的缺包故障，本轮停在有反证和实测支撑的诊断，没有通过改阈值或改语义让性能断言变绿。没有产品改动，故不新增生产回归测试；所有临时包装均在独立诊断原型中。所记录产品源码hash与上一轮一致。

## 复现

在仓库根目录执行。隔离GPU环境沿用上一轮，不安装系统包。

```bash
JAX_PLATFORMS=cpu OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .scratch/latency-3-gpu-venv/bin/python docs/planning/oscbf-reuse/research/gpu-latency-diagnosis/probe.py --name rerun-cpu
JAX_PLATFORMS=cuda XLA_PYTHON_CLIENT_PREALLOCATE=false OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .scratch/latency-3-gpu-venv/bin/python docs/planning/oscbf-reuse/research/gpu-latency-diagnosis/probe.py --name rerun-gpu
python3 docs/planning/oscbf-reuse/research/gpu-latency-diagnosis/check.py rerun-cpu rerun-gpu
```

`--path short`最小化，`--trace`生成CUPTI trace，`--hlo`保存压缩HLO。首轮含构造/编译约19–83s；缓存后的迭代测量本身数秒，初始化单独记录。原始大文件保存在`.scratch/gpu-latency-diagnosis/*-trace/`及`full-gpu.hlo.txt.gz`，本目录提交有界的摘要与原型。`analyze_trace.py`读取该原始trace位置；机器迁移时应重采或复制此目录。

官方依据：[JAX 0.6.2 GPU性能文档](https://github.com/jax-ml/jax/blob/jax-v0.6.2/docs/gpu_performance_tips.md)说明性能flags依赖版本，且文档主要针对神经网络，不能把推荐开关当成九轴控制的性能保证；[NVIDIA CUDA 12.6 指南的指令吞吐表](https://docs.nvidia.com/cuda/archive/12.6.0/cuda-c-programming-guide/index.html#arithmetic-instructions)区分架构与浮点精度的吞吐。本报告具体收益与差异以本机实测为准。
