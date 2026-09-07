# 笔记本 CPU/GPU 启用、分工与验证研究

- 日期：2026-09-07。
- 状态：研究报告；用户已接受其中的后端对照实测与验证准入路线，见[阶段决议](../issues/05-control-adoption.md)。其余具体操作建议仍需在对应实施/验证阶段落实。用户已接受 CPU/PCL 距离查询基线、GPU 距离场通过验证后接入；**控制求解采用 CPU 还是 GPU 尚未定案**。
- 关联：[连续障碍观测契约与距离表示](../issues/04-safety-observation.md)、[上游控制接入与九轴扩展](../issues/05-control-adoption.md)。
- 方法：查验官方文档及版本源码，结合主会话只读环境核验；没有安装软件、调用 `jax.devices()`、运行 GPU 负载、连接机器人或测量性能。

## 推荐结论

**这台笔记本可以采用 CPU/GPU 协同架构，但应按模块显式选择后端。近期保留 CPU/PCL 感知基线，建议控制先建立同版本 CPU 基线，再用 GPU 做独立对照；GPU 达到全链路验收门槛后，才接管对应模块。** CPU/GPU 不按两个传感器一对一分配；两源应继续融合成统一观测，计算设备是实现选择。

| 模块 | 近期建议 | GPU 接入条件 |
|---|---|---|
| 官方传感器驱动、TF、时效/覆盖检查、通信与停车监督 | CPU 执行，保留独立故障处理 | GPU 加速不能取代消息年龄、覆盖及执行命令检查 |
| 双源自体过滤、ROI、PCL 空间索引及距离基线 | 沿用已接受的 CPU/PCL 路线 | 只有剖析证明此处是瓶颈，才核验上游 GPU 模块的功能等价及搬运收益 |
| JAX/CBFpy/qpax 控制求解 | **建议**同版本 CPU 基线；GPU 在隔离环境离线/影子对照 | 同精度、同约束，含输入上传和输出回主机的尾延迟改善，且故障行为通过验证 |
| 稠密 TSDF/ESDF 建图 | 可选研究，暂不作为必需依赖 | 本机 6 GB 显存低于 Isaac ROS release-3.2 官方 x86 基线；精简组件实验需单独证明可行 |

表中分工是项目设计推论，不是厂商给出的性能结论；用户已批准的路线范围以关联阶段决议为准。

## 本机与现有代码事实

主会话本轮只读检查得到：Ubuntu 22.04.5、i7-11800H（8 核 16 线程）、约 16 GB RAM、RTX 3060 Laptop 6 GiB、NVIDIA 驱动 610.43.02。`/usr/bin/python3` 为 3.10.12；其包元数据包含 JAX/jaxlib 0.6.2、CBFpy 0.0.1、qpax 0.1.4，未发现 jax-cuda12/13-plugin 或相应 PJRT 包；`/opt/ros/humble` 存在。这仅描述被检查的解释器，不能推出其他虚拟环境或正在运行的 ROS 节点也使用同一套包，亦不能认定 JAX GPU 已成功启用。

仓库 [控制配置](../../../../config/oscbf_controller.yaml)设置 `enable_x64: true`；[控制 facade](../../../../portable_oscbf/work/jax_control_facade.py)设置 JAX x64；[ROS 控制器](../../../../src/robot_safecontrol_moveit/oscbf_controller.py)将多项计算结果转换为 `np.asarray`。因此 GPU 对照必须包含最终结果回主机的成本，保留当前 FP64 与求解容差，不能通过降低精度换取表面速度。

## JAX GPU 如何启用：版本必须分开看

官方当前安装文档支持 Linux x86_64 的 NVIDIA GPU。CUDA 12 路线要求 SM 5.2+、Linux 驱动 525+；CUDA 13 路线要求 SM 7.5+、驱动 580+。NVIDIA 将 GeForce RTX 3060 列为计算能力 8.6；结合本机驱动，本机具备候选硬件条件，但这不证明指定 Python 环境的插件和运行库已经兼容。[JAX 安装](https://docs.jax.dev/en/latest/installation.html)、[NVIDIA 计算能力表](https://developer.nvidia.com/cuda/gpus)

当前文档推荐通过 pip 配套 CUDA/cuDNN，并推荐 CUDA 13；**本机 JAX 0.6.2 则应查对应版本**：其 `setup.py` 要求 Python ≥3.10，`cuda12` extra 将 jaxlib 与 CUDA 12 plugin 约束到 0.6.2 配套范围。近期对照应在独立环境按这一版本族建立 CPU/GPU 对照锁文件，再做兼容核验；不要直接在 ROS 系统 Python 执行不锁版本的升级命令。较新 JAX 的 Python 支持及依赖范围会变化，升级 JAX、CBFpy 与启用 GPU 应分开验证。[JAX 0.6.2 依赖源码](https://raw.githubusercontent.com/jax-ml/jax/jax-v0.6.2/setup.py)、[Python 支持策略](https://docs.jax.dev/en/latest/deprecation.html)

验收应记录实际启动解释器、jax/jaxlib/plugin/PJRT/CUDA/cuDNN 版本、实际设备与数组所在设备；明确 CPU、GPU 两种启动配置，GPU 配置若未发现指定设备则启动失败，避免静默换到未经验证的后端。上述是项目建议；本轮没有安装或启用操作。

## 为什么小规模控制求解未必适合直接搬到 GPU

JAX 有 JIT 编译与异步派发：首次调用包含编译成本，计时时必须等待计算完成；官方基准指南还要求区分主机到设备传输、精度和预热。`np.asarray` 等主机读取会等待结果，不能只量 Python 函数返回的时间。[JAX 基准指南](https://docs.jax.dev/en/latest/benchmarking.html)、[异步派发](https://docs.jax.dev/en/latest/async_dispatch.html)

据此对本项目作出的推论是：九轴单次 QP 的实际代价还取决于约束数量、运动学/导数、求解迭代与搬运；GPU 的并行优势可能被派发和回传成本抵消，也可能在较大的几何/约束批量下胜出。**目前没有本机 CPU 胜出或 GPU 胜出的实测证据。** 不应把上游演示频率或 GPU FLOPS 当成双传感器到控制命令的周期保证。

建议预热所有允许输入形状，并在运行期间检测非预期重编译；先固定模型、约束容量和精度，再比较。由 `np.asarray` 形成的必要命令回传保留，非必要诊断可另行评估降频，但不能在本轮研究中当作已完成优化。

## PCL 不会因为安装 CUDA 而自动变为 GPU

PCL 1.12.1 的 GPU 子系统需满足 `BUILD_GPU` 与 `CUDA_FOUND`；公开 API 的 `pcl::gpu::Octree` 使用 `DeviceArray`，区分主机查询与 GPU 批量查询。普通 PCL 管线不会仅因为显卡存在自动转为这些 API。选择 GPU 意味着显式构建、调用与数据搬运适配。[PCL 1.12.1 构建源码](https://raw.githubusercontent.com/PointCloudLibrary/pcl/pcl-1.12.1/gpu/CMakeLists.txt)、[GPU Octree API](https://pointclouds.org/documentation/classpcl_1_1gpu_1_1_octree.html)

因此先复用 CPU 模块并测量瓶颈；若要替换查询后端，应验证精确/近似查询、返回距离单位、容量上限及边界行为，而非重写 CUDA 空间索引。GPU 实现仍须满足同一障碍覆盖与不确定性契约。

## 显存与 nvblox 的实际约束

Isaac ROS release-3.2 明确面向 ROS 2 Humble；官方 x86 Compute Setup 要求 Ubuntu 22.04+、16 GB 主存、Ampere 或更新、CUDA 12.6+，**最低 8 GB 显存，推荐 12 GB+**。本机 6 GB 低于这套官方基线，所以不推荐将整套 Isaac ROS nvblox 作为本机近期默认部署。该结论不等于单独精简 nvblox 核心绝对无法运行；后者是需要固定版本、限定地图/分辨率并实测的新实验，不能宣称获得整套官方配置支持。[Humble 支持](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/index.html)、[官方硬件要求](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/hardware_setup/compute/index.html)

JAX 默认在首次 GPU 运算预分配 75% 总显存；对 6 GiB 卡约为 4.5 GiB，可能与桌面、可视化或建图争用。官方提供预分配比例及关闭预分配选项；关闭可能增加碎片，`platform` 分配器释放更积极但很慢。建议按组合负载制定预算并实测，不把某个环境变量当成实时性保证。[JAX 显存管理](https://docs.jax.dev/en/latest/gpu_memory_allocation.html)

## 接入验证门：先测量，再定生产后端

以下是拟议验收要求，数值预算由现有全链路时延/故障票据确定，本轮不杜撰通过阈值。

1. **可复现环境：** CPU/GPU 对照使用同一算法、模型、CBFpy/qpax 版本、FP64、容差和场景；记录完整环境与实际后端。先完成离线测试，再影子运行，最后才考虑控制接管。
2. **正确性：** 对比最终命令、约束残差、不可行/松弛/迭代上限、NaN 和超时处理。速度结果不能掩盖数值行为差异。
3. **全链路延迟：** 分开报告冷启动编译、预热稳态、传感器数据年龄、融合/距离查询、输入上传、求解与同步回传、最终发令。报告 p50/p95/p99、观测最大值和截止期违约次数；样本最大值不是严格最坏情况证明。
4. **组合负载：** 同时运行两个真实传感器、规划、日志及预期可视化；记录 CPU/RAM/显存、温度、功耗、频率与丢帧，并在热稳定后评估。NVIDIA 提供相关监控项，GeForce 某些项可能不可用，应记录 N/A，不补造读数。[nvidia-smi 官方说明](https://docs.nvidia.com/deploy/nvidia-smi/index.html)
5. **故障：** GPU 不可用、内存不足、查询超时或进程退出，应使对应观测/控制结果失效，并触发既定停车或已验证降级。CPU 后备只有预热、覆盖、容量、时延与切换均验证后才能接管；不能临时重新编译后继续发令。

建议供下一轮决策的表述：**允许 CPU/GPU 并用；近期 CPU/PCL 为观测基线，控制以同版本 CPU 基线和 GPU 对照实测选择；GPU 只在兼容、正确性、组合负载尾延迟及故障验证通过后启用为生产后端。本机暂不把完整 Isaac ROS nvblox 设为必需项。**
