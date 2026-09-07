# 上位机部署约束与本机事实

2026-09-07 用户明确：当前笔记本运行 Ubuntu 22.04，作为上位机发送控制指令，允许使用 CPU 和 GPU。此为部署边界，不是硬实时或吞吐量验收结论。

| 项目 | 本机只读查询结果 |
|---|---|
| 系统 | Ubuntu 22.04.5 LTS，x86_64 |
| CPU | Intel Core i7-11800H，8 核 / 16 逻辑处理器 |
| 内存 | free -h 显示总计约 15 GiB |
| GPU | NVIDIA GeForce RTX 3060 Laptop GPU |
| 显存 | 6144 MiB（6 GiB） |
| NVIDIA 驱动 | 610.43.02 |

来源：本机 lscpu、/etc/os-release、free -h、nvidia-smi 查询。未安装或升级任何依赖；未运行压力测试、传感器并发、CUDA/JAX/nvblox 兼容或延迟测试。驱动查询成功不证明 CUDA toolkit、各框架版本或 GPU 后端已可用。

CPU/GPU 分工随[距离观测决策](issues/04-safety-observation.md)、[控制接入决策](issues/05-control-adoption.md)与[全链路验证](issues/06-validation-contract.md)确定。需要在该笔记本上验证感知、规划、求解并发时的显存/内存占用、数据搬运和尾延迟，不能沿用单组件演示性能。笔记本供电与热稳态条件应随性能结果记录。


## 当前解释器依赖快照

同日本地 `/usr/bin/python3` 为 Python 3.10.12，包元数据为 JAX/jaxlib 0.6.2、CBFpy 0.0.1、qpax 0.1.4；该解释器未查到 jax-cuda12/13-plugin 或对应 pjrt 包。`/opt/ros/humble` 存在。这只是包元数据盘点，不代表其他环境的状态，也不等于已执行 JAX 设备发现或 GPU 计算验证。

控制器在 `oscbf_controller.py` 中将多项内核结果转回 NumPy；如果评估 GPU，应将这些主机同步/数据搬运计入每周期延迟。不能仅计时异步提交或忽略实际命令发布所需的数据回传。

CPU/GPU 启用与分工的官方证据、版本建议和实测准入条件见[部署研究](research/cpu-gpu-deployment.md)。本轮未安装或启用 GPU 后端。

用户已确认[CPU/GPU 对照实测与准入路线](issues/05-control-adoption.md)；这是部署选择方法，尚非后端启用或性能验收结果。
