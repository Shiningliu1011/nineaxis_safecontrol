# CPU/GPU 同版本同精度对照（2026-09-11）

两后端均 JAX/jaxlib0.6.2、float64，使用同一个原型、同样300步/场景。GPU为RTX3060 Laptop 6GiB，禁用预分配；CUDA插件0.6.2安装在独立`.scratch/latency-3-gpu-venv`，完整依赖见gpu-environment.txt。当前系统解释器未安装CUDA插件，不受此隔离安装影响。

## 公开路径调用（包含所需主机边界）

| 模式 | 外障 | CPU p95 ms | GPU p95 ms | CPU max ms | GPU max ms | GPU >20ms | GPU QP失败 |
|---|---|---:|---:|---:|---:|---:|---:|
| elastic | disabled | 8.495 | 17.415 | 9.067 | 32.521 | 0.33% | 0/300 |
| elastic | enabled_far | 12.444 | 19.634 | 13.412 | 24.975 | 1.67% | 0/300 |
| rate_slack | disabled | 8.881 | 16.761 | 9.473 | 31.532 | 0.67% | 0/300 |
| rate_slack | enabled_far | 13.205 | 19.240 | 13.973 | 24.165 | 0.67% | 0/300 |

## 固定输入的设备驻留内核与初始化

| 模式 | 场景 | CPU内核p50/p95 ms | GPU内核p50/p95 ms |
|---|---|---:|---:|
| elastic | disabled | 6.918/7.724 | 14.006/14.477 |
| elastic | enabled_far | 9.681/10.663 | 15.573/15.855 |
| rate_slack | disabled | 7.215/7.825 | 13.590/13.950 |
| rate_slack | enabled_far | 9.956/10.789 | 14.731/15.053 |

- GPU elastic init_cbf（构造+编译+预热同步）82.822s；预热后首次调用22.263ms。

- GPU rate_slack init_cbf（构造+编译+预热同步）54.107s；预热后首次调用22.287ms。

## 结论边界

- 本轮当前GPU配置四组均未显示优于CPU的路径调用尾延迟；设备驻留内核也更慢，因此不能只归因于PCIe拷贝。保持CPU基线，不将此GPU配置推荐为生产替代。不是对所有GPU或所有编译优化的结论。
- 每后端四组的独立模块与融合输出在rtol=atol=1e-7一致；这只证明该后端的探针没有改变结果，不是跨后端轨迹等价门。CPU/GPU回放终点进度有差异，例如elastic disabled约0.144381m与0.143731m；完整命令/任务误差对照尚未完成，不把同精度说成逐位相同。
- GPU使用新装且记录的CUDA依赖，CPU/GPU同精度而非同运行库；未做多轮交错ABBA、功耗/热稳态控制、组合负载或GPU失效回退注入。后台桌面仍运行。不能宣布性能差距为全工况上界。
- 数据传输解释见CPU报告：重复物化已就绪输出可能缓存，独立device_put含已驻留参数；不将其充作新鲜H2D/D2H带宽证据。生产比较以完整公开调用为主，拷贝不重复累计。
- GPU环境初次venv创建因ensurepip缺失失败，改用without-pip/system-site-packages并由宿主pip --python定向安装后成功。未安装系统apt包、未更改系统JAX。

复现：

```bash
JAX_PLATFORMS=cuda OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 XLA_PYTHON_CLIENT_PREALLOCATE=false .scratch/latency-3-gpu-venv/bin/python docs/planning/oscbf-reuse/research/3-latency-evidence/profile_prototype.py --output gpu-new
```

原始结果：[GPU results](gpu/results.json)、[GPU metadata](gpu/metadata.json)、[CPU results](cpu/results.json)。两后端测量前后所记录产品源码hash一致。
