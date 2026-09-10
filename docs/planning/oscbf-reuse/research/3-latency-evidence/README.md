# 当前 CPU 成本分布（2026-09-11）

单位 ms，NumPy linear percentile。每个固定/移动测量300样本，模块拆分100样本；逐样本数据见 [results.json](cpu/results.json)。

## 移动回放：完整 path_tracking_step 返回至主机

| 模式 | 外障 | p50 | p95 | p99 | max | >10ms | >20ms | QP失败 |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| elastic | disabled | 7.712 | 8.495 | 8.851 | 9.067 | 0.0% | 0.0% | 0 / 300 |
| elastic | enabled_far | 11.510 | 12.444 | 12.951 | 13.412 | 100.0% | 0.0% | 0 / 300 |
| rate_slack | disabled | 8.264 | 8.881 | 9.177 | 9.473 | 0.0% | 0.0% | 0 / 300 |
| rate_slack | enabled_far | 12.124 | 13.205 | 13.706 | 13.973 | 100.0% | 0.0% | 0 / 300 |

## 固定输入的阶段测量

下表为 p50 / p95；不同运行段的分位数不能相加。CPU device_put 不是GPU传输证据。仪器化入口在内核返回处同步，诊断耗时不再包含等待内核；它会改变执行排布。

| 阶段 | elastic disabled | elastic enabled_far | rate_slack disabled | rate_slack enabled_far |
|---|---:|---:|---:|---:|
| 公开入口 diagnostics=True | 7.984 / 8.970 | 11.449 / 12.492 | 8.139 / 8.997 | 11.844 / 12.745 |
| 公开入口 diagnostics=False | 8.016 / 9.094 | 11.539 / 12.637 | 8.339 / 9.106 | 11.831 / 12.787 |
| 融合内核，输入已在设备 | 6.918 / 7.724 | 9.681 / 10.663 | 7.215 / 7.825 | 9.956 / 10.789 |
| 入口实参device_put（部分已驻留） | 0.244 / 0.285 | 0.243 / 0.267 | 0.243 / 0.277 | 0.253 / 0.289 |
| 已同步输出重复物化（可缓存） | 0.054 / 0.065 | 0.050 / 0.052 | 0.054 / 0.061 | 0.053 / 0.086 |
| 入口输入准备（含边界） | 0.267 / 0.385 | 1.072 / 1.357 | 0.275 / 0.410 | 1.125 / 1.475 |
| 主机诊断，内核已就绪 | 0.335 / 0.496 | 0.291 / 0.409 | 0.372 / 0.559 | 0.405 / 0.546 |
| 冻结QP矩阵构造 | 不适用 | 不适用 | 6.695 / 7.652 | 10.823 / 11.920 |
| 冻结QP纯求解 | 不适用 | 不适用 | 0.153 / 0.199 | 0.142 / 0.160 |

## 独立模块探针

现有九个 JIT 模块分别 dispatch 和同步。它回答候选成本分布，不是XLA融合后逐算子精确归因。拆分输出逐叶与融合输出在 rtol=atol=1e-7 一致。

| 模块 | elastic disabled p50/p95 | elastic enabled_far p50/p95 | rate_slack disabled p50/p95 | rate_slack enabled_far p50/p95 |
|---|---:|---:|---:|---:|
| _path_kinematics | 0.076 / 0.129 | 0.080 / 0.106 | 0.077 / 0.118 | 0.079 / 0.112 |
| _path_sample | 0.063 / 0.097 | 0.068 / 0.091 | 0.063 / 0.102 | 0.073 / 0.100 |
| _path_cap_nominal | 0.090 / 0.121 | 0.095 / 0.111 | 0.091 / 0.121 | 0.095 / 0.130 |
| _path_constraint_terms | 6.450 / 7.356 | 9.344 / 10.048 | 6.220 / 7.144 | 9.344 / 10.046 |
| _path_feed_caps | 0.131 / 0.201 | 0.145 / 0.190 | 0.137 / 0.203 | 0.164 / 0.229 |
| _path_advance | 0.192 / 0.275 | 0.177 / 0.228 | 0.185 / 0.259 | 0.297 / 0.384 |
| _path_control_nominal | 0.095 / 0.138 | 0.100 / 0.138 | 0.091 / 0.135 | 0.122 / 0.158 |
| _path_solve | 0.335 / 0.465 | 0.345 / 0.435 | 0.273 / 0.360 | 0.274 / 0.354 |
| _path_finalize | 0.091 / 0.132 | 0.095 / 0.116 | 0.076 / 0.121 | 0.084 / 0.123 |

## 初始化与发布调用

新建独立 JAX 持久缓存目录，初始化计时包含构造、编译、预热执行及同步，不能标成纯编译耗时。内部打印的预热阶段和外层 init_cbf 并非同一边界。
- elastic: init_cbf 33.342 s；预热后首次公开调用 10.076 ms。
- rate_slack: init_cbf 20.973 s；预热后首次公开调用 9.586 ms。
- 独立 ROS serialization: p50/p95/p99=0.0059/0.0078/0.0088 ms。
- 独立 ROS publish_call: p50/p95/p99=0.0059/0.0062/0.0068 ms。

发布探针使用独立 DDS domain181 与 `/wayfinder_profile3/offline_probe`，没有订阅者，只测序列化/发布API返回，不测传输到达或实际执行。两个成本存在包含关系，不能相加。首次选domain237超过DDS端口范围而失败，改用181后退出0。

## 复现与限制

```bash
JAX_PLATFORMS=cpu OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 docs/planning/oscbf-reuse/research/3-latency-evidence/profile_prototype.py --output cpu-new
source /opt/ros/humble/setup.bash
python3 docs/planning/oscbf-reuse/research/3-latency-evidence/publish_prototype.py
```

- i7-11800H，接交流电，JAX/jaxlib0.6.2、x64；源码前后hash一致。精确版本、环境和起点见 metadata.json / host.json。浏览器/桌面仍在运行，不是专用实时测试机；后半程有隔离GPU依赖下载活动，未人为隔离调度噪声。
- 使用14992点仓库蝴蝶轨迹、surface_normal、feedrate_scale=3.5、当前5D/x64/增益。移动回放为300步理想q_next反馈，未跑全程、MuJoCo、双源点云、真实低通执行或组合负载。
- enabled_far是八槽位于(10,10,10)m、半径0.1m的人工远障，启用几何分支；不代表近障、接触、未知覆盖或最坏场景。disabled只是对照工况，不是优化时删除障碍。
- elastic为当前节点构造模式；rate_slack是冻结QP研究候选，不能以其更快/更慢证明可替换生产模式。冻结QP矩阵为11变量、81不等式。
- diagnostics开关在path_tracking_step中未读取，两组差异是运行噪声，不是关闭诊断的收益；不把普通tracking_step的fast分支误当路径fast分支。
- 没有重放历史19.367ms的同一版本/配置/工况，无法证明相对历史的因果回归幅度。历史值不作当前验收依据。
- 未改产品代码；冻结QP既有独立一致性测试1 passed。初次脚本导入actuator_limits失败，修正为work.actuator_limits后四组完成。

GPU解释补充：重复对同一已就绪输出做np.asarray可命中主机缓存，不是新鲜D2H带宽测试；device_put入口实参也含已驻留数组，不是全量新鲜H2D。完整公开调用包含本轮所需拷贝、同步和主机转换，是当前后端比较的主要口径。独立传输项不能再次加到完整调用耗时。

同脚本GPU补测已完成，见 [CPU/GPU对照](GPU-COMPARISON.md)。
