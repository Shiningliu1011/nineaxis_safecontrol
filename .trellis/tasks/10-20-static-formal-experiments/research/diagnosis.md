# 本机复现环境

## 几何来源记录

入口为 `portable_oscbf/tests/test_collision_geometry_artifact.py`。
测试通过两个生成器的实际检查，核对闭合 STL、URDF、顶点包含关系、
来源 SHA-256、tetrahedralization、覆盖证明及完整 geometry。

NumPy/OpenBLAS 的动态 CPU 选择影响顶点与平面的最后几位计算。
本机分别使用 `OPENBLAS_CORETYPE=Sandybridge` 和 `Haswell` 执行来源检查，
Haswell 与归档保存的全部来源数值一致。原始运行的 manifest 已保存生成器、
provenance 和模型的身份，因此本机环境固定使用 Haswell。
原始资源和浮点精度完整保留，检查继续采用精确相等。

环境入口为 `.scratch/host-recovery/env.sh`。
`static_experiment_provenance.environment_identity()` 保存
`OPENBLAS_CORETYPE`，用于区分数值计算环境。

验证命令：

```bash
source .scratch/host-recovery/env.sh
python -m pytest portable_oscbf/tests/test_collision_geometry_artifact.py -q
```

结果：10 passed in 30.84s，退出 0。
日志：`.scratch/host-recovery/geometry-final.log`。

## 独立绘图环境

`--report-python` 保留虚拟环境 Python 的符号链接路径。
子进程继承 ROS Python 路径，并将当前项目源码置于最前。
独立绘图环境包含实际 FCL 扩展、控制评价依赖和项目字体。
完整 S0 按需重算并导出六类图表，命令退出 0，
图片完整性、尺寸、字符覆盖、来源数据和字体 SHA-256 均通过。

## 控制周期测量

本机 i7-14700 同时包含 Performance-core 和 Efficient-core。
性能验收使用 `taskset -c 0-15`，将控制进程及其子进程限制到本机
Performance-core 的逻辑 CPU；环境身份保存 CPU affinity。

`tests/test_oscbf_controller_smoke.py::test_perf_report_p95_within_budget`
实际执行 300 步，P50 为 10.878 ms、P95 为 13.890 ms、
最大 17.936 ms，20 ms 预算内无超时，QP 失败数量为 0。
日志 `.scratch/host-recovery/performance-pcores.log`；
报告 `output/host-recovery-regression/pcores/oscbf_m10_perf_isolated.md`。
测试阈值和控制算法保持项目定义。

完整紧凑 S0 sim 的周期和容量证据见 `storage-validation.md`。
