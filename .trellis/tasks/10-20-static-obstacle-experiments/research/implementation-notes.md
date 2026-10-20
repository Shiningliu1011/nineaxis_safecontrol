# 实现依据

## 现有基础

- 基准 commit：`34a2e34`，集成分支：`codex/static-obstacle-experiments`。
- `OscbfController`、`TrackingRun`、`JaxControlLoop` 组成现有控制链。
- `OscbfPlant` 使用 `SCurveDriverSimulator`，默认 500 Hz、80 ms 最短运动规划时间。
- 当前路径来源为 `data/nurbs/ik_input.mat`，有 14992 个点；公共变换和圆柱处理已存在。
- `jax_kernel_factory.py` 的路径控制先计算进给限制，再调用 QP；
  工具轴参考由实际投影取得，位置参考采用前方参考点。#173 需要共同参考及移动参考导数。
- `PathGeometry.project_local` 提供局部线段投影；S0 评价增加以前一次弧长为中心的窗口。

## 初始几何核查

命令：`python3 .scratch/static-experiments/check_initial_geometry.py`。
使用 `NineaxisKinematics`、仓库已有闭合 collision mesh 和 FCL；
外部查询使用生成 OBB 与 S0 工件、台面。结果：

- 36 个非相邻组合全部查询。
- 最小自碰撞间距为 Link3 / Link5：0.017028813686502647 m。
- 最小外部间距为 Link9 / workpiece：0.050002486121922055 m。
- 查询没有报告相交。此结果只覆盖 S0 初始状态。
- 完整输出：`.scratch/static-experiments/initial-geometry.json`。

`models/ninezzhou/collision_meshes/provenance.json` 记录已有闭合 mesh 的来源与 SHA-256。
本次实际加载确认十个 mesh 均闭合且为凸体。
FCL 官方 Python binding 文档支持直接用闭合凸 mesh 的顶点和面构造 `fcl.Convex`：
https://github.com/BerkeleyAutomation/python-fcl
正文已完整读取。该表示保持这些已有凸 mesh 的形状，支持实体查询。

## Generalized-SNS 来源

作者仓库：https://github.com/amrkzp/Generalized-SNS
读取的 commit：`0d4df38bc735e137a69796dc3ca3396c9de3a616`。
README、LICENSE、`GSNS_VL.m`、`FindScaleFactor.m` 已完整读取。
许可：CC BY 4.0。后续移植需保留作者、来源、许可及改编说明。

算法以任务缩放处理不可行的期望任务速度，逐个选择并饱和广义限制，
更新剩余零空间。作者实现包含多任务顺序、阻尼伪逆、缩放最优记录和限制复查。
实现通过解析实例和独立 HiGHS 任务缩放比较；原始输入与预期值保存在
`tests/test_static_sns.py`。E/S0 完整反馈已经通过精度、全部几何、
执行器限制、参考结算和运行时间检查。

论文全文：https://arxiv.org/html/2202.13184
全文、作者 `GSNS_VL.m`、`FindScaleFactor.m` 和 `damped_pinv.m` 已读取。
