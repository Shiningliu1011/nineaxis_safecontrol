# Generalized-SNS 来源

已读取作者论文全文：
[Kinematic Control of Redundant Robots with Online Handling of Variable Generalized Hard Constraints](https://arxiv.org/html/2202.13184)。
算法将限制统一写为 `B_min <= A u <= B_max`，每次饱和最限制任务的一个组合，
重新计算投影与速度；自由度不足时采用此前最大有效任务缩放量。
误差校正属于期望任务速度的一部分，缩放保持整个任务速度的方向。

作者实现为 `amrkzp/Generalized-SNS`，
commit `0d4df38bc735e137a69796dc3ca3396c9de3a616`。
已读取 README、LICENSE、GSNS_VL.m、FindScaleFactor.m、damped_pinv.m。
许可为 CC BY 4.0；移植需要保留作者、来源与许可说明。
原实现使用 SVD 阻尼参数 epsilon=0.1、lambda=0.001，以及 rank 容差 1e-6。

E 组需保留该任务缩放含义，记录缩放量和实际路径进给；共同构型辅助必须保留
首要任务结果及所有限制。独立核对须使用作者实现或独立数值优化，运行与精度
结果由实际记录决定。当前机器未安装 Octave/MATLAB。
