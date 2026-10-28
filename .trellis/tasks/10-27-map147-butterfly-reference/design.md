# 技术安排

## 数学表达与所有权

`pipe_assets` 生成器通过既有 `pipe_nurbs_geometry` 加载原三次单位权重曲线，并用 SciPy 将其转换为区间幂系数。共享相似变换由 `reference_trajectory_transform` 提供；两次投影参数由真实 `IKTrajectoryData` 加载及 `set_surface_normal_orientation` 过程捕获。生成器验证全部来源样本后保存参数。

`portable_oscbf/work/continuous_path_reference.py` 管理 ROS 无关的连续表达、弧长、分段导数、来源验证与查询。NumPy 和 JAX 使用同一数值函数及区间系数，JAX 编译边界返回固定字段和有效性。归一化弧长反解使用预计算分段弧长与固定次数 Newton，残差进入有效性检查。

位置为 `Phi1(Phi0(scale*R*C(u)/1000+t))`。两次投影的导数均通过单位向量的一至三阶导数计算。工具轴为最终圆柱向内径向，名义旋转保持圆柱轴为局部 `+Y`；源 `+Z` 表达的转换矩阵随元数据保存。弧长积分采用最终位置的一阶导数，源节点划分区间；对 `s` 的导数使用完整反函数链式关系。

## 文件及消费

版本目录保存原控制点/节点和名义规划文件的字节快照、`reference.npz`、`source_samples.npz`、`samples.npz`、`manifest.json`。production 与 S0 各自记录实际参数及身份。manifest 的产物 SHA256 校验在加载时执行；source MAT 身份、变换和圆柱参数在原入口连接时验证。

`PathGeometry` 增加连续参考适配，位置数组继续用于分支搜索和已有形状接口，真实弧长、采样和最终局部投影查询连续表达。JAX 适配同时携带相同连续数组。原来的纯采样路径继续使用既有公开接口。

## 名义时间和几何身份

来源 MAT 的最后两个样本具有相同 `u=1`，保留时间区间和两个索引。查询分别返回名义时间范围，事件记录端点保持。原 `speed_series` 有一个非有限样本，该字段原样保存并带有效性；连续位置与新导数独立计算。工作距离只使用有来源的场景记录，S0 为 50 mm，默认拟合模式的物理工件距离状态为未提供。

## 验证依据

外部事实笔记：`/home/lsn/robot/map147-exploration/butterfly-reference-evidence.md`；实际数学探针和 JSON 同目录。默认最终曲线长度约 `1.8167657348637825 m`；S0 约 `1.8167619852376167 m`。独立 SciPy 积分和原实际加载结果作为验收依据。
