# CollisionSafety 接口

`portable_oscbf/work/collision_safety.py` 提供无 ROS 的 `CollisionSafety`。当前接口供后续几何、场景、规划和控制模块共同接入，生产控制器尚未使用它。规格来源为 [CollisionSafety 统一碰撞安全模块规格说明书](../../planning/oscbf-reuse/spec-collision-safety-module-20260921.md)。

## 公开操作

- `prepare_scene(scene, identities)` 检查固定 shape、数据类型、有限值、时间顺序和身份。原始 support 坐标可用 `float32`；返回的 `PreparedScene` 中，参与计算的数组统一为 `float64`。
- `query(query_batch, prepared_scene, query_mode)` 接受 `QueryMode.STATE_VALIDITY`、`QueryMode.OSCBF_BARRIER` 或 `QueryMode.DISTANCE_MM`。九轴状态批次形状由 `CollisionSafetyConfig.query_batch_size` 固定。
- `certify(segment_batch, prepared_scene)` 接受固定数量的九轴区间及起止时间。

`CollisionSafetyConfig.from_policy(policy)` 从已验证的 [CollisionParameterArtifact](collision-parameter-artifact.md) 读取 support、查询批次、primitive 行和区间批次容量，以及查询与证明的 deadline。配置必须绑定 policy，所有值都须与 artifact 一致。模块的 `config` 只读，artifact 与 policy 均深层不可变。

配置包含 32 字节的 geometry、kernel 和 policy identity。`CollisionIdentities` 还包含 16 字节的 `scene_epoch` 和整数 `scene_revision`。身份在 JAX 边界使用 `uint8` 数组，所有运行时间字段使用同一进程的单调时钟，单位为纳秒。场景采集时间与准备时间必须由同一时间基准产生。

三个操作都核对当前 identity；已经准备的场景也必须符合当前 policy。有效 support 的 environment clearance 必须等于 artifact 中的数值，半径不能超过其上限。检查失败返回 `INVALID_SCENE` 并保持结果无效，超时仍由 `DEADLINE_MISSED` 表达。

## 固定结果

`QueryResult` 与 `SegmentCertificateBatch` 都包含 `ResultHeader`：status、scene epoch/revision、geometry/kernel/policy identity、开始时间、完成时间、运行时间、deadline 和超时标志。它们采用 JAX PyTree 可识别的 `NamedTuple` 结构。

`QueryResult` 的 OSCBF 字段包括 `valid_mask`、`barrier`、`grad_h_q`、`partial_h_partial_t`、`proximity_scale`、primitive pair identity、solver residual、iteration 和 health。状态有效性与毫米距离各有独立的有效 mask。`SegmentCertificateBatch` 包含区间有效 mask、证明结果、下界、二分深度和失败区间。所有未测量字段的有效 mask 均为 `False`；数值槽位不能单独说明安全。

`query` 已支持原生 JAX ellipsoid–ellipsoid self DCOL。`certify` 仍返回 `CERTIFICATE_FAILED`，环境 point-scale 与毫米距离等待对应能力接入。场景已有的非 `OK` 状态会进入结果 header；超过 deadline 时返回 `DEADLINE_MISSED`。输入 shape、dtype、配置或 `query_mode` 不合法时，公开方法直接抛出异常。

## Self DCOL

构造时使用 `CollisionSafety(config, geometry_artifact=document)` 绑定已加载的 [geometry artifact](collision-geometry-artifact.md)。构造函数验证内容 SHA-256 与 policy 的 geometry identity、运动学 link 名称、完整候选集合、允许接触记录、槽位容量及每个 link pair 的 clearance。所有 link 必须具有有效 ellipsoid；半轴必须有限且大于零。资产的 mesh 覆盖证明由离线生成器 `--verify` 核对。解析数值验收也可提供相同几何字段，并明确标识解析数据来源。

构造时把有效 self primitive pair 编成固定行，行数不能超过 artifact 的 `max_primitive_rows`。primitive identity 为 `link_index * slot_capacity + slot_index`，其中 `link_index` 是资产的 link 顺序。未使用的结果行和未启用的批次状态保持无效 mask。几何对象转换为固定 JAX arrays，调用方随后修改原始 document 不影响已构造模块。

计算使用原生 ellipsoid scaling functions，求解最小共同线性缩放量。大于 1 表示分离，等于 1 表示接触，小于 1 表示相交。求解器将两项凸二次约束转换为一维凹对偶问题，使用有界二分求解；梯度由最优值导数经共享九轴 POE 运动学传播。每个 primitive pair 独立返回 `proximity_scale`、`grad_h_q` 和 barrier。

Interface 将 `clearance_mm` 乘以 `1e-3`，再计算
`scale_margin = 1 + clearance_m / (a_min_i + a_min_j)`，
`barrier = proximity_scale - scale_margin`。半轴之和及换算结果必须有限，构造时拒绝发生数值溢出的配置。self 几何没有环境运动时间项，`partial_h_partial_t` 为零。

`solver_primal_residual` 是 witness 超过原始 ellipsoid scaling inequality 的最大量；`solver_dual_residual` 汇集归一化 stationarity、dual 权重归一性、双方 scaling 一致性、complementarity 和线性方程 residual。两个 residual 都采用无量纲值，阈值和最大二分次数来自 parameter artifact。`solver_iteration` 记录初始中点之后的二分次数。只有全部数值与梯度有限、两个 residual 均在限值内且梯度有定义时，`solver_healthy` 才有效。中心重合时 scale 为零，线性 scale 梯度不唯一，该 pair 返回无效 health。

在 scene 为 `OK`、没有有效 environment support、全部活动 self pair 健康且查询未超时时，`STATE_VALIDITY` 与 `OSCBF_BARRIER` 返回 `OK`；`state_valid` 另外要求所有 self barrier 非负。带有有效 environment support 的查询当前提供 self 数值诊断并返回非 `OK`，全部准入 mask 无效。缺少 geometry、请求尚未实现的距离模式或任一活动 pair 未收敛时，同样返回非 `OK`。查询测量包含等待 device 计算完成的时间；首次 JIT 编译也计入 deadline，调用方应在进入周期任务前完成预热。

数值测试通过三个公开操作调用真实求解器，涵盖解析球体、轴向 ellipsoid、80 位独立 KKT 参考、九轴差分梯度、相反危险方向、生产资产全部 45 个 pair、迭代上限与拒绝路径。安装高精度测试依赖使用 `portable_oscbf/requirements-collision-test.txt`。

官方 Julia DCOL 参考数据位于 `portable_oscbf/tests/reference/ellipsoid_dcol/official.json`，保存上游 revision、Julia 版本、求解容差、输入 identity 和参考程序 SHA-256。该文件由同目录的 `generate.py` 调用原版 `DifferentiableCollisions.proximity` 生成；普通 Python 测试直接核对保存的结果。重新生成时指定 Julia 和已完成 `Pkg.instantiate()` 的上游 checkout：

```bash
python3 portable_oscbf/tests/reference/ellipsoid_dcol/generate.py --julia <julia-path> --project <DifferentiableCollisions-checkout> --work-dir .scratch/i21/reference/corpus --output portable_oscbf/tests/reference/ellipsoid_dcol/official.json
```

本地数值证据不授予生产命令权限；目标设备 deadline 继续由独立验收处理。
