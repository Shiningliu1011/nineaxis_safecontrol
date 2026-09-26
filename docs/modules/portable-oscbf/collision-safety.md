# CollisionSafety 接口

`portable_oscbf/work/collision_safety.py` 提供无 ROS 的 `CollisionSafety`。当前接口供后续几何、场景、规划和控制模块共同接入，生产控制器尚未使用它。规格来源为 [CollisionSafety 统一碰撞安全模块规格说明书](../../planning/oscbf-reuse/spec-collision-safety-module-20260921.md)。

## 公开操作

- `prepare_scene(scene, identities)` 在 host 核验完整场景副本，检查 checksum、frame、单位、固定 shape、身份、时间、覆盖、容量和 tracking。浮点输入可用 `float32` 或 `float64`；返回的不可变 `PreparedScene` 中，参与计算的数组统一为 `float64`。完整字段与失败规则见 [场景准入](scene-preparation.md)。
- `query(query_batch, prepared_scene, query_mode)` 接受 `QueryMode.STATE_VALIDITY`、`QueryMode.OSCBF_BARRIER` 或 `QueryMode.DISTANCE_MM`。九轴状态批次形状由 `CollisionSafetyConfig.query_batch_size` 固定。
- `certify(segment_batch, prepared_scene)` 接受固定数量的九轴区间及起止时间。

`CollisionSafetyConfig.from_policy(policy)` 从已验证的 [CollisionParameterArtifact](collision-parameter-artifact.md) 读取 support、查询批次、primitive 行和区间批次容量，以及查询与证明的 deadline。配置必须绑定 policy，所有值都须与 artifact 一致。模块的 `config` 只读，artifact 与 policy 均深层不可变。

配置包含 32 字节的 geometry、kernel 和 policy identity。`CollisionIdentities` 还包含 16 字节的 `scene_epoch`、整数 `scene_revision`、整数 `fixed_environment_revision` 和 32 字节的 `required_space_hash`。hash 与 epoch 在 JAX 边界使用 `uint8` 数组，revision 使用 `int64` 标量。场景采集、准备和有效期使用同一机器的 `time.monotonic_ns()`，运行时间使用进程内单调计时，单位均为纳秒。

三个操作都核对当前 identity；已经准备的场景也必须符合当前 policy。有效 support 的 environment clearance 必须等于 artifact 中的数值，半径不能超过其上限。检查失败返回 `INVALID_SCENE` 并保持结果无效，超时仍由 `DEADLINE_MISSED` 表达。

## 固定结果

`QueryResult` 与 `SegmentCertificateBatch` 都包含 `ResultHeader`：status、scene epoch/revision、geometry/kernel/policy identity、固定模型 revision、required-space hash、场景采集时间 `source_stamp_ns`、查询开始时间、完成时间、运行时间、deadline 和超时标志。成功与失败结果均携带输入场景的采集时间。它们采用 JAX PyTree 可识别的 `NamedTuple` 结构。

`QueryResult` 的 OSCBF 字段包括 `valid_mask`、`barrier`、`grad_h_q`、`partial_h_partial_t`、`proximity_scale`、primitive pair identity、solver residual、iteration 和 health。状态有效性与毫米距离各有独立的有效 mask。`SegmentCertificateBatch` 包含区间有效 mask、证明结果、下界、二分深度和失败区间。所有未测量字段的有效 mask 均为 `False`；数值槽位不能单独说明安全。

`query` 支持原生 JAX ellipsoid–ellipsoid self DCOL、environment point-scale 与独立毫米距离。`certify` 仍返回 `CERTIFICATE_FAILED`。场景已有的非 `OK` 状态会进入结果 header；超过 deadline 时返回 `DEADLINE_MISSED`。输入 shape、dtype、配置或 `query_mode` 不合法时，公开方法直接抛出异常。

## Self DCOL

构造时使用 `CollisionSafety(config, geometry_artifact=document)` 绑定已加载的 [geometry artifact](collision-geometry-artifact.md)。构造函数验证内容 SHA-256 与 policy 的 geometry identity、运动学 link 名称、完整候选集合、允许接触记录、槽位容量及每个 link pair 的 clearance。所有 link 必须具有有效 ellipsoid；半轴必须有限且大于零。资产的 mesh 覆盖证明由离线生成器 `--verify` 核对。解析数值验收也可提供相同几何字段，并明确标识解析数据来源。

构造时把有效 self primitive pair 编成固定行，行数不能超过 artifact 的 `max_primitive_rows`。primitive identity 为 `link_index * slot_capacity + slot_index`，其中 `link_index` 是资产的 link 顺序。未使用的结果行和未启用的批次状态保持无效 mask。几何对象转换为固定 JAX arrays，调用方随后修改原始 document 不影响已构造模块。

计算使用原生 ellipsoid scaling functions，求解最小共同线性缩放量。大于 1 表示分离，等于 1 表示接触，小于 1 表示相交。求解器将两项凸二次约束转换为一维凹对偶问题，使用有界二分求解；梯度由最优值导数经共享九轴 POE 运动学传播。每个 primitive pair 独立返回 `proximity_scale`、`grad_h_q` 和 barrier。

Interface 将 `clearance_mm` 乘以 `1e-3`，再计算
`scale_margin = 1 + clearance_m / (a_min_i + a_min_j)`，
`barrier = proximity_scale - scale_margin`。半轴之和及换算结果必须有限，构造时拒绝发生数值溢出的配置。self 几何没有环境运动时间项，`partial_h_partial_t` 为零。

`solver_primal_residual` 是 witness 超过原始 ellipsoid scaling inequality 的最大量；`solver_dual_residual` 汇集归一化 stationarity、dual 权重归一性、双方 scaling 一致性、complementarity 和线性方程 residual。两个 residual 都采用无量纲值，阈值和最大二分次数来自 parameter artifact。`solver_iteration` 记录初始中点之后的二分次数。只有全部数值与梯度有限、两个 residual 均在限值内且梯度有定义时，`solver_healthy` 才有效。中心重合时 scale 为零，线性 scale 梯度不唯一，该 pair 返回无效 health。

在 scene 为 `OK`、全部活动 self/environment pair 健康、容量充足且查询未超时时，`STATE_VALIDITY` 与 `OSCBF_BARRIER` 返回 `OK`；`state_valid` 另外要求所有 barrier 非负。缺少 geometry 或任一活动 pair 未收敛时返回非 `OK`。查询测量包含等待 device 计算完成的时间；首次 JIT 编译也计入 deadline，调用方应在进入周期任务前完成预热。

数值测试通过三个公开操作调用真实求解器，涵盖解析球体、轴向 ellipsoid、80 位独立 KKT 参考、九轴差分梯度、相反危险方向、生产资产全部 45 个 pair、迭代上限与拒绝路径。安装高精度测试依赖使用 `portable_oscbf/requirements-collision-test.txt`。

官方 Julia DCOL 参考数据位于 `portable_oscbf/tests/reference/ellipsoid_dcol/official.json`，保存上游 revision、Julia 版本、求解容差、输入 identity 和参考程序 SHA-256。该文件由同目录的 `generate.py` 调用原版 `DifferentiableCollisions.proximity` 生成；普通 Python 测试直接核对保存的结果。重新生成时指定 Julia 和已完成 `Pkg.instantiate()` 的上游 checkout：

```bash
python3 portable_oscbf/tests/reference/ellipsoid_dcol/generate.py --julia <julia-path> --project <DifferentiableCollisions-checkout> --work-dir .scratch/i21/reference/corpus --output portable_oscbf/tests/reference/ellipsoid_dcol/official.json
```

本地数值证据不授予生产命令权限；目标设备 deadline 继续由独立验收处理。

## Environment point-scale

support 的中心坐标使用米；`support_radii_mm` 表示 occupied 包络半径 `rho_mm`，`required_clearance_mm` 单独保存。Interface 集中转换为米。对局部坐标 `u = R.T @ (support - center)`，计算 `point_scale_sq = sum((u / radii)^2)` 和
`barrier = point_scale_sq - (1 + (rho_m + required_clearance_m) / min(radii))^2`。
`proximity_scale` 返回线性 point scale。`grad_h_q` 由共享九轴运动学求导；`partial_h_partial_t` 包含输入 support 名义速度的解析贡献。

每个有效 ellipsoid 与每个有效 support 都保留独立行。self 行之后按 geometry slot、support 输入位置排列环境行。`primitive_kind` 用 `PrimitiveKind.SELF` 与 `PrimitiveKind.ENVIRONMENT` 区分；环境 `primitive_pair_id` 保存机器人 primitive id 与 support id。未使用的 kind 为 `-1`，对应 mask 无效。活动 support id 必须非负且唯一。

当前完整保留环境 pair，任何 pair 均没有省略证明。有效 support 数量超过 `max_environment_pairs_per_ellipsoid`，或 self 与 environment 总行数超过 `max_primitive_rows` 时，返回 `CONSTRAINT_OVERFLOW` 并清除全部准入 mask。后续活动集合、动态误差和连续证明按执行地图接入；本次数值查询不连接生产命令。

## 独立毫米距离

`QueryMode.DISTANCE_MM` 对全部有效 ellipsoid–support pair 求有符号欧氏距离，返回其中最小值。距离使用 ellipsoid 到 support 中心的有符号距离减去 `rho_m`；要求间距单独用于 `remaining_margin_mm = distance_mm - required_clearance_mm`。正值表示分离，零表示接触，负值表示所选保守 pair 相交。组合几何相交时输出最小 pair 的有符号距离。

内部最近点求解采用单调 secular equation，处理内部点、中心、轴上点及重复最短半轴；几何依据见 [Eberly 的点到 ellipsoid 距离推导](https://geometrictools.com/Documentation/DistancePointEllipseEllipsoid.pdf)。迭代次数采用 artifact 的 `distance_max_iterations`，根区间 witness 误差界限采用 `distance_tolerance_mm`。超出次数仍不能满足误差界限时返回 `SOLVER_UNHEALTHY`。

`nearest_pair_id` 保存 robot primitive id 与 support id；`nearest_link_index` 是 geometry artifact 的 link 顺序，`nearest_ellipsoid_slot` 是该 link 的槽位。`nearest_robot_point_m` 与 `nearest_support_point_m` 使用机器人规范坐标系和米，分别位于 ellipsoid 表面与 support sphere 表面。距离相等时按固定 primitive/support 顺序选择。

`CollisionScene.support_track_status` 与 `PreparedScene.support_track_status` 为固定长度 `int32` 数组，取值 `SupportTrackStatus.UNTRACKED` 或 `SupportTrackStatus.TRACKED`。距离结果的 `track_status` 直接携带所选 support 的状态，独立于速度数值。

距离模式只启用 `distance_valid_mask`，`valid_mask`、`state_valid_mask`、`state_valid` 和 solver/barrier 命令字段保持无效。空 support 场景返回 `OK` 与无效距离 mask，数值槽位不表示已测量间距。距离模式不受 barrier 行容量限制。任一活动距离 pair 无效、scene 不健康或查询超时，距离 mask 同样无效。
