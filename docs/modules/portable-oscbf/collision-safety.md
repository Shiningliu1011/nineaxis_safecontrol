# CollisionSafety 接口

`portable_oscbf/work/collision_safety.py` 提供无 ROS 的 `CollisionSafety`。当前接口供后续几何、场景、规划和控制模块共同接入，生产控制器尚未使用它。规格来源为 [CollisionSafety 统一碰撞安全模块规格说明书](../../planning/oscbf-reuse/spec-collision-safety-module-20260921.md)。

## 公开操作

- `prepare_scene(scene, identities)` 检查固定 shape、数据类型、有限值、时间顺序和身份。原始 support 坐标可用 `float32`；返回的 `PreparedScene` 中，参与计算的数组统一为 `float64`。
- `query(query_batch, prepared_scene, query_mode)` 接受 `QueryMode.STATE_VALIDITY`、`QueryMode.OSCBF_BARRIER` 或 `QueryMode.DISTANCE_MM`。九轴状态批次形状由 `CollisionSafetyConfig.query_batch_size` 固定。
- `certify(segment_batch, prepared_scene)` 接受固定数量的九轴区间及起止时间。

`CollisionSafetyConfig` 要求明确给出 support、查询批次、primitive 行和区间批次容量，以及查询与证明的 deadline。配置包含 32 字节的 geometry、kernel 和 policy identity。`CollisionIdentities` 还包含 16 字节的 `scene_epoch` 和整数 `scene_revision`。身份在 JAX 边界使用 `uint8` 数组，所有运行时间字段使用同一进程的单调时钟，单位为纳秒。场景采集时间与准备时间必须由同一时间基准产生。

## 固定结果

`QueryResult` 与 `SegmentCertificateBatch` 都包含 `ResultHeader`：status、scene epoch/revision、geometry/kernel/policy identity、开始时间、完成时间、运行时间、deadline 和超时标志。它们采用 JAX PyTree 可识别的 `NamedTuple` 结构。

`QueryResult` 的 OSCBF 字段包括 `valid_mask`、`barrier`、`grad_h_q`、`partial_h_partial_t`、`proximity_scale`、primitive pair identity、solver residual、iteration 和 health。状态有效性与毫米距离各有独立的有效 mask。`SegmentCertificateBatch` 包含区间有效 mask、证明结果、下界、二分深度和失败区间。所有未测量字段的有效 mask 均为 `False`；数值槽位不能单独说明安全。

当前的 `query` 对有效场景返回 `SOLVER_UNHEALTHY`，`certify` 返回 `CERTIFICATE_FAILED`。场景已有的非 `OK` 状态会进入结果 header；超过 deadline 时返回 `DEADLINE_MISSED`。碰撞 kernel 与连续区间证明由统一执行地图中的后续票实现，当前接口不会生成碰撞命令或轨迹准入。输入 shape、dtype、配置或 `query_mode` 不合法时，公开方法直接抛出异常。
