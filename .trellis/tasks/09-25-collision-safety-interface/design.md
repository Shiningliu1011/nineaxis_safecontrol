# CollisionSafety 接口设计

## 范围与所有权

- 新接口放在 `portable_oscbf/work/collision_safety.py`，由 `CollisionSafety` 持有三个公开操作和所有结果类型；`portable_oscbf/tests/test_collision_safety.py` 验证公开行为。
- 当前控制器不调用新接口。后续几何、场景、kernel、规划及控制票通过这三个操作接入。本票不生成碰撞安全的 `OK` 查询结果或通过证书。
- `docs/modules/portable_oscbf.md` 增加主题入口；`docs/modules/portable-oscbf/collision-safety.md` 说明当前接口、输入、结果和命令权限。

## 输入和身份

- `CollisionSafetyConfig` 固定 support、查询批次、primitive 行及区间批次容量和 deadline；构造时校验正数、身份长度及 JAX x64 能力。容量由调用方提供，当前不内置未经测量的安全数值。
- `CollisionScene` 使用固定 support 槽位：坐标、半径、要求间距、速度、support identity、活动 mask、采集与准备时间及场景状态。允许原始坐标为 `float32`，`prepare_scene` 转成 `float64`。所有数值槽位及 mask 都有固定 shape。
- `CollisionIdentities` 使用固定长度字节数组承载 scene epoch、geometry hash、kernel version 与 collision policy hash，并使用整数 scene revision。配置中的三个固定身份与传入值相符才可形成有效 `PreparedScene`。
- `QueryBatch` 与 `SegmentBatch` 使用九轴状态、固定批次和活动 mask；区间输入包含起止时间。JAX 计算数值以 `float64` 表示。

## 结果与状态

- `PreparedScene` 与结果采用 `NamedTuple`，数组由 JAX 持有且形状固定。`ResultHeader` 包含全部 identity、status、开始/完成时间、运行时间与 deadline 状态。
- `QueryResult` 固定包含状态有效性、OSCBF primitive 行及毫米距离字段；每组数值都配有独立有效 mask。`SegmentCertificateBatch` 固定包含区间证明 mask、下界、二分深度和失败区间。
- 输入形状、dtype、配置及 `query_mode` 的程序错误在数值计算前抛出 `ValueError` 或 `TypeError`。场景内容、身份、来源时间及非有限值使 `PreparedScene` 返回 `INVALID_SCENE` 或来源明确的非 `OK` 状态。
- `query` 在 scene 为 `OK` 且碰撞 kernel 未接入时返回 `SOLVER_UNHEALTHY`；`certify` 返回 `CERTIFICATE_FAILED`。两者对已有场景失败状态直接传播；deadline 超限返回 `DEADLINE_MISSED`。所有尚未计算的有效 mask 为 `False`。

## JAX 与计时边界

- `prepare_scene` 的固定形状转换和状态判定支持 JAX 编译；结果类型可以穿过 `jax.jit`。`query`、`certify` 的公开方法在 host 执行调用校验、同步和单调时钟测量，其数值部分以后由私有 JAX kernel 编译执行。当前非 `OK` 结果无需运行碰撞 kernel。
- 计时使用同一进程的 `time.perf_counter_ns()`；结果记录完整经过时间，并在 deadline 超限时覆盖状态。未测量或未计算的数值不能因默认零值获得有效性。

## 兼容与验证

- 新文件没有 ROS import，也不改变生产入口或安全门。以后接入真实 kernel 时沿用输入和结果类型，保留身份、有效 mask 与 deadline 判定。
- 验证覆盖固定结构、JAX PyTree、无 ROS 导入、`float32` 到 `float64`、身份、非法输入、全部状态与非 `OK` 准入限制。
