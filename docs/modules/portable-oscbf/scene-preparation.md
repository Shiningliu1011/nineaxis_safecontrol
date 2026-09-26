# CollisionSafety 场景准入

`CollisionSafety.prepare_scene(scene, identities)` 是场景读取端的 host 操作。发布者提供 `CollisionScene`，读取者提供期望的 `CollisionIdentities`。检查成功后，数据复制到当前 JAX device，返回不可变 `PreparedScene`。准备操作包含 checksum、时钟和数据复制，不作为 JIT 函数；返回值为固定结构 PyTree，支持 JAX 消费。

## 输入与身份

类型从 `work.collision_safety` 导入，内部数据检查由 `_collision_scene.py` 实现。

- `SceneMetadata` 必须明确 `layout_version=1`、`frame_id="base_link"`、`point_unit="m"`、`radius_unit="mm"`、`velocity_unit="m/s"`、`time_unit="ns"` 和 `clock_id="monotonic"`。未知值拒绝准入。
- `CollisionScene.identities` 保存发布者的 scene epoch/revision、geometry/kernel/policy identity、固定模型 revision 和 required-space hash。每项都必须与读取者传入的期望值相同，三个模块身份还必须符合当前 policy。epoch 和 required-space hash 不能全零，revision 不能为负。
- support 数据包括中心、半径、要求间距、名义速度、identity、活动 mask、有效 mask、tracking 状态和 track identity。每个活动 support 的 identity 非负且唯一，clearance 与 artifact 相等，半径不超过 artifact 上限。
- `SceneTracks` 保存固定数量的 track identity、`active_mask`、`valid_mask`、`velocity_m_s`、`acceleration_bound_m_s2`、`error_bound_mm`、`updated_stamp_ns` 和 `valid_until_ns`。误差与加速度范围非负，全部浮点槽位有限。

support 容量来自 artifact 的 `max_support_points`，track 容量来自 `max_tracks`。`SceneMetadata.support_count` 和 `track_count` 表示发布者需要容纳的数量；超过容量返回 `CAPACITY_OVERFLOW`，容量内的数量必须与活动 mask 一致。读取端不裁剪输入，也不执行 support 合并。shape 与 dtype 错误直接抛出异常。

## Checksum 与不可变结果

发布者在填完全部字段后调用 `scene.compute_checksum()`，将返回的 32 字节 SHA-256 写入 `SceneMetadata.checksum`。该方法只负责数据编码与摘要，不生成准入结论。

编码顺序为：除 checksum 外的 metadata 按 key 排序形成紧凑 UTF-8 JSON；随后依 `CollisionScene` 字段顺序遍历数据，嵌套 record 按其字段顺序遍历。每个数组前写入 `[外层字段名, 内层字段名]` 的紧凑 JSON，再用 NumPy `save(..., allow_pickle=False)` 写入 C 顺序 NPY 内容。普通字段的内层名称为空字符串。dtype、shape、未活动槽位、源身份和覆盖期限均参与摘要。

读取端首先复制输入数组，检查副本的 checksum，随后把副本统一转换为 float64 JAX 数据。调用方修改原始 NumPy 数据不会改变返回结果。整数、布尔 mask 与 identity 保留其原有 dtype；失败结果的 support/track 有效 mask 全部为 False。

## 时间、覆盖与 tracking

所有 scene 时间使用同一机器的 `time.monotonic_ns()`。必须满足 `0 < source_stamp_ns <= prepared_stamp_ns <= 当前时间`。采集至发布准备的允许时长为 artifact 的 `transport_delay_ns + scene_preparation_delay_ns`；场景总有效期再加 `control_delay_ns`。读取完成时仍需满足这一期限。

`required_space_covered` 必须为 True，`coverage_space_hash` 必须等于 identity 中的 `required_space_hash`，`coverage_valid_until_ns` 必须覆盖当前时间。即使 support 数量为零，也需要完整覆盖依据；覆盖缺失、空间不符或覆盖过期返回 `UNKNOWN_REQUIRED_SPACE`。覆盖证据的产生由场景服务负责。

TRACKED support 必须关联一个活动、有效且 identity 唯一的 track，support 名义速度须与 track 相同，support 半径须包含 tracking 位置误差。track 更新时间不得晚于场景采集时间，有效期须覆盖当前时间。失效的 TRACKED 输入返回 `INVALID_SCENE`。发布者在 tracking 中断后按占据与运动规则提供 UNTRACKED 场景，保留 occupied；UNTRACKED support 使用 `support_track_ids=-1` 和零名义速度，未跟踪运动上界保存在 parameter artifact。

PreparedScene 保存场景、覆盖和 tracking 三项到期时间；tracking 到期时间取全部活动 track 的最早期限。准备阶段在成功结果全部准备完成并等待 device 后采样完成时间，检查 preparation deadline、device transfer deadline 及三项有效期。

query 与 certify 使用 PreparedScene 时重新检查期限。query 在最终 JAX 验证完成后，用 header 的 `completed_ns` 再次核对三项期限；完成时过期的结果不能得到有效 mask。场景不完整、身份错误、checksum 损坏、非有限值和 tracking 错误返回 `INVALID_SCENE`；准备或 device 复制超过 artifact 的对应 deadline 返回 `DEADLINE_MISSED`。已有非 OK status 保持诊断含义。

## 验证与当前范围

`portable_oscbf/tests/test_prepare_scene.py` 通过公开操作验证每项准入条件、错误状态、不可变性、期限和固定 shape。解析输入的时间预算只用于本机接口与数值测试，不能用于生产准入。

场景服务、共享内存原子发布、support 合并、活动集合证明和动态 reachable support tube 由对应执行票提供。当前接口没有生产命令权限；其余操作说明见 [CollisionSafety 接口](collision-safety.md)。
