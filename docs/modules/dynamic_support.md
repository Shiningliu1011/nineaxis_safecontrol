# 动态 support 与未来范围

`portable_oscbf/work/dynamic_support.py`、`lidar_occupancy.py` 与 `fixed_environment.py` 提供
#101 的环境时间模型与固定模型来源。
`CollisionSafety.query` 在每行 `query_time_ns` 同时计算环境中心、范围和范围增长率；毫米距离与
barrier 使用同一结果。`certify` 继续返回 `CERTIFICATE_FAILED`，完整机器人运动与停止过程
证明由 #113 提供。

## 观测、关联与身份

`LidarOccupancyScene.observations()` 返回不可变的 `OccupiedObservation`，包含 occupied
voxel 的 support id、中心、几何半径、测量范围、原采集时间、时间误差及来源 hash。同一 voxel
的各个仍有效采集时间分别保留，时间相同时保留覆盖全部测量的最大范围。测量范围的组成来自
parameter artifact，包括 range、incidence、外参及角误差。`observation_hash(records)`
绑定完整观测内容，空间范围和 policy 另有身份。

`TrackingFrame.from_dict(document)` 读取 JSON 回放对象；`to_dict()` 可由标准 `json` 保存。
schema version 为 1，具有 `scene_epoch/required_space_hash/policy_hash/clock_id/tracks`。
每个 `MotionTrack` 保存：

| 字段 | 含义 |
|---|---|
| `track_id/support_ids/observation_hashes` | 唯一关联与逐 support 原观测身份 |
| `velocity_m_s` | 更新时刻的名义速度 |
| `position_error_mm/velocity_error_mm_s` | 位置及速度估计误差范围 |
| `acceleration_bound_m_s2` | 完整有效区间的加速度上界 |
| `timestamp_error_ns` | track 更新时间的误差范围 |
| `updated_stamp_ns/valid_until_ns/valid` | 更新时间、适用期限及估计有效状态 |
| `evidence` | 不同的 source/validation SHA-256、计算验证方法、device 与 scenario |

`snapshot(now_ns=..., tracking=frame)` 仅采用与 scene、空间、policy、device、scenario 和
clock 一致的关联。track 只允许引用已有 occupied support。关联内容改变、新观测、无效估计、
中断或过期均采用未跟踪范围，occupied 的清除仍由射线证据决定。异源合并使用
`SupportTrackStatus.MIXED`，逐来源 track id 继续保存在时间数据中。未跟踪速度与加速度上界
适用于整个设备与场景范围；track 速度在扣除估计误差后仍超出已验证速度上界时，输入直接拒绝。
tracking 的类型、身份、关联与内容验证在观测过期处理和内部时间、revision 更新以前完成。
拒绝外部请求后，既有有效观测及快照时间保持原状态。

```python
import json
from work.dynamic_support import TrackingFrame, reachable_support_tube

frame = TrackingFrame.from_dict(json.loads(replay_text))
snapshot = producer.snapshot(now_ns=preparation_ns, tracking=frame)
prepared = collision_safety.prepare_scene(snapshot.scene, snapshot.scene.identities)
tube = reachable_support_tube(prepared, interval_start_ns, interval_end_ns, policy=collision_safety.config.policy)
```

当前 scene layout 使用正数 host monotonic ns。其他 clock 与从零开始的共同仿真物理时间
尚需 #154 接口实现，当前入口会拒绝不支持的 clock。场景 epoch、clock hash、policy、完整
source 成员与原观测 hash 均进入 scene checksum。数据经 `prepare_scene` 复制到固定形状的
float64 JAX 数组；成员数量由既有 `support_merge_max_points` 限定，未增加 policy 字段。

## 时间范围与合并

未跟踪 support 使用原观测至目标时间的年龄，加上原采集时间误差，计算
`V*age + A*age^2/2`。V 与 A 来自带来源、适用范围及独立验证的 parameter artifact。
有效 track 更新以前的历史年龄为
`(updated_stamp_ns-point_stamp_ns+point_timestamp_error_ns+track.timestamp_error_ns)*1e-9`，
完整覆盖实际更新时间的误差区间。更新以后单独计算名义位移、位置与速度误差、加速度及
track 时间误差。snapshot 保存当时完整范围，后续仅增加 snapshot 以后的
时间贡献。track 过期后生成新 snapshot 时，从原观测重新计算完整未跟踪年龄。

各来源在 snapshot 时刻 t0 保存 `p_i/rho_i/v_i/r_i/a_i`，其中 r_i 是范围增长率。
未来位置为 `p_i+v_i*dt`，范围为 `rho_i+r_i*dt+a_i*dt^2/2`。合并中心与速度采用成员平均值，
初始半径覆盖全部成员球，增长率至少为每个成员的
`norm(v_i-v_group)*1000+r_i`，加速度采用成员最大值。三角不等式保证完整有效时间区间内
每个成员都处于合并球中。

固定成员来自实际 `FixedEnvironmentModel`。`CollisionScene` 与 `PreparedScene` 的可选
`fixed_environment_model` 保存模型原始 support 几何、内容 hash、来源和验证说明、policy 及
revision；原来的 `work.lidar_occupancy.FixedEnvironmentModel` 导入方式继续可用。
`prepare_scene` 复制并核验模型内容，模型 hash 进入 scene checksum。模型作为不可变 JAX
static PyTree，对象序列化往返会重新执行只读数组和内容校验。

固定来源必须同时符合模型的 policy、revision、成员内容 hash 和原始几何。有效成员使用零
采集时间、零速度和零增长；其余动态来源的原采集时间必须为正数。准入、query/distance 与
直接 tube 均执行相同对应关系检查。动态观测时间修改为零不能提供固定模型来源。
成功场景须保留全部模型成员：每个成员均由有效 source 表示，并由有效 group 完整包围。
旧静态布局仍核验全部模型成员的 group 几何覆盖。容量失败继续返回明确失败状态。

`reachable_support_tube(scene, start_ns, end_ns, policy=...)` 返回区间两端及整个区间的包围球。
线性名义中心与单调二次半径使区间包围可由端点上界计算。`valid_mask` 检查原内容、身份、
成员包围、coverage、物理期限和半径容量。实际 `CollisionPolicy` 为必需的有效性依据，缺少
policy 或身份不一致时全部 mask 无效。它仅提供环境范围，不能代替机器人连续运动证明。

`prepare_scene`、query/distance 与直接 tube 查询共同核验每个动态未跟踪 source：初始半径
至少包含原观测年龄范围与 policy 的 voxel、range、incidence、外参平移误差；增长率至少为
`V+A*age`，加速度至少为 A；来源期限不超过原观测时间加 `occupied_max_age_ns`。
group 的系数另须包围所有 source。track 来源还核对可由原观测时间、更新时间与 scene tracks
确定的历史半径和增长下限。完整 track 误差项由绑定原观测内容的 producer 计算。

上述数组依据由 `SupportMotionBounds.from_policy` 生成。`CollisionSafety` 保存自身配置生成的
固定数组，输入 scene 无法替换其 policy 身份与物理参数。固定模型来源由可选
`fixed_environment_model` 字段保存；`OccupancySnapshot` 与 `CollisionPolicy` 字段结构保持原样。
`evaluate_support_time` 仅执行已定义的时间多项式计算，调用者通过上述入口取得有效性结论。

## 查询与导数

`QueryBatch(q, active_mask, query_time_ns=...)` 的时间数组形状与查询批次数相同，dtype 为
int64。`QueryResult.query_time_ns` 返回各行实际采用的目标时间。时间差在 int64 中相减后
转换为 float64 秒，保留较大参考时间附近的纳秒差别。

带时间模型的查询省略 `query_time_ns` 时使用本次查询开始时间。缺少时间模型的旧 scene
保持已有当前查询行为；显式未来查询返回 `UNKNOWN_REQUIRED_SPACE`。超出模型起止时间或
coverage 期限同样拒绝；未来半径超限返回 `CAPACITY_OVERFLOW`。当前 freshness 与计算
deadline 分别检查。首次 JIT 编译计入查询 deadline，周期任务需要提前预热。

`h=sum((local/radii)^2)-beta^2`，`beta=1+(rho+clearance)/a_min`。
环境时间项包含名义中心速度的解析贡献，以及 `-2*beta*rho_dot/a_min`。
`required_clearance_mm` 与 occupied 范围分别保存；毫米距离不提供命令准入。

## 验证

```bash
python -m pytest portable_oscbf/tests/test_dynamic_support.py \
  portable_oscbf/tests/test_prepare_scene.py \
  portable_oscbf/tests/test_collision_safety.py \
  portable_oscbf/tests/test_ellipsoid_point.py -q \
  --basetemp .scratch/dynamic-support-validation/pytest
```

测试使用真实公开对象、JSON 回放格式、实际 JAX kernel、解析几何与独立公式，覆盖运动、
关联变化、异源合并、原时间范围、ns 边界、checksum 与失效行为。测试参数的适用范围为
解析软件接口；#117 提供设备范围，#122 完成真实 LiDAR 回放验收。生产控制接入由后续任务处理。
