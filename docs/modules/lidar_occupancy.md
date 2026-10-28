# LiDAR 射线占据场景

`portable_oscbf/work/lidar_occupancy.py` 提供无 ROS 的射线占据生产者，输出既有
`CollisionScene`，由真实 `CollisionSafety.prepare_scene` 核对身份、时间、checksum、覆盖及容量。
它属于 [#108](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/108) 的独立场景能力。

## 输入与调用

`OccupancyGrid(origin_m, shape, voxel_size_mm, required_mask=...)` 定义有界立方 voxel 和必要空间。
`required_space_hash` 根据完整几何及必要空间 mask 生成，必须写入对应 `CollisionIdentities`。
voxel 尺寸必须与已加载的 [参数 artifact](portable-oscbf/collision-parameter-artifact.md) 相同。

`LidarRayBatch` 保存 `(N,3)` 的 `origins_m/endpoints_m`、`(N,)` 的逐点
`point_stamp_ns/return_kind`，以及严格递增的 `scan_id`。位置使用 `base_link` 和 m，时间使用
同一机器的 monotonic ns。整数时间和分类不能接受小数或整数溢出。数据在边界复制并检查；
外参转换、逐点时间转换和 mesh 自体过滤由相应上游提供有效结果。

`RayReturn.HIT` 提供环境返回；其终点前的可见射线更新 free，终点提供 occupied。
`NO_RETURN/OCCLUDED` 保留未知含义，不提供清除证据。较近的环境 hit 会保留后方已有
occupied，直到其获得独立清除证据或过期。`AMBIGUOUS` 撤销测量不确定范围相交 voxel 的
覆盖，已有 occupied 和包围范围继续保留；对应空间需要新的有效 hit 或 free 观测解除
ambiguous 标记。同批次 ambiguous 优先于 free。

[逐点 mesh 自体过滤](lidar_self_filter.md) 的批次另外携带可见 free 前缀、unknown 完整区间、
误差半径、过滤身份和输入有效状态。self/occluded 仅在明确可见的前缀更新 free，unknown
区间撤销旧 free 并保留 occupied。无效输入调用 `invalidate`，通过真实 `prepare_scene`
保持不可准入。过滤身份改变后需要新的场景实例；详细字段和调用入口见上述模块说明。

```python
from work.lidar_occupancy import LidarOccupancyScene

producer = LidarOccupancyScene(config, identities, grid)
producer.update(ray_batch, now_ns=arrival_monotonic_ns)
snapshot = producer.snapshot(now_ns=preparation_monotonic_ns)
prepared = collision_safety.prepare_scene(snapshot.scene, snapshot.scene.identities)
```

生产者由一个 owner 顺序调用；snapshot 的数据副本用于读取者。`cells` 保存包含固定障碍的
`UNKNOWN/FREE/OCCUPIED`，逐格最近观测、最近 hit、free 证据时间序列、次数及 ambiguous
范围和来源时间另行保存。场景版本随有效
更新、证据过期和范围变化增加。重复 scan、逆序时间、未来采集时间和过期传输在更新前报告错误。

## 证据与范围

有效 hit 立即产生 occupied。occupied 只在连续、独立 free 证据达到要求后清除；每个 scan
对同一 voxel 最多增加一次，重复时间不增加次数，无证据、再次 hit 或过大间隔重新开始确认。
free 与 occupied 各自过期为 unknown，其他位置的新观测不能延长其有效期。清除 occupied
时，计入确认的每条 free 证据都必须处于当前 `free_max_age_ns` 窗口内，过期证据从时间序列
与计数中一起移除。

以下参数为 artifact 中的必需字段，分别保存单位、来源、适用范围与独立验证记录：

| 字段 | 含义 |
|---|---|
| `occupied_max_age_ns` | occupied 证据有效期 |
| `free_max_age_ns` | free 证据有效期 |
| `free_evidence_required_count` | 清除 occupied 的独立连续次数，至少两次 |
| `free_evidence_min_interval_ns` | 独立观测的最小间隔 |
| `free_evidence_max_gap_ns` | 连续证据允许的最大间隔，必须不大于 free 有效期 |

occupied 包络含 voxel 半对角线、LiDAR 距离/入射误差、外参平移误差、距离乘以角误差范围，
以及每项仍有效观测年龄对应的未跟踪速度和加速度范围。原始逐点时间和误差证据保留；
同格较新的点不能覆盖较旧有效证据的来源时间与较大范围。`source_stamp_ns` 使用仍被采用
证据的最早时间。名义 hit 位于网格外部、其测量不确定范围进入网格时，在相邻边界 voxel
保留 occupied，包围半径同时覆盖完整原始测量不确定球和 voxel 体积。必要空间 coverage
从逐格有效证据计算，存在 unknown 或 ambiguous 必要空间时拒绝准入。

## 固定模型与容量

`FixedEnvironmentModel.from_mesh` 使用真实 `trimesh.Trimesh`，要求封闭正体积、有限坐标、
显式 `base_link/m`、来源 SHA-256、验证说明、误差范围和 `model_revision`。转换以包围完整
mesh 体积的分区盒生成球形 support；其半径覆盖分区盒半对角线与模型误差，包含相应额外障碍空间。
`set_fixed_environment(model)` 将其与 LiDAR occupied 取并集，LiDAR free 不删除固定模型。
模型内容变化必须增加 revision，旧场景与新的期望身份不符时由 `prepare_scene` 拒绝。
`geometry_hash` 保存原 mesh 及误差身份；`model_hash` 覆盖版本、几何身份、support 中心与
半径、需求数量、来源、验证说明和 policy。构造与接收都重新核验完整内容，生产者保存自己的
数据副本；内容与原身份不符的替换直接报告错误。

只有完整包含在固定保守障碍包络中的 voxel 能获得固定 occupied 覆盖；其外部的未知空间
继续等待有效观测。固定几何保留独立的来源、内容 hash 与版本，其范围不增加未跟踪运动误差。

support 数量超过容量时，合并球采用 `rho_mm = max(norm(p_j-c)*1000 + rho_j_mm)`，完整
覆盖全部原球。合并同时遵守成员数量、半径、总 support 容量和准备时间要求。snapshot 保留
原 support、合并成员、最早来源、最严格有效期与最大运动范围。原始需求超限、半径超限或
无法完成证明时返回 `CAPACITY_OVERFLOW`；真实 `prepare_scene` 清除对应准入 mask。

## 验证与接入范围

在项目独立 Python 环境和 JAX x64 配置下执行：

```bash
python -m pytest portable_oscbf/tests/test_lidar_occupancy.py \
  portable_oscbf/tests/test_prepare_scene.py \
  portable_oscbf/tests/test_collision_parameters.py -q \
  --basetemp .scratch/lidar-occupancy-validation/pytest
```

行为测试通过真实公开对象、解析射线、真实封闭 mesh、参数 artifact 和 `prepare_scene`，覆盖
清除、遮挡、过期、固定冲突与范围合并。解析参数仅证明软件行为；设备测量由 #117 完成。
逐点 mesh 自体过滤见 [self-filter 模块](lidar_self_filter.md)。动态 reachable support tube、
场景 server、真实 LiDAR 回放和生产碰撞切换分别由 #101、#104、#122、#123 管理；当前生产控制链
继续保持现有准入条件。
