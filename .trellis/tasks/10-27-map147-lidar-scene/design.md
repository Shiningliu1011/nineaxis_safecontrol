# #108 实现设计

## 现状与责任

`static_occupancy.OccupancyTracker` 处理当前点集的持续出现分类。新行为属于带射线证据的占据场景生产者。下游沿用 `_collision_scene.CollisionScene`、`CollisionIdentities` 和 `CollisionSafety.prepare_scene`，数值参数沿用 `CollisionParameterArtifact`。

## 公开接口

- `LidarRayBatch`：`base_link` 中的射线原点、终点、逐点单调采集时间、scan 身份与 hit/no-return/occluded/ambiguous 分类；数据在边界验证并复制保存。
- `OccupancyGrid`：有界 voxel 几何和必要空间 mask，提供固定空间身份。此处 voxel 体积语义与既有 ESDF 插值网格分别定义。
- `FixedEnvironmentModel`：由经过验证的封闭固定 mesh 及来源证据构造版本化保守 support。使用现有 trimesh 数据对象；覆盖完整 mesh 包围体积的规则可独立核验，不自行读取 mesh 文件字节。
- `LidarOccupancyScene.update(batch, now_ns=...)`：一次 scan 按逐点时间更新状态；同批 hit 优先；每个 scan 对同一 voxel 最多增加一次 free 证据。
- `LidarOccupancyScene.snapshot(now_ns=...)`：保存三类 voxel、逐格来源时间、状态年龄、固定模型身份和带 checksum 的固定形状 `CollisionScene`。

## 证据与数值

占据和 free 有各自有效期；连续 free 的最少次数、最小独立间隔及最大连续间隔进入参数 artifact 的必需字段，保留来源和验证记录。运行时不覆盖安全参数。测试参数保持解析接口测试身份，现场数值由 #117 测量。

射线只为命中前的可见区间提供 free 证据，终点及其不确定范围保留 occupied 优先。无返回及 ambiguous 不产生 free，遮挡终点之后维持现有证据直至过期。每次状态或有效范围变化增加 revision。

未跟踪 occupied 使用 artifact 中的速度和加速度上界增加 `rho_mm`；固定几何保持自身误差，不累加动态运动范围。各 support 的来源时间及最严格有效期参与导出。必要空间 coverage 从逐格证据计算，不允许由调用者提供通过标记。

容量合并计算 `rho=max(norm(p_j-c)+rho_j)`，同时限制成员数、合并范围、完整 preparation 预算与最终 radius。独立检查原 support 球均被覆盖。失败保留明确错误状态，不能截断为通过场景。

## 修改范围与兼容性

新增纯 NumPy 场景模块、公开行为测试与模块说明；在现有 parameter schema 和解析接口样例中补充本票使用的安全参数。必要时修改 `prepare_scene` 的错误状态传播，使容量失败继续返回容量错误且有效 mask 为 false。保留现有生产感知链和控制行为；新场景由后续 server/self-filter 任务接入。

所有 Python 代码仅由 apply_patch 编辑。当前没有提交、发布或现场操作授权；集成由主会话组织。规范与 ADR 保持原内容，候选可复用知识记录在任务交接中。
