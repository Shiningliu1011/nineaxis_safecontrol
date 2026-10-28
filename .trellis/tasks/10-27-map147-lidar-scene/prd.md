# LiDAR 射线占据场景与固定模型并集

来源为 GitHub #108、已确认的 #149，以及 ADR 0006、0014–0018、0028、0045。当前项目研究机构为现有九轴机械臂。

## 行为要求

- 时间、坐标和来源有效的 LiDAR hit 立即产生 occupied。
- 每个 voxel 保存来源时间及连续 free 证据。只有独立、连续、时间有效的 free 射线才能清除 occupied；单次 free、重复观测、无返回和遮挡均不能清除。
- 陈旧 occupied 转为 unknown；free 同样具有有效期。新帧不能延长未观测空间的证据。
- occupied 输出包含 voxel 半对角线、测量误差、外参误差与观测年龄运动范围的 `support_point + rho_mm`。
- 经过验证的固定模型转换为版本化 support，与 LiDAR occupied 取并集。LiDAR free 不能删除固定模型；修改模型身份使旧场景身份失效。
- 超过固定 support 容量时，仅允许完整覆盖所有来源范围的合并；范围、来源、有效期或预算无法证明时返回 `CAPACITY_OVERFLOW`。
- 输出沿用 `CollisionScene` 的身份、单位、时间、checksum 与固定形状，由真实 `CollisionSafety.prepare_scene` 复核。未知必要空间不得取得准入。

## 验收

公开接口行为测试使用真实射线记录、真实参数 artifact、真实固定几何与真实 `prepare_scene`，覆盖 hit、连续 free、重复及中断、无返回、遮挡、陈旧 occupied、unknown、固定模型冲突、模型变更和保守合并及拒绝。测试预期来自解析几何、需求和独立覆盖检查。

输入错误在来源报告；没有 ROS graph、设备动作或真实 LiDAR 回放要求。本票交付纯场景能力，现场标定、自体过滤、tracking、场景服务和完整闭环由对应任务验收。
