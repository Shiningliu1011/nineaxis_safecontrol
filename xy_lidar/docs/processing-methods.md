# LiDAR 处理方法

## 多尺度点云配准

源码：[lidar_registration.py](../reference/tmbs/backend/app/algorithms/lidar_registration.py)、[calibration.py](../reference/tmbs/backend/app/utils/calibration.py)。

入口 `run_targetless_registration` 从 PCD 的 `devId` 或 `deviceId` 属性区分设备，以一台设备的点云作为参考。每台设备至少需要 100 个点。

处理过程：

1. 使用已有外参把各设备点云变换到参考坐标系。
2. 按 `0.20、0.10、0.05 m` 的默认体素尺度分别降采样，并估计表面法向。
3. 在每个尺度执行 GICP 或 point-to-plane ICP，将本次变换作为下一尺度的初值。
4. 使用 `candidate_matrix = delta @ default_matrix` 生成外参候选。
5. 输出各尺度点数、匹配比例 `fitness`、匹配残差 `inlierRmse`，以及相对初值的平移和旋转变化量。

可借鉴之处是逐尺度求解、保留初始外参与增量关系，以及完整输出质量指标。算法需要具有重叠区域的参考点云和可用的初始外参。

源码 `_quality_gate` 的默认条件为：

| 字段 | 条件 | 含义 |
| --- | --- | --- |
| `fitness` | ≥ 0.35 | 匹配比例 |
| `inlierRmse` | ≤ 0.15 m | 匹配残差 |
| `deltaTranslationNorm` | ≤ 0.30 m | 相对初值的平移变化量 |
| `deltaRotationDeg` | ≤ 8° | 相对初值的旋转变化量 |

当前程序生成 `lidar → reference_lidar` 候选。机械臂需要 `lidar → base_link`，因此参考几何必须具有已知的基座坐标。当前项目要求独立验收位置误差不超过 20 mm、角度误差不超过 5°；匹配残差和相对初值变化量不能代替独立验收误差。

## 点云预处理与阶段统计

源码：[preprocess.py](../reference/tmbs/backend/app/algorithms/distance_detection/preprocess.py)。

入口 `preprocess_stage1` 依次执行：

1. 清理 `NaN`、`Inf` 和全零坐标点。
2. 按工作区域保留点云。
3. 按配置的车辆 AABB 删除内部点。
4. 使用 Open3D 进行体素降采样。
5. 根据配置执行统计离群点过滤。

范围裁剪与 AABB 处理安排在体素降采样之前，使范围外的点不会参与保留区域内代表点的位置计算。`Stage1Diagnostics` 记录输入点数、各阶段保留点数及删除原因；任一阶段点数不足时返回空结果和 `passed=False`。

本项目可以借鉴处理顺序和阶段计数，用于检查裁剪范围、参数变化以及点数减少的原因。固定工作区裁剪适合现有点云裁剪实验。移动机械臂的自体过滤需要采集时刻对应的关节状态和表面模型，固定车辆 AABB 的删除规则不能直接用于全臂自身点分类。

统计滤波与体素降采样需要使用真实样本检查最小障碍物的保留情况。碰撞支持点还需要携带覆盖被合并空间及误差的半径；删除点、空结果和点数不足都不能自动生成 `free` 证据。

## Livox tag 分组处理

源码：[livox_tag_filter.py](../reference/tmbs/backend/app/drivers/livox_tag_filter.py)。

`livox_tag_keep_mask` 根据配置生成保留掩码，原始 `tag` 可以继续随点保存。该源码采用以下分组：

| 配置 | 掩码 | 源码中的分组说明 | 默认值 |
| --- | --- | --- | --- |
| `dropGlue` | `0x03` | 相邻物体之间的异常回波 | `true` |
| `dropRainFogDust` | `0x0C` | 雨、雾、尘相关回波 | `false` |
| `dropOther` | `0x30` | 其他探测异常 | `false` |

`bit6..7` 保留原值，不参与这些选项的筛选。上述含义来自所附源码，尚未在本次整理中针对现场 MID-360S 固件重新验证。

本项目的连续流默认保留全部点及 `tag`。可以借鉴可配置的分组和统计方法，在消费端记录各组数量及空间分布，并用真实目标检查处理效果。低质量或无法判定的观测需要保留不确定性；只删除异常回波不能证明该方向具有有效覆盖。

## 坐标变换与矩阵校验

源码：[rigid_transform.py](../reference/tmbs/backend/app/algorithms/distance_detection/rigid_transform.py)、[calibration.py](../reference/tmbs/backend/app/utils/calibration.py)。

`transform_lidar_to_vehicle` 明确处理 `lidar → reference_lidar → vehicle`，组合顺序为：

```text
T_lv = T_rv @ T_lr
```

`apply_rigid_transform` 对按行保存的点执行 `points @ rotation.T + translation`。矩阵校验检查形状、有限值、最后一行、旋转正交性及行列式接近 `+1`。

`calibration.py` 的 Euler 表示使用角度，旋转顺序为 `Rz(rz) @ Ry(ry) @ Rx(rx)`。把该表示接入使用弧度的接口时需要明确转换。

可借鉴之处是把输入坐标系、输出坐标系、组合顺序及矩阵检查放在明确的函数边界。当前机械臂基坐标系为固定、Y 向上的 `base_link`，需要使用经过验收的 `B_T_L`。更改坐标名称不能建立传感器到基座的真实空间关系。

## 异步结果有效性与顺序检查

源码：[TaskResultValidator.cs](../reference/drilllidardrv/src/Detection/TaskResultValidator.cs)、[DistanceDetectionController.cs](../reference/drilllidardrv/src/Detection/DistanceDetectionController.cs)、[LidarModels.cs](../reference/drilllidardrv/src/LidarModels.cs)。

`TaskResultValidator.Validate` 检查应用身份、当前任务身份、`stateVersion` 与轮次中的版本是否一致，并校验 `valid / invalid / aborted` 状态。`DistanceDetectionController.ApplyCycle` 按 `sequence` 拒绝重复或较早轮次，随后检查结果 schema、距离状态和几何结果。

可借鉴的处理规则：

- 将任务身份和结果序号一起检查，防止旧任务的数据进入当前处理。
- 为每次结果明确表示有效、无效或终止。
- 更新多个相关数值时明确标记结果有效状态。
- 记录观测时间与有效期，防止长时间没有新结果时继续使用历史数据。

所附 C# 实现在无效轮次保留距离寄存器的旧数值，并将功能有效状态清零。借鉴时应保留数值与有效性的关联检查；`sequence` 只能说明处理顺序，单独使用它无法证明数据仍然有效。
