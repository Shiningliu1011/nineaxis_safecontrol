# 与 OSCBF 项目的关系

以下状态依据 2026-10-02 检查的项目代码与配置记录。方法说明见[处理方法](processing-methods.md)。

## 现有入口

| 项目入口 | 已有内容 | 参考资料的用途 |
| --- | --- | --- |
| [livox_mid360](../../src/robot_safecontrol_moveit/livox_mid360/README.md) | 已适配 TMBS 协议、设备控制及连续点云接收；发布包含 `tag` 和逐点时间的 `PointCloud2` | 继续核对设备字段和分组诊断 |
| [lidar_crop_experiment.py](../../src/robot_safecontrol_moveit/lidar_crop_experiment.py) | 点云裁剪实验及诊断发布 | 借鉴阶段计数和过滤效果检查 |
| [perception_bridge.py](../../src/robot_safecontrol_moveit/perception_bridge.py) | 读取标定记录、坐标转换、工作区裁剪与融合 | 接入时明确保留采集时间、原点及观测有效性 |
| [sensor_extrinsics.yaml](../../config/sensor_extrinsics.yaml) | 运行时外参记录；LiDAR 当前为 `calibrated: false` | 配准候选通过独立验收后使用现有 schema 保存 |
| [_collision_scene.py](../../portable_oscbf/work/_collision_scene.py) | `CollisionScene` 与场景准备、有效性检查 | 生成符合其字段、时间、身份及覆盖要求的环境场景 |

现有驱动来源与既有验证记录见[驱动复用记录](../../docs/planning/oscbf-reuse/research/tmbs-mid360-python-driver-reuse.md)。这些历史记录不作为本次运行验证结果。

## 配准结果的使用条件

配准工具应接收具有已知基座参考的几何数据，输出方向明确的 `B_T_L`。求解集和独立验收集分别记录，验收应覆盖重复性、已知尺寸物体及其基座位置。

当前 T1 的验收要求包括位置误差不超过 20 mm、角度误差不超过 5°，以及已知尺寸物体的点云位置误差不超过 20 mm。完整条件见[现场准备清单](../../docs/planning/t1-lidar-calibration/field-checklist.md)。写入外参时保留真实设备身份、方法、时间、误差及 `calibration_id`；部署时核对加载路径和文件 SHA-256。

## 连续观测的数据要求

所附 TMBS [mid360_driver.py](../reference/tmbs/backend/app/drivers/mid360_driver.py) 的 `_parse_pcl_packet` 读取包时间字段，但返回的点数组为 `x/y/z/reflectivity/tag`。批次缓冲增加 `devId`、`batch` 后形成七列数组，没有逐点时间列。[CaptureParameters](../reference/tmbs/backend/app/algorithms/distance_detection/parameters.py) 默认采集时长为 0.5 s。

本项目 [stream.py](../../src/robot_safecontrol_moveit/livox_mid360/stream.py) 保留逐点时间；其真实时间基准仍需要结合现场同步状态确认。现有 `perception_bridge._sensor_callback` 提取 `xyz` 并使用消息头时间，因此后续接入自体过滤和射线场景生产时需要明确传递以下内容：

- 传感器身份、坐标系、外参版本。
- 点的采集时间、时间基准及有效期。
- 原始回波有效性、`tag` 和传感器原点。
- 采集时刻对应的完整九轴关节状态。
- 空间误差与无法可靠分类的观测。

接收时间、采集时间和处理完成时间需要分别说明用途。处理完成后更新消息时间不能延长原观测的有效期。

## 局部场景与碰撞输入

项目的[障碍观测定义](../../docs/domain/observation.md)区分 `occupied`、`free` 和 `unknown`。有用的点云处理方法接入此处时，需要保持以下规则：

1. 原始有效射线提供局部空间的占据与空闲证据；显示裁剪不应删除必要的射线证据。
2. 自体过滤使用点采集时刻的关节状态及机器人表面模型，并保留遮挡和无法明确分类的区域。
3. 体素合并后，代表点及其半径需要覆盖被合并的占据空间和测量误差。
4. 容量不足、信息过期或必要空间缺少有效观测时，场景需要报告相应限制。

`CollisionScene` 包含 `support_points_m`、`support_radii_mm`、`source_stamp_ns`、`prepared_stamp_ns`、场景身份，以及 `required_space_covered` 和覆盖有效期。仅有点云或三个方向的距离无法完整填写这些信息。相关设计与当前实现核对见[机械臂附近碰撞感知记录](../../docs/planning/oscbf-reuse/research/robot-centered-lidar-local-scene-20260927.md)。

## 后续适配的验证依据

- 标定：已知基座参考、独立验收集、重复测量和部署身份记录。
- 点云处理：真实样本中小目标、邻近连杆目标及不同 `tag` 回波的保留情况。
- 时间：采集时刻与关节状态的对应关系、观测到控制使用的延迟。
- 场景：遮挡、目标离开、无返回、数据过期和容量不足时的状态。
- 异步输入：重复结果、较早结果、无效轮次及持续没有新结果时的处理。

本次完成资料整理与静态接口核对。上述算法效果、现场测量和控制接入均需要对应的独立验证。
