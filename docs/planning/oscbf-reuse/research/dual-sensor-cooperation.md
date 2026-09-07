# Mid-360S 与 Gemini 335L 协作、冲突处理和自体过滤研究

- 日期：2026-09-07
- 状态：研究建议，供「确定连续障碍观测契约与距离表示」讨论；不代表用户已经批准本轮两项建议。
- 范围：Ubuntu 22.04 笔记本上位机，CPU/PCL 距离查询基线；GPU 路线保持后续实测准入。未运行设备测试或性能测试。

## 推荐回答

**问题一：推荐两个传感器正常运行时持续互补协作，并形成带来源、时间和覆盖状态的统一障碍观测。** 重叠区域交叉核验，非重叠区域补充覆盖；一源有效观测到障碍，不能因为另一源未看到而删除。单源异常时依据剩余有效覆盖和当前运动/停车所需空间判断能否继续，不能仅因另一设备仍在线就继续。此项是依据下面设备特性与软件行为作出的项目设计建议，不是厂商提供的安全保证。

**问题二：推荐复用上游几何自体过滤，在各源采样时刻、各自视点下处理，再融合。** 共享 URDF 碰撞几何和标定来源；仅剔除可信本体点。边界不确定点保留为潜在障碍，或令对应区域成为覆盖未知；另一传感器可以提供不同视点的补充证据，但没有证据不能宣称该区域空闲。不要扩大 padding 来掩盖时间、标定或模型错误。这是项目策略，须由薄适配层和验证补齐，不是现成库的默认保证。

## 设备事实及协作分工

| 设备 | 官方事实 | 本项目建议（推论） |
|---|---|---|
| Livox Mid-360S | 水平 360°，垂直 -7°～52°；典型 10 Hz、200,000 点/秒；支持 PTPv2/GPS 时间同步。官方对 0.1～1 m 低反射或细小目标提示检测效果不能保证。[规格](https://www.livoxtech.com/mid-360s/specs) | 广域周边观测，并在相机视锥之外补覆盖；360°不能等同完整球面、无遮挡或近场全覆盖。 |
| Orbbec Gemini 335L | 官方 USB 系列规格表列深度视场约 90°×65°，最高 1280×800@30fps；理想范围 0.25～6 m，深度处理在机内 ASIC 完成；模式与条件影响工作范围和表现。[规格书 §2、§4.5、§4.7](https://orbbec.com/wp-content/uploads/2024/04/Gemini-330-series-Datasheet-V1.4_20250124.pdf) | 面向操作空间提供局部较密集深度几何，与 LiDAR 保留适当重叠以核验配准并减少遮挡；不能不经安装/目标实测就指定为所有近场的可靠替代。 |

安装验收应检查九轴运动中的视线遮挡及传感器实际有效区。两个视点应共同覆盖所需区域；安装方位、模式、速度上限和允许的降级区域现在不能仅凭规格定值。

## 如何合作，而不只是拼点云

推荐链路：

```text
Mid-360S → 官方驱动 → 时效/质量检查 → 该源时刻的自体分类 ─┐
                                                           ├→ base_link 中统一观测 → CPU/PCL 距离查询
Gemini335L → 官方驱动 → 时效/质量检查 → 该源时刻的自体分类 ─┘
                          固定几何模型、来源/时间/覆盖/冲突状态并行保留
```

以上是拟议架构，不能理解为当前仓库已经完成双源接线。

1. **先标定和校时，再关联。** TF 提供按采样时间的坐标变换；它不替代标定文件的单一事实来源。Livox 驱动原生点格式有逐点时间，CustomMsg 有 timebase 和 offset_time，转成 XYZ 时不应无意丢失必要时序信息。[Livox 官方驱动](https://github.com/Livox-SDK/livox_ros_driver2)
2. **两设备的时间戳类型必须核验。** Orbbec wrapper 提供 device/global/system 时间域；Gemini 330 启动文件的 PTP 参数明确仅用于 Gemini 335Le，不能把 335L 当作同一型号并声称两机已经硬件同步。[时间参数](https://orbbec.github.io/OrbbecSDK_ROS2/en/source/camera_devices/4_application_guide/launch_parameters.html)、[Gemini 330 启动文件](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_camera/launch/gemini_330_series.launch.py)
3. **观测更新与跨源匹配分开。** 建议各源有效数据及时进入观测，跨源关联采用有界时间差；不要让较快相机总等待 LiDAR。ROS message_filters 的 ApproximateTime 是按时间戳排队/配对的机制，不是硬件校时器；队列、最大间隔和丢帧均影响输出。[Humble 源码](https://raw.githubusercontent.com/ros2/message_filters/humble/include/message_filters/sync_policies/approximate_time.h)
4. **不能用最新时间覆盖旧数据年龄。** 统一观测至少保留 source_id、采样区间、接收/处理时间、TF/标定版本、有效期、误差界及有效/遮挡/未知覆盖。若点级元数据成本过高，可按源分块或分层存储。运动中的机器人应使用对应时刻姿态；若单帧持续时间导致不可忽略误差，应分段/逐点补偿或将误差计入边界。这是从逐点采样和按时刻 TF 接口推出的要求。
5. **几何测量融合不等于安全裁决。** 同一表面、时间和误差范围内的观测可去重、加权或聚合；冲突观测先保留并检查时间/TF/反射等原因，不能简单平均到更远位置。障碍并集也不是安全证明：未观测区域仍需覆盖状态。持久化、清除和过期策略必须区分“有效射线证明空闲”和“没有点/无回波”。

## 两个问题的具体行为建议

| 情况 | 推荐行为 | 原因/限制 |
|---|---|---|
| 同一区域双源观测一致 | 去重并融合几何，保留双源支持信息 | 关联成功是增强信息，不应成为新障碍入场门槛。 |
| 一源看到障碍，另一源没有点 | 保留有效障碍；后者视线之外、遮挡或无效深度记为没有证据 | 视场、工作范围和目标条件不同，未看到不等于 free。 |
| 双源对同一区域有不一致有效几何 | 先保留保守障碍表示并报告冲突；检查时空关联和质量 | 不用平均抵消近障碍，也不立即把全部冲突判为设备故障。 |
| 一设备失效或过期 | 重新评估剩余覆盖；仅在已验证降级包络内继续，否则按既有契约停车锁存 | 双源合作通常含不可替代区域；“另一源在线”不足以说明覆盖成立。 |
| 两源对近机器人点均无法判清归属 | 保留潜在障碍或记覆盖未知 | 多一个传感器不能自动消除共有标定误差/共同遮挡。 |

## 上游自体分类的真实语义

### MoveIt Humble

`ShapeMask` 的枚举只有 `INSIDE=0 / OUTSIDE=1 / CLIP=2`，没有 SHADOW 或 UNKNOWN。实现用缩放/膨胀后的几何做包含测试；CLIP 对应 min/max 距离裁剪，实际距离使用点坐标范数，故不能任意改变输入坐标系后仍假定范围相对原传感器。该实现不执行阴影射线分类。[头文件](https://raw.githubusercontent.com/moveit/moveit2/humble/moveit_ros/perception/point_containment_filter/include/moveit/point_containment_filter/shape_mask.h)、[实现](https://raw.githubusercontent.com/moveit/moveit2/humble/moveit_ros/perception/point_containment_filter/src/shape_mask.cpp)

`PointCloudOctomapUpdater` 在消息时刻查 TF 和形状变换，并以传感器原点做射线更新；有 occupied/model/clip/free cell 分支。因此它适合复用到规划地图，但不能将其 Octomap 清除语义直接等同控制侧未知空间契约。两个源先拼成单一 frame 的点云会失去独立射线原点语义；建议规划侧也按源使用 updater，控制侧保留来源和覆盖层。[Humble updater 源码](https://raw.githubusercontent.com/moveit/moveit2/humble/moveit_ros/perception/pointcloud_octomap_updater/src/pointcloud_octomap_updater.cpp)

### robot_self_filter v1.1.0

分类为 `INSIDE / OUTSIDE / SHADOW`，没有 UNKNOWN 或 CLIP；intersection 路径中，过近点也可被标为 INSIDE，SHADOW 来自点到传感器方向与机器人几何的射线相交。包含与射线分类不同，不能把 INSIDE 一概理解成高置信本体。源码按消息时间取 TF，但 TF 异常会保留旧本体姿态；传感器 TF 异常可退回零原点。因此项目必须增加显式有效性检查，不能继承静默回退作为有效安全观测。[v1.1.0 SelfMask 源码](https://raw.githubusercontent.com/leggedrobotics/robot_self_filter/v1.1.0/include/robot_self_filter/self_mask.h)

该版本 README 快速安装针对 Jazzy，CMake 文件有 `cmake_policy(VERSION 3.28)`，应先核验 Humble/Jammy 构建，并固定可用版本或最小补丁。不要直接把 README 性能当作本机性能。[README](https://github.com/leggedrobotics/robot_self_filter/tree/v1.1.0)、[CMake](https://raw.githubusercontent.com/leggedrobotics/robot_self_filter/v1.1.0/CMakeLists.txt)

**推荐复用边界：** 复用几何包含/射线计算，不自写另一套；用薄适配层输出项目的“可信本体、潜在障碍、覆盖未知/无效”状态，保存分类原因。padding 用实测标定/模型误差确定；机器人用于避碰的保守外包络与点云剔除范围必须分开，扩大后者会删掉真实外物。

## CPU/PCL 基线与最小验证

沿用已接受的 CPU/PCL 路线。PCL 已有 VoxelGrid 和 KD-tree；前者以体素内点的质心降采样，后者查询采样点的邻近关系。应记录体素与测量误差，不能把质心点的最近距离当连续障碍表面的保守距离。[VoxelGrid](https://pointclouds.org/documentation/tutorials/voxel_grid.html)、[KD-tree](https://pointclouds.org/documentation/tutorials/kdtree_search.html)

建议先验证双源数据新鲜度、按源过滤和连续障碍覆盖，再测 CPU/GPU 并发下端到端尾延迟。暂不因有 RTX 3060 就提高目标帧率或引入 GPU 距离场；现有硬件条件并不能给出实时性能结论。

在批准具体数值前，至少需要以下证据：

- 静态标靶和运动边缘的跨源配准/时间差分布，涵盖九轴运动及温漂。
- 一源断线、延迟、丢帧、乱序时，新鲜数据仍更新；旧数据不被新时间戳伪装。
- 机器人各连杆边缘的贴近物体在两视点下不被无依据剔除；注入 TF 失败、时间偏差和 padding 变化。
- 对重叠区矛盾观测、共同遮挡、容量溢出，保留冲突/未知并触发既有覆盖不足策略。
- 有效空闲射线与无回波/无效深度分开回放；分类和跟踪转换不使障碍中断。

待定的是具体安装、时间误差上限、帧龄、体素尺寸、剔除边界及降级运动包络；这些需要测量，不在本研究中编造数值或关闭决策票。
