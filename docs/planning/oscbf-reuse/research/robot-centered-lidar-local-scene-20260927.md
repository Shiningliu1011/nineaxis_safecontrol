# 外置 LiDAR 的机械臂附近碰撞感知

日期：2026-09-27。研究范围：固定外置 MID-360S、九轴机械臂、ROS 2 Humble，以及本仓库当前感知与碰撞接口。本文记录研究建议和代码核对结果；现场检测率、制动距离和处理延迟尚未验证。

## 按机械臂表面定义关注范围

建议保留外部固定安装，在 `base_link` 下建立随着机械臂姿态变化的关注区域：保留距离任意连杆、工具和所持物体足够近的环境信息，并覆盖即将经过的空间。雷达到点的距离用于判断设备有效量程，点到机械臂的距离用于决定是否参与附近碰撞计算。

例如，机械臂距雷达约 3 m，一件物体距雷达 3.1 m、距前臂表面 0.1 m，应进入附近障碍物集合；另一件物体距雷达 0.6 m、距机械臂约 2.4 m，只有在后续运动或物体接近速度使它进入关注范围时才需要参与附近计算。这是几何说明，数值没有作为现场阈值。

令 `B(q)` 表示当前关节状态 `q` 对应的机器人占据体积，包括全部连杆、工具和所持物体。关注区域可写为：

\[
\operatorname{ROI}(q)=W\cap\{p:\operatorname{dist}(p,B(q))\le R\}
\]

其中 `W` 是覆盖整个使用空间和接近余量的固定工作区域，`R` 是从机器人表面向外计算的保留距离。这是本次研究提出的几何规则。NIST 的研究明确讨论了仅跟踪 TCP 会遗漏肘部运动，以及工具、所持物体和冗余关节运动需要参与距离评估的问题。[NIST 原文，第 3 节](https://tsapps.nist.gov/publication/get_pdf.cfm?pub_id=914783)

对九轴系统，`q` 必须包含 J1。`base_link` 固定时，J1 的平移仍然会改变其后连杆的位置。每次更新都应从关节反馈计算所有相关几何体的位置。

## 一套处理流程

1. 接收有效回波，保留原始采样时间、传感器原点、回波位置和有效性信息。
2. 使用标定外参，把传感器原点和点变换到 `base_link`；根据点的采样时间取得机械臂状态。
3. 使用机器人表面模型识别自身回波，并保留遮挡与无法确定的分类。
4. 在覆盖工作区的局部地图中更新 `occupied`、`free`、`unknown`，保留观测时间。
5. 依据全臂当前形状和即将经过的空间选择相关区域，生成附近环境点和碰撞支持体。
6. RViz 显示机器人模型、附近环境点及未知区域；控制接口同时取得障碍物和有效覆盖信息。

显示用点云可以完全隐藏远处墙面；局部地图更新仍可使用这些墙面回波在关注区域内经过的射线。计算量可以通过射线与局部地图范围的相交测试控制。

外置雷达的原点可以处于局部地图之外。处理时检查整段有效射线与地图边界的交集，保留相交区间；原点和回波终点均无需处于关注区域内。地图保留的区域应覆盖后续运动和停稳期间需要检查的空间，机器人表面附近的筛选可以在地图更新之后执行。

### 固定工作区域与移动关注区域

固定工作区域 `W` 适合排除房间内与工作无关的位置。其边界应覆盖所有允许姿态、J1 行程、工具与负载，以及物体从外围进入所需的预警空间。若运动预测超出 `W`，必须报告覆盖不足。

在 `W` 内使用各连杆几何体的距离查询，或者使用向外扩展后的连杆包围盒进行快速候选筛选。最后依据到几何体表面的距离保留点。扩展各连杆包围盒可以多保留部分空间，同时保持计算简单。

`pcl::CropBox` 支持最小边界、最大边界、位置、旋转和过滤前坐标变换，可以承担固定区域及包围盒筛选；全臂几何、运动预测和覆盖判定由场景处理模块负责。[PCL CropBox API](https://pointclouds.org/documentation/classpcl_1_1_crop_box.html)

已有椭球模型也可用于相关性筛选，但距离必须依据实体表面计算。对椭球三个半轴直接增加同一个数值，通常无法准确表示等距离扩展；使用成熟距离查询或包含扩展区域的包围盒更容易检查覆盖关系。

### 机器人自身点与邻近环境点

自体过滤采用与真实表面接近的几何模型，包含工具、负载和需要保护的外部附件。碰撞计算所用的外包几何可以较大；自体删除区域的尺寸需要单独确定，以免把紧邻连杆的物体一起删除。

建议按照点的采样时刻，检查回波距离与射线首次碰到机器人表面的距离：明确属于机器人表面的回波作为自身点处理；机器人前方的独立回波作为环境点保留；处在标定、模型和时间误差影响区域内的回波记录为 `ambiguous`。`ambiguous` 可以限制运动，不能生成确定空闲的证据。

这与仓库已有的 [ADR 0028：mesh ray 自体过滤](../../../adr/0028-mesh-ray-lidar-self-filter.md) 的设计方向一致。各项误差必须单独测量，再决定表面匹配容差。

固定外置雷达的传感器位姿可以保持不变，但机械臂在一帧采集期间可能运动。因此需要 `q(t_point)`，或具有明确误差上界的时间分组。`robot_body_filter` 上游说明支持逐点采样时间和 TF 插值，还提供 containment、shadow 及模型尺寸调整；其逐点接口要求相应时间和 viewpoint 字段。[robot_body_filter README](https://github.com/ctu-vras/robot_body_filter)

RViz 中使用关节状态驱动的 RobotModel 表示机械臂，同时显示自体过滤后的环境点。机械臂表面点可作为独立诊断图层保留，用于检查外参与模型误差。

### 保留远处回波提供的局部空闲证据

局部地图需要区分三种状态：

| 状态 | 建立依据 | 后续含义 |
| --- | --- | --- |
| `occupied` | 有效回波命中，或已确认的固定场景几何 | 作为障碍物处理 |
| `free` | 合格测量射线实际穿过，且没有被机器人或其他表面遮挡 | 在观测有效期内支持空闲判定 |
| `unknown` | 未观测、被遮挡、信息过期或分类不确定 | 保留覆盖限制 |

OctoMap 原生区分占据、空闲与未知，并提供根据传感器原点和观测更新地图的接口。[OctoMap 官方说明](https://octomap.github.io/octomap/doc/)

单条射线为相交体素提供传感器模型中的空闲证据。实际运动准入还要检查扫描线之间的间隔、最小目标尺寸、体素分辨率和误差范围；局部区域没有返回点，不能直接证明整个区域空闲。这沿用本项目已有的[局部占据研究](lidar-local-occupancy-q5-20260921.md)中的覆盖要求。

例如，射线穿过机械臂旁的一小段空间，最终命中远处墙面：墙面的点可以不进入附近障碍物输出，射线在附近经过的部分仍可更新局部 `free`。若在射线处理中删除了这个回波，就失去了这部分证据。

MoveIt Humble 的 `PointCloudOctomapUpdater` 源码对 `CLIP` 回波计算截断端点，再把对应射线经过的单元加入 `free_cells`，并处理 `occupied_cells` 与自身 `model_cells`。这证明距离截断与直接丢弃回波具有不同效果。[MoveIt Humble updater 源码](https://raw.githubusercontent.com/moveit/moveit2/humble/moveit_ros/perception/pointcloud_octomap_updater/src/pointcloud_octomap_updater.cpp)

本项目建议采用以下更新规则：

- 原始有效回波在进入显示裁剪和点数限制前，为射线更新提供输入。
- 只保存射线与局部地图相交的区间，远处命中点不必作为本地障碍物保存。
- 射线在首次可靠命中位置终止；机器人背后区域保留 `unknown`。
- 移出显示区域、没有收到点、无效零点和历史过期，都不能直接将区域改为 `free`。
- 新进入关注区域的空间初始为 `unknown`，必须经过观测和覆盖检查；历史障碍物被遮挡后按既定有效期处理。

这些规则与 [ADR 0014](../../../adr/0014-local-lidar-occupancy-scene.md) 和 [ADR 0015](../../../adr/0015-asymmetric-occupancy-evidence.md) 的局部占据设计一致。当前代码的完成程度见后文。

## 保留距离由速度、延迟和制动决定

下面是用于设计裁剪范围的工程上界，参数仍需现场测量：

\[
R\ge d_{\mathrm{clear}}+
v_{\mathrm{robot,max}}T_{\mathrm{delay}}+
d_{\mathrm{stop,max}}+
v_{\mathrm{obs,max}}(T_{\mathrm{delay}}+T_{\mathrm{stop,max}})+\epsilon
\]

- `d_clear`：希望保留的最终间距。
- `v_robot,max`：受保护机器人表面点在响应阶段的速度上界，需要包含全部连杆和工具。
- `T_delay`：从目标进入监测范围到制动实际开始的最长延迟，包含扫描覆盖间隔、成帧、传输、处理和命令生效。
- `d_stop,max`、`T_stop,max`：制动期间机器人继续运动的距离和时间上界。
- `v_obs,max`：障碍物向机器人接近的速度上界。
- `epsilon`：标定、几何、采样、时间同步和空间离散误差的保留量，分别记账以免重复加入。

这一公式用于当前机器人几何外扩。如果直接采用从当前时刻到停稳期间所有机器人几何的并集 `B_swept`，则机器人自身移动已经包含在几何中，外围只需覆盖 `d_clear`、障碍物接近量和误差量。两种表达不能把同一段机器人移动计算两次。

NIST 论文讨论了反应时间、制动过程、接近运动与测量误差对分离距离的贡献，并指出制动性能随速度、负载和姿态变化；本段据此给出工程推导，没有把公式作为现场能力证明。[NIST 原文，第 1、4、5、6 节](https://tsapps.nist.gov/publication/get_pdf.cfm?pub_id=914783)

在碰撞控制中，还需要核对短时间预测使用的轨迹或可达范围。超出预测区域的后续动作应重新取得场景覆盖。长距离规划应保留完整工作区域中的固定障碍物，附近筛选只限制当前碰撞计算所需的数据。

MID-360S 典型帧率为 10 Hz；单纯换算得到的 0.1 秒只代表一帧的时间尺度。障碍物若以 1 m/s 接近，0.1 秒内已经移动 0.1 m。这个算例没有包含漏检、遮挡和处理延迟，所以无法由帧率直接指定一个有效的安全距离。[Livox 官方参数](https://www.livoxtech.com/mid-360s/specs)

## 降采样与工具选择

`pcl::VoxelGrid` 用每个体素内点的质心减少点数。它可以用于显示或候选点生成，碰撞接口还需保留体素尺寸和误差，使一个代表点覆盖被合并的观测范围。[PCL VoxelGrid 教程](https://pointclouds.org/documentation/tutorials/voxel_grid.html)

若代表点位于立方体中心，边长 `h` 的半对角线为 `sqrt(3) * h / 2`；若代表点使用任意质心，需要依据该代表位置计算能包住整个体素或全部被合并点的半径。空间离散误差与测量误差还要分别计入支持半径。仓库已有 [ADR 0017](../../../adr/0017-support-point-metric-radius.md) 描述支持半径，[ADR 0018](../../../adr/0018-conservative-support-capacity.md) 描述覆盖合并和容量处理。

推荐保留本仓库的碰撞控制接口，使用成熟点云和几何库完成感知前处理。工具职责如下：

| 工具 | 本任务中的职责 | 需要补充的项目内容 |
| --- | --- | --- |
| PCL `CropBox` | 固定工作区和几何候选区域筛选 | 根据关节状态计算各连杆区域 |
| PCL `VoxelGrid` | 减少局部候选点和显示点数 | 支持半径、最小目标保留率、容量检查 |
| MoveIt Humble `PointCloudOctomapUpdater` | ROS 2 点云占据更新、形状排除、射线处理参考 | 全臂动态关注区域、时间有效期和项目覆盖要求 |
| 现有碰撞内核 | 全部活动机器人几何与环境支持体的距离查询 | 可信场景生产、工具与负载几何、覆盖证据 |

MoveIt 的 `max_range` 按传感器测量设置范围，不能直接表达点到全臂表面的保留距离。若把处理后的点云换到 `base_link` 后交给依赖 `header.frame_id` 确定传感器原点的组件，还需要专门核对原点语义；原点位置错误会改变射线经过的空间。[MoveIt Humble updater 源码](https://raw.githubusercontent.com/moveit/moveit2/humble/moveit_ros/perception/pointcloud_octomap_updater/src/pointcloud_octomap_updater.cpp)

`robot_body_filter` 可作为自体过滤算法与调试输出的参考。上游有 `ros2` 分支，但本次核对的提交 `7bd77ea7ea7cd159196a2c2c90db41f3a847c961` 的 `package.xml` 仍声明 `catkin`、`roscpp` 和 `dynamic_reconfigure`；本项目 Humble 的构建与接口适配需要独立验证。[对应提交的 package.xml](https://github.com/ctu-vras/robot_body_filter/blob/7bd77ea7ea7cd159196a2c2c90db41f3a847c961/package.xml)

## 本仓库的实际基础

以下是本次主会话检查当前文件得到的结果；代码位置以本次工作目录为准。

| 检查项 | 当前状态与影响 | 文件依据 |
| --- | --- | --- |
| 坐标和外参 | 配置使用 `base_link`，Y 向上；LiDAR `calibrated: false` | [perception_runtime.yaml](../../../../config/perception_runtime.yaml)、[sensor_extrinsics.yaml](../../../../config/sensor_extrinsics.yaml) |
| 固定裁剪 | `workspace_min=[-0.9,-0.2,-0.6]`、`workspace_max=[0.9,0.8,2.0]`，单位为米；其覆盖全部允许姿态的能力尚未验证 | [perception_runtime.yaml](../../../../config/perception_runtime.yaml) |
| LiDAR 接入 | `source_topic_lidar` 为空；当前配置尚未启用该输入 | [perception_runtime.yaml](../../../../config/perception_runtime.yaml) |
| 点数与体素 | `max_points=5000`；相关体素设置为 `0.03 m`；桥接代码在坐标转换和工作区裁剪前随机限点 | [perception_bridge.py](../../../../src/robot_safecontrol_moveit/perception_bridge.py) |
| 关节与自体处理 | 桥接代码缓存最新 `JointState`；当前点云流程没有消费 FK 做自体过滤；读取点云只提取 `xyz` | [perception_bridge.py](../../../../src/robot_safecontrol_moveit/perception_bridge.py) |
| 旧处理入口 | 执行变换、固定 AABB 裁剪、可选机器人球体过滤和体素处理；ROS 桥接调用未传入 `robot_spheres` | [safety_snapshot.py](../../../../portable_oscbf/work/safety_snapshot.py)、[perception_bridge.py](../../../../src/robot_safecontrol_moveit/perception_bridge.py) |
| 机器人几何 | 几何清单为 `base_link` 和 `Link1` 至 `Link9`；没有独立命名的工具或负载条目，实际包围范围需要核对 | [collision_geometry_artifact.json](../../../../portable_oscbf/config/collision_geometry_artifact.json) |
| 距离查询 | 当前内核对活动机器人椭球和环境支持体计算欧氏距离；连杆变换使用共享九轴运动学 | [_ellipsoid_point.py](../../../../portable_oscbf/work/_ellipsoid_point.py)、[_collision_geometry.py](../../../../portable_oscbf/work/_collision_geometry.py) |
| 覆盖检查 | `CollisionScene` 检查时间、标识、容量与 `required_space_covered` 等生产者信息；空间覆盖证据需要由场景生产者建立 | [_collision_scene.py](../../../../portable_oscbf/work/_collision_scene.py) |
| 占据历史 | 当前 `static_occupancy.py` 依据命中体素和时间分类，输出与本帧占据相交；当前没有射线更新，不能据此认定被遮挡物体持续保留 | [static_occupancy.py](../../../../portable_oscbf/work/static_occupancy.py) |

`collision_scene_server` 的场景历史、自体过滤、射线占据和覆盖职责已经写入 [ADR 0038](../../../adr/0038-dedicated-collision-scene-server.md)。本次在 `src/` 和 `portable_oscbf/work/` 中未找到承担这套完整职责的运行实现，因此这些能力仍属于设计与实现之间需要完成的部分。

对当前桥接处理顺序，后续实现应将附近候选选择放在候选点数量限制之前，同时独立保存射线更新输入。容量不足必须保守合并或报告覆盖失败；随机保留少量点无法证明小障碍物被保留。

当前展示窗口直接使用原始 `/livox/lidar`。桥接流程的点数限制不作用于这个显示输入，本次代码检查无法用该限制解释截图中的点数分布。

## 现场能力的验证条件

MID-360S 官方列出水平 360°、垂直 −7° 至 52°、每秒 200,000 点的参数，同时说明 0.1–0.2 m 的检测精度不保证，0.1–1 m 内低反射或微小目标的检测效果不保证。软件裁剪只选择已有观测；其效果不会增加扫描点数或补全遮挡区域。[Livox 官方参数及备注](https://www.livoxtech.com/mid-360s/specs)

验收应使用真实记录或受控采集，至少覆盖以下情形：

1. 在每段连杆、工具和负载附近放置规定最小尺寸与材质的目标，检查原始观测、过滤后保留率和首次检测延迟。
2. 机械臂折叠、伸展及 J1 两端位置下，检查附近区域是否仍然处于有效观测范围，并记录遮挡造成的未知区域。
3. 目标紧邻连杆时检查自体过滤；目标暂时被遮挡时检查占据历史和覆盖限制。
4. 目标离开后，用新的有效射线确认空闲；只移出 RViz 显示范围时，检查地图状态没有被直接清空。
5. 记录从实际观测到控制使用的最长延迟，依据真实速度、负载和制动性能计算保留距离。
6. 检查低点数、数据过期、关节状态缺失和场景容量不足时，系统确实报告能力受限。

本次完成了资料研究与当前代码核对，没有启动设备采集、改变安装位置、修改运行配置或执行机器人运动。上述验收尚未执行。
