# 双传感器感知与环境距离链：上游复用核验

研究日期：2026-09-07。对应票据：[核验双传感器感知与环境距离链的复用方案](../issues/02-perception-upstream.md)。这是研究证据及待决接口，不是已批准的技术选型；未连接传感器，未构建上游 ROS 工作空间，未证明端到端时延或安全性。

## 可复用矩阵

| 职责 | 上游及版本证据 | 复用方式 | 兼容、维护与边界 |
|---|---|---|---|
| Mid-360S 通信、点云、IMU | [livox_ros_driver2 1.2.6](https://github.com/Livox-SDK/livox_ros_driver2/releases/tag/1.2.6)、[SDK2 v1.3.1](https://github.com/Livox-SDK/Livox-SDK2/releases/tag/v1.3.1) | 官方驱动及 SDK 依赖；改配置、消息适配，避免自写 UDP 协议栈 | 两个发布分别明确增加 Mid-360S 支持；不能沿用仅支持 Mid360 的旧锁定版本。驱动 README 提供 Ubuntu 22.04/Humble 构建入口，但将自身定位为测试/调试工具，产品使用需针对性完善。尚未核对实际设备固件、网络、PPS/时钟配置与丢包表现。 |
| Gemini 335L 深度、标定信息、点云 | [OrbbecSDK_ROS2 v2-main](https://github.com/orbbec/OrbbecSDK_ROS2) | 官方 wrapper；优先配置 `gemini_330_series.launch.py` | 官方支持 Humble；335L 在 v2-main 为新设计推荐，在 main 为完整维护。核验版本 README 推荐固件 1.6.00；这不等同于项目设备已升级。SDK v1 与 v2 的最低固件不同，不能混用文档。USB 带宽、深度模式、时间戳、外参与部署架构需实测。 |
| 从点云移除机器人本体 | [leggedrobotics/robot_self_filter](https://github.com/leggedrobotics/robot_self_filter) v1.1.0 系列 | 保留几何过滤实现；Humble 上先验证或最小回移补丁 | 当前官方快速开始和发布验证是 Jazzy，不能宣称 Humble 即装即用。使用 URDF/TF 及碰撞几何。Generic XYZ 模式可作为 Livox/Orbbec 的统一输入候选；应显式选择 `lidar_sensor_type=0`，不可照抄默认 Ouster 启动配置。 |
| 点云 ROI、采样、空间索引、最近邻 | [PCL](https://pointclouds.org/documentation/tutorials/kdtree_search.html)；[Jammy libpcl-dev 1.12.1](https://packages.ubuntu.com/jammy/libpcl-dev) | 直接依赖 PCL，薄封装业务输入输出 | `nearestKSearch`/`radiusSearch` 返回点索引及平方距离；无需自写 KD-tree。库 API 不是完整机器人表面距离、动态跟踪或实时调度保证。上游文档 1.15.1-dev 不应直接当 Humble 的实际 ABI。 |
| 规划环境/占据图接入 | [MoveIt 2 Humble PointCloudOctomapUpdater 源码](https://github.com/moveit/moveit2/blob/humble/moveit_ros/perception/pointcloud_octomap_updater/src/pointcloud_octomap_updater.cpp) | 复用 PlanningScene/OctoMap updater 或 CollisionObject 接口 | 已存在点云接入与自体遮罩路径，先判断是否满足过滤需求，再引入独立过滤节点。该链为规划场景更新，不应将其更新率、锁和地图延迟直接转移成控制周期保证。 |
| 可选 TSDF/ESDF 建图与查询 | [nvblox](https://github.com/nvidia-isaac/nvblox)、[Isaac ROS nvblox release-3.2](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_nvblox/index.html) | 独立可选后端；使用既有集成，不自写体素融合/ESDF 引擎 | Humble 应核验 release-3.2 组合。当前 latest 文档已切 Jazzy，不能混用。GPU、CUDA、位姿与相机同步条件成立时才评估；动态重建不等于具身份/速度估计的通用多目标跟踪器。 |

## 两处必须改正的兼容性假设

1. **robot_self_filter 的 ROS2 标记不足以证明 Humble 兼容。** 已读当前 [CMakeLists.txt](https://github.com/leggedrobotics/robot_self_filter/blob/4c3badd7135066762d65549224fad3b319f0d754/CMakeLists.txt)：虽然声明最低 CMake 3.8，紧接着无条件执行 `cmake_policy(VERSION 3.28)`。[CMake 官方规则](https://cmake.org/cmake/help/latest/command/cmake_policy.html)要求该最低 policy 版本不高于运行版本；[Jammy 默认 CMake 是 3.22.1](https://packages.ubuntu.com/jammy/cmake)。因此常规 Ubuntu 22.04 工具链有明确配置阻碍。可选路线是固定旧 revision、维护小补丁或升级构建工具，但仍须验证 TinyXML2、TF、PCL 等依赖以及运行行为，不能由“改一行”推出完全兼容。
2. **nvblox 有发行版/算力成本。** [release-3.2 平台要求](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/hardware_setup/compute/index.html)列出 x86 NVIDIA Ampere 或更新、CUDA 12.6+、16 GB 主存、至少 8 GB 显存，推荐 Docker；Jetson 路线配套 JetPack。该代包文档明确 Humble，Jetson Orin 配 JetPack 6.1/6.2。当前 [latest 文档](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html)是 Jazzy/新 CUDA 平台。不能仅安装最新包后假设适配现有环境；也不能用组件平均 benchmark 代替双传感器到控制命令的尾延迟。

MoveIt 的 [Humble 感知教程](https://moveit.picknik.ai/humble/doc/examples/perception_pipeline/perception_pipeline_tutorial.html)仍含 `roslaunch`/ROS1 XML 示例；因此本研究把 Humble 分支源码作为接口依据，教程仅用于理解占据图、自体过滤与 TF 关系，不将其启动示例直接复制到 ROS2。

## 许可证与可复核版本

以下 SHA 是 2026-09-07 通过 GitHub 官方 API 读取的分支快照，不表示已集成。时间为相应提交的 committer UTC 日期；依赖清单还需在后续选型时锁定。

| 上游 | 检索快照/日期 | 许可证证据 |
|---|---|---|
| livox_ros_driver2 master | `4a1def929e5b59c7a8122d19fce6efba581ce9f7`，2026-07-31；Mid-360S 发布 1.2.6 为 2026-04-14 | [LICENSE.txt](https://github.com/Livox-SDK/livox_ros_driver2/blob/4a1def929e5b59c7a8122d19fce6efba581ce9f7/LICENSE.txt)：列明自有部分 MIT，附第三方条款；不能把 GitHub 的 Other 误读成无许可，也不应把全部文件简单标 MIT。 |
| Livox-SDK2 master | `08f523c930b2f0ba1e98a6afaa8d7476bf479908`，2026-07-31；v1.3.1 发布为 2026-04-15 | [LICENSE.txt](https://github.com/Livox-SDK/Livox-SDK2/blob/08f523c930b2f0ba1e98a6afaa8d7476bf479908/LICENSE.txt)：所列 SDK 部分 MIT，并有第三方清单。 |
| OrbbecSDK_ROS2 v2-main | `8e7cad2bfa2c4a6ac4e779be99c64e72166043af`，2026-08-07 | [wrapper LICENSE](https://github.com/orbbec/OrbbecSDK_ROS2/blob/8e7cad2bfa2c4a6ac4e779be99c64e72166043af/LICENSE) 为 Apache-2.0；[OrbbecSDK_v2 LICENSE](https://github.com/orbbec/OrbbecSDK_v2/blob/main/LICENSE.txt)主库 MIT，另列第三方许可。实际 wrapper 携带的 SDK 二进制/版本需随部署锁定核对。 |
| robot_self_filter main | `4c3badd7135066762d65549224fad3b319f0d754`，2026-08-04；package.xml 1.1.0 | [LICENSE](https://github.com/leggedrobotics/robot_self_filter/blob/4c3badd7135066762d65549224fad3b319f0d754/LICENSE) BSD-3-Clause；可见测试与 Jazzy 优化提交，尚非本项目 Humble 实测证据。 |
| PCL master | `a3fa3f147c1812808f960266de25db0c4e5aec90`，2026-09-04 | [LICENSE.txt](https://github.com/PointCloudLibrary/pcl/blob/a3fa3f147c1812808f960266de25db0c4e5aec90/LICENSE.txt) BSD 三条款；部署候选为 Jammy 1.12.1 系列，非该 master。 |
| MoveIt 2 humble | `c283a36186a6f7a5985360e6674bf8fd0790e485`，日期未单独核验 | [LICENSE.txt](https://github.com/moveit/moveit2/blob/humble/LICENSE.txt) BSD-3-Clause；ROS 发行包与插件依赖另锁。 |
| nvblox public | `24eee4948768682fa1ffb969b881efee4fca29c2`，2026-07-03 | [LICENSE.md](https://github.com/nvidia-isaac/nvblox/blob/24eee4948768682fa1ffb969b881efee4fca29c2/LICENSE.md) Apache-2.0，并保留 ETHZ ASL 等第三方条款。 |
| isaac_ros_nvblox release-3.2 | `7908a183acf84f4f1ab3fda7b6d6caf3eefc1f78`，日期未单独核验；main 活动至 2026-08-19 | [LICENSE](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox/blob/release-3.2/LICENSE) Apache-2.0；CUDA/容器/模型等依赖不能由 wrapper 许可概括。 |

## 距离观测与跟踪应分开定义

以下是基于上述 API 语义的工程推论，尚待决策票据确认：

- **即时观测**回答“在本帧观测、坐标和时效条件下，哪些机器人几何部分靠近哪些已观测点/体素”。可复用 PCL、几何库或 ESDF 查询；输出至少要带原始采样时间、使用的 TF/状态时间、来源、有效性、最近几何对与距离语义。无返回点应表示未知/无有效观测，不能自动变成无穷安全距离。
- **动态跟踪**回答“跨帧是否是同一对象、估计速度/不确定性如何、丢失后可信多久”。对象 ID、速度与 TTL 不应绑成产生即时避障观测的先决条件；聚类未稳定时仍可能出现真实近障碍。是否确需对象跟踪以及复用哪一个框架，应由后续任务场景决定。
- **控制安全适配**才负责把观测误差、总数据龄、障碍运动边界、机器人几何误差与制动能力写入约束/失效策略。这是本项目需要的系统契约，不是另造驱动、KD-tree 或建图库的理由。

### 为什么最近点不构成安全证明

这是集合距离的直接推论：如果无噪声观测点集 P 只是实际障碍表面 O 的子集，则 `dist(R, P) >= dist(R, O)`。测到的最近点距离可能**高估**真实间隙。有限视场、稀疏采样和遮挡会扩大该差异；增大经验 padding 本身也不能证明覆盖了所有未知障碍。

需要在后续距离契约中显式处理：

| 因素 | 不足及应验事项 |
|---|---|
| 稀疏/遮挡/无效深度 | 看不到的区域不等于空闲；双传感器覆盖需实测，不能按两者视场简单相加。 |
| 下采样与模型简化 | 体素质心、离群剔除、机器人有限采样可能丢失最近薄物体/真实表面；误差需有边界或有效域。 |
| TF 与采样时间 | 一帧点云可能跨时间，不能一律用最新 TF；滤除本体的外参/关节延迟误差可能把环境点一起滤掉。 |
| 地图陈旧及运动 | Occupancy/ESDF 默认或历史融合假设与动态障碍不一致；记录更新时间与未知策略。 |
| 距离及梯度切换 | `min` 对应的最近点/几何对切换时可不光滑；不能将上一帧法向或对离散索引直接求导的结果当全局有效梯度。 |
| 实时路径 | 建索引/体素更新、ROS 排队、锁、GPU 竞争与控制求解总和需测尾延迟；组件“实时”宣传不等于硬实时保证。 |

## 交给地图的待决问题

1. 目标系统保持 Humble 时，选择 robot_self_filter 回移验证，还是优先利用 MoveIt 已有自体过滤？验收需覆盖贴近本体障碍保留、TF 缺失、点字段与时延。
2. 即时距离选择机器人几何到原始/下采样点集、保守占据表示，或可选 ESDF？各自需什么误差界、未知区行为和距离梯度定义？
3. 动态对象身份/速度是否真是当前控制任务的必需输入？如果是，再研究跟踪上游；不先自写多目标跟踪器。
4. 有无满足 nvblox 选定发行版的算力/时钟/相机条件？没有证据前保持可选，不阻塞基础驱动复用。

研究保存分支目标：`research/oscbf-reuse-perception`。仅本文件进入独立研究提交；未修改实现、未远端写 issue、未替用户关闭 HITL 决策。
