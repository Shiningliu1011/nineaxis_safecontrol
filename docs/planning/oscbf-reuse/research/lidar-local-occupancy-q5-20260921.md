# Q5：LiDAR 局部占据场景

研究日期：2026-09-21。本文为设计分析，尚未修改运行代码或确认性能参数。

建议采用选项 2：有界三维局部占据场景，查询接口区分 occupied、free、unknown；occupied 导出供 ellipsoid–point 使用的 support points。场景更新与控制查询使用不同频率，消费者读取不可变 CollisionScene。

## 仓库依据

- `CONTEXT.md:198` 定义 CollisionScene，要求版本、采集时间、占据证据、覆盖状态和误差范围。
- `docs/adr/0010-unified-scale-collision-safety-module.md:7` 要求规划、轨迹执行、OSCBF 共享场景及机器人几何，并保留 ellipsoid–point 环境查询。
- `portable_oscbf/work/static_occupancy.py:98` 的 static_mask、unconfirmed_mask 均与本帧 occupied 相交。last_seen 记录不会使遮挡 voxel 继续进入输出。该类的 static/unconfirmed/instant 是时间分类，与 occupied/free/unknown 含义不同。
- `portable_oscbf/work/fusion_engine.py:284` 的 perception_valid 依据来源和 fusion_age，没有空间覆盖检查。
- `src/robot_safecontrol_moveit/perception_bridge.py:486` 提取 XYZ、限制点数并预处理；场景射线更新需要保留传感器原点、采集时间、有效返回含义和相应变换。

## 开源依据

[OctoMap 官方介绍](https://octomap.github.io/)说明 occupied/free 与隐式 unknown 的三维占据表示。
[OccupancyOcTreeBase.hxx](https://raw.githubusercontent.com/OctoMap/octomap/devel/octomap/include/octomap/OccupancyOcTreeBase.hxx)的 insertPointCloud、computeUpdate 对射线路径与终点分别生成 free_cells、occupied_cells，同批更新优先保留 occupied；updateNode 使用 log-odds。因此其实现支持射线证据融合，具体判定仍受传感器模型及阈值影响。

[MoveIt Planning Scene Monitor](https://moveit.picknik.ai/main/doc/concepts/planning_scene_monitor.html)使用 OccupancyMapMonitor 和 OctoMap 维护三维环境，PlanningSceneMonitor 提供线程安全访问。这支持持久场景与读取查询分离的设计。文档没有建立本项目的逐 voxel 过期、unknown 准入或 scene_revision 规则。

[Nav2 Voxel Layer](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/voxel/)区分 marking、clearing，并使用三维 raycasting。observation_persistence 表示消息缓存时间。其模型投影到二维供规划控制使用；本项目需要保留全三维查询。本文据此借鉴更新语义。

这些是上游当前文档及源码依据，具体 ROS 2 Humble 依赖版本和运行性能尚未验证。

## 建议的场景规则

1. 有效物体返回提供 occupied 证据；可靠射线在终点之前提供 free 证据。未返回数据、无效返回和丢包不得自动生成清空射线。只有传感器明确提供可靠的无命中方向及有效距离时，才允许相应 free 更新。
2. 射线止于可见遮挡物，保留后方已有占据证据至其失效；过期证据转为 unknown。free 同样具有逐 voxel 有效期限。新帧不能延长未观测区域的有效期。
3. 机器人自体过滤同时保留射线遮挡含义。点云降采样后产生的中心点不能未经验证就作为原始测量射线终点。
4. 射线穿过 voxel 是离散传感器模型中的 free 证据。需要结合分辨率、最小可检测障碍尺寸、射线覆盖和误差，验证该空间是否足以准许运动。
5. 规划检查全身经过的空间；轨迹执行检查下一执行区间和停车所需区域；OSCBF 输出准入检查相应运动及停车区域。相关区域包含 unknown 或过期证据时拒绝正常运动准入。无关远处 unknown 不影响当前命令。
6. occupied 的体积和测量误差必须进入 support points 查询的保守几何范围。仅检查中心点会遗漏 voxel 边缘；仅增加角点也不能保证覆盖 voxel 内部。立方 voxel 中心到任意位置的距离上界为 sqrt(3)*voxel_size/2，该米制上界需要经过验证后进入 ellipsoid 尺度计算。
7. scene_revision 标识一次完整快照提交。有效观测更新、过期状态改变、局部范围改变及场景重建均生成新版本；重复读取保持版本。时间新鲜度另行检查，不能只看版本号。
8. 一次查询固定一个版本；命令提交前确认所需区域的依据仍然有效。新版本触发相关区域复查，能够证明变化无关时可复用结果。
9. 历史占据不能预测遮挡中的运动物体。动态环境需要给出障碍运动与观测年龄造成的空间不确定范围，并在无法覆盖所需区域时拒绝准入。

## 验证范围

本次完成代码静态核查和第一方资料阅读；没有运行仿真、性能测试或硬件测试。
实施时需验证遮挡保留、明确 free 清除、occupied/free 过期为 unknown、断流、重复帧、时间异常、自体遮挡、场景版本并发读取，以及 voxel 体积覆盖。
地图分辨率、有效期限、support points 容量和更新频率需要根据传感器数据、运动范围、停车能力及延迟测试确定。
