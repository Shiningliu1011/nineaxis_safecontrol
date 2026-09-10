# MoveIt ShapeMask 逐源自体过滤核验

核验日期：2026-09-11。固定 MoveIt2 `2.5.9`、geometric_shapes `2.3.4` 官方源码。范围为首版薄适配的上游证据；不改变已定的逐源过滤架构、不引入 shadow filtering、不安装依赖或修改运行代码。

结论：ShapeMask 可作为独立几何包含内核复用，公共库确实安装并导出；但不能直接把所有 `INSIDE` 当成可信本体点。必须在外层保证当前帧变换完整、几何删点区域可信、边界不确定点保留。当前 URDF mesh 是否符合可信删除区域仍需本地审计。

## API、依赖与分发

头文件为 `<moveit/point_containment_filter/shape_mask.h>`，命名空间 `point_containment_filter`。核心接口为 `addShape(ShapeConstPtr, scale=1.0, padding=0.0)`、`removeShape`、`setTransformCallback`、`maskContainment(PointCloud2, sensor_pos, min_sensor_dist, max_sensor_dist, vector<int>&)`；另有单点 `getMaskContainment`。回调类型为 `bool(ShapeHandle, Eigen::Isometry3d&)`，没有时间参数。输出仅为每点整数 mask，没有发布器、删点云或有效性状态。[固定版接口](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/perception/point_containment_filter/include/moveit/point_containment_filter/shape_mask.h#L43-L82)

`moveit_point_containment_filter` 是 SHARED 库，其直接 ament 依赖为 `rclcpp`、`sensor_msgs`、`geometric_shapes`；包含 Eigen 类型。子目录安装头文件；父级把该库列入安装 TARGETS 并通过 `ament_export_targets(export_moveit_ros_perception HAS_LIBRARY_TARGET)` 导出。因此“不导出公共库、必须 vendor”不成立。后续应验证已安装包的 CMake target 使用方式和实际链接；不能将源码导出证据写成当前环境编译已通过。[内核构建](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/perception/point_containment_filter/CMakeLists.txt)、[包安装与导出](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/perception/CMakeLists.txt#L55-L115)

所属包 `moveit_ros_perception` 版本 2.5.9，整个包还依赖 MoveIt、图像与 OpenGL 等组件；独立调用内核不等于安装包没有这些依赖。包标记 BSD，ShapeMask 源文件含三条款 BSD 条件；若复制源码须保留原版权、条件及免责声明。[包声明](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/perception/package.xml)、[文件许可证](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/perception/point_containment_filter/include/moveit/point_containment_filter/shape_mask.h#L1-L32)

本机核验：主线程 `dpkg-query` 确认未安装 `ros-humble-moveit-ros-perception`；存在 geometric-shapes `2.3.4-1jammy.20260726.110819`。本子任务亦未找到 ShapeMask 头文件。尚无本机链接、ABI 或运行证明；此处 2.5.9 是核验目标，不声称它已安装。

## 分类及失败语义

| 项目 | 固定源码事实及适配约束 |
|---|---|
| 分类 | `INSIDE=0`、`OUTSIDE=1`、`CLIP=2`；无 UNKNOWN。INSIDE 由 body.containsPoint 判断。 |
| 范围 | CLIP 来自点到输入坐标原点的距离超出 min/max；传入的 sensor_origin 被忽略。空 body 集合直接全部 OUTSIDE，连距离裁剪也不执行。 |
| TF 失败 | 回调 false 只记录错误，既不抛出有效性失败，也不移除该 body；后续仍遍历所有 body，可能沿用旧 pose。球包围缓存只更新成功部分。 |
| 时间 | 内核不读取 header.stamp/frame_id，不查 TF；必须由外层按该源该帧 stamp 构造不可变的完整变换快照。 |
| 点格式 | 使用 float XYZ 迭代器，点数按 data.size()/point_step 计算；外层须验证布局和有限值，不能假设任意 PointCloud2 均安全。 |
| 阴影 | 只做几何包含，不做射线遮挡，也没有 shadow 分类。 |
| 单点接口 | 不刷新变换，直接使用 body 当前 pose；不能当成会按时间更新的独立快捷接口。 |

以上来自 [ShapeMask 实现](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/perception/point_containment_filter/src/shape_mask.cpp#L110-L182)。适配结论：先全量验证变换和几何，再运行 mask；失败不得调用可能使用旧 pose 的内核并把结果标记成功。首版只删除经额外可信门槛验证的 INSIDE；CLIP 与 OUTSIDE 都不代表自由空间。失败帧保留原始数据及 unknown/invalid 状态，具体上层处置遵循既定接口。

## 几何与边界：零 padding 不是可信性证明

ShapeMask 将 scale、padding 交给 geometric_shapes Body，并更新内部数据；不自行推导误差带。[形状注册](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/perception/point_containment_filter/src/shape_mask.cpp#L63-L95)

`MESH` 工厂创建的是 `ConvexMesh`。[类型分派](https://github.com/moveit/geometric_shapes/blob/2.3.4/src/body_operations.cpp)。`ConvexMesh::useDimensions` 用 Qhull 对顶点建凸包；`containsPoint` 用包围盒和凸包平面判定。球半径为 `radius*scale+padding`；其他形状也各自处理缩放及 padding。因此正 padding 会扩大删点候选区域；凹网格即使 scale=1、padding=0，凸包也可能覆盖真实本体外的凹陷空间。负 padding 亦不能自动证明凸包成为真实实体的子集。边界存在形状相关比较规则，没有统一未知带。[几何实现：Sphere::updateInternalData、ConvexMesh::containsPoint/useDimensions](https://github.com/moveit/geometric_shapes/blob/2.3.4/src/bodies.cpp)

**工程推论：未经审计的 mesh INSIDE 不足以授权删点。** 薄适配必须另有可信几何覆盖证明和误差带策略；边界、凹陷及未证明区域保留/未知。不能用 `scale=1, padding=0` 或直接负 padding 代替该证明。本研究没有验证本项目 mesh 的凸性、实体忠实度、传感器误差或可用内缩量。

## 可独立使用，但不要把 Octomap 发布路径当成纯过滤节点

官方 `PointCloudOctomapUpdater` 先按点云 frame_id/stamp 更新变换缓存，再调用 ShapeMask；其可选过滤云沿用输入 header，仅创建 XYZ 字段，收集 OUTSIDE 点且受 subsample 影响。发布发生在 Octomap 更新之后；插件本身依赖 monitor/tree，不能因为提供 filtered_cloud_topic 就视作独立逐源过滤器。[调用、时间戳与输出路径](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/perception/pointcloud_octomap_updater/src/pointcloud_octomap_updater.cpp#L176-L355)

所以可复用的是公共 ShapeMask 库。逐源订阅、stamp/身份传递、URDF collision origin 到云坐标的变换、附加字段保留、成功/未知状态及删点置信门槛由适配层负责。是否保留 organized 布局必须遵循项目接口，不从上述 XYZ 调试输出继承。库源码未提供本项目吞吐/延迟保证，本次亦未做安装、构建、点云回放或真机验证。
