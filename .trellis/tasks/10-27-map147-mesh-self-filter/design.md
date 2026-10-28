# 接口与数据流

`RobotRayGeometry` 使用 urdf_parser_py 与 trimesh 读取原始 collision 三角面、origin 和 scale，局部查询树复用，逐点射线通过共享 NineaxisKinematics.link_poses 变换。资源内容 hash 与独立验证记录共同构成几何身份。

`ClockMapping` 明确时间域、整数锚点、有效范围及误差。`TimedLidarScan` 保存原始点和采集时间；`JointStateHistory` 使用真实反馈或明确仿真样本，严格包围逐点完整时间误差区间。使用明确来源的逐轴速度界与插值误差计算位置范围。ROS 纯适配读取真实 PointCloud2 和 JointState，不创建 node。

`MeshRaySelfFilter.classify` 输出 environment/self/occluded/ambiguous、首交点 link/triangle、距离范围、可见前缀及 unknown 区间。测量区间与机器人表面区间重叠时保留 ambiguous；证据支持精确表面时生成 self。物理误差缺少来源或无效时输出全局无效。

结果转换为扩展 LidarRayBatch。占据 owner 对 unknown 区间撤销旧 free，保留仍有效 occupied；同批 free 不能覆盖 unknown。全局无效记录使场景保持不可准入，直到后续有效批次重新取得覆盖。

现有 #108 参数和 OccupancySnapshot 结构保持原定义；#101 修改动态 support 与 query 时间的内容由主会话集成。

原九轴当前 mesh 拓扑和标定未取得完整物理验证，接口必须保留这种状态。数学解析场景的零误差仅用于有明确解析来源的离线算法验证。
