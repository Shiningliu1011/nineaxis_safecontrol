# portable_oscbf

`portable_oscbf/` 包含九轴机械臂的 JAX OSCBF 轨迹跟踪控制器。按需要阅读对应主题；当前依赖与行为以源码和测试为准。

- [目录结构](portable-oscbf/structure.md)：主要文件与职责。
- [控制管线](portable-oscbf/pipeline.md)：输入、输出和处理路径。
- [运动学接口](portable-oscbf/kinematics.md)：POE 正运动学、PSO 逆运动学、关节范围及 mm/° 输入。
- [九轴共同模型与 M07 限制](nineaxis_model_limits.md)：模型身份、TCP 高阶运动、研究参数、阶段检查及逐轴 Ruckig。
- [蝴蝶连续路径参考](../butterfly_reference.md)：原曲线、真实弧长、一至三阶导数及共同消费。
- [外部依赖](portable-oscbf/dependencies.md)：运行和开发依赖。
- [快速开始](portable-oscbf/quickstart.md)：安装、调用与测试。
- [移植到新项目](portable-oscbf/porting.md)：模型、轨迹和控制框架接入。
- [排除的 ROS 文件](portable-oscbf/exclusions.md)：模块边界。
- [CollisionSafety 接口](portable-oscbf/collision-safety.md)：三个公开操作、固定结果结构和当前准入状态。
- [LiDAR 射线占据场景](lidar_occupancy.md)：occupied/free/unknown、固定模型并集、版本与 support 容量。
- [LiDAR 逐点自体过滤](lidar_self_filter.md)：原始 mesh、逐点反馈、首交点、可见范围与覆盖撤销。
- [CollisionParameterArtifact](portable-oscbf/collision-parameter-artifact.md)：安全参数格式、来源校验、不可变配置与 policy identity。
- [Collision geometry artifact](portable-oscbf/collision-geometry-artifact.md)：封闭网格生成、离线几何资产与覆盖检查。
- [动态 support 与未来范围](dynamic_support.md)：可选 tracking、原观测与固定模型来源、逐行预测时间、半径增长及未来包围。
- [参考文献](portable-oscbf/references.md)：相关资料。
