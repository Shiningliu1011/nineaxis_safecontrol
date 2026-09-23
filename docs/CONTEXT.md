# 领域词汇

本项目使用 9 自由度机械臂，在 MuJoCo 中执行无碰撞过渡，并由 OSCBF 控制器跟踪参考轨迹。按主题阅读术语定义，设计目标与当前完成状态需要分别核对。

- [系统结构](domain/system.md)：状态流、过渡管线和控制入口。
- [机器人](domain/robot.md)：关节、坐标和被控对象。
- [轨迹与几何](domain/trajectory-and-geometry.md)：参考轨迹与几何表示。
- [真机部署](domain/hardware.md)：硬件模式和反馈。
- [障碍观测](domain/observation.md)：传感器输入与覆盖。
- [标定记录与准入](domain/calibration.md)：标定身份与有效性。
- [安全约束与准入](domain/safety.md)：约束、裕度和状态。
