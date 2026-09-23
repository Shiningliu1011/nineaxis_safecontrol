# robot_safecontrol 项目开发规范

本目录记录当前仓库已经采用、并由源码或测试支持的开发规则。编写代码前，应按改动范围读取对应文档。

## 资料优先级

1. 根目录及目标目录中的 `AGENTS.md` 负责工作方式和授权边界。
2. `docs/CONTEXT.md` 与 `docs/adr/` 负责领域术语、坐标语义和设计决定。阅读时要区分设计目标与当前完成状态。
3. 公共源码入口负责运行时取值，例如 `robot_spec.py`、`ros_conventions.py` 和 `oscbf_trajectory.py`。
4. 生产配置、launch 文件与当前测试负责证明当前行为。

## 规范索引

- [目录与模块职责](./directory-structure.md)：代码位置、依赖方向和公共入口。
- [ROS 接口、坐标与时间](./ros-interfaces.md)：topic、QoS、关节身份、坐标变换和 freshness。
- [配置与生成文件](./configuration-and-generated-files.md)：生产配置、资源解析、launch 参数和生成物维护。
- [控制内核运行依赖](./control-kernel-dependencies.md)：cbfpy、JAX、qpax 固定版本，运行接口和升级验证。
- [错误处理](./error-handling.md)：输入校验、错误类型、边界转换和清理规则。
- [日志与运行证据](./logging-guidelines.md)：日志级别、稳定事件码、报告和运行快照。
- [安全边界](./safety-guidelines.md)：仿真、shadow、live、反馈来源和控制安全门。
- [质量与验证](./quality-guidelines.md)：编码约定、测试范围、构建命令和工作区保护。

## 使用方法

- 修改 ROS node 或 launch 时，至少读取 ROS、配置、错误、日志和质量规范。
- 修改 `portable_oscbf/work` 时，至少读取目录、配置、安全和质量规范。
- 修改控制内核依赖、`CBFConfig` 或 QP 求解接口时，必须读取控制内核运行依赖规范。
- 修改真机、感知、碰撞或控制门控时，必须读取安全规范，并核对相关 ADR 与测试。
- 数据跨越三个以上组件时，同时读取 `../guides/cross-layer-thinking-guide.md`。
- 准备新增公共常量或转换函数时，同时读取 `../guides/code-reuse-thinking-guide.md`。

规范只记录已经存在的项目规则。新的设计决定应在代码、测试和相关 ADR 完成后同步到这里。
