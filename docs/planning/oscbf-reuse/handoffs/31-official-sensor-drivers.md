# [P0] 双传感器官方驱动接入与版本冻结

- Tracker：https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/31
- 日期：2026-09-08；认领：Shiningliu1011。
- 状态：研究与本地只读核查完成；候选版本已明确，未实施、未达到部署冻结与关闭门槛。
- 已同步[阶段进度评论](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/31#issuecomment-5586798382)，不是 resolution；保持 OPEN，不追加地图已完成索引。

## 起点与范围

用户原始指令：“继续下一个ticket”。依照[标定工具链交接](25-calibration-toolchain.md)与[推荐窗口顺序](../DECISION-WINDOW-ORDER.md)，本窗口处理本票。已先认领；原生依赖仍为开放的[规格与术语修订](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/20)和[标定 SSOT 接线](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/27)。仅推进不依赖缺失前置的研究，不撤销依赖。

起点 main，commit `659da6c6db598abddc272ad0941b72f77793211b`。保留已有 ADR、MAP、感知规格修改及未跟踪 handoffs、研究、.scratch 和大然电机资料。本轮不安装依赖、不启动传感器、不修改运行代码或配置；研究分支与产品分支分离：本地 `codex/research/official-sensor-drivers`，commit `05dbfa433dfd34ce59a38f990620bafd14f22819`，worktree `.scratch/oscbf-reuse-wayfinder/31/research-worktree`；未推送，研究文件已复制回主工作区供审阅。

## 本机核验

原始命令、返回码、输出及相关源码/配置 SHA256 见[本地审计 JSON](../../../../.scratch/oscbf-reuse-wayfinder/31/local-audit.json)。

- Ubuntu 22.04.5 LTS，x86_64，内核 `6.8.0-138-generic`；ROS Humble 已安装。系统 libusb 开发包为 `2:1.0.25-1ubuntu2`。
- 仅加载 `/opt/ros/humble/setup.bash` 后，`ros2 pkg list` 未匹配 livox/orbbec；这只描述所检查的环境，不排除其他未加载 overlay 或目录。
- 本次 `lsusb` 没有可识别的 Orbbec 设备；不能据此推断以太网雷达不存在。没有扫描网络、查询固件或触发数据采集。
- 用户已确认型号为 MID-360S / Gemini 335L，原话：“mid-360s和gemini335l，现在还没法连接，明天可以连接两个设备”。连接窗口预期为明天；未安排自动任务。序列号、固件及在线设备身份仍待读取，型号确认不等于本机兼容实测通过。
- `package.xml` 没有两个官方驱动依赖；当前 launch 没有启动它们。配置注释中出现的驱动命令不是本机可复现证据。
- 相机配置预期 `/camera/depth_registered/points`、`camera_color_optical_frame`；LiDAR 在 runtime 与 real profile 中仍禁用。不要直接打开 topic 即宣称接入完成。
- bridge 两路订阅均为 `sensor_msgs/PointCloud2` + `qos_profile_sensor_data`，解码仅保留 xyz；逐点时间、tag、line 等不会保留到其当前缓存。
- `_sensor_callback` 将 header stamp 转为秒，使用配置 input_frame 做变换，未检查实际 header.frame_id。时间域、错误帧拒绝和空帧健康语义须由既有时间、SSOT、观测/健康票落实，不能用驱动成功启动代替。

## 复用与接口研究

见[官方传感器驱动研究](../research/official-sensor-drivers.md)。候选固定版本、许可证、构建/启动说明与字段证据由该研究持有；在本机复现之前只是候选冻结，不是部署冻结。

关键接入差异：官方 MID-360S launch 默认 `xfer_format=1`（CustomMsg），与 bridge 的 PointCloud2 订阅不匹配；该脚本使用 Python 常量，不能假定追加 `xfer_format:=0` 就会覆盖。实施应在本项目提供小型 launch 适配，直接给官方节点传入 `xfer_format=0` 等已核验参数，保留上游源码不改；启动后再核对实际 topic 类型。`livox_frame` 是默认值而非不可变常量。相机配置的彩色注册点云 topic/frame 在候选源码中吻合，但实际设备流配置仍需连接后确认。相机内部帧同步和设备到主机的时间映射不构成相机与雷达硬同步。

## 编码与验收门

可准备隔离的依赖清单、构建环境和只读消息审计工具。正式双源接入仍需前置规格/SSOT 契约落地，并按实物身份选定驱动配置；本轮不自动实施产品代码。

实施时只做官方驱动依赖、launch/参数与消息薄适配；现有 FusionEngine 与几何/控制算法保留，未发现必须 fork 上游的证据。外参只应用一次：驱动输出维持其明确的传感器局部帧，传感器至 base_link 由 SSOT 负责。相机彩色/深度内部标定不等于本项目 sensor→base 标定。

每路独立验收并留存：

1. 精确硬件型号、序列号、固件、USB链路/网卡配置；源码 commit、SDK版本、许可证与依赖锁、构建命令及日志、实际 ament 解析路径。
2. `ros2 topic info --verbose` 的类型/QoS/发布者；样本的 header、fields、point_step、row_step、endianness、实际 frame 与米单位对照；避免仅凭话题名判断格式。
3. 点云时间戳的时钟域、基准时刻和扫描/曝光语义，记录设备时间与主机接收时间，不能将回调时间或近似配对当成硬同步。漂移、跳变、断流与重启重同步交给感知时间票定量核验。
4. 错帧、错消息类型、空帧、掉线、时间回退、错误序列号与重连的可观测行为；缺源不能等价为空且安全。
5. 两路独立通过后才做同机并发采集，记录丢帧、USB/网络负载、CPU/内存和延迟；之后再接统一观测链。

没有执行本机驱动构建、设备采集、bag 回放或并发性能测量，不能关闭本票或宣称双源准入。文档研究不需要运行产品测试套件。

## 回退与下一窗口

未来驱动在独立 workspace 部署，保留旧依赖/参数/标定与模型快照；停止相关节点后切回已记录版本并重新核验实际解析路径和身份，旧假定标定不会因回退获得准入。

下一研究窗口为[感知时间同步与延迟模型](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/7)；本票未完成仍是其整票原生阻塞，可先研究测量方法，不得声称已获得设备时钟证据。本窗口不自动开展下一票。
