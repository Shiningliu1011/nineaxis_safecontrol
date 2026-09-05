# 传感器标定外参以 config/sensor_extrinsics.yaml 为单一真源（Calibration SSOT）

2026-09-05 定案（05B post-review P0-1）：05A 审计已证明
`config/sensor_extrinsics.yaml` 当前**未被生产感知链消费**——真正进入
`perception_bridge` 的 Camera/LiDAR 静态矩阵来自 `config/perception_runtime.yaml`
的 ROS 参数（`camera_to_world_static` / `lidar_to_world_static`），而 05B/T5 的
标定工具链又把新标定结果写入 `sensor_extrinsics.yaml`。两者一旦分叉，就会出现
split-brain：启动自检认为「标定已通过」，实际送进 CBF 的却是另一套矩阵。
我们决定：**`config/sensor_extrinsics.yaml` 是唯一 calibration authority**——
标定工具写它（T5）、bridge/launch 读它（T7）、T6 启动自检验证
「被校验的 calibration record == 实际 runtime 加载的 record」
（以 `calibration_id`/内容 hash 判定）。

**Considered Options**：A（本决定）sensor_extrinsics.yaml 唯一真源——标定产物
与运行时输入同一文件，split-brain 在结构上不可能；B runtime yaml 唯一真源——
被否：标定工具链（FAST-Calib2/AX=YB，T5）的输出目标与运行时输入分离，仍会
分叉，除非把标定结果反手写回 runtime yaml（两条写路径，违背单一真源）；
C 双文件 + 启动时比对——被否：比对称不上「真源」，只是把分叉检测后置，
且需额外构造 hash 同步机制（T6 已要求 record==runtime 等价校验，C 使该校验
语义混乱）。

**Consequences**：`perception_runtime.yaml` 只保留 topic / frame / voxel /
timeout / fusion 参数，**不再持有任何 calibration matrix**（T7 移除）；
现有「假定安装位姿」相机矩阵（无 provenance、非标定值）迁移到
`sensor_extrinsics.yaml`，标记 `calibrated: false` + provenance 字段
（method=assumed、operator=assignment），保持单 Camera 行为不变（向后兼容
硬约束）；`sensor_extrinsics.yaml` 记录结构扩展：`calibrated`、
`sensor_serial`、`calibration_id`、`method`、`timestamp`、`operator`、
`residual`、`error_estimate`、`frame_from`、`frame_to`；T6 校验中加入
「record == runtime 加载值（calibration_id/hash）」。**T7 为 P0，置于
T1/T2/T3 之前执行**（该三票的安全语义仰赖「读到的就是校验证过的」）。
