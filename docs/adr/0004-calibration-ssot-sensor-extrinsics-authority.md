# 传感器标定外参以 config/sensor_extrinsics.yaml 为单一真源（Calibration SSOT）

2026-09-05 定案（05B post-review P0-1；二轮 post-review 增补「运行时文件身份」）：
05A 审计已证明
`config/sensor_extrinsics.yaml` 当前**未被生产感知链消费**——真正进入
`perception_bridge` 的 Camera/LiDAR 静态矩阵来自 `config/perception_runtime.yaml`
的 ROS 参数（`camera_to_world_static` / `lidar_to_world_static`），而 05B/T5 的
标定工具链又把新标定结果写入 `sensor_extrinsics.yaml`。两者一旦分叉，就会出现
split-brain：启动自检认为「标定已通过」，实际送进 CBF 的却是另一套矩阵。
我们决定：**`config/sensor_extrinsics.yaml` 是唯一 calibration authority**——
标定工具写它（T5）、bridge/launch 读它（T7）、T6 启动自检验证
「被校验的 calibration record == 实际 runtime 加载的 record」
（以 `calibration_id`/内容 hash 判定）。**且"实际 runtime 加载"必须落到唯一
resolved 绝对路径**——源码树 `config/` 与 `setup.py` 安装进
`share/<pkg>/config/` 的是两个物理副本，T7 必须令 bridge/launch 经
`get_package_share_directory('robot_safecontrol_moveit')/config/sensor_extrinsics.yaml`
（ament_index_python，与 launch 同一解析机制）解析出唯一路径加载，
启动时记录该路径 + `calibration_id` + 内容 hash（日志与 `/perception/status`），
T6 校对的是节点**实际加载**的 ID/hash，而不是自己再读一份源码树副本。

**Considered Options**：A（本决定）sensor_extrinsics.yaml 唯一真源——标定产物
与运行时输入同一文件（见下「运行时文件身份」），split-brain 在结构上不可能；
B runtime yaml 唯一真源——
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
「record == runtime 加载值（calibration_id/hash）——以节点实际加载路径的
值为准」。**T7 为 P0，置于 T1/T2/T3 之前执行**（该三票的安全语义仰赖
「读到的就是校验证过的」）。

**运行时文件身份（node 实际加载的路径，二轮 post-review P0-3 增补）**：
同一份 YAML 在仓库里存在两个物理位置——源码树 `config/sensor_extrinsics.yaml`
与 `setup.py` `data_files` 安装进 `install/.../share/robot_safecontrol_moveit/config/`
的副本；现有 launch 明确使用 `get_package_share_directory()` 解析的共享目录。
因此：

- **唯一权威路径**：运行时（launch 与 bridge 节点）一律经
  `get_package_share_directory('robot_safecontrol_moveit')/config/sensor_extrinsics.yaml`
  （ament_index_python 包索引，与 launch 现有解析一致）得到 **resolved
  绝对路径**；源码树路径只属于开发生成（T5 写入目标）与测试，不是运行时真源。
- **启动记录**：bridge/T7 启动时把 resolved 路径、`calibration_id`、
  内容 sha256 hash 写入日志与 `/perception/status`（可检索、可断言）。
- **T6 校对**：T6 比较「自检校验通过的 record」与「bridge 实际加载的 record」，
  以 bridge 上报的 path + hash + calibration_id 为准——不是再次读取源码树副本
  去比较，否则 source/install 分叉时 T6 必然误报通过。
- **T5 收尾**：T5 写入源码树文件后，必须**重新部署**（colcon build/install）
  并验证 `record == runtime 加载值`（经 ament 路径的 hash）——写文件本身不会
  生效，节点继续读旧 install 副本的风险是显式的、被检查的。
- **开发模式**：仅当节点以源码树方式运行（如测试直接 import）时不被视为违反
  ——但那不是生产路径；生产路径（launch）必须走 ament share 唯一路径。