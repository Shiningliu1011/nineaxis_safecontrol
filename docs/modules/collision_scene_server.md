# collision_scene_server 与共享场景

`collision_scene_server` 是独立 ROS 进程，持有关节反馈历史、原始 mesh self-filter、射线占据、固定模型、tracking、动态 support、覆盖和场景健康。它通过 `SharedSceneWriter` 发布固定布局的双缓冲场景。每个消费者使用 `SharedSceneReader.refresh()` 准备本地 `PreparedScene`，控制查询直接使用该本地引用。

## 运行入口

安装需要项目已声明的 Python 依赖、ROS 2 Humble 和系统 `libatomic1`。共享传输要求 Linux `pidfd`、`flock`、`mmap` 以及对齐的 lock-free 64-bit 原子操作；平台不满足要求时启动失败。

```bash
source install/setup.bash
ros2 launch robot_safecontrol_moveit collision_scene_server.launch.py \
  source_manifest:=/absolute/path/qualified-scene.json \
  calibration_record:=/absolute/path/sensor_extrinsics.yaml \
  lidar_source_clock:=verified_lidar_clock \
  lidar_source_topic:=/livox/lidar
```

launch 同时启动 `perception_bridge` 的 `scene_server_mode` 和场景服务。此模式要求相机输入为空、LiDAR 输入明确、有效 CalibrationRecord 以及显式原始时间域。bridge 对 frame、XYZ 和逐点 FLOAT64 `timestamp` 进行检查，保留原 PointCloud2 的完整序列化字节、字段、数据行和时间。它不创建 FusionEngine、ESDF、tracks 或碰撞对象发布。

`/perception/timed_lidar` 使用 `UInt8MultiArray`，内容由已有 `msgpack` 编码，包含 schema 版本、原始时间域、CalibrationRecord 文件 hash 与 LiDAR calibration_id、bridge 实例身份、有效状态、原因和 ROS 原始 PointCloud2 字节。来源与点云位于同一条消息中。无效帧仍传递明确拒绝状态，使服务立即撤销旧场景。服务按 manifest 核对来源并恢复真实 PointCloud2。bridge 实例变化需要重启场景服务，取得新 epoch。

关节输入默认为 `/mujoco_joint_states`，使用公共 `state_stream_qos()`。只有真实仿真反馈或 encoder 反馈可以进入历史，关节顺序按 `DEFAULT_JOINT_NAMES` 转换。两种时间映射必须具有共同 run 和目标 monotonic 时钟，原始测量时间通过映射保留。

节点配置为 [collision_scene_server.yaml](../../config/collision_scene_server.yaml)。`source_manifest` 是必需输入；空值立即拒绝启动。`shared_path` 默认为 `/dev/shm/robot_safecontrol_collision_scene`，`health_period_s` 决定独立场景刷新周期。ROS 参数启动后只读。`package_portable_root()` 根据当前 Python 包的位置选择对应源码或安装资源；`portable_oscbf_root` 可明确指定内核路径。

## 来源 manifest

`load_scene_sources()` 使用 JSON schema 和生产对象构造器验证所有资源。相对路径以 manifest 所在目录为基础。字段如下：

| 字段 | 内容与校验 |
|---|---|
| `schema_version` | 固定为整数 `1`。未知字段拒绝。 |
| `parameter_artifact`、`device`、`scenario` | 有独立来源和验证记录的 CollisionParameterArtifact，适用范围必须匹配。 |
| `geometry_artifact`、`kernel_version` | 完整碰撞 geometry artifact 与 32-byte kernel 身份的十六进制文本；由 CollisionSafety 检查内容、policy 和对象对。 |
| `ray_geometry` | `urdf` 与 `package_roots`，或具有 `link/path` 的 `link_meshes` 与 `source_id`；同时必须提供 GeometryValidation 的完整字段。 |
| `calibration_record` | 与 bridge 相同的 CalibrationRecord。完整文件 hash、frame 和 admission 状态参与核验。 |
| `lidar_clock_mapping`、`joint_clock_mapping` | ClockMapping 的全部字段，包括锚点、有效范围、误差、证据、run 和目标时钟。 |
| `grid` | `origin_m`、三维 `shape`、`required_mask`；mask 使用 bool NPY 文件，`null` 表示全部网格均为必要空间。voxel 大小来自参数 artifact。 |
| `history` | `feedback_source`、`zero_calibration_id`、`capacity`、九维 `joint_speed_bound_si`、`maximum_gap_ns` 和 `evidence_id`。 |
| `max_range_m` | 有效 LiDAR 测量范围，单位 m。 |
| `fixed_environment` | `null`，或 `mesh_path`、`mesh_sha256`、`model_revision`、`validation_evidence`、`error_mm`；使用真实 mesh 生成 FixedEnvironmentModel。 |

原始 mesh 必须经过封闭正体积检查并具备匹配的验证身份。固定模型保存来源、模型 hash、完整 support 与需求数量。运行时 tracking 可以通过管线的 `update_tracking(TrackingFrame)` 提供，关联必须绑定原 occupied 观测；未提供或失效的 track 使用已有的未跟踪物理范围。完整字段定义位于 `collision_scene_sources.SOURCE_SCHEMA`、`ClockMapping` 和 `GeometryValidation`。

`tests/test_collision_scene_server.py` 的 `write_analytic_manifest()` 提供可运行的解析接口数据和完整 manifest，真实进程测试直接消费这些文件。该数据证明软件接口、几何分类和进程传输。生产参数、现场几何与时钟资格分别需要对应测量和验证来源；当前仓库的未准入记录不会自动成为有效来源。

## 固定布局与发布

共享文件由一个发布字、不可变 header 和两个相同大小的 slot 组成。NumPy structured dtype 定义全部固定字段与数组形状。浮点数据全部转为 float64；support、track 和 source member 容量来自参数 artifact。固定模型最多保存 `max_support_points * support_merge_max_points` 个原始 support，验证证据文本上限为 4096 个 UTF-8 bytes。超出布局容量立即拒绝。

header 保存传输版本、布局 hash、公共资源路径 hash、来源身份、容量、slot 大小、epoch、writer PID、创建时间及独立 SHA-256。来源身份由实际加载的完整 CalibrationRecord 内容身份、标定矩阵与 frame、原始 ray 几何及验证、SelfFilterBounds、两种 ClockMapping 全部字段与 run、反馈来源、零位标定、历史容量、测量范围、policy、grid 和固定模型计算。manifest 的路径和 JSON 排列不参与身份。slot 包含全部 CollisionScene 字段、SceneMetadata、SceneTracks、CollisionIdentities、完整 SupportTimeModel 和 FixedEnvironmentModel 来源。slot SHA-256 保护完整固定布局字节；场景自身 checksum 同时校验。没有相应 payload 的可选对象使用明确标志。

writer 通过稳定的 `.writer.lock` 和 `flock(LOCK_EX | LOCK_NB)` 获得唯一写入权。已有目标必须属于本用户，且文件类型、大小、协议 header、资源与布局身份和 checksum 完整；无法确认时保留原文件并拒绝启动。它创建新的共享文件并原子替换已确认的公共路径，每次启动生成新 UUID epoch。启用 slot 保持不可变；只有未启用 slot 可以写入。输入来源校验、float64 转换、完整场景检查、容量和 checksum 全部通过后，writer 在发布前再次检查时间与覆盖条件。场景准备时间必须属于当前 writer 生命周期。

发布字同时编码递增的 56-bit revision、健康状态和 slot index。通过系统 `libatomic` 执行 release store 和 acquire load，并在运行时核验 8-byte 对齐及 lock-free。Python 或 NumPy 普通赋值不承担原子发布职责。完整 revision 能检测两次切换后返回同一 slot 的情况。语义依据为 [GCC atomic builtins](https://gcc.gnu.org/onlinedocs/gcc/_005f_005fatomic-Builtins.html) 和 [Python mmap](https://docs.python.org/3.10/library/mmap.html)。

失败准备、覆盖未知、容量超限和关闭会原子发布对应非 OK 健康状态，消费者立即撤销本地引用。场景和错误状态都推进 revision。正常关闭保留最后的不可用共享文件与稳定 lock 文件，供现有 reader 明确识别退出；后续 writer 获得锁后创建新 epoch。进程异常退出由 reader 的真实 `pidfd` 检测。

## 消费与恢复

`/collision_scene/revision` 使用可靠、depth 1、transient-local 的 `String`，内容是小于 600 UTF-8 bytes 的 JSON：epoch、revision、采集时间、准备时间、active slot、layout hash、来源身份与状态。大数组保存在共享内存中。

```python
from robot_safecontrol_moveit.oscbf_trajectory import bootstrap_portable, package_portable_root

bootstrap_portable(package_portable_root())

from robot_safecontrol_moveit.collision_scene_sources import load_scene_sources
from work.collision_safety import CollisionStatus

sources = load_scene_sources("/absolute/path/qualified-scene.json")
reader = sources.create_reader(
    "/dev/shm/robot_safecontrol_collision_scene",
    slot_timeout_ns=qualified_slot_timeout_ns,
)

# 此路径由独立刷新任务执行。
result = reader.refresh()
if result.status == CollisionStatus.OK:
    local_scene = result.prepared_scene

# 控制查询只消费已经交接的本地对象。
result = sources.safety.query(query_batch, local_scene, query_mode)
```

reader 使用不等待其他端点的文件打开方式，在映射前确认普通文件类型，之后仅使用 `mmap.ACCESS_READ`。`refresh()` 校验文件身份与大小、不可变 header、布局、writer 存活、全部静态身份、scene 与 slot checksum、期限、覆盖和模型来源。`sources.create_reader()` 提供本地加载来源的预期身份，并与 header 及通知严格比较。直接调用通用 `SharedSceneReader` 时，生产场景必须明确传入相同的 `source_identity`；省略参数仅允许读取同样未绑定来源的通用传输。复制前后以及真实 JAX device 准备完成后都检查完整发布字，任何切换均拒绝该副本。每个有效 revision 只进行一次真实 `prepare_scene` device 准备；重复刷新复用相同 PreparedScene，并重新验证健康和时间。

`reader.prepared_scene` 和 `reader.result` 只读取本地状态，不读取共享内存、不等待跨进程锁。发现新 revision 后，复制和 device 准备开始前立即清除公开旧引用并设置 `REVISION_PENDING`。调用方必须持续运行独立刷新路径，并以最近的 OK 结果交接本地引用；失败后停止使用此前保存的引用。`reader.device_transfer_count` 提供真实准备次数诊断。

首次加入可以读取当前有效 revision。后续 revision 跳跃、损坏、超时或 writer 退出会锁存故障并清除本地准入。排除原因后，由外部监控调用 `reset_fault()`，之后仍须完成实际刷新。epoch 改变返回 `REVISION_PENDING`，由调用方完成恢复检查并明确调用 `accept_epoch(new_epoch)`；随后重新准备当前场景。恢复操作本身不返回有效场景。

## 验证

```bash
python -m pytest portable_oscbf/tests/test_shared_collision_scene.py \
  tests/test_collision_scene_server.py tests/test_timed_lidar_transport.py \
  tests/test_lidar_self_filter_adapter.py \
  --basetemp .scratch/scene-server-validation/pytest -q
```

用例使用独立 writer、真实 `/dev/shm`、系统原子操作、真实 ROS bridge/server 进程和 JAX。覆盖唯一写入、完整来源恢复、只读映射、checksum、期限、revision 跳跃、复制期间切换、返回同一 slot、正常及异常退出、epoch 恢复与每 revision 单次 device 准备。ROS 进程验收比较原始 PointCloud2 序列化字节，并验证无效原始输入立即撤销缓存。

正式控制消费者接入、固定真实 LiDAR 回放、设备参数测量及目标设备性能分别由对应任务验收。现有 sim、shadow、live 准入条件继续适用。
