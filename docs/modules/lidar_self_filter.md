# LiDAR 逐点时间 mesh ray self-filter

`portable_oscbf/work/lidar_self_filter.py` 提供无 ROS 的原始 collision mesh 读取、共享 FK、逐点反馈插值、首交点和误差分类。`src/robot_safecontrol_moveit/lidar_self_filter_adapter.py` 读取真实 PointCloud2、JointState 与 CalibrationRecord，并把结果交给 [射线占据场景](lidar_occupancy.md)。实现范围来自任务 #105 与 [ADR 0028](../adr/0028-mesh-ray-lidar-self-filter.md)；[ADR 0038](../adr/0038-dedicated-collision-scene-server.md) 规定单个场景 owner 管理历史、更新、revision 和发布。

## 几何与误差来源

`RobotRayGeometry.from_urdf(path, package_roots=...)` 使用 urdf_parser_py 解析原 URDF，以 trimesh 原始三角面读取 collision 文件，应用 mesh scale 和 collision origin，再通过 `NineaxisKinematics.link_poses` 转换射线。原文件、URDF、collision 变换、共享 FK 源码、生成运动链和后端版本共同进入 `geometry_hash`。URDF 运动链必须与当前共享 FK 一致；tool0 使用既有生成器规定的 ee_link 名称。

局部 mesh 查询树供全部点复用。相同关节坐标的 FK 只计算一次，每条射线在对应点时刻的各个 link 坐标系查询，取全部原三角面的最近正向交点。基座和工具 mesh 都参与查询。`verify_identity` 检查源文件和内部三角面，内容发生改变时直接报告错误。必须安装 `trimesh==5.1.0`、`rtree==1.4.1` 和 `urdf-parser-py==0.0.4`；必需依赖缺少时立即报告错误。

`GeometryValidation` 绑定完整 geometry hash、`surface_error_mm`、证据身份与完整性状态。此误差必须证明实体内外两侧的有符号距离范围：真实实体位于模型外扩范围内，模型内缩范围位于真实实体内；工具和附件也须包含在完整性证明中。可准入几何还必须具有封闭正体积表面。当前原九轴包含 10 个 mesh、696330 个三角面，原始 STL 的表面完整性尚未取得准入证据；当前 `sensor_extrinsics.yaml` 的 LiDAR 也保持未标定状态。读取这些真实资源可以核验来源和数学查询，不能因此取得物理准入。

`SelfFilterBounds` 的全部数值均需明确提供，不能把缺少的物理误差补为零。`from_collision_policy` 复用现有 CollisionPolicy 的距离、入射、角度、外参、逐点时间与关节插值误差，另行要求逐轴反馈速度界、最大插值区间及其证据。长度使用 m 或字段明确注明的 mm，转动关节使用 rad，时间使用整数 ns。完整参数定义见 [单位约定](../units.md) 和 [参数 artifact](portable-oscbf/collision-parameter-artifact.md)。

## 时间与反馈

`ClockMapping` 保存源时间域、整数源锚点、对应 monotonic 锚点、源有效区间、offset 与速率误差、证据、run_id 和 target_clock_id。映射身份覆盖全部字段。整数时间按相邻锚点计算；FLOAT64 原始时间额外计入半个浮点间距及整数舍入误差。回跳、区间外时间和溢出立即报告错误。当前 driver 的 FLOAT64 timestamp 字段需要调用者提供有证据的原始时间域及映射，不能直接当作 monotonic 时间。

`TimedLidarScan` 保留所有原始行号、原始时间、规范化采集时间和时间误差、原始 frame、接收时刻、最大观测距离，以及 run/clock/map 身份。`JointStateHistory` 保存严格递增的反馈采集时间、逐轴 SI 坐标、时间误差、simulation/encoder 来源、模型、零位、运行和时间映射身份。LiDAR 与关节历史必须属于同一次运行和同一目标时钟。完整逐点时间不确定区间必须由有效历史包围，使用相邻样本插值；不进行外推。过大历史间隔、超过来源速度界、模型身份变化或误差超限均生成明确无效结果。

时间误差乘以逐轴速度界，再加有来源的插值误差，构成关节坐标范围。J1 的位移范围与全部转动关节的保守力臂范围、mesh 表面误差、外参误差及射线角误差共同形成几何位移界。`SelfFilterResult.error_sources` 保留各项来源身份及完整 scan/history 内容 hash，`filter_identity` 同时绑定实际数值、验证状态和时钟身份。几何公开身份只读；新的验证记录通过 `with_validation` 建立独立对象。

## 分类与射线区间

`MeshRaySelfFilter.classify(scan, history)` 返回每个点的分类、首交点 measured 标志、link、`first_hit_part_index`、triangle、名义距离、保守距离区间、可见 free 前缀、unknown 区间与误差半径。同一 link 的多个 collision 元素由 part index 区分；未测量距离始终伴随独立 measured/valid 字段。

| 分类 | 确认条件与占据含义 |
|---|---|
| environment | 测量距离上界位于机器人可能首交点之前；真实环境返回保留 occupied，机器人表面及后方仍保持 unknown |
| self | 完整误差界支持精确机器人表面的一致性；表面及后方保持 unknown |
| occluded | 有完整机器人相交证明，测量下界位于首交点上界之后；表面及后方保持 unknown |
| ambiguous | 测量或模型区间重叠、传感器位于机器人内部/表面附近或其他证据不足；相关空间撤销覆盖 |

无误差的解析几何直接使用原三角面首交点。存在几何位移误差时，扩展局部 bounds 提供可能首交点的保守下界；该 bounds 只用于限制可见空间。完整相交上界来自封闭 mesh 的内部见证点：点到全部表面的距离必须大于完整几何位移界。缺少这种证明时不会认定后方空间可见。

非零测量区间与机器人表面重叠时保持 ambiguous。仅凭一个容差内的距离匹配不能删除贴近本体的环境点。正常解析表面的 self 验证采用有明确解析来源的零误差输入；真实设备仍需对应误差与几何验证。

## 占据更新与失败行为

`SelfFilterResult.to_ray_batch()` 输出带身份的 `LidarRayBatch`，其中扩展字段为 `visible_free_until_m`、`unknown_start_m`、`unknown_end_m`、`unknown_radius_mm`、`input_valid`、`filter_identity` 和 `invalid_reason`。原环境 HIT 规则继续工作；self/occluded 提供明确可见前缀和后方 unknown，ambiguous 提供不确定空间。

占据 owner 对完整 unknown 区间及相交 voxel 撤销旧 free，保留仍有效的 occupied。同批 unknown 优先于 free。首次绑定过滤身份时撤销已有无过滤身份的 free；模型、标定、误差、run 或 clock/map 身份改变后，当前生产者持续拒绝准入，需要场景 owner 建立新的场景实例和 epoch。无效输入会通过真实 `prepare_scene` 返回 `INVALID_SCENE`；必要空间存在 unknown/ambiguous 时返回 `UNKNOWN_REQUIRED_SPACE`，对应有效 mask 全部为 False。

生产者公开 `invalidate(now_ns=..., reason=...)`，供输入解码或资源失败时立即撤销旧覆盖。主包公共入口 `update_scene_from_pointcloud` 在完整解码、分类和占据处理边界处理所有 `Exception`，调用失效操作后继续抛出原始异常，并保留异常参数和 traceback；调用者能够记录具体来源，旧 free 同时失效。第三方解析库报告的异常遵循同样规则。此入口由唯一场景 owner 顺序调用，不创建 node、socket、控制器或第二份场景。

```python
from robot_safecontrol_moveit.lidar_self_filter_adapter import update_scene_from_pointcloud

result = update_scene_from_pointcloud(
    cloud, mapping=lidar_clock_mapping, scan_id=scan_id,
    received_monotonic_ns=arrival_ns, now_ns=processing_ns,
    max_range_m=qualified_range_m, filter_module=self_filter,
    history=joint_history, producer=occupancy_scene,
)
snapshot = occupancy_scene.snapshot(now_ns=processing_ns)
prepared = collision_safety.prepare_scene(snapshot.scene, snapshot.scene.identities)
```

PointCloud2 使用 sensor_msgs_py 成熟解码接口，保留当前 driver offset=18 的未对齐 FLOAT64 timestamp 字段；JointState 按公共 DEFAULT_JOINT_NAMES 映射。当前 sensor_msgs_py 版本不支持的行 padding 会明确拒绝。CalibrationRecord 的 file hash、calibration_id、frame 和 admission_ready 状态进入过滤输入。

## 验证与接入

在项目依赖已经安装、ROS 2 消息可导入的隔离环境中运行：

```bash
python -m pytest portable_oscbf/tests/test_lidar_self_filter.py \
  tests/test_lidar_self_filter_adapter.py -q \
  --basetemp .scratch/lidar-self-filter-validation/pytest
```

测试覆盖真实原始 URDF/STL、独立解析盒体、间隔几何、全部 link 首交点、origin/scale、逐点运动、时间误差、反馈缺失、原资源变更、传感器位于本体内部、近表面障碍、unknown 撤销及真实 `prepare_scene`。`tests/data/lidar_self_filter/analytic_scan_v1.json` 是明确标记的数学接口回放，保留原时间、原点、标定、反馈、身份与独立预期，并通过真实消息解码和完整场景更新。它没有提供真实 LiDAR、现场几何或物理停车能力证据。

[独立场景服务](collision_scene_server.md)组合此入口与占据、固定模型、动态 support 和共享发布；`perception_bridge` 的场景转发模式保留完整逐点数据与来源身份。固定真实 LiDAR 跨进程回放、设备参数和生产控制切换分别由对应任务继续验收。当前生产控制链与硬件模式维持已有准入条件。
