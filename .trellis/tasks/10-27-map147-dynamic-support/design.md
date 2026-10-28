# 时间模型与接口

`LidarOccupancyScene` 提供不可变的原观测记录，保留各个 occupied voxel 的全部有效 hit 时间与测量范围。可选 `TrackingFrame` 具有 scene epoch、clock、policy、来源及内容身份，关联仅接受已存在的 support id。

每个原 support 在 snapshot 时间具有名义中心、范围、速度、范围增长率与加速度上界。有效 track 的更新以前使用原观测的未跟踪历史范围，历史年龄包含原采集时间误差与 track 更新时间误差，覆盖实际更新时间的完整区间。更新以后使用 track 速度、位置及速度误差与加速度范围。无效 track 使用原观测至查询时间的完整未跟踪范围。固定模型保持零速度和零增长。

合并模型采用成员中心与速度的平均值；初始范围包围全部成员，增长率包含各成员相对名义速度，二阶项采用成员最大加速度。三角不等式与单调二次范围保证完整未来区间的包围。固定形状的 `SupportTimeModel` 保存各成员和来源 hash，包含于 scene checksum，并绑定 scene epoch 与 clock。

对成员 i，snapshot 时刻记为 t0：`p_i(t)=p_i(t0)+v_i*(t-t0)`，`rho_i(t)=rho_i(t0)+r_i*(t-t0)+a_i*(t-t0)^2/2`。合并速度为全部成员速度的平均值，合并初始范围为 `max_i(norm(p_i-p_group)*1000+rho_i)`，增长率为 `max_i(norm(v_i-v_group)*1000+r_i)`，加速度为 `max_i(a_i)`。全部范围计算采用 float64 并向外舍入。成员原几何、测量误差、观测时间及 track 的来源 hash 可分别追踪。

`TrackingFrame` 仅适用于相同 scene epoch、clock、required space、policy、device 与 scenario。每个关联绑定当前原观测内容 hash；新增观测使旧关联失效，该 support 使用未跟踪范围。track 的位置误差、速度误差、加速度及时间误差必须具有独立验证来源。track 更新前的历史时间继续使用 parameter artifact 的未跟踪上界。

`QueryBatch` 增加可选逐行 int64 ns，旧调用保持当前查询行为。带时间数据的 scene 由共同求值函数产生各行位置、范围和增长率；同一结果用于 barrier 与毫米距离。时间差先作 int64 减法，再转换为 float64 秒。新数据适用于当前 layout 的 host monotonic 时间；共同仿真 clock 与从零开始的物理时间由 #154 协调，当前入口明确拒绝不支持的 clock。

`SupportMotionBounds.from_policy` 从实际不可变 `CollisionPolicy` 取得 policy 身份、未跟踪速度与加速度、采集时间误差、基本几何与测量范围及最大 occupied 年龄。`CollisionSafety` 保存自身配置生成的固定数组，输入场景无法提供或替换。未跟踪动态 source 的历史半径、增长率、加速度与期限分别与这些依据核验；group 再核验对全部成员的包含关系。tracked source 另核验由原观测时间和 SceneTracks 确定的历史半径与增长下限。

`reachable_support_tube` 提供完整时间区间的保守环境范围及有效 mask，显式接收实际 `CollisionPolicy`，缺少或身份不一致时 mask 无效。它不提供机器人运动证明。旧 scene 缺少时间数据时保持已有语义，明确未来时间查询拒绝。所有非 OK 结果清除准入 mask。

`FixedEnvironmentModel` 位于 `fixed_environment.py`，既有 lidar_occupancy 导入方式保留。模型含原始成员几何、来源与验证说明、policy、revision 和内容 hash，数组不可变，并以内容身份参与 JAX static PyTree。CollisionScene/PreparedScene 可选字段携带实际模型；host 检查完整模型身份，公共时间模型核验每个固定 source 的成员 hash、几何、policy 及 revision。零采集时间须同时具有该模型对应关系，动态来源保持正数原观测时间。OccupancySnapshot 与 CollisionPolicy 字段结构保持原样。

snapshot 在修改内部状态前生成仍有效的原观测副本，完成 tracking 类型、身份、关联和内容校验。输入通过以后才执行观测过期、内部时间与 revision 更新；被拒绝的请求不会改变既有有效状态。

实际固定模型的全部成员须由有效 source 完整表示，并由有效 group 包围。公共核验分别检查模型至 source 的完整包含关系和模型至 group 的几何包含关系；旧静态布局仍检查后者。已有 source 对 group 的系数上界检查继续保证完整未来时间范围。容量失败保留其明确状态，全部成功入口必须满足完整覆盖。

#108 未提交依赖通过 apply_patch 物化并记录主仓库 SHA-256。工作区 Git 基准不包含这些未提交文件。
