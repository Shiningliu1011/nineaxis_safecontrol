## 安全约束与准入

**碰撞安全模块**:
规划、轨迹执行与 OSCBF 在各自进程内共同加载的无 ROS JAX 碰撞判定来源；它持有统一
机器人碰撞几何与 `CollisionScene`，并返回尺度、屏障值、关节梯度、危险对象身份和查询状态。
_Avoid_: 把规划碰撞检查、轨迹复查和 OSCBF 碰撞约束称为三套独立检测器

**PreparedScene**:
`CollisionSafety.prepare_scene` 完成 identity、容量、时间、覆盖与几何检查后返回的不可变
场景；只有 PreparedScene 可以进入状态查询与区间证明。
_Avoid_: 原始点云、未经验证的 CollisionScene

**PreparedScene 发布**:
感知工作线程完成整个不可变场景后，通过原子引用一次替换控制线程可见版本；控制线程不能
读取构造中的场景，也不能等待场景构造。
跨进程发布使用固定布局的双缓冲共享内存；ROS 只传递 revision、时间、布局身份与健康状态。
读取进程每个 revision 检查 identity 与 checksum，并只向本地 JAX device buffer 复制一次。
_Avoid_: 原地修改 scene、控制线程处理 LiDAR、用 ROS 传递完整 PreparedScene

**碰撞场景服务（collision_scene_server）**:
CollisionScene、`scene_epoch`、`scene_revision` 与共享内存 slot 的唯一写入进程；负责 mesh
self-filter、射线占据、固定环境合并、tracking、support 生成和场景验证。控制器、规划器与
轨迹验证器只读取它发布的不可变场景。
_Avoid_: 多个进程各自构造环境场景、控制器拥有占据状态

**碰撞计算精度**:
LiDAR 原始坐标可以使用 `float32` 传入；`prepare_scene` 将参与碰撞计算的数据统一转换为
`float64`。DCOL、point-scale、barrier、梯度、区间证明、距离查询与 QP 输入均保持
`float64`，该策略属于 `kernel_version`。
_Avoid_: Module 内部混合精度、运行期间降低精度

**组合 ellipsoid 包络**:
每个机器人 link 由固定容量槽位中的一个或多个 ellipsoid 共同覆盖；`active_mask`
标识有效槽位，完整包络具有稳定的 `geometry_hash`。
_Avoid_: 单 ellipsoid link、球体集合、OBB 包络

**Collision geometry artifact**:
由 closed collision mesh 自动生成并通过四面体覆盖证明与运行时间验证的组合 ellipsoid
数据，包含 mesh hash、固定槽位、覆盖证明和 `geometry_hash`。
_Avoid_: 手工参数列表、未经证明的拟合结果

**精确 ellipsoid DCOL**:
直接使用两个 ellipsoid scaling functions 求最小线性 `alpha_star` 的原生 JAX 查询；结果
包含梯度、残差、迭代次数和求解状态。
_Avoid_: 42 面 polytope 近似、米制 OBB distance

**允许接触列表**:
碰撞安全模块中唯一允许跳过检查的 link pair 集合；每项都记录允许原因、适用几何身份
和验证依据，其余完整 link pair 默认进入自碰撞查询。
_Avoid_: 固定 14 pair、隐式忽略列表

**自碰撞 primitive 行**:
一个有效 ellipsoid pair 对应的一条独立 DCOL barrier 约束；link pair 最小值只用于
诊断展示，不替代各 primitive 的梯度与安全判定。
_Avoid_: link pair 聚合行、全身最小碰撞行

**环境活动约束集合**:
每个 robot ellipsoid 当前最危险且必须进入 OSCBF 的固定容量 support pair 集合；每个
pair 保留独立 barrier 行，容量不足时场景进入 `CONSTRAINT_OVERFLOW`。
_Avoid_: 每个 robot ellipsoid 一个聚合行、无证明省略近场 point

**碰撞不可松弛行**:
命令 QP 中不能使用 slack 放宽的 self collision 或 environment collision CBF 行；任何
冲突、求解失败或无效梯度都不能产生被准入的运动命令。
_Avoid_: relaxed collision row、用诊断 slack 授权命令

**碰撞区间证明**:
对关节运动区间计算 `proximity_scale` 下界的验证结果；无法直接证明时递归二分，计算
预算耗尽仍未获得证明的区间不能用于规划或轨迹执行。
_Avoid_: 端点安全、固定步长采样

**规划场景准入**:
规划结果在最新 `CollisionScene` 上通过整条路径复查并取得 `admitted_revision` 的状态；
只有已准入路径才能进入轨迹执行。
_Avoid_: 规划完成、旧 revision 路径

**重新规划触发**:
剩余名义轨迹出现未通过区间、覆盖不足、动态预测冲突或 `remaining_margin_mm` 低于阈值时
产生的重新规划请求；无关 revision 更新不触发重新规划。
_Avoid_: 每次 LiDAR 更新都重新规划、只在碰撞后重新规划

**碰撞临时等待**:
revision 更新、前视验证或重新规划尚未完成时的保持状态；取得当前场景准入后可以自动继续。
_Avoid_: collision fault、人工恢复状态

**碰撞故障锁存**:
标定、时间、覆盖、容量、碰撞 solver 或命令 QP 已失去安全依据后的保持状态；条件恢复、
当前状态与轨迹重新验证并收到人工确认后才能继续。
_Avoid_: revision pending、条件恢复后自动继续

**受限恢复模式**:
安全间距已经越界、几何仍然分离且全部碰撞依据有效时，经过人工确认执行的低速恢复轨迹；
每个证明区间都让全部已违反 barrier 严格单调增加，并保持其余碰撞约束。
_Avoid_: 物理接触恢复、unknown 状态运动、自动恢复原任务

**碰撞场景（CollisionScene）**:
由 LiDAR 射线更新形成的带版本局部占据快照，包含 `scene_revision`、采集时间、
`occupied/free/unknown`、覆盖状态、误差范围、有效槽位和几何身份。
_Avoid_: 点云数组、障碍物列表

**碰撞内核身份（kernel_version）**:
碰撞查询算法、数值设置和接口版本的稳定身份；与 `geometry_hash`、`scene_revision`
共同证明不同运行进程使用相同算法、机器人几何和环境输入。
_Avoid_: ROS 节点版本、程序启动时间

**邻近缩放量（proximity_scale）**:
当前几何达到接触所需的无量纲线性缩放量；大于一表示分离，等于一表示接触，
小于一表示相交。自碰撞由 ellipsoid–ellipsoid DCOL 给出，环境碰撞由
ellipsoid–point scale 给出。
_Avoid_: distance、clearance、CBF alpha

**缩放安全阈值（scale_margin）**:
邻近缩放量必须达到的下界，数值不直接表示米制距离；传感器误差、体素范围和时间误差
需要通过经过验证的几何范围进入查询。
_Avoid_: d_safe、cbf_gain、metric_inflation_m

**碰撞间距（clearance_mm）**:
机器人碰撞包络与外部 occupied 几何包络之间的保守欧氏间距，公开配置、查询与诊断统一
使用毫米；JAX 内部在 facade 边界转换为米。
_Avoid_: proximity_scale、原始 LiDAR range、无单位 distance

**外部距离查询**:
返回机器人组合 ellipsoid 与外部 occupied 保守包络之间的最小有符号 `distance_mm`、要求
间距、剩余余量、最近 pair 和场景身份；用于规划代价与诊断，不授权 OSCBF 安全状态。
_Avoid_: 用 point-scale 直接标为毫米、用距离查询替代 environment barrier

**动态 support 安全模型**:
已跟踪 occupied support 的名义速度进入 point-scale barrier 的 `partial_h_partial_t`，运动估计
误差、加速度范围与延迟进入 `beta` 及 `beta_dot`。规划与区间证明使用 time-indexed reachable
support tube；tracking 失效后改用未跟踪运动上界。
_Avoid_: 用 tracking 删除 occupancy、只按常速度外推未来位置

**碰撞 CBF 响应率（cbf_rate）**:
一阶 CBF 条件中乘以 barrier 的固定时间响应参数，单位为 `s^-1`。自碰撞与环境碰撞分别使用
`self_collision_cbf_rate_s_inv` 和 `environment_cbf_rate_s_inv`；动态障碍物运动通过
`partial_h_partial_t` 表达。
_Avoid_: CBF alpha、运行期间按 pair 改变响应率、DCOL proximity_scale

**碰撞查询状态**:
`query()` 与 `certify()` typed result 的公共枚举状态；只有 `OK` 结果及其有效 mask 可以进入
命令 QP、规划边接受或轨迹准入。其余状态的数值只用于诊断。
_Avoid_: 用 NaN、inf、负距离或全零梯度表示失败

**碰撞切换证据门**:
新 CollisionSafety Module 取得命令权限前必须通过离线回放、MuJoCo、shadow、独立几何参考、
覆盖证明、区间证明、故障注入与 deadline 标准。通过后新 Module 成为唯一判定来源，旧运行
路径与 fallback 被移除。
_Avoid_: 把现有 OBB 或球体输出当作正确性参考、生产双实现切换

**碰撞验收证据 manifest**:
记录几何 kernel、mesh 覆盖、真实 LiDAR 回放、规划与 OSCBF、故障注入、目标设备运行时间和
独立验收数据结果的版本化文件；绑定代码、配置、geometry、kernel、数据与设备 identity。
_Avoid_: 只记录测试是否通过、验收期间修改阈值

**碰撞参数 artifact（CollisionParameterArtifact）**:
由离线测量与验证生成的不可变参数集合，包含误差范围、延迟、间距、CBF 响应率、固定容量、
solver 限制与 deadline；通过 `parameter_hash` 参与 `collision_policy_hash`。
_Avoid_: 运行期间参数覆盖、在线调整安全范围、无数据来源的常量

**安全裕度**:
每条安全约束当前还剩多少余量，是屏障函数的自变量；关节限位行以 rad 计，统一碰撞
模块的自碰撞与环境行使用各自经过论证的无量纲 barrier。
_Avoid_: 用 d_safe 泛指裕度；把裕度为负当作故障

**安全阈值**:
把安全裕度与「够不够」相比的判据，按约束类各自定义——关节限位、自碰撞、障碍三者的物理含义不同，不互为同义词。
_Avoid_: margin、d_safe（作为三类阈值的统称）

**可行 / 不可行**:
可行指存在至少一组关节速度同时满足全部安全约束与执行器速度上限，不可行指不存在；安全裕度为负不等于不可行。
_Avoid_: 把「裕度为负」「求解器未收敛」当作不可行

**弹性松弛**:
弹性 QP 为每条安全约束引入的非负变量，加在该约束的上界上使问题恒有解；其量纲是约束变化率，除以该类增益才换回裕度单位。花掉松弛即放弃安全要求，不是数值误差。
碰撞安全模块的命令 QP 禁止对碰撞行使用弹性松弛。
_Avoid_: 把松弛当作长度、用碰撞 slack 授权命令

**准入检查**:
求解之后把该 tick 放弃的安全要求量与该类约束的允许额度相比，超限则该命令不被采信，进入冻结/锁存路径。
_Avoid_: 健康门（现状只检查解有限与求解器收敛）
