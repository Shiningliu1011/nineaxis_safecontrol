# 九轴共同模型与 M07 限制

本模块对应 #153，采用现有 `J1`–`J9`、1P8R 九轴机构。参数依据为已经确认的 #136、#138 和 #141；跟踪目标采用现有蝴蝶曲线。入口为 [nineaxis_model.py](../../portable_oscbf/work/nineaxis_model.py)、[process_limits.py](../../portable_oscbf/work/process_limits.py) 和 [static_motion.py](../../portable_oscbf/work/static_motion.py)。

## 模型与单位

`NineaxisModel.load(root=...)` 读取 [模型配置](../../portable_oscbf/config/nineaxis_model.yaml)，核对 URDF、SRDF、生成关节链、原始 STL、封闭碰撞网格及其来源 hash，并在建立模型参数前启用 JAX float64。后续精度状态改变时 host 求值明确拒绝，纯 JAX 结果保存精度有效性。关节顺序固定为 `J1`–`J9`；计算输入为 `m_rad`，J1 使用 m，其余关节使用 rad。项目单位转换复用 `robot_units`。位置范围来自原 URDF，控制余量和初始 IK 余量复用 `joint_limit_contract`。

基坐标为 Y-up 的 `base_link`，J1 沿 `+Z` 移动。`fk` 返回 TCP 的齐次变换，`jacobian` 使用 `[linear; angular]` 行顺序。`link_poses` 同时提供各原始 link、`tool0` 和修正后的 `tcp`。`tool_axis_jacobian` 对应实际 TCP 局部 `+X` 工作轴。原 URDF 的 `tool0` 沿 `Link9` 局部 `+X` 延伸 235 mm。

当前模型 revision 为 `ninezzhou-m02-j1-origin85mm-v2`，J1 固定起点为 `0.085 m`，关节行程为 `0–0.5 m`。新坐标与原坐标满足 `q_old_m=0.085+q_new_m`；模型身份、生成关节链、封闭碰撞来源和几何制品随该定义更新。网格的局部几何保持来源身份，历史实验与参考关节配置保留原模型身份。旧标定须重新验证适用性。三组位置、两组其余关节姿态的参考数据与离线验证见[交接记录](../joint_motion_handoff_20261028.md)。

`RigidTransform` 明确来源和目标 frame、SE(3) 矩阵、平移单位 m、来源、版本、验证状态及残差。`ModelCalibration` 保存 `T_tool0_tcp`、可选 `T_base_workpiece` 和关节映射证据。两类记录都需要 keyword-only 的 `applicable_model_hash=model.asset_model_hash`。该身份由未应用标定的模型 revision、配置和已验证源资产产生，独立于标定结果。容器核对各份变换的适用身份；`load` 和 `with_calibration` 核对记录与目标模型，模型 revision 或基准工具配置变化时拒绝使用不同身份的记录。标定变化生成新的完整模型和几何身份，`asset_model_hash` 保持其源资产含义。`source_pose_to_base` 对源 `+Z` 轴表达应用固定旋转，再得到项目 TCP 的 `+X` 工作轴；工件变换作用于位置和姿态。

当前默认配置保存名义仿真 TCP。真实工件、关节设备映射、TCP 和附加工具几何的取得方式保持可见。标定检查只说明所列标定项目的状态，真实设备执行仍需要相应现场任务与系统准入证据。

## 实际工具运动

`tool_motion(q, q_dot, q_ddot, q_jerk, sample_source=..., time_s=..., clock_id=...)` 接受同一时刻、同一来源的九轴运动，返回不可修改的 `ToolMotion`。`sample_source` 明确区分 `simulation_feedback`、`measured_feedback` 和 `generated_motion`；实际观察拒绝使用 `generated_motion`。实际加速度和 jerk 的估计条件需要由反馈提供者记录，指令回送不能成为实际反馈。

`model.jax_parameters` 提供不可修改的共同关节链与修正 TCP 参数，供预测与 `tool_motion_jax` 共用。`tool_motion_jax` 提供固定形状的 `(4,4,4)` TCP 变换及前三阶时间导数，并返回独立的有限性状态。host 接口拒绝无效输入与结果。导数通过共同 JAX 运动学的方向导数取得，包含 TCP 修正的旋转和平移。返回值分别保存：

- TCP 位置、速度、加速度和 jerk，使用 m、m/s、m/s²、m/s³。
- 工具工作轴及其前三阶时间导数。
- 完整空间角速度、角加速度和角 jerk，使用 rad/s、rad/s²、rad/s³。

九轴跟踪任务约束三维位置和工具轴二维方向。任务 Jacobian 的秩为 5 时有四个局部冗余自由度；任务缩放、秩检查和控制使用由 M09 负责。绕工作轴的转动自由，完整角运动仍独立记录。运动分段连接处按实际段的有效导数或对应单侧值提供输入。

## 共同配置与绑定

[M07 配置](../../portable_oscbf/config/nineaxis_research_limits.yaml) 对每项保存 `quantity/object/frame/value_or_bounds/unit/phases/source/assumptions/verification_state/acquisition`。配置身份包含数值和来源。共同研究数组直接保存 J1 的 500/5000/62500 mm/s、mm/s²、mm/s³，J2–J6 的 120/120/600 °/s、°/s²、°/s³，以及 J7–J9 的 150/150/750。数值进入计算时只转换一次。

`ProcessLimits.load()` 拒绝未知字段、单位、版本、非有限值和错误数组。`bind` 绑定共同模型、最终路径 hash/总弧长、工件 hash、场景 hash、阶段和证据范围，生成 `BoundProcessLimits`。控制余量的代码来源 hash 也参与绑定身份。当前配置范围为 `research_simulation`，设备用途明确拒绝。

`ProcessObservation` 分别标记 `source_nominal`、`execution_reference` 和 `actual`，各自保存来源身份。输入包括同一时刻的有效有序投影、切向及其时间导数、工件法向、参考位置和工作轴、实际投影位置，以及由碰撞/工件几何提供者取得的间距。投影与工具运动的时间和时钟身份必须相同；参考进给同时匹配路径身份、最终弧长和时刻。

`BoundProcessLimits.measure` 分别计算切向进给、进给的一阶和二阶时间导数、Cartesian 切向投影、路径法向平面运动、工件法向速度、完整 XYZ 运动、工作轴和完整旋转运动。切向为 `T` 时，实际进给导数使用 `T_dot·v + T·a`，进给二阶导数使用 `T_ddot·v + 2*T_dot·a + T·j`。工具轴夹角使用 `atan2(norm(cross), dot)`。

`ReferenceFeed` 使用最终总弧长 L 和 `nu/nu_dot/nu_ddot`，分别计算 `L*nu`、`L*nu_dot`、`L*nu_ddot`、`kappa*v²` 和 `kappa²*v³`。原蝴蝶 80 mm/s 与 400 mm/s² 保留源名义身份；共同研究上限为 60 mm/s、100 mm/s²、300 mm/s² 和 16400 mm/s³。源名义报告也按共同上限核验，保留具体违反。

`check` 按各参数的 `phases` 检查当前样本的工具、跟踪与间距限制，返回数值、具体违反、必需缺项、未配置项、暂停信号和来源记录。间距只在指定阶段检查必需输入。`to_record` 可直接序列化。`tracking_quality` 汇总实际 cross-track 的 RMS/p95/max 和工具轴真实夹角的 RMS/max，并按对应参数阶段检查。关节运动与位置边界覆盖全部运行阶段；恢复、端点、总时限和源参数等具有固定职责，加载器拒绝未实现的阶段组合。外部输入的距离需要由 M04 保证对象覆盖、时间和计算有效性。

最低正加工进给、完整 XYZ 加速度/jerk、工具轴与完整角运动等额外数值上限可以在有来源的配置中提供，检查按声明阶段生效。工作距离容差、反馈停止、时序和五维奇异性完整定义仍等待相应任务；当前加载器保持这些字段为 `unconfigured`。既有位置 Jacobian 奇异值乘积及 0.005/0.05 阈值保存来源身份和实际指标；五维任务的缩放与秩检查按 M09 定义。请求缺失项的检查会返回具体未满足条件。最近曲面距离、法向距离和切向分量分别保存，S0 已包含的 50 mm 工作距离仅作为具有场景身份的来源记录。

## 阶段、运动生成与检查范围

`phase_status` 使用实际观察及 `PhaseEvidence` 处理恢复、端点和入场条件。适用阶段的暂停信号进入 `reasons`，并使 `can_continue=False`。恢复需要位置/轴向阈值、正向候选和连续确认；入场和恢复共用的位置/轴向阈值明确登记为 `entry/recovery` 两个阶段。端点需要完整有序覆盖、实际端点误差、停止及连续保持。暂停和恢复共用累计等待时间；完整运行累计时间覆盖全部阶段。状态维护者提供连续条件证据与累计时间，阶段切换继续保留这些时间。

`MotionPlanner.from_limits(bound, minimum_duration_s=...)` 将共同逐轴速度、加速度和 jerk 直接交给真实 Ruckig，候选同时包含连续停止运动和限制身份。`from_record` 拒绝不同身份。原 `jerk_time` 调用仍有效；明确逐轴 jerk 与 `jerk_time` 同时提供时拒绝。现有静态资源入口直接传递其 `execution_limits()["max_jerk"]`。

`check_joint_motion` 使用真实 Ruckig 的分段多项式范围，检查完整候选及停止区间的关节位置、速度、加速度和 jerk。工具 `check` 的结果属于指定样本的 M07 数值核验；整个时间区间的工具与几何证明由运动/参考协调和执行准入任务继续完成。当前静态实验配置保持其已有来源，三种正式比较方法随后通过对应接入任务消费共同 M07 对象。

## 安装与离线使用

Ubuntu 22.04 / Python 3.10 环境先安装控制内核的既有 requirements，并安装 `ros-humble-urdfdom-py` 与 `ros-humble-srdfdom`。`package.xml` 已声明两个解析库。`srdfdom` 的 Python 包由官方 CMake 的 `ament_python_install_package` 安装，其依赖 `urdfdom_py` 由[官方 2.0.7 清单](https://github.com/moveit/srdfdom/blob/2.0.7/package.xml)给出；构建入口见[官方 CMakeLists](https://github.com/moveit/srdfdom/blob/2.0.7/CMakeLists.txt)。

离线机器可以预先取得相应 Debian 包及系统依赖，使用 `apt-get install ./包文件.deb` 安装。当前实际验证的发行包为 `ros-humble-srdfdom 2.0.7-1jammy.20260908.002942` 和 `ros-humble-urdfdom-py 1.2.1-1jammy.20260907.220847`。其 SHA-256 分别为 `77248edb5ef4be31e91016b06a38e123ab0d01a71496f7c476aa587f9a123388` 和 `36df610a7cae1f2a97ea012bdeb91b461716ff4e0073fe9970768619b09f0370`。

独立 Python 进程需要将 `portable_oscbf`、URDF Python 包目录和 SRDF Python 包目录加入搜索路径。Humble 的后两项为 `/opt/ros/humble/lib/python3.10/site-packages` 和 `/opt/ros/humble/local/lib/python3.10/dist-packages`。模型默认从自身代码位置解析资源根目录；安装后的 package share 中使用相同结构。

本任务使用 `python -I` 和独立提取的两个 Debian 包完成加载、SRDF 读取与真实 FK，未加载系统 `/opt/ros` 路径，`rclpy/rospy/ament` 均未导入。命令、包 hash、完整 `sys.path` 和模块来源保存在任务验证记录中。

## 验证入口

- `portable_oscbf/tests/test_nineaxis_model.py`：真实 URDF/JAX/MuJoCo 逐轴 FK、各 link、Jacobian、工具轴，以及 TCP 修正后一至三阶运动的独立时间差分。
- `portable_oscbf/tests/test_process_limits.py`：参数来源/单位/身份、实际工艺定义、参考分层、缺项、阶段与累计期限。
- `portable_oscbf/tests/test_research_motion.py`：九轴逐轴真实 Ruckig、明确 jerk、完整停止和身份拒绝。

测试和仿真证据的范围保存在 Trellis 任务 `.trellis/tasks/10-27-map147-model-limits/implement.md`。设备能力、真实标定、正式周期与完整三方法结果由各自任务验收。
