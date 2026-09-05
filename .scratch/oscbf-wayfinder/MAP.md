# OSCBF Wayfinder Map — 从当前仓库到可验证的 9DOF LiDAR+depth OSCBF 系统

**生成方式:** 基于代码逐行核对、launch/配置实测、全量测试重跑(M12 报告只作对照,
不当事实)。本文档只记录「已实现且能运行」的事实与「真正未知」的决策;能明确表达的
未知已建 ticket(见 issues/),需用户拍板的列在文末讨论项,数据/物理上无法精确
定义的标 Not yet specified。

---

## 0. 已实现且验证的运行闭环(实测)

```
[controller] oscbf_controller (100Hz)
   ├─ 订阅 /mujoco_joint_states → 内部自积分 q → path_tracking_step (JIT)
   ├─ 发布 /oscbf_command (9 关节位置, τ=0.02s 低通)   ← 与 replay 双写竞争
[plant]     oscbf_plant: S 曲线 jerk 限幅 + kp=80 位置环 → /mujoco_joint_states
[perception] perception_bridge → /perception/tracks (8×10 球槽) → controller 障碍约束
[transition] TransitionExecutor + AEB-RRT* (MoveIt OMPL C++ 插件) + joint_state_replay
             → /oscbf_controller/start_tracking 服务交接
[display]   MuJoCo viewer: 仅显示,无物理仿真 in loop
```

**关键事实(与 README/规格的差距):**
- MuJoCo viewer 只是显示,闭环里没有物理仿真(plant 是纯几何积分模型)。
- `hardware_bridge` 无条件启动但不读任何 launch 参数;backend 从不注入 → 恒为
  shadow 且**不发状态**;SocketCAN 后端 `NotImplementedError`。真机链路从未验证。
- 默认 launch: `oscbf_wait_for_start=false` + `notify_oscbf_start=true` → 交接退化,
  controller 从 t=0 与 replay 双写 /oscbf_command;settle 超时(5s)仅告警继续。
- ESDF(32 球)只在 perception_demo 消费,生产控制器启用 `enable_sdf=false`。
- 跟踪语义是**弧长路径跟随 + 进给率调度**,不是时间参数化复现（06A 已定案,
  详见 issues/06a；修正:所谓"条件前馈"是死代码(ff_scale 未使用),
  `maximum_reference_feedrate_step_m_s` 同理为死参数）。

**M12 验收对照(2026-08-13):** 无障碍基线最大位置误差 0.0352mm、姿态 0.00684°、
QP 失败 0(旧配置 kp_pos=60/lead=1e-5);JIT 预热 28.75s;p95 6.177ms。
**本次实测(2026-09-04):** 新配置(kp_pos=160/lead=0.01/tool_axis_5d)下
**p95=19.367ms > 10ms 预算**——性能回归(见 ticket 01A)。

---

## 1. 现状 vs 未知决策(12 类)

### 1.1 当前代码真实状态
- **已实现:** 上表闭环 + 全量测试(见 §2)+ M12 无障碍验收报告。
- **未知:*"—" 本身是已知项;真正的未知是 1.5/1.6/1.9 等决策。
- **结论:** `README.md / CONTEXT.md / CLAUDE.md` 的表述(如"感知融合已就绪"
  "真机可下")与真实状态有出入——按代码为准。

### 1.2 动力学/运动学假设
- **已实现:** POE FK/Jacobian(nineaxis_manipulator_jax.py);质量矩阵为
  硬编码对角占位且**未使用**(当前为速度级 P-only,无动力学层);plant 侧
  S 曲线+kp 位置环模拟执行器。
- **未知决策 [DISCUSS-F]:** 是否需要动力学级控制(加速度级 OSC / 力矩分配)
  还是速度级足够(取决于跟踪精度与真机执行器模型)。
- **Not yet specified:** 连杆质量/惯量/重心参数来源(URDF 未核对);关节摩擦
  /重力补偿是否需要(无动力学时靠 QP 自然抵抗,位置环有 kp=80)。
- **ticket:** #12(几何/惯量数据核对)。

### 1.3 OSC nominal controller
- **已实现:** 速度级 P-only OSC;`task_mode: tool_axis_5d`(5D 工具轴,参考
  = 路径点,工具 X 轴对准圆柱轴心,`orientation_mode: surface_normal`);
  kp_pos=160 / kp_orient=10 / damping=0.05 / w_pos=40 / w_orient=10 /
  w_joint=0.1 / temporal_lambda=0.2;`use_nullspace_policy: false`。
- **未知决策 [DISCUSS-F]:** 零空间策略是否启用(冗余利用方式,ticket #07A/#07B 调研);
  5D vs 6D 任务(roll 是否放弃,取决于加工/演示需求);P-only 是否够(1.2)。
- **tickets:** #06A/#06B(语义刻画+拍板)、#07A/#07B(冗余策略)。

### 1.4 CBF formulation
- **已实现:** 速度级 CBF;h_1 = 距离-裕度,`alpha(h)=baseline*h`;
  有冲突注释(h_2 标注相对度 2,但 f=0/g=I——实际同为度 1 形式);
  行序=关节限位18 + 自碰撞14 + 障碍(10×8 槽或聚合10) + ESDF32(可选) + 奇异性1;
  弹性 QP(default)与 hard rate-slack(2 slack 增广)两种汇编;
  `apply_qp_health_gate` 失败→零速。
- **未知决策 [DISCUSS]:** 相对度与高阶 CBF 是否需要(速度级下度 1 即安全,但
  对"限位/速率"的表示在弹性 slack 下允许瞬时违反);alpha 增益策略(gain 与
  margin 的耦合,是否改 class-K 常数增益)。
- **Not yet specified:** 各约束的 d_safe/width 的**理论**取值(目前靠经验)
  —— 数据来源需 ticket #08A/#08B、#12 支撑。
- **tickets:** #08A/#08B(可行性测量+拍板)、#12(几何核对)。

### 1.5 Collision representation
- **已实现:** 自碰撞 M2 OBB(10 OBB/14 对/exclusions Link3–Link5),dpax 内核
  (12×12 边对+点面)返回**米制真距离**;障碍 8×10 槽球(感知 tracks 编码)
  或聚合 min;`DCOL` 内核含 FCL 验证基线参考。
- **未知决策:** 排除对(Link3–Link5)是否安全、可行走区间是否覆盖;
  8 槽球 vs OBB 的保守度——见 ticket #12。
- **Not yet specified:** 真机环境几何(地面/工装/墙壁)是否需要 FCL/ESDF 模型
  (当前 ESDF 仅 32 球,生产未启用)。

### 1.6 LiDAR/depth fusion
- **已实现:** 点云级融合 = 时间戳配对 + voxel 降采样;三层占据(instant /
  unconfirmed / static);`OccupancyTracker` 时间戳驱动;8×10 槽球编码;
  tracks 过滤+速度估计(obs_vel/obs_radius_dot)。
- **决策 [DISCUSS-D] 已定(2026-09-05,05B):** 传感器外参**待标定**——方案已定
  两段式(FAST-Calib2 → T_lidar^cam;AX=YB → T_cam^base;复合入
  sensor_extrinsics.yaml + provenance),20mm 已知物体验收为硬门槛,P6 机械测量
  为回退;`world_frame=base_link` 与规格"固定环境系"冲突以 **spec 修订(T0)**
  解决——基座固定 ⇒ base_link 即固定环境系。
- **Not yet specified:** 传感器真实数据/回放文件不存在;LiDAR 与 depth 的
  时钟同步硬件方案(现为软件配对)。
- **tickets:** #05B ✅、#05A、#10(时间/延迟模型);impl: T5(标定工具链)/T6(启动自检)。

### 1.7 Coordinate frames
- **已实现:** 控制器/感知 `world_frame=base_link`;tool0 名义工具;
  `trajectory_offset_m=[0,0.343,1.587]`(整条参考轨迹被平移);MATLAB 蝴蝶曲线
  被**径向投影到最小二乘圆柱面**再作为参考(近似工具轴=柱轴心)。
- **决策 [DISCUSS-D] 已定(2026-09-05,05B):** base_link vs 独立环境系之争拍板为
  **A——全系统 base_link**;base_link 正式定义为用户规定的 Y-up 机架固定系
  (+Y 竖直、+Z 导轨水平、+X 横向),MuJoCo Z-up 仅 viewer 显示。
- **未知决策 [DISCUSS-D]:** 参考轨迹为什么是圆柱投影(原始 MATLAB 轨迹无姿态,
  投影假设来自哪?)。
- **tickets:** #05A/#05B ✅、#06A/#06B;impl: T0(spec 修订)/T1(tracks 契约)。

### 1.8 Timing/latency
- **已实现:** 100Hz;JIT 预热 8.6–28.75s(模块化/全量);单步延迟上报
  (`latency_p95_ms`);发布低通 τ=0.02;反馈路径含状态新鲜度/冻结/stall 检测。
- **未知决策 [DISCUSS-G]:** 10ms 预算是否保留——**实测 p95=19.367ms 已超**;
  JIT 预热 8.6s~28.75s 是否需要在 launch 前显式完成(热启动脚本)。
- **Not yet specified:** 感知→CBF 的延迟模型(ticket #10);Python/GIL + 无 RT
  调度下的硬实时保证。
- **tickets:** #01A/#01B、#10。

### 1.9 Redundant DOF
- **已实现:** 9-DOF 对 5D 任务;`use_nullspace_policy=false`;w_joint=0.1
  阻尼项让 QP 自然分配。
- **未知决策 [DISCUSS-F]:** 冗余策略(四选一,ticket #07A/#07B 数据支撑);
  避障是否应占高于任务的优先级(现在 CBF 是硬约束=最高,零空间自运动可以在
  CBF 限制内优选构型——未实现)。
- **tickets:** #07A/#07B。

### 1.10 QP feasibility
- **已实现:** qpax;弹性 vs hard;2 slack 速率;健康门→零速;M12 无障碍 0 失败。
- **未知决策 [DISCUSS-H]:** 有障碍工况的可行性**无数据**(ticket #08A/#08B 补);
  失败时零速是否接受(先停再恢复 vs 保性能降级)。
- **tickets:** #08A/#08B。

### 1.11 Trajectory tracking definition
- **已实现 (06A 定案):** 弧长路径跟随 + 进给率调度（nominal×3.5 再受
  joint/cbf/rate/tool-axis/endpoint-brake 五 cap + gamma 跨轨停止 +
  bounded lead 0.01m + endpoint 零速 + 无条件前馈——ff_scale 死代码）;
  `source_time_s`(参考时间)仅用于完成度显示/卡死检测/evaluator 完成度,
  **不参与任何推进或误差计算**;path_state 5 分量无时间坐标;
  参考由测量投影通过 lead cap 冻结;集总 43 行 CBF cap 在线推导;
  rate 约束生产**未启用**(rate_cap=inf);tool-axis cap 数值不绑定
  (min 0.5412 > nominal 峰值 0.2709 m/s);`maximum_reference_feedrate_step_m_s`
  为死参数;参考初始化为弧长 0(不播种实测位姿);5D 任务=3D 位置+工具轴
  sin(θ),roll 自由;姿态参考=surface_normal 帧(弧长索引)。
- **已定案 [06B]:** 验收按**分层语义的几何路径跟随**;cross_track=primary,
  pos_error=diagnostic;completion=弧长比;5D tool-axis(roll自由);
  关节电机能力为速度上限;50Hz预算+100Hz目标。完整定案见issues/06b。
- **tickets:** #06A(resolved)/#06B(resolved)。

### 1.12 Sim/evaluation methodology + real-time
- **已实现:** plant=几何积分模型(无物理);`tracking_evaluator`→
  `output/tracking_report.md`(位置/姿态误差、横偏、进给、QP 成功率、最小障碍
  距离、完成度、综合评分);M12 报告。
- **未知决策 [DISCUSS-E]:** 是否引入物理仿真(MuJoCo 物理/重力/惯量/接触)以
  隔离验证控制率?是否引入传感器仿真(LiDAR/depth 回放或 Gazebo 仿真)?
- **未知决策 [DISCUSS-G]:** 实时性:100Hz Python 无 RT 保障,当前 19.4ms 单步
  实际会丢周期——需要提前设计(降频 / C++ 内核 / 抢占,见 ticket #01A/#01B)。
- **Not yet specified:** 无物理仿真下"安全"claim 的边界(只在仿真"几何"意义上
  安全,真机 CAN 链路未验证)。真机演示目标(加工?抓取?避障演示?)未定。

---

## 2. 测试与质量现状(本次实测,2026-09-04)

| 套件 | 结果 | 说明 |
|---|---|---|
| `tests/`(主包, ROS2) | **4 failed / 241 passed / 141.95s** | settle×2(测试与实现不同步);e2e steps=10<200;perf p95=19.4ms>10ms |
| `portable_oscbf/tests`(全量) | **1 failed / 140 passed / 34 skipped / 560s** | tool-axis roll-only 容差回归(断言 atol=1e-8,实际偏差 2.7e-08,JAX 精度级) |
| `run_all_tests.sh` | **不可用** | `set -u` + ROS setup.bash `AMENT_TRACE_SETUP_FILES: unbound variable` 提前退出 |

对照 M12 所述 "85 passed/43 skipped" 是部分子集;真实全量数字以本表为准。
失败项全部复现,非环境负载造成(单独重跑同样失败)。

---

## 3. 需用户拍板的决策项(不阻塞,匹配 ticket)

| # | 决策 | 选项摘要 | 关联 |
|---|---|---|---|
| A | 跟踪语义与验收指标 | **已定案(06B):** 分层语义,cross_track=primary,RMS/p95/max | #06B ✅ |
| B | 参考轨迹/姿态来源 | **已定案(06B):** 算法测试轨迹,通用接口,source_time=诊断 | #06B ✅ |
| C | 验收阈值 | **已定案(06B):** cross_track≤0.15/0.5/2.0mm,orient≤0.05°/0.5° | #06B ✅ |
| D | 感知坐标系 | **已定案(05B):** A 全系统 base_link（基座固定 ⇒ base_link=固定环境系,数学等价;Y-up 机架系）+ spec 修订;标定两段式(FAST-Calib2 + AX=YB)+ 20mm 验收;失效策略 3 层(启动 fail-fast / 单帧 drop / 持续零速闭锁);**否决** B/C 实现(记录演进)/0.5s 重试/静默 identity | #05B ✅ |
| E | 仿真保真度 | 保持几何积分 plant / 引入 MuJoCo 物理 / 传感器仿真回放 | — |
| F | 冗余策略 | 4 选 1(见 #07A);是否启用零空间;避障 vs 任务优先 | #07B |
| G | 性能预算 | **已定案(06B):** 50Hz 预算(20ms)+100Hz 目标;miss rate≤1% | #01B |
| H | QP 失败策略 | 零速(现状) vs 降级;d_safe/margin 取值 | #08B |

---

## 4. Tickets 索引(`.scratch/oscbf-wayfinder/issues/`)

**2026-09-04 决策(修订):** 17 票分两层——Wayfinder 核心 12 票 + 普通 implementation
backlog 5 条(已知怎么做,直接实现,不进 Wayfinder)。
**拆分原则(本次修订):** 核心问题中 5 条混合「研究+拍板」的票按 research/prototype
(A, AFK: agent 可独立完成的事实调查)与 grilling(B, HITL: 只能由用户拍板的
目标与取舍)拆开;5 张 B 票对应 §3 决策项 A/B/C/D/F/G/H,标 `ready-for-human`。

**Tracker:** Wayfinder map = [#1](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/1)（GitHub 权威副本）；
17 个 ticket 已发布为 child issues（`Part of #1`，原生阻塞依赖已建）。本地文件每票含 `**Tracker:**` 行。

### 4.1 Wayfinder 核心队列(research / prototype / grilling)

| # | 标题 | 类型 | 依赖 | Tracker | 说明 |
|---|---|---|---|---|---|
| 06A | 精确刻画当前实现的跟踪语义 | research | — | [#2](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/2) | **P0 起点,已 resolved (2026-09-04)**;AFK 可做;只答「现状为何」→ 见 §5 |
| 06B | 选定目标跟踪语义与验收标准 | grilling | 06A | [#14](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/14) | **P0 决策枢纽,已 resolved (2026-09-04)**;HITL 8 项拍板 → 见 §5 |
| 05A | 感知世界坐标系审计(base_link vs 环境系) | research | — | [#4](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/4) | **已 resolved (2026-09-04)**;影响面对比表 + CBF frame contract → 见 issues/05a |
| 05B | 选定规范世界坐标系与标定策略 | grilling | 05A | [#15](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/15) | **已 resolved (2026-09-05)**;HITL 8 项拍板:选系 A + 标定两段式 + 失败 3 层策略 → 见 §5;迁移拆 T0–T7 |
| 07A | 冗余自由度策略对比(9-DOF vs 5D) | research | — | [#5](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/5) | 原 07 对 06 的依赖已解:策略对比可独立 AFK |
| 07B | 选定冗余目标与优先级 | grilling | 06B+07A | [#16](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/16) | **HITL 决策**:4 选 1+nullspace+优先级(§3 F) |
| 01A | 19.4ms 回归剖面与成本分布 | prototype | — | [#3](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/3) | 只测「慢在哪」+优化候选清单 |
| 01B | 选定控制率/延迟预算与实现策略 | grilling | 01A | [#17](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/17) | **HITL 决策**:10ms 保留?降频/C++?(§3 G);02 的 perf 口径依赖此票 |
| 08A | QP 可行性实证测量(障碍+自碰撞) | prototype | — | [#6](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/6) | 必须靠实验回答;先跑原型量化,不先改产品实现 |
| 08B | 选定不可行/裕度/降级策略 | grilling | 08A+12 | [#18](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/18) | **HITL 决策**:零速 vs 降级、裕度与汇编(§3 H) |
| 10 | 感知时间同步与延迟模型 | research+impl | — | [#7](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/7) | 为 08B/01A 提供延迟/年龄诊断输入 |
| 12 | 自碰撞/障碍几何模型核对 | research | —(部分依赖 URDF 数据) | [#8](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/8) | 决定 CBF 几何真实性;08B 的输入;数据不可得则标 Not yet specified |
| 13 | 更新 evaluator 指标与阈值 | impl | 06B | TBD | **06B 定案后新增**:cross_track 升级 primary、completion 改弧长比、新增 diagnostic 指标、阈值写入配置 |

### 4.2 Implementation backlog(已移出 Wayfinder)

| # | 标题 | 类型 | Tracker | 说明 |
|---|---|---|---|---|
| 02 | 主包测试套件修复(settle/e2e/perf 孤立性) | impl | [#9](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/9) | 已知怎么修;perf 项最终通过仍依赖 01B 结论 |
| 03 | run_all_tests.sh set -u 退出修复 | impl | [#10](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/10) | 单文件修复 |
| 04 | portable 容差回归修复 | impl | [#11](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/11) | 核对语义后放宽容差 |
| 09 | 配置一致性清理(alpha 漂移/遗留/死参数) | impl | [#12](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/12) | **06B 后优先级提升**:删除 ff_scale 死代码 + maximum_reference_feedrate_step_m_s 死参数 + reference_feedrate_scale 设大 |
| 11 | SocketCAN 后端与参数注入 | impl(blocked) | [#13](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13) | 硬件不足暂移出;vcan 代码部分可另行拆小票 |
| T0 | spec 与参数注释修订(坐标系/验收/故障反应) | impl | [#20](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/20) | **05B 后新增(立即)**:base_link 固定系表述/J1 直线导轨称法/world_frame 语义/20mm 验收/configured stop + interlock;perception_runtime.yaml 头注释同步 |
| T7 | 标定 SSOT 接线(sensor_extrinsics.yaml 唯一真源) | impl | [#27](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/27) | **P0(新增,post-review),T1/T2/T3 前置**:bridge/launch 读 sensor_extrinsics.yaml、runtime yaml 去矩阵、假定矩阵迁为 calibrated:false、校验 record==runtime(ID/hash);**运行时经 ament share 解析唯一 resolved 绝对路径加载,启动记录 path+calibration_id+内容 hash,T6 对比节点实际加载值(ADR 0004 运行时文件身份)** |
| T1 | `/perception/tracks` 契约升级(显式 obstacle observation) | impl | [#21](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/21) | **P0,depends T7**:timestamp + frame_id(→ 运行期 frame 校验) |
| T2 | 自体过滤 bridge 接线(robot_spheres) | impl | [#22](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/22) | **P0**(与 T1 并行),**depends T7**:JointState 缓存 → robot_spheres → feed_camera/feed_lidar(引擎侧已支持 fusion_engine.py:33-42) |
| T3 | 感知健康接线(perception_valid → 零速+锁存) | impl | [#23](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/23) | **P0**:depends T1+T2+T7+#10 模型;age_stop 暂=perception_timeout_s=1.0s provisional;apply_qp_health_gate 已存在 |
| T4 | 合成传感器仿真(闭环验证用) | impl | [#24](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/24) | **P1**: 仅无现场条件时闭环验证;现场条件具备后以真机为准 |
| T5 | 标定工具链(FAST-Calib2 ROS2 移植 + AX=YB + 板/夹具) | impl | [#25](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/25) | **P1**(P6 前置):DIY PVC 板(4 反光环+4 视觉标记)+ 法兰 Apriltag |
| T6 | 启动自检程序(数学合法性+标定状态+record==runtime) | impl | [#26](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/26) | **P1**:depends T5+T1+T7;按 05B item 6 分组判据(无「非单位阵」;det≈+1 不取 |det|≈1);失败拒绝进入避障模式 |

---

## 5. Decisions so far（已决问题,2026-09-04）

### 06A — 跟踪语义刻画（resolved）
**决策:** 当前实现的跟踪语义确定为 **弧长参数化路径跟随 + 进给率调度**,
不是时间参数化轨迹复现;`source_time_s` 无控制作用(仅完成度显示/卡死检测/
evaluator 完成度);参考与测量投影经 bounded lead(0.01m)弱耦合,投影停滞时
参考冻结。完整规格、差异矩阵、evaluator 映射见 issues/06a。

**新增事实(修正先前记述):**
- 「条件前馈」是死代码:`ff_scale` 已计算但从未使用,实际为无条件前馈
  (仅受 gamma/caps 削弱的 feedrate 缩放)。
- `maximum_reference_feedrate_step_m_s=0.005` 为死参数(仅声明+校验)。
- 生产 QP 的 rate 约束未启用(rate_cap=∞);temporal_proximal λ=0.2 生效。
- tool-axis cap 在当前参数下数值不绑定(min 0.5412 > nominal 峰值 0.2709 m/s)。
- ROS 控制器参考初始化为弧长 0(不播种实测位姿)。
- 实测:等价调度(纯复现)走完 1.8168 m 弧长需 8.48 s(源时长 29.98 s,×3.5)。

**移交 06B:** 验收语义/误差口径(pos 3D vs 横偏)、completion 定义、
roll 是否控制、名义进给率与阈值、死代码取舍、参考起始播种——共 7 项
(见 issues/06a §11)。

### 06B — 目标跟踪语义与验收标准（resolved）**决策:** 最终控制目标 = **分层语义的几何路径跟随**。8 项拍板如下:

1. **任务语义 (Q1=C):** 硬目标 = 几何路径跟随(cross-track + 工具轴);
   性能 = 关节电机能力上限下的速度表现。安全减速不计为误差。
2. **位置误差 (Q2=C):** cross_track = mandatory primary; pos_error(3D) =
   diagnostic。RMS/p95/max 三者保留。
3. **Orientation (Q3=A):** 继续 5D tool-axis tracking,roll 自由。冗余 DOF
   更多(4 vs 3),后续可 additive upgrade 到 6D。
4. **参考轨迹 (Q4=B):** 蝴蝶轨迹 = 算法测试轨迹(发论文好看),后续换真实
   任务。trajectory loader 设计成通用接口。source_time 保留为诊断字段。
5. **Feedrate (Q5):** 以关节电机能力为速度上限。reference_feedrate_scale
   设大(≥10)让 nominal 不绑定;启用 rate constraint 防跳变;temporal_proximal
   保留。feedrate 调度 = gamma × min(joint,cbf,rate,tool_axis,endpoint_brake)。
6. **Completion (Q6=A):** 唯一定义 = projected_arc / total_arc。source_time
   不参与完成度。endpoint hold 独立记录。completion=100%(mandatory)。
7. **Startup (Q7=B):** 启动时投影播种(复用 initial_path_follower_state)。
   对任意轨迹通用(投影不依赖圆柱几何)。
8. **Acceptance (Q8):** cross_track RMS≤0.15mm / p95≤0.5mm / max≤2.0mm
   (mandatory,provisional); orient RMS≤0.05° / max≤0.5°
   (mandatory,provisional); QP≥99.9%(mandatory); min clearance≥0(mandatory)
   +≥30mm(provisional,ISO手指级); deadline miss rate≤1%(provisional,
   50Hz预算,100Hz目标)。详见 issues/06b acceptance table。

**已否决:** 时间参数化跟踪(B)、6D orientation(B)、nominal 硬目标(B)、
3D pos_error primary(B)、精确过渡到起点(A)、source_time 完成度(B)、
25Hz 控制频率。

**对后续的影响:** trajectory loader 泛化(rate constraint 启用 + orientation
from_data 接口);path tracking kernel 零改动;evaluator 指标重定义;nullspace
/07B 受益于 5D 冗余;01B 按 50Hz 预算+100Hz 目标;09 清点死代码。完整
consequences 见 issues/06b。

**新增 tickets:** 09 优先级提升(ff_scale/maximum_reference_feedrate_step_m_s
删除 + reference_feedrate_scale 设大);evaluator 更新 ticket(新增,依赖本决策)。

### 05A — 感知世界坐标系审计（resolved）

**审计结论（2026-09-04,决策仍归 05B）:** **软件内部 frame convention 当前自洽**——
全链路点云/占据/tracks/FK 几何实际共用 `base_link`（数学自洽,仅为软件内部约定,
不代表物理坐标正确;当前无物理标定精度证据:相机外参仅假定安装位姿、无
calibration provenance,LiDAR 外参仍未验证）;`world_frame` 参数**不控制**任何
变换——它只作为 3 个输出话题的 frame_id + 休眠中的 TF 目标（use_tf=false）;
真正绑定 frame 的是 `camera_to_world_static`（perception_runtime.yaml,非单位阵
假定矩阵）与控制器侧隐式约定。`config/sensor_extrinsics.yaml` 为**死配置**
（零代码消费）;TF 缺失时行为 = 警告 + 回落静态矩阵/identity 的 silent degradation
（非 fail-fast / 非 drop）。

**关键事实（修正 MAP 1.6 表述）:**
- 「外参全占位」不准确:**相机外参**实际加载的是 perception_runtime.yaml 的非单位
  阵假定安装位姿（无标定来源/无 provenance）;LiDAR 外参为 unit 占位且整条链未启用
  （source_topic_lidar=""）;sensor_extrinsics.yaml 的占位矩阵未被加载。
- base_link vs 环境系的冲突仅在**基座运动**时成立;仿真闭环中基座固定（URDF 根
  = base_link、J1 升降在链内、plant 只随机关节）- 等价假设(基座恒定刚体位姿 +
  传感器刚体安装 + 同单位)下 base_link 与固定环境系数学等价,但**一旦基座移动**,
  三层占据（instant/unconfirmed/static）跨帧持久化语义、ESDF 与 track 速度估计
  会破坏（环境视运动 + 双影 + 假移动 track）;单帧几何仍自洽。
- 与 spec 口径差异:spec 称 "base_link 随升降轴运动",与 URDF（J1 prismatic 位于
  base_link 之下,移动的是 Link1）不符;真机基座**固定落地**（无升降台,该"升降台
  Not yet specified"项已由 05B 拍板:基座固定不动;J1 为水平直线导轨,见 §5 05B）。
  -- 本文中"升降台/升降轴"术语自 05B 起终止使用（实为直线导轨）。
- CBF frame contract:C1-C6（机器人几何与障碍 pos/vel/radii 同系=base_link,
  tracks 消息无 frame 元数据,无任何运行期校验,违反=静默错误）。
- 05B 决策输入（选系矩阵 / 标定四选一 / 缺失数据清单 / 迁移接口清单）全部在
  issues/05a §4、§7 与附录,交由人类拍板。

### 05B — 选定规范世界坐标系与标定策略（resolved,2026-09-05）

**决策内容（8 项拍板,详见 issues/05b §Resolution）:**

1. **基座运动边界:** 基座物理固定;1P8R,J1 = 工作台直线导轨/丝杠(链内关节 q1,
   URDF 轴 z,0–0.585m),移动的是滑台+臂,不是 base_link;
   **否决/终止**"升降轴/升降台"称谓(实为直线导轨)。
2. **坐标系约定:** base_link = 用户规定的 Y-up 机架系(+Y 竖直、+Z 导轨水平、
   +X 横向);感知/控制器/MoveIt/POE FK 全链路共用;MuJoCo Z-up 仅 viewer 显示。
3. **选系: A(全系统 base_link)。** 依据:基座固定 + 传感器臂外固定安装 ⇒
   base_link 与固定环境系数学等价、零改动零风险。B/C 不实现;**C 保留为演进路径**
   (未来移动基座:感知持久化存环境系 + 边界带 timestamp 变换)。
4. **spec 处理:** 修订错误表述(base_link 固定/J1 直线导轨/world_frame 语义/
   20mm 验收/故障反应)→ **T0**(立即)。
5. **失效策略(3 层):** 启动期 fail-fast(分组校验:数学合法性+标定状态+record 一致
   →拒避障模式);运行期**仅瞬时数据异常**单帧→drop(沿用上一帧+d_safe);
   持续 fusion_age>age_stop→`apply_qp_health_gate` 零速+锁存(restart interlock)。
   **否决:** 0.5s 有限重试;静默 identity 回退(现 `_sensor_to_world`
   current→latest→static→identity 链废弃)。**post-review(2026-09-05):**
   age_stop 暂=perception_timeout_s=1.0s 为 **provisional** 工程值,由 #10
   陈旧数据模型(d_safe,eff=d0+v_bound·age+σ_calib+σ_tracking;分档 use+inflate/
   conservative/zero+latch)结算;静态外参无效/provenance 缺失=立即 invalid 不入
   避障,非 drop 后继续。本段为 **Target/decided behavior**——T1/T2/T3/T5/T6/T7
   完成前不得视为 current behavior。
6. **不可信判据(post-review 修订:删除「单位阵」判据,拆两组):** 数学合法性
   (元素非有限/R^T R≠I/det≠+1/bottom row/translation 机械范围)+ 标定状态
   (calibrated:true+provenance 字段完整+20mm 验收+record==runtime 加载值
   ID/hash)+ 运行期健康(时间戳/age/自体过滤生效)。identity 可以是合法外参,
   不代表未标定。
7. **标定策略(两段式):** FAST-Calib2(PVC+4 反光环+4 视觉标记 DIY 板,≥3 场景,
   lidar_center_test 前置)→ T_lidar^cam;法兰小 Apriltag + AX=YB → T_cam^base;
   复合 T_lidar^base 入 sensor_extrinsics.yaml + provenance;20mm 验收硬门槛;
   P6 机械测量回退;**否决**反光球/$2k 孔板主标定、传感器装臂/运动部件。
   **post-review:** 该文件为**唯一 calibration authority**(ADR 0004/T7)。
8. **迁移范围(T0–T7 全立项,Part of #1):** T0 spec 修订(立即);
   **T7 标定 SSOT 接线(P0,T1/T2/T3 前置,新增)**——sensor_extrinsics.yaml 唯一真源、
   runtime yaml 去矩阵、假定矩阵迁为 calibrated:false、record==runtime 校验;
   **运行时经 ament share 解析唯一 resolved 绝对路径加载,启动记录
   path+calibration_id+内容 hash,T6 对比节点实际加载值(ADR 0004)**;
   T1 tracks 契约 timestamp+frame_id(P0,depends T7);
   T2 自体过滤 bridge 接线(P0,与 T1 并行,depends T7);
   T3 感知健康 perception_valid→零速+锁存(P0,依赖 T1+T2+T7+#10 模型;
   age_stop 暂 1.0s provisional);
   T4 合成传感器仿真(P1);T5 标定工具链(P1,P6 前置);
   T6 启动自检程序(P1,依赖 T5+T1+T7;判据按 item 6 分组);
   **仅 P6 现场数值标定+20mm 验收受现场阻塞**。

**文档产物:** ADR 0002(base_link 唯一规范系;post-review 修正 B/C 定义)/
ADR 0003(感知失效策略;post-review 修订判据分组+provisional 1.0s)/
ADR 0004(标定 SSOT;二轮补充「运行时文件身份」);CONTEXT.md 词汇(世界坐标系/标定 provenance);
spec 修订(T0 内容)。以上失败/超时/闭锁行为均为 Target/decided,
非 current behavior(实现见 T1–T7)。
