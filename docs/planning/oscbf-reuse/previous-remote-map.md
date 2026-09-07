> 本地镜像: `.scratch/oscbf-wayfinder/MAP.md`（本 issue 为下游权威副本；child tickets 见下方列表）

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
- 跟踪语义是**弧长路径跟随 + 进给率调度**,不是时间参数化复现。

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
- **未知决策 [DISCUSS-D]:** 传感器外参**全占位**——标定方案/数据无;
  `world_frame=base_link` 与规格"固定环境系"冲突(机器人基座动则世界机系动)。
- **Not yet specified:** 传感器真实数据/回放文件不存在;LiDAR 与 depth 的
  时钟同步硬件方案(现为软件配对)。
- **tickets:** #05A/#05B(坐标系审计+拍板)、#10(时间/延迟模型)。

### 1.7 Coordinate frames
- **已实现:** 控制器/感知 `world_frame=base_link`;tool0 名义工具;
  `trajectory_offset_m=[0,0.343,1.587]`(整条参考轨迹被平移);MATLAB 蝴蝶曲线
  被**径向投影到最小二乘圆柱面**再作为参考(近似工具轴=柱轴心)。
- **未知决策 [DISCUSS-D]:** 参考轨迹为什么是圆柱投影(原始 MATLAB 轨迹无姿态,
  投影假设来自哪?);base_link vs 环境系。
- **tickets:** #05A/#05B、#06A/#06B。

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
- **已实现:** 弧长路径跟随 + 进给率调度(五限速+cross-track stop+bounded lead
  0.01m+条件前馈);`source_time_s`(参考时间)仅用于完成度;
  `reference_lead_m=0.01`;`reference_feedrate_scale=3.5`;`max_tool_axis_speed_rad_s=2.0`;
  endpooint-brake 入闸。
- **未知决策 [DISCUSS-A]:** 这是"路径跟随"而非"时间参数化跟踪"——
  **验收到底按哪种语义**?跟踪误差该按弧长横偏(现在)还是按参考时间?
- **tickets:** #06A(刻画)/#06B(拍板)。

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
| A | 跟踪语义与验收指标 | 路径跟随(现状) vs 时间参数化;误差定义/阈值(当前基线 0.0352mm 是低速路径偏差) | #06B |
| B | 参考轨迹/姿态来源 | 圆柱面投影近似是否可接受;是否需要真实加工/演示目标(用途决定轨迹参数化) | #06B |
| C | 验收阈值 | 位置/姿态/横偏/进给率的通过阈值(现有 evaluator 已算,阈值未定) | #06B |
| D | 感知坐标系 | base_link(现状) vs 固定环境系(规格);外参标定方案 | #05B |
| E | 仿真保真度 | 保持几何积分 plant / 引入 MuJoCo 物理 / 传感器仿真回放 | — |
| F | 冗余策略 | 4 选 1(见 #07A);是否启用零空间;避障 vs 任务优先 | #07B |
| G | 性能预算 | 10ms 是否保留;JIT 预热时长接受度;RT 保障方案 | #01B |
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
| 06A | 精确刻画当前实现的跟踪语义 | research | — | [#2](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/2) | **P0 起点**:AFK 可做;只答「现状为何」 |
| 06B | 选定目标跟踪语义与验收标准 | grilling | 06A | [#14](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/14) | **HITL 决策**:演示场景/语义/roll/阈值(§3 A/B/C) |
| 05A | 感知世界坐标系审计(base_link vs 环境系) | research | — | [#4](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/4) | 影响面对比表,AFK 可做 |
| 05B | 选定规范世界坐标系与标定策略 | grilling | 05A | [#15](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/15) | **HITL 决策**:base_link vs 固定环境系(§3 D) |
| 07A | 冗余自由度策略对比(9-DOF vs 5D) | research | — | [#5](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/5) | 原 07 对 06 的依赖已解:策略对比可独立 AFK |
| 07B | 选定冗余目标与优先级 | grilling | 06B+07A | [#16](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/16) | **HITL 决策**:4 选 1+nullspace+优先级(§3 F) |
| 01A | 19.4ms 回归剖面与成本分布 | prototype | — | [#3](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/3) | 只测「慢在哪」+优化候选清单 |
| 01B | 选定控制率/延迟预算与实现策略 | grilling | 01A | [#17](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/17) | **HITL 决策**:10ms 保留?降频/C++?(§3 G);02 的 perf 口径依赖此票 |
| 08A | QP 可行性实证测量(障碍+自碰撞) | prototype | — | [#6](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/6) | 必须靠实验回答;先跑原型量化,不先改产品实现 |
| 08B | 选定不可行/裕度/降级策略 | grilling | 08A+12 | [#18](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/18) | **HITL 决策**:零速 vs 降级、裕度与汇编(§3 H) |
| 10 | 感知时间同步与延迟模型 | research+impl | — | [#7](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/7) | 为 08B/01A 提供延迟/年龄诊断输入 |
| 12 | 自碰撞/障碍几何模型核对 | research | —(部分依赖 URDF 数据) | [#8](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/8) | 决定 CBF 几何真实性;08B 的输入;数据不可得则标 Not yet specified |

### 4.2 Implementation backlog(已移出 Wayfinder)

| # | 标题 | 类型 | Tracker | 说明 |
|---|---|---|---|---|
| 02 | 主包测试套件修复(settle/e2e/perf 孤立性) | impl | [#9](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/9) | 已知怎么修;perf 项最终通过仍依赖 01B 结论 |
| 03 | run_all_tests.sh set -u 退出修复 | impl | [#10](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/10) | 单文件修复 |
| 04 | portable 容差回归修复 | impl | [#11](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/11) | 核对语义后放宽容差 |
| 09 | 配置一致性清理(alpha 漂移/遗留/死参数) | impl | [#12](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/12) | 机械清点 |
| 11 | SocketCAN 后端与参数注入 | impl(blocked) | [#13](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13) | 硬件不足暂移出;vcan 代码部分可另行拆小票 |
