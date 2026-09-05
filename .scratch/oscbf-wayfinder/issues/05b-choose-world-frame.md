# 05B — 选定规范世界坐标系与标定策略（grilling）

**What to build:** HITL 决策票（只能由用户拍板，AFK agent 无法回答），由 05A 的
影响面对比表驱动，回答：

- **规范世界坐标系选哪个候选架构？** 05A 审计后候选由二变三：
  - **A. 全系统 base_link（现状）**；
  - **B. 全系统 fixed-world（固定环境系，spec 要求）**；
  - **C. 感知 world + 控制器 base_link（混合，新增候选）**——perception 使用固定
    workcell/world 作为 canonical persistence frame；OSCBF/POE/robot collision
    geometry 继续使用 base_link；在 perception→controller 边界通过**带 timestamp
    的显式 transform** 将 obstacle position/velocity 转换到 base_link。
  三案八维对比见下方「候选架构对比（A/B/C）」。
- **外参标定策略：**传感器外参目前全为占位/假定（方案/数据无）——需要标定哪些量、
  如何获取（工装测量 / 外部软件 / 手眼标定）、标定失败时的默认与降级行为（四选一
  对比见 issues/05a §4）；
- **TF / extrinsic 无效、过期或缺失时如何失败？**（必须由用户拍板——现有
  timestamp TF → latest TF → static → identity 的静默 fallback 不能未经决策
  直接作为 safety-critical CBF 输入策略，见「必拍板问题」第 3 条）;
- **迁移范围：**按 05A 清单决定本次只改配置，还是需要拆 impl 票（明确票面）。

**Blocked by:** ~~05A~~ — 已解除：05A 已于 2026-09-04 resolved
（影响面清单 / CBF frame contract / 外参真实状态 / 决策输入见 issues/05a §Resolution）。

**Type:** grilling（HITL）

**Queue:** wayfinder-core — 决策票：世界系与标定策略（MAP 讨论项 D）。
**Tracker:** #15 (https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/15)

**Status:** resolved（2026-09-05，HITL 拍板完成，见 §Resolution）

- [x] 选定候选架构（A：全系统 base_link / B：全系统 fixed-world / C：感知 world + 控制器 base_link）并给出理由
- [x] 选定外参标定策略（方案 + 失败降级 + 数据归属）
- [x] 拍板 TF/extrinsic 无效、过期、缺失时的失败策略（禁止静默 fallback 直入 CBF）
- [x] 确认迁移范围（只改配置 / 拆 impl 票并明确票面）

---

## 候选架构对比（A/B/C）

**定义：**
- **A — 全系统 base_link**（现状）：点云 / 占据 / tracks / FK 全部以 base_link 为唯一坐标系。
- **B — 全系统 fixed-world**：感知与控制器全部改用固定环境系（map/workcell/world，
  repo 中目前不存在该帧，需先定义物理基准）。
- **C — 感知 world + 控制器 base_link**（混合，新候选）：perception 以固定
  workcell/world 作为 **canonical persistence frame**（三层占据 / ESDF / track
  关联全部在该帧内做）；OSCBF/POE/robot collision geometry 继续用 base_link；
  perception→controller 边界通过**带 timestamp 的显式 transform** 把 obstacle
  position/velocity 转换到 base_link——与 05A §6 末「可选补充」一致（自适应层变换，
  kernel 保持 frame-agnostic）。

| 维度 | A: base_link 全域 | B: fixed-world 全域 | C: 感知 world + 控制器 base_link |
|---|---|---|---|
| **persistent occupancy** | 基座不动时 OK；基座动 → 体素身份 = base_link 格（static_occupancy.py:70-78），跨帧持久化/静态确认语义破坏 + 双影（05A §5 1-2） | 体素身份 = 环境格，持久化语义正确 | 与 B 相同：持久化在环境系内做，语义正确 |
| **dynamic obstacle velocity** | 基座不动时 OK；基座动 → 环境整体视运动 ≈ −v_base，静态点可能被聚成假移动 track（05A §5 3） | 速度 = 环境真实速度 | 感知侧速度 = 环境真实速度；边界变换必须**同时变换速度**（旋转 + 平动微分项），不能只转位置 |
| **LiDAR/depth fusion** | 两传感器经静态外参到 base_link，融合自洽（FusionEngine frame-agnostic） | 外参目标帧改为 env（或 sensor→base→env 复合），TF 链/时间戳查询复杂度上升 | 融合在 env 内进行（传感器→env 外参，或经由 sensor→base→env 复合）；融合引擎本体不改 |
| **CBF kernel 改动量** | **零** | 内核或调用方需处理 env 帧输入（05A 附清单 #4/#5/#6） | **零**——kernel 保持 frame-agnostic；只改适配层：oscbf_controller/facade 加变换接入点（05A 附清单 #4/#5） |
| **固定基座（现状）** | 零改动、零风险；与 spec「world_frame 必须是固定环境系」直接冲突（可走 spec 修订/偏差登记） | 需先定义环境帧（repo 中不存在）+ 基座→env 标定；固定基座下与 A 数学等价，短期收益为零 | 需定义环境帧 + 1 次静态基座→env 标定；固定基座下短期收益有限，收益在「基座可能移动」的预留 |
| **未来移动基座** | **语义崩溃**（05A §5） | 语义正确；需动态 T_base→env(t) TF + 逐周期查询（延迟/时间戳一致性与 #10 交织） | 语义正确；传感器→env 需含基座动态（或动态段放在 sensor→base→env），控制器侧查询 env→base @ 消息到达时刻 + 相对速度修正——持久化/速度语义与 B 同，改动集中在边界适配层 |
| **真机迁移** | 「装好即用」：装传感器→测外参→填矩阵；无环境锚点，与外部工装/多机对齐需额外变换 | 需环境物理基准（地面/立柱/工装角点）+ 基座→env 测量；长期可迁移、符合 spec | 同样需环境物理基准 + 基座→env 测量；但控制器/内核无改动（边界适配层一次性实现），感知与控制器边界清晰、可分段验证与迁移 |
| **calibration complexity** | 2×传感器→base_link 外参（无动态项） | 2×传感器→基座 + 1×基座→env；基座可动时 + 动态 T_base_env(t) | 标定量与 B 相同（2×传感器→env + 1×基座→env，可静态或动态）；责任拆为「感知内部标定」与「感知→控制边界变换」两段，验证边界更清晰 |

**非绑定观察（agent 侧，供拍板参考，不代作决策）:** 三案均建立在同一 05A 事实上——
当前无物理标定精度证据（相机外参 = 假定位姿、LiDAR 外参未验证），且
/perception/tracks 无 frame/time 元数据。据此推断：验证期内选 A 风险最低；若「未来
基座可动」进入 plan，C 在保留 B 的环境语义收益的同时，CBF kernel 零改动、控制器侧
改动集中在边界适配层，是 B 的最小侵入替代；B 仅在「外部工装/多机对齐」成为显式需求
时才必要。

---

## 必须由用户拍板的问题

1. **选系（A/B/C）：** 见上表。判定前请先明确「基座是否可能移动」的边界（当前仿真闭环
   基座固定；真机落地/升降台状态 Not yet specified）。
2. **标定策略（四选一）：** 机械测量/工装标定；TF static transform；外部标定工具；
   hand-eye/target-based（对比见 issues/05a §4）。同时拍板：误差估计与 calibration
   provenance 是否入仓库、error_estimate 为 null 时是否放行、标定数据归属。
3. **TF / extrinsic 无效、过期或缺失时如何失败？**（**新增必拍板问题**）
   - 现状行为链：timestamp TF（msg.stamp）→ 失败回落 latest TF（Time()）→
     失败回落静态矩阵 → 空/identity——全部仅 warn + 继续（silent degradation，
     05A §3.2）。`use_tf=false` 时前两级不可达，恒走静态矩阵分支。
   - 问题实质：该 fallback 链把「变换不可信」静默降级为「用最近/假定变换继续」，
     直接决定障碍位置与速度是否可信——这是 safety-critical CBF 输入策略，
     **不能未经决策直接采用**。
   - 候选选项（可组合）：
     - ① fail-fast：校验失败 → 丢弃该帧并置感知健康异常，进入安全状态
       （零速/保持，与 08B 的失败策略对齐）；
     - ② 有限容错：N 帧 / T 秒内重试，超时进入安全状态；
     - ③ drop frame：跳过该帧，安全状态不变，CBF 沿用上一帧有效障碍
       （旧数据过时预算需定义，与 #10 交织）。
   - 需要一并明确：「不可信」的判据（帧率/延迟窗口/矩阵非有限值/单位校验）与
     校验时机（消息到达时 vs 每周期）。
4. **迁移范围：** 只改配置 / 拆 impl 票（明确票面）。若选 C，impl 票主体 =
   消息契约升级（见 Consequences #1）+ 边界变换适配层；若选 B 全量，范围见
   05A 附清单。

---

## Consequences（后续影响 / 前瞻 ticket 登记）

以下为 05B 之后值得立项的潜在 implementation ticket（**前瞻记录：不阻塞本票、
不构成选系前提，仅在 05B 拍板后按选系决定优先级**）：

1. **`/perception/tracks` 契约升级（潜在 impl ticket；建议依赖 05B 选系 + #10）**
   现状 `/perception/tracks` 是无 frame / 无 time 的隐式 `Float32MultiArray` 契约
   （8 槽 × 10 float；无 header、无 frame_id、无时间字段，05A 契约表 + §6 C4）——
   frame 信息完全不随消息传递，违反 C3/C4 时是静默的几何/速度错误。
   升级为**显式 obstacle observation contract**：每条 obstacle 观测带
   `timestamp` + `frame_id`（可选：来源传感器 / 置信度 / provenance 指纹）。
   效果：
   - C4/C6 从「跨包约定」变为**运行期可校验**的契约（控制器校验 frame_id == FK 系，
     不一致 → 报警/拒绝，不再静默）；
   - 为候选 C 的边界变换提供「带 timestamp 的显式 transform」所需的数据语义
     （velocity 归属 frame、position 带时间戳）；
   - 与 #10（感知时间同步/延迟模型）共享时间戳语义——建议作为 #10 的前置或并行票。
   优先级依赖选系：选 A 时最低（仍值得做，因运行期校验可执行）；选 C 时此票 =
   边界适配层的依赖项（高）。→ **2026-09-05 拍板选 A 后定为 T1（P0）**，见 §Resolution 第 8 条。

**对 MAP 的回填:** 05B 定案后回填 MAP §3 讨论项 D 与 §4.1 05B 行（本次不执行，
避免预判结论）。→ **2026-09-05 已回填**，见 §Resolution 第 8 条。

---

## Resolution（2026-09-05，HITL 拍板）

**拍板顺序结论汇总（8 项）：**

1. **基座运动边界：** 基座物理固定不动；1P8R 拓扑，J1 = 工作台直线导轨/丝杠
   （链内关节 q1，URDF 轴 (0,0,1)，行程 0–0.585m），运动的是滑台与臂、不是
   base_link；base_link 固定于工作环境。规格中「base_link 随升降轴运动」的表述
   **错误**，同步修订为「直线导轨（非升降轴）」并删除该错误声明。
2. **坐标系约定：** base_link = 用户规定的 Y-up 机架系（+Y 竖直向上、+Z 直线导轨
   水平方向、+X 横向），感知/控制器/MoveIt/POE FK 全链路共用；MuJoCo Z-up 仅
   `display_frame` 显示层使用（不参与物理回路）。
3. **选系：A（全系统 base_link）**。依据：基座固定 + 传感器均臂外固定安装 ⇒
   base_link 与任何固定环境系数学等价，A 零改动、零风险，且与
   `base_link` 定义的 Y-up 语义一致。B/C 不实现；**C 保留为演进路径记录**
   （未来基座变为移动基座时：感知持久占用/动态目标存固定世界系、OSCBF/
   碰撞几何留 base_link、边界做带 timestamp 显式变换），当前不立项。
4. **spec 处理（口径：A 的配套修订，不改变选系）：** 修订 spec 的错误表述——
   base_link 固定 / J1 为直线导轨（在链内，非基座运动）/ `world_frame` 语义
   （= base_link，删除「必须独立环境系」）/ 20mm 已知物体验收 / 故障反应
   （configured stop + restart interlock）；`perception_runtime.yaml` 头部注释
   同步修正。
5. **失效策略：** ①启动期 fail-fast（见第 6 条分组校验 → 任一失败拒绝进入避障
   模式）；②运行期**仅瞬时数据异常**（单帧 TF 查询失败/丢帧/解码错）→ 丢弃该帧
   （CBF 沿用上一帧 + d_safe 裕度）；③持续失效 fusion_age >
   `perception_timeout_s` → 经 `apply_qp_health_gate` 置零速 + 锁存
   （restart interlock）。
   **否决：② 0.5s 有限重试窗口（与 ③ 阈值语义重叠、陈旧数据窗口只是包装风险）；
   ③ 静默身份回退（几何含义错误、不可追踪的静默降级）。**
   当前 `_sensor_to_world` 的 current→latest→static→identity 兜底与 static 参数
   为空时静默 `_identity_extrinsics()` 行为按本策略废弃。
   **（2026-09-05 post-review 修订：① 细化按下条分组；「运行期单帧失败」明确
   限定为瞬时数据异常——静态外参本身无效/provenance 缺失时**立即 invalid、
   不允许进入 perception-based avoidance**，不是「drop 一帧继续」；
   ③ 的 perception_timeout_s=1.0s 为 provisional engineering value，
   最终由 #10 陈旧数据模型结算，见 §Post-review。）**
6. **不可信判据（启动校验 + 运行期每周期）分两组（2026-09-05 post-review 修订：
   删除「非单位阵」判据——合法刚体外参可以为 identity，identity 只是配置管理
   约定，不等价于「未标定」；把数学合法性与标定状态分开，标定状态判据取代
   matrix≠identity）：**
   - **数学合法性**：元素非有限 / R^T R 偏离单位阵 / det≠+1 / bottom row
     非 [0 0 0 1] / translation 超出机械合理范围；
   - **标定状态**：`calibrated: false` / provenance 字段缺失
     （sensor_serial、calibration_id、method、timestamp、operator、residual、
     error_estimate、frame_from、frame_to）/ 与记录偏差超 ≤20mm/5° /
     calibration record ≠ runtime 加载值（ID/hash，ADR 0004）/ 已知物体
     20mm 验收失败；
   - **运行期数据健康**：时间戳超 camera_max_age(0.5s) / fusion_age>1s
     （provisional，#10 结算）/ 自体过滤未生效（tracks 与机器人几何重叠）。
   任意一条成立 ⇒ 按第 5 条策略处置。
7. **标定策略（两段式）：** Stage 1 = FAST-Calib2
   （PVC 板 + 4 反光环 + 4 视觉标记 DIY 标定板，≥3 场景，`lidar_center_test`
   为前置条件）→ T_lidar^cam；Stage 2 = 法兰小 Apriltag + 眼在手外
   AX=YB 求解 → T_cam^base；合成 T_lidar^base = T_cam^base·T_lidar^cam 写入
   `config/sensor_extrinsics.yaml` 并携带 provenance（方法/日期/操作者/残差/
   误差估计）；20mm 已知物体验收为硬门槛。**回退：P6 机械手动测量**（仅标定
   失败/不可用时）；**否决：反光球靶标与 $2k 孔板作为主标定方案；传感器装
   臂上/运动部件（外参变时变，否决）。** 未校准证据前不宣称物理精度
   （与 05A §Resolution 一致）。
8. **迁移范围（T0–T7 全部立项于 impl backlog，Part of #1）：**
   - **T0** spec 修订（立即；本批完成的 spec 修订即其内容，按惯例列票归档）——
   - **T7 标定 SSOT 接线（2026-09-05 post-review 新增，P0，T1/T2/T3 前置）**：
     `sensor_extrinsics.yaml` 成为唯一 calibration authority——bridge/launch
     从它加载外参，`perception_runtime.yaml` 移除 `camera_to_world_static` /
     `lidar_to_world_static`（仅留 topic/frame/voxel/timeout/fusion）；
     现有假定相机矩阵迁移为 `calibrated: false` 记录（保持单 Camera 行为不变）；
     结构扩展 calibrated/sensor_serial/calibration_id/method/timestamp/operator/
     residual/error_estimate/frame_from/frame_to；T6 校验 record==runtime（ID/hash）。
     详见 ADR 0004 + issues/07b-post-review（新票见 §Post-review）；
   - **T1** `/perception/tracks` 契约升级：显式 obstacle observation contract
     （timestamp + frame_id，可选来源/置信度/provenance 指纹）（**P0**，
     **depends T7**）；
   - **T2** 自体过滤 bridge 接线：`perception_bridge` 缓存 JointState → 构建
     robot_spheres → `feed_camera/feed_lidar` 传入（引擎侧已支持，
     fusion_engine.py:33-42，bridge 侧 TODO）（**P0**，与 T1 并行，
     **depends T7**）；
   - **T3** 感知健康接线：`/perception/status` 的 perception_valid →
     控制器健康通道 → 零速 + 锁存（**P0**，依赖 T1+T2 提供时间戳语义 +
     T7（外参可信）+ #10 stale-data 模型；T1 之前 `_tracks_callback` 无新鲜度
     检查、obs_state 永久缓存；age_stop 暂用 perception_timeout_s=1.0s
     provisional，由 #10 结算）；
   - **T4** 合成传感器仿真（**P1**：仅用于闭环验证，未现场条件下替代；
     现场条件具备后以真机为准）；
   - **T5** 标定工具链：FAST-Calib2 ROS2 移植 + AX=YB 求解器
     + 标定板/夹具采购（**P1**，P6 前置）；
   - **T6** 启动自检程序：矩阵校验 + provenance 比对 + 已知物体 20mm 验收
     （**P1**，依赖 T5+T1）。
   - **仅 P6（现场数值标定 + 20mm 验收）受现场条件阻塞**；P0 三票与 T4 不阻塞于现场。

**被否决并记录（供后续审阅）：** B（fixed-world 全域）、C（混合且不排除演进）、
反光球/机械测量为主标定、0.5s 有限重试（②）、静默身份回退、传感器装臂/
运动部件、升降台（本项目无升降轴，术语终止使用）。

---

## Post-review（2026-09-05，用户审查 2e21db4 后）

**结论：05B 决策 PASS with changes（核心架构决策保留，本批非推翻重做）。**
6 项修整与本票的落点：

1. **P0-1 标定 SSOT**——`config/sensor_extrinsics.yaml` 为唯一 calibration
   authority（标定工具写它、bridge/launch 读它）；`perception_runtime.yaml`
   移除 calibration matrix，只留 topic/frame/voxel/timeout/fusion；
   T6 验证「被校验 record == runtime 加载 record」（calibration_id/hash）。
   **新立项 T7（P0，T1/T2/T3 前置）**；ADR 0004 记录决策；
   本票 item 5/6/8 已同步（拆分校验分组 + 记录一致性）。
2. **P0-2 删除「非单位阵=有效标定」**——合法 SE(3) 外参可为 identity；
   identity 是配置管理约定，不是几何合法性规则。拆为「数学合法性」
   （R^T R ≈ I / det≈1 / 有限 / bottom row / translation 机械范围）+
   「标定状态」（calibrated: true + provenance 字段完整 + 20mm 验收 +
   record 一致）；ADR 0003、本票 item 6、T6 已同步。
3. **P0-3 陈旧障碍物使用模型**——Ticket #10 须输出
   `d_safe,eff = d0 + v_bound·age + σ_calib + σ_tracking` 及分档
   （age ≤ age_warn use+inflate；age_warn < age ≤ age_stop conservative；
   age > age_stop zero/latch）；`perception_timeout_s=1.0s` 明确为
   **provisional engineering value**，由 #10 验证/修订；
   静态外参无效与瞬时单帧异常分离——前者立即 invalid、不入感知避障。
   ADR 0003、本票 item 5/6、T3 已同步；#10（#7）已留言要求输出该模型。
4. **P1 ADR 0002 B/C 定义修正**——B = 全系统 fixed-world（感知+FK+CBF 全部
   独立环境系）；C = 感知 fixed-world + 控制器/CBF base_link（混合，未来
   移动基座演进路线）；修复后与 05B 原票定义一致。
5. **P1 LiDAR 默认关闭**——`perception_runtime.yaml` 回退
   `source_topic_lidar: ""` / `input_frame_lidar: ""`（与 spec 默认一致）；
   另设 `config/perception_dual_sensor_real.yaml` 显式 profile，
   仅 calibration gate（T5/T6/T7）通过后可用。
6. **P1 措辞**——ADR 0003/本票 5-6 节统一标为 Target/Decided behavior；
   当前实现状态（T1/T2/T3/T5/T6/T7 未完成：自体过滤 TODO、
   perception_valid 无人订阅、兜底链未改）在 MAP/ADR 标注，避免
   「规格写好了=实现好了」的误读。

**附带修复（旧问题）**：`PerceptionBridge.__init__` 中参数读取块位于
`FusionEngine(...)` 构造**之前**（工作树已验证：实际启动 source build 成功，
`PERCEPTION_BRIDGE_STARTED`；git main 2e21db4 上为乱序会 AttributeError），
随 fix commit 提交。
