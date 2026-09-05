# 感知输入失效策略：启动期 fail-fast + 运行期单帧丢弃 + 持续超时零速闭锁

2026-09-05 定案（05B；当日 post-review 修订 P0-2/P0-3/措辞）：感知输入（相机/激光
雷达经外参变换到 base_link）是 OSCBF 动态避障 CBF 约束的唯一障碍数据源，其失效
方式必须被显式定义，不能退化为隐式静默降级。策略分两层语义：**静态外参本身
不可信**与**运行期瞬时数据异常**必须分开处置；总体分三层：
(1) 启动期 fail-fast；(2) 运行期单帧异常 → drop；(3) 持续超时 → 零速闭锁。
**注：本 ADR 描述的是 Target behavior（decided）——T1/T2/T3/T6 未对接前，当前
代码实现的仍是 05A 审计到的旧行为（`_sensor_to_world` 的
current→latest→static→identity 兜底、`perception_valid` 无人订阅、obs_state
永久缓存），不得据本 ADR 推断「已实现」。**

**Considered Options**：①（本决定）1-3-4 分层；② 0.5s 有限重试窗口——被否：
重试窗口内 CBF 仍在使用陈旧数据，窗口只是把风险包装得更好看，且与「持续超时
零速」的阈值语义重叠冲突；③ 静默回退单位阵/当前 TF 兜底——被否：外参回退
单位阵意味着把传感器数据当作 base_link 原点处数据，几何含义完全错误；TF
查询链 current→latest→static→identity 的隐式兜底（原 `_sensor_to_world` 行为、
`perception_bridge.py` 中 static 参数为空时静默 `_identity_extrinsics()`）引入
不可追踪的静默降级，与安全失效语义相反。

**Consequences**：

- **启动期 fail-fast（静态外参不可信 ⇒ 不允许进入感知避障）**：分两组校验，
  非单位阵**不是**有效判据——数学上合法的刚体外参就可能是 identity（例如传感器
  坐标系被定义为与 base_link 重合），identity 是不是 placeholder 只是配置管理
  约定，不是 SE(3) 几何合法性规则；「matrix != identity」不能替代「这份数据
  真的被标定过」：
  - **数学合法性**：元素均有限；`R^T R ≈ I`（按残差阈值）；`|det(R)| ≈ 1`；
    bottom row ≈ [0 0 0 1]；translation 在机械合理范围内（按 workspace + 支架
    物理包络设定）。
  - **标定状态（provenance 完整）**：`calibrated: true`；`sensor_serial`、
    `calibration_id`、`method`、`timestamp`、`operator`、`residual`、
    `error_estimate`、`frame_from`、`frame_to` 完整且与记录一致；与 provenance
    记录比对偏差 ≤20mm/5°；已知尺寸物体 20mm 验收通过。
  - **记录一致性**：被校验的 calibration record == 实际 runtime 加载的
    record（`calibration_id`/内容 hash），见 ADR 0004（Calibration SSOT）。
  - 任一失败 → 拒绝进入避障模式（fail-fast，不进静默 identity）。
- **运行期（仅瞬时数据异常）单帧失效 → drop frame**：CBF 沿上一帧有效障碍
  + 安全裕度继续；瞬时异常指传感器丢帧/单帧解码错/单帧 TF 查询失败，
  **不**包括静态外参校验失败——后者属于「不确定可信度」，必须立即判 invalid
  并退出感知避障，不允许「drop 一帧继续」。
- **持续失效 → 零速 + 锁存**：`perception_timeout_s` **当前取 1.0s，是
  provisional engineering value，不是最终安全合同**——最终值由 Ticket #10
  （感知时间同步与延迟模型研究）给出并被验证/修订：`#10` 须输出陈旧数据使用
  模型 `d_safe,eff = d0 + v_bound·age + σ_calib + σ_tracking`（v_bound 为
  障碍最大相对速度、σ 为标定/跟踪不确定性）及分档
  `age ≤ age_warn → use + inflate`；
  `age_warn < age ≤ age_stop → conservative behavior`；
  `age > age_stop → zero/latch`。在 #10 结算前，age_stop 暂用 1.0s
  （perception_timeout_s）工程值并标注 provisional。达到阈值后经
  `apply_qp_health_gate` 置零速并闭锁（锁存，须重启解除，符合
  ISO 10218-1:2025 故障反应 configured stop cat.0/1 + restart interlock）。
- 无法证明可信度的数据（静态外参无效、provenance 缺失）**不参与**
  perception-based avoidance，而不是「用旧的数据继续」。

本策略与 ISO/PAS 21448 SOTIF 的感知不确定性处理对齐：把不确定性显式建模为
失效条件，而非依赖回退猜测。
