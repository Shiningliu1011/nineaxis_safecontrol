# 感知输入失效策略：启动期 fail-fast + 运行期单帧丢弃 + 持续超时零速闭锁

2026-09 定案（05B）：感知输入（相机/激光雷达经外参变换到 base_link）是 OSCBF
动态避障 CBF 约束的唯一障碍数据源，其失效方式必须被显式定义，不能退化为隐式
静默降级。我们决定采用三层策略：(1) 启动期 fail-fast——静态外参做矩阵自检
（元素有限、det=+1、非单位阵）并与标定 provenance 记录比对（偏差 ≤20mm/5°），
再做已知尺寸物体 20mm 验收（见标定两段式流程），任一不通过则拒绝进入避障
模式；(2) 运行期单帧 TF/外参失效——丢弃该帧（CBF 使用上一帧 + d_safe 安全裕度），
不中断控制；(3) 持续失效——融合 age 超过 `perception_timeout_s`（1.0s）即经
`apply_qp_health_gate` 置零速并闭锁（锁存，须重启解除，符合 ISO 10218-1:2025
故障反应 configured stop cat.0/1 + restart interlock）。

**Considered Options**：①（本决定）1-3-4 分层；② 0.5s 有限重试窗口——被否：
重试窗口内 CBF 仍在使用陈旧数据，窗口只是把风险包装得更好看，且与「持续超时
零速」的阈值语义重叠冲突；③ 静默回退单位阵/当前 TF 兜底——被否：外参回退
单位阵意味着把传感器数据当作 base_link 原点处数据，几何含义完全错误；TF
查询链 current→latest→static→identity 的隐式兜底（原 `_sensor_to_world` 行为、
`perception_bridge.py` 中 static 参数为空时静默 `_identity_extrinsics()`）引入
不可追踪的静默降级，与安全失效语义相反。

**Consequences**：`/perception/status` 的 `perception_valid`（有传感器被采用且
fusion_age < 阈值）目前由 bridge 发布但无任何节点订阅，须接线到控制器健康通道
（T3）；运行时「超时→零速」需要控制器侧感知新鲜度检查（`/perception/tracks`
当前无时间戳、obs_state 永久缓存，T1 是 T3 前置）；启动期自检需要标定工具链
先行（T5 → T6）；「20mm 已知物体验收」与「外参 provenance 比对」为安全硬门槛，
机械测量（P6）仅为标定失败时的后备。本策略与 ISO/PAS 21448 SOTIF 的感知
不确定性处理对齐：把不确定性显式建模为失效条件，而非依赖回退猜测。
