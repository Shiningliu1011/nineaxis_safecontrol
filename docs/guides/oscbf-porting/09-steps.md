## 9. 实现步骤（从零开始，Step 1→9）

> 这是新项目的**主线实现顺序**，无分叉。🟢 步从 `portable_oscbf/` 复制参考代码改造；🟡 步按本文档规格（§4 模块 / §5 公式 / §6 参数 / §7 碰撞）从零实现。每步通过验收门再进下一步。

### Step 1: 建立机器人模型（JAX）【🟢 复制】
- 从 `portable_oscbf/work/nineaxis_manipulator_jax.py` 复制，改 `JOINT_CHAIN`/`home_pose`/`joint_limits`/`joint_max_velocities` 匹配你的 URDF
- 规格参考：§4.1、§5.1
- **验证**: FK 与 URDF 一致（关节 0 位姿、随机位姿抽查）；Jacobian 数值微分校验

### Step 2: 标定 OBB 包络盒 + DCOL alpha（自碰撞）【🟡 新建】
- 对每个连杆 STL 求对齐关节系 xyz 轴的 OBB：顶点主轴旋转 + 最小包围盒半边长（规格：§7.1、§4.1 `obb_collision_model.py`）
- 写入 `obb_collision_model.py` + `obb_model.yaml`；确定非相邻自碰撞对（含机械近邻豁免，L6/L71 流程）
- 接入 `dpax_collision.py`（DCOL 可微距离，从参考的 `DCOLuse/` 迁入热路径），逐对离线校准 DCOL alpha（V2 校准流程，§6.2）
- **验证**: OBB 与 STL 贴合（RViz 渲染重合；包络体积比接近 1）；零位无假碰撞；DCOL 距离/梯度与 FCL 基准一致

### Step 3: 接入轨迹【🟢 复制】
- 从 `portable_oscbf/work/ik_data_loader.py` 复制，适配你的轨迹数据格式（规格：§4.2）
- **验证**: 弧长路径几何生成成功，`path_geometry` 点/切线/姿态连续

### Step 4: 跑通控制内核（模块化 JIT）【🟢 复制 + 🟡 重构】
- 复制 `portable_oscbf` 的 `jax_control_facade.py` / `jax_kernel_factory.py` / `oscbf_velocity_config.py`（规格：§4.3、§5.2-§5.5）
- 🟡 把 `jax_kernel_factory.py` 从整块编译重构为各子函数独立 JIT（模块化，L94）
- **验证**: `qp_ok=True`、无 NaN、末端误差收敛；首次编译时间可接受（模块化图小）

### Step 5: 接入障碍物 + 弹性 QP【🟢 复制 + 🟡 改语义】
- 构造 `obs_*` 输入（固定 8 槽位，🟢 复制 `jax_barrier_terms.py`），障碍物距离走 DCOL，验证 JIT 不重编译（cache size 保持 1）
- 🟡 切换 `relax_cbf=True`（弹性 QP，取消受控停车——⚠️ 安全语义改动，需重新验证不可行场景行为，见 §5.5）
- **验证**: 障碍物逼近时 `dyn_min` 保持正裕度，正常工况 `δ_slack≈0`

### Step 6: 接入零空间策略【🟡 新建】
- 实现 `ManipulabilityMetric`（φ=½logdet，JAX 自动微分梯度）→ 用中心有限差分对照梯度（规格：§4.4、§5.2、参考 `OSCBF_在线可操作度零空间目标_架构参考.md`）
- `ManipulabilityGradientPolicy` 接入 `u_null`（替换关节中点）；首版 `activation.enabled=false`（§6.3）
- **验证**: 梯度有限差分相对误差 < 容差；零空间残差 `‖J·qdot_N‖` 小；可操作度上升行为可复现

### Step 7: 接入环境感知（可选）【🟢 复制】
- 复制 `point_cloud_obstacles.py` / `safety_snapshot.py` 转 `obs_*` / `sdf_*`（规格：§4.6、§7.2）
- **验证**: 感知链路端到端正常，`obs_*` 输入合法

### Step 8: 嵌入目标控制框架【🟢 复制】
- 在目标框架的控制周期内调用 `ctrl.path_tracking_step(q, path_state, obs_*, sdf_*)`（规格：§4.5 核心用法）
- **验证**: 100Hz 实测 p95 < 10ms

### Step 9: 调优与验收
- 按 §10 验收标准逐项核对；每个 🟡 改动单独验证后再组合

---
