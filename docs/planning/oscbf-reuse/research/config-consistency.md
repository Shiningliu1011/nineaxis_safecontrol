# 配置一致性清理：消费链复核与验收契约

日期：2026-09-11；读取时 HEAD：`84797408ac0cbe344bcb9f06fc70da5ae29dd417`。只读当前工作树一手源码；未执行 ROS、JAX 数值测试、参数修改或实机操作。下列源码引用为仓库根相对路径及读取时行号。

## 结论及交接校正

[配置一致性清理交接](../handoffs/12-config-consistency.md)的核心结论仍成立：不能把实施简化成“关节 alpha 从 8 改为 5”。这些可见字段不是两份已经生效的配置；实际统一 CBF 基准为 10，障碍输入另带每槽 alpha。

定位细节需校正：nineaxis 顶层键是 `controller`，不是 `control`；ROS 跟踪入口是 `step_once → path_tracking_step`；主 ROS 障碍回调自行解码第十槽，没有调用 `obstacle_extractor` 的解码函数（仅导入槽位常量）。测试应覆盖生产入口，而非仅覆盖共享格式工具。

## 参数真正来源

| 参数 | 一手源码与判断 |
| --- | --- |
| 关节/碰撞/奇异性 alpha | `portable_oscbf/config/nineaxis.yaml:135` 定义关节值 5；`portable_oscbf/work/oscbf_velocity_config.py:81` 定义属性 8/5/5。Python/YAML/shell 字段搜索仅发现定义，没有属性读取，不能认定关节实际用 8。 |
| 基准 CBF alpha | `portable_oscbf/work/oscbf_velocity_config.py:111` 为 10，`:341` 的 alpha(h) 用它乘 h；`portable_oscbf/work/jax_kernel_factory.py:100` 获取，`:200` 用它从未修正的 QP 上界还原 h。逐类增益变更会改变这一前提。 |
| 障碍 alpha | `src/robot_safecontrol_moveit/perception_bridge.py:497` 写第十槽 1.5；`src/robot_safecontrol_moveit/oscbf_controller.py:387` 解析；`portable_oscbf/work/jax_control_facade.py:800` 缺省补 10。`portable_oscbf/work/jax_barrier_terms.py:70` 修正非聚合行；`:84`、`:96` 计算 softmin 加权 alpha 与聚合修正。`nineaxis.yaml:195` 的 cbf_alpha 无按字段读取证据。 |
| 跟踪 kp | `config/oscbf_controller.yaml:19` 为 160/10/0.45 → `launch/mujoco_transition_final.launch.py:161` 加载 → controller `:196` declare、`:395` get → facade `:601` → kernel `:635`、`:650`。controller `:398` 的 q_des=q 会令普通关节回中项为零，不能用输出不变判断 kp_joint 未传播。 |
| 权重/近端项 | YAML `:33` 为 40/10/0.1、lambda=0.2；controller `:305` 传给 facade，facade `:305` 将 lambda 传给 CBF config。facade `:99` 的离线默认 w_pos=20、lambda=0 和 `:383` 的 warmup kp_pos=50 不是生产来源。 |
| dt | controller `:153` 默认 0.002，生产 YAML `:9` 为 0.01；controller `:306` → facade `:330` → kernel `:353` 安全积分。dt_path 均为 0.01（controller `:166`、YAML `:11`），kernel `:582`、`:687` 用于路径推进。timer 独立使用 controller `:112` 的 1/publish_frequency_hz。 |
| nineaxis controller 字段 | `src/robot_safecontrol_moveit/oscbf_trajectory.py:62` 与 `portable_oscbf/work/ik_data_loader.py:83` 读取 kinematics，没有读取 controller 增益。`nineaxis.yaml:125` 的固定/非固定 kp 不是 ROS profile；`:212` 的 EMA alpha=0.8 是平滑系数，不是 CBF 增益。 |
| 旧 torque 文件 | `portable_oscbf/config/controller_params.yaml:1` 为 torque/dt/kp/kd 配置；显式文件名搜索无消费者。但 `setup.py:46` 通配安装全部 portable config YAML，不能称其未打包或无外部消费者。 |

主代理 AST 核验记录：`output/audit-12/research-refresh-20260911.json`（本机证据）。24 个双方共有、可静态解析的字面量默认项唯一差异是 dt；另有 6 个 YAML 项对应非字面量默认、4 个节点独有项。这不等于“全参数一致”。YAML `:3` 的全部值与节点默认一致声明不实。

## 路径和生命周期的新核验

controller `:150` 在声明时从 share_dir 计算 portable root 和配置路径；`:200` 对空路径分别回退 `_default_parameters`。仅覆盖 portable_oscbf_root、portable_config_yaml 留空时，后者仍来自 share 默认路径，不会从覆盖后的 root 派生。YAML `:15` 的 `<portable_oscbf_root>/config/nineaxis.yaml` 注释不适用于这个组合。

controller `:305` 建 loop 时读取 dt/权重/lambda，`:112` 建 timer 时读取频率，`:395` 每次 step 读取 kp。整理应明确启动参数与运行时更新的区别，不能因 ROS 能设置参数便承诺 kernel/timer 会重建。

硬件 launch `:179` 已传 YAML/mode，但 `src/robot_safecontrol_moveit/hardware_bridge.py:63` 使用 Python 构造参数、`:67` 默认 shadow，未发现 declare/get。该项由[真机执行链路交接](../handoffs/13-socketcan.md)负责，机械整理不顺带接通真实模式。

## 测试已有内容与缺口

- `tests/test_oscbf_controller_smoke.py:54` 使用手写 overrides，未加载生产控制 YAML；`:109` 验证单步有界且可行，不证明生产参数传播。
- `portable_oscbf/tests/test_jax_cbf_h_reuse.py:8` 验证原始 QP 上界除基准可还原 h，不覆盖 YAML/ROS，且缺 cbfpy 会跳过。
- `portable_oscbf/tests/test_jax_smooth_dynamic_barrier.py:8` 验证聚合形状和保守性，使用基准 alpha，不证明 ROS 第十槽到最终上界。
- `portable_oscbf/tests/test_jax_dynamic_obstacle_contract.py:16` 整模块 skip；`:107` 的 alpha/time-term 上界断言不能当作当前已执行的保护。

## 后续可执行验收（建议，未执行）

1. 从真实生产 YAML 构造 ROS 参数，并覆盖与默认不同的合法哨兵值；观察 loop 构造与 path_tracking_step 实参，覆盖 dt、dt_path、权重、lambda、kp、damping。另测无 YAML 入口保留 dt=0.002。不强求离线与生产 profile 数字相同。
2. 覆盖路径四组合：全留空、仅 root、仅 config、两者均覆盖；锁定当前“仅 root 不派生 config”的事实。若改派生规则，应标为兼容性行为变更。安装后从 share 加载一次，不能只查源文件存在。
3. 用非默认第十槽 alpha 经真实 ROS 回调、facade 到 QP；固定 q、几何和运动，仅改变 alpha。非聚合障碍行满足 `Δrhs=Δalpha*h`；聚合行按 softmin 权重满足 `Δrhs=Δalpha_aggregate*h_aggregate`。非障碍行不变、全禁用无修正。保留未修正上界还原 h 的测试。不能只比较最终 u，约束不激活时它可不变。
4. 独立扰动 dt、dt_path、频率，观察积分、路径推进和 timer 各自来源。仅在无裁剪、QP 可行状态断言 `q_next-q=dt*u_safe`；其他状态遵循安全输出策略。不强制三个时钟相等。
5. 按 controller `:214` 的既有允许域检查非法 dt/频率/增益/权重/damping/solver_tol；kernel `:96` 检查 dt_path 有限且为正。lambda、每槽 alpha 若需新增校验，先明确允许域和校验归属，不把建议说成已有行为。运行中更新另立契约。
6. 行为不变范围优先改错注释、标明字段状态和 profile 来源，保留旧文件与数值。抽取配置表示时比较前后有效参数及关键 QP 数据；只改说明不需新增数值测试。传播测试用于后续接线或重构，本轮静态研究不替代执行验收。

负面搜索仅限当前源码和显式字段名，未排除外部脚本、反射与第三方消费者。本研究不选定新 alpha、不统一周期、不改变硬件接线，不证明现值适合真机。
