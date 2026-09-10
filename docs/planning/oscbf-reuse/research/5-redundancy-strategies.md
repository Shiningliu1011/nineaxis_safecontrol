# 9-DOF 对 5D 工具轴任务：冗余策略对比

研究日期：2026-09-10。对应 [冗余策略对比：9-DOF 对 5D 工具轴任务](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/5)。本地审计基线 `786e0aec61b7abbf8e8c0a5c5aa99d20c3e54f21`。本文只研究候选与适配；不修改运行代码，不替后续决策票接受新的目标或优先级。9DOF（1P8R）、5D 工具轴任务、冗余优先与必要时减进给是已接受边界。

## 研究结论

保留现有 5D 任务表示、CBFpy/qpax 接口、OBB 和路径层作为比较基线。`use_nullspace_policy=false` 只关闭显式可操作性梯度策略，不能推出 QP 不使用冗余。QP 在九维关节速度中优化，任务与零空间项允许它分配自运动；但有限权重目标不保证严格任务优先。

最小可比较候选是现状、关节中位姿态偏好、现有可操作性梯度。后两者只改变名义输入，均仍需同一个安全 QP。上游 OSCBF 可复用目标构造思想；原类固定六维、依赖质量矩阵与特定 Manipulator，不能原样替换本地五维速度实现。CBFpy 与 qpax 更适合作为通用依赖边界。是否引入严格层级/路径进给变量须由后续决策明确行为要求，再依据实验选取。

依据：[既有九轴研究](nineaxis-redundancy-safety-priority.md)、[既有上游审计](control-upstream.md)、以下源码核验与独立实验。旧研究中只凭 `relax_cbf=True` 的安全描述在本票进一步沿入口核实。

## 当前实现到底做了什么

| 观察 | 本次源码证据 | 含义 |
|---|---|---|
| ROS 默认 `task_mode=tool_axis_5d`、`use_nullspace_policy=false`、`damping=0.05`；每周期 `q_des=q` | [控制配置](../../../../config/oscbf_controller.yaml)、[ROS 控制器](../../../../src/robot_safecontrol_moveit/oscbf_controller.py) | 默认显式姿态偏好为零；不等于九维 QP 的冗余被禁用 |
| `u_nom=J# v + N qdot_null`；名义速度最终逐关节限幅 | [tracking_step](../../../../portable_oscbf/work/jax_kernel_factory.py) | 投影与限幅都要纳入跟踪误差解释，不能只看理想零空间公式 |
| 跟踪的 Hessian 用当前五维任务 Jacobian；备用 `step()` 仍用六维 | [task_hessian_from_jacobian / task_hessian_from_q](../../../../portable_oscbf/work/jax_kernel_factory.py) | 性能/策略实验须走 tracking 或 path_tracking；只调 `step()` 不能代表生产 5D 路径 |
| 目标为任务差、零空间差与时序近端加权和 | [P/q](../../../../portable_oscbf/work/oscbf_velocity_config.py)、[kernel](../../../../portable_oscbf/work/jax_kernel_factory.py) | `w_joint=0.1` 为零空间偏差权重，并非自动最大化限位裕度；时序项也可能与当前任务折中 |
| 奇异 CBF 用位置 Jacobian 三个奇异值的乘积减 `0.005` | [h_2](../../../../portable_oscbf/work/oscbf_velocity_config.py) | 它约束三维位置可操作性，不能证明五维任务满秩或轴向跟踪能力 |
| 现有梯度按完整六维 `robot.ee_jacobian` 的正则化 logdet 求导，传入的 N 来自五维 DLS；策略内部投影后 caller 再投影 | [policy](../../../../portable_oscbf/work/nullspace_policy.py)、[metric](../../../../portable_oscbf/work/manipulability_metric.py)、[kernel](../../../../portable_oscbf/work/jax_kernel_factory.py) | 六维指标会关心用户未要求的滚转；DLS N 不幂等，重复投影也不等于严格零空间保持。应测任务泄漏，不能将该策略称作现成五维奇异回避 |

`J#=Jᵀ(JJᵀ+damping I)⁻¹` 是名义跟踪实现；任务 Hessian 内另用 `task_damping²=10⁻⁶`。二者阻尼不同，不能混成同一个投影。由公式可得 `JN = damping (JJᵀ+damping I)⁻¹J`，一般非零，尤其在奇异附近须量化泄漏。[源码](../../../../portable_oscbf/work/jax_kernel_factory.py)

### CBF 硬约束的实际入口

`NineaxisOSCBFVelocityConfig` 的确配置 `relax_cbf=True`、惩罚 `1e5`。但 kernel 的 `enable_rate_limit=True` 分支会改用硬 `qpax.solve_qp`：CBF 与速度盒行不带松弛，仅两个共享速度变化率松弛进入增广 QP。没有 rate limit 时则走 elastic 求解，并以收敛与有限值门控，`delta_slack` 是诊断。[配置](../../../../portable_oscbf/work/oscbf_velocity_config.py)、[组装与求解](../../../../portable_oscbf/work/jax_kernel_factory.py)

本次审计的 ROS `oscbf_controller` 构造 `JaxControlLoop` **没有传入 `rate_limit_du_max`**；facade 默认 `None`，因而 `enable_rate_limit=False`。所以这一入口目前是 elastic CBF 分支。YAML 中 `soft_rate_limit.enabled=true` 不能独自证明节点启用了硬分支。直接传入正 rate limit 的其他调用则会走硬分支，必须逐入口陈述。[ROS 构造](../../../../src/robot_safecontrol_moveit/oscbf_controller.py)、[facade](../../../../portable_oscbf/work/jax_control_facade.py)、[YAML](../../../../portable_oscbf/config/nineaxis.yaml)

即使硬分支求解成功，也只说明当前离散实现及数值容差内满足构造的问题，不自动成为连续时间、感知误差或执行延迟下的安全保证。CBF 的保证条件见 [Ames 等原论文](https://arxiv.org/abs/1903.11199)。

## 五维秩、冗余与奇异

工具轴任务是位置三维加单位轴方向两维，自由滚转已包含在零空间内。局部有效的 `J₅∈R⁵ˣ⁹` 满秩时 `dim ker(J₅)=9−5=4`；奇异处应按实际 rank 计算。零空间维数增加并非性能改善：它可能意味着某些任务方向已无法实现。可行自运动还受速度盒、关节边界、OBB/环境约束的共同限制。[本地五维实现](../../../../portable_oscbf/work/tool_axis_task.py)、[既有推导](nineaxis-redundancy-safety-priority.md)

工程评价应同时记数值秩、缩放后最小奇异值、位置/轴向误差、关节速度饱和和 `||J₅ qdot_N||`。P 关节与 R 关节单位不同，位置行与角度行尺度也不同；rank 阈值、特征长度与关节速度归一化必须随数据一并记录，不能跨尺度直接比较条件数。轴方向误差的局部切平面表达也需要区分机械奇异与表示退化，不能把一次秩降低笼统命名为机械奇异。[任务实现](../../../../portable_oscbf/work/tool_axis_task.py)

## 可复用上游的固定版本

2026-09-10 通过 GitHub 官方 API 核验默认分支 SHA，并读取下列固定源码；版本是该 commit 的包元数据，不声称等于已发布 wheel。OSCBF、CBFpy 与前次快照相同。

| 上游 | SHA / 包版本 / 许可 | 可直接复用 | 最小适配与验证 |
|---|---|---|---|
| StanfordASL/oscbf | `6082329f8e1d61ee8e875a63a5cd80a14d513e12` / 0.0.1 / MIT | 任务一致加权目标设计、CBFpy 接口模式 | 原 `OSCBFVelocityConfig.task_dim=6`，assert 专用 Manipulator，并调用私有 Jacobian/质量矩阵接口；保留本地 5D、处理速度级几何 DLS 与动力学一致逆的区别。依赖锁 `jax/jaxlib==0.4.30, cbfpy==0.0.1` |
| StanfordASL/cbfpy | `f5bff8b93609d6a164fd8beeef962c92e4476dcb` / 0.0.4 / MIT | `CBFConfig`、barrier 自动微分、QP 数据生成与求解入口 | 不负责九轴任务优先级；核旧 0.0.1 monkey patch、松弛/返回值、动态障碍时间项，不把升级视作数值等价 |
| qpax-solver/qpax（原 kevin-tracy 路径重定向） | `a5721f8889d855480162dd2e946409a5b93cf9b6` / 0.1.4 / Apache-2.0 | JAX QP 求解、等式/不等式、elastic API | 当前含 explicit/implicit backend，默认 explicit；不提供机器人目标与自动严格层级。升级须核 frozen QP、返回值、终态健康检查和 JIT/性能，不因同名 `solve_qp` 就直接替换 |

来源：[OSCBF 配置](https://github.com/StanfordASL/oscbf/blob/6082329f8e1d61ee8e875a63a5cd80a14d513e12/oscbf/core/oscbf_configs.py)、[包](https://github.com/StanfordASL/oscbf/blob/6082329f8e1d61ee8e875a63a5cd80a14d513e12/pyproject.toml)、[许可](https://github.com/StanfordASL/oscbf/blob/6082329f8e1d61ee8e875a63a5cd80a14d513e12/LICENSE)；[CBFpy 包](https://github.com/StanfordASL/cbfpy/blob/f5bff8b93609d6a164fd8beeef962c92e4476dcb/pyproject.toml)、[CBF 源码](https://github.com/StanfordASL/cbfpy/blob/f5bff8b93609d6a164fd8beeef962c92e4476dcb/cbfpy/cbfs/cbf.py)、[许可](https://github.com/StanfordASL/cbfpy/blob/f5bff8b93609d6a164fd8beeef962c92e4476dcb/LICENSE)；[qpax 包](https://github.com/qpax-solver/qpax/blob/a5721f8889d855480162dd2e946409a5b93cf9b6/pyproject.toml)、[solve_qp](https://github.com/qpax-solver/qpax/blob/a5721f8889d855480162dd2e946409a5b93cf9b6/qpax/pdip.py)、[许可全文](https://github.com/qpax-solver/qpax/blob/a5721f8889d855480162dd2e946409a5b93cf9b6/LICENSE)。qpax GitHub license API 返回 NOASSERTION，但该固定 LICENSE 正文明示 Apache-2.0，故以正文为准。

## 候选策略与接口改动面

以下为据源码的工程推断，性能收益以实验为准。

| 候选 | 可保留/直接复用 | 必要最小适配 | 预期收益与验证重点 |
|---|---|---|---|
| 现状：零姿态偏好 + 加权安全 QP | 全部现有入口 | 无 | 控制基线；测自然重分配能力，不能把小关节项等同限位目标 |
| 关节中位/工作姿态偏好 | 已有 `q_des` + `kp_joint` 路径 | 固定或配置参考，优先按关节范围/速度归一化；仍经同一 QP | 增加限位裕度、减少慢漂；可能牺牲跟踪或进入其他碰撞/奇异，不能保证中位姿态全臂安全 |
| 现有可操作性梯度 | `ManipulabilityGradientPolicy` 与 JAX autodiff | 直接调用用于对照；生产候选需验证五维指标、限位正则、二次投影与限速 | 可能改善操作性，也可能驱向限位。配置已提示此类历史表现，本票不能无条件开启 |
| 严格层级/词典序 QP | qpax 等式/不等式与现有 CBF 行 | 在高层最优解集内优化低层目标；定义容差/不可行及进给变量 | 若必须“有保持任务的解就不因姿态偏好损害它”，表达更准确；需额外层级求解、最坏时延与冲突验证 |

上游 OSCBF 论文目标是加权和，而 HQP 将低层限制在高层最优解集；增大有限权重只能改变折中，不等于严格优先。[OSCBF 论文](https://arxiv.org/html/2503.06736v2)、[Escande 等作者机构全文](https://gepettoweb.laas.fr/uploads/Publications/2014_escande_ijrr.pdf)

SNS 文献中的“约束内利用冗余、必要时缩放任务”提供行为实现参考，但关节速度饱和的 SNS 不是本地动态 OBB/ESDF CBF 的现成替代。若用路径参数速度调节，应保持路径几何/工具轴并重新求安全控制；不能把已求出的九维速度任意统一缩小。[Flacco 等作者机构全文](https://iris.uniroma1.it/retrieve/e383532a-4954-15e8-e053-a505fe0a3de9/Flacco_Postprint_Control-of-Redundant_2015.pdf)、[既有研究](nineaxis-redundancy-safety-priority.md)

## 验证口径与后续决策输入

本票已完成三个现有接口候选、两个既有测试初态的固定端点局部 Euler 实验：每组 300 步、dt=0.01 s，无外部障碍，内置 CBF 几何保留，沿当前 ROS 同类 elastic 分支。CPU、JAX/JAXlib 0.6.2、CBFpy 0.0.1、qpax 0.1.4。初态与源码哈希见 [metadata](5-redundancy-evidence/metadata.json)，完整参数及指标定义见 [证据说明](5-redundancy-evidence/README.md)，结果见 [summary](5-redundancy-evidence/summary.json)，复现见 [compare.py](5-redundancy-evidence/compare.py)。

| 策略 / 初态 | 最大位置误差 mm | 最大轴误差 rad | 最大轴角速度 rad/s | 归一限位裕度（初→最小） | 缩放 σ₅（初→末） |
|---|---:|---:|---:|---|---|
| default / tracking_start | 1.8767e-06 | 6.664e-08 | 4.317e-08 | 0.14850→0.14838 | 0.37826→0.37808 |
| default / roll_test_start | 9.5565e-06 | 9.996e-08 | 6.5255e-07 | 0.07983→0.07942 | 0.47532→0.47732 |
| joint_midpoint / tracking_start | 0.030547 | 0.00093423 | 0.009185 | 0.14850→0.13390 | 0.37826→0.32520 |
| joint_midpoint / roll_test_start | 0.49321 | 0.0017619 | 0.011876 | 0.07983→0.07192 | 0.47532→0.49484 |
| existing_manipulability / tracking_start | 0.46338 | 0.00077048 | 0.0037234 | 0.14850→0.13830 | 0.37826→0.33626 |
| existing_manipulability / roll_test_start | 0.05829 | 0.00013086 | 0.00079047 | 0.07983→0.07208 | 0.47532→0.50745 |

六组各 300 步 QP 均通过当前门控，五维数值秩始终 5，内核全部 G 行的主动步比例均为 0（该统计并非独立的纯 CBF 主动率），最大 delta_slack 为 6.99e-9。中位姿态和现有梯度在两个初态均降低最差关节限位裕度；五维最小奇异值一组降、一组升，因此本次数据不支持将任一策略直接默认启用。六维正则 logdet 梯度与五维最小奇异值本来就是不同目标，二者变化不一致不能单独判定算法错误。

这只是局部无冲突自运动证据：未触发活跃安全约束，也未测完整路径、动态障碍、真实 plant 或实机、最坏性能；不能据此排出完整任务上的策略优劣，不能声称避障、严格层级或安全验收通过。

后续需要真人决定的是剩余目标的严格性与优先表达：姿态/限位/可操作性之间如何取舍；允许的工具误差与进给调节范围；是否必须严格高层最优保持。无需重新投票 5D 或指定某关节独立运动。研究建议先用现有接口比较，再在收益或严格性要求明确时扩展层级，而不是因九轴直接替换整套控制库。
