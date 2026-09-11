# 感知年龄、几何膨胀与停止预算的适用边界

研究日期：2026-09-11。对应[感知时间同步与延迟模型](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/7)。本轮采用 research 技能核验三篇一手论文；仅补充决策证据，没有运行设备、修改控制实现或设定最终 age_warn / age_stop。承接[时间语义研究](perception-time-model.md)与[阶段交接](../handoffs/7-perception-time-model.md)。

## 一手证据及其实际支持范围

| 来源 | 已核验内容 | 本项目不能直接据此声称什么 |
|---|---|---|
| Dean et al., CoRL 2020 / PMLR 2021, [Guaranteeing Safety of Learned Perception Modules via Measurement-Robust Control Barrier Functions](https://proceedings.mlr.press/v155/dean21a/dean21a.pdf)，§3、Theorem 2、式(14)–(15) | 已知确定性状态估计误差集合时，需对 CBF 导数条件作鲁棒收紧；收紧量包含误差界、Lie 导数及 α∘h 的 Lipschitz 常数和输入范数。安全定理还要求有效 MR-CBF 和适当连续性。 | 不是延迟阈值表；米制位置误差不能直接充当混合位置/速度/关节状态的全状态误差范数；简单增加 d_safe 不自动实现该条件。 |
| Breeden, Garg, Panagou, [Control Barrier Functions in Sampled-Data Systems](https://public.websites.umich.edu/~dpanagou/assets/documents/JBreeden_LCSS21.pdf)，IEEE Control Systems Letters 6 (2022), §II、Theorem 1 | 固定周期、分段常值输入下，以整个采样间隔可达集上的导数变化界收紧条件。原文采用 h≤0 为安全，不能直接抄入 h≥0 的实现。 | 采样点 QP 可行不保证两次更新之间安全；固定 T 结论不自动覆盖 ROS 抖动、丢周期、命令排队、驱动实际保持方式。 |
| Cosner et al., IROS 2021, [Measurement-Robust Control Barrier Functions: Certainty in Safety with Uncertainty in State](https://andrewjamestaylor.github.io/assets/paper_materials/cosner2021measurement_robust_control_barrier_functions_certainty_in_safety_with_uncertainty_in_state/paper.pdf)，§II-B、式(7)、§III | backup 轨迹须全程在安全集内，并于有限时域末到达已知控制不变的 backup set；测量误差还须进入约束。 | 发出“停止”命令不等于已建立安全 backup；机器人静止也不保证面对继续接近的外部障碍永久不碰撞。 |

下列预算是依据运动学与集合包含关系的**本项目工程推导**，不是以上论文原样给出的机器人安全定理。论文证明与本项目待验证假设分开记录。

## 先固定几何的时间基准，再选择速度

记真实控制消费时刻为 t，实际几何观测时刻为 s，可信年龄上界为 A。A 必须覆盖时钟映射/曝光扫描语义误差；映射未知、超界未来时间、时钟失锁先判不可用，不能裁成零年龄。多源沿用逐源最旧有效贡献，不由融合发布日期刷新。时间字段的具体证据见[已有时间语义研究](perception-time-model.md)。

令 R(t) 为机器人完整碰撞几何，O(s) 为障碍几何，ε_geom 为与所用距离同单位的几何误差界。以下要求共同固定参考系，以及在整个时间区间内有效的**几何点运动界**；旋转/形变障碍不能仅使用质心速度。机器人也不能只用 TCP 速度代替所有可能碰撞连杆的速度。

**当前机器人 + 旧障碍：**若世界坐标障碍满足 O(t) ⊆ O(s) ⊕ Ball(v_o A)，则

`dist(R(t), O(t)) >= dist(R(t), O_hat(s)) - epsilon_geom - v_o*A`。

此处旧数据只在障碍一侧，age 项使用障碍自身运动界 v_o；已通过当前机器人状态重算的历史机器人位移不再收费。机器人状态本身若旧，另以它自己的年龄 A_r 和相应界 v_r 计入误差。移动相机的采集姿态/外参错误须另入 ε_geom；仅重新标记 frame_id 不构成运动补偿。

**旧时刻相对间距：**若使用的是 dist(R(s),O(s))，双方都旧，三角不等式给出下降界 `(v_r+v_o)*A`；也可用经过证明的更紧相对接近速度界 v_rel。它与前一种几何基准不同，不应在已经按 R(t) 重算的距离上再加机器人 s→t 的位移并称为必要补偿。

**预测到当前：**若 O_hat(t)=O_hat(s)+v_hat*A，应计预测残差界，而非把完整 v_o*A 与预测位移机械叠加。保守常速预测残差可写 `epsilon_p + epsilon_v*A + 0.5*a_o*A^2`，前提是初始速度误差及全区间加速度都有界。仅向前外推并重打时间戳不能更新最后测量年龄。

这些球膨胀只证明特定时刻的几何包络/间距下界，且只针对已有覆盖内的障碍；它们不证明未观测障碍不存在、身份关联正确，或遮挡后区域仍然安全。

## 为什么 v_bound × age 不是 CBF 证明

采用 h≥0 表示安全时，连续时间条件涉及 `L_f h + L_g h*u + alpha(h)`。MR-CBF 的一种充分收紧形式是

`L_f h(x_hat) + L_g h(x_hat)*u + alpha(h(x_hat)) >= epsilon_x*(L_Lfh + L_alphah + L_Lgh*||u||)`。

其中 ε_x 为定理状态空间范数下的误差界，各常数须在适用域有效；该式不能仅由位置球半径替代。输入受限时还必须在实际允许输入集合中有可行控制。若把时间变化膨胀写入 `h(x,t)=d(x,O_hat(s))-d0-r(A(t))`，帧间通常有 `dr/dt>0`，因此需处理 ∂h/∂t；将膨胀半径在每个 QP 内视为永远不变会漏掉这一项。上述 MR-CBF 形式和前提来自 [Dean et al. §3](https://proceedings.mlr.press/v155/dean21a/dean21a.pdf)；时间膨胀例子为本项目推导。

同样，控制消费后的下一更新/实际执行前，状态继续运动。sampled-data 条件需要在这段可达轨迹上控制导数变化；最大保持时长、抖动和丢周期处理都要进入实现假设。已有论文的固定周期定理提供分析路径，不是把平均频率代入就得到本系统证明。[Breeden et al. Theorem 1](https://public.websites.umich.edu/~dpanagou/assets/documents/JBreeden_LCSS21.pdf)

## 停止预算：按互不重叠区间记账

推荐先定义 `B_now = dist(R(t), O_hat(s))`。设 T_react 是从本次消费到有效制动开始的最大时间，**包含**最坏看门狗检测/轮询等待、QP/软件、命令传输及驱动响应中实际会经过的部分；T_brake 为随后制动持续时间上界。分开记录：

`B_now >= d0 + epsilon_geom + v_o*A + v_rel_react*T_react + D_r_brake + D_o_brake`。

其中 D_r_brake 为制动开始后机器人碰撞几何的最坏接近位移，D_o_brake 为同一制动区间内障碍最坏接近位移。若来自旧相对距离 B_old，才把前面的 v_o*A 换为适用于旧相对状态的 v_rel_age*A。这样更精确地限定了[旧交接通式](../handoffs/7-perception-time-model.md)中 B_i 的基准。

若 D_stop 已从“失效判定时刻”实测到停稳，且包含反应等待，不再额外加同一等待的 v_rel*T；若只包含机器人，仍需计障碍位移。所有项必须声明起止事件。简单 `D_r=v_start^2/(2*a_min)` 只适用于沿所关心接近方向的理想恒定/有保证最小减速度模型；实际多轴耦合、负载、重力、jerk、跟踪误差和机械制动均需对应证据，不能借此给真机停止距离定值。

在所有其他项固定且速度系数 v_age>0 时，几何准入上界可反解为：

`A_allow = (B - d0 - epsilon_geom - v_rel_react*T_react - D_r_brake - D_o_brake)/v_age`。

它依赖工况，非全局常量；分子非正意味着无正年龄准入余量。如果 age_stop 由轮询触发，超过阈值到检测到的超调时间必须已包含在 T_react，不能用刚好等于阈值的理想时刻漏算。age_warn 与 age_stop 的差需容纳经验证的降速响应，单有严格大小关系不足以保证有效预警。

上述仅给直到停稳的有限时域间距预算。停稳后若障碍可继续任意接近，停止状态未必控制不变；无限时域安全须另有环境约束或可执行 backup 策略。该区别与 [Cosner et al. 式(7)](https://andrewjamestaylor.github.io/assets/paper_materials/cosner2021measurement_robust_control_barrier_functions_certainty_in_safety_with_uncertainty_in_state/paper.pdf)的全程安全及终端不变集要求一致。

## 退化情形与证据缺口

- **速度未知：**跟踪输出缺失/为零/异常均不能替代确定上界。需要场景允许的硬界或另一个已验证保守包络；否则上述有限膨胀不成立。
- **速度真正为零：**只有证明相关对象在整个使用区间静止，age 位移项才为零；不能除以零求年龄阈值，也不能由此推导无限 TTL。覆盖、时钟、身份变化、测量完整性及机器人未来运动仍有独立期限。
- **初速为零但能加速：**位移项为 `0.5*a_bound*A^2`，不为零。已知初始速度上界 v0 时用 `v0*A+0.5*a_bound*A^2`；若另有全区间速度硬界 v_max，可取两种有效位移界的较小者。年龄预算相应求二次不等式，不能继续用单一常速反解。
- **本项目尚缺：**障碍完整几何运动界及适用场景、机器人各碰撞部位状态年龄/速度界、几何误差界、真实最大命令保持/响应时长、分工况停止轨迹与时间、鲁棒约束可行域和实际执行一致性、停止后的环境约束。旧数据中的速度估计 dt 缺口见[阶段交接](../handoffs/7-perception-time-model.md)，不能把估计速度提升为这些界。

因此本轮足以决定“预算必须携带几何基准、区间、速度界来源及未知状态”，仍不足以批准最终年龄秒数或宣告 CBF 安全性成立。
