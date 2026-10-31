# 已有规格、模块边界与 Ruckig 核验

核验日期：2026-10-31。范围为本地规格、默认代码及大地图相关任务的只读检查。未运行仿真、设备或替代算法测试。

## 直接继承的要求

- `docs/specs/config_consistency_spec.md:50/69/79`：配置负责装配、校验与记录；消费者使用同一份有效配置，运行中参数固定；传播验证覆盖真实入口。`dt`、`dt_path`、发布频率具有各自物理含义。
- `docs/specs/dcol-collision-core.md:108/128/136`：双源分别保存时间、原点、标定和来源；对应采集时刻进行过滤；场景服务是唯一写入者，发布不可变场景；碰撞采用 `prepare_scene/query/certify`。
- 同规格 `:88/96`：当前本体有 36 组非相邻 link pair，工具与附件按归属生成；自碰撞 10 mm、环境 30 mm，几何外扩后采用 `h=alpha-1`；碰撞行不可松弛。用户当前选择将机器人几何明确为 OBB。
- `docs/specs/off02_obb_geometry_admission_spec.md:36/107/241`：SAT、身份绑定与局部区域核验可供独立验证。其原 14+22 分区和局部证据仅适用于原范围，公共完整查询按当前碰撞规格执行。
- `docs/specs/dual_sensor_perception_fusion_spec.md:168`：独立缓冲、重复观测处理、`alive/used` 与标定身份可复用。动作资格由当前碰撞规格规定的覆盖、时间和完整运动决定。
- 当前清理决定按有效用途保留资产。历史兼容别名和配置保留要求不自动扩展为长期业务实现。

## 大地图的限制与执行决定

本次通过 GitHub connector 实际读取 [#147](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/147)、[#138](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/138)、[#143](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/143)、[#156](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/156)、[#157](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/157)，以及 #143 全部评论。#138/#143 为已完成决定；#156/#157 仍为开放实现任务。

- #138 管理逐轴 `max_velocity/max_acceleration/max_jerk`，三种方法、预测、实时、最终运动和评价共用。
- [#143 完整方案](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/143#issuecomment-6442638255)定义 M09 的目标速度变量；速度上限和局部可达范围进入 QP，加速度和 jerk 依据有效 `qdot/qddot` 与周期形成必要约束。最终 Ruckig 跟踪和停止运动仍需完整检查。
- [#143 确认记录](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/143#issuecomment-6442700214)确认了局部必要工艺与关节限制、实际 Ruckig / C2 参考完整检查，以及 #156/#157 的实现分工。
- #156 负责最终短段、工艺、任务、关节、碰撞和停止检查，以及实际采用区间与反馈。#157 负责五维联合 QP 及运动参考协调。

大地图已有速度、加速度和 jerk 的限制要求；现有确认方案同时保留连续运动生成。所查记录没有确定将三类限制全部改写为 CBF 后直接删除 Ruckig。

## 当前代码的控制量与 Ruckig 用途

| 位置 | 事实 |
|---|---|
| `oscbf_velocity_config.py:26/153/167` | 状态为 `q`，控制为 `qdot`，直接输入上下界为关节速度；没有完整速度、加速度状态。 |
| `oscbf_controller.py:431/501/555` | 默认未启用 rate limit，控制消费者读取位置，命令发布位置。 |
| `jax_kernel_factory.py:256` | 可选 rate limit 约束相邻目标速度变化，含可放松变量；没有 jerk 的完整运动保证。 |
| `jax_barrier_terms.py:111` | 数值更新为 `q+u*dt`；无效解置零速度的数值处理不生成连续停止运动。 |
| `driver_simulator.py:43`、`oscbf_plant.py:58` | Ruckig position interface 生成有速度、加速度和 jerk 限制的仿真运动。 |
| `static_motion.py:136/155/183/220` | Ruckig velocity interface 生成跟踪与停止；检查完整区间以及逐轴运动上限。 |
| `static_motion_bounds.py:47` | 读取 Ruckig profile 取得连续分段范围。 |
| `static_execution.py:31/78` | 检查状态接续与证据，执行 `motion.at_time()`；无新命令时继续已生成的停止。 |
| `models/ninezzhou_moveit_config/config/ompl_planning.yaml:9` | 规划时间处理使用 Ruckig smoothing。 |
| `collision_safety.py:117/457` | 当前公共连续输入未表达全部运动分段，`certify` 返回失败；尚无直接替代验证能力。 |

路径省略部分均为 `portable_oscbf/work/` 或 `src/robot_safecontrol_moveit/`，完整源码导航见 [模块审查](module-audit.md)。

## 当前决定与取消条件

用户希望取消 Ruckig，并允许在暂时无法取消时保留。当前保留其必要功能并集中依赖。取消需要新的控制与运动生成实现能够从有效 `q/qdot/qddot` 连续接续，提供完整跟踪和停止、速度/加速度/jerk 的区间证据、任务与碰撞检查、超时及命令中断处理、执行反馈与进度一致性。

`docs/planning/nineaxis-full-project/mpc-solver-research.md:30/41/95` 已研究常值 jerk 区间积分，但该探针未覆盖完整工具高阶限制、未来碰撞、连续证明和实际执行。它尚不能作为取消 Ruckig 的完成证据。本次没有把取消登记为已实现，也没有改变共同限值或发布远程变更。
