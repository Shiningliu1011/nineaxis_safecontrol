# 复用优先重梳说明

2026-09-07。状态：本地可审阅草案；未发布 GitHub，未执行算法迁移。工作区原有大量未提交改动。初轮在临时目录形成材料，随后按用户要求迁入 docs/planning/oscbf-reuse；迁入不改变决策状态。远端地图经公开 API 读取存为 previous-remote-map.md；gh 未登录。

## 先纠正项目地图的事实基线

以下是本轮源码静态审计，不是实机或测试运行结果。路径相对仓库根目录。

| 事实 | 证据 | 对地图的影响 |
|---|---|---|
| 已使用 CBFpy/qpax | `portable_oscbf/work/oscbf_velocity_config.py:23`；`jax_kernel_factory.py:198` | 替换重点是重复的配置/汇编与模型封装，不能宣称现在全是自研求解器 |
| 生产只消费 tracks、未启用距离场 | `src/robot_safecontrol_moveit/oscbf_controller.py:118`、`:305`；`jax_control_facade.py:125` | 将连续障碍覆盖列为首要接口决策 |
| static 进入距离场，unconfirmed 进入 tracks | `portable_oscbf/work/fusion_engine.py:268`；`static_occupancy.py:101` | 据代码推断：静态确认可能令障碍从当前被消费表示消失；必须有回放反例测试，不能继续先扩展跟踪器 |
| 8 槽按团块点数截断 | `portable_oscbf/work/dynamic_clustering.py:87`；`config/perception_runtime.yaml:81` | 容量溢出、薄小障碍和覆盖必须成为接口验收项 |
| ROS 缓存 tracks 无消息年龄检查 | `src/robot_safecontrol_moveit/oscbf_controller.py:374`、`:456` | typed observations 和失效处理应在统一契约下接线 |
| 目前距离场是无符号 EDT | `portable_oscbf/work/safety_snapshot.py:211` | 不把“已有 ESDF”视作完整距离场建图、自由空间或未知空间语义已解决 |
| QP 后还有位置低通、执行端动态 | `src/robot_safecontrol_moveit/oscbf_controller.py:579`；`oscbf_plant.py:211` | 验收必须覆盖最终执行命令，不只 QP 可行 |
| 默认配置允许 CBF 松弛 | `portable_oscbf/work/oscbf_velocity_config.py:158` | 不能把默认约束写成严格硬安全约束 |
| 自研 warm-start 已禁用 | `portable_oscbf/work/jax_control_facade.py:140` | 不为非生产路径单独安排迁移 |
| 当前 tick 使用最新反馈 q | `src/robot_safecontrol_moveit/oscbf_controller.py:455` | 旧地图“仅内部自积分”的描述不可直接继承 |

历史测试数量、p95、硬件状态均保留为历史证据，本轮不声称重新验证。远端“选定控制率/延迟预算与实现策略”仍 open，而本地已记录 50Hz/20ms 决议；恢复 tracker 写入后应核对原评论与本地记录，不能重复询问或自动关闭。

## 复用职责草案

| 层 | 优先复用 | 本项目保留/适配 | 定案前需证明 |
|---|---|---|---|
| 传感器 | 厂商 Livox/Orbbec ROS2 驱动 | launch、设备参数、标定记录 | 精确型号、版本、Humble、时间戳/输出格式 |
| 坐标与自体过滤 | TF2、经核验的 ROS2 自体过滤包 | 标定文件发布 TF、URDF/关节反馈连接 | 采集时刻变换、几何外包络、Humble 编译和误删 |
| 环境几何 | PCL 距离查询或适用的上游距离场 | 到控制器的观测适配、有效性契约 | 障碍覆盖、未知空间、误差界、延迟、梯度 |
| 跟踪 | 有明确需求再选现成跟踪模块 | 速度估计接口与有效期 | 关联失效不能使几何障碍消失 |
| 控制 | Stanford OSCBF / CBFpy / qpax | 九轴、5D任务、路径推进、必要动态项 | 上游接口差异与旧新控制结果一致性 |
| 运动学/机器人几何 | frax、Bubblify 等候选 | 九轴 URDF、包络校验和关节单位 | 1P8R/FK/Jacobian/碰撞覆盖一致性 |
| 低频规划 | MoveIt2/OMPL/PlanningScene | 既有任务衔接；研究地位见[已确认边界](issues/03-reuse-boundary.md) | 无需自建通用规划场景管理 |
| 执行 | 已有 CAN 依赖与设备协议；评估适用的标准控制接口 | DrEmpower 单位/零位、命令权限与故障锁存 | 真实模式、反馈时效、停止/响应测量 |

这是一组候选职责，不是最终包选型。研究证据在各研究票的 Context pointer 中，具体版本不得以 main/latest 作为可重现安装约束。

## 不应照搬引用讨论的地方

- TF 是运行时变换分发机制，标定文件是带 provenance 的标定权威；二者可以构成单向发布关系。不能以“TF 唯一真源”为由删除既有标定来源决策或制造两个可写真源。
- 有点云最近邻并不意味着遮挡/稀疏/未知空间安全已证明；跟踪速度也不是障碍存在性的前置条件。
- 不无条件删除 vendor 或碰撞验证代码：先核许可证、修改差异、锁定依赖和等价基线。独立验证基线可能有长期价值。
- “ROS2”“上游机器人演示”“kHz”分别不等于 Humble 兼容、九轴即插即用、当前机器达标。

## 原票据如何接续

| 原票据 | 草案处置 |
|---|---|
| [选定目标跟踪语义与验收标准](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/14) | 保留结论，上游兼容性以该语义为基线 |
| [选定规范世界坐标系与标定策略](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/15) | 保留坐标与来源决策；标定工具实现可重新评估复用 |
| [冗余策略对比：9-DOF 对 5D 工具轴任务](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/5) / [选定冗余目标与优先级](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/16) | 复用原证据；在上游控制接入票中引用，不再另做同题研究 |
| [感知时间同步与延迟模型](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/7) | 纳入连续障碍观测及端到端验证门的前置证据，不以固定 1.0s 当已验证阈值 |
| [自碰撞/障碍几何模型核对](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/8) | 保留独立几何验收职责；按已接受决议核验自碰撞 OBB、环境全臂 OBB 与占据体素/FCL，不执行球模型替换 |
| [CBF/QP 可行性实证测量](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/6) / [选定不可行/裕度/降级策略](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/18) | 复用原票，待表示与控制接入清晰后补充旧新对比 |
| [自体过滤 bridge 接线](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/22) | 建议重写为上游自体过滤接入与验收；在选型前不继续扩大本地过滤算法 |
| [tracks 契约升级](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/21) | 建议扩展为即时几何+速度增强契约；只给 tracks 加 header 不解决静态覆盖 |
| [感知健康接线](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/23) | 保留故障要求，依赖新观测契约；覆盖状态不可从 tracks 是否为空推断 |
| [标定工具链](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/25) | “必须 ROS2 移植”改为待核验，先找可用上游/离线输出适配再决定 |
| [标定 SSOT 接线](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/27) / [启动自检程序](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/26) | 保留权威来源、来源校验和启动门；实现接到选定上游链 |
| 测试修复、配置清理、SocketCAN 接线及真机操作票 | 保留普通 implementation backlog；不冒充决策票，不因地图改版自动关闭 |

## 发布接续

恢复 `gh` 认证后先重新读取远端地图、子票、评论与 assignee，合并期间他人决议。将 MAP 正文更新到现有地图；历史细节留在既有票/链接资产。新增票先创建取得真实 issue 数据库 ID，再加原生 sub-issue 和 dependencies。研究结果作为 resolution comment 后关闭研究票，最后更新地图索引。

不得将本地 research 的完成冒充远端 issue 已关闭。不得在未取得用户回答时关闭任何 grilling 票。独有能力边界已取得用户答复并本地记录，本轮其余 grilling 票现也已逐轮取得答复，详见各票最终 resolution；原实施与证据票不自动关闭。
