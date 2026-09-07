# OSCBF 控制与机器人模型上游核验

核验日期：2026-09-07。对应决策票据：[核验 OSCBF 上游能否承接九轴控制与碰撞模型](../issues/01-control-upstream.md)。结论是研究事实与候选边界，尚未替用户决定迁移路线。本次只读源码、官方文档与 GitHub API；未安装依赖、运行九轴对照或实机测试。

## 固定审计快照

下列版本号来自对应 commit 的 pyproject.toml（不是声称已有同名 release）；日期为默认分支最新 commit 日期，不能把它等同于持续维护承诺。完整快照由 shallow clone 与 GitHub commits API 交叉获得。

| 项目 | 固定 commit / 最新提交日期 | 包版本与许可证据 | 维护/定位 |
|---|---|---|---|
| [StanfordASL/oscbf](https://github.com/StanfordASL/oscbf) | `6082329f8e1d61ee8e875a63a5cd80a14d513e12` / 2026-02-17 | 0.0.1；[MIT LICENSE](https://github.com/StanfordASL/oscbf/blob/6082329f8e1d61ee8e875a63a5cd80a14d513e12/LICENSE) | IROS 2025 配套控制器与示例，不能仅凭论文认定工业成熟度 |
| [StanfordASL/oscbf_hardware_ws](https://github.com/StanfordASL/oscbf_hardware_ws) | `26f251f6956450475031ab906ebb08f3d675e73a` / 2026-02-17 | 两包 0.0.1；[package.xml](https://github.com/StanfordASL/oscbf_hardware_ws/blob/26f251f6956450475031ab906ebb08f3d675e73a/src/oscbf_control/package.xml) 与 [消息包](https://github.com/StanfordASL/oscbf_hardware_ws/blob/26f251f6956450475031ab906ebb08f3d675e73a/src/oscbf_control_msgs/package.xml) 均为 TODO License declaration；根目录无 LICENSE，API license=null | Franka Panda 实验工作区；不能自动继承隔壁 oscbf 的 MIT |
| [StanfordASL/cbfpy](https://github.com/StanfordASL/cbfpy) | `f5bff8b93609d6a164fd8beeef962c92e4476dcb` / 2026-07-22 | 0.0.4；[MIT LICENSE](https://github.com/StanfordASL/cbfpy/blob/f5bff8b93609d6a164fd8beeef962c92e4476dcb/LICENSE) | 可直接依赖的通用 CBF 框架；需固定 API 版本 |
| [StanfordASL/frax](https://github.com/StanfordASL/frax) | `0e1e4532a78f48502cb478bb4e6f7414fc817d99` / 2026-07-05 | 0.0.5；[MIT LICENSE](https://github.com/StanfordASL/frax/blob/0e1e4532a78f48502cb478bb4e6f7414fc817d99/LICENSE) | [README](https://github.com/StanfordASL/frax/blob/0e1e4532a78f48502cb478bb4e6f7414fc817d99/README.md) 明确 beta 内部操作可能改变 |
| [bheijden/bubblify](https://github.com/bheijden/bubblify) | `35b44a1690c6933a485510cece6df6a6001f0b8b` / 2025-09-09 | 0.1.0；[pyproject](https://github.com/bheijden/bubblify/blob/35b44a1690c6933a485510cece6df6a6001f0b8b/pyproject.toml) 声明 Apache-2.0、Alpha；根目录未见独立 LICENSE，GitHub 未识别 | 离线交互球化工具；真实 owner 为 bheijden，StanfordASL/bubblify 返回 404 |

## 复用矩阵

| 能力 / 本地落点 | 已有上游能力 | 候选复用方式 | 必须保留或验证的缺口 |
|---|---|---|---|
| CBF 构造与 QP / `portable_oscbf/work/oscbf_velocity_config.py`、`jax_control_facade.py` | CBFpy `CBFConfig`、`CBF.from_config`、`safety_filter`、`qp_data`，JAX 自动微分 | **直接依赖 CBFpy**，本项目只实现任务与环境 barrier/目标配置 | 本地已有 cbfpy 0.0.1 monkey patch；升级 0.0.4 必须对齐返回值、松弛、QP 数据接口，不能把替换包名视作完成 |
| 1P8R FK/Jacobian / `nineaxis_manipulator_jax.py`、`nineaxis_kinematics.py` | frax 从 URDF 建 Robot/Manipulator，支持 P/R 与 joint_ordering、ee_offset | **frax + 薄机器人适配**是消除自写运动学的主要候选 | 九轴活动顺序、固定关节融合、末端/TCP、单位与 Jacobian 排列须对照；Manipulator 断言串联链；不能证明此九轴已受上游测试 |
| 速度级 OSCBF / 本地 `NineaxisOSCBFVelocityConfig` | 上游 `OSCBFVelocityConfig` 有 z=q、u=qdot、f=0、g=I | 优先派生/组合；只有接口无法覆盖时才最小 fork | 原实现 assert 原 `oscbf` Manipulator 类型，固定 task_dim=6，调用 `_ee_jacobian`/`_mass_matrix` 等私有接口；frax 类并非直接替身 |
| 5D 工具轴 / `tool_axis_task.py`、预计算 task_p | 原 OSCBF 给完整姿态 6D 权重与动力学一致广义逆 | **保留薄 5D 任务适配**，复用上游 FK 与 CBF | 5D 留自由工具滚转，不能用 rot_obj_weight=0 替代（这会去掉全部姿态任务）；需验证任务 Jacobian、零空间与奇异性定义。上游质量矩阵度量和本地阻尼几何伪逆也不等价 |
| 自/环境碰撞 / `dpax_collision.py`、`obb_collision_model.py` | frax 球碰撞模型、self_collision_distances、世界球位置；bubblify 输出球化 URDF/YAML | 可复用 **模型编辑工具 + frax 球 FK** | frax 不提供本地 DCOL OBB 距离或 ESDF 采样；球模型不能无验证替换 14 对 OBB/10 link 动障约束。九轴覆盖率、保守性、球数对 QP 行数影响需验收 |
| 动态障碍 / 本地固定 8 槽、速度/半径导数、ESDF 参数 | OSCBF 有 dynamic_obstacle demo；CBFpy 支持 *args | **复用通用 CBF 接口**，保留经验证的障碍时变适配 | 上游 demo 只取末端一个球、机器人速度设零并 TODO 相对速度；不覆盖全臂多障碍、传感器时延与失效状态 |
| 路径与进给 / `jax_path_following.py` | 上游可接 nominal input 与 EE 参考轨迹 | 保留路径层，以 qdot_nominal/task metric/环境输入连接安全滤波器 | 未发现与现有弧长投影、进给限幅、工具轴限速、端点制动等价的完整替代；不要重写它来模仿正弦 demo |
| ROS / `src/robot_safecontrol_moveit/oscbf_controller.py` | hardware_ws Python/C++ 节点、状态/力矩/EE 参考链路 | 仅作为架构参考；许可澄清后再考虑代码复用 | 上游是 Panda 力矩链路，本地是九轴速度/仿真接口；需确认执行器契约，不能原样接入 |

证据：[OSCBFVelocityConfig 固定源码](https://github.com/StanfordASL/oscbf/blob/6082329f8e1d61ee8e875a63a5cd80a14d513e12/oscbf/core/oscbf_configs.py)、[frax Robot](https://github.com/StanfordASL/frax/blob/0e1e4532a78f48502cb478bb4e6f7414fc817d99/frax/core/robot.py)、[frax Manipulator](https://github.com/StanfordASL/frax/blob/0e1e4532a78f48502cb478bb4e6f7414fc817d99/frax/core/manipulator.py)、[CBFpy CBF](https://github.com/StanfordASL/cbfpy/blob/f5bff8b93609d6a164fd8beeef962c92e4476dcb/cbfpy/cbfs/cbf.py)、[动态障碍 demo](https://github.com/StanfordASL/oscbf/blob/6082329f8e1d61ee8e875a63a5cd80a14d513e12/oscbf/examples/dynamic_obstacle.py)、[碰撞建模文档](https://github.com/StanfordASL/frax/blob/0e1e4532a78f48502cb478bb4e6f7414fc817d99/docs/modeling_collision.md)。矩阵中“候选”是结合这些源码与本地接口的研究推断。

## 环境与数学兼容边界

- 原 [OSCBF pyproject](https://github.com/StanfordASL/oscbf/blob/6082329f8e1d61ee8e875a63a5cd80a14d513e12/pyproject.toml) 精确依赖 `jax==0.4.30`、`jaxlib==0.4.30`、`cbfpy==0.0.1`；最新 [CBFpy](https://github.com/StanfordASL/cbfpy/blob/f5bff8b93609d6a164fd8beeef962c92e4476dcb/pyproject.toml) 和 [frax](https://github.com/StanfordASL/frax/blob/0e1e4532a78f48502cb478bb4e6f7414fc817d99/pyproject.toml) 只声明未固定 JAX，推荐 CPU 0.4.30。因此原 oscbf 与 cbfpy 0.0.4 不能不改依赖就声称处于同一受支持环境。应分别评估“固定原 OSCBF 栈”与“frax + CBFpy 薄配置”，先以隔离环境导入/JIT/数值对照决定。
- [hardware README](https://github.com/StanfordASL/oscbf_hardware_ws/blob/26f251f6956450475031ab906ebb08f3d675e73a/README.md) 要求 Python 与 ROS 版本一致（Humble 3.10、Jazzy 3.12），只列 Panda；旧硬件说明依赖修改版 libfranka 0.8.0。它不是通用九轴 ros2_control 驱动。
- CBFpy 源码 `h_and_Lfh`/`Lgh` 的 JVP 只对 z 求导。将障碍位置速度作为额外参数并不自动产生障碍时变项。完整动态约束应证明增广状态或显式时间导数修正的等价性；这属于适配验证，不需要另写 QP 求解器。
- 本地速度配置仍以 `h_2` 命名/挂接几何约束，f=0，而上游速度示例使用 `h_1`。不能仅依据“相对度 2”注释判断正确与否；必须展开最终 QP 比对增益、符号、动态项。现有 P/q 可接预计算 task_p，故不能只看备用 `_P` 就断言运行时固定 6D。
- frax 作者的 CPU JAX 版本建议与 OSCBF 千赫兹论文结果只能成为实验假设。本次没有九轴控制周期、最坏延迟、误差或安全统计，不能填作本机指标。

## 供后续决策使用的验收问题

1. 是否选 frax + CBFpy 为基础依赖，还是原 OSCBF 固定栈？通过 1P8R 同一 URDF 的 FK/全链 Jacobian/TCP/关节排序与现有参考对照后选择。
2. 5D 目标与时变 barrier 的差异能否全部以配置/组合实现？只有明确无法覆盖的接口才列为最小 fork，单列补丁、上游 issue 与回归用例。
3. 球模型能否以更少维护替代部分碰撞实现，同时保留几何保守性？bubblify 仅工具候选；不得因“球更快”默认删去 OBB/ESDF。
4. 核心依赖锁定后，是否可删除旧 monkey patch 与重复运动学？以结果等价/性能预算为删除门槛，路径跟随和执行器/感知契约继续属于机器人特有层。
5. hardware_ws 代码许可需由维护者澄清；bubblify 已有 Apache-2.0 元数据但打包时应补齐许可文本核验。这些是候选自身的具体材料缺口，并非请求用户此刻批准部署。
