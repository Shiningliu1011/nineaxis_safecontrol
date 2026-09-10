# 配置一致性清理（alpha 增益漂移 / 遗留参数 / 死 launch 参数）

- Tracker：https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/12
- 日期：2026-09-11；已认领 Shiningliu1011。
- 本窗口：静态事实核验和实施交接完成；实现未执行，issue 保持 OPEN。
- 用户指令：“wayfinder 继续下一张ticket”。依正式 tracker 查询，本票是唯一无阻塞、未认领的开放子票；其他已认领票未接管。
- 起点 HEAD：`3ec19a41aebfa1567d55caa851feed3bf4d9cbee`；保留原有全部未提交工作。遵循 DECISION-WINDOW-ORDER.md 的规划窗口边界。

## 核验结论

票面不能继续按“把 8 改为 5”实施。现有主控制链的参数路径如下（路径均相对仓库根）：

| 参数/配置 | 实际路径与结论 |
|---|---|
| `nineaxis.yaml: control` 下跟踪增益 | `oscbf_trajectory.load_repository_trajectory` 和 `work/ik_data_loader.py` 仅读取 `kinematics`；当前 ROS 控制链不从该文件读取跟踪增益。固定/非固定 kp 字段在跟踪的 Python/shell/YAML 中只有定义。 |
| `alpha_joint_limit=5`、对象属性 `8` | YAML 有 5；`oscbf_velocity_config.py` 有属性赋值 8；仓库静态搜索无该属性读取。`alpha_collision`、`alpha_singularity` 同样只有赋值。不能从字段名称推断实际逐类增益。 |
| 基准 CBF alpha | `NineaxisOSCBFVelocityConfig.alpha(h)` 返回 `obstacle_h_baseline_alpha * h`，当前为 10。factory 从 config 获取该值，构造约束后还用它还原 h；修改为逐行增益必须一起审计此处。factory 没有票面所说的关节 alpha=8 默认值。 |
| 障碍 alpha | `perception_bridge.py` 第十槽硬编码 1.5 → `obstacle_extractor` 解码 → facade `obs_alpha` → `jax_barrier_terms` 修正/聚合障碍行。缺省输入采用 config 基准 10；`nineaxis.yaml` 的 `cbf_alpha: 1.5` 无按名称消费证据，数值相等不能证明已接线。 |
| 跟踪 kp | `config/oscbf_controller.yaml` → launch → ROS declare/get → `_tracking_step` → facade → factory；生产 kp_pos=160、kp_orient=10、kp_joint=0.45。测试/离线调用显式给出的 50/60 等不应统一覆盖。 |
| 权重与近端项 | ROS w_pos=40、w_orient=10、w_joint=0.1、temporal_lambda=0.2 传入 facade；库默认值可不同，属于不同入口，不能仅因不同判错。factory 还含内部 task damping 与反馈限幅常量，不在本次机械清理中调参。 |
| dt | ROS 节点字面量默认 0.002，生产 YAML 0.01；AST 比较共有的可静态解析默认值，唯一差异为 dt。YAML 顶部“所有值与节点默认一致”的说明不实。dt_path=0.01，发布频率=100；积分步、路径推进步、发布周期不能混为同一个字段。 |
| `controller_params.yaml` | 旧 torque/kp/kd 配置。跟踪的 Python/shell/YAML 未找到文件名引用；但 setup.py 通配符会安装全部 portable config YAML，不能声称没有打包或不存在仓库外消费者。 |
| hardware 配置 | launch 已传 drempower.yaml 和 hardware_mode；HardwareBridge 无 declare/get，实际用 Python 构造默认 shadow。不是 launch 缺参数，而是消费端断线。参见[真机执行链路交接](13-socketcan.md)，避免重复修补。 |

`nineaxis.yaml` 的 EMA `alpha=0.8` 是平滑系数，与 CBF alpha 不是同一个概念。不存在“把所有 alpha 统一成同一个值”的清理规则。

## 证据边界

静态结果：`output/audit-12/static-audit.json`（gitignored，本机留存），由 Python AST 提取节点 defaults，与 PyYAML 读取的生产配置比较，并扫描 `git ls-files` 中 Python/shell/YAML 引用。上述表格同时经过主链源码阅读核对。未排除外部脚本、运行时反射或外部依赖消费；未执行数值仿真、ROS launch、CAN、全量测试。静态核验不等于控制行为验证。

## 可以编码的范围与验收

本票无需新算法或外部框架选型；复用现有 ROS 参数机制、PyYAML 和 facade，不新增配置框架。

可以开始行为不变的清理：为旧 torque 文件和未接入 YAML 字段标明实际状态，修正不存在的生产路径（实际为根目录 config/oscbf_controller.yaml）、过时默认一致声明及 launch 注释。优先标记遗留而非删除；尚未证明外部调用无影响。

后续配置接线应先形成有效配置快照，保留部署与离线 profile 的区别；验证 YAML 覆盖能到达消费方、缺省路径可解释、非法值拒绝、安装路径可加载。测试应观测参数传播或最终约束，不能只断言两个硬编码数字相等。硬件参数接线沿用执行链路原票范围，必须验证 sim/shadow/live 行为；仅让参数开始生效会暴露真实后端及模式缺口，不能作为机械清理顺带启用。

尚不能借本票选定 5/8/10 的新 CBF 数值、把统一 alpha 改成逐类 alpha，或统一积分周期。这些改变控制行为，需对应可行性、控制率/延迟预算和裕度策略票提供证据；保持当前值不是其正确性已获验收。数值变化须比较约束残差、QP 健康、路径进给与任务误差，并复验受影响时延。

## 状态与后续

实施清单尚未完成，不发布 resolution、不关闭、不向地图 Decisions so far 写入已解决条目；事实核验作为本票进展。下一步是在明确实施指令后执行上述行为不变范围；数值与硬件接线仍按原票验收，不创建同义 backlog。本窗口不自动启动其他票。

回退本窗口只需移除此新增交接及本机 audit-12 证据；未修改产品代码、配置、领域词汇或原有未提交文件，未提交或部署。
