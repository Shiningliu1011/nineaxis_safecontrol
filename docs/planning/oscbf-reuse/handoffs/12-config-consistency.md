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


## 2026-09-11 续研：配置来源与验收契约

用户本轮原始指令：“开始研究ticket#12”。本轮起点 HEAD 为 `84797408ac0cbe344bcb9f06fc70da5ae29dd417`。GitHub 原票仍为 OPEN、`wayfinder:task`、implementation-backlog；已继续认领 Shiningliu1011，原生 blocked_by 查询为空。地图规划已收束，本票实施未完成，因此本轮记录研究进展，不作 resolution，不关闭或重复创建票。

保留起点已有的感知时间模型、自体过滤交接修改及 perception-age-budget.md；本轮仅追加本交接和配置研究资产。详见[配置一致性研究](../research/config-consistency.md)，其中保存逐项源码引用与验收建议。

### 本轮事实增量

重新以 Python AST + PyYAML 比较节点默认值和部署 YAML：24 个共同且可静态解析的字段中，只有 `dt` 不同，节点 0.002、部署 0.01。另有 6 个 YAML 字段的默认值涉及路径、常量或函数调用，未纳入字面量对比；节点另声明 4 个 YAML 未列字段。不能把这项核验表述为“全部配置只有一处差异”。完整本机快照为 `output/audit-12/research-refresh-20260911.json`（gitignored，含 HEAD、节点源码 SHA256、扫描方法与匹配位置）。

重新扫描受版本控制的 Python/YAML/shell：`alpha_joint_limit` 只有 YAML 定义和对象赋值；`alpha_collision`、`alpha_singularity` 只有对象赋值；`cbf_alpha` 只有 YAML 定义；旧 torque 配置文件名没有字面量引用。此结果与上一轮一致，仍不排除外部消费者和动态读取。

源码再次确认 launch 已传硬件参数，节点却只读 Python 构造参数；现有硬件测试直接注入构造参数，不能证明 launch/YAML 参数生效。控制器 smoke fixture 也没有加载生产 YAML，不能拿它作为部署配置传播证据。

### 推荐实施顺序与验收

1. **行为不变的清理可开始编码**：标注未接入字段和旧 torque 文件的遗留状态，纠正生产 YAML 路径、默认一致声明及过时注释。保留解析后的所有数值及现有安装资产；用 YAML 解析前后对照和安装清单核对验证这一步，不以字符串相等测试替代行为验证。
2. **配置传播与快照**：沿用 rclpy 参数机制、PyYAML 和现有 facade。分别记录部署 profile、无 YAML 启动及离线库入口的最终有效值与来源，保留三者区别。覆盖实际生产 YAML 加载、单项显式覆盖到消费方、空路径回退、已安装 share 路径、非法/非有限值拒绝。测试使用非默认哨兵值并观测 facade 构造参数或跟踪调用参数；对 CBF 则检查最终约束残差/行系数，不只读取一个同名属性。
3. **硬件接线**：沿用[真机执行链路交接](13-socketcan.md)的 ROS 参数、模式与失败契约；本票跟踪其验收证据，不单独补一层重复 launch 注入。sim 不占执行链、shadow 不发送运动命令、live 缺后端/反馈/标定拒绝，均需隔离测试。原票“注入或移除二选一”的文字不能掩盖消费端和反馈健康缺口。
4. **数值变更仍不能由清理票定案**：CBF 5/8/10、逐类 alpha、积分步长及发布周期的统一均改变行为，需原有可行性、控制预算与裕度策略证据。当前值只作为回归基线。最终验收比较约束残差、QP 健康、任务误差、路径进给和受影响时延。

“YAML 唯一来源”建议落实为每个部署参数有明确权威来源、覆盖规则和最终有效值；不把独立离线入口默认值或不同物理意义的参数强行统一。该建议不是用户已接受的新架构决议。

无需新增配置框架、算法、第三方依赖或 fork；本票的开源替换比较不适用，现有机制已可承担参数传递，缺口在消费端与验证。下一步为按上述范围实施；本轮研究请求不包含产品实施。未运行 ROS launch、数值仿真或硬件；静态核验不作为运行验收。撤销本轮只需移除本追加章节及新研究资产，保留已有交接和其他窗口改动。

本轮研究进展已[同步至正式票据评论](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/12#issuecomment-5627989490)。本地研究文档未提交或推送。


本轮逐行核验还修正前文三处路径表述：nineaxis 顶层键应为 `controller`；ROS 跟踪调用为 `step_once → path_tracking_step`；生产 ROS 障碍回调在控制器内自行解码第十槽，未调用 `obstacle_extractor` 的解码函数（仅导入槽位常量）。详见研究文档，后续传播测试须命中真实入口。另确认：只覆写 portable root 时空 config 路径仍回退 share 默认；dt/权重在 loop 构造时读取、频率在 timer 创建时读取，kp 每步读取，不能承诺所有 ROS 参数都能动态生效。动态障碍契约测试当前整模块 skip，不能作为已执行验收。

## 2026-09-11 人审对齐：八项配置契约

用户要求调用 wayfinder 与 grill-me，先对齐想法，每次一个问题并给推荐选项。以下八问用户均明确回答“A”；这是逐项已接受的选择，整体范围总结尚待最后核对，尚未授权产品实施。

1. 按运行入口明确权威来源：生产 YAML 管理部署参数，明确覆盖规则与有效值来源；离线入口保留独立默认值。验收观测真实消费方的参数传播。
2. 生产部署找不到指定 YAML 时拒绝启动并报告路径；离线入口可显式使用默认值。
3. 启动显式覆盖优先于生产 YAML；覆盖值通过校验，记录覆盖前后值与来源。
4. 本票管理的控制参数在启动时形成快照，运行中修改明确拒绝，调整后重启；实现须消除当前不同读取时机造成的部分生效。
5. 影响控制行为的参数（增益、权重、周期等）必填，缺失拒绝启动；可选项允许有记录的默认值。具体字段清单由实际消费路径核验形成，不将所有模块字段混作一个配置范围。
6. 未接入的旧控制字段标记遗留，保留旧文件供追溯；若作为当前生产入口的控制配置使用，明确报错。检查限定当前入口范围，保留其他模块合法字段。
7. 硬件模式、后端与反馈接线继续由《真机执行链路：SocketCAN 后端与参数注入》负责；本票定义配置规则并核对交接和验收证据，不重复实施。
8. 每次启动保存最终有效配置快照，包含最终值、来源、显式覆盖记录及配置版本标识；日志打印摘要和快照路径。保存失败拒绝启动。

实施边界建议：沿用 rclpy、PyYAML 与现有 facade，不新增配置框架；当前实际生效数值作为回归基线，CBF 新数值、逐类增益和周期调整仍依赖对应原票。配置缺失/非法、写入失败与运行中修改的拒绝行为属于本轮有意改变的启动及管理契约，不应宣称整个实现“行为不变”。

后续实施应先形成逐入口参数清单（消费方、必填/可选、合法域、来源），验证生产 YAML 和非默认覆盖到达真实消费方，缺项/遗留字段/非法值拒绝，安装路径加载，运行中修改拒绝，快照内容与最终值一致及写入失败门禁。尚未执行这些验证，尚未关闭原票或标记地图完成。仅追加本交接，保留已有未提交工作。

## 对齐完成与实施交接

用户对第九问明确选择“A｜确认，整理实施交接”：接受以上八项为本票实施契约，本轮结束对齐，不开始产品实施。本节优先于前文研究阶段的建议：生产入口不再允许无 YAML 静默回退，运行中控制参数修改不再保留旧的部分生效行为。起点 HEAD 仍为 `84797408ac0cbe344bcb9f06fc70da5ae29dd417`，原有未提交研究及其他票交接保留。

### 参数清单与入口责任

以下依据当前 `config/oscbf_controller.yaml`、`src/robot_safecontrol_moveit/oscbf_controller.py` 的声明/校验及研究中的实际调用链形成；必填分类是已接受规则的实施展开，不是新的数值决议。

| 分组 | 字段 | 实施要求与消费方 |
| --- | --- | --- |
| 周期 | dt、dt_path、publish_frequency_hz | 生产必填；分别进入积分、路径推进和 timer，不要求数值相等。保留生产 0.01/0.01/100 基线。 |
| 跟踪与求解 | kp_pos、kp_orient、kp_joint、damping、w_pos、w_orient、w_joint、temporal_lambda、enable_x64、solver_tol | 生产必填；沿 loop 构造及 path_tracking_step 传到 facade/kernel；kp 改为读取已固定的有效配置。 |
| 任务与进给 | task_mode、use_nullspace_policy、reference_feedrate_scale、nullspace_speed_limit、max_tool_axis_speed_rad_s、reference_lead_m | 生产必填；覆盖任务构造、参考进给和零空间策略，保留当前值与任务语义。 |
| 参考几何 | orientation_mode、cylinder_axis_direction | 生产必填；进入轨迹构造；cylinder_center 可选，省略表示沿用从轨迹拟合，快照记录该来源及解析结果可得部分。 |
| 启动与感知开关 | wait_for_start、enable_perception_obstacles | 生产必填，YAML 明示基值后允许 launch 显式覆盖；当前 YAML 缺少后者，实施时补当前节点基值 false，不改变 launch 覆盖含义。 |
| 关节及通信 | joint_names、joint_state_topic、publish_joint_state_topic、perception_tracks_topic | 生产明确列出；当前感知话题缺项补既有默认，不重命名话题。校验关节身份/数量并记录最终连接。 |
| 资源路径 | trajectory_mat、portable_oscbf_root、portable_config_yaml | 允许现有安装 share 默认/空值解析；记录原始值、解析路径及来源，资源缺失或不可读拒绝启动。保留“仅覆盖 root 不自动派生 config”的现状并修正注释。生产控制 YAML 本身必须存在，不能与这些可选资源路径回退混淆。 |
| 诊断 | telemetry_period_s、perf_report_path、latency_budget_ms | 可选并记录默认；诊断预算不作为本票新选定的运行周期或安全门槛。 |
| 快照元数据 | 运行标识、最终值、来源、覆盖链、配置内容标识、软件版本标识、输出路径 | 实施新增诊断产物，不作为控制增益；避免运行间覆盖，记录实际配置内容摘要；未提交工作不能只用 HEAD 冒充完整版本，可附 dirty 状态及相关源码内容摘要。 |
| 内核/障碍来源 | obstacle_h_baseline_alpha、障碍输入第十槽 alpha | 保留现有来源和数值；启动快照说明基准来源及障碍 alpha 来自运行时观测，不把未来消息伪装为启动固定参数。逐类 alpha 接线不在本次实施范围。 |
| 遗留项 | nineaxis 中未消费的 controller 字段、alpha_joint_limit/alpha_collision/alpha_singularity、旧 controller_params.yaml | 保留资产与遗留说明；当前入口的生产控制配置若提交未接入字段则报错。nineaxis 的 kinematics 仍是合法资源，不能因为该资源还含遗留 controller 区域就拒绝整个资源文件。 |

离线库入口维持现有默认及显式传参方式；生产 ROS 入口即使绕过 launch 直接运行，也必须满足生产配置加载契约，不能以“节点默认值齐全”绕过。无需将离线测试全部迁移为生产 profile。

### 实施顺序

1. 沿用现有 rclpy、PyYAML、facade，建立轻量参数清单与来源记录；在合并覆盖前验证生产 YAML 必填字段，不能用代码默认或启动覆盖掩盖 YAML 缺项。检查限定控制器参数空间；拼写错误/未知字段明确拒绝。
2. 解析资源路径并完成类型、有限性及既有合法域校验，再应用并校验显式覆盖，形成唯一有效配置。保留已有允许域，不在本票引入未经证据支持的安全阈值。未覆盖字段的数学合法性沿真实消费方规则补齐，不能用浮点隐式转换接受错误类型。
3. 所有控制消费者使用同一份启动配置。路径解析等内部启动赋值结束后锁定管理范围；运行中 ROS 参数修改返回明确失败，不出现参数服务器显示新值而控制器仍用旧值的情况。
4. 在启动命令发布/timer 前完成快照持久化；采用避免半写文件的写入方式，失败时中止启动并报告原因。日志给出摘要和绝对路径。独立离线入口不强加生产快照门禁。
5. 更新生产 YAML、注释、遗留状态说明及针对真实入口的验证；硬件接线只引用原票证据。新旧实现使用相同有效参数比较，不能把允许的启动管理行为变化写成完全行为不变。

### 必须通过的验收

- 真实生产 YAML 从源码及安装 share 路径均能加载；文件缺失/不可读、必填项缺失、非法类型/非有限值/越界值、当前控制范围中的遗留或未知字段均拒绝启动，且不发布控制命令。
- 使用合法非默认哨兵覆盖，检查 facade 构造、跟踪调用及 timer 实参；快照中的最终值和来源与消费者一致，覆盖前值可追溯。仅比较 YAML 与代码数字相等不构成验收。
- 独立改变 dt、dt_path、频率，确认三个不同消费路径；保留现有生产与离线差异。验证三资源路径的默认、单独覆盖及组合覆盖，并验证安装后资源真实可读。
- 运行中修改受管参数（单项及批量）明确失败，有效配置及实际消费者不变；启动覆盖仍正常生效。
- 快照正常生成、内容完整、运行间不覆盖；模拟目录不可写或持久化失败，确认启动拒绝且无命令发布。涉及失败门禁的验证应观察发布行为，不能仅断言异常文本。
- 相同有效配置下比较必要的控制回归与关键约束数据，保留基准 alpha 还原 h 的既有保护。若实现实际触及障碍 alpha 传播，应追加研究文档中的真实 ROS 回调到约束行测试；未触及则不顺带重构该链。
- 测试报告区分执行通过、跳过和缺失依赖；静态清单不能替代 ROS 配置传播验证。硬件验收仍由原票提供，不因此声明真机准入通过。

### 完成条件、限制与回退

本轮规划已经完成；可以按本契约开始配置管理实施，但须用户另发实施指令。实现及上述必要验收完成后才可解决原票；本轮不关闭、不写入地图已完成索引。CBF 数值、控制周期调优与硬件执行接线仍按对应原票推进。

实施时单独形成可审阅改动，回退恢复代码与配套生产 YAML，保留已产生的运行快照作为证据；不得借回退覆盖其他票未提交改动。本轮仅追加交接，没有运行产品测试、仿真或硬件，也未提交/推送代码。
