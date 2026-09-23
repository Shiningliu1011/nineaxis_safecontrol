# robot_safecontrol

ROS 2 Humble / MoveIt2 / MuJoCo 的九轴仿真闭环，控制内核位于 `portable_oscbf/work`。

## 构建与测试入口

```bash
bash build_aeb_moveit.sh   # 嵌套 C++ 插件需由此脚本构建
source install/setup.bash
bash scripts/agent_check.sh  # 快速启发式检查，不替代完整测试
bash run_all_tests.sh        # 主包 + portable OSCBF 完整测试
bash run_demo.sh             # 仿真演示；会清理旧 demo 进程
```

修改 AEB C++ 后需重新构建并重启 launch，运行中的 move_group 不会热加载 `.so`。
环境依赖见 [ONBOARDING](../../ONBOARDING.md) 与 [控制内核说明](../../modules/portable_oscbf.md)。
当前测试真值由上述命令产生；历史测试数量和 issue body 不代表当前 HEAD。
当前 tracker 见 [GitHub 地图](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/1)。

## 当前硬件安全边界

- sim：hardware bridge 不创建真实控制 I/O。
- shadow：仅记录命令请求与安全拒绝，不收发 CAN；无真实反馈时不发布硬件状态或报告健康。
- live：final launch 与 direct hardware bridge 均 fail closed。
- command、补零、重打时间戳及人工 acknowledgement 均不能替代真实反馈。
- containment、测试通过不等于真机准入或物理停车能力；硬件后续工作见 [#13](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13)。

## Canonical sources 与导航

- 架构、入口与当前状态流 ownership：[ONBOARDING](../../ONBOARDING.md)。
- 领域词汇与坐标语义：[CONTEXT](../../CONTEXT.md)、[ADR](../../adr/)；ADR 目标与实现进度分开核对。
- ROS topic/QoS：[ros_conventions.py](../../../src/robot_safecontrol_moveit/ros_conventions.py)。
- 关节身份/顺序：[robot_spec.py](../../../src/robot_safecontrol_moveit/robot_spec.py)。
- 共享轨迹变换：[oscbf_trajectory.py](../../../src/robot_safecontrol_moveit/oscbf_trajectory.py)。
- 生产 OSCBF 配置：[oscbf_controller.yaml](../../../config/oscbf_controller.yaml)，入口说明见 [项目运行说明](../../PROJECT_GUIDE.md)。
- 参数/接口历史教训：[LESSONS_LEARNED](../../LESSONS_LEARNED.md)；复用时核对适用环境。
- Reviewer 检查：[CODING_STANDARDS](../../CODING_STANDARDS.md)。
- Tracker 操作：[issue-tracker](../issue-tracker.md)；领域文档流程：[domain](../domain.md)。
- 真机目标与验收步骤：[runbook](../../real_robot_runbook.md)；当前能力以 containment 实现和 #13 最新进展为准。
