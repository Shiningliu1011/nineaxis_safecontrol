# Reviewer checks

本文件只定义 reviewer 的检查动作。架构事实、术语和决策以
[CONTEXT](CONTEXT.md)、[相关 ADR](adr/) 和下面的 canonical modules
为准；发现冲突时给出证据和独立修订建议，不在这里另建一份事实源。
按变更范围检查，报告具体不一致及验证缺口；不要求无关改动完成全表。

| 检查动作 | 依据与入口 |
|---|---|
| 核对新术语、坐标变换方向及跨模块边界；若与已接受决策不符，明确指出，不静默改写含义。 | [Domain workflow](agents/domain.md)、[CONTEXT](CONTEXT.md)、[ADR](adr/) |
| 检查实现是否复用 canonical identity/order 和共享变换入口；确认已有契约测试覆盖变更的接缝，避免新增独立常量副本。 | [robot_spec.py](../src/robot_safecontrol_moveit/robot_spec.py)、[oscbf_trajectory.py](../src/robot_safecontrol_moveit/oscbf_trajectory.py)、[轨迹一致性测试](../tests/test_trajectory_transform_unity.py) |
| 审查 ROS 实体创建处的 topic、QoS、remap、callback group 和 ownership；要求解释偏离 canonical transport 的理由，并核对 raw/resolved topic 各自的用途。 | [ros_conventions.py](../src/robot_safecontrol_moveit/ros_conventions.py)、[生产入口契约](../tests/test_oscbf_production_config.py) |
| 追踪配置从文件到 override 再到实际消费者的来源；确认生产、portable/offline、legacy 边界没有因共享 loader 或默认值而丢失。 | [生产 profile](../config/oscbf_controller.yaml)、[配置契约](../tests/test_oscbf_production_config.py)、[运行入口](../src/robot_safecontrol_moveit/oscbf_controller.py) |
| 对控制/硬件变更检查失败路径在何时创建 publisher、timer、backend，及所有可到达的发送路径（包括 polling）；要求证明缺条件时的拒绝行为。 | [hardware_bridge.py](../src/robot_safecontrol_moveit/hardware_bridge.py)、[hardware_contract.py](../src/robot_safecontrol_moveit/hardware_contract.py)、[bridge 测试](../tests/test_hardware_bridge.py) |
| 检查 feedback 的真实来源、时间基准、freshness、watchdog 和人工恢复；拒绝以 command、补零或重打时间戳替代真实反馈的验收证据。 | [硬件合同](../src/robot_safecontrol_moveit/hardware_contract.py)、[硬件运行手册](real_robot_runbook.md) |
| 检查失败修复是否削弱门禁、放宽断言或混淆 sim/shadow/live；已有检查足够时只补缺失的接缝验证，不叠加相同规则。 | [硬件合同测试](../tests/test_hardware_contract.py)、[生产入口契约](../tests/test_oscbf_production_config.py) |
| 核对依赖是否越过 portable 控制内核边界；新增运行依赖必须显式声明，不能以开发机碰巧安装作为依据。 | [ONBOARDING](ONBOARDING.md)、[package.xml](../package.xml)、[portable_oscbf](../portable_oscbf/) |
| 核对“已通过/已完成”的陈述与 commit、dirty diff、命令、环境及退出码是否对应；缺失验证明确标出。历史 issue body 不替代当前结果。 | [Tracker workflow](agents/issue-tracker.md)、[完整测试入口](../run_all_tests.sh) |
| 对规则/文档改动先检查代码、测试、诊断能否解决；常驻 steering 只保留必要引导，非显然 lesson 保留适用条件，避免复制架构事实和高波动状态。 | [历史导航](agents/legacy/CLAUDE.md)、[LESSONS_LEARNED](LESSONS_LEARNED.md)、[ONBOARDING](ONBOARDING.md) |

自动检查通过不表示语义审查完成；reviewer 应引用已有覆盖，而不是要求把
每条检查变成 CLAUDE.md 规则。安全行为改变需独立审查与验收。
