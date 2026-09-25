# [I17] 移除轨迹平移参数并统一关节限幅来源

## Goal

完成 GitHub 实现票 [I17] 的参数清理、来源统一与验收。

## Requirements

- 删除旧轨迹平移参数的声明、配置、读取、传递、形参与测试配置。
- `robot_params.yaml` 和 `nineaxis.yaml` 不再手写关节限幅。关节位置限幅以项目 URDF 生成的常量为准；仿真速度及加速度限幅以 `actuator_modules.yaml` 为准。
- 轨迹加载时核对关节名称及顺序，相关测试检查运行时来源和限幅。
- 保持现有轨迹坐标与时间序列逐值相同，不改变控制命令或硬件安全限制。
- 同步更新相关使用文档。

## Acceptance Criteria

- [x] Git 跟踪文件检索没有旧轨迹平移参数的名称。
- [x] `robot_params.yaml` 与 `nineaxis.yaml` 没有手写的 `joint_limits`；运行时位置限幅仍与 URDF 一致，速度与加速度限幅仍来自项目执行器配置。
- [x] 真实 `ik_input.mat` 的相关轨迹坐标与时间序列在修改前后逐值相同。
- [x] 相关测试及 `bash run_all_tests.sh` 通过；结果和命令记入实现票尾评论。

## Notes

- 来源：GitHub issue 128；[规格] 速度级 OSCBF 架构规格说明书（仅激光雷达感知）（GitHub issue 79）。
- 当前工作区的 `docs/agents/issue-tracker.md` 已有用户改动，保持其内容。
