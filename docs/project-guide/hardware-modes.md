## 硬件模式（当前 containment）

当前处于 fail-closed containment：

- `sim`：hardware bridge 不创建真实硬件控制 I/O。
- `shadow`：仅记录命令请求与安全拒绝，不收发 CAN、不发布真实硬件状态。
- `live`：final launch 在启动任何 Node/TimerAction 前拒绝 `hardware_mode=live`；`hardware_bridge` 自身也保留独立拒绝，用于保护直接启动节点的入口。

`hardware_mode` 仅接受 `sim` / `shadow` / `live`；非法值与 `live` 都会在 final launch 启动节点前失败，其中 `live` 是 containment 禁用，`sim`/`shadow` 为允许模式。

真实 backend、配置、标定、真实 feedback freshness/watchdog 和 real-hardware acceptance 完成前，live 仍不可用。反馈不可用时不会报告 healthy，人工确认不能解除这一条件。禁止发送不等于物理制动，物理急停仍是独立链路。以下 live 命令仅用于说明拒绝行为；后续准入见[真机运行手册](../real_robot_runbook.md)。

```bash
# shadow 模式：记录请求及拒绝结果；真实反馈 unavailable；不收发 CAN
ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
    hardware_mode:=shadow start_oscbf_plant:=false

# live 参数形式：final launch 在启动任何节点前报错退出
ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
    hardware_mode:=live start_oscbf_plant:=false

# 即使启用感知，final launch 仍在启动任何节点前拒绝 live
ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
    hardware_mode:=live start_oscbf_plant:=false start_perception:=true
```

标定与实机操作属于后续独立准入流程，参见[真机运行手册](../real_robot_runbook.md)，不是当前 containment 的运行步骤。

话题约定：OSCBF 控制器订阅植物状态 `/mujoco_joint_states`，把安全命令发布到
`/oscbf_command`；`oscbf_plant` 节点把命令经 jerk 限幅积分后作为植物状态发回
`/mujoco_joint_states`。查看器、过渡服务器、控制器共用同一套校准轨迹变换
（`oscbf_trajectory.py`），保证 tool0 与显示的蝴蝶曲线重合。

每次修改 AEB 的 C++ 代码后，都要重新执行 `bash build_aeb_moveit.sh`，再重启
整个 launch（直接运行 `bash run_demo.sh` 即可）。运行中的 `move_group` 不会自动
加载新编译的 `.so` 插件。
