## Common Tasks

```bash
# 构建（必须用脚本，普通 colcon build 发现不了嵌套 C++ 插件）
bash build_aeb_moveit.sh
source install/setup.bash

# 全自动演示：随机起始位姿 → 无碰撞过渡 → OSCBF 跟踪蝴蝶轨迹（无需键盘）
bash run_demo.sh

# shadow 记录模式：无 CAN I/O，不提供真实硬件反馈
ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
    hardware_mode:=shadow start_oscbf_plant:=false

# 测试：结果以当前 checkout 实际执行为准
bash scripts/agent_check.sh              # 快速启发式检查
bash run_all_tests.sh                    # 完整主包 + 内核回归

# 独立脚本（无 ROS）
python3 src/aeb_rrtstar/single_run.py    # 查看 aeb_rrtstar 用法
```

注意：修改 AEB C++ 代码后必须重新 `bash build_aeb_moveit.sh` 并重启整个
launch——运行中的 move_group 不会自动加载新编译的 `.so` 插件。
