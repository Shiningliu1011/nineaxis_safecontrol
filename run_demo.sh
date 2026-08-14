#!/bin/bash
# ===================================================================
# 一键启动：任意起始位姿 → 自动过渡 → OSCBF 跟踪蝴蝶轨迹
#
# 用法:
#   bash run_demo.sh
#
# 该脚本是 mujoco_transition_final.launch.py 的薄包装，
# 只负责清理旧进程并启动完整系统。
# ===================================================================
set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$PROJECT_DIR"

# --- 清理旧进程 ---
echo "清理旧进程..."
pkill -INT -f transition_planning_server 2>/dev/null || true
pkill -INT -f mujoco_joint_state_viewer 2>/dev/null || true
pkill -INT -f move_group 2>/dev/null || true
pkill -INT -f robot_state_publisher 2>/dev/null || true
pkill -INT -f mujoco_viewer 2>/dev/null || true
pkill -INT -f oscbf_controller 2>/dev/null || true
pkill -INT -f oscbf_plant 2>/dev/null || true
sleep 2
echo "  清理完成"

# --- 加载环境 ---
echo ""
echo "=== 加载 ROS2 环境 ==="
source /opt/ros/humble/setup.bash
source install/setup.bash

# 检查 Python 闭环包和 AEB-RRT* MoveIt 插件是否都安装。
# 后者位于嵌套 ament_cmake 包中，普通 ``colcon build`` 不会发现它。
if ! ros2 pkg prefix robot_safecontrol_moveit &>/dev/null; then
    echo "错误: robot_safecontrol_moveit 未安装"
    echo "请运行: bash build_aeb_moveit.sh"
    exit 1
fi
if ! ros2 pkg prefix aeb_rrtstar_ompl &>/dev/null; then
    echo "错误: AEB-RRT* MoveIt 插件 aeb_rrtstar_ompl 未安装"
    echo "请运行: bash build_aeb_moveit.sh"
    exit 1
fi
echo "  OK"

echo ""
echo "=== 启动全自动流程 ==="
echo "  move_group(AEB-RRT*) + 过渡服务器 + 被控对象 + OSCBF 控制器 + Viewer"
echo "  机械臂每次从随机工作位姿出发，自动规划无碰撞过渡到轨迹起点，"
echo "  回放完成后 OSCBF 控制器接管并跟踪蝴蝶轨迹到终点。全程无需键盘。"
echo ""

exec ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py \
    start_oscbf_plant:=true oscbf_wait_for_start:=true \
    oscbf_randomize_start:=true \
    transition_replay_topic:=/transition_replay_viz
