#!/bin/bash
# ===================================================================
# 一键启动：MoveIt2 + MuJoCo 查看器 + 过渡规划
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
echo "=== 启动完整系统 ==="
echo "  move_group(AEB-RRT*) + robot_state_publisher + planning server + Viewer"
echo ""
echo "  按 M → 调整关节 → 按 T 触发规划"
echo ""

exec ros2 launch robot_safecontrol_moveit mujoco_transition_final.launch.py
