#!/bin/bash
# ===================================================================
# 一键启动：MoveIt2 + MuJoCo 查看器 + 冗余自由度避障演示
#
# 用法:
#   bash run_demo.sh           启动全部演示 (约 3 分钟)
#   bash run_demo.sh --stop    停止所有组件
#   bash run_demo.sh --restart 重新启动
# ===================================================================
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$PROJECT_DIR"

# --- 参数解析 ---
STOP_ONLY=false
case "${1:-}" in
    --stop)   STOP_ONLY=true ;;
    --restart) ;;
    "")       ;;
    *)        echo "用法: bash run_demo.sh [--stop|--restart]"; exit 1 ;;
esac

# --- 清理旧进程 ---
_cleanup() {
    echo "清理旧进程..."
    pkill -f "demo.launch.py"       2>/dev/null || true
    pkill -f "move_group"           2>/dev/null || true
    pkill -f "ros2_control_node"    2>/dev/null || true
    pkill -f "robot_state_publisher" 2>/dev/null || true
    pkill -f "controller_manager"   2>/dev/null || true
    pkill -f "mujoco_viewer"        2>/dev/null || true
    pkill -f "joint_state_broadcaster" 2>/dev/null || true
    sleep 3
    echo "  清理完成"
}

_cleanup

if $STOP_ONLY; then
    echo "已停止所有组件。"
    exit 0
fi

# --- 1. 加载环境 ---
echo ""
echo "=== [1/5] 加载 ROS2 环境 ==="
source /opt/ros/humble/setup.bash
source install/setup.bash

# 检查必要的包
MISSING=""
for pkg in ninezzhou_moveit_config robot_safecontrol_moveit aeb_rrtstar_ompl; do
    if ! ros2 pkg prefix "$pkg" &>/dev/null; then
        MISSING="$MISSING $pkg"
    fi
done
if [ -n "$MISSING" ]; then
    echo "错误: 以下包未安装:$MISSING"
    echo "请运行: source /opt/ros/humble/setup.bash && colcon build"
    exit 1
fi
echo "  OK"

# --- 2. 启动 MoveIt2 ---
echo ""
echo "=== [2/5] 启动 MoveIt2 (move_group + mock ros2_control) ==="
ros2 launch ninezzhou_moveit_config demo.launch.py > /tmp/demo_bringup.log 2>&1 &
MOVEIT_PID=$!
echo "  PID=$MOVEIT_PID"

# 等待 move_group 就绪 (最多 40s, URDF 加载 + STL 网格需要时间)
for i in $(seq 1 40); do
    if ros2 node list 2>/dev/null | grep -q move_group; then
        echo "  move_group 就绪 (${i}s)"
        break
    fi
    sleep 1
done
if ! ros2 node list 2>/dev/null | grep -q move_group; then
    echo "错误: move_group 启动超时。日志: /tmp/demo_bringup.log"
    tail -10 /tmp/demo_bringup.log
    kill $MOVEIT_PID 2>/dev/null
    exit 1
fi

# 等待关键服务
for svc in plan_kinematic_path compute_ik check_state_validity; do
    for i in $(seq 1 20); do
        if ros2 service list 2>/dev/null | grep -q "$svc"; then break; fi
        sleep 1
    done
done
echo "  所有服务就绪"

# --- 3. 启动 MuJoCo 查看器 ---
echo ""
echo "=== [3/5] 启动 MuJoCo 查看器 ==="
echo "  (查看器窗口将弹出 — 请勿关闭)"
ros2 run robot_safecontrol_moveit mujoco_viewer \
    --ros-args -p show_dynamic_obstacles:=true \
    -p show_obstacles:=true \
    -p show_target_path:=true \
    -p show_tracking_cylinder:=true \
    > /tmp/viewer_demo.log 2>&1 &
VIEWER_PID=$!
echo "  PID=$VIEWER_PID"

# 等待查看器加载模型 (URDF → MuJoCo 转换 + 16 个动态障碍槽)
for i in $(seq 1 20); do
    if grep -q "MuJoCo viewer ready" /tmp/viewer_demo.log 2>/dev/null; then
        echo "  查看器就绪 (${i}s)"
        break
    fi
    sleep 1
done
if ! grep -q "Pre-allocated.*dynamic obstacle" /tmp/viewer_demo.log 2>/dev/null; then
    echo "  警告: 查看器可能未完全启动, 继续..."
fi

# 确认查看器订阅了 /collision_object
sleep 2
SUB_COUNT=$(ros2 topic info /collision_object 2>/dev/null | grep -c "Subscriber" || echo 0)
echo "  /collision_object 订阅者: $SUB_COUNT (应为 ≥1)"

# --- 4. 运行动态障碍基准 ---
echo ""
echo "============================================================"
echo "  运行动态障碍基准 (移动扫掠 + 突发障碍, ~2 min)"
echo "============================================================"
echo ""
timeout 180 python3 benchmarks/aeb_rrtstar/run_dynamic_obstacles.py 2>&1 | \
    grep -E "Link column|sweep_t=|offset=|SUMMARY|Total|Saved|start=|goal=" || true
RESULT_DIR="benchmarks/aeb_rrtstar/dynamic_obstacles"
LATEST_JSON=$(ls -t "$RESULT_DIR"/*.json 2>/dev/null | head -1)
echo ""
echo "  基准结果: $LATEST_JSON"

# --- 5. 运行冗余自由度避障演示 ---
echo ""
echo "============================================================"
echo "  运行冗余自由度避障演示 (~1 min)"
echo "============================================================"
echo ""
python3 benchmarks/aeb_rrtstar/demo_redundant_avoidance.py 2>&1 | \
    grep -vE "WARN.*Joint states not available|INFO.*Joint states available|GLFWError" || true

# --- 完成 ---
echo ""
echo "============================================="
echo "  演示全部完成！"
echo ""
echo "  查看器窗口应显示:"
echo "    1. 橙色球体扫过机械臂连杆柱"
echo "    2. 橙色盒子出现在连杆 x=0 位置"
echo "    3. 机械臂用不同关节配置到达同一末端位姿"
echo ""
echo "  停止所有组件:"
echo "    bash run_demo.sh --stop"
echo ""
echo "  日志文件:"
echo "    MoveIt2: /tmp/demo_bringup.log"
echo "    查看器:  /tmp/viewer_demo.log"
echo ""
echo "  进程 PID:"
echo "    MoveIt2:   $MOVEIT_PID"
echo "    查看器:    $VIEWER_PID"
echo "============================================="
echo "按 Ctrl+C 停止所有组件..."
trap "echo '停止中...'; kill $MOVEIT_PID $VIEWER_PID 2>/dev/null; echo '已停止'" EXIT
wait
