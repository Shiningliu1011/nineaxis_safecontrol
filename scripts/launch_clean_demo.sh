#!/bin/bash
# ===================================================================
# 清场启动 MuJoCo 全流程仿真 + 异常自动记录
#
# 用法:
#   bash scripts/launch_clean_demo.sh
#
# 1. 强制终止所有残留仿真进程（SIGINT -> SIGKILL 兜底），并验证零残留；
# 2. 以 run_demo.sh 启动完整闭环，输出写入 log/clean_demo_<时间戳>/demo.log；
# 3. 后台监控器每 5 s 检查一次：
#      - launch/被控对象进程存活、是否存在第二实例争用；
#      - 日志中的 ERROR/Traceback/进程死亡/IK 失败；
#      - 跟踪指标异常（pos_err > 10 mm, cross_track > 2.0 mm）；
#    异常行自动追加到该目录下 anomalies.log（带时间戳），供事后分析。
# ===================================================================
set -uo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_DIR"

TS="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$PROJECT_DIR/log/clean_demo_$TS"
DEMO_LOG="$RUN_DIR/demo.log"
ANOMALY_LOG="$RUN_DIR/anomalies.log"
MONITOR_LOG="$RUN_DIR/monitor.log"
mkdir -p "$RUN_DIR"

PATTERNS=(
  "mujoco_transition_final.launch.py"
  "transition_planning_server"
  "mujoco_joint_state_viewer"
  "mujoco_viewer_with_cylinder"
  "mujoco_viewer"
  "oscbf_controller"
  "oscbf_plant"
  "hardware_bridge"
  "perception_bridge"
  "move_group"
  "robot_state_publisher"
)

echo "[$(date +%H:%M:%S)] 清理残留仿真进程 ..." | tee -a "$RUN_DIR/cleanup.log"
for pattern in "${PATTERNS[@]}"; do
  pkill -INT -f "$pattern" 2>/dev/null || true
done
sleep 3

# SIGKILL 兜底，最多 5 轮，直到零残留
for _ in 1 2 3 4 5; do
  leftover="$(pgrep -f "mujoco_transition_final.launch.py|oscbf_plant|mujoco_joint_state_viewer|transition_planning_server|move_group|oscbf_controller" | wc -l)"
  if [ "$leftover" -eq 0 ]; then
    break
  fi
  for pattern in "${PATTERNS[@]}"; do
    pkill -KILL -f "$pattern" 2>/dev/null || true
  done
  sleep 1
done

final_leftover="$(pgrep -f "mujoco_transition_final.launch.py|oscbf_plant|mujoco_joint_state_viewer|transition_planning_server|move_group|oscbf_controller" | wc -l)"
if [ "$final_leftover" -ne 0 ]; then
  echo "[$(date +%H:%M:%S)] 错误: 仍有 $final_leftover 个残留进程，终止。" | tee -a "$RUN_DIR/cleanup.log"
  exit 1
fi
echo "[$(date +%H:%M:%S)] 清场完成，零残留。" | tee -a "$RUN_DIR/cleanup.log"

# --- 启动完整闭环 -------------------------------------------------
echo "[$(date +%H:%M:%S)] 启动 robot_safecontrol 全流程仿真 (log: $DEMO_LOG)"
nohup bash run_demo.sh >"$DEMO_LOG" 2>&1 &
DEMO_PID=$!
echo "$DEMO_PID" > "$RUN_DIR/launch_pid.txt"

# --- 后台监控器（异常自动记录） ------------------------------------
nohup bash -c '
  DEMO_LOG="$1"; ANOMALY_LOG="$2"; MONITOR_LOG="$3"
  last_lines=0
  while true; do
    ts="$(date "+%F %H:%M:%S")"

    # 进程级异常。匹配用节点唯一标志 __node:=[o]scbf_plant (注释与消息均
    # 保持 [o] 锚定写法, 避免监控器 cmdline 自匹配本模式导致误报)。
    if ! pgrep -f "[m]ujoco_transition_final.launch.py" >/dev/null; then
      echo "[$ts] ANOMALY launch 进程已退出" >> "$ANOMALY_LOG"
    fi
    plant_count="$(pgrep -f "__node:=[o]scbf_plant" | wc -l)"
    if [ "$plant_count" -gt 1 ]; then
      echo "[$ts] ANOMALY 检测到 $plant_count 个 oscbf_plant（多实例争用 /mujoco_joint_states）" >> "$ANOMALY_LOG"
    fi

    # 日志级异常（只扫描新增行）
    if [ -f "$DEMO_LOG" ]; then
      total="$(wc -l < "$DEMO_LOG")"
      if [ "$total" -gt "$last_lines" ]; then
        new_lines="$(sed -n "$((last_lines + 1)),\$p" "$DEMO_LOG")"
        # 启动期 move_group 未就绪时的瞬态检查失败是正常时序, 不是系统异常
        filtered="$(echo "$new_lines" | grep -vE "MoveIt service check failed|No 3D sensor plugin")"
        echo "$filtered" | grep -E "\[ERROR\]|Traceback|process has died|GOAL_IK_FAILED|IK_FAILED|IK_DISCONTINUITY|PLANT_SETTLED_FAIL" >> "$ANOMALY_LOG"
        echo "$new_lines" | grep -oE "pos_err=[0-9.]+mm|cross_track=[0-9.]+mm" | while read -r metric; do
          value="$(echo "$metric" | grep -oE "[0-9.]+")"
          case "$metric" in
            pos_err=*) threshold=10.0 ;;
            cross_track=*) threshold=2.0 ;;
          esac
          if awk "BEGIN{exit !($value > $threshold)}"; then
            echo "[$ts] ANOMALY $metric (阈值 ${threshold}mm)" >> "$ANOMALY_LOG"
          fi
        done
      fi
      last_lines="$total"
    fi

    # 心脏搏动，便于事后确认监控器存活
    echo "[$ts] monitor-alive" >> "$MONITOR_LOG"
    sleep 5
  done
' _monitor "$DEMO_LOG" "$ANOMALY_LOG" "$MONITOR_LOG" >/dev/null 2>&1 &
echo "[$(date +%H:%M:%S)] 监控器已启动 (PID $!)，异常记录: $ANOMALY_LOG"
echo "[$(date +%H:%M:%S)] 完成，全流程仿真开始。日志: $DEMO_LOG"
