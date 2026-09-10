#!/usr/bin/env bash
# 全量测试入口：主包（ROS 侧）+ portable_oscbf（JAX 内核侧）。
# 依赖：ROS 2 Humble 与 install/setup.bash 必须存在（先跑 build_aeb_moveit.sh）。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_ROOT"

if [ -f /opt/ros/humble/setup.bash ]; then
  # ROS/colcon 生成的 setup 会读取未定义的可选变量。
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi
if [ ! -f install/setup.bash ]; then
  echo "未找到 install/setup.bash，请先执行: bash build_aeb_moveit.sh" >&2
  exit 1
fi
set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u

echo "==> 主包 tests/"
main_status=0
python3 -m pytest tests/ -q || main_status=$?

echo "==> portable_oscbf/tests/"
kernel_status=0
python3 -m pytest portable_oscbf/tests -q || kernel_status=$?

echo "==> 测试汇总"
echo "主包退出码: $main_status"
echo "portable_oscbf 退出码: $kernel_status"
if (( main_status == 0 && kernel_status == 0 )); then
  echo "==> 全部通过"
  exit 0
fi
echo "==> 测试未全部通过" >&2
exit 1
