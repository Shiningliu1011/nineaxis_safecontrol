#!/usr/bin/env bash
# 全量测试入口：主包（ROS 侧）+ portable_oscbf（JAX 内核侧）。
# 依赖：ROS 2 Humble 与 install/setup.bash 必须存在（先跑 build_aeb_moveit.sh）。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_ROOT"

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [ ! -f install/setup.bash ]; then
  echo "未找到 install/setup.bash，请先执行: bash build_aeb_moveit.sh" >&2
  exit 1
fi
# shellcheck disable=SC1091
source install/setup.bash

echo "==> 主包 tests/"
python3 -m pytest tests/ -q

echo "==> portable_oscbf/tests/"
python3 -m pytest portable_oscbf/tests -q

echo "==> 全部通过"
