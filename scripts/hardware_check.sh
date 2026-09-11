#!/usr/bin/env bash
# No ROS sourcing/build required. Default doctor never opens CAN.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$REPO_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
if [ "$#" -eq 0 ]; then set -- doctor; fi
exec python3 -m robot_safecontrol_moveit.hardware_probe "$@" --config "$REPO_ROOT/config/drempower.yaml"
