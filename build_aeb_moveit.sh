#!/usr/bin/env bash
# Build both packages in this repository.
#
# The AEB-RRT* plugin is a nested ament_cmake package, so a plain
# `colcon build` at the repository root does not discover it.

# ROS Humble's setup scripts reference optional shell variables, so enable
# nounset only after sourcing them (or simply keep the wrapper compatible
# with a clean interactive shell).
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --base-paths "$project_dir" "$project_dir/src/aeb_rrtstar_ompl" \
  --packages-select aeb_rrtstar_ompl robot_safecontrol_moveit

echo
echo "Build complete. In each terminal run:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source $project_dir/install/setup.bash"
