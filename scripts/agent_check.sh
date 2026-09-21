#!/usr/bin/env bash
# Run from any directory: bash /path/to/repository/scripts/agent_check.sh
set -euo pipefail

repo_root="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
cd "$repo_root"
status=0

run_check() {
    local label="$1"
    shift
    printf '\n[check] %s\n' "$label"
    if "$@"; then
        printf '[PASS] %s\n' "$label"
    else
        printf '[FAIL] %s\n' "$label" >&2
        status=1
    fi
}

printf '%s\n' 'fast-check is heuristic feedback; it does not replace bash run_all_tests.sh.'
run_check compileall python3 -m compileall -q src tests portable_oscbf/work

pure_logic_modules=(
    robot_spec
    calibration_record
    cylinder_geometry
    drempower_can
    hardware_contract
    hardware_probe
    moveit_runtime_config
    obstacle_extractor
    oscbf_trajectory
    python_can_backend
    runtime_snapshot
    socketcan_backend
    task_target
    tracking_contract
    tracking_evaluator
    tracking_report_writer
    unit_conversion
    transition_executor
)
for module in "${pure_logic_modules[@]}"; do
    run_check "pure import: $module" env PYTHONPATH="$repo_root/src" \
        python3 -c 'import importlib, sys; importlib.import_module(sys.argv[1]); assert "rclpy" not in sys.modules, f"{sys.argv[1]} imported rclpy"' \
        "robot_safecontrol_moveit.$module"
done
run_check 'ros_conventions import without rclpy' env PYTHONPATH="$repo_root/src" \
    python3 -c 'import importlib, sys; importlib.import_module(sys.argv[1]); sys.exit(f"{sys.argv[1]} imported rclpy") if "rclpy" in sys.modules else None' \
    robot_safecontrol_moveit.ros_conventions

# Syntax only: never execute launch, calibration, vcan or hardware scripts.
while IFS= read -r -d '' script; do
    [[ -f "$script" ]] || continue
    run_check "shell syntax: $script" bash -n "$script"
done < <(git ls-files --cached --others --exclude-standard -z -- '*.sh')

run_check 'unstaged diff' git diff --check
run_check 'staged diff' git diff --cached --check

# Fixed, small, pure contract suite. No ROS startup, CAN backend or JAX rollout.
run_check 'pure contracts' python3 -m pytest -q -p no:cacheprovider \
    tests/test_hardware_contract.py \
    tests/test_unit_conversion.py \
    tests/test_drempower_can.py

printf '\n%s\n' 'fast-check is heuristic feedback; it does not replace bash run_all_tests.sh.'
exit "$status"
