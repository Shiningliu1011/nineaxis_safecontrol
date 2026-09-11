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
