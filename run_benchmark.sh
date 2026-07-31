#!/bin/bash
# AEB-RRT* benchmark orchestrator
# Each trial runs in a separate Python process for isolation.
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
SINGLE_RUN="$PROJECT_ROOT/src/aeb_rrtstar/single_run.py"
OUTPUT_DIR="$PROJECT_ROOT/benchmarks/aeb_rrtstar"
mkdir -p "$OUTPUT_DIR"

RAW_CSV="$OUTPUT_DIR/raw_runs.csv"
SUMMARY_CSV="$OUTPUT_DIR/summary.csv"

SCENARIOS=(
    "easy_zeros_to_mid"
    "easy_zeros_to_quarter"
    "easy_near_start"
    "medium_zeros_to_halfmax"
    "medium_mid_to_extreme"
    "hard_extreme_to_extreme"
    "hard_zeros_to_extreme"
    "regression_zero_to_ik_start"
)

PLANNERS=(
    "AEB_RRTSTAR_FAITHFUL"
    "AEB_RRTSTAR_ANYTIME"
    "OMPL_RRTSTAR"
    "OMPL_RRTCONNECT"
)

TIME_BUDGETS=(0.5 1.0 2.0 5.0)
NUM_SEEDS=30
WARMUP=2

TOTAL_RUNS=$(( ${#SCENARIOS[@]} * ${#PLANNERS[@]} * ${#TIME_BUDGETS[@]} * NUM_SEEDS ))

echo "============================================================"
echo "  AEB-RRT* Benchmark"
echo "  Scenarios:  ${#SCENARIOS[@]}"
echo "  Planners:   ${#PLANNERS[@]}"
echo "  Budgets:    ${TIME_BUDGETS[*]}"
echo "  Seeds:      $NUM_SEEDS (+$WARMUP warmup per cell)"
echo "  Total runs: $TOTAL_RUNS"
echo "  Output:     $RAW_CSV"
echo "============================================================"

# Write CSV header
HEADER="scenario_id,planner_id,seed,time_budget_s,solved,approximate,first_solution_time_s,total_time_s,raw_cost,path_states,nodes_start,nodes_goal,motion_checks,all_states_valid,all_edges_valid,failure_reason,error"
echo "$HEADER" > "$RAW_CSV"

START_TIME=$(date +%s)
COMPLETED=0
FAILED=0
SOLVED_COUNT=0

for scenario in "${SCENARIOS[@]}"; do
    for planner in "${PLANNERS[@]}"; do
        for budget in "${TIME_BUDGETS[@]}"; do
            # Warmup runs (discarded)
            for ((s=0; s<WARMUP; s++)); do
                seed=$(( (s + 1) * 100 ))
                timeout 15 python3 "$SINGLE_RUN" "$scenario" "$planner" "$seed" "$budget" > /dev/null 2>&1 || true
            done

            # Actual runs
            for ((s=1; s<=NUM_SEEDS; s++)); do
                seed=$(( s * 1000 ))
                OUTPUT=$(timeout 15 python3 "$SINGLE_RUN" "$scenario" "$planner" "$seed" "$budget" 2>/dev/null) || true
                # Extract data row (skip CSV header)
                DATA=$(echo "$OUTPUT" | tail -1)
                if [ -n "$DATA" ] && [[ "$DATA" == *,* ]]; then
                    echo "$DATA" >> "$RAW_CSV"
                    if [[ "$DATA" == *",1,"* ]]; then
                        SOLVED_COUNT=$((SOLVED_COUNT + 1))
                    fi
                else
                    FAILED=$((FAILED + 1))
                    # Write a failure row
                    echo "${scenario},${planner},${seed},${budget},0,0,,,,,,,,,process_crash," >> "$RAW_CSV"
                fi
                COMPLETED=$((COMPLETED + 1))
                if [ $((COMPLETED % 100)) -eq 0 ]; then
                    ELAPSED=$(($(date +%s) - START_TIME))
                    echo "  [$COMPLETED/$TOTAL_RUNS] ${ELAPSED}s | ${SOLVED_COUNT} solved | ${FAILED} failed"
                fi
            done
        done
    done
done

ELAPSED=$(($(date +%s) - START_TIME))
echo ""
echo "============================================================"
echo "  BENCHMARK COMPLETE"
echo "  Total:    $COMPLETED runs in ${ELAPSED}s"
echo "  Solved:   $SOLVED_COUNT"
echo "  Failed:   $FAILED"
echo "  Raw data: $RAW_CSV"
echo "============================================================"
echo ""

# Compute summary
python3 << 'PYEOF' 2>/dev/null
import csv
import numpy as np
from collections import defaultdict
from pathlib import Path

csv_path = Path("$RAW_CSV")  # shell will substitute
if not csv_path.is_file():
    print("ERROR: CSV not found")
    raise SystemExit(1)

with open(csv_path) as f:
    reader = csv.DictReader(f)
    rows = list(reader)

print(f"Read {len(rows)} rows from {csv_path}")

groups = defaultdict(list)
for row in rows:
    key = (row['scenario_id'], row['planner_id'], float(row['time_budget_s']))
    groups[key].append(row)

def pct(values, p):
    if not values:
        return float('nan')
    idx = int(np.ceil(p / 100.0 * len(values))) - 1
    return values[max(0, min(idx, len(values) - 1))]

summary_fields = [
    'scenario_id', 'planner_id', 'time_budget_s', 'num_runs', 'num_solved',
    'success_rate', 'median_time_s', 'p90_time_s', 'p95_time_s',
    'median_cost', 'p90_cost', 'mean_path_states', 'mean_motion_checks',
    'all_valid_ratio',
]

summary_path = Path("$SUMMARY_CSV")
with open(summary_path, 'w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=summary_fields)
    writer.writeheader()
    for (scenario, planner, budget), group in sorted(groups.items()):
        solved = [r for r in group if r['solved'] == '1']
        n = len(group)
        n_solved = len(solved)
        times = sorted([float(r['total_time_s']) for r in solved])
        costs = sorted([float(r['raw_cost']) for r in solved if r['raw_cost'] != 'nan'])
        row = {
            'scenario_id': scenario, 'planner_id': planner,
            'time_budget_s': budget, 'num_runs': n, 'num_solved': n_solved,
            'success_rate': n_solved / n if n > 0 else 0.0,
            'median_time_s': pct(times, 50),
            'p90_time_s': pct(times, 90),
            'p95_time_s': pct(times, 95),
            'median_cost': pct(costs, 50),
            'p90_cost': pct(costs, 90),
            'mean_path_states': float(np.mean([int(r['path_states']) for r in solved])) if solved else float('nan'),
            'mean_motion_checks': float(np.mean([int(r['motion_checks']) for r in solved])) if solved else float('nan'),
            'all_valid_ratio': float(np.mean([int(r['all_states_valid']) and int(r['all_edges_valid']) for r in solved])) if solved else float('nan'),
        }
        writer.writerow(row)

print(f"Summary written to {summary_path}")

# Print key results
print("\n=== KEY RESULTS ===")
for (scenario, planner, budget), group in sorted(groups.items()):
    solved = [r for r in group if r['solved'] == '1']
    rate = len(solved) / len(group) if group else 0
    times = sorted([float(r['total_time_s']) for r in solved])
    costs = sorted([float(r['raw_cost']) for r in solved if r['raw_cost'] != 'nan'])
    if times:
        print(f"{scenario:30s} {planner:25s} b={budget:.1f}s  "
              f"rate={rate:.0%}  med_t={pct(times,50):.4f}s  "
              f"p95_t={pct(times,95):.4f}s  med_cost={pct(costs,50):.3f}")
PYEOF

echo ""
echo "Results:"
echo "  Raw:     $RAW_CSV"
echo "  Summary: $SUMMARY_CSV"
