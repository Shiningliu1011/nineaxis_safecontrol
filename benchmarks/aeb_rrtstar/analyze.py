#!/usr/bin/env python3
"""Analyze AEB-RRT* vs OMPL comparison benchmark results."""
import csv
import sys
import numpy as np
from collections import defaultdict
from pathlib import Path

CSV_PATH = Path(__file__).parent / "comparison.csv"
if not CSV_PATH.exists():
    print(f"ERROR: {CSV_PATH} not found. Run the benchmark first.")
    sys.exit(1)

with open(CSV_PATH) as f:
    rows = list(csv.DictReader(f))

print(f"Loaded {len(rows)} benchmark runs\n")

# Group by (scenario, planner, budget)
groups = defaultdict(list)
for r in rows:
    key = (r['scenario_id'], r['planner_id'], float(r['time_budget_s']))
    groups[key].append(r)

def pct(values, p):
    if not values: return float('nan')
    idx = int(np.ceil(p/100.0 * len(values))) - 1
    return sorted(values)[max(0, min(idx, len(values)-1))]

# ===================================================================
#  Per-scenario summary
# ===================================================================
print("=" * 110)
print(f"{'Scenario':<28s} {'Planner':<22s} {'Budget':>6s}  {'Rate':>6s}  {'MedTime':>8s}  {'P95Time':>8s}  {'MedCost':>8s}  {'States':>7s}  {'Valid':>5s}")
print("-" * 110)

for (scenario, planner, budget), group in sorted(groups.items()):
    solved = [r for r in group if r['solved'] == '1']
    n = len(group); ns = len(solved)
    times = sorted([float(r['total_time_s']) for r in solved])
    costs = sorted([float(r['raw_cost']) for r in solved if r.get('raw_cost','nan') != 'nan'])
    states = [int(r['path_states']) for r in solved]
    valid = [int(r['all_states_valid']) and int(r['all_edges_valid']) for r in solved]

    rate = f"{ns/n:.0%}" if n > 0 else "N/A"
    mt = f"{pct(times,50):.4f}s" if times else "N/A"
    p95 = f"{pct(times,95):.4f}s" if times else "N/A"
    mc = f"{pct(costs,50):.4f}" if costs else "N/A"
    st = f"{int(np.mean(states))}" if states else "N/A"
    vl = f"{sum(valid)}/{len(valid)}" if valid else "N/A"

    print(f"{scenario:<28s} {planner:<22s} {budget:>5.1f}s  {rate:>6s}  {mt:>8s}  {p95:>8s}  {mc:>8s}  {st:>7s}  {vl:>5s}")

# ===================================================================
#  Head-to-head: AEB Faithful vs RRTConnect (current default)
# ===================================================================
print("\n" + "=" * 110)
print("HEAD-TO-HEAD: AEB-RRT* Faithful vs OMPL RRTConnect (2.0s budget)")
print("-" * 110)
print(f"{'Scenario':<28s} {'AEB Time':>9s} {'RRTCon Time':>11s} {'Delta':>7s}  {'AEB Cost':>9s} {'RRTCon Cost':>11s} {'Delta':>7s}  {'Winner':>10s}")
print("-" * 110)

for scenario_id in sorted(set(r['scenario_id'] for r in rows)):
    aeb = [r for r in rows if r['scenario_id']==scenario_id and r['planner_id']=='AEB_RRTSTAR_FAITHFUL' and float(r['time_budget_s'])==2.0 and r['solved']=='1']
    rrt = [r for r in rows if r['scenario_id']==scenario_id and r['planner_id']=='OMPL_RRTCONNECT' and float(r['time_budget_s'])==2.0 and r['solved']=='1']

    if not aeb or not rrt: continue

    aeb_t = np.median([float(r['total_time_s']) for r in aeb])
    rrt_t = np.median([float(r['total_time_s']) for r in rrt])
    aeb_c = np.median([float(r['raw_cost']) for r in aeb if r.get('raw_cost','nan')!='nan'])
    rrt_c = np.median([float(r['raw_cost']) for r in rrt if r.get('raw_cost','nan')!='nan'])

    dt = (aeb_t - rrt_t) / rrt_t * 100 if rrt_t > 0 else 0
    dc = (aeb_c - rrt_c) / rrt_c * 100 if rrt_c > 0 else 0

    # Winner: faster AND better cost wins
    winner = "AEB-RRT*" if (dt <= 0 and dc <= 0) else ("RRTConnect" if (dt >= 0 and dc >= 0) else "Mixed")

    print(f"{scenario_id:<28s} {aeb_t:>8.4f}s {rrt_t:>10.4f}s {dt:>+6.1f}%  {aeb_c:>8.4f}  {rrt_c:>10.4f}  {dc:>+6.1f}%  {winner:>10s}")

# ===================================================================
#  OMPL RRT* comparison
# ===================================================================
print("\n" + "=" * 110)
print("AEB-RRT* Faithful vs OMPL RRT* (unidirectional)")
print("-" * 110)
for scenario_id in sorted(set(r['scenario_id'] for r in rows)):
    aeb = [r for r in rows if r['scenario_id']==scenario_id and r['planner_id']=='AEB_RRTSTAR_FAITHFUL' and float(r['time_budget_s'])==2.0 and r['solved']=='1']
    rrs = [r for r in rows if r['scenario_id']==scenario_id and r['planner_id']=='OMPL_RRTSTAR' and float(r['time_budget_s'])==2.0 and r['solved']=='1']

    if not aeb or not rrs: continue

    aeb_t = np.median([float(r['total_time_s']) for r in aeb])
    rrs_t = np.median([float(r['total_time_s']) for r in rrs])
    aeb_c = np.median([float(r['raw_cost']) for r in aeb if r.get('raw_cost','nan')!='nan'])
    rrs_c = np.median([float(r['raw_cost']) for r in rrs if r.get('raw_cost','nan')!='nan'])

    speedup = rrs_t / aeb_t if aeb_t > 0 else float('inf')
    cost_ratio = aeb_c / rrs_c if rrs_c > 0 else float('inf')

    print(f"  {scenario_id:<28s} AEB={aeb_t:.4f}s vs RRT*={rrs_t:.4f}s (AEB {speedup:.1f}x faster, cost={cost_ratio:.2f}x)")

# ===================================================================
#  Overall summary
# ===================================================================
print("\n" + "=" * 110)
print("OVERALL SUMMARY (2.0s budget)")
print("=" * 110)

aeb_all = [r for r in rows if r['planner_id']=='AEB_RRTSTAR_FAITHFUL' and float(r['time_budget_s'])==2.0 and r['solved']=='1']
rrc_all = [r for r in rows if r['planner_id']=='OMPL_RRTCONNECT' and float(r['time_budget_s'])==2.0 and r['solved']=='1']
rrs_all = [r for r in rows if r['planner_id']=='OMPL_RRTSTAR' and float(r['time_budget_s'])==2.0 and r['solved']=='1']

aeb_times = [float(r['total_time_s']) for r in aeb_all] if aeb_all else [0]
rrc_times = [float(r['total_time_s']) for r in rrc_all] if rrc_all else [0]
rrs_times = [float(r['total_time_s']) for r in rrs_all] if rrs_all else [0]
aeb_costs = [float(r['raw_cost']) for r in aeb_all if r.get('raw_cost','nan')!='nan']
rrc_costs = [float(r['raw_cost']) for r in rrc_all if r.get('raw_cost','nan')!='nan']
rrs_costs = [float(r['raw_cost']) for r in rrs_all if r.get('raw_cost','nan')!='nan']
aeb_valid = sum(1 for r in aeb_all if int(r.get('all_states_valid',0)) and int(r.get('all_edges_valid',0)))

print(f"{'':>20s} {'AEB Faithful':>15s} {'RRTConnect':>15s} {'RRT*':>15s}")
print(f"{'Success rate':>20s} {len(aeb_all)/max(len([r for r in rows if r['planner_id']=='AEB_RRTSTAR_FAITHFUL' and float(r['time_budget_s'])==2.0]),1):>14.0%} {len(rrc_all)/max(len([r for r in rows if r['planner_id']=='OMPL_RRTCONNECT' and float(r['time_budget_s'])==2.0]),1):>14.0%} {len(rrs_all)/max(len([r for r in rows if r['planner_id']=='OMPL_RRTSTAR' and float(r['time_budget_s'])==2.0]),1):>14.0%}")
print(f"{'Median time':>20s} {np.median(aeb_times):>14.4f}s {np.median(rrc_times):>14.4f}s {np.median(rrs_times):>14.4f}s")
print(f"{'P95 time':>20s} {pct(aeb_times,95):>14.4f}s {pct(rrc_times,95):>14.4f}s {pct(rrs_times,95):>14.4f}s")
print(f"{'Median cost':>20s} {np.median(aeb_costs) if aeb_costs else float('nan'):>14.4f} {np.median(rrc_costs) if rrc_costs else float('nan'):>14.4f} {np.median(rrs_costs) if rrs_costs else float('nan'):>14.4f}")
print(f"{'Avg path states':>20s} {np.mean([int(r['path_states']) for r in aeb_all]):>14.1f} {np.mean([int(r['path_states']) for r in rrc_all]):>14.1f} {np.mean([int(r['path_states']) for r in rrs_all]):>14.1f}")
print(f"{'Path valid ratio':>20s} {aeb_valid}/{len(aeb_all):>13d} {'N/A':>15s} {'N/A':>15s}")

print("\nDone.")
