#!/usr/bin/env python3
"""One-shot verification script for ticket 06A (tracking semantics characterization).

Answers, from the real repository data + pure-NumPy path state machine:

1. How the source trajectory is loaded/preprocessed (units, transform, cylinder snap).
2. PathGeometry arc-length parameterization: length, feedrate profile, omega_per_m
   profile (-> tool-axis cap), source_time vs arc monotonicity.
3. Feed-rate scheduler: which cap binds where, cross-track stop, endpoint brake,
   lead cap behavior, and whether the reference advance uses source_time at all.

NOT part of production code; read-only analysis.
"""

import sys

import numpy as np

sys.path.insert(0, "/home/lsn/robot_safecontrol/portable_oscbf")
sys.path.insert(0, "/home/lsn/robot_safecontrol/portable_oscbf/work")

from work.ik_data_loader import load_repository_trajectory  # noqa: E402
from work.path_following import (  # noqa: E402
    PathFollowingConfig,
    PathFollower,
    initial_path_follower_state,
    endpoint_braking_feedrate_limit,
)

MAT = "/home/lsn/robot_safecontrol/data/nurbs/ik_input.mat"
YAML = "/home/lsn/robot_safecontrol/portable_oscbf/config/nineaxis.yaml"

print("=" * 78)
print("[1] trajectory loading / preprocessing")
print("=" * 78)
traj = load_repository_trajectory(MAT, config_yaml_path=YAML, feedrate_scale=3.5)
print(f"Ts={traj.Ts:.6f} s  num_points={traj.num_points}  "
      f"raw_time[-1]={traj._raw_time[-1]:.4f} s  num_points*Ts={traj.num_points*traj.Ts:.4f} s")
print(f"scale={traj.scale:.6f}  T_traj_translation={traj.t_traj}")
# Mirror the production controller sequence (oscbf_controller._build_controller):
# orientation_mode=surface_normal with cylinder_axis_direction=[0,1,0],
# cylinder_center empty -> axis point fitted from the trajectory.
traj.set_surface_normal_orientation([0.0, 1.0, 0.0])
print(f"surface fit: centre={traj.surface_centre}, radius={traj.surface_radius:.6f} m, "
      f"axis={traj.surface_axis}")
print(f"feedrate (pre-scale-3.5) range: "
      f"{np.min(traj._raw_feedrate/1000.0*traj.scale):.5f} .. "
      f"{np.max(traj._raw_feedrate/1000.0*traj.scale):.5f} m/s")
print(f"feedrate (post-scale, x3.5) range: "
      f"{np.min(traj._feedrate):.5f} .. {np.max(traj._feedrate):.5f} m/s")
print(f"position range (world, m): min={np.min(traj._pos_world,axis=0)} max={np.max(traj._pos_world,axis=0)}")

# radial deviation from the fitted cylinder (should be ~0 after snap)
rel = traj._pos_world - traj.surface_centre
axial = np.outer(rel @ traj.surface_axis, traj.surface_axis)
radial = rel - axial
radial_dist = np.linalg.norm(radial, axis=1)
print(f"post-snap radial deviation from fitted radius: "
      f"max|r-R| = {np.max(np.abs(radial_dist - traj.surface_radius))*1e3:.6f} mm")

print()
print("=" * 78)
print("[2] PathGeometry arc-length parameterization")
print("=" * 78)
geom = traj.path_geometry()
print(f"points={geom.num_points} segments={geom.num_segments} "
      f"total_length_m={geom.total_length_m:.6f} m")
print(f"source_time_s[0]={geom.source_time_s[0]:.4f} "
      f"source_time_s[-1]={geom.source_time_s[-1]:.4f}")
# source_time monotonic along arc? (used for completion display only)
print(f"source_time monotone: {np.all(np.diff(geom.source_time_s) >= -1e-9)}")
print(f"source_time[-1]/num_points*Ts = {geom.source_time_s[-1]/(traj.num_points*traj.Ts):.4f}")
omega_norm = np.linalg.norm(geom.omega_per_m, axis=1)
print(f"||omega_per_m|| range: {np.min(omega_norm):.4f} .. {np.max(omega_norm):.4f} rad/m")
tool_cap = 2.0 / np.maximum(omega_norm, 1e-9)
print(f"tool-axis cap ell_dot=2.0/||omega_per_m|| range: "
      f"{np.min(tool_cap):.4f} .. {np.max(tool_cap):.4f} m/s")
print(f"nominal feedrate (interp) range: {np.min(geom.feedrate_m_s):.5f} .. "
      f"{np.max(geom.feedrate_m_s):.5f} m/s")

print()
print("=" * 78)
print("[3] scheduler: nominal vs caps along the path (no feedback, projection=reference)")
print("=" * 78)
cfg = PathFollowingConfig()
follower = PathFollower(geom, cfg)

# Seed with the path start position so the reference/projection start at 0 and
# the reference-advance numbers are the scheduler-only figures.
p_start = geom.positions_m[0]
state = initial_path_follower_state(geom, cfg, p_start)
print(f"initial seeded state: ref={state.reference_progress_m:.6f} "
      f"proj={state.projected_progress_m:.6f} cfg.lead={cfg.reference_lead_m}")

LIMITS = dict(feedrate_joint_limit_m_s=float("inf"),
              feedrate_cbf_limit_m_s=float("inf"),
              feedrate_rate_limit_m_s=float("inf"))
feedrates = []
reasons = []
ref_advance = []
steps = 4000
dt = 0.01
# Perfect tracking: the measured EE is taken to be AT the current reference
# position each cycle, so the projected progress follows the reference and the
# only limits on the advance are the schedule + lead cap (= schedule at 0 lead
# holdback).  This isolates the source feedrate schedule itself.
ee_pos = geom.sample(0.0).position_m.copy()
follower2 = PathFollower(geom, cfg)
_ = initial_path_follower_state(geom, cfg, ee_pos)
for _ in range(steps):
    step = follower2.step(ee_pos, dt_s=dt, **LIMITS)
    feedrates.append(step.feedrate_m_s)
    reasons.append(step.limiting_reason)
    ref_advance.append(step.reference.progress_m)
    if step.reference.at_endpoint:
        break
    # move the measured point to the just-commanded reference => zero error
    ee_pos = geometry_sample = geom.sample(step.reference.progress_m).position_m
    follower2.reconcile_after_motion(ee_pos, dt_s=dt)
feedrates = np.asarray(feedrates)
print(f"perfect-tracking run: steps until endpoint={len(feedrates)} "
      f"({len(feedrates)*dt:.3f} s control time; source duration 29.980 s)")
print(f"feedrate: min={np.min(feedrates):.5f} max={np.max(feedrates):.5f} "
      f"mean={np.mean(feedrates):.5f} m/s")
print(f"limiting_reason counts (no feedback caps): {dict(zip(*np.unique(reasons, return_counts=True)))}")
print(f"reference progress monotone: {bool(np.all(np.diff(ref_advance) >= -1e-9))} "
      f"final={ref_advance[-1]:.6f} / total={geom.total_length_m:.6f}")

# Where would each cap bind if it were the only limit? sample mid-path:
for idx in (0, len(geom.positions_m)//4, len(geom.positions_m)//2,
            3*len(geom.positions_m)//4, len(geom.positions_m)-2):
    s = geom.arc_length_m[idx]
    ref = geom.sample(s)
    print(f"  s={s:.4f}m feed_nom={ref.feedrate_m_s:.5f} "
          f"||omega_per_m||={np.linalg.norm(ref.omega_per_m):.4f} "
          f"tool_cap={2.0/max(np.linalg.norm(ref.omega_per_m),1e-9):.4f}")

print()
print("=" * 78)
print("[4] lead-cap freeze demonstration (projection lags reference)")
print("=" * 78)
# Clean setup: emulate a mid-path transient where the measured EE is 5 mm
# BEHIND the reference, exactly on the path (transverse error = 0, so gamma=1
# and feedrate = nominal).  The lead cap (projected + lead) then freezes the
# reference: it may never get more than 0.01 m ahead of the projection.
from work.path_following import PathFollowerState as _PFS
s0 = geom.total_length_m * 0.5
sa0, _ = geom._segment_fraction(s0)
ref_sample = geom.sample(s0)
ee_fixed = ref_sample.position_m - ref_sample.tangent * 0.005   # 5 mm behind, on-path
state4 = _PFS(reference_progress_m=s0, projected_progress_m=s0 - 0.003,
              projection_segment=int(sa0), endpoint_hold_s=0.0, completed=False)
follower4 = PathFollower(geom, cfg)
follower4.state = state4
prog4, feed4 = [], []
for _ in range(300):
    step = follower4.step(ee_fixed.copy(), dt_s=dt, **LIMITS)
    prog4.append(step.reference.progress_m)
    feed4.append(step.feedrate_m_s)
    follower4.reconcile_after_motion(ee_fixed.copy(), dt_s=dt)
print(f"seeded: ref={s0:.6f} proj={s0-0.003:.6f} (lead=0.003m) ee fixed 5mm behind on-path")
print(f"after 300 steps (3 s): ref moved {prog4[-1]-s0:+.6f} m "
      f"proj={follower4.state.projected_progress_m:.6f} "
      f"lead={follower4.state.reference_progress_m-follower4.state.projected_progress_m:.6f} m "
      f"feedrate_nominal={step.feedrate_nominal_m_s:.5f} feedrate={step.feedrate_m_s:.5f} "
      f"gamma={step.gamma:.3f} reason={step.limiting_reason} cross_track={step.cross_track_error_m*1e3:.3f} mm")
print("=> with cross-track=0 and nominal feedrate>0, a stalled EE still freezes the "
      "reference once the lead reaches reference_lead_m: the reference is coupled to "
      "the MEASURED projection, not to wall-clock time.")

print()
print("=" * 78)
print("[5] source_time_s is NOT used in path advance (code audit check)")
print("=" * 78)
# advance_path_state (numpy) receives no time argument other than dt; source_time
# only travels inside PathReference. Verify the reference carries it but nothing
# in advance_path_state reads it for scheduling:
import inspect
from work import path_following as pf
src = inspect.getsource(pf.advance_path_state)
print("advance_path_state references 'source_time':", "source_time" in src)
src2 = inspect.getsource(pf.advance_path_state)
print("advance_path_state parameters:", list(inspect.signature(pf.advance_path_state).parameters))

print()
print("=" * 78)
print("[6] endpoint brake: v=sqrt(2*a*s) values near the end")
print("=" * 78)
for rem in (0.5, 0.2, 0.1, 0.05, 0.01):
    print(f"  remaining={rem:.3f} m -> brake cap = "
          f"{endpoint_braking_feedrate_limit(rem, 0.5):.4f} m/s")
