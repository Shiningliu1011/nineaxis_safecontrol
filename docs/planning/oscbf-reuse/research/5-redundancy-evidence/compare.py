"""Ticket 5 offline comparison. No ROS imports or hardware command path."""
import os
os.environ.setdefault('JAX_PLATFORMS', 'cpu')
import sys, json, subprocess, hashlib
from pathlib import Path
import importlib.metadata as md
ROOT = next(p for p in Path(__file__).resolve().parents if (p / 'portable_oscbf/work').is_dir())
sys.path.insert(0, str(ROOT / 'portable_oscbf'))
sys.path.insert(0, str(ROOT / 'portable_oscbf/vendor/dpax'))
import jax
jax.config.update('jax_enable_x64', True)
import numpy as np
from work.jax_control_facade import JaxControlLoop
from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
from work.nullspace_policy import ManipulabilityGradientPolicy
from work.tool_axis_task import task_jacobian_5d

OUT = Path(__file__).resolve().parent
robot = NineaxisManipulatorJAX()
lo, hi = np.asarray(robot.joint_lower_limits), np.asarray(robot.joint_upper_limits)
starts = {
    'tracking_start': np.array([.2303562,.1112539,1.0167209,-.6810303,-1.8294025,-.4664294,.4743473,-1.0429228,.0289233]),
    'roll_test_start': np.array([.25,.16,-.98,.53,-2.64,-.85,-.16,-.97,1.18]),
}
fk = jax.jit(robot.ee_position)
rot = jax.jit(robot.ee_rotation)
jac = jax.jit(robot.ee_jacobian)
rows = []
for strategy in ('default', 'joint_midpoint', 'existing_manipulability'):
    policy = ManipulabilityGradientPolicy(robot) if strategy == 'existing_manipulability' else None
    loop = JaxControlLoop(dt=.01, task_mode='tool_axis_5d', w_pos=40., w_orient=10.,
                          w_joint=.1, temporal_lambda=.2, enable_x64=True, nullspace_policy=policy)
    loop.init_cbf()
    for scene, q0 in starts.items():
        q = q0.copy()
        loop._last_u_safe = np.zeros(9)
        p0, r0 = np.asarray(fk(q)), np.asarray(rot(q))
        records = []
        for step in range(300):
            full = np.asarray(jac(q)); rr = np.asarray(rot(q))
            j5 = task_jacobian_5d(full, rr, r0)
            scaled = np.diag([1,1,1,.4,.4]) @ j5 @ np.diag(np.asarray(robot.joint_max_velocities))
            svals = np.linalg.svd(scaled, compute_uv=False)
            result = loop.tracking_step(q=q, task_pos=p0, task_vel=np.zeros(3), task_rot=r0,
                task_omega=np.zeros(3), kp_pos=160., kp_orient=10., kp_joint=.45,
                q_des=(lo+hi)/2 if strategy == 'joint_midpoint' else q,
                nullspace_speed_limit=.18, damping=.05)
            qnext, u, unom, err, _, _, ok, _ = result
            axis_speed = np.linalg.norm(np.cross(full[3:] @ u, rr[:,0]))
            pnext, rnext = np.asarray(fk(qnext)), np.asarray(rot(qnext))
            records.append(dict(step=step, pos_error_m=float(np.linalg.norm(pnext-p0)),
                axis_error_rad=float(np.arccos(np.clip(rnext[:,0] @ r0[:,0], -1, 1))),
                axis_speed_rad_s=float(axis_speed), min_normalized_limit_margin=float(np.min(np.minimum(qnext-lo,hi-qnext)/(hi-lo))),
                rank5=int(np.sum(svals > svals[0]*1e-10)), sigma5=float(svals[-1]),
                qp_ok=bool(ok), active_count=loop.last_qp_active_count, delta_slack=loop.last_delta_slack,
                min_h=float(np.min(loop._last_cbf_h)), qp_correction_norm=float(np.linalg.norm(u-unom))))
            q = qnext
        (OUT / f'{strategy}-{scene}.json').write_text(json.dumps(records, indent=2))
        row = dict(strategy=strategy, scene=scene, steps=len(records),
            max_position_error_m=max(r['pos_error_m'] for r in records),
            max_axis_error_rad=max(r['axis_error_rad'] for r in records),
            max_axis_speed_rad_s=max(r['axis_speed_rad_s'] for r in records),
            initial_normalized_limit_margin=float(np.min(np.minimum(q0-lo,hi-q0)/(hi-lo))),
            min_normalized_limit_margin=min(r['min_normalized_limit_margin'] for r in records),
            final_normalized_limit_margin=records[-1]['min_normalized_limit_margin'],
            sigma5_start=records[0]['sigma5'],sigma5_end=records[-1]['sigma5'],
            ranks=sorted(set(r['rank5'] for r in records)),
            qp_failures=sum(not r['qp_ok'] for r in records),
            active_step_fraction=sum(r['active_count']>0 for r in records)/len(records),
            max_delta_slack=max(r['delta_slack'] for r in records),
            min_h=min(r['min_h'] for r in records),
            max_qp_correction_norm=max(r['qp_correction_norm'] for r in records))
        rows.append(row)
        print(json.dumps(row), flush=True)
        (OUT/'summary.json').write_text(json.dumps(rows, indent=2))
metadata = dict(head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
    packages={p:md.version(p) for p in ('jax','jaxlib','numpy','cbfpy','qpax')},
    backend=jax.default_backend(), starts={k:v.tolist() for k,v in starts.items()}, steps=300,dt=.01,
    scope='fixed endpoint, Euler integration, no external obstacles, current built-in CBF geometry; not path/plant/hardware validation',
    source_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'portable_oscbf/work').glob('*.py')})
(OUT/'metadata.json').write_text(json.dumps(metadata,indent=2))
