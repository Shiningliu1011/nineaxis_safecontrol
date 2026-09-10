"""Throwaway ticket 3 offline profiler: no ROS or hardware publishing.
Run with JAX_PLATFORMS=cpu OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 <this file>.
Module timings are separately dispatched probes, NOT additive fused-kernel costs.
"""
import os, sys, time, json, hashlib, subprocess, types, argparse
from pathlib import Path
from collections import defaultdict
import importlib.metadata as md
ROOT = next(p for p in Path(__file__).resolve().parents if (p/'portable_oscbf/work').is_dir())
for p in ('portable_oscbf','portable_oscbf/vendor/dpax'):
    sys.path.insert(0,str(ROOT/p))
a=argparse.ArgumentParser(); a.add_argument('--samples',type=int,default=300); a.add_argument('--output',default='cpu'); args=a.parse_args()
OUT=Path(__file__).resolve().parent/args.output; OUT.mkdir(exist_ok=True)
os.environ['JAX_COMPILATION_CACHE_DIR']=str(OUT/'jit-cache')
import jax
jax.config.update('jax_enable_x64',True)
import numpy as np
from work.jax_control_facade import JaxControlLoop
from work.ik_data_loader import load_repository_trajectory
from work.path_following import PathFollowingConfig
from work.actuator_limits import command_delta_limits, load_actuator_limit_profile

def block(x): return jax.block_until_ready(x)
def timed(fn):
    t=time.perf_counter_ns(); out=block(fn()); return out,(time.perf_counter_ns()-t)/1e6
def stats(v):
    x=np.array(v); return dict(n=len(v),p50_ms=float(np.percentile(x,50)),p95_ms=float(np.percentile(x,95)),p99_ms=float(np.percentile(x,99)),max_ms=float(x.max()),over10_pct=float(np.mean(x>10)*100),over20_pct=float(np.mean(x>20)*100))
def bench(fn,n=None):
    for _ in range(10): block(fn())
    values=[timed(fn)[1] for _ in range(n or args.samples)]
    return dict(summary=stats(values),samples_ms=values)
sources=list((ROOT/'portable_oscbf/work').glob('*.py'))+[ROOT/'config/oscbf_controller.yaml',ROOT/'data/nurbs/ik_input.mat']
hashes=lambda:{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
meta=dict(head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),packages={p:md.version(p) for p in ('jax','jaxlib','numpy','cbfpy','qpax')},devices=[str(d) for d in jax.devices()],x64=jax.config.x64_enabled,env={k:os.environ.get(k) for k in ('JAX_PLATFORMS','OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','XLA_FLAGS')},source_hashes_before=hashes(),platform=subprocess.check_output(['uname','-a'],text=True),load_start=os.getloadavg(),sample_count=args.samples)
(OUT/'metadata.json').write_text(json.dumps(meta,indent=2))
trajectory=load_repository_trajectory(str(ROOT/'data/nurbs/ik_input.mat'),feedrate_scale=3.5)
trajectory.set_surface_normal_orientation([0.,1.,0.]); geometry=trajectory.path_geometry()
q0=np.array([.2303562,.1112539,1.0167209,-.6810303,-1.8294025,-.4664294,.4743473,-1.0429228,.0289233])
report={}
for mode in ('elastic','rate_slack'):
    opts={} if mode=='elastic' else dict(rate_limit_du_max=command_delta_limits(load_actuator_limit_profile(),dt_s=.01),rate_limit_penalty=1e3)
    loop=JaxControlLoop(dt=.01,dt_path=.01,w_pos=40.,w_orient=10.,w_joint=.1,temporal_lambda=.2,task_mode='tool_axis_5d',enable_x64=True,**opts)
    loop.configure_path(geometry,PathFollowingConfig(reference_lead_m=.01))
    t=time.perf_counter(); loop.init_cbf(); report[mode]={'initialization_and_warmup_s':time.perf_counter()-t}
    raw=loop._path_tracking_fn
    kw=dict(q=q0,path_state=loop.initial_path_state(),kp_pos=160.,kp_orient=10.,kp_joint=.45,q_des=q0,nullspace_speed_limit=.18,damping=.05,u_safe_prev=np.zeros(9))
    capture={}
    def capture_raw(*xs): capture['args']=xs; return raw(*xs)
    loop._path_tracking_fn=capture_raw
    r,first=timed(lambda:loop.path_tracking_step(**kw)); loop._path_tracking_fn=raw
    report[mode]['first_public_call_ms']=first
    for scene in ('disabled','enabled_far'):
        skw=dict(kw)
        if scene=='enabled_far':
            skw.update(obs_pos=np.tile([10.,10.,10.],(8,1)),obs_radii=np.full(8,.1),obs_enabled=np.ones(8))
        loop._path_tracking_fn=capture_raw
        reference=loop.path_tracking_step(**skw); loop._path_tracking_fn=raw
        xs=capture['args']; device_xs=block(jax.tree.map(jax.numpy.asarray,xs))
        row={}
        for flag in (True,False):
            loop.collect_cbf_diagnostics=flag
            row['public_diagnostics_'+str(flag)]=bench(lambda:loop.path_tracking_step(**skw))
        loop.collect_cbf_diagnostics=True
        row['fused_device_resident']=bench(lambda:raw(*device_xs))
        ready=block(raw(*device_xs))
        row['host_materialize_ready_outputs']=bench(lambda:jax.tree.map(np.asarray,ready))
        # Host->device boundary for all arguments; CPU aliases/copies are not GPU transfer evidence.
        row['input_device_put']=bench(lambda:jax.device_put(xs))
        original_prepare=loop._prepare_jax_inputs; parts=defaultdict(list)
        def prepare(*x,**k):
            result,dt=timed(lambda:original_prepare(*x,**k)); parts['prepare_inputs'].append(dt); return result
        def sync_raw(*x):
            result,dt=timed(lambda:raw(*x)); parts['fused_with_boundary'].append(dt); return result
        original_diag=loop._update_qp_diagnostics
        def diag(*x,**k):
            t=time.perf_counter_ns(); result=original_diag(*x,**k); parts['diagnostic_host_ready'].append((time.perf_counter_ns()-t)/1e6); return result
        loop._prepare_jax_inputs=prepare; loop._path_tracking_fn=sync_raw; loop._update_qp_diagnostics=diag
        row['instrumented_public']=bench(lambda:loop.path_tracking_step(**skw))
        loop._prepare_jax_inputs=original_prepare; loop._path_tracking_fn=raw; loop._update_qp_diagnostics=original_diag
        row['instrumented_parts']={k:stats(v[10:]) for k,v in parts.items()}
        # Recompose existing exported module functions with synchronization for attribution only.
        fun=raw.__wrapped__; module_times=defaultdict(list)
        def wrap(name,f):
            def call(*x,**k):
                result,dt=timed(lambda:f(*x,**k)); module_times[name].append(dt); return result
            return call
        def cell(x): return (lambda:x).__closure__[0]
        cells=tuple(cell(wrap(name,c.cell_contents)) if name.startswith('_path_') and callable(c.cell_contents) else c for name,c in zip(fun.__code__.co_freevars,fun.__closure__))
        split=types.FunctionType(fun.__code__,fun.__globals__,fun.__name__,fun.__defaults__,cells)
        split_result=block(split(*device_xs))
        differences=[]
        for left,right in zip(jax.tree.leaves(ready),jax.tree.leaves(split_result)):
            np.testing.assert_allclose(np.asarray(left),np.asarray(right),rtol=1e-7,atol=1e-7,equal_nan=True)
        row['module_vs_fused_allclose']='rtol=atol=1e-7 passed'
        row['split_total']=bench(lambda:split(*device_xs),100)
        row['separately_dispatched_modules']={k:stats(v[11:]) for k,v in module_times.items()}
        if mode=='rate_slack':
            obs={k:v for k,v in skw.items() if k.startswith('obs_')}
            problem=block(loop.freeze_qp_problem(q0,reference.u_nom,u_safe_prev=np.zeros(9),**obs))
            row['qp_shapes']=[list(v.shape) for v in problem]
            row['freeze_problem']=bench(lambda:loop.freeze_qp_problem(q0,reference.u_nom,u_safe_prev=np.zeros(9),**obs))
            row['solve_frozen']=bench(lambda:loop.solve_frozen_qp_problem(problem))
        # Moving trajectory sample, preserving q and path history, explicit previous command.
        moving=dict(skw); durations=[]; failures=0
        for i in range(args.samples):
            result,dt=timed(lambda:loop.path_tracking_step(**moving)); durations.append(dt); failures+=not result.qp_ok
            moving.update(q=result.q_next,q_des=result.q_next,path_state=result.path_state,u_safe_prev=result.u_safe)
        row['moving_replay']=dict(summary=stats(durations),samples_ms=durations,qp_failures=failures,final_progress_m=float(result.path_state[0]))
        row['fixed_qp_ok']=bool(reference.qp_ok)
        report[mode][scene]=row
        (OUT/'results.json').write_text(json.dumps(report,indent=2)); print(mode,scene,json.dumps(row['moving_replay']['summary']),flush=True)
meta.update(source_hashes_after=hashes(),load_end=os.getloadavg()); meta['sources_unchanged']=meta['source_hashes_before']==meta['source_hashes_after']
(OUT/'metadata.json').write_text(json.dumps(meta,indent=2))
assert meta['sources_unchanged'], 'source changed during measurement'
