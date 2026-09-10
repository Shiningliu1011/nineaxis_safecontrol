"""Offline GPU latency diagnosis. No ROS/hardware; precision changes only here."""
import os,sys,time,json,argparse,gzip,re,subprocess
from pathlib import Path
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'portable_oscbf/work').is_dir())
for name in ('portable_oscbf','portable_oscbf/vendor/dpax'):sys.path.insert(0,str(ROOT/name))
a=argparse.ArgumentParser();a.add_argument('--name',required=True);a.add_argument('--path',choices=['full','short'],default='full');a.add_argument('--precision',type=int,default=64);a.add_argument('--trace',action='store_true');a.add_argument('--hlo',action='store_true');a.add_argument('--n',type=int,default=100);args=a.parse_args()
OUT=Path(__file__).resolve().parent
os.environ.setdefault('JAX_COMPILATION_CACHE_DIR',str(ROOT/'docs/planning/oscbf-reuse/research/3-latency-evidence'/('gpu' if os.environ.get('JAX_PLATFORMS')=='cuda' else 'cpu')/'jit-cache'))
import jax
jax.config.update('jax_enable_x64',args.precision==64)
import numpy as np
from work.jax_control_facade import JaxControlLoop
from work.ik_data_loader import load_repository_trajectory
from work.path_following import PathGeometry,PathFollowingConfig
loop=JaxControlLoop(dt=.01,dt_path=.01,w_pos=40.,w_orient=10.,w_joint=.1,temporal_lambda=.2,task_mode='tool_axis_5d',enable_x64=args.precision==64)
q=np.array([.2303562,.1112539,1.0167209,-.6810303,-1.8294025,-.4664294,.4743473,-1.0429228,.0289233],dtype=np.float64 if args.precision==64 else np.float32)
if args.path=='full':
 trajectory=load_repository_trajectory(str(ROOT/'data/nurbs/ik_input.mat'),feedrate_scale=3.5);trajectory.set_surface_normal_orientation([0.,1.,0.]);geometry=trajectory.path_geometry()
else:
 p=np.asarray(loop.robot.ee_position(q));r=np.asarray(loop.robot.ee_rotation(q));geometry=PathGeometry.from_samples(np.array([p,p+[.001,0.,0.]]),np.array([r,r]),np.array([.05,.05]),np.array([0.,.02]))
loop.configure_path(geometry,PathFollowingConfig());start=time.perf_counter();loop.init_cbf();warm=time.perf_counter()-start
kw=dict(q=q,path_state=loop.initial_path_state(),kp_pos=160.,kp_orient=10.,kp_joint=.45,q_des=q,nullspace_speed_limit=.18,damping=.05,u_safe_prev=np.zeros(9,dtype=q.dtype))
raw=loop._path_tracking_fn;captured={}
def capture(*x):captured['args']=x;return raw(*x)
loop._path_tracking_fn=capture;result=loop.path_tracking_step(**kw);loop._path_tracking_fn=raw
xs=jax.block_until_ready(jax.tree.map(jax.numpy.asarray,captured['args']))
def measure(fn,n=args.n):
 for _ in range(10):jax.block_until_ready(fn())
 v=[]
 for _ in range(n):
  t=time.perf_counter_ns();jax.block_until_ready(fn());v.append((time.perf_counter_ns()-t)/1e6)
 return {'p50':float(np.percentile(v,50)),'p95':float(np.percentile(v,95)),'max':max(v),'samples':v}
monitor=None
if os.environ.get('GPU_MONITOR')=='1':
 monitor_file=(OUT/(args.name+'-power.csv')).open('w')
 monitor=subprocess.Popen(['nvidia-smi','--query-gpu=timestamp,pstate,clocks.sm,clocks.mem,power.draw,temperature.gpu,utilization.gpu,clocks_throttle_reasons.active','--format=csv','--loop-ms=200'],stdout=monitor_file,stderr=subprocess.STDOUT)
out={'name':args.name,'backend':jax.default_backend(),'device':str(jax.devices()[0]),'precision':args.precision,'path':args.path,'flags':os.environ.get('XLA_FLAGS'),'warmup_s':warm,'public':measure(lambda:loop.path_tracking_step(**kw)),'resident':measure(lambda:raw(*xs)),'iterations':float(loop.last_qp_iterations) if hasattr(loop,'last_qp_iterations') else None,'qp_ok':bool(result.qp_ok),'u_safe':result.u_safe.tolist(),'u_nom':result.u_nom.tolist()}
if monitor is not None:
 monitor.terminate();monitor.wait();monitor_file.close()
# The verdict is a diagnostic symptom signal, not a production safety threshold.
(OUT/(args.name+'.json')).write_text(json.dumps(out,indent=2));print(json.dumps({k:v for k,v in out.items() if k not in ('u_safe','u_nom','public','resident')},default=str),flush=True);print('PUBLIC/RESIDENT p50:',out['public']['p50'],out['resident']['p50'],flush=True)
if args.hlo:
 hlo=raw.lower(*xs).compile().as_text()
 with gzip.open(OUT/(args.name+'.hlo.txt.gz'),'wt') as f:f.write(hlo)
 print('HLO chars:',len(hlo),flush=True)
if args.trace:
 with jax.profiler.trace(str(OUT/(args.name+'-trace'))):
  for i in range(20):
   with jax.profiler.TraceAnnotation('resident_control_step',step_num=i):jax.block_until_ready(raw(*xs))
 print('trace complete',flush=True)
