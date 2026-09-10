"""Throwaway offline measurement. No ROS imports, publishers or robot access."""
import os,sys,time,json,hashlib,subprocess,argparse,csv
from pathlib import Path
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'portable_oscbf/work').is_dir())
for p in ('portable_oscbf','portable_oscbf/vendor/dpax'):sys.path.insert(0,str(ROOT/p))
P=argparse.ArgumentParser();P.add_argument('--name',required=True);P.add_argument('--max-steps',type=int,default=20000);P.add_argument('--stress-steps',type=int,default=3000);args=P.parse_args()
OUT=Path(__file__).resolve().parent
os.environ['JAX_COMPILATION_CACHE_DIR']=str(ROOT/'.scratch/offline-latency-cache'/os.environ.get('JAX_PLATFORMS','cpu'))
import jax
jax.config.update('jax_enable_x64',True)
import numpy as np
from work.jax_control_facade import JaxControlLoop
from work.ik_data_loader import load_repository_trajectory
from work.path_following import PathFollowingConfig

def hashes():
 return {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(list((ROOT/'portable_oscbf/work').glob('*.py'))+[ROOT/'src/robot_safecontrol_moveit/oscbf_controller.py',ROOT/'config/oscbf_controller.yaml'])}
def stats(x):
 x=np.asarray(x);return dict(n=len(x),p50_ms=float(np.percentile(x,50)),p95_ms=float(np.percentile(x,95)),p99_ms=float(np.percentile(x,99)),max_ms=float(x.max()),over10_pct=float(np.mean(x>10)*100),over20_pct=float(np.mean(x>20)*100))
meta=dict(head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),hashes_before=hashes(),backend=jax.default_backend(),devices=[str(x) for x in jax.devices()],x64=jax.config.x64_enabled,jax=jax.__version__,load_start=os.getloadavg(),dt=.01,model='ideal q_next feedback; unpaced; no output lowpass/ROS/CAN',trajectory_sha256=hashlib.sha256((ROOT/'data/nurbs/ik_input.mat').read_bytes()).hexdigest())
trajectory=load_repository_trajectory(str(ROOT/'data/nurbs/ik_input.mat'),feedrate_scale=3.5);trajectory.set_surface_normal_orientation([0.,1.,0.]);geometry=trajectory.path_geometry()
meta.update(path_length_m=float(geometry.arc_length_m[-1]),path_samples=len(geometry.arc_length_m),source_duration_s=float(geometry.source_time_s[-1]))
loop=JaxControlLoop(dt=.01,dt_path=.01,w_pos=40.,w_orient=10.,w_joint=.1,temporal_lambda=.2,task_mode='tool_axis_5d',enable_x64=True)
loop.configure_path(geometry,PathFollowingConfig(reference_lead_m=.01));t=time.perf_counter();loop.init_cbf();meta['init_including_warmup_s']=time.perf_counter()-t
q0=np.array([.2303562,.1112539,1.0167209,-.6810303,-1.8294025,-.4664294,.4743473,-1.0429228,.0289233]);ee0=np.asarray(loop.robot.ee_position(q0))
results={}
for scene in ('full_disabled','full_far8','moving_near8','far8_unloaded','far8_fusion_load'):
 kw=dict(q=q0.copy(),path_state=loop.initial_path_state(),kp_pos=160.,kp_orient=10.,kp_joint=.45,q_des=q0.copy(),nullspace_speed_limit=.18,damping=.05,u_safe_prev=np.zeros(9))
 if scene!='full_disabled':kw.update(obs_pos=np.tile([10.,10.,10.],(8,1)),obs_radii=np.full(8,.1),obs_enabled=np.ones(8))
 if scene=='moving_near8':
  # Synthetic obstacles surrounding INITIAL TCP; some may overlap the arm.
  # This is an adverse workload, never a collision-free fixture claim.
  angles=np.arange(8)*np.pi/4
  centers=ee0+np.column_stack([.25*np.cos(angles),.25*np.sin(angles),np.linspace(-.15,.15,8)])
  kw.update(obs_pos=centers,obs_radii=np.full(8,.08),obs_vel=np.zeros((8,3)))
 for _ in range(10):loop.path_tracking_step(**kw)
 worker=None;log=None
 if scene=='far8_fusion_load':
  log=(OUT/(args.name+'-fusion.log')).open('w');worker=subprocess.Popen([sys.executable,str(OUT/'fusion_load.py'),'--output',str(OUT/(args.name+'-fusion.json'))],stdout=log,stderr=subprocess.STDOUT)
  ready=OUT/(args.name+'-fusion.json.ready')
  deadline=time.monotonic()+45
  while not ready.exists():
   if worker.poll() is not None or time.monotonic()>deadline:raise RuntimeError('fusion worker failed to become ready')
   time.sleep(.1)
 values=[];failures=0;max_err=0.;minimum=float('inf');complete=False;reason='step_cap';wall=time.perf_counter()
 limit=args.max_steps if scene.startswith('full_') else args.stress_steps
 try:
  with (OUT/(args.name+'-'+scene+'.csv')).open('w') as f:
   writer=csv.writer(f);writer.writerow(['step','public_ms','progress_m','projected_m','qp_ok','cross_track_m','min_obs_dist_m','gamma','reason_code','qp_iterations'])
   for i in range(limit):
    if scene=='moving_near8':
     phase=i*.01;shift=.04*np.sin(phase)
     kw.update(obs_pos=centers+np.array([shift,0.,0.]),obs_vel=np.tile([.04*np.cos(phase),0.,0.],(8,1)))
    t=time.perf_counter_ns();r=loop.path_tracking_step(**kw);dt=(time.perf_counter_ns()-t)/1e6
    values.append(dt);failures+=not r.qp_ok;max_err=max(max_err,float(r.cross_track_error_m));minimum=min(minimum,float(r.min_obs_dist))
    writer.writerow([i,dt,*r.path_state[:2],int(r.qp_ok),r.cross_track_error_m,r.min_obs_dist,r.gamma,r.limiting_reason_code,getattr(loop,'last_qp_iterations',None)])
    if not np.all(np.isfinite(r.q_next)) or not np.all(np.isfinite(r.u_safe)):reason='nonfinite';break
    kw.update(q=r.q_next,q_des=r.q_next,path_state=r.path_state,u_safe_prev=r.u_safe)
    complete=bool(r.path_state[4]>.5)
    if (i+1)%1000==0:print(args.name,scene,i+1,'progress',float(r.path_state[0]),'/',meta['path_length_m'],'failures',failures,flush=True)
    if complete and scene.startswith('full_'):reason='completed';break
 finally:
  if worker:
   worker.terminate();worker.wait(timeout=30);log.close()
   if worker.returncode!=0:raise RuntimeError('fusion load failed: '+str(worker.returncode))
 row=dict(timing=stats(values),wall_s=time.perf_counter()-wall,simulated_s=len(values)*.01,qp_failures=failures,completed=complete,stop_reason=reason,progress_m=float(r.path_state[0]),progress_fraction=float(r.path_state[0]/meta['path_length_m']),max_cross_track_m=max_err,min_obstacle_distance_m=minimum,final_q=r.q_next.tolist())
 results[scene]=row;print(args.name,scene,json.dumps(row),flush=True)
 (OUT/(args.name+'.json')).write_text(json.dumps(dict(metadata=meta,scenes=results),indent=2))
meta.update(hashes_after=hashes(),load_end=os.getloadavg());meta['sources_unchanged']=meta['hashes_before']==meta['hashes_after']
(OUT/(args.name+'.json')).write_text(json.dumps(dict(metadata=meta,scenes=results),indent=2));assert meta['sources_unchanged']
