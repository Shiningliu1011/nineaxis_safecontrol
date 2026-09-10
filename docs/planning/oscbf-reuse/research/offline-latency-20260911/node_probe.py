"""Isolated DDS domain + probe-only topic. Actual callback, no hardware endpoints."""
import os,sys,time,json,argparse
from pathlib import Path
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'portable_oscbf/work').is_dir())
sys.path.insert(0,str(ROOT/'src'))
for p in ('portable_oscbf','portable_oscbf/vendor/dpax'):sys.path.insert(0,str(ROOT/p))
os.environ['JAX_COMPILATION_CACHE_DIR']=str(ROOT/'.scratch/offline-latency-cache'/os.environ.get('JAX_PLATFORMS','cpu'))
import numpy as np
import yaml,rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from robot_safecontrol_moveit.oscbf_controller import OscbfController
p=argparse.ArgumentParser();p.add_argument('--name',required=True);p.add_argument('--aligned',action='store_true');a=p.parse_args();OUT=Path(__file__).resolve().parent
params=yaml.safe_load((ROOT/'config/oscbf_controller.yaml').read_text())['/**']['ros__parameters']
params.update(trajectory_mat=str(ROOT/'data/nurbs/ik_input.mat'),portable_oscbf_root=str(ROOT/'portable_oscbf'),portable_config_yaml=str(ROOT/'portable_oscbf/config/nineaxis.yaml'),joint_state_topic='/wayfinder_offline3/input',publish_joint_state_topic='/wayfinder_offline3/output',wait_for_start=False,perf_report_path=str(OUT/(a.name+'-node-report.md')))
ctx=Context();rclpy.init(context=ctx,domain_id=182);node=OscbfController(node_name='wayfinder_offline3',context=ctx,parameter_overrides=[Parameter(k,value=v) for k,v in params.items()]);node._timer.cancel();node._telemetry_timer.cancel()
q0=np.array([.2303562,.1112539,1.0167209,-.6810303,-1.8294025,-.4664294,.4743473,-1.0429228,.0289233]);q=q0.copy()
if a.aligned:
 sys.path.insert(0,str(ROOT/'portable_oscbf/tests'))
 from test_baseline_tracking import _work_start_configuration
 from work.ik_data_loader import load_repository_trajectory
 tr=load_repository_trajectory(params['trajectory_mat']);tr.set_surface_normal_orientation([0.,1.,0.]);q0=_work_start_configuration(tr);q=q0.copy()
# Warm public path independently; no measured callback state consumed.
for _ in range(10):node._loop.path_tracking_step(q=q0,path_state=node._loop.initial_path_state(),kp_pos=160.,kp_orient=10.,kp_joint=.45,q_des=q0,nullspace_speed_limit=.18,damping=.05)
samples=[];core=[];pub=[];original=node._publish_positions

def measured_publish(x):
 start=time.perf_counter_ns();original(x);pub.append((time.perf_counter_ns()-start)/1e6)
node._publish_positions=measured_publish
for i in range(3000):
 node._latest_q=q.copy();t=time.perf_counter_ns();node._control_tick();samples.append((time.perf_counter_ns()-t)/1e6);core.append(node._step_durations[-1])
 q=node._q_cmd_smooth.copy() if node._q_cmd_smooth is not None else q
 if node._hold_q is not None:break

def stats(x):return dict(n=len(x),p50_ms=float(np.median(x)),p95_ms=float(np.percentile(x,95)),p99_ms=float(np.percentile(x,99)),max_ms=max(x))
out=dict(domain_id=182,output_topic=params['publish_joint_state_topic'],scope='manual unpaced real _control_tick through publish return; ideal smoothed-command feedback; no DDS delivery or timer scheduling or actuator',callback=stats(samples),nested_step_once=stats(core),nested_publish=stats(pub),samples_ms=samples,initial_q=q0.tolist(),aligned_start=a.aligned,hold=bool(node._hold_q is not None),progress_m=float(node._path_state[0]),qp_failures=node._qp_fail_count,parameters=params)
(OUT/(a.name+'-node.json')).write_text(json.dumps(out,indent=2));print(json.dumps({k:v for k,v in out.items() if k not in ('samples_ms','parameters')}),flush=True);node.destroy_node();rclpy.shutdown(context=ctx)
