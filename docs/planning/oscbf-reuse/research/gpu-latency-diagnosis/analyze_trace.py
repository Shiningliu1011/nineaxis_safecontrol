import gzip,json,collections
from pathlib import Path
root=Path(__file__).resolve().parent;repo=next(p for p in root.parents if (p/'portable_oscbf/work').is_dir());p=next((repo/'.scratch/gpu-latency-diagnosis/full-gpu-trace').rglob('*.trace.json.gz'));es=json.load(gzip.open(p,'rt'))['traceEvents'];gpu=[e for e in es if e.get('pid')==1 and e.get('ph')=='X'];steps=[e for e in es if e.get('name')=='resident_control_step' and e.get('ph')=='X'];groups=collections.defaultdict(list)
for e in gpu:groups[e['name']].append(e)
top=[]
for name,rs in groups.items():
 duration=sum(r.get('dur',0) for r in rs)
 top.append(dict(name=name,count=len(rs),count_per_step=len(rs)/len(steps),total_us=duration,mean_us=duration/len(rs)))
top.sort(key=lambda r:r['total_us'],reverse=True)
out={'steps':len(steps),'gpu_events':len(gpu),'gpu_events_per_step':len(gpu)/len(steps),'sum_device_event_us_per_step':sum(e.get('dur',0) for e in gpu)/len(steps),'mean_profiled_host_step_us':sum(e['dur'] for e in steps)/len(steps),'top_device_events':top[:30],'d2h_examples':[e for e in gpu if e['name']=='MemcpyD2H'][:3],'host_graph_execute_per_step':sum(e.get('name')=='command_buffer::execute' for e in es)/len(steps)}
(root/'trace-summary.json').write_text(json.dumps(out,indent=2));print(json.dumps({k:v for k,v in out.items() if k not in ('top_device_events','d2h_examples')},indent=2));print(json.dumps(top[:10],indent=2));print('D2H:',out['d2h_examples'][:1])
