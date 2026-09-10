"""Summarize raw offline replay CSV; recompute tails without rounding inputs."""
import csv,json
from pathlib import Path
import numpy as np
P=Path(__file__).resolve().parent
out={}
for backend in ('cpu','gpu'):
 p=P/(backend+'.json')
 if not p.exists():continue
 report=json.loads(p.read_text());rows={}
 for scene in report['scenes']:
  data=list(csv.DictReader((P/(backend+'-'+scene+'.csv')).open()))
  times=np.array([float(r['public_ms']) for r in data]);errors=np.array([float(r['cross_track_m']) for r in data]);iterations=np.array([int(r['qp_iterations']) for r in data])
  rows[scene]=dict(n=len(data),p50=float(np.percentile(times,50)),p95=float(np.percentile(times,95)),p99=float(np.percentile(times,99)),maximum=float(times.max()),qp_iterations_p50=float(np.median(iterations)),qp_iterations_max=int(iterations.max()),initial_cross_track_m=float(errors[0]),cross_track_p95_m=float(np.percentile(errors,95)),final_cross_track_m=float(errors[-1]),last_progress_m=float(data[-1]['progress_m']))
  assert len(data)==report['scenes'][scene]['timing']['n']
  assert abs(rows[scene]['p99']-report['scenes'][scene]['timing']['p99_ms'])<1e-9
 out[backend]=rows
(P/'analysis.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
