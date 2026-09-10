"""Synthetic contention load using real FusionEngine, no sensor/ROS access."""
import sys,time,json,signal,argparse
from pathlib import Path
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'portable_oscbf/work').is_dir());sys.path.insert(0,str(ROOT/'portable_oscbf'))
import numpy as np
from work.fusion_engine import FusionEngine
from work.safety_snapshot import SafetyGridSpec
p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();out=Path(a.output)
rng=np.random.default_rng(317);spec=SafetyGridSpec(np.array([-1.,-1.,-1.]),np.array([1.,1.,1.]),.04);engine=FusionEngine(spec)
# 2 x 12000 points, eight clusters. Fixed synthetic geometry, 10 Hz target.
centers=rng.uniform(-.7,.7,(8,3));clouds=[np.repeat(centers,1500,axis=0)+rng.normal(0,.035,(12000,3)) for _ in range(2)]
alive=True
def stop(*_):
 global alive
 alive=False
signal.signal(signal.SIGTERM,stop);times=[];valid=0;start=time.monotonic()
while alive:
 begin=time.monotonic();stamp=1000.+begin-start
 engine.feed_camera(clouds[0],stamp);engine.feed_lidar(clouds[1],stamp);r=engine.fuse(stamp)
 times.append((time.monotonic()-begin)*1000);valid+=bool(r.status['perception_valid'])
 if len(times)==1:Path(str(out)+'.ready').write_text('ready')
 time.sleep(max(0,.1-(time.monotonic()-begin)))
json.dump(dict(kind='synthetic FusionEngine contention only; outputs not fed into controller',points_per_source=12000,grid_shape=[int(x) for x in spec.shape],target_hz=10,cycles=len(times),valid_cycles=valid,wall_s=time.monotonic()-start,p50_ms=float(np.median(times)),p99_ms=float(np.percentile(times,99)),max_ms=max(times),samples_ms=times),out.open('w'),indent=2)
Path(str(out)+'.ready').unlink(missing_ok=True)
