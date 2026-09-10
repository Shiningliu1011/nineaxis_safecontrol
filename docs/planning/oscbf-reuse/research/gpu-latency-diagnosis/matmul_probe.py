import os,time,json
from pathlib import Path
import jax
jax.config.update('jax_enable_x64',True);jax.config.update('jax_default_matmul_precision','highest')
import numpy as np
rng=np.random.default_rng(3);rows=[]
f=jax.jit(lambda a,b:a@b)
for backend in ('cpu','gpu'):
 device=jax.devices(backend)[0]
 for dtype in (np.float32,np.float64):
  for n in (9,81,1024):
   a,b=[jax.device_put(rng.standard_normal((n,n)).astype(dtype),device) for _ in range(2)]
   jax.block_until_ready((a,b));result=f(a,b);jax.block_until_ready(result)
   for _ in range(10):jax.block_until_ready(f(a,b))
   ts=[]
   for _ in range(100 if n<1024 else 20):
    t=time.perf_counter_ns();jax.block_until_ready(f(a,b));ts.append((time.perf_counter_ns()-t)/1e6)
   row={'backend':backend,'dtype':np.dtype(dtype).name,'n':n,'p50_ms':float(np.median(ts)),'p95_ms':float(np.percentile(ts,95)),'finite':bool(np.isfinite(np.asarray(result)).all())};rows.append(row);print(row,flush=True)
Path(__file__).with_name('matmul-results.json').write_text(json.dumps(rows,indent=2))
