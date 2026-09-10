import json,sys
from pathlib import Path
p=Path(__file__).resolve().parent
cpu=json.loads((p/(sys.argv[1]+'.json')).read_text());gpu=json.loads((p/(sys.argv[2]+'.json')).read_text())
r=gpu['resident']['p50']/cpu['resident']['p50'];print(f"GPU/CPU resident p50 ratio={r:.3f}; CPU={cpu['resident']['p50']:.3f} ms, GPU={gpu['resident']['p50']:.3f} ms")
if r>1.2:print('RED: reproducible GPU slowdown >20%');sys.exit(1)
print('GREEN: GPU slowdown >20% not reproduced')
