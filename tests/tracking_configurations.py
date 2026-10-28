import json
from pathlib import Path

import numpy as np


_DOCUMENT = json.loads((Path(__file__).parent / "data/tracking_configurations_j1_origin85mm.json").read_text())
START = np.asarray(_DOCUMENT["start_q_si"])
DISTANT = np.asarray(_DOCUMENT["distant_q_si"])
