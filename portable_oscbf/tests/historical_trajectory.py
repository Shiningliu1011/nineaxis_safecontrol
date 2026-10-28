from hashlib import sha256
import json
from pathlib import Path

import numpy as np

from work.ik_data_loader import IKTrajectoryData


def load_historical_trajectory(*, feedrate_scale=1.0):
    root = Path(__file__).resolve().parents[2]
    source = root / "data/nurbs/ik_input.mat"
    reference = json.loads((root / "tests/data/trajectory_reference_4c98a64.json").read_text())
    assert sha256(source.read_bytes()).hexdigest() == reference["source_mat_sha256"]
    trajectory = IKTrajectoryData(str(source), np.asarray(reference["trajectory_transform"]),
                                  feedrate_scale=feedrate_scale)
    trajectory.set_orientation_mode("fixed", fixed_R=np.asarray(reference["fixed_rotation"]))
    return trajectory
