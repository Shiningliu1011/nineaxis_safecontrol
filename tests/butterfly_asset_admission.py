import sys
from pathlib import Path

from work.ik_data_loader import load_repository_trajectory

root = Path(sys.argv[1])
trajectory = load_repository_trajectory(str(root / "data/nurbs/ik_input.mat"),
                                        str(root / "portable_oscbf/config/nineaxis.yaml"), 3.5)
trajectory.set_surface_normal_orientation([0., 1., 0.])
geometry = trajectory.path_geometry(require_continuous=True)
assert geometry.continuous_reference is not None
assert Path(sys.modules["work.ik_data_loader"].__file__).is_relative_to(root)
print(geometry.reference_identity, geometry.total_length_m)
