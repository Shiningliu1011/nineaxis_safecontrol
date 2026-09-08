# pytest configuration for robot_safecontrol_moveit tests.
import sys
from pathlib import Path

# Allow imports from the source tree without installing the package.
_src = Path(__file__).resolve().parents[1] / "src"
if str(_src) not in sys.path:
    sys.path.insert(0, str(_src))
