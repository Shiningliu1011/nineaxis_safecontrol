import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from robot_safecontrol_moveit.pipe_assets.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
