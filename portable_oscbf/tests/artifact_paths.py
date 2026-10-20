import os
from pathlib import Path


TEST_OUTPUT = Path(
    os.environ.get(
        "OSCBF_TEST_OUTPUT_DIR",
        Path(__file__).resolve().parents[2] / ".scratch" / "portable-oscbf-tests",
    )
).resolve()
