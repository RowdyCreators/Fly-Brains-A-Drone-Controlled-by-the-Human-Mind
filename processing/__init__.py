"""Brain-signal processing: train a baseline, assign input patterns, fly.

    python -m processing train  --profile NAME [--sim]
    python -m processing assign --profile NAME [--sim]
    python -m processing fly    --profile NAME [--sim]
"""

import sys
from pathlib import Path

# Make `signals` and `hardwaretodrone` importable without installing anything,
# matching how the rest of the repo is run (python from the repo root).
ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "Hardware"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))
