"""Processing entry point for the brain-signal controller.

Same as `python -m processing ...` (train / assign / fly); see processing/cli.py.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from processing.cli import main  # noqa: E402

if __name__ == "__main__":
    main()
