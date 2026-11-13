"""Processing entry point for the brain-signal controller."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from signals.signal import main as read_signals


def main():
    """Collect a short batch of sample readings and print them."""
    return read_signals()


if __name__ == "__main__":
    main()
