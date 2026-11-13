"""A user's profile: baseline, learned input patterns, role mapping, control scheme.

Saved as JSON in profiles/<name>.json (git-ignored: it's personal EEG data).
"""

import json
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

from . import ROOT

PROFILE_DIR = ROOT / "profiles"

# Roles an input pattern can be assigned to.
#   primary  short = next mode, hold = go          (required)
#   prev     short = previous mode                 (optional)
#   lock     hold = lock / unlock all controls     (optional)
ROLES = ("primary", "prev", "lock")

DEFAULT_MODES = ["forward", "turn_left", "turn_right", "up", "down", "speed_up", "speed_down"]


@dataclass
class Profile:
    name: str
    created: float = field(default_factory=time.time)

    # stream settings the profile was recorded with (must match at fly time)
    fs: int = 0
    channels: int = 0
    bands: List[str] = field(default_factory=list)
    window_sec: float = 0.0
    update_sec: float = 0.0

    # phase 1: baseline (log10 band power) per feature
    baseline_mean: List[float] = field(default_factory=list)
    baseline_std: List[float] = field(default_factory=list)

    # phase 2: classifier over z-scored features (see classifier.Classifier)
    classifier: Optional[dict] = None
    accuracy: Dict[str, float] = field(default_factory=dict)  # per class, cross-validated
    roles: Dict[str, str] = field(default_factory=dict)       # role -> pattern label

    # control scheme
    modes: List[str] = field(default_factory=lambda: list(DEFAULT_MODES))
    tap_max_sec: float = 1.0    # pattern shorter than this = tap
    hold_min_sec: float = 1.2   # pattern longer than this = hold
    speed_step: int = 10
    start_strength: int = 30

    @property
    def has_baseline(self):
        return bool(self.baseline_mean)

    @property
    def has_patterns(self):
        return self.classifier is not None and "primary" in self.roles

    @staticmethod
    def path_for(name):
        return PROFILE_DIR / f"{name}.json"

    def save(self, path=None):
        path = Path(path) if path else self.path_for(self.name)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(asdict(self), indent=2))
        return path

    @classmethod
    def load(cls, name_or_path):
        p = Path(name_or_path)
        if p.suffix != ".json":
            p = cls.path_for(name_or_path)
        data = json.loads(p.read_text())
        known = cls.__dataclass_fields__
        return cls(**{k: v for k, v in data.items() if k in known})

    def check_stream(self, fs, channels, bands, window_sec, update_sec):
        """Raise if the live stream doesn't match how this profile was recorded."""
        want = (self.fs, self.channels, list(self.bands), self.window_sec, self.update_sec)
        got = (fs, channels, list(bands), window_sec, update_sec)
        if want != got:
            raise ValueError(
                f"profile {self.name!r} was recorded with fs/channels/bands/window/update "
                f"{want} but the stream is {got}; re-run `train`")
