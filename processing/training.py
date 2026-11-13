"""Phase 1: record a resting baseline and store it in the profile.

The user sits relaxed, eyes open, looking at the drone, for `seconds`.
We store the mean and spread of log10 band power for every (channel, band);
later phases z-score everything against this, so patterns are measured as
"how different from your normal" rather than raw volts.
"""

import numpy as np

from . import stream as st

STD_FLOOR = 0.05   # log10 units; stops a near-constant feature from dominating


def record_baseline(frames, profile, seconds=60.0, cue=print, *,
                    fs=st.FS, channels=st.CHANNELS, bands=st.BANDS,
                    window_sec=st.WINDOW_SEC, update_sec=st.UPDATE_SEC):
    """Fill profile's baseline from `seconds` of frames. Returns the feature array."""
    cue("rest", f"Relax, eyes open, breathe normally. Recording {seconds:.0f} s...")
    X = st.collect(frames, seconds)
    if len(X) < 10:
        raise RuntimeError(f"only got {len(X)} frames; is the EEG source running?")

    profile.fs, profile.channels = fs, channels
    profile.bands, profile.window_sec, profile.update_sec = list(bands), window_sec, update_sec
    profile.baseline_mean = X.mean(axis=0).tolist()
    profile.baseline_std = np.maximum(X.std(axis=0), STD_FLOOR).tolist()
    # a new baseline invalidates patterns learned against the old one
    profile.classifier, profile.accuracy, profile.roles = None, {}, {}
    return X


def describe_baseline(profile):
    names = st.feature_names(profile.channels)
    rows = [f"  {n:<12} {m:7.2f} ± {s:.2f}"
            for n, m, s in zip(names, profile.baseline_mean, profile.baseline_std)]
    return "baseline (log10 V^2):\n" + "\n".join(rows)
