"""Turn raw EEG samples into feature frames, live or offline.

A frame is one feature vector: log10 band power for every (channel, band),
ordered ch0_alpha, ch0_beta, ch0_gamma, ch1_alpha, ...

Sampling and cleaning reuse signals/eeg_pipeline.py (Acquirer thread,
StreamCleaner, FS, CHANNELS, BANDS). Everything stays in memory, in one process.
"""

import time
from collections import deque
from dataclasses import dataclass

import numpy as np
from scipy.signal import welch

from signals import eeg_pipeline as pipe

FS = pipe.FS
CHANNELS = pipe.CHANNELS
BANDS = pipe.BANDS

WINDOW_SEC = 0.5   # short, so a clench/action shows up quickly
UPDATE_SEC = 0.1   # one frame every 100 ms (well inside the drone's 0.5 s watchdog)
SETTLE_SEC = 2.0   # ignore frames while the filters warm up


def feature_names(n_channels=CHANNELS, bands=BANDS):
    return [f"ch{c}_{b}" for c in range(n_channels) for b in bands]


def band_log_powers(window, fs, bands=BANDS):
    """window: (channels, samples) of cleaned data -> (channels * bands,) log10 power."""
    n = window.shape[-1]
    f, psd = welch(window, fs=fs, nperseg=n, axis=-1)
    df = f[1] - f[0]
    cols = []
    for lo, hi in bands.values():
        mask = (f >= lo) & (f < hi)
        cols.append(psd[:, mask].sum(axis=-1) * df)
    return np.log10(np.stack(cols, axis=1) + 1e-20).ravel()


@dataclass
class Frame:
    t: float               # seconds (stream clock)
    features: np.ndarray   # (channels * bands,)


class FeatureExtractor:
    """Cleans incoming chunks and keeps a rolling window of the latest samples."""

    def __init__(self, fs=FS, n_channels=CHANNELS, window_sec=WINDOW_SEC):
        self.fs = fs
        self.cleaner = pipe.StreamCleaner(fs, n_channels)
        self.win_len = int(round(window_sec * fs))
        self.window = deque(maxlen=self.win_len)

    def push(self, raw):
        """raw: (channels, samples)."""
        if raw.shape[1] == 0:
            return
        self.window.extend(self.cleaner.process(raw).T)

    @property
    def ready(self):
        return len(self.window) == self.win_len

    def features(self):
        return band_log_powers(np.array(self.window).T, self.fs)


class LiveStream:
    """Real-time frames from a source with .read() -> [v_ch0, v_ch1, v_ch2]
    (eeg_pipeline.ADSSource on the Pi, or sim.SimBrain)."""

    def __init__(self, source, fs=FS, n_channels=CHANNELS,
                 window_sec=WINDOW_SEC, update_sec=UPDATE_SEC, settle_sec=SETTLE_SEC):
        self.extractor = FeatureExtractor(fs, n_channels, window_sec)
        self.acq = pipe.Acquirer(source, fs)
        self.update_sec = update_sec
        self.settle_sec = settle_sec
        self.actual_fs = None

    @property
    def missed(self):
        return self.acq.missed

    def frames(self):
        if not self.acq.is_alive():
            self.acq.start()
        start = time.perf_counter()
        next_t = start
        while self.acq.running:
            next_t += self.update_sec
            time.sleep(max(0.0, next_t - time.perf_counter()))
            items = self.acq.drain()
            if not items:
                continue
            if len(items) > 1:
                span = items[-1][0] - items[0][0]
                if span > 0:
                    self.actual_fs = (len(items) - 1) / span
            self.extractor.push(np.array([v for _, v in items]).T)
            now = time.perf_counter()
            if self.extractor.ready and now - start >= self.settle_sec:
                yield Frame(now, self.extractor.features())

    def close(self):
        self.acq.running = False

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class OfflineStream:
    """Same frames as LiveStream but generated as fast as possible from a
    SimBrain (for tests and quick dry runs). The stream clock starts at 0."""

    def __init__(self, brain, fs=FS, n_channels=CHANNELS,
                 window_sec=WINDOW_SEC, update_sec=UPDATE_SEC, settle_sec=SETTLE_SEC):
        self.brain = brain
        self.fs = fs
        self.extractor = FeatureExtractor(fs, n_channels, window_sec)
        self.update_sec = update_sec
        self.settle_sec = settle_sec
        self.actual_fs = fs
        self.missed = 0

    def frames(self):
        per = max(1, int(round(self.update_sec * self.fs)))
        i = 0
        while True:
            ts = (i + np.arange(per)) / self.fs
            i += per
            self.extractor.push(self.brain.block(ts))
            t = i / self.fs
            if self.extractor.ready and t >= self.settle_sec:
                yield Frame(t, self.extractor.features())

    def close(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def collect(frames, seconds, skip_sec=0.0):
    """Pull frames for `seconds` of stream time; drop the first `skip_sec`.
    Returns a (n, features) array."""
    out, start = [], None
    for fr in frames:
        if start is None:
            start = fr.t
        elapsed = fr.t - start
        if elapsed >= skip_sec:
            out.append(fr.features)
        if elapsed >= seconds:
            break
    return np.array(out)
