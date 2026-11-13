"""Simulated 3-channel EEG whose "mental state" you can change.

Lets every phase (train, assign, fly) run on a VM with no ADS1115.
The voltages look like the BioAmp Pill output: 1.65 V DC offset, 60 Hz hum,
background alpha/beta/gamma rhythms and noise. Each action changes the
alpha/beta/gamma mix the way the real thing roughly does:

    rest         baseline
    clench       jaw clench: broadband muscle noise, big beta/gamma jump
    focus        concentration: alpha drops, beta rises
    eyes_closed  alpha (8-13 Hz) gets much stronger
"""

import time

import numpy as np

MAINS_HZ = 60

# per-state multipliers for (alpha, beta, gamma) amplitude, plus extra muscle noise
STATES = {
    "rest":        {"alpha": 1.0, "beta": 1.0, "gamma": 1.0, "emg": 0.0},
    "clench":      {"alpha": 1.0, "beta": 1.5, "gamma": 2.0, "emg": 0.02},
    "focus":       {"alpha": 0.4, "beta": 2.5, "gamma": 1.2, "emg": 0.0},
    "eyes_closed": {"alpha": 5.0, "beta": 0.9, "gamma": 1.0, "emg": 0.0},
}
ACTIONS = [s for s in STATES if s != "rest"]


class SimBrain:
    """`state` can be changed at any time (from a cue or a key press)."""

    def __init__(self, n_channels=3, seed=None):
        self.n_channels = n_channels
        self.state = "rest"
        self.rng = np.random.default_rng(seed)
        self.phase = self.rng.uniform(0, 2 * np.pi, size=(n_channels, 3))
        self._t0 = time.perf_counter()

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        if value not in STATES:
            raise ValueError(f"unknown sim state {value!r}; options: {', '.join(STATES)}")
        self._state = value

    def block(self, ts):
        """Voltages for sample times `ts` (seconds). Returns shape (channels, len(ts))."""
        ts = np.asarray(ts, dtype=float)[None, :]
        s = STATES[self._state]
        ph = self.phase[:, :, None]
        n = (self.n_channels, ts.shape[1])
        v = (1.65
             + 0.004 * s["alpha"] * np.sin(2 * np.pi * 10 * ts + ph[:, 0])
             + 0.003 * s["beta"] * np.sin(2 * np.pi * 20 * ts + ph[:, 1])
             + 0.0015 * s["gamma"] * np.sin(2 * np.pi * 38 * ts + ph[:, 2])
             + 0.03 * np.sin(2 * np.pi * MAINS_HZ * ts)
             + 0.002 * self.rng.standard_normal(n))
        if s["emg"]:
            v = v + s["emg"] * self.rng.standard_normal(n)
        return v

    def read(self):
        """Live use (same interface as eeg_pipeline.ADSSource): one value per channel."""
        return list(self.block([time.perf_counter() - self._t0])[:, 0])
