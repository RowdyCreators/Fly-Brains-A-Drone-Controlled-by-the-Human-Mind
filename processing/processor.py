"""Process the 3-channel brain signals produced by ``signals.signal``.

This is the processing layer for the ``Brain_sim_Test`` branch. It sits
between ``FlyBrain`` (``signals/signal.py``) and whatever drives the drone,
and it works with the exact data ``FlyBrain`` produces:

* one sample is ``(timestamp, alpha, beta, gamma)`` -- the tuple returned
  by ``FlyBrain.receive_signals()`` and stored in ``FlyBrain.buffer``
* ``FlyBrain.listen()`` hands the same tuples to an ``on_sample`` callback
* ``signals/samples/samples.csv`` has the same columns:
  ``timestamp,alpha,beta,gamma``

What it does, in order, for every sample:

1. Validate -- drop non-finite values and clip to a plausible voltage
   range (``-1.0`` to ``3.3`` V). Dropped samples are counted, never
   silently turned into a number.
2. Smooth -- an exponential moving average per channel, to take the
   jitter out of the raw ADS1115 / simulator readings.
3. Baseline -- the first ``calibrate_samples`` samples set a per-channel
   mean and standard deviation. Later samples are reported as z-scores
   against that baseline, because the raw voltages are offsets, not
   absolute band powers: in the repo's own ``samples.csv`` beta sits
   around ``-0.02`` V and gamma around ``0.33`` V, so a fixed threshold
   on the raw value would be meaningless.
4. Window features -- mean, std, min, max and RMS per channel over the
   last ``window_size`` samples, plus each channel's share of the total
   absolute signal.
5. State -- ``focus`` / ``relax`` / ``active`` / ``neutral``, from the
   z-scores (see ``classify``). These thresholds are starting points
   only: they must be re-calibrated with labelled data from a real
   session before they drive a drone. See ``THRESHOLDS`` below.
6. Command -- a drone command with debouncing, so one noisy window
   cannot flip the drone back and forth. Commands use the exact
   lowercase names from ``hardwaretodrone.Drone.COMMANDS`` on ``main``
   (``hover`` / ``forward`` / ``up`` / ...), so a processed result can
   be passed straight to ``Drone.command(result["command"])``. The
   safe default, and the command for ``neutral`` and ``relax``, is
   ``hover``. Note the Drone's 0.5 s watchdog: the caller must send
   the current command every cycle, even when it has not changed --
   see ``drive_drone()`` below and ``processing/main.py``.
7. Live channel -- ``active_channel`` is the single channel that is
   currently activated, or ``"none"``: the channel with the highest
   z-score, provided it is at least ``active_threshold`` (default
   1.0 z) and beats the next channel by at least ``active_margin``
   (default 0.25 z). Debounced like the command, so the live output
   does not flicker between channels on noise. This is the output to
   use when you want the program to say, live, just ``alpha`` /
   ``beta`` / ``gamma`` / ``none`` -- see ``--mode channel`` in
   ``processing/main.py``. Alongside it, ``channel_active`` gives
   each channel independently as +1 / 0 / -1 (above baseline /
   dead zone / below baseline, debounced per channel) -- that is
   what the Option C drone mapper in ``processing/drone_map.py``
   reads, one channel = one signed drone axis.

Stdlib only -- no numpy / scipy -- so it runs on the Pi as-is, next to
``signals/signal.py``.
"""

import csv
import math
from collections import deque

CHANNELS = ("alpha", "beta", "gamma")

# Starting-point thresholds, in z-score units against the calibration
# baseline. They are NOT tuned: the simulator and the 20-row samples.csv
# in this repo have no labelled focus/relax sessions to tune them on.
# Calibrate them by recording a session where the wearer is deliberately
# focusing, then one where they are relaxed, and picking thresholds that
# separate the two.
DEFAULT_FOCUS_THRESHOLD = 1.0   # z_beta - z_alpha
DEFAULT_RELAX_THRESHOLD = 1.0   # z_alpha - z_beta
DEFAULT_GAMMA_THRESHOLD = 1.0   # z_gamma, for the "active" state
DEFAULT_ACTIVE_THRESHOLD = 1.0  # highest channel z, for active_channel
DEFAULT_ACTIVE_MARGIN = 0.25    # lead over the next channel, in z

# Plausible ADS1115 / BioAmp voltage range for these channels.
MIN_VOLTAGE = -1.0
MAX_VOLTAGE = 3.3

# Z-scores divide by the baseline std; a flat baseline (std 0, e.g.
# identical simulator samples) would make every z-score 0, so floor
# the std at the simulator's own noise level (0.01 V in brain_sim.py).
MIN_STD = 0.01

STATE_TO_COMMAND = {
    "focus": "forward",
    "active": "up",
    "relax": "hover",
    "neutral": "hover",
}


def drive_drone(drone, result):
    """Send a processed result to a ``hardwaretodrone.Drone``.

    Call this on every processed window, even when the command has
    not changed: the Drone's watchdog returns the sticks to hover
    if ``command()`` is not called for 0.5 s. ``drone`` is duck-typed
    (only ``.command(name)`` is used) so this also works with a fake
    in tests, and so this module does not import ``hardwaretodrone``
    -- that package lives on ``main``, not on ``Brain_sim_Test``.
    """
    if result is None:
        return None
    drone.command(result["command"])
    return result["command"]


def _mean(values):
    return sum(values) / len(values) if values else 0.0


def _std(values, mean):
    if len(values) < 2:
        return 0.0
    return math.sqrt(sum((v - mean) ** 2 for v in values) / (len(values) - 1))


def _rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else 0.0


class SignalProcessor:
    """Turn a stream of (timestamp, alpha, beta, gamma) into states/commands."""

    def __init__(self, window_size=10, smooth=0.3, calibrate_samples=5,
                 focus_threshold=DEFAULT_FOCUS_THRESHOLD,
                 relax_threshold=DEFAULT_RELAX_THRESHOLD,
                 gamma_threshold=DEFAULT_GAMMA_THRESHOLD,
                 active_threshold=DEFAULT_ACTIVE_THRESHOLD,
                 active_margin=DEFAULT_ACTIVE_MARGIN,
                 debounce=3):
        self.window_size = window_size
        self.smooth = smooth  # EMA weight for the newest sample, 0..1
        self.calibrate_samples = calibrate_samples
        self.focus_threshold = focus_threshold
        self.relax_threshold = relax_threshold
        self.gamma_threshold = gamma_threshold
        self.active_threshold = active_threshold
        self.active_margin = active_margin
        self.debounce = debounce

        self.window = deque(maxlen=window_size)  # smoothed (t, a, b, g)
        self._smoothed = None  # (alpha, beta, gamma), EMA state
        self._calibration = []  # raw samples collected for the baseline
        self.baseline = None  # {channel: (mean, std)} once calibrated
        self.dropped = 0
        self.n_seen = 0

        self.state = "neutral"
        self.command = "hover"
        self._pending_state = "neutral"
        self._pending_count = 0
        self.active_channel = "none"
        self._pending_channel = "none"
        self._pending_channel_count = 0
        # Per-channel activation, debounced independently (Option C drone
        # mapping): +1 = channel above baseline, -1 = below, 0 = dead zone.
        self.channel_active = {ch: 0 for ch in CHANNELS}
        self._pending_channel_active = {ch: 0 for ch in CHANNELS}
        self._pending_channel_active_count = {ch: 0 for ch in CHANNELS}

    # -- calibration ---------------------------------------------------
    def calibrate(self, samples):
        """Set the baseline from an iterable of (t, a, b, g) samples."""
        rows = [self._clean(s) for s in samples]
        rows = [r for r in rows if r is not None]
        if not rows:
            raise ValueError("calibrate: no valid samples")
        self.baseline = {}
        for i, name in enumerate(CHANNELS):
            values = [r[1 + i] for r in rows]
            mean = _mean(values)
            self.baseline[name] = (mean, max(_std(values, mean), MIN_STD))
        return self.baseline

    # -- per-sample ----------------------------------------------------
    def _clean(self, sample):
        """Normalise a sample to (timestamp, alpha, beta, gamma) floats.

        Accepts a 4-tuple, a 3-tuple (timestamp is then None), or a dict
        with alpha/beta/gamma keys. Returns None for an invalid sample.
        """
        try:
            if isinstance(sample, dict):
                timestamp = sample.get("timestamp")
                values = [sample[ch] for ch in CHANNELS]
            elif len(sample) == 4:
                timestamp, *values = sample
            elif len(sample) == 3:
                timestamp, values = None, list(sample)
            else:
                return None
            values = [float(v) for v in values]
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(v) for v in values):
            return None
        values = [min(MAX_VOLTAGE, max(MIN_VOLTAGE, v)) for v in values]
        timestamp = float(timestamp) if timestamp is not None else None
        return (timestamp, values[0], values[1], values[2])

    def add_sample(self, sample):
        """Feed one sample through the pipeline.

        Returns a dict with the smoothed values, window features,
        z-scores, state and command -- or None if the sample was
        dropped, or if we are still collecting the calibration baseline.
        """
        clean = self._clean(sample)
        if clean is None:
            self.dropped += 1
            return None
        self.n_seen += 1

        if self.baseline is None:
            self._calibration.append(clean)
            if len(self._calibration) >= self.calibrate_samples:
                self.calibrate(self._calibration)
                self._calibration = []
            # Calibration samples set the baseline only; they are not
            # processed as data, so the first N calls return None.
            return None

        _, alpha, beta, gamma = clean
        if self._smoothed is None:
            self._smoothed = (alpha, beta, gamma)
        else:
            w = self.smooth
            self._smoothed = tuple(
                w * v + (1 - w) * old
                for v, old in zip((alpha, beta, gamma), self._smoothed)
            )
        self.window.append((clean[0], *self._smoothed))
        return self._features(clean)

    def _features(self, raw):
        window = list(self.window)
        n = len(window)
        feats = {"timestamp": raw[0], "raw": raw[1:], "n": n,
                 "smoothed": self._smoothed, "dropped": self.dropped}

        # Per-channel window statistics, on the smoothed values.
        total_abs = 0.0
        for i, name in enumerate(CHANNELS):
            values = [row[1 + i] for row in window]
            mean = _mean(values)
            feats[f"{name}_mean"] = mean
            feats[f"{name}_std"] = _std(values, mean)
            feats[f"{name}_min"] = min(values)
            feats[f"{name}_max"] = max(values)
            feats[f"{name}_rms"] = _rms(values)
            total_abs += abs(mean)
        for name in CHANNELS:
            feats[f"{name}_share"] = (
                abs(feats[f"{name}_mean"]) / total_abs if total_abs else 0.0
            )

        # Z-scores against the calibration baseline.
        z = {}
        for i, name in enumerate(CHANNELS):
            mean, std = self.baseline[name]
            z[name] = (self._smoothed[i] - mean) / std if std else 0.0
        feats["z"] = z

        # Scores: beta up + alpha down = focus, the reverse = relax,
        # gamma up = active. All in z-score units.
        feats["focus_score"] = z["beta"] - z["alpha"]
        feats["relax_score"] = z["alpha"] - z["beta"]
        feats["activity_score"] = z["gamma"]

        feats["state"] = self.classify(feats)
        feats["command"] = self._debounced_command(feats["state"])
        feats["active_channel"] = self._debounced_channel(
            self._raw_active_channel(feats["z"]))
        feats["channel_active"] = self._debounced_channel_active(feats["z"])
        return feats

    def _raw_channel_activation(self, z):
        """Per-channel sign: +1 if z >= threshold, -1 if z <= -threshold, else 0."""
        out = {}
        for ch in CHANNELS:
            if z[ch] >= self.active_threshold:
                out[ch] = 1
            elif z[ch] <= -self.active_threshold:
                out[ch] = -1
            else:
                out[ch] = 0
        return out

    def _debounced_channel_active(self, z):
        """Only change a channel's activation after `debounce` windows agree.

        Each channel is debounced independently, so alpha can be +1
        while beta is -1 and gamma is 0 in the same window -- that is
        what the Option C drone mapper needs (one channel = one axis).
        """
        raw = self._raw_channel_activation(z)
        for ch in CHANNELS:
            if raw[ch] == self._pending_channel_active[ch]:
                self._pending_channel_active_count[ch] += 1
            else:
                self._pending_channel_active[ch] = raw[ch]
                self._pending_channel_active_count[ch] = 1
            if self._pending_channel_active_count[ch] >= self.debounce:
                self.channel_active[ch] = raw[ch]
        return dict(self.channel_active)

    # -- live channel ---------------------------------------------------
    def _raw_active_channel(self, z):
        """Channel with the highest z-score, or "none" if none stands out.

        A channel counts as activated only if its z-score reaches
        ``active_threshold`` and it leads the next channel by at
        least ``active_margin`` -- otherwise two channels rising
        together would flip the output back and forth.
        """
        ranked = sorted(CHANNELS, key=lambda ch: z[ch], reverse=True)
        top, second = ranked[0], ranked[1]
        if z[top] >= self.active_threshold and z[top] - z[second] >= self.active_margin:
            return top
        return "none"

    def _debounced_channel(self, channel):
        """Only change active_channel after `debounce` windows agree."""
        if channel == self._pending_channel:
            self._pending_channel_count += 1
        else:
            self._pending_channel = channel
            self._pending_channel_count = 1
        if self._pending_channel_count >= self.debounce:
            self.active_channel = channel
        return self.active_channel

    # -- state / command ------------------------------------------------
    def classify(self, feats):
        """Map window scores to focus / relax / active / neutral."""
        if (feats["activity_score"] >= self.gamma_threshold
                and feats["focus_score"] >= 0.5):
            return "active"
        if feats["focus_score"] >= self.focus_threshold:
            return "focus"
        if feats["relax_score"] >= self.relax_threshold:
            return "relax"
        return "neutral"

    def _debounced_command(self, state):
        """Only change command after `debounce` windows in a row agree."""
        if state == self._pending_state:
            self._pending_count += 1
        else:
            self._pending_state = state
            self._pending_count = 1
        if self._pending_count >= self.debounce and state != self.state:
            self.state = state
            self.command = STATE_TO_COMMAND[state]
        return self.command

    # -- batch inputs ----------------------------------------------------
    def process_buffer(self, buffer):
        """Process every (timestamp, alpha, beta, gamma) in a FlyBrain.buffer."""
        return [r for r in (self.add_sample(s) for s in buffer) if r is not None]

    def process_csv(self, path, output_path=None):
        """Process a samples.csv file; optionally write processed rows out."""
        results = []
        with open(path, newline="") as fh:
            for row in csv.DictReader(fh):
                result = self.add_sample((
                    float(row["timestamp"]),
                    float(row["alpha"]),
                    float(row["beta"]),
                    float(row["gamma"]),
                ))
                if result is not None:
                    results.append(result)
        if output_path:
            self.write_csv(results, output_path)
        return results

    @staticmethod
    def write_csv(results, output_path):
        """Write processed rows (one per window) to a CSV file."""
        fields = ["timestamp", "n", "alpha_mean", "beta_mean", "gamma_mean",
                  "alpha_std", "beta_std", "gamma_std",
                  "alpha_share", "beta_share", "gamma_share",
                  "focus_score", "relax_score", "activity_score",
                  "active_channel", "state", "command"]
        with open(output_path, "w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=fields)
            writer.writeheader()
            for r in results:
                writer.writerow({f: r.get(f, "") for f in fields})
        return output_path
