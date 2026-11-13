"""
Fly Brains - EEG signal pipeline
BioAmp EXG Pill (x3) -> ADS1115 -> Raspberry Pi over I2C

On the Pi:        python3 eeg_pipeline.py
On a laptop:      python3 eeg_pipeline.py --sim     (fake EEG, no hardware)

Install:  pip install numpy scipy adafruit-circuitpython-ads1x15
          (on the Pi, do this inside a venv)

What it does, in order:
  1. A background thread reads all 3 channels at a fixed rate (FS).
  2. Each new chunk is cleaned: DC offset removed, 60 Hz hum notched out.
     The filters keep their state between chunks, so this works on a live stream.
  3. Every UPDATE_SEC, we take the last WINDOW_SEC of clean data and measure
     how much power is in alpha, beta and gamma on each channel.
  4. Those numbers go to decide_command(), which is the part you write.
"""

import argparse
import threading
import time
from collections import deque

import numpy as np
from scipy import signal

# ---------------- settings ----------------

FS = 125            # target samples per second PER CHANNEL.
                    # 3 channels share the ADS1115's single converter, and Python
                    # adds I2C overhead, so don't expect much more than this.
                    # # 125 also keeps 60 Hz below Nyquist (62.5) so the notch can catch it.
CHANNELS = 3        # one EXG Pill per channel, outputs wired to A0, A1, A2
WINDOW_SEC = 2.0    # longer = steadier numbers, but slower reaction
UPDATE_SEC = 0.25   # how often band powers are recomputed
MAINS_HZ = 60       # US wall power

BANDS = {
    "alpha": (8, 13),
    "beta":  (13, 30),
    "gamma": (30, 45),  # capped: above this we're near the notch and Nyquist
}


# ---------------- data sources ----------------

class ADSSource:
    """Reads real voltages from the ADS1115."""

    def __init__(self):
        import board
        import busio
        import adafruit_ads1x15.ads1115 as ADS
        from adafruit_ads1x15.analog_in import AnalogIn
        # If ADS.P0 errors, your library version renamed the pin constants.
        # Check the README for the version you installed.

        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS.ADS1115(i2c)
        ads.gain = 1          # +/-4.096 V input range. Power the Pills from 3.3 V
                              # so their output never goes past the ADS1115's supply.
        ads.data_rate = 860   # fastest setting. Shared by all channels.
        pins = [ADS.P0, ADS.P1, ADS.P2][:CHANNELS]
        self.chans = [AnalogIn(ads, p) for p in pins]

    def read(self):
        # Channels are read one after another, so they're a couple ms apart.
        return [c.voltage for c in self.chans]


class SimSource:
    """Fake EEG for testing without hardware.
    Alpha switches strong/weak every 5 s to mimic eyes closed/open.
    Includes a DC offset and 60 Hz hum, so you can see the cleaner work."""

    def __init__(self):
        self.t0 = time.perf_counter()
        self.rng = np.random.default_rng()

    def read(self):
        t = time.perf_counter() - self.t0
        alpha_amp = 0.02 if int(t // 5) % 2 else 0.004
        out = []
        for ch in range(CHANNELS):
            v = (1.65                                          # DC offset, like the Pill
                 + alpha_amp * np.sin(2 * np.pi * 10 * t + ch)  # alpha
                 + 0.006 * np.sin(2 * np.pi * 20 * t)           # beta
                 + 0.003 * np.sin(2 * np.pi * 38 * t)           # gamma
                 + 0.03 * np.sin(2 * np.pi * MAINS_HZ * t)      # wall power hum
                 + 0.004 * self.rng.standard_normal())          # noise
            out.append(v)
        return out


# ---------------- acquisition thread ----------------

class Acquirer(threading.Thread):
    """Samples on a fixed schedule instead of sleep(period), which drifts.
    Each sample is stored with a timestamp so you can check the real rate."""

    def __init__(self, source, fs):
        super().__init__(daemon=True)
        self.source = source
        self.period = 1.0 / fs
        self.buf = deque(maxlen=fs * 10)   # (timestamp, [v0, v1, v2])
        self.lock = threading.Lock()
        self.missed = 0                    # times we fell behind schedule
        self.running = True

    def run(self):
        next_t = time.perf_counter()
        while self.running:
            vals = self.source.read()
            ts = time.perf_counter()
            with self.lock:
                self.buf.append((ts, vals))

            next_t += self.period
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                # Too slow. Resync instead of firing a burst to catch up,
                # since bursts would mess up the even spacing filters expect.
                self.missed += 1
                next_t = time.perf_counter()

    def drain(self):
        """Take everything collected since the last call."""
        with self.lock:
            items = list(self.buf)
            self.buf.clear()
        return items


# ---------------- cleaning ----------------

class StreamCleaner:
    """High-pass at 0.5 Hz (kills DC offset and slow drift) + 60 Hz notch.
    Uses sosfilt with saved state (zi), so chunk after chunk acts like
    one continuous signal. sosfiltfilt can't do that: it needs the whole
    recording up front."""

    def __init__(self, fs, n_ch):
        if MAINS_HZ >= fs / 2:
            raise ValueError(
                f"FS={fs} puts {MAINS_HZ} Hz above Nyquist. It will alias to "
                f"{abs(fs - MAINS_HZ)} Hz and the notch can't remove it.")
        hp = signal.butter(2, 0.5, btype="highpass", fs=fs, output="sos")
        b, a = signal.iirnotch(MAINS_HZ, Q=30, fs=fs)
        self.sos = np.vstack([hp, signal.tf2sos(b, a)])
        self.zi_base = signal.sosfilt_zi(self.sos)   # (sections, 2)
        self.zi = None

    def process(self, x):
        """x shape: (channels, samples). Returns cleaned data, same shape."""
        if self.zi is None:
            # Start the filter as if it had already been seeing the first value.
            # Avoids a big startup spike from the DC offset.
            self.zi = self.zi_base[:, None, :] * x[:, 0][None, :, None]
        y, self.zi = signal.sosfilt(self.sos, x, axis=-1, zi=self.zi)
        return y


# ---------------- band power ----------------

def band_powers(window, fs):
    """window shape: (channels, samples) of cleaned data.

    Returns two arrays, shape (channels, 3), columns = alpha, beta, gamma:
      abs_p: absolute power (V^2). Depends on electrode contact, gain, etc.
      rel_p: fraction of total 1-45 Hz power. Usually more stable to use.
    """
    f, psd = signal.welch(window, fs=fs, nperseg=int(fs), axis=-1)
    df = f[1] - f[0]

    abs_p = np.zeros((window.shape[0], len(BANDS)))
    for i, (lo, hi) in enumerate(BANDS.values()):
        mask = (f >= lo) & (f < hi)
        abs_p[:, i] = psd[:, mask].sum(axis=-1) * df

    total = psd[:, (f >= 1) & (f < 45)].sum(axis=-1) * df
    rel_p = abs_p / np.maximum(total, 1e-20)[:, None]
    return abs_p, rel_p


# ---------------- your part ----------------

def decide_command(abs_p, rel_p):
    """Turn band power into a drone command.

    rel_p[ch, 0] = alpha share on channel ch
    rel_p[ch, 1] = beta share
    rel_p[ch, 2] = gamma share

    Suggested first step: record ~30 s eyes open and ~30 s eyes closed,
    look at how alpha changes, then pick a threshold between the two.
    """
    return None


# ---------------- main loop ----------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", action="store_true", help="use fake EEG data")
    args = ap.parse_args()

    source = SimSource() if args.sim else ADSSource()
    cleaner = StreamCleaner(FS, CHANNELS)
    win_len = int(WINDOW_SEC * FS)
    window = deque(maxlen=win_len)   # rolling window of cleaned samples

    acq = Acquirer(source, FS)
    acq.start()
    print(f"Sampling at {FS} Hz target. Ctrl+C to stop.", flush=True)

    try:
        while True:
            time.sleep(UPDATE_SEC)
            items = acq.drain()
            if len(items) < 2:
                continue

            ts = np.array([t for t, _ in items])
            raw = np.array([v for _, v in items]).T   # (channels, samples)
            actual_fs = (len(ts) - 1) / (ts[-1] - ts[0])

            clean = cleaner.process(raw)
            window.extend(clean.T)                    # one row per sample
            if len(window) < win_len:
                print("filling window...", flush=True)
                continue

            abs_p, rel_p = band_powers(np.array(window).T, FS)

            row = "  ".join(
                f"ch{c}: a={rel_p[c,0]:.0%} b={rel_p[c,1]:.0%} g={rel_p[c,2]:.0%}"
                for c in range(CHANNELS))
            print(f"{row}  | rate={actual_fs:.0f}Hz missed={acq.missed}", flush=True)

            cmd = decide_command(abs_p, rel_p)
            if cmd is not None:
                pass   # send cmd to the drone controller here

    except KeyboardInterrupt:
        acq.running = False
        print("\nstopped")


if __name__ == "__main__":
    main()
