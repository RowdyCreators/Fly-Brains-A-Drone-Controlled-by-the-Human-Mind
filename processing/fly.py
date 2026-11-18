import argparse
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT, ROOT / "Hardware"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import numpy as np  # noqa: E402

from processing.live_bridge import iter_eeg_pipeline  # noqa: E402
from signals import eeg_pipeline as pipe  # noqa: E402

BANDS = ("alpha", "beta", "gamma")
ACTION_HINT = {
    "alpha": "CLOSE your eyes",
    "beta": "concentrate hard (mental math)",
    "gamma": "CLENCH your jaw",
}
MAX_STRENGTH = 50
MIN_GAP = 0.15 # the band's share must rise at least this much during the action
STALE_SEC = 1.0 # no new brain reading for this long -> hover
KEEPALIVE_SEC = 0.1


class CalibrationError(Exception):
    pass


def band_score(rel_p, band):
    """Mean share of power in `band` over all connected channels (rel_p: channels x 3)."""
    return float(np.mean(np.asarray(rel_p)[:, BANDS.index(band)]))


def calibrate(relax_scores, action_scores, min_gap=MIN_GAP):
    """Returns (low, high) thresholds, centered between the two medians.

    The band must be clearly HIGHER during the action than at rest, or we refuse:
    a flat or reversed result means bad electrode contact or the wrong action."""
    if len(relax_scores) < 4 or len(action_scores) < 4:
        raise CalibrationError("not enough readings collected")
    lo, hi = float(np.median(relax_scores)), float(np.median(action_scores))
    gap = hi - lo
    if gap < min_gap:
        raise CalibrationError(
            f"band share only moved {lo:.0%} -> {hi:.0%} (need a rise of at least "
            f"{min_gap:.0%}). Check electrode contact, or try a different action/band.")
    mid = (lo + hi) / 2
    return mid - 0.15 * gap, mid + 0.15 * gap


class Decider:

    def __init__(self, low, high, confirm=2):
        self.low, self.high, self.confirm = low, high, confirm
        self.command = "hover"
        self._cand, self._n = None, 0

    def update(self, score):
        if score >= self.high:
            want = "forward"
        elif score <= self.low:
            want = "turn_right"
        else:
            want = self.command if self.command != "hover" else None
        if want is None or want == self.command:
            self._cand, self._n = None, 0
        elif want == self._cand:
            self._n += 1
        else:
            self._cand, self._n = want, 1
        if self._cand is not None and self._n >= self.confirm:
            self.command, self._cand, self._n = self._cand, None, 0
        return self.command


class Keepalive:
    """Re-sends the current command every KEEPALIVE_SEC so the drone's watchdog stays fed,
    but falls back to hover if set() hasn't been called for `stale` seconds."""

    def __init__(self, drone, strength, stale=STALE_SEC, period=KEEPALIVE_SEC):
        self.drone, self.strength = drone, strength
        self.stale, self.period = stale, period
        self._cmd, self._stamp = "hover", time.monotonic()
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="fly-keepalive", daemon=True)
        self._thread.start()

    def set(self, command):
        with self._lock:
            self._cmd, self._stamp = command, time.monotonic()

    def current(self):
        with self._lock:
            return "hover" if time.monotonic() - self._stamp > self.stale else self._cmd

    def _loop(self):
        while self._running:
            self.drone.command(self.current(), strength=self.strength)
            time.sleep(self.period)

    def stop(self):
        self._running = False
        self._thread.join(timeout=1.0)
        self.drone.hover()


def collect(frames, seconds, settle, band):
    """Band scores from the stream for `seconds`, ignoring the first `settle` seconds
    (the 2 s analysis window still contains the previous phase)."""
    t0, scores = time.monotonic(), []
    for _abs_p, rel_p in frames:
        t = time.monotonic() - t0
        if t >= settle:
            scores.append(band_score(rel_p, band))
        if t >= seconds:
            break
    return scores


def fly(frames, decider, keepalive, band, show=print, seconds=None):
    """Brain loop: one decision per frame. Returns the commands issued, in order."""
    t0, issued = time.monotonic(), []
    try:
        for _abs_p, rel_p in frames:
            score = band_score(rel_p, band)
            cmd = decider.update(score)
            keepalive.set(cmd)
            issued.append(cmd)
            show(f"{time.monotonic() - t0:6.1f}s  {band} {score:4.0%}  "
                 f"(turn<{decider.low:.0%} fwd>{decider.high:.0%})  -> {cmd}")
            if seconds and time.monotonic() - t0 >= seconds:
                break
    except KeyboardInterrupt:
        pass
    return issued


class ScriptedSim(pipe.SimSource):
    #Simulated EEG (one value per connected channel) where one band can be switched strong/weak (`active`), standing in for 'eyes closed' / 'jaw clench'. Includes the DC offset and 60 Hz hum.

    BASE = {"alpha": 0.004, "beta": 0.006, "gamma": 0.003}
    ACTIVE = {"alpha": 0.02, "beta": 0.03, "gamma": 0.02}

    def __init__(self, band):
        super().__init__()
        self.band, self.active = band, False

    def read(self):
        t = time.perf_counter() - self.t0
        amp = dict(self.BASE)
        if self.active:
            amp[self.band] = self.ACTIVE[self.band]
        return [1.65
                + amp["alpha"] * np.sin(2 * np.pi * 10 * t + ch)
                + amp["beta"] * np.sin(2 * np.pi * 20 * t)
                + amp["gamma"] * np.sin(2 * np.pi * 38 * t)
                + 0.03 * np.sin(2 * np.pi * pipe.MAINS_HZ * t)
                + 0.004 * self.rng.standard_normal()
                for ch in range(pipe.CHANNELS)]

    def auto_toggle(self, period):
        """Flip the action on and off by itself (demo mode for --sim)."""
        def run():
            while True:
                time.sleep(period)
                self.active = not self.active
        threading.Thread(target=run, daemon=True).start()


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="simulated EEG (always a fake DAC)")
    ap.add_argument("--real", action="store_true", help="move the REAL drone (needs real EEG)")
    ap.add_argument("--signal", choices=BANDS, default="alpha",
                    help="band whose share of power drives the drone (default alpha = eyes closed)")
    ap.add_argument("--channels", type=int, default=1, choices=(1, 2, 3),
                    help="BioAmp Pills wired to ADS1115 A0..A(n-1) (default 1); unwired inputs float and add noise")
    ap.add_argument("--cal-sec", type=float, default=8.0, help="seconds per calibration phase")
    ap.add_argument("--strength", type=int, default=30, help=f"stick strength 1-{MAX_STRENGTH}")
    ap.add_argument("--seconds", type=float, help="stop flying after this many seconds")
    ap.add_argument("--yes", action="store_true", help="skip the typed FLY confirmation")
    args = ap.parse_args(argv)
    if not 1 <= args.strength <= MAX_STRENGTH:
        ap.error(f"--strength must be 1-{MAX_STRENGTH}")
    if args.sim and args.real:
        ap.error("--sim and --real can't be combined")
    return args


def confirm_real(args):
    if not args.real: return
    print("REAL DRONE. Remote on, drone armed and taken off by hand, area clear.\n"
          "Ctrl+C = hover and stop. To cut power, use the remote.")
    if args.yes: return
    try:
        confirmed = sys.stdin.isatty() and input("Type FLY to continue: ").strip() == "FLY"
    except EOFError:
        confirmed = False
    if not confirmed:
        sys.exit("not confirmed; nothing sent to the drone")


def main(argv=None):
    args = parse_args(argv)
    confirm_real(args)

    from hardwaretodrone import Drone

    band = args.signal
    pipe.CHANNELS = args.channels   # ADSSource, the filters and the sim all read this when used
    if args.sim: source = ScriptedSim(band)
    else: source = pipe.ADSSource()

    def cue(msg, active):
        if isinstance(source, ScriptedSim): source.active = active
        print(f"\a>> {msg}", flush=True)

    settle = pipe.WINDOW_SEC + 1.0
    with Drone(dry_run=not args.real, strength=args.strength) as drone:
        keepalive = Keepalive(drone, args.strength)
        try:
            frames = iter_eeg_pipeline(source)
            cue("RELAX, eyes open, jaw loose", False)
            relax = collect(frames, args.cal_sec, settle, band)
            cue(f"now {ACTION_HINT[band]} and hold it", True)
            action = collect(frames, args.cal_sec, settle, band)
            low, high = calibrate(relax, action)
            print(f"calibrated: relax {np.median(relax):.0%}, action {np.median(action):.0%}"
                  f" -> turn below {low:.0%}, forward above {high:.0%}")
            if isinstance(source, ScriptedSim):
                source.auto_toggle(8.0)
            cue(f"flying: {ACTION_HINT[band]} = forward, relax = turn right", False)
            fly(frames, Decider(low, high), keepalive, band, seconds=args.seconds)
        except CalibrationError as e:
            print(f"CALIBRATION FAILED, not flying: {e}")
            return 1
        finally:
            keepalive.stop()
    print("stopped, sticks centered")
    return 0


if __name__ == "__main__":
    sys.exit(main())
