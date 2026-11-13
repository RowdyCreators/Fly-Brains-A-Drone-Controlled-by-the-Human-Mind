"""Real-life simulation: how well would the Option C mapping work?

Runs the actual SignalProcessor + DroneMapper (from
workspace/flybrains-processing) against three inputs, in order of
how close they are to real life:

  A) main's REAL EEG data: data/processed/eeg_motor_imagery_features.csv
     (3680 trials, LEFT_HAND / RIGHT_HAND, C3/Cz/C4 x alpha/beta/gamma
     band powers). Uses one electrode's alpha/beta/gamma as the 3
     channels, and also the mean across electrodes.
  B) A labelled synthetic session: calm, then deliberate focus
     (beta up), relax (alpha up), and gamma up, with realistic
     noise + slow drift -- the session we do NOT have from real life.
  C) main's eeg_pipeline SimSource: fake raw EEG at 125 Hz, cleaned
     and turned into band powers the way the real pipeline does it
     (2 s window, every 0.25 s), then fed to the processor.

Stdlib only for A and B; C imports numpy/scipy (the pipeline's own
dependencies) and the pipeline file from the main worktree.

Usage: python3 sim_real_life.py
Writes sim_real_life_report.txt next to this file.
"""

import csv
import math
import random
import statistics
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from processing.processor import SignalProcessor  # noqa: E402
from processing.drone_map import DroneMapper  # noqa: E402

MAIN = Path.home() / "workspace" / "flybrains-main"
EEG_CSV = MAIN / "data" / "processed" / "eeg_motor_imagery_features.csv"

OUT = []


def log(line=""):
    print(line)
    OUT.append(line)


def run_stream(samples, label_fn=None):
    """Feed (t, alpha, beta, gamma) samples through processor+mapper.

    Returns dict of measured outcomes.
    """
    p = SignalProcessor(window_size=10, calibrate_samples=5, debounce=3)
    m = DroneMapper()
    n = 0
    axes_hist = []
    active_counts = {"alpha": 0, "beta": 0, "gamma": 0, "none": 0}
    state_counts = {}
    saturated = {"pitch": 0, "roll": 0, "throttle": 0}
    roll_by_label = {}
    pitch_by_label = {}
    for i, s in enumerate(samples):
        r = p.add_sample(s)
        if r is None:
            continue
        n += 1
        axes = m.update(r)
        axes_hist.append(axes)
        active_counts[r["active_channel"]] += 1
        state_counts[r["state"]] = state_counts.get(r["state"], 0) + 1
        for ax in saturated:
            if abs(axes[ax]) >= 0.5:
                saturated[ax] += 1
        if label_fn is not None:
            lab = label_fn(i)
            if lab is not None:
                roll_by_label.setdefault(lab, []).append(axes["roll"])
                pitch_by_label.setdefault(lab, []).append(axes["pitch"])
    def mean(v):
        return statistics.mean(v) if v else 0.0
    return {
        "windows": n,
        "active_counts": active_counts,
        "state_counts": state_counts,
        "saturated": saturated,
        "mean_abs_pitch": mean([abs(a["pitch"]) for a in axes_hist]),
        "mean_abs_roll": mean([abs(a["roll"]) for a in axes_hist]),
        "mean_abs_throttle": mean([abs(a["throttle"]) for a in axes_hist]),
        "frac_moving": (sum(1 for a in axes_hist
                            if abs(a["pitch"]) > 0 or abs(a["roll"]) > 0
                            or abs(a["throttle"]) > 0) / n) if n else 0.0,
        "mean_roll_by_label": {k: mean(v) for k, v in roll_by_label.items()},
        "mean_pitch_by_label": {k: mean(v) for k, v in pitch_by_label.items()},
    }


def show(name, res):
    log(f"--- {name} ---")
    log(f"  windows: {res['windows']}")
    log(f"  time moving (any axis != 0): {res['frac_moving']:.1%}")
    log(f"  mean |velocity|: pitch {res['mean_abs_pitch']:.2f}  "
        f"roll {res['mean_abs_roll']:.2f}  throttle {res['mean_abs_throttle']:.2f}")
    log(f"  windows at full speed (0.5): {res['saturated']}")
    log(f"  active_channel: {res['active_counts']}")
    log(f"  states: {res['state_counts']}")
    if res["mean_roll_by_label"]:
        log(f"  mean roll by label: {res['mean_roll_by_label']}")
        log(f"  mean pitch by label: {res['mean_pitch_by_label']}")
    log("")


# ---------------------------------------------------------------- A
log("SIM A — main's real EEG motor-imagery data (3680 trials)")
log("Each trial = one sample. Calibration = first 5 trials only.")
rows = list(csv.DictReader(open(EEG_CSV)))
log(f"rows: {len(rows)}, labels: "
    f"{sum(1 for r in rows if r['label'] == 'LEFT_HAND')} LEFT, "
    f"{sum(1 for r in rows if r['label'] == 'RIGHT_HAND')} RIGHT")
for ch in ("alpha", "beta", "gamma"):
    v = [float(r[f"C3_{ch}"]) for r in rows]
    log(f"C3_{ch}: mean {statistics.mean(v):.2f}, max {max(v):.1f}, "
        f"fraction clipped at 3.3 V by the processor: "
        f"{sum(1 for x in v if x > 3.3) / len(v):.1%}")
log("")

for electrode in ("C3", "Cz", "C4"):
    samples = [(i * 0.1, float(r[f"{electrode}_alpha"]),
                float(r[f"{electrode}_beta"]), float(r[f"{electrode}_gamma"]))
               for i, r in enumerate(rows)]
    show(f"A-{electrode}: {electrode} alpha/beta/gamma as the 3 channels",
         run_stream(samples, label_fn=lambda i, rows=rows: rows[i]["label"]))

# Mean across the 3 electrodes, per band.
samples = []
for i, r in enumerate(rows):
    samples.append((i * 0.1,
                    statistics.mean([float(r[f"{e}_alpha"]) for e in ("C3", "Cz", "C4")]),
                    statistics.mean([float(r[f"{e}_beta"]) for e in ("C3", "Cz", "C4")]),
                    statistics.mean([float(r[f"{e}_gamma"]) for e in ("C3", "Cz", "C4")])))
show("A-mean: mean across C3/Cz/C4 per band", run_stream(samples))

# ---------------------------------------------------------------- B
log("SIM B — labelled synthetic session (what calibration SHOULD look like)")
log("30 calm, then 30 focus (beta +0.10), 30 relax (alpha +0.10),")
log("30 gamma-up (+0.10), 30 calm again. Noise 0.01, slow drift 0.02.")
rng = random.Random(7)
base = (0.12, -0.02, 0.33)
blocks = ([(0.12, -0.02, 0.33)] * 30
          + [(0.12, 0.08, 0.33)] * 30
          + [(0.22, -0.02, 0.33)] * 30
          + [(0.12, -0.02, 0.43)] * 30
          + [(0.12, -0.02, 0.33)] * 30)
samples = []
for i, (a, b, g) in enumerate(blocks):
    drift = 0.02 * math.sin(i / 50.0)
    samples.append((i * 0.1,
                    a + drift + rng.gauss(0, 0.01),
                    b + drift + rng.gauss(0, 0.01),
                    g + drift + rng.gauss(0, 0.01)))
# Per-block accuracy: which axis should be moving in each block?
p = SignalProcessor(window_size=10, calibrate_samples=5, debounce=3)
m = DroneMapper()
block_axes = []
for i, s in enumerate(samples):
    r = p.add_sample(s)
    if r is None:
        continue
    block_axes.append((i // 30, m.update(r)))
for b, want in [(1, "roll"), (2, "pitch"), (3, "throttle")]:
    vals = [a[want] for blk, a in block_axes if blk == b]
    correct = sum(1 for v in vals if (b == 1 and v > 0) or (b == 2 and v > 0)
                  or (b == 3 and v > 0))
    log(f"  block {b} ({want} should be > 0): {correct}/{len(vals)} windows "
        f"correct, mean {want} = {statistics.mean(vals):+.2f}")
# Latency: windows from block start to first correct step.
for b, want in [(1, "roll"), (2, "pitch"), (3, "throttle")]:
    first = next((i for i, (blk, a) in enumerate(block_axes)
                  if blk == b and a[want] > 0), None)
    start = next(i for i, (blk, a) in enumerate(block_axes) if blk == b)
    log(f"  block {b}: first correct step after "
        f"{(first - start) if first is not None else 'never'} results "
        f"(one result per sample at 10 Hz, debounce is 3 samples = 0.3 s)")
log("")

# ---------------------------------------------------------------- C
log("SIM C — main's eeg_pipeline SimSource, through its own band powers")
try:
    import numpy as np
    sys.path.insert(0, str(MAIN / "signals"))
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "eeg_pipeline", MAIN / "signals" / "eeg_pipeline.py")
    ep = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ep)

    src = ep.SimSource()
    cleaner = ep.StreamCleaner(ep.FS, ep.CHANNELS)
    # 30 s of fake EEG at 125 Hz, in 0.25 s chunks, 2 s band-power window.
    # Sleep 0.25 s per chunk so the SimSource's own clock (alpha flips
    # every 5 s) runs in real time: 120 updates in 30 s.
    import time as _time
    window = []
    samples = []
    t = 0.0
    for _ in range(120):
        chunk = np.array([src.read() for _ in range(31)]).T  # (3, ~31)
        clean = cleaner.process(chunk)
        window.extend(clean.T.tolist())
        window = window[-250:]
        if len(window) >= 250:
            abs_p, rel_p = ep.band_powers(np.array(window).T, ep.FS)
            # One "sample" per update: mean band power across the 3 pills.
            samples.append((t, float(abs_p[:, 0].mean()),
                            float(abs_p[:, 1].mean()),
                            float(abs_p[:, 2].mean())))
        t += 0.25
        _time.sleep(0.25)
    log(f"  band-power updates produced: {len(samples)}")
    if samples:
        for name, idx in (("alpha abs_p", 1), ("beta abs_p", 2), ("gamma abs_p", 3)):
            v = [s[idx] for s in samples]
            log(f"  {name}: mean {statistics.mean(v):.6f}, "
                f"max {max(v):.6f} (processor MIN_STD floor is 0.01 V)")
        show("C: pipeline abs band powers as alpha/beta/gamma",
             run_stream(samples))
except Exception as e:  # noqa: BLE001 - report, don't crash the whole sim
    log(f"  C failed: {type(e).__name__}: {e}")

(Path(__file__).resolve().parent / "sim_real_life_report.txt").write_text("\n".join(OUT))
log("Report written to sim_real_life_report.txt")
