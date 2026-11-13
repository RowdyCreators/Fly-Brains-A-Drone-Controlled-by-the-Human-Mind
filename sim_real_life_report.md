# Real-life simulation — will the Option C mapping work?

Run 2026-11-13, in `workspace/flybrains-processing/sim_real_life.py`.
Full numbers in `sim_real_life_report.txt` next to it.

## Short answer

The code works. The system, as it stands, would **not** work in real
life: on real EEG data it flies the drone almost constantly, mostly at
full speed, in directions that have nothing to do with what the wearer
intends. The mapper logic is sound (Sim B: 29–30/30 correct on a clean
labelled signal), but the input it gets in real life is not that signal.

## What main actually is now (head 316a9e2)

- New file `signals/eeg_pipeline.py` is the real pipeline: 3 BioAmp
  pills → ADS1115 at 125 Hz/channel, DC-offset removal + 60 Hz notch,
  2 s window, band powers recomputed every 0.25 s (alpha 8–13,
  beta 13–30, gamma 30–45 Hz, Welch). `decide_command()` is still a
  stub returning `None` — the classifier→drone link is still not
  wired up, on main or anywhere else.
- `data/processed/eeg_motor_imagery_features.csv`: 3680 trials,
  LEFT_HAND / RIGHT_HAND, C3/Cz/C4 × alpha/beta/gamma band powers.
- **Main disagrees with itself about what a channel is:**
  `Brain_sim_Test` `signals/signal.py` (and our processor) treat
  A0/A1/A2 as alpha/beta/gamma voltages; `eeg_pipeline.py` treats
  each of 3 channels as an electrode and computes all 3 bands per
  electrode (9 numbers); `Hardware/hardware.yaml` wires only ONE
  BioAmp output to A0. These cannot all be true at once.
- `processing/main.py` and `tests/test_signal.py` on main are still
  the broken `FlyBrian` versions.

## Sim A — main's real EEG data (3680 motor-imagery trials)

Fed each electrode's alpha/beta/gamma as the 3 channels, calibrated
on the first 5 trials only:

| Input | Moving | Mean \|velocity\| | At full speed (0.5) |
|---|---|---|---|
| C3 | 95.4% of windows | pitch .15, roll .37, throttle .42 | roll 2659/3675, throttle 3009/3675 |
| Cz | 98.4% | pitch .19, roll .38, throttle .47 | throttle 3395/3675 |
| C4 | 96.2% | pitch .42, roll .28, throttle .41 | pitch 3067/3675 |
| mean of C3/Cz/C4 | 97.7% | pitch .36, roll .41, throttle .43 | all axes ~3000/3675 |

- **The label test:** mean roll for LEFT_HAND vs RIGHT_HAND differs
  by < 0.001 (C3: −0.072 vs −0.072; C4: −0.139 vs −0.139). Same for
  pitch. The mapping carries **zero information** about the actual
  motor-imagery label. A drone driven by this would turn left/right
  at random relative to what the wearer imagined.
- **Clipping:** band powers go up to 20–42, mean ~2.4. The processor
  clips at 3.3 V, so 24% of C3 alpha values are clipped flat. The
  processor was built for raw voltages (~0.1 V), not band powers.
- **Why it moves so much:** trials are independent rows, not a time
  stream, so every trial is a new "jump" past the 1.0z threshold.
  But the same effect happens on a real stream: any sustained drift
  past baseline saturates an axis at 0.5 and holds it there.

## Sim B — labelled synthetic session (the session we don't have)

30 calm, then 30 focus (beta +0.10), 30 relax (alpha +0.10),
30 gamma-up (+0.10), 30 calm, noise 0.01, slow drift 0.02:

- Focus block: roll > 0 in 29/30 windows, mean +0.47
- Relax block: pitch > 0 in 30/30, mean +0.50
- Gamma block: throttle > 0 in 30/30, mean +0.50
- First correct step 0–1 results after a block starts
  (one result per sample at 10 Hz, debounce 3 samples = 0.3 s)

So: **if** the input is a clean, deliberate, labelled signal, the
mapper does exactly what it should, fast. The failure in Sim A is
the input, not the mapper.

## Sim C — main's real pipeline, SimSource, 30 s in real time

Raw fake EEG (125 Hz, alpha flips strong/weak every 5 s) → cleaner
→ 2 s Welch band powers every 0.25 s → processor + mapper:

- Absolute band powers are ~0.00001 V². The processor's
  `MIN_STD = 0.01 V` floor (built for raw voltages) makes every
  z-score ≈ 0.001. **The drone never moves: 0.0% of 107 windows,
  all states neutral, all channels none.**
- Feeding **relative** band power (`rel_p`, fraction of 1–45 Hz
  power, 0.02–0.19) instead fixes the deadness — but then it moves
  98.1% of the time, pitch saturating at −0.50, because the sim's
  alpha flips every 5 s and the baseline was set during a strong-
  alpha period. Movement again, but not intentional movement.

## End-to-end with the real drone library (dry-run)

`--mode map --csv … --drone-dry-run` with main's real
`hardwaretodrone.Drone(dry_run=True)` works: `set(pitch=0.25,
throttle=0.5)` → status shows target reached, DAC codes
`[2559, 2048, 2048, 3065]` (pitch and throttle moved, roll/yaw at
centre 2048). The output side is fine; the input side is the problem.

## What would need to change before this flies for real

1. **Decide what a channel is, as a team.** One electrode with
   bands computed in software (pipeline model), or three pills each
   wired to one band (Brain_sim_Test model), or the single-A0 wiring
   in `hardware.yaml`? Right now the three halves of the repo
   assume three different answers, and the mapper can only be
   right for one of them.
2. **Fix the input scale.** The processor must take band powers
   (relative power, `rel_p`, is the stable one), not raw volts:
   drop the −1.0…3.3 V clip and the 0.01 V std floor for that path,
   or add a separate band-power input mode. As-is, real pipeline
   output either clips (motor-imagery CSV) or is dead (SimSource).
3. **Get a labelled session from a real person.** The thresholds
   (1.0z, step 0.25, max 0.5) are guesses. Record ~30 s eyes-open
   vs eyes-closed (the pipeline's own suggested first step) and
   focus vs relax, then pick thresholds that separate them — and
   re-run Sims A/B on that data before any real flight.
4. **Pick a command signal that actually exists in the data.**
   LEFT/RIGHT motor imagery does not show up in alpha/beta/gamma
   band powers the way the mapper needs (Sim A: zero label signal).
   Either train a real classifier on the motor-imagery features
   (C3/C4 beta, etc.) and map *its* output to the drone, or use a
   signal that does separate — e.g. alpha up/down (eyes closed/
   open) for one axis only, and prove it separates first.
5. **Safety before flight:** keep the 0.5 max stick, keep the
   dead zone and debounce, add a physical kill switch / a way to
   force hover from the remote, and test the whole chain on the
   bench (Pi + DAC + remote, props off) with the real pipeline
   before the drone ever leaves the ground. Takeoff/landing stay
   manual, as the drone library already requires.
