# Processing pipeline documentation

This document covers the `processing/` package: what it does, what each file is for, and what changed when it was added (branch `processing`).

## Purpose

`processing/` connects the EEG side of the project to the drone. It runs on the Raspberry Pi as **one program**: EEG samples, features, classification and drone commands all stay in memory. Nothing is written to or read from a CSV, socket or other file while flying.

There are three phases, run one after the other for each person:

1. **Train:** record a resting baseline of the person's alpha, beta and gamma signals and save it as their profile.
2. **Assign:** record a few chosen actions (for example jaw clench, focus, eyes closed), learn what each looks like compared to the baseline, and assign each one to a control role.
3. **Fly:** recognise those actions live and turn them into drone commands through a moded control scheme.

## How data flows while flying

```
3 BioAmp Pills -> ADS1115 -> Acquirer thread (125 Hz per channel)
  -> filter (DC offset and 60 Hz hum removed)
  -> every 0.1 s: alpha/beta/gamma power per channel over the last 0.5 s (9 numbers)
  -> compare to the person's baseline
  -> classifier: which recorded action (or rest) is this?
  -> event detector: did it start or stop, and how long has it been held?
  -> mode controller: which drone command does that mean?
  -> drone.command(...)   (sent on every reading, even when it is "hover")
```

## Control scheme

| Role | Required? | Short (tap) | Held |
|---|---|---|---|
| `primary` | Yes | Next mode | Move in the current mode for as long as it is held |
| `prev` | No | Previous mode | Nothing |
| `lock` | No | Nothing | Lock or unlock all controls |

- Default mode order: `forward → turn_left → turn_right → up → down → speed_up → speed_down`, then back to the start.
- Sideways movement is done by turning and then going forward, which keeps the mode list short.
- In `speed_up` / `speed_down`, each hold changes the drone's strength by one step (default 10, kept between 10 and 100).
- By default a tap is anything shorter than 1.0 s and a hold is anything longer than 1.2 s.
- If a `lock` role is assigned, the program starts locked.
- The mode list, timings and speed step are stored in the profile, so they can be changed without editing code.

## Safety behaviour

- Nothing detected, or anything the classifier is unsure about, means **hover**.
- Readings that look unlike anything recorded during assignment are treated as rest, so a stray movement or bad electrode contact is not turned into a command.
- `drone.command()` is sent every 0.1 s. If the program freezes or crashes, the drone's existing 0.5 s watchdog returns it to hover.
- Ctrl+C sends hover and the drone library centers every stick on exit.
- Takeoff and landing are still done by hand with the remote's button.

## How to run it

```bash
# On a VM or laptop (simulated EEG, fake DAC)
python -m processing train  --profile me --sim --seconds 20
python -m processing assign --profile me --sim --primary clench --lock eyes_closed --trials 4
python -m processing fly    --profile me --sim

# On the Pi
python -m processing train  --profile me
python -m processing assign --profile me           # asks which action to use for each role
python -m processing fly    --profile me --dry-run # real EEG, drone does not move
python -m processing fly    --profile me           # real EEG, real drone
```

- `--sim` uses simulated EEG and a fake DAC. In `train` and `assign` the simulator acts out each prompt by itself. In `fly` you drive it with keys: `c` clench, `f` focus, `e` eyes closed (press again or space to return to rest), `q` quit. Key control needs a real terminal window.
- `assign` options: `--trials` (default 8), `--rest-sec` (default 4), `--action-sec` (default 3).
- `python processing/main.py ...` works the same as `python -m processing ...`.
- Requirements: numpy and scipy, plus `pip install -e Hardware` for the drone library. No new dependencies were added.

## Files

### New files

| File | Purpose |
|---|---|
| `processing/__init__.py` | Makes `processing` a package and adds the repo root and `Hardware/` to the import path, so `signals` and `hardwaretodrone` import without extra setup. |
| `processing/__main__.py` | Lets the package run as `python -m processing`. |
| `processing/cli.py` | The command line: the `train`, `assign` and `fly` subcommands, their options, and the on-screen prompts and status line. |
| `processing/stream.py` | Turns raw samples into readings. Reuses `Acquirer`, `StreamCleaner`, `FS`, `CHANNELS` and `BANDS` from `signals/eeg_pipeline.py`. `LiveStream` runs in real time (hardware or simulator); `OfflineStream` generates simulated readings faster than real time for tests. Each reading is the log10 power of alpha, beta and gamma for each channel. |
| `processing/training.py` | Phase 1. Records the resting baseline (average and spread of each of the 9 numbers) into the profile. Recording a new baseline clears any previously learned actions, since they were measured against the old one. |
| `processing/assignment.py` | Phase 2. Prompts rest and each chosen action in shuffled trials, skips the first moment after each prompt (reaction time), trains the classifier and tests it by holding out one trial at a time. Prints an accuracy report and warns when anything is below 80%, especially rest being mistaken for an action. |
| `processing/classifier.py` | `Classifier` matches each reading to the closest learned pattern (rest or one of the actions) and rejects readings unlike anything seen in training. `EventDetector` ignores single-reading blips (a pattern must appear in 2 readings in a row to start, and be missing from 2 in a row to stop) and measures how long a pattern is held. Uses numpy only. |
| `processing/modes.py` | `ModeController`: the control scheme above as plain logic with no hardware or timing, so it is easy to test and change. |
| `processing/control.py` | Phase 3. `Pilot` runs one reading through the classifier, event detector and mode controller, then sends the command to the drone. `fly()` loops over live readings until Ctrl+C. |
| `processing/userprofile.py` | `Profile`: everything saved for one person (baseline, learned patterns, role choices, accuracy, mode list, timings, and the recording settings). Saved as `profiles/NAME.json`. Refuses to be used if the sample rate, channel count, bands or window size have changed since it was recorded. Named `userprofile.py` rather than `profile.py` so it does not clash with Python's built-in `profile` module. |
| `processing/sim.py` | `SimBrain`: simulated 3-channel EEG with a rest state and three actions (`clench`, `focus`, `eyes_closed`), including the DC offset and 60 Hz hum of the real hardware. |
| `processing/keys.py` | Reads single key presses (Linux, macOS, Windows) to switch the simulator's state while flying. |
| `tests/test_processing.py` | 15 tests: mode logic, tap/hold detection, classifier, profile save/load, and a full train → assign → fly run on the simulator against a fake drone. |
| `documentation.md` | This file. |

### Modified files

| File | Change |
|---|---|
| `processing/main.py` | Previously a stub that ran `signals/signal.py`. Now launches the same command line as `python -m processing`. |
| `.gitignore` | Added `profiles/`, since profiles contain personal EEG data. |
| `CLAUDE.md` | Describes the new pipeline and simulator commands; added numpy and scipy to the VM install command. |

### Unchanged

`signals/eeg_pipeline.py`, `signals/signal.py`, `signals/brain_simulator.py`, `processing/convert.py`, and everything in `Hardware/`.

## Known limitations and follow-ups

- **Only tested on simulated EEG.** The simulator gives near-perfect accuracy; real signals will be noisier. Use the accuracy report from `assign` to choose actions that work for each person.
- **Timings are starting guesses.** The 1.0 s tap and 1.2 s hold limits may need tuning on real data (`tap_max_sec` and `hold_min_sec` in the profile).
- **`Hardware/hardware.yaml` still lists one EEG sensor on A0.** The team uses 3 (A0 to A2). The EEG code does not read this file, but it should be updated to stay the single source of truth.
- **`CLAUDE.md` still says `brain_simulator` does not exist.** It does now; that note is out of date.
- **The training CSV is not used yet.** `data/processed/eeg_motor_imagery_features.csv` was recorded at 250 Hz on different electrode positions, so it does not match the live setup directly.

## Running the tests

```bash
python -m unittest discover tests
```

All 25 tests pass (10 existing, 15 new). No hardware is needed.
