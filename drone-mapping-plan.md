# Drone mapping plan — alpha / beta / gamma → 3D movement

**Status: PLAN ONLY — for Chan's review. No code written yet.**
**Branch context:** `Brain_sim_Test`, processor in `processing/processor.py`, drone in `Hardware/hardwaretodrone/drone.py` on `main`.

## 1. What we have

### Inputs — only 3 signals
From `SignalProcessor`, per window (after calibration, EMA smoothing, debounce = 3 windows):
- z-scores: `z_alpha`, `z_beta`, `z_gamma` (against the calibration baseline, std floored at 0.01 V)
- `active_channel`: single winner (`alpha` / `beta` / `gamma` / `none`) — highest z, ≥ 1.0z and ≥ 0.25z ahead of the next channel
- `state`: `focus` / `relax` / `active` / `neutral`
- `focus_score = z_beta − z_alpha`, `relax_score = z_alpha − z_beta`, `activity_score = z_gamma`

Important: the winner-take-all `active_channel` **loses information** — it can only say one channel at a time. For 3D movement we need all three channels at once, so the mapper must read the three z-scores directly, not just `active_channel`.

### Drone interface — `hardwaretodrone.Drone` (main branch)
- Axes: `pitch` (forward + / backward −), `roll` (right + / left −), `yaw` (turn right + / turn left −), `throttle` (up + / down −). Each −1.0 … +1.0.
- `command(name, strength=30)`: discrete, **one axis at a time** — every other axis returns to centre. Names: `hover`, `forward`, `backward`, `left`, `right`, `turn_left`, `turn_right`, `up`, `down`.
- `set(pitch, roll, yaw, throttle)`: continuous, −1..+1 per axis, **axes left out keep their current target**. This is the only call that can move in XY *and* up/down at the same time.
- Watchdog 0.5 s: if no `command()` / `set()` arrives, all sticks return to centre (hover). So the mapper must call `set()` **every window, even when nothing changed**.
- Slew limit 4.0/s: sticks ease toward the target, 0 → full in 0.25 s.
- Starts in failsafe (hover). Takeoff / landing are by hand with the remote's physical button — this mapper never takes off or lands.
- `Drone(dry_run=True)` uses a FakeDAC — all testing happens in dry-run first.

## 2. The core problem

3 signals → 4 axes (pitch, roll, yaw, throttle) = 8 directions (+/− each).

- 3 binary channels give only 7 non-empty combinations. That is not enough for 8 independent directions, and it is not enough if each channel is used as one direction.
- The z-scores are **continuous**, not binary, so each channel can give a sign (+/−) as well as a size. That is what makes 3D control possible at all: each channel becomes one signed axis.

So: **one channel = one axis, sign = rising vs falling relative to baseline.**

## 3. Recommended mapping — Option C (one channel, one axis, signed)

| Signal | Axis | +1 direction | −1 direction | Dead zone |
|---|---|---|---|---|
| alpha (z_alpha) | pitch (X, forward/back) | z_alpha ≥ +1.0 → forward | z_alpha ≤ −1.0 → backward | −1.0 < z < +1.0 → hold |
| beta (z_beta) | roll (Y, left/right) | z_beta ≥ +1.0 → right | z_beta ≤ −1.0 → left | −1.0 < z < +1.0 → hold |
| gamma (z_gamma) | throttle (Z, up/down) | z_gamma ≥ +1.0 → up | z_gamma ≤ −1.0 → down | −1.0 < z < +1.0 → hold |
| — | yaw (turn) | **not mapped — stays 0** | — | — |

- **XY plane movement** = alpha + beta together (e.g. alpha up + beta up = forward-right diagonal).
- **Up/down movement** = gamma alone.
- **All three together** = a diagonal in 3D, via one `drone.set(pitch, roll, throttle)` call per window.
- Yaw is deliberately left at 0 in v1: there is no 4th signal to drive it, and turning while also moving in 3 axes would be unlearnable. It can be added later (see §6).

### How the +1 / −1 increments work (stepwise velocity, not a jump)

The mapper keeps a velocity state, one value per axis, each clamped to [−1, +1]:

```
velocity[axis] starts at 0 (hover)
each debounced window:
  if channel is +active (z ≥ +1.0):  velocity += STEP   (+1 increment)
  if channel is −active (z ≤ −1.0):  velocity -= STEP   (−1 increment)
  if channel is in the dead zone:     velocity decays toward 0 by STEP
  velocity = clamp(velocity, −MAX_SPEED, +MAX_SPEED)
  drone.set(pitch=velocity[alpha], roll=velocity[beta], throttle=velocity[gamma], yaw=0)
```

- `STEP = 0.25` (starting value, tunable): 4 confirmed windows to go from hover to full stick on one axis. One window = one +1/−1 increment.
- `MAX_SPEED = 0.5` for first real flights (starting value, tunable): the drone never gets more than half stick from brain control, even if the user holds a channel active.
- Decay on dead zone means: stop activating a channel and that axis eases back to 0 (hover on that axis) instead of sticking at speed. `none` on all three → all axes decay → hover.
- Debounce (reuse the processor's 3-window debounce, per channel, independently): a channel must be +active or −active for 3 windows in a row before its first increment counts. This is what stops noise from nudging the drone.
- Every window calls `drone.set(...)` with the current velocity, even when it is unchanged — that is what keeps the 0.5 s watchdog happy.

### Worked example (STEP 0.25, MAX 0.5)
| Windows | z_alpha | z_beta | z_gamma | pitch | roll | throttle | Drone does |
|---|---|---|---|---|---|---|---|
| 1–3 | +1.4 | 0.0 | 0.0 | +0.25 | 0 | 0 | forward, slow |
| 4–6 | +1.4 | 0.0 | 0.0 | +0.50 | 0 | 0 | forward, capped |
| 7–9 | +1.4 | +1.2 | 0.0 | +0.50 | +0.25 | 0 | forward-right |
| 10–12 | 0.0 | 0.0 | +1.5 | +0.25 | 0.0 | +0.25 | easing forward, starting up |
| 13–15 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | hover |

## 4. Alternatives considered (for review)

### Option A — discrete `command()` steps
Each debounced activation calls `drone.command("forward")` / `"backward"` / etc. (strength 30).
- Pro: uses the existing `drive_drone()` helper, simplest code.
- Con: `command()` centres every other axis, so **XY + up/down together is impossible** — the drone can only do one direction at a time, and it snaps back to centre between commands. Rejected as the main design, but kept as a fallback if `set()` is unavailable.

### Option B — mode switching
Gamma (or a held pattern) switches mode: XY mode, then Z mode, then back. In XY mode alpha/beta = forward/back/left/right, in Z mode alpha = up, beta = down.
- Pro: each mode is simple to learn.
- Con: the user must remember which mode they are in, a wrong mode means a wrong direction (e.g. intending "up" and getting "forward"), and switching costs time. More state to get wrong, worse for safety. Rejected for v1, possible later if yaw needs a mode.

### Option D — continuous proportional (no steps)
`pitch = clamp(z_alpha / 2)`, `roll = clamp(z_beta / 2)`, `throttle = clamp(z_gamma / 2)`, sent straight to `set()`.
- Pro: smoothest, fastest response.
- Con: no dead-zone stepping — every wobble in the z-scores moves the drone immediately, and a drifting baseline (which we already see in the simulator and in `samples.csv`) would make the drone drift on its own. The stepwise version (Option C) is the same mapping with a dead zone, debounce and speed cap added — that is why C is recommended over D.

## 5. What would change in code (only after approval)

New file, no changes to `FlyBrain` or to the drone library:
- `processing/drone_map.py` — `DroneMapper` class: holds the velocity state, takes a processor result dict (`z` scores), returns `{pitch, roll, yaw, throttle}` and calls `drone.set(...)` every window. Constants at the top: `STEP = 0.25`, `MAX_SPEED = 0.5`, `POS_THRESHOLD = 1.0`, `NEG_THRESHOLD = −1.0`, `DEBOUNCE = 3`.
- `processing/processor.py` — small addition only: expose per-channel activation (`alpha_active` ∈ `+1` / `0` / `−1`, same for beta/gamma) alongside the existing `active_channel`, so the mapper does not recompute thresholds. `active_channel` / `state` / `command` stay as they are.
- `processing/main.py` — new flag `--mode map` (alongside existing `--mode command` / `--mode channel`): prints one line per window like `t=… pitch=+0.25 roll=0.00 throttle=+0.25 yaw=0.00`, and with `--drone-dry-run` calls the mapper against a dry-run `Drone`.
- `tests/test_drone_map.py` — new, stdlib only, FakeDrone with a `set()` recorder. Tests: alpha up → pitch steps +0.25 then +0.50 (capped); alpha down → pitch −0.25; beta up + gamma up together → roll + throttle in one `set()`; dead zone decays to 0; `none` on all channels → hover; `set()` called every window (watchdog); velocity never exceeds MAX_SPEED.

## 6. Open questions for Chan (decide before I build it)

1. **Channel → axis assignment:** is alpha = forward/back, beta = left/right, gamma = up/down right? Or would you rather have beta = forward/back (beta = focus feels more like "go") and alpha = left/right? The physiology is a guess either way until we calibrate — pick the one that feels most natural to you.
2. **STEP and MAX_SPEED:** 0.25 per window and 0.5 max stick to start — too slow, too fast, or fine for a first test?
3. **Yaw:** leave it at 0 for v1, or do you want a way to turn (e.g. a mode, or a double-activation pattern)? If yes, which pattern?
4. **Down:** gamma falling below baseline = down. Is that learnable, or would you rather have "down" be something else (e.g. relax / alpha high = descend)? Note the current `relax` state already maps to hover, not down.
5. **Sign of beta:** beta up = right, beta down = left — or the reverse? Same question for alpha (up = forward) and gamma (up = up). Confirm the signs feel right before I wire them in.

## 7. Safety — non-negotiable, in v1

- Dry-run (`Drone(dry_run=True)`, FakeDAC) only until the mapping is verified in tests and in `--mode map` output on simulated data. No real DAC, no real drone, until you say so.
- `MAX_SPEED = 0.5` cap, dead zone ±1.0z, 3-window debounce — all three stay in v1.
- Any `none` on all channels, any invalid/dropped sample, or any exception → velocity decays to 0 and `set(0, 0, 0, 0)` (hover) is sent that window.
- Takeoff / landing stay manual on the remote's physical button. The mapper never takes off, lands, or arms anything.
- Thresholds are still untuned (1.0z is a starting point, from unlabelled simulator / `samples.csv` drift). A labelled session is needed before this flies for real — the same caveat as the processor.
- `close()` on exit centres the sticks, as the drone library already does.
