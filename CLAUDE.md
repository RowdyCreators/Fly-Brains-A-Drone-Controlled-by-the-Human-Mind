# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Brain-controlled drone: EEG (BioAmp EXG Pill → ADS1115 ADC) on a Raspberry Pi 4 is classified into movement commands, which are sent to a Holy Stone HS210 Pro by driving its *stock remote's* joystick pots with an MCP4728 DAC. The drone is never talked to directly.

## Layout / architecture

Two halves joined by `processing/` (one process on the Pi; data stays in memory, no files/sockets between stages).

- **`processing/`** — the train → assign → fly pipeline (`python -m processing {train,assign,fly} --profile NAME [--sim]`). `stream.py` reuses `signals/eeg_pipeline.py` (Acquirer thread, StreamCleaner) to emit a frame every 0.1 s = log10 alpha/beta/gamma power per channel over a 0.5 s window (3 channels → 9 features). `training.py` stores a resting baseline; `assignment.py` cues rest/action trials, fits `classifier.Classifier` (numpy diagonal-LDA on baseline z-scores, rejects out-of-distribution frames as rest) and reports leave-one-trial-out accuracy; `control.py` `Pilot` runs classifier → `EventDetector` (debounce, durations) → `modes.ModeController` (pure logic: primary tap = next mode / hold = go, optional `prev` and `lock` roles) → `drone.command()` every frame. Profiles are JSON in `profiles/` (git-ignored). `sim.py` `SimBrain` fakes EEG states (rest/clench/focus/eyes_closed); `OfflineStream` runs it faster than real time for tests. The profile module is `userprofile.py` (not `profile.py`, which would shadow the stdlib when `processing/main.py` is run as a script).

- **`Hardware/hardwaretodrone/`** — the installable control library (the mature part). `Drone` (`drone.py`) runs a background output thread at `rate_hz` that slews each axis toward its target and writes all four DAC channels. `axes.py` maps -1..+1 → volts → 12-bit code using per-axis calibration; `dac.py` has the `MCP4728` driver and a `FakeDAC`; `config.py` loads `Hardware/hardware.yaml` (single source of truth for pins, I2C addresses, and per-axis `min_v/center_v/max_v` calibration — don't hardcode these in code).
- **`signals/signal.py`, `processing/main.py`, `tests/`** — EEG-side stubs. Note naming drift: `signal.py` defines `FlyBrain` but `processing/main.py` imports `FlyBrian` and `tests/test_signal.py` expects `FlyBrian`/`read_signals()`/`sample_rate`; verify against the file before relying on either name. `signals/signal.py` also shadows the stdlib `signal` module name if `signals/` is put on `sys.path`. `signals/signal.py.my-changes` is a stray backup.
- **`Hardware/hardwaretesting/`** — bench tools, not part of the package: `dronevirtualcontroller.py` (pygame on-screen controller), `pot4testing.py`, `testDACsignals.c` (sweeps DAC channels for multimeter checks).
- **`examples/basicreading.py`** — live ADS1115 voltage bar (needs Pi + blinka).

## Commands

```bash
pip install -e Hardware          # installs hardwaretodrone (smbus2, PyYAML) editable
python -m unittest discover tests   # run tests (unittest; single: python -m unittest tests.test_signal)
```

No linter/build config exists. `Drone(dry_run=True)` uses `FakeDAC`, so the library runs on a dev machine without I2C. Root `requirements.txt` is the Pi-side set (adafruit ADS1x15 + blinka); `Hardware/requirements.txt` adds smbus2, PyYAML, pygame (for the virtual controller).

## Running without a Pi (team VMs)

Teammates run this on VMs that simulate the Pi; there is no I2C bus, DAC, ADS1115, or drone. Everything must go through dry-run paths.

### Setup (once)

```bash
git clone https://github.com/RowdyCreators/Fly-Brains-A-Drone-Controlled-by-the-Human-Mind
cd Fly-Brains-A-Drone-Controlled-by-the-Human-Mind
python3 -m venv .venv && source .venv/bin/activate    # Windows: .venv\Scripts\activate
pip install -e Hardware pygame numpy scipy             # NOT root requirements.txt: adafruit-blinka fails off-Pi
```

pygame is installed separately because `Hardware/pyproject.toml` only lists smbus2 and PyYAML.

### Run

```bash
python -m unittest discover tests                                       # tests (no hardware needed)
python Hardware/hardwaretesting/dronevirtualcontroller.py --dry-run     # click-and-hold UI, Esc quits
```

Brain-control pipeline on the simulator (needs numpy + scipy; `--sim` = fake EEG + fake DAC):

```bash
python -m processing train  --profile me --sim --seconds 20
python -m processing assign --profile me --sim --primary clench --lock eyes_closed --trials 4
python -m processing fly    --profile me --sim     # keys: c/f/e toggle clench/focus/eyes_closed, q quits
```

In `train`/`assign` the simulator acts out the cues by itself; in `fly` it's driven by keys (needs a real terminal). On the Pi, drop `--sim` (or use `fly --dry-run` for real EEG + fake DAC).

Headless / no display (prints the DAC codes the real remote would receive):

```python
from hardwaretodrone import Drone

with Drone(dry_run=True, verbose=True) as drone:
    drone.hold("forward", 2)            # repeats command() for 2s, then hover
    drone.command("turn_left", strength=60)
    print(drone.status())               # target / current / codes / failsafe
    print(drone.dac.codes, drone.dac.writes)   # FakeDAC records every write
```

### Gotchas

- **Always pass `dry_run=True` / `--dry-run`.** Without it `Drone()` opens `/dev/i2c-1` and crashes on a VM.
- The virtual controller needs a GUI display (VM desktop, WSLg, or X forwarding); a headless VM can't open the pygame window — use the snippet above instead.
- A dry-run `Drone` still has the 0.5 s watchdog: call `command()` every loop or it returns to hover.
- `smbus2` is imported lazily, so dry runs work even if it isn't installed.
- **EEG side:** `get_source()` in `signals/signal.py` falls back to `brain_simulator.simulate_brain` when ADS1115/blinka imports fail, but no `brain_simulator` module exists in the repo, so running `signals/signal.py` off-Pi currently fails. Either add one (generator yielding `(timestamp, alpha, beta, gamma)` tuples) or feed `FlyBrain.receive_signals(alpha, beta, gamma)` directly.
- **Not runnable on a VM:** `examples/basicreading.py`, `Hardware/hardwaretesting/testDACsignals.c`, `pot4testing.py` (all need the real ADS1115/DAC).

## Safety-critical behavior (preserve when editing `drone.py`)

- Starts in failsafe with all sticks centered (center = hover; throttle center = hold altitude).
- Watchdog (default 0.5 s): if `command()`/`set()` isn't called, all axes ease back to center — so any control loop must call `command()` every iteration even if unchanged.
- `close()` (also via `atexit`) centers every stick. Output is slew-limited (default 4.0/s).
- Takeoff/landing are done by hand with the remote's physical button; axis names assume Mode 2 and the calibration in `hardware.yaml` was measured on the real remote.
- `MCP4728` only writes volatile DAC registers (never EEPROM) so it is safe at loop rate; keep it that way.
