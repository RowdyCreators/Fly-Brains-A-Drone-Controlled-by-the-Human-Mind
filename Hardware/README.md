# Hardware — `hardwaretodrone`

Python control library for the Holy Stone HS210 Pro, driven through its stock
remote. A Raspberry Pi 4 sets the remote's four joystick voltages with an
MCP4728 DAC, so the remote "thinks" its sticks are being moved.

This folder is the hardware layer of the project: other parts of the repo
(e.g. the EEG classifier) send movement commands through it.

## Setup

From the repo root:
```bash
pip install -e Hardware
```
This installs `hardwaretodrone` (and its dependencies) in editable mode, so
`import hardwaretodrone` works from anywhere in the repo, and edits to the
code take effect immediately.

Enable I2C on the Pi (`/boot/firmware/config.txt`):
```
dtparam=i2c_arm=on
dtparam=i2c_arm_baudrate=400000
```
Check that `i2cdetect -y 1` shows `0x48` (ADS1115) and `0x60` (MCP4728).

## Usage

```python
from hardwaretodrone import Drone

with Drone(strength=30) as drone:   # finds Hardware/hardware.yaml; starts centered = hover
    drone.command("forward")          # discrete move at drone.strength (1-100)
    drone.command("turn_left", strength=60)
    drone.set(pitch=0.4, yaw=-0.2)    # continuous, -1..+1 per axis (full stick)
    drone.hover()
```

**Commands:** `hover`, `forward`, `backward`, `left`, `right`,
`turn_left`, `turn_right`, `up`, `down`

**Takeoff / landing:** press the remote's one-key button by hand. With
altitude hold, throttle at center means hold height, so `up` and `down` adjust altitude.

## Safety behavior

| Situation | What happens |
|---|---|
| Startup | All sticks centered (hover) |
| No `command()`/`set()` for 0.5s (`watchdog=`) | All sticks ease back to center (hover) |
| Program exits, crashes, or Ctrl-C | Sticks centered on close |
| Any change | Eased at `slew=4.0`/s (0 to full stick in 0.25s), no jerks |

Because of the watchdog, **a control loop must call `command()` every
iteration**, even when the command hasn't changed.

## Files

| File | Purpose |
|---|---|
| `hardware.yaml` | Pins, addresses, per-axis stick calibration (loaded automatically) |
| `pyproject.toml` / `requirements.txt` | Install metadata / dependencies |
| `dac_sweep.c` | Standalone C tool: sweep each DAC channel to check voltages with a multimeter |
| `hardwaretodrone/config.py` | Loads + validates `hardware.yaml` |
| `hardwaretodrone/dac.py` | MCP4728 driver (smbus2), plus `FakeDAC` for testing |
| `hardwaretodrone/axes.py` | -1..+1 → volts → DAC code using calibration |
| `hardwaretodrone/drone.py` | `Drone` class: commands, watchdog, slew, output thread |
