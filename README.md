# Fly-Brains-A-Drone-Controlled-by-the-Human-Mind
A drone controlled by the human mind!

### Meet the Team
Samuel Saylor\
Donny Phounsavath\
Chan Polavarapu\
Aaban Moiz\
Dennis Morgan\
Angel Moya\
Shrey Patel\
Kosha Antala\
Jasper Stephens\
Shah Zaman     

## Resources:
- BioAmp EXG Pill:
  - [BioAmp EXG Pill GitHub](https://github.com/upsidedownlabs/BioAmp-EXG-Pill)
  - [BioAmp EXG Pill Documentation](https://robu-prod-media.s3.ap-south-1.amazonaws.com/uploads/2024/04/BioAmp-EXG-Pill-Data-Sheet.pdf)
- [Raspberry Pi Documentation](https://www.raspberrypi.com/documentation/)
- [Arduino Documentation](https://docs.arduino.cc)
- [Adafruit ADS1115 Information](https://www.adafruit.com/product/1085)

## Hardware

| Part | Role |
|---|---|
| Raspberry Pi 4 | Runs the EEG pipeline, classification, and drone control |
| BioAmp EXG Pill | EEG front end that amplifies brain/blink signals from the electrodes |
| ADS1115 (I2C `0x48`) | 16-bit ADC that digitizes the BioAmp output for the Pi |
| MCP4728 (I2C `0x60`) | 4-channel 12-bit DAC that drives the remote's joystick axes |
| Holy Stone HS210 Pro | Drone with optical-flow hover and WiFi FPV camera, flown through its stock remote |
| EEG electrodes + jumper wires | Signal pickup and wiring |

Pin mapping and channel assignments live in [`hardware.yaml`](Hardware/hardware.yaml).

## Running the Demo (on the Pi)

### Setup (once)

```bash
cd ~/Downloads/Fly-Brains-A-Drone-Controlled-by-the-Human-Mind
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt && pip install -e Hardware
```

- If the first run asks to install missing Blinka dependencies (`gpiod`, `rpi_ws281x`, etc.), answer `y`.
- I2C must be on: `sudo raspi-config` → Interface Options → I2C → Enable. Check with `i2cdetect -y 1`: the ADS1115 shows at `48`, the MCP4728 at `60`.

### Every time (including after a reboot)

```bash
cd ~/Downloads/Fly-Brains-A-Drone-Controlled-by-the-Human-Mind
source .venv/bin/activate          # prompt should start with (.venv)
```

### Electrode placement (jaw clench, no hair needed)

Hair blocks the electrodes, so use the **cheek muscle (masseter)**. To find it, put your fingertips on your cheek about halfway between the earlobe and the corner of your mouth, just above the jawbone, then clench. The part that pops out is the muscle.

| Pill pin | Where |
|---|---|
| **IN+** | Center of the cheek bulge |
| **IN−** | Same muscle, ~2 cm below IN+ (toward the jaw corner) |
| **REF** | Earlobe or collarbone (bony, hair-free, no muscle) |

- Use **fresh** gel electrodes (they dry out after 1–2 uses), wipe the skin with alcohol first, and press each one on firmly.
- Tape the leads down so they don't tug.
- Run the Pi and any connected laptop on battery, not wall power, while electrodes are on someone.

### Step 1: check the raw signal

```bash
python examples/basicreading.py
```

The bar should sit mid-range (~1.5 V) and **jump when you clench**. If it doesn't, fix the electrodes before going further (see Troubleshooting).

### Step 2: bench test (real EEG, fake DAC, drone does NOT move)

```bash
python processing/fly.py --signal gamma --cal-sec 10
```

- **Relax phase:** jaw loose, teeth apart, sit the way you will while flying.
- **Clench phase:** firm, steady clench (~60% effort, not max) for the whole phase.
- A good calibration looks roughly like `relax 10%, action 60%+`. While flying, clench should show `forward` and relax should show `turn_right`.

### Step 3: fly the real drone

```bash
python processing/fly.py --signal gamma --cal-sec 10 --real
```

- Only `--real` sends commands to the MCP4728 DAC → remote joysticks. Without it, nothing reaches the drone.
- You must type `FLY` to confirm. Stick strength is capped at 50.
- Clench = forward, relax = turn right, unsure / signal lost = hover.
- **Take off and land by hand** with the remote's button; the script never does.
- Do one `--real` run with the drone on the ground first to confirm the directions.
- **Ctrl+C** centers all sticks. Keep a hand near the remote's land/power button.

### Backup demo (no electrodes / any machine)

```bash
python processing/fly.py --sim     # simulated EEG + fake DAC, shows the full pipeline
```

### Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `AttributeError: ... has no attribute 'P0'` | Old code + new ADS1x15 library. Pull the latest code (pins are now plain `0, 1, 2` in `signals/eeg_pipeline.py`). |
| `CALIBRATION FAILED ... moved 0% -> 0%` or barely moves | No muscle signal is reaching the Pill: electrodes dry, covered by hair, or not on the muscle. Re-place with fresh electrodes. |
| Calibration shows action *lower* than relax | Clenching too hard (electrode slides / Pill overloads). Use a firm but moderate clench. |
| Always `forward`, never turns | Calibration clench was weaker than your real clenches, or the leads shifted after calibrating. Recalibrate; tape the leads. |
| `basicreading.py` sits at 0 V | Pill has no power or OUT is off A0. Check Pill VCC → Pi 3.3 V, GND → GND, OUT → ADS1115 A0. |
| Bar moves when you unclip IN+ but not when you clench | Pill/ADC/Pi are fine; the problem is the electrodes/skin contact. |
| `No I2C device` / `Remote I/O error` | I2C disabled or a wire is loose; see Setup and `i2cdetect -y 1`. |
| `python3 -m pip` says "externally-managed-environment" | You're outside the venv; run `source .venv/bin/activate`. |
| Want alpha (eyes closed) instead of jaw clench | `--signal alpha` with IN+ on the back of the head (needs scalp contact through hair). |

## Assumed Data Format:
3 Channels with alpha, beta, and gamma waves (one row per channel)
[
    [alpha_ch0, beta_ch0, gamma_ch0],
    [alpha_ch1, beta_ch1, gamma_ch1],
    [alpha_ch2, beta_ch2, gamma_ch2]
]

Example: 
[
    [0.42, 0.31, 0.27],
    [0.38, 0.35, 0.27],
    [0.47, 0.29, 0.24]
]

## Testing WITHOUT the Hardware

If you wish to give it a shot at testing this project WITHOUT access to the hardware assumed for the project, we do provide CLAUDE.md with instructions for an LLM to simulate the program. Keep in mind it isn't intended for vibecoding, but instead using LLMs as your "hardware". We actually created that file as a solution to our most prevalent issue: our campus' WiFi being terrible and making SSHing nearly impossible.
