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

Pin mapping and channel assignments live in [`hardware.yaml`](hardware.yaml).

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
