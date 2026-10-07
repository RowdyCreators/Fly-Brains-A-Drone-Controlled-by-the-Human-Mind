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
Jasper Stephens
### Resources:
- BioAmp EXG Pill:
  - [BioAmp EXG Pill GitHub](https://github.com/upsidedownlabs/BioAmp-EXG-Pill)
  - [BioAmp EXG Pill Documentation](https://robu-prod-media.s3.ap-south-1.amazonaws.com/uploads/2024/04/BioAmp-EXG-Pill-Data-Sheet.pdf)
- [Raspberry Pi Documentation](https://www.raspberrypi.com/documentation/)
- [Arduino Documentation](https://docs.arduino.cc)
- [Adafruit ADS1115 Information](https://www.adafruit.com/product/1085)


### Assumed Data Format:
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