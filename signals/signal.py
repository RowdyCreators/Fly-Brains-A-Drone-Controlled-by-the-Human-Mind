'''
TODO (somebody do this plz)

import the follwing

adafruit-circuitpython-ads1x15 is the driver for the ADS1115. It gives you voltage readings over I2C.
adafruit-blinka is the compatibility layer that lets CircuitPython libraries run on a Raspberry Pi, and the ADS1115 library needs it.
RPi.GPIO or gpiozero let you use GPIO pins for status LEDs, a physical kill button, and similar things.
smbus2 gives lower-level I2C access if anyone wants to talk to the ADS1115 directly without Adafruit's wrapper. It's good for learning what's actually happening.
'''
import csv
import os
import time
from collections import deque


class FlyBrain:
    """Receives 3-channel signals (alpha, beta, gamma)."""

    def __init__(self, buffer_size=256):
        # Rolling buffer of the most recent samples: (timestamp, alpha, beta, gamma)
        self.buffer = deque(maxlen=buffer_size)
        self.latest = None

    def receive_signals(self, alpha, beta, gamma, timestamp=None):
        """Receive one sample from each of the 3 BCI channels."""
        for name, value in (("alpha", alpha), ("beta", beta), ("gamma", gamma)):
            if not isinstance(value, (int, float)):
                raise TypeError(f"{name} must be a number, got {type(value).__name__}")

        if timestamp is None:
            timestamp = time.time()

        self.latest = (timestamp, float(alpha), float(beta), float(gamma))
        self.buffer.append(self.latest)
        return self.latest

    def listen(self, source, on_sample=None, log_path=None):
        """Pull samples from `source`. If log_path is given, also save each one to a CSV."""
        received = []
        file = open(log_path, "w", newline="") if log_path else None
        try:
            writer = csv.writer(file) if file else None
            if writer:
                writer.writerow(["timestamp", "alpha", "beta", "gamma"])

            for alpha, beta, gamma in source:
                sample = self.receive_signals(alpha, beta, gamma)
                received.append(sample)
                if writer:
                    writer.writerow(sample)
                    file.flush()  # push to disk right away
                if on_sample:
                    on_sample(sample)
        finally:
            if file:
                file.close()  # always runs, even on errors or Ctrl+C
        return received


def open_ads1115():
    """Set up the ADS1115 and return its 3 channels. Raises if the hardware isn't there."""
    import board
    import busio
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn

    ads = ADS.ADS1115(busio.I2C(board.SCL, board.SDA))
    return tuple(AnalogIn(ads, pin) for pin in (0, 1, 2))  # A0=alpha, A1=beta, A2=gamma


def read_ads1115(channels, rate_hz=10, max_samples=None):
    """Yield (alpha, beta, gamma) in volts from the already-opened channels."""
    alpha_ch, beta_ch, gamma_ch = channels

    count = 0
    while max_samples is None or count < max_samples:
        yield alpha_ch.voltage, beta_ch.voltage, gamma_ch.voltage
        count += 1
        time.sleep(1.0 / rate_hz)


def get_source(rate_hz=10, max_samples=None):
    """Use the real ADS1115 if available, otherwise fall back to the brain simulator."""
    try:
        channels = open_ads1115()  # setup happens here, inside the try
        print("Reading from ADS1115 hardware.")
        return read_ads1115(channels, rate_hz, max_samples)
    except (ImportError, NotImplementedError, ValueError, OSError, RuntimeError):
        from brain_sim import simulate_brain

        print("No ADS1115 found, using the brain simulator.")
        return simulate_brain(rate_hz=rate_hz, max_samples=max_samples)


def main():
    brain = FlyBrain()

    def show(sample):
        timestamp, alpha, beta, gamma = sample
        print(f"t={timestamp:.3f}  alpha={alpha}  beta={beta}  gamma={gamma}")

    # Folder called "samples" next to this script
    samples_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "samples")
    os.makedirs(samples_dir, exist_ok=True)

    received = brain.listen(
        get_source(rate_hz=10, max_samples=20),
        on_sample=show,
        log_path=os.path.join(samples_dir, "samples.csv"),
    )

    print(f"Received {len(received)} samples")
    return received


if __name__ == "__main__":
    main()