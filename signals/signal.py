'''
TODO (somebody do this plz)

import the follwing

adafruit-circuitpython-ads1x15 is the driver for the ADS1115. It gives you voltage readings over I2C.
adafruit-blinka is the compatibility layer that lets CircuitPython libraries run on a Raspberry Pi, and the ADS1115 library needs it.
RPi.GPIO or gpiozero let you use GPIO pins for status LEDs, a physical kill button, and similar things.
smbus2 gives lower-level I2C access if anyone wants to talk to the ADS1115 directly without Adafruit's wrapper. It's good for learning what's actually happening.


'''
'''
TODO (somebody do this plz)

import the follwing

adafruit-circuitpython-ads1x15 is the driver for the ADS1115. It gives you voltage readings over I2C.
adafruit-blinka is the compatibility layer that lets CircuitPython libraries run on a Raspberry Pi, and the ADS1115 library needs it.
RPi.GPIO or gpiozero let you use GPIO pins for status LEDs, a physical kill button, and similar things.
smbus2 gives lower-level I2C access if anyone wants to talk to the ADS1115 directly without Adafruit's wrapper. It's good for learning what's actually happening.


'''
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

    def listen(self, source, on_sample=None):
        """Receive data for real: pull (alpha, beta, gamma) from `source` until it ends.

        source:    any iterable/generator that yields (alpha, beta, gamma)
        on_sample: optional function called with each stored sample as it arrives
        Returns the list of every sample received (the array updates live).
        """
        received = []
        for alpha, beta, gamma in source:
            sample = self.receive_signals(alpha, beta, gamma)
            received.append(sample)
            if on_sample:
                on_sample(sample)
        return received


def read_ads1115(rate_hz=10, max_samples=None):
    """Yield (alpha, beta, gamma) in volts from an ADS1115 (A0=alpha, A1=beta, A2=gamma)."""
    import board
    import busio
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn

    ads = ADS.ADS1115(busio.I2C(board.SCL, board.SDA))
    alpha_ch, beta_ch, gamma_ch = (AnalogIn(ads, pin) for pin in (0, 1, 2))

    count = 0
    while max_samples is None or count < max_samples:
        yield alpha_ch.voltage, beta_ch.voltage, gamma_ch.voltage
        count += 1
        time.sleep(1.0 / rate_hz)


def get_source(rate_hz=10, max_samples=None):
    """Use the real ADS1115 if available, otherwise fall back to the brain simulator."""
    try:
        import board  # noqa: F401  (fails on a laptop / without blinka)
        import adafruit_ads1x15.ads1115  # noqa: F401

        print("Reading from ADS1115 hardware.")
        return read_ads1115(rate_hz, max_samples)
    except (ImportError, NotImplementedError):
        try:
            from .brain_simulator import simulate_brain
        except ImportError:  # run directly as a script from signals/
            from brain_simulator import simulate_brain

        print("No ADS1115 found, using the brain simulator.")
        return simulate_brain(rate_hz=rate_hz, max_samples=max_samples)


def main():
    brain = FlyBrain()

    def show(sample):
        timestamp, alpha, beta, gamma = sample
        print(f"t={timestamp:.3f}  alpha={alpha}  beta={beta}  gamma={gamma}")

    # Array that stores every sample we receive (fills as data arrives)
    received = brain.listen(get_source(rate_hz=10, max_samples=20), on_sample=show)

    print(f"Received {len(received)} samples")
    return received


if __name__ == "__main__":
    main()