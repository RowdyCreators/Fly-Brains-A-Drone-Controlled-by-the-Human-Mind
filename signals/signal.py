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


def main():
    brain = FlyBrain()

    # Array that stores every sample we receive
    received = []

    # Stand-in for incoming BCI data (replace with the real stream later)
    incoming = [
        (0.12, -0.05, 0.33),
        (0.15, -0.02, 0.30),
        (0.09, -0.07, 0.35),
        (0.11, -0.04, 0.31),
    ]

    for alpha, beta, gamma in incoming:
        sample = brain.receive_signals(alpha, beta, gamma)
        received.append(sample)

    print(f"Received {len(received)} samples:")
    for timestamp, alpha, beta, gamma in received:
        print(f"t={timestamp:.3f}  alpha={alpha}  beta={beta}  gamma={gamma}")

    return received


if __name__ == "__main__":
    main()