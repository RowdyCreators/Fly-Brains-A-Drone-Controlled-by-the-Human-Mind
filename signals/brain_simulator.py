import math
import random
import time


def simulate_brain(rate_hz=10, max_samples=None):
    """Yield fake (alpha, beta, gamma) values that drift a little, like a real signal."""
    count = 0
    while max_samples is None or count < max_samples:
        t = count / rate_hz
        alpha = 0.12 + 0.05 * math.sin(2 * math.pi * 0.5 * t) + random.gauss(0, 0.01)
        beta = -0.04 + 0.03 * math.sin(2 * math.pi * 0.3 * t) + random.gauss(0, 0.01)
        gamma = 0.32 + 0.02 * math.sin(2 * math.pi * 0.2 * t) + random.gauss(0, 0.01)
        yield alpha, beta, gamma
        count += 1
        time.sleep(1.0 / rate_hz)
