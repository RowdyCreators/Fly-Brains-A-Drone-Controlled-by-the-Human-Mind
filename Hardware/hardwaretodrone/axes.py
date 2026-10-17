"""Convert normalized stick positions (-1..+1) into DAC codes using calibration.

Sign convention (matches max_dir in hardware.yaml):
    pitch    +1 = right stick UP     (forward)
    roll     +1 = right stick RIGHT
    yaw      +1 = left stick RIGHT   (turn right)
    throttle +1 = left stick UP      (climb; 0 = hold altitude)
"""

from .config import AxisCal
from .dac import MAX_CODE


def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


def axis_to_volts(cal: AxisCal, value: float) -> float:
    """-1 → min_v, 0 → center_v, +1 → max_v (piecewise, so center is exact
    even when the stick's range isn't symmetric)."""
    v = clamp(value)
    if v >= 0:
        return cal.center_v + v * (cal.max_v - cal.center_v)
    return cal.center_v + v * (cal.center_v - cal.min_v)


def volts_to_code(volts: float, vdd: float) -> int:
    volts = max(0.0, min(vdd, volts))
    return int(round(volts / vdd * MAX_CODE))


def axis_to_code(cal: AxisCal, value: float, vdd: float) -> int:
    return volts_to_code(axis_to_volts(cal, value), vdd)
