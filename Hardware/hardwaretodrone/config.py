"""Load hardware.yaml into plain dataclasses."""

from dataclasses import dataclass
from pathlib import Path

import yaml

AXIS_NAMES = ("pitch", "roll", "yaw", "throttle")
CHANNELS = "ABCD"

# Hardware/hardware.yaml, found no matter which directory a script runs from
DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "hardware.yaml"


@dataclass(frozen=True)
class AxisCal:
    name: str
    channel: int        # 0..3  (A..D)
    min_v: float        # stick fully toward min_dir
    center_v: float     # stick released
    max_v: float        # stick fully toward max_dir


@dataclass(frozen=True)
class HardwareConfig:
    i2c_bus: int
    dac_address: int
    vdd: float
    axes: dict          # name -> AxisCal


def load_config(path=None) -> HardwareConfig:
    data = yaml.safe_load(Path(path or DEFAULT_CONFIG).read_text())

    pi = data["pi"]
    ctl = data["control"]
    vdd = float(pi.get("logic_voltage", 3.3))

    axes = {}
    for name in AXIS_NAMES:
        raw = ctl["axes"].get(name)
        if raw is None:
            raise ValueError(f"hardware.yaml: control.axes.{name} is missing")
        for key in ("channel", "min_v", "center_v", "max_v"):
            if raw.get(key) is None:
                raise ValueError(f"hardware.yaml: control.axes.{name}.{key} is not set")

        ch = str(raw["channel"]).upper()
        if ch not in CHANNELS:
            raise ValueError(f"hardware.yaml: {name}.channel must be A-D, got {ch!r}")

        cal = AxisCal(
            name=name,
            channel=CHANNELS.index(ch),
            min_v=float(raw["min_v"]),
            center_v=float(raw["center_v"]),
            max_v=float(raw["max_v"]),
        )
        if not (0 <= cal.min_v < cal.center_v < cal.max_v <= vdd + 0.05):
            raise ValueError(f"hardware.yaml: {name} voltages must satisfy "
                             f"0 <= min < center < max <= {vdd}V, got {cal}")
        axes[name] = cal

    used = sorted(a.channel for a in axes.values())
    if used != [0, 1, 2, 3]:
        raise ValueError("hardware.yaml: each axis needs its own channel A-D")

    return HardwareConfig(
        i2c_bus=int(pi.get("i2c_bus", 1)),
        dac_address=int(ctl["dac"]["i2c_address"]),   # YAML reads 0x60 as int
        vdd=vdd,
        axes=axes,
    )
