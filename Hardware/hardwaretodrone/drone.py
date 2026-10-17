"""High-level drone control over the wired remote.

    with Drone() as drone:              # all sticks centered = hover
        drone.command("forward")        # discrete, uses drone.strength (1-100)
        drone.set(pitch=0.4, yaw=-0.2)  # continuous, -1..+1 per axis

A background thread owns the DAC: it runs at `rate_hz`, eases outputs toward
their targets (slew limit), and enforces the watchdog. If no command or set()
arrives for `watchdog` seconds, every axis returns to center (hover). So a
running classifier must keep sending its current command every cycle.

Takeoff / landing are done with the remote's physical button.
"""

import atexit
import logging
import threading
import time

from .axes import axis_to_code, clamp
from .config import AXIS_NAMES, load_config
from .dac import MCP4728, FakeDAC

log = logging.getLogger("hardwaretodrone")

# command name -> direction per axis (scaled by strength)
COMMANDS = {
    "hover":      {},
    "forward":    {"pitch": +1},
    "backward":   {"pitch": -1},
    "left":       {"roll": -1},
    "right":      {"roll": +1},
    "turn_left":  {"yaw": -1},
    "turn_right": {"yaw": +1},
    "up":         {"throttle": +1},
    "down":       {"throttle": -1},
}


class Drone:
    def __init__(self, config=None, *, strength=30, watchdog=0.5,
                 rate_hz=50, slew=4.0, dry_run=False, verbose=False):
        """
        config    path   hardware.yaml (default: Hardware/hardware.yaml)
        strength  1-100  how far discrete commands push the stick (default gentle)
        watchdog  sec    no input for this long -> hover
        rate_hz          output loop rate
        slew      1/sec  max change per axis per second (4.0 = 0 to full in 0.25s)
        dry_run          use a fake DAC (no hardware needed)
        """
        self.cfg = load_config(config)
        self.strength = strength
        self.watchdog = float(watchdog)
        self.slew = float(slew)
        self._dt = 1.0 / float(rate_hz)

        self.dac = FakeDAC(verbose=verbose) if dry_run else \
            MCP4728(self.cfg.i2c_bus, self.cfg.dac_address)

        self._lock = threading.Lock()
        self._target = {a: 0.0 for a in AXIS_NAMES}
        self._current = {a: 0.0 for a in AXIS_NAMES}
        self._last_input = 0.0               # never -> starts in failsafe (hover)
        self._failsafe = True
        self._codes = None
        self.last_error = None

        # Start centered. Per-channel writes also force Vref = VDD, gain x1.
        for name, cal in self.cfg.axes.items():
            self.dac.set_code(cal.channel, axis_to_code(cal, 0.0, self.cfg.vdd))

        self._running = True
        self._thread = threading.Thread(target=self._loop, name="drone-output", daemon=True)
        self._thread.start()
        atexit.register(self.close)

    # ── settings ──────────────────────────────────────────────

    @property
    def strength(self):
        return self._strength

    @strength.setter
    def strength(self, value):
        value = int(value)
        if not 1 <= value <= 100:
            raise ValueError("strength must be 1-100")
        self._strength = value

    # ── control ───────────────────────────────────────────────

    def command(self, name, strength=None):
        """Discrete move. Axes not involved in the command return to center."""
        if name not in COMMANDS:
            raise ValueError(f"unknown command {name!r}; options: {', '.join(COMMANDS)}")
        s = (self._strength if strength is None else strength) / 100.0
        if not 0.01 <= s <= 1.0:
            raise ValueError("strength must be 1-100")
        target = {a: 0.0 for a in AXIS_NAMES}
        for axis, direction in COMMANDS[name].items():
            target[axis] = direction * s
        self._apply(target)

    def set(self, pitch=None, roll=None, yaw=None, throttle=None):
        """Continuous control, -1..+1 per axis (full stick, NOT scaled by strength).
        Axes you leave out keep their current target."""
        with self._lock:
            target = dict(self._target)
        for axis, v in (("pitch", pitch), ("roll", roll), ("yaw", yaw), ("throttle", throttle)):
            if v is not None:
                target[axis] = clamp(float(v))
        self._apply(target)

    def hover(self):
        self.command("hover")

    def hold(self, name, seconds, strength=None):
        """Blocking helper for scripts/tests: repeat a command for `seconds`,
        then hover. (A BCI loop should call command() each cycle instead.)"""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.command(name, strength)
            time.sleep(min(0.1, self.watchdog / 3))
        self.hover()

    # ── status ────────────────────────────────────────────────

    @property
    def failsafe_active(self):
        return self._failsafe

    def status(self):
        with self._lock:
            return {
                "target": dict(self._target),
                "current": {a: round(v, 3) for a, v in self._current.items()},
                "codes": list(self._codes) if self._codes else None,
                "failsafe": self._failsafe,
                "strength": self._strength,
            }

    # ── internals ─────────────────────────────────────────────

    def _apply(self, target):
        with self._lock:
            self._target = target
            self._last_input = time.monotonic()
            if self._failsafe:
                self._failsafe = False
                log.info("input received, failsafe cleared")

    def _loop(self):
        step = self.slew * self._dt
        while self._running:
            t0 = time.monotonic()
            with self._lock:
                if t0 - self._last_input > self.watchdog:
                    if not self._failsafe:
                        log.warning("watchdog: no input for %.2fs -> hover", self.watchdog)
                        self._failsafe = True
                    self._target = {a: 0.0 for a in AXIS_NAMES}

                for a in AXIS_NAMES:                      # slew toward target
                    diff = self._target[a] - self._current[a]
                    self._current[a] += max(-step, min(step, diff))

                codes = [0] * 4
                for name, cal in self.cfg.axes.items():
                    codes[cal.channel] = axis_to_code(cal, self._current[name], self.cfg.vdd)

            if codes != self._codes:
                try:
                    self.dac.set_codes(codes)
                    self._codes = codes
                except OSError as e:                      # I2C hiccup: retry next tick
                    if str(e) != str(self.last_error):
                        log.error("DAC write failed: %s", e)
                    self.last_error = e

            time.sleep(max(0.0, self._dt - (time.monotonic() - t0)))

    def close(self):
        """Stop the loop and leave every stick centered (hover)."""
        if not getattr(self, "_running", False):
            return
        self._running = False
        self._thread.join(timeout=1.0)
        try:
            for name, cal in self.cfg.axes.items():
                self.dac.set_code(cal.channel, axis_to_code(cal, 0.0, self.cfg.vdd))
        except OSError as e:
            log.error("could not center sticks on close: %s", e)
        self.dac.close()
        log.info("closed, sticks centered")

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
