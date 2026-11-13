"""The moded control scheme, as pure logic (no hardware, no clock).

Inputs are pattern events from classifier.EventDetector; output is the drone
command to send this frame. Nothing held -> "hover".

    primary  tap  -> next mode
             hold -> go: fly in the current mode while held
                     (speed modes: change strength once per hold)
    prev     tap  -> previous mode                       (optional role)
    lock     hold -> toggle lock; while locked, only the lock input works
                     (optional role; if assigned, controls start locked)
"""

from dataclasses import dataclass
from typing import Dict, List, Optional

SPEED_MODES = {"speed_up": +1, "speed_down": -1}


@dataclass
class Output:
    command: str          # a hardwaretodrone.COMMANDS name
    strength: int
    message: Optional[str] = None   # something worth announcing (mode change, lock...)


class ModeController:
    def __init__(self, roles: Dict[str, str], modes: List[str], *, tap_max_sec=1.0,
                 hold_min_sec=1.2, strength=30, speed_step=10,
                 min_strength=10, max_strength=100):
        if "primary" not in roles:
            raise ValueError("a 'primary' input pattern is required")
        if not modes:
            raise ValueError("need at least one mode")
        self.role_of = {label: role for role, label in roles.items()}
        self.modes = list(modes)
        self.tap_max = tap_max_sec
        self.hold_min = hold_min_sec
        self.strength = strength
        self.speed_step = speed_step
        self.min_strength, self.max_strength = min_strength, max_strength
        self.index = 0
        self.locked = "lock" in roles
        self._hold_used = False     # one-shot actions fire once per hold

    @property
    def mode(self):
        return self.modes[self.index]

    def _shift(self, step):
        self.index = (self.index + step) % len(self.modes)
        return f"mode -> {self.mode.upper()} [{self.index + 1}/{len(self.modes)}]"

    def step(self, events, active: Optional[str], active_for: float) -> Output:
        """events: this frame's Events; active: label currently held (or None);
        active_for: how long it's been held (s)."""
        msg = None
        for ev in events:
            role = self.role_of.get(ev.label)
            if ev.kind == "start":
                self._hold_used = False
            elif ev.kind == "end" and ev.duration < self.tap_max and not self.locked:
                if role == "primary":
                    msg = self._shift(+1)
                elif role == "prev":
                    msg = self._shift(-1)

        role = self.role_of.get(active) if active else None
        held = role is not None and active_for >= self.hold_min

        if held and role == "lock" and not self._hold_used:
            self._hold_used = True
            self.locked = not self.locked
            msg = "LOCKED" if self.locked else f"UNLOCKED, mode {self.mode.upper()}"

        if self.locked or not (held and role == "primary"):
            return Output("hover", self.strength, msg)

        if self.mode in SPEED_MODES:
            if not self._hold_used:
                self._hold_used = True
                s = self.strength + SPEED_MODES[self.mode] * self.speed_step
                self.strength = max(self.min_strength, min(self.max_strength, s))
                msg = f"strength -> {self.strength}"
            return Output("hover", self.strength, msg)

        return Output(self.mode, self.strength, msg)
