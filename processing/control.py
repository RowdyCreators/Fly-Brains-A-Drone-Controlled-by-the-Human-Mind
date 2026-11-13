"""Phase 3: turn live frames into drone commands.

frame -> z-score vs baseline -> classifier -> event detector -> mode controller
      -> drone.command(...)   (sent EVERY frame, so the drone's watchdog stays fed;
                               if this loop stalls, the drone returns to hover)
"""

from dataclasses import dataclass

from .classifier import REST, Classifier, EventDetector, zscore
from .modes import ModeController, Output


@dataclass
class Tick:
    t: float
    label: str
    confidence: float
    active: str
    active_for: float
    out: Output


class Pilot:
    """One step per frame. Kept separate from the loop so it can be tested offline."""

    def __init__(self, profile, drone=None, on_frames=2, off_frames=2):
        if not profile.has_patterns:
            raise RuntimeError("profile has no assigned patterns; run `assign` first")
        self.profile = profile
        self.clf = Classifier.from_dict(profile.classifier)
        self.detector = EventDetector(on_frames, off_frames)
        self.modes = ModeController(
            profile.roles, profile.modes, tap_max_sec=profile.tap_max_sec,
            hold_min_sec=profile.hold_min_sec, strength=profile.start_strength,
            speed_step=profile.speed_step)
        self.drone = drone

    def step(self, frame) -> Tick:
        z = zscore(frame.features, self.profile.baseline_mean, self.profile.baseline_std)
        label, conf = self.clf.predict(z)
        events = self.detector.update(label, frame.t)
        active = self.detector.active
        held = self.detector.duration(frame.t)
        out = self.modes.step(events, active, held)
        if self.drone is not None:
            self.drone.command(out.command, strength=out.strength)
        return Tick(frame.t, label, conf, active or REST, held, out)


def status_line(pilot, tick):
    m = pilot.modes
    state = "LOCKED" if m.locked else "armed "
    return (f"{state} | mode {m.mode.upper():<10} [{m.index + 1}/{len(m.modes)}] "
            f"| str {tick.out.strength:3d} | see {tick.label:<11} {tick.confidence:4.0%} "
            f"| held {tick.active:<11} {tick.active_for:4.1f}s | -> {tick.out.command}")


def fly(frames, pilot, show=print, status=None):
    """Run until the frame source ends or Ctrl+C. `show` gets announcements,
    `status` (optional) gets a status string every frame."""
    try:
        for frame in frames:
            tick = pilot.step(frame)
            if tick.out.message:
                show(tick.out.message)
            if status:
                status(status_line(pilot, tick))
    except KeyboardInterrupt:
        pass
    finally:
        if pilot.drone is not None:
            pilot.drone.hover()
