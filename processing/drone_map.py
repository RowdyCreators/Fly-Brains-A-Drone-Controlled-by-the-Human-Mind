"""Option C drone mapping: 3 brain signals -> 3D drone movement.

Implements the map in ``drone-mapping-plan.md`` (reviewed and approved
by Chan, 2026-11-11). One channel = one signed drone axis:

    alpha -> pitch    (forward +1 / backward -1)   X, front/back
    beta  -> roll     (right   +1 / left     -1)   Y, left/right
    gamma -> throttle (up      +1 / down     -1)   Z, up/down
    yaw   -> always 0 in v1 (no 4th signal to drive it)

XY plane movement is alpha + beta together; up/down is gamma alone;
all three together is a 3D diagonal, sent in a single
``drone.set(pitch, roll, yaw, throttle)`` call. ``command()`` is NOT
used here: it centres every other axis, so XY + up/down together
would be impossible with it.

How the +1 / -1 increments work: the mapper keeps a velocity per
axis, clamped to [-MAX_SPEED, +MAX_SPEED]. Each processed window:

* channel +1 (debounced, from ``result["channel_active"]``):
  velocity += STEP
* channel -1: velocity -= STEP
* channel 0 (dead zone): velocity decays toward 0 by STEP
* then clamp, then ``drone.set(...)`` with the current velocity.

STEP = 0.25, MAX_SPEED = 0.5 to start: 2 confirmed windows to reach
half stick on one axis, and the drone never gets more than half
stick from brain control. Stop activating a channel and that axis
eases back to hover instead of sticking at speed.

Safety: ``drive()`` / ``update()`` must be called on every processed
window, even when nothing changed -- the Drone's 0.5 s watchdog
returns the sticks to hover otherwise, and calling ``set()`` every
window is what keeps it fed. ``hover()`` resets all velocities to 0
and, if given a drone, sends ``set(0, 0, 0, 0)``. Any result of None
is treated as hover (velocity decays). Takeoff / landing are never
done here -- they are by hand with the remote's physical button.

Stdlib only, no numpy. ``drone`` is duck-typed (only ``.set()`` is
used), so this works with a fake in tests and with the real
``hardwaretodrone.Drone`` -- that package lives on ``main``, not on
``Brain_sim_Test``, so this module does not import it.
"""

# One channel = one axis. Sign: +1 = channel above baseline,
# -1 = channel below baseline, 0 = dead zone (see processor.py).
CHANNEL_TO_AXIS = {"alpha": "pitch", "beta": "roll", "gamma": "throttle"}
AXES = ("pitch", "roll", "yaw", "throttle")

DEFAULT_STEP = 0.25
DEFAULT_MAX_SPEED = 0.5
DEFAULT_POS_THRESHOLD = 1.0
DEFAULT_NEG_THRESHOLD = -1.0
DEFAULT_DEBOUNCE = 3


def _clamp(value, low, high):
    return max(low, min(high, value))


class DroneMapper:
    """Stepwise velocity mapper from processor results to drone axes."""

    def __init__(self, step=DEFAULT_STEP, max_speed=DEFAULT_MAX_SPEED,
                 pos_threshold=DEFAULT_POS_THRESHOLD,
                 neg_threshold=DEFAULT_NEG_THRESHOLD,
                 debounce=DEFAULT_DEBOUNCE):
        self.step = step
        self.max_speed = max_speed
        self.pos_threshold = pos_threshold
        self.neg_threshold = neg_threshold
        self.debounce = debounce
        self.velocity = {axis: 0.0 for axis in AXES}
        # Fallback debounce state, only used when a result carries no
        # "channel_active" (e.g. fed raw z-scores directly).
        self._pending_active = {"alpha": 0, "beta": 0, "gamma": 0}
        self._pending_count = {"alpha": 0, "beta": 0, "gamma": 0}

    def _activations_from_z(self, z):
        """Raw +1/0/-1 per channel from z-scores, debounced per channel."""
        out = {}
        for ch in ("alpha", "beta", "gamma"):
            if z[ch] >= self.pos_threshold:
                raw = 1
            elif z[ch] <= self.neg_threshold:
                raw = -1
            else:
                raw = 0
            if raw == self._pending_active[ch]:
                self._pending_count[ch] += 1
            else:
                self._pending_active[ch] = raw
                self._pending_count[ch] = 1
            if self._pending_count[ch] >= self.debounce:
                out[ch] = raw
            else:
                out[ch] = 0
        return out

    def activations(self, result):
        """Per-channel +1/0/-1 for a processor result dict.

        Prefers the processor's own debounced ``channel_active``
        (already debounced per channel, so no second debounce here).
        Falls back to the z-scores when ``channel_active`` is absent.
        A None result means hover: all channels 0.
        """
        if result is None:
            return {"alpha": 0, "beta": 0, "gamma": 0}
        if "channel_active" in result:
            return {ch: int(result["channel_active"].get(ch, 0))
                    for ch in ("alpha", "beta", "gamma")}
        return self._activations_from_z(result["z"])

    def update(self, result):
        """Advance the velocity by one window and return the axes dict.

        Does not touch a drone -- use ``drive()`` for that. Returns a
        copy of the current velocity: {pitch, roll, yaw, throttle},
        each in [-max_speed, +max_speed], yaw always 0.0 in v1.
        """
        active = self.activations(result)
        for ch, axis in CHANNEL_TO_AXIS.items():
            a = active[ch]
            v = self.velocity[axis]
            if a > 0:
                v += self.step
            elif a < 0:
                v -= self.step
            else:
                # Dead zone: decay toward 0 by one step.
                if v > 0:
                    v = max(0.0, v - self.step)
                elif v < 0:
                    v = min(0.0, v + self.step)
            self.velocity[axis] = _clamp(v, -self.max_speed, self.max_speed)
        self.velocity["yaw"] = 0.0
        return dict(self.velocity)

    def drive(self, drone, result):
        """Advance one window and send the axes to the drone via set().

        Call on every processed window, even when the velocity has
        not changed, so the Drone's 0.5 s watchdog stays fed.
        Returns the axes dict that was sent.
        """
        axes = self.update(result)
        drone.set(pitch=axes["pitch"], roll=axes["roll"],
                  yaw=axes["yaw"], throttle=axes["throttle"])
        return axes

    def hover(self, drone=None):
        """Reset all velocities to 0 and, if given a drone, centre it."""
        self.velocity = {axis: 0.0 for axis in AXES}
        if drone is not None:
            drone.set(pitch=0.0, roll=0.0, yaw=0.0, throttle=0.0)
        return dict(self.velocity)
