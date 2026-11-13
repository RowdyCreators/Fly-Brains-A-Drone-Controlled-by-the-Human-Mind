import pathlib
import sys
import tempfile
import time
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "Hardware"))

try:
    import numpy as np
    import scipy  # noqa: F401
except ImportError:
    raise unittest.SkipTest("numpy/scipy not installed")

from processing import assignment, training
from processing import stream as st
from processing.classifier import REST, Classifier, Event, EventDetector
from processing.control import Pilot
from processing.modes import ModeController
from processing.sim import SimBrain
from processing.userprofile import Profile


def ev(kind, label, duration=0.0):
    return [Event(kind, label, 0.0, duration)]


class ModeControllerTests(unittest.TestCase):
    def make(self, **roles):
        roles = roles or {"primary": "clench"}
        return ModeController(roles, ["forward", "up", "speed_up", "speed_down"],
                              tap_max_sec=1.0, hold_min_sec=1.2, strength=30, speed_step=10)

    def test_idle_is_hover(self):
        self.assertEqual(self.make().step([], None, 0).command, "hover")

    def test_tap_cycles_mode_and_wraps(self):
        m = self.make()
        m.step(ev("end", "clench", 0.4), None, 0)
        self.assertEqual(m.mode, "up")
        for _ in range(3):
            m.step(ev("end", "clench", 0.4), None, 0)
        self.assertEqual(m.mode, "forward")

    def test_hold_goes_only_after_hold_min(self):
        m = self.make()
        self.assertEqual(m.step(ev("start", "clench"), "clench", 0.2).command, "hover")
        self.assertEqual(m.step([], "clench", 1.5).command, "forward")
        # releasing a hold is not a tap
        out = m.step(ev("end", "clench", 1.6), None, 0)
        self.assertEqual((out.command, m.mode), ("hover", "forward"))

    def test_speed_mode_steps_once_per_hold(self):
        m = self.make()
        m.index = 2  # speed_up
        m.step(ev("start", "clench"), "clench", 0.1)
        for t in (1.3, 1.5, 2.0):
            out = m.step([], "clench", t)
        self.assertEqual((out.command, m.strength), ("hover", 40))
        m.step(ev("end", "clench", 2.1), None, 0)
        m.step(ev("start", "clench"), "clench", 0.1)
        m.step([], "clench", 1.3)
        self.assertEqual(m.strength, 50)

    def test_strength_is_clamped(self):
        m = self.make()
        m.index, m.strength = 3, 10  # speed_down at minimum
        m.step(ev("start", "clench"), "clench", 0.1)
        m.step([], "clench", 1.3)
        self.assertEqual(m.strength, 10)

    def test_prev_role(self):
        m = self.make(primary="clench", prev="focus")
        m.step(ev("end", "focus", 0.3), None, 0)
        self.assertEqual(m.mode, "speed_down")

    def test_lock_starts_locked_and_blocks_everything_else(self):
        m = self.make(primary="clench", lock="eyes")
        self.assertTrue(m.locked)
        m.step(ev("end", "clench", 0.3), None, 0)
        self.assertEqual(m.mode, "forward")
        self.assertEqual(m.step([], "clench", 2.0).command, "hover")
        # hold lock to unlock (toggles once per hold)
        m.step(ev("start", "eyes"), "eyes", 0.1)
        m.step([], "eyes", 1.3)
        m.step([], "eyes", 2.0)
        self.assertFalse(m.locked)
        m.step(ev("end", "eyes", 2.1), None, 0)
        m.step(ev("start", "clench"), "clench", 0.1)
        self.assertEqual(m.step([], "clench", 1.3).command, "forward")

    def test_requires_primary(self):
        with self.assertRaises(ValueError):
            ModeController({"lock": "eyes"}, ["forward"])


class EventDetectorTests(unittest.TestCase):
    def feed(self, det, labels, dt=0.1):
        out = []
        for i, lab in enumerate(labels):
            out += det.update(lab, i * dt)
        return out

    def test_single_frame_blip_is_ignored(self):
        events = self.feed(EventDetector(2, 2), [REST, "clench", REST, REST, REST])
        self.assertEqual(events, [])

    def test_start_end_and_duration(self):
        labels = [REST] + ["clench"] * 5 + [REST] * 3
        events = self.feed(EventDetector(2, 2), labels)
        self.assertEqual([(e.kind, e.label) for e in events],
                         [("start", "clench"), ("end", "clench")])
        self.assertAlmostEqual(events[1].duration, 0.6, places=6)

    def test_single_dropout_does_not_end_pattern(self):
        labels = ["clench"] * 4 + [REST] + ["clench"] * 4
        events = self.feed(EventDetector(2, 2), labels)
        self.assertEqual([e.kind for e in events], ["start"])


class ClassifierTests(unittest.TestCase):
    def test_separates_clusters_and_rejects_outliers(self):
        rng = np.random.default_rng(0)
        X = np.concatenate([rng.normal(0, 1, (200, 9)), rng.normal(4, 1, (200, 9))])
        y = np.array([REST] * 200 + ["clench"] * 200)
        clf = Classifier.fit(X, y)
        self.assertEqual(clf.predict(np.full(9, 4.0))[0], "clench")
        self.assertEqual(clf.predict(np.zeros(9))[0], REST)
        # far beyond anything seen in training -> treated as rest, never a command
        self.assertEqual(clf.predict(np.full(9, 40.0))[0], REST)
        again = Classifier.from_dict(clf.to_dict())
        self.assertEqual(again.predict(np.full(9, 4.0))[0], "clench")


class ProfileTests(unittest.TestCase):
    def test_save_load_roundtrip(self):
        p = Profile("x", fs=125, channels=3, bands=["alpha"], baseline_mean=[1.0],
                    baseline_std=[0.5], roles={"primary": "clench"})
        with tempfile.TemporaryDirectory() as d:
            path = p.save(pathlib.Path(d) / "x.json")
            q = Profile.load(path)
        self.assertEqual(q, p)

    def test_stream_mismatch_raises(self):
        p = Profile("x", fs=125, channels=3, bands=list(st.BANDS), window_sec=0.5, update_sec=0.1)
        p.check_stream(125, 3, st.BANDS, 0.5, 0.1)
        with self.assertRaises(ValueError):
            p.check_stream(250, 3, st.BANDS, 0.5, 0.1)


class EndToEndSimTests(unittest.TestCase):
    """train -> assign -> fly on the simulator, faster than real time."""

    @classmethod
    def setUpClass(cls):
        cls.brain = SimBrain(seed=1)
        cls.frames = st.OfflineStream(cls.brain).frames()
        cls.profile = Profile("sim")
        cue = lambda state, msg: setattr(cls.brain, "state", state)  # noqa: E731
        training.record_baseline(cls.frames, cls.profile, 20, cue)
        assignment.record_patterns(cls.frames, cls.profile, {"primary": "clench"}, cue,
                                   trials=4, seed=0)
        st.collect(cls.frames, 1.0)   # flush the last action out of the window

    def run_state(self, pilot, state, seconds):
        self.brain.state = state
        start, ticks = None, []
        for fr in self.frames:
            start = fr.t if start is None else start
            ticks.append(pilot.step(fr))
            if fr.t - start >= seconds:
                return ticks

    def test_profile_is_reliable(self):
        self.assertTrue(self.profile.has_patterns)
        for label, acc in self.profile.accuracy.items():
            self.assertGreaterEqual(acc, assignment.MIN_ACCURACY, label)

    def test_rest_tap_hold(self):
        from hardwaretodrone import Drone
        with Drone(dry_run=True) as drone:
            pilot = Pilot(self.profile, drone)
            ticks = self.run_state(pilot, "rest", 3)
            self.assertTrue(all(t.out.command == "hover" for t in ticks))
            self.assertEqual(pilot.modes.mode, "forward")

            self.run_state(pilot, "clench", 0.3)          # tap
            self.run_state(pilot, "rest", 1.5)
            self.assertEqual(pilot.modes.mode, "turn_left")

            ticks = self.run_state(pilot, "clench", 2.5)  # hold
            self.assertEqual(ticks[-1].out.command, "turn_left")
            time.sleep(0.2)                               # let the output thread slew
            self.assertLess(drone.dac.codes[2], 2048)     # yaw (channel C) pushed left

            ticks = self.run_state(pilot, "rest", 1.0)
            self.assertEqual(ticks[-1].out.command, "hover")


if __name__ == "__main__":
    unittest.main()
