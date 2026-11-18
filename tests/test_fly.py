# Claude generated test-cases.

import pathlib
import sys
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

from processing import fly


class FakeDrone:
    def __init__(self):
        self.calls = []

    def command(self, name, strength=None):
        self.calls.append((name, strength))

    def hover(self):
        self.calls.append(("hover", None))


def frame(alpha):
    """One band-power update: 3 electrodes, columns alpha/beta/gamma."""
    return None, np.array([[alpha, 0.3, 0.1]] * 3)


class CalibrateTests(unittest.TestCase):
    def test_thresholds_sit_between_relax_and_action(self):
        low, high = fly.calibrate([0.2] * 8, [0.8] * 8)
        self.assertTrue(0.2 < low < 0.5 < high < 0.8)

    def test_refuses_when_band_does_not_rise(self):
        with self.assertRaises(fly.CalibrationError):
            fly.calibrate([0.30] * 8, [0.35] * 8)

    def test_refuses_when_band_goes_down(self):
        with self.assertRaises(fly.CalibrationError):
            fly.calibrate([0.8] * 8, [0.2] * 8)

    def test_refuses_with_too_few_readings(self):
        with self.assertRaises(fly.CalibrationError):
            fly.calibrate([0.2], [0.8])


class DeciderTests(unittest.TestCase):
    def test_starts_hover_then_confirms_after_two_updates(self):
        d = fly.Decider(0.4, 0.6)
        self.assertEqual(d.update(0.9), "hover")      # 1st sighting: not confirmed
        self.assertEqual(d.update(0.9), "forward")    # 2nd: confirmed
        self.assertEqual(d.update(0.1), "forward")
        self.assertEqual(d.update(0.1), "turn_right")

    def test_single_blip_is_ignored(self):
        d = fly.Decider(0.4, 0.6)
        d.update(0.9); d.update(0.9)
        for score in (0.1, 0.9, 0.9, 0.9):
            d.update(score)
        self.assertEqual(d.command, "forward")

    def test_dead_band_keeps_current_command(self):
        d = fly.Decider(0.4, 0.6)
        d.update(0.9); d.update(0.9)
        for _ in range(5):
            d.update(0.5)
        self.assertEqual(d.command, "forward")

    def test_dead_band_from_hover_stays_hover(self):
        d = fly.Decider(0.4, 0.6)
        for _ in range(5):
            d.update(0.5)
        self.assertEqual(d.command, "hover")


class KeepaliveTests(unittest.TestCase):
    def test_resends_command_and_falls_back_to_hover_when_stale(self):
        drone = FakeDrone()
        ka = fly.Keepalive(drone, 30, stale=0.3, period=0.02)
        ka.set("forward")
        time.sleep(0.15)
        self.assertIn(("forward", 30), drone.calls)
        n = len(drone.calls)
        self.assertGreater(n, 3)                 # sent repeatedly, not once
        time.sleep(0.5)                          # brain loop stalls
        self.assertEqual(drone.calls[-1], ("hover", 30))
        ka.stop()
        self.assertEqual(drone.calls[-1], ("hover", None))


class FlyLoopTests(unittest.TestCase):
    def test_signal_above_is_forward_otherwise_turn_right(self):
        drone = FakeDrone()
        ka = fly.Keepalive(drone, 30, period=0.01)
        frames = iter([frame(0.9)] * 4 + [frame(0.1)] * 4)
        issued = fly.fly(frames, fly.Decider(0.4, 0.6), ka, "alpha", show=lambda _: None)
        ka.stop()
        self.assertEqual(issued[:2], ["hover", "forward"])
        self.assertEqual(issued[3], "forward")
        self.assertEqual(issued[-1], "turn_right")
        self.assertNotIn("backward", issued)

    def test_calibrate_then_fly_end_to_end(self):
        relax = [fly.band_score(frame(0.2)[1], "alpha")] * 8
        action = [fly.band_score(frame(0.8)[1], "alpha")] * 8
        low, high = fly.calibrate(relax, action)
        drone = FakeDrone()
        ka = fly.Keepalive(drone, 30, period=0.01)
        issued = fly.fly(iter([frame(0.8)] * 3 + [frame(0.2)] * 3),
                         fly.Decider(low, high), ka, "alpha", show=lambda _: None)
        ka.stop()
        self.assertEqual(issued[-1], "turn_right")
        self.assertIn("forward", issued)


class ChannelTests(unittest.TestCase):
    def test_default_is_one_channel(self):
        self.assertEqual(fly.parse_args(["--sim"]).channels, 1)

    def test_band_score_works_with_a_single_channel(self):
        self.assertAlmostEqual(fly.band_score(np.array([[0.7, 0.2, 0.1]]), "alpha"), 0.7)

    def test_sim_returns_one_value_per_connected_channel(self):
        old = fly.pipe.CHANNELS
        try:
            fly.pipe.CHANNELS = 1
            self.assertEqual(len(fly.ScriptedSim("alpha").read()), 1)
        finally:
            fly.pipe.CHANNELS = old


class ArgsTests(unittest.TestCase):
    def test_strength_is_capped(self):
        with self.assertRaises(SystemExit):
            fly.parse_args(["--sim", "--strength", "80"])

    def test_sim_and_real_cannot_combine(self):
        with self.assertRaises(SystemExit):
            fly.parse_args(["--sim", "--real"])

    def test_real_refuses_without_typed_confirmation(self):
        args = fly.parse_args(["--real"])
        with self.assertRaises(SystemExit):
            fly.confirm_real(args)               # stdin isn't a tty under test


if __name__ == "__main__":
    unittest.main()
