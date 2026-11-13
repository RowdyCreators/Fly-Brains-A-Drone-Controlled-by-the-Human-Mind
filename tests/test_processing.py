"""Tests for processing/processor.py, against the Brain_sim_Test API.

Uses only the stdlib (unittest), like the rest of the repo, and runs
without hardware: samples are fed directly, or through FlyBrain's
buffer, exactly as signals/signal.py produces them.
"""

import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from processing.processor import SignalProcessor, drive_drone  # noqa: E402
from signals.signal import FlyBrain  # noqa: E402


def _samples(n, alpha=0.12, beta=-0.02, gamma=0.33, start=1000.0):
    return [(start + i * 0.1, alpha, beta, gamma) for i in range(n)]


class FakeDrone:
    def __init__(self):
        self.commands = []

    def command(self, name):
        self.commands.append(name)


class ProcessorTests(unittest.TestCase):
    def test_calibrates_then_returns_features(self):
        p = SignalProcessor(window_size=5, calibrate_samples=5)
        # First 5 samples are calibration only: no output yet.
        for s in _samples(5):
            self.assertIsNone(p.add_sample(s))
        result = p.add_sample((1000.5, 0.12, -0.02, 0.33))
        self.assertIsNotNone(result)
        self.assertEqual(result["n"], 1)
        self.assertEqual(result["state"], "neutral")
        self.assertEqual(result["command"], "hover")
        for ch in ("alpha", "beta", "gamma"):
            self.assertIn(f"{ch}_mean", result)
            self.assertIn(f"{ch}_share", result)

    def test_focus_state_and_command(self):
        # Baseline: calm. Then beta jumps up and alpha drops: focus.
        p = SignalProcessor(window_size=5, calibrate_samples=5, debounce=1)
        for s in _samples(5):
            p.add_sample(s)
        result = None
        for i in range(5):
            result = p.add_sample((1000.5 + i * 0.1, 0.02, 0.08, 0.33))
        self.assertEqual(result["state"], "focus")
        self.assertEqual(result["command"], "forward")

    def test_relax_state_stays_hover(self):
        p = SignalProcessor(window_size=5, calibrate_samples=5, debounce=1)
        for s in _samples(5):
            p.add_sample(s)
        result = None
        for i in range(5):
            result = p.add_sample((1000.5 + i * 0.1, 0.22, -0.08, 0.33))
        self.assertEqual(result["state"], "relax")
        self.assertEqual(result["command"], "hover")

    def test_debounce_holds_command_until_state_repeats(self):
        p = SignalProcessor(window_size=5, calibrate_samples=5, debounce=3)
        for s in _samples(5):
            p.add_sample(s)
        # One focus window only: command must not flip yet.
        p.add_sample((1000.5, 0.02, 0.08, 0.33))
        self.assertEqual(p.command, "hover")
        # Two more focus windows: now it flips.
        p.add_sample((1000.6, 0.02, 0.08, 0.33))
        p.add_sample((1000.7, 0.02, 0.08, 0.33))
        self.assertEqual(p.command, "forward")

    def test_invalid_samples_are_dropped_not_crash(self):
        p = SignalProcessor(window_size=5, calibrate_samples=5)
        for s in _samples(5):
            p.add_sample(s)
        self.assertIsNone(p.add_sample((1000.5, float("nan"), -0.02, 0.33)))
        self.assertIsNone(p.add_sample((1000.5, "not-a-number", -0.02, 0.33)))
        self.assertEqual(p.dropped, 2)

    def test_process_flybrain_buffer(self):
        brain = FlyBrain()
        for s in _samples(10):
            brain.receive_signals(s[1], s[2], s[3], timestamp=s[0])
        p = SignalProcessor(window_size=5, calibrate_samples=5)
        results = p.process_buffer(brain.buffer)
        self.assertEqual(len(results), 5)  # 5 calibration, 5 processed

    def test_process_repo_samples_csv(self):
        p = SignalProcessor(window_size=5, calibrate_samples=5)
        results = p.process_csv(str(ROOT / "signals" / "samples" / "samples.csv"))
        self.assertEqual(len(results), 15)  # 20 rows - 5 calibration
        self.assertTrue(all(r["command"] in ("hover", "forward", "up")
                            for r in results))


    def test_active_channel_alpha_when_alpha_spikes(self):
        p = SignalProcessor(window_size=5, calibrate_samples=5, debounce=1)
        for s in _samples(5):
            p.add_sample(s)
        result = None
        for i in range(5):
            result = p.add_sample((1000.5 + i * 0.1, 0.22, -0.02, 0.33))
        self.assertEqual(result["active_channel"], "alpha")

    def test_active_channel_beta_when_beta_spikes(self):
        p = SignalProcessor(window_size=5, calibrate_samples=5, debounce=1)
        for s in _samples(5):
            p.add_sample(s)
        result = None
        for i in range(5):
            result = p.add_sample((1000.5 + i * 0.1, 0.12, 0.08, 0.33))
        self.assertEqual(result["active_channel"], "beta")

    def test_active_channel_gamma_when_gamma_spikes(self):
        p = SignalProcessor(window_size=5, calibrate_samples=5, debounce=1)
        for s in _samples(5):
            p.add_sample(s)
        result = None
        for i in range(5):
            result = p.add_sample((1000.5 + i * 0.1, 0.12, -0.02, 0.43))
        self.assertEqual(result["active_channel"], "gamma")

    def test_active_channel_none_when_nothing_stands_out(self):
        p = SignalProcessor(window_size=5, calibrate_samples=5, debounce=1)
        for s in _samples(5):
            p.add_sample(s)
        result = p.add_sample((1000.5, 0.12, -0.02, 0.33))
        self.assertEqual(result["active_channel"], "none")

    def test_active_channel_debounced(self):
        p = SignalProcessor(window_size=5, calibrate_samples=5, debounce=3)
        for s in _samples(5):
            p.add_sample(s)
        p.add_sample((1000.5, 0.22, -0.02, 0.33))
        self.assertEqual(p.active_channel, "none")
        p.add_sample((1000.6, 0.22, -0.02, 0.33))
        p.add_sample((1000.7, 0.22, -0.02, 0.33))
        self.assertEqual(p.active_channel, "alpha")

    def test_drive_drone_calls_command_every_time(self):
        drone = FakeDrone()
        p = SignalProcessor(window_size=5, calibrate_samples=5, debounce=1)
        for s in _samples(5):
            p.add_sample(s)
        for i in range(3):
            drive_drone(drone, p.add_sample((1000.5 + i * 0.1, 0.02, 0.08, 0.33)))
        # Even though the command is the same each time, it is sent
        # every cycle -- the Drone watchdog needs that.
        self.assertEqual(drone.commands, ["forward", "forward", "forward"])


if __name__ == "__main__":
    unittest.main()
