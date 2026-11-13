import importlib.util
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "Hardware"))

from signals import brain_simulator
from signals.signal import FlyBrain


def load_by_path(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class FlyBrainTests(unittest.TestCase):
    def test_receive_signals_stores_sample(self):
        brain = FlyBrain(buffer_size=2)
        sample = brain.receive_signals(1, 2.5, 3, timestamp=10.0)
        self.assertEqual(sample, (10.0, 1.0, 2.5, 3.0))
        self.assertEqual(brain.latest, sample)

    def test_receive_signals_rejects_non_numbers(self):
        with self.assertRaises(TypeError):
            FlyBrain().receive_signals("x", 1, 1)

    def test_buffer_is_rolling(self):
        brain = FlyBrain(buffer_size=3)
        for i in range(5):
            brain.receive_signals(i, i, i)
        self.assertEqual(len(brain.buffer), 3)

    def test_listen_with_simulator(self):
        brain = FlyBrain()
        source = brain_simulator.simulate_brain(rate_hz=1000, max_samples=5)
        received = brain.listen(source)
        self.assertEqual(len(received), 5)
        self.assertEqual(len(received[0]), 4)  # timestamp, alpha, beta, gamma


class EEGPipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import numpy  # noqa: F401
            import scipy  # noqa: F401
        except ImportError:
            raise unittest.SkipTest("numpy/scipy not installed")
        cls.pipe = load_by_path("eeg_pipeline", ROOT / "signals" / "eeg_pipeline.py")

    def test_alpha_dominates_band_powers_for_10hz_sine(self):
        import numpy as np
        fs = self.pipe.FS
        t = np.arange(0, 4, 1 / fs)
        window = np.tile(np.sin(2 * np.pi * 10 * t), (3, 1))
        _, rel = self.pipe.band_powers(window, fs)
        self.assertEqual(rel.shape, (3, 3))
        self.assertTrue((rel[:, 0] > 0.9).all())

    def test_cleaner_removes_dc_offset_and_mains(self):
        import numpy as np
        fs = self.pipe.FS
        t = np.arange(0, 6, 1 / fs)
        raw = np.tile(1.65 + 0.05 * np.sin(2 * np.pi * self.pipe.MAINS_HZ * t), (3, 1))
        clean = self.pipe.StreamCleaner(fs, 3).process(raw)
        self.assertLess(np.abs(clean[:, -fs:]).max(), 0.01)

    def test_cleaner_rejects_fs_below_mains_nyquist(self):
        with self.assertRaises(ValueError):
            self.pipe.StreamCleaner(100, 3)

    def test_sim_source_returns_one_value_per_channel(self):
        values = self.pipe.SimSource().read()
        self.assertEqual(len(values), self.pipe.CHANNELS)


class BrainToDroneTests(unittest.TestCase):
    def test_command_moves_dac_and_watchdog_centers(self):
        import time
        from hardwaretodrone import Drone
        with Drone(dry_run=True, strength=30, watchdog=0.2) as drone:
            for _ in range(10):
                drone.command("turn_right")
                time.sleep(0.05)
            self.assertGreater(drone.dac.codes[2], 2048)   # yaw is channel C
            time.sleep(0.6)
            self.assertTrue(drone.failsafe_active)
            self.assertEqual(drone.dac.codes, [2048] * 4)


if __name__ == "__main__":
    unittest.main()
