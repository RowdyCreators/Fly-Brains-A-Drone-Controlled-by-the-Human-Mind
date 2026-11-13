"""Tests for processing/live_bridge.py -- signals -> processing, live."""

import unittest

from processing.drone_map import DroneMapper
from processing.live_bridge import (
    PipelineBridge,
    band_powers_to_sample,
    stream_band_powers_to_processor,
    stream_to_processor,
)
from processing.processor import SignalProcessor


def _calm(n=5):
    return [(0.12, -0.02, 0.33)] * n


class FakeDrone:
    def __init__(self):
        self.commands = []
        self.sets = []

    def command(self, name):
        self.commands.append(name)

    def set(self, pitch, roll, yaw, throttle):
        self.sets.append((pitch, roll, yaw, throttle))


class BridgeTests(unittest.TestCase):
    def test_raw_triples_stream_yields_results_after_calibration(self):
        samples = _calm(5) + [(0.22, -0.02, 0.33)] * 10
        results = list(stream_to_processor(samples))
        self.assertEqual(len(results), 10)
        self.assertEqual(results[-1]["state"], "relax")
        self.assertEqual(results[-1]["command"], "hover")

    def test_dict_samples_stream(self):
        samples = [{"alpha": 0.12, "beta": -0.02, "gamma": 0.33}] * 5
        samples += [{"alpha": 0.12, "beta": 0.08, "gamma": 0.33}] * 10
        results = list(stream_to_processor(samples))
        self.assertEqual(len(results), 10)
        self.assertEqual(results[-1]["state"], "focus")

    def test_on_result_called_once_per_result(self):
        seen = []
        samples = _calm(5) + [(0.12, -0.02, 0.43)] * 6
        list(stream_to_processor(samples, on_result=seen.append))
        self.assertEqual(len(seen), 6)

    def test_drone_gets_command_every_window(self):
        drone = FakeDrone()
        samples = _calm(5) + [(0.12, -0.02, 0.33)] * 5
        list(stream_to_processor(samples, drone=drone))
        self.assertEqual(len(drone.commands), 5)

    def test_mapper_axes_attached_and_drone_set_every_window(self):
        drone = FakeDrone()
        mapper = DroneMapper()
        samples = _calm(5) + [(0.22, -0.02, 0.33)] * 10
        results = list(stream_to_processor(samples, mapper=mapper, drone=drone))
        self.assertEqual(len(drone.sets), 10)
        self.assertIn("axes", results[-1])
        self.assertGreater(results[-1]["axes"]["pitch"], 0.0)

    def test_band_powers_to_sample_takes_electrode_row(self):
        rel_p = [[0.05, 0.10, 0.03], [0.02, 0.15, 0.08], [0.08, 0.04, 0.12]]
        sample = band_powers_to_sample(rel_p, electrode=1, timestamp=1.5)
        self.assertEqual(sample, (1.5, 0.02, 0.15, 0.08))

    def test_band_power_stream_feeds_processor(self):
        # rel_p rows: calm baseline, then alpha-heavy.
        calm = (None, [[0.05, 0.10, 0.03]])
        alpha_up = (None, [[0.15, 0.10, 0.03]])
        stream = [calm] * 5 + [alpha_up] * 10
        results = list(stream_band_powers_to_processor(stream))
        self.assertEqual(len(results), 10)
        self.assertEqual(results[-1]["state"], "relax")

    def test_band_power_stream_ignores_abs_p(self):
        # Same rel_p, wildly different abs_p -> identical results.
        stream_a = [(1e-9, [[0.05, 0.10, 0.03]])] * 5
        stream_a += [(1e-9, [[0.15, 0.10, 0.03]])] * 5
        stream_b = [(1e-3, [[0.05, 0.10, 0.03]])] * 5
        stream_b += [(1e-3, [[0.15, 0.10, 0.03]])] * 5
        ra = list(stream_band_powers_to_processor(stream_a))
        rb = list(stream_band_powers_to_processor(stream_b))
        self.assertEqual([r["state"] for r in ra], [r["state"] for r in rb])
        self.assertEqual([r["z"] for r in ra], [r["z"] for r in rb])

    def test_bridge_on_band_powers_none_during_calibration_then_result(self):
        bridge = PipelineBridge()
        rel_calm = [[0.05, 0.10, 0.03]]
        for _ in range(5):
            self.assertIsNone(bridge.on_band_powers(None, rel_calm))
        result = bridge.on_band_powers(None, [[0.05, 0.10, 0.13]])
        self.assertIsNotNone(result)
        self.assertIn("command", result)

    def test_bridge_on_sample_with_mapper_drives_axes(self):
        bridge = PipelineBridge(mapper=DroneMapper())
        drone = FakeDrone()
        for _ in range(5):
            bridge.on_sample((0.12, -0.02, 0.33))
        result = bridge.on_sample((0.22, -0.02, 0.33), drone=drone)
        # First alpha window is still debouncing; axes exist, drone fed.
        self.assertIn("axes", result)
        self.assertEqual(len(drone.sets), 1)

    def test_low_confidence_is_hover_with_zero_axes(self):
        # PR #15, point 5: a low-confidence window (nothing clearly
        # above baseline) must be hover, with all axes at zero -- not
        # a drifted command.
        mapper = DroneMapper()
        samples = [(0.12, -0.02, 0.33)] * 5 + [(0.12, -0.02, 0.33)] * 10
        results = list(stream_to_processor(samples, mapper=mapper))
        self.assertEqual(len(results), 10)
        last = results[-1]
        self.assertEqual(last["state"], "neutral")
        self.assertEqual(last["command"], "hover")
        self.assertEqual(last["axes"],
                         {"pitch": 0.0, "roll": 0.0,
                          "yaw": 0.0, "throttle": 0.0})

    def test_command_matches_axes_in_map_mode(self):
        # PR #15, point 2, end to end: an alpha spike must give
        # command=forward (from the pitch axis), never hover with
        # pitch at +0.50; a gamma spike must give command=up.
        mapper = DroneMapper()
        samples = [(0.12, -0.02, 0.33)] * 5 + [(0.22, -0.02, 0.33)] * 10
        results = list(stream_to_processor(samples, mapper=mapper))
        self.assertGreater(results[-1]["axes"]["pitch"], 0.0)
        self.assertEqual(results[-1]["command"], "forward")
        mapper = DroneMapper()
        samples = [(0.12, -0.02, 0.33)] * 5 + [(0.12, -0.02, 0.43)] * 10
        results = list(stream_to_processor(samples, mapper=mapper))
        self.assertGreater(results[-1]["axes"]["throttle"], 0.0)
        self.assertEqual(results[-1]["command"], "up")

    def test_drone_set_called_every_window_even_at_hover(self):
        # PR #15, point 5 (and the 0.5 s watchdog): drone.set() on
        # every processed window, even when the axes never move.
        drone = FakeDrone()
        mapper = DroneMapper()
        samples = [(0.12, -0.02, 0.33)] * 5 + [(0.12, -0.02, 0.33)] * 10
        results = list(stream_to_processor(samples, mapper=mapper, drone=drone))
        self.assertEqual(len(drone.sets), len(results))
        self.assertEqual(len(drone.sets), 10)
        self.assertTrue(all(s == (0.0, 0.0, 0.0, 0.0) for s in drone.sets))


if __name__ == "__main__":
    unittest.main()
