"""Tests for processing/drone_map.py (Option C drone mapping).

Stdlib only, no hardware: a FakeDrone records every set() call, and
results are fed to DroneMapper directly as channel_active dicts, or
through a real SignalProcessor fed with fake samples.
"""

import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from processing.drone_map import DroneMapper  # noqa: E402
from processing.processor import SignalProcessor  # noqa: E402


def _result(alpha=0, beta=0, gamma=0):
    return {"channel_active": {"alpha": alpha, "beta": beta, "gamma": gamma}}


class FakeDrone:
    def __init__(self):
        self.set_calls = []

    def set(self, pitch=None, roll=None, yaw=None, throttle=None):
        self.set_calls.append(
            {"pitch": pitch, "roll": roll, "yaw": yaw, "throttle": throttle})


class MapperTests(unittest.TestCase):
    def test_alpha_up_steps_pitch_and_caps_at_max_speed(self):
        m = DroneMapper()
        self.assertEqual(m.update(_result(alpha=1))["pitch"], 0.25)
        self.assertEqual(m.update(_result(alpha=1))["pitch"], 0.5)
        # Held active: stays capped at MAX_SPEED, never above it.
        self.assertEqual(m.update(_result(alpha=1))["pitch"], 0.5)
        self.assertEqual(m.update(_result(alpha=1))["pitch"], 0.5)

    def test_alpha_down_steps_pitch_negative(self):
        m = DroneMapper()
        self.assertEqual(m.update(_result(alpha=-1))["pitch"], -0.25)
        self.assertEqual(m.update(_result(alpha=-1))["pitch"], -0.5)

    def test_beta_and_gamma_together_move_two_axes_in_one_call(self):
        drone = FakeDrone()
        m = DroneMapper()
        axes = m.drive(drone, _result(beta=1, gamma=1))
        self.assertEqual(axes["roll"], 0.25)
        self.assertEqual(axes["throttle"], 0.25)
        self.assertEqual(axes["pitch"], 0.0)
        self.assertEqual(len(drone.set_calls), 1)
        self.assertEqual(drone.set_calls[0],
                         {"pitch": 0.0, "roll": 0.25, "yaw": 0.0,
                          "throttle": 0.25})

    def test_dead_zone_decays_velocity_to_hover(self):
        m = DroneMapper()
        m.update(_result(alpha=1))
        m.update(_result(alpha=1))  # pitch = 0.5
        self.assertEqual(m.update(_result())["pitch"], 0.25)
        self.assertEqual(m.update(_result())["pitch"], 0.0)
        self.assertEqual(m.update(_result())["pitch"], 0.0)

    def test_none_on_all_channels_is_hover(self):
        m = DroneMapper()
        axes = m.update(_result())
        self.assertEqual(axes, {"pitch": 0.0, "roll": 0.0,
                                "yaw": 0.0, "throttle": 0.0})
        axes = m.update(None)
        self.assertEqual(axes, {"pitch": 0.0, "roll": 0.0,
                                "yaw": 0.0, "throttle": 0.0})

    def test_yaw_always_zero(self):
        m = DroneMapper()
        for _ in range(3):
            axes = m.update(_result(alpha=1, beta=1, gamma=1))
            self.assertEqual(axes["yaw"], 0.0)

    def test_set_called_every_window_even_when_unchanged(self):
        # Watchdog: the drone must hear set() every cycle, even at hover.
        drone = FakeDrone()
        m = DroneMapper()
        for _ in range(4):
            m.drive(drone, _result())
        self.assertEqual(len(drone.set_calls), 4)
        self.assertTrue(all(c == {"pitch": 0.0, "roll": 0.0, "yaw": 0.0,
                                  "throttle": 0.0} for c in drone.set_calls))

    def test_velocity_never_exceeds_max_speed_on_any_axis(self):
        m = DroneMapper()
        for _ in range(10):
            axes = m.update(_result(alpha=1, beta=-1, gamma=1))
        for axis, value in axes.items():
            self.assertLessEqual(abs(value), 0.5)

    def test_hover_resets_and_centres_drone(self):
        drone = FakeDrone()
        m = DroneMapper()
        m.update(_result(alpha=1, gamma=1))
        axes = m.hover(drone)
        self.assertEqual(axes, {"pitch": 0.0, "roll": 0.0,
                                "yaw": 0.0, "throttle": 0.0})
        self.assertEqual(drone.set_calls[-1],
                         {"pitch": 0.0, "roll": 0.0, "yaw": 0.0,
                          "throttle": 0.0})

    def test_processor_alpha_spike_drives_pitch_after_debounce(self):
        # End to end: processor debounces the channel (3 windows),
        # then the mapper steps pitch. Calm baseline first.
        p = SignalProcessor(window_size=5, calibrate_samples=5, debounce=3)
        m = DroneMapper()
        for i in range(5):
            p.add_sample((1000.0 + i * 0.1, 0.12, -0.02, 0.33))
        axes = None
        for i in range(5):
            r = p.add_sample((1000.5 + i * 0.1, 0.22, -0.02, 0.33))
            axes = m.update(r)
        # 3 debounced windows of alpha +1, then 2 more: pitch steps
        # 0.25 -> 0.5 and stays capped at MAX_SPEED.
        self.assertEqual(axes["pitch"], 0.5)
        self.assertEqual(axes["roll"], 0.0)
        self.assertEqual(axes["throttle"], 0.0)


if __name__ == "__main__":
    unittest.main()
