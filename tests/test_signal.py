import importlib.util
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "signals" / "signal.py"


class FlyBrianSignalTests(unittest.TestCase):
    def test_module_imports_without_hardware_dependencies(self):
        spec = importlib.util.spec_from_file_location("signal_module", MODULE_PATH)
        module = importlib.util.module_from_spec(spec)
        sys.modules["signal_module"] = module
        spec.loader.exec_module(module)

        self.assertTrue(hasattr(module, "FlyBrian"))
        brain = module.FlyBrian(sample_rate=0.0)
        self.assertIsInstance(brain.read_signals(), list)
        self.assertEqual(len(brain.read_signals()), 3)


if __name__ == "__main__":
    unittest.main()
