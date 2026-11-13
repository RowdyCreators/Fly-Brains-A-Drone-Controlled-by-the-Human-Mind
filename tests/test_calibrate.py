import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from processing.calibrate import (  # noqa: E402
    analyse,
    load_labelled_csv,
    suggest_thresholds,
)


def _rows():
    rows = []
    t = 0.0
    for _ in range(10):
        rows.append((t, 0.10, 0.10, 0.10, "baseline"))
        t += 0.1
    for _ in range(20):
        rows.append((t, 0.05, 0.30, 0.10, "focus"))
        t += 0.1
    for _ in range(20):
        rows.append((t, 0.30, 0.05, 0.10, "relax"))
        t += 0.1
    for _ in range(20):
        rows.append((t, 0.10, 0.10, 0.35, "active"))
        t += 0.1
    return rows


class CalibrateTests(unittest.TestCase):
    def test_analyse_groups_by_label(self):
        summary = analyse(_rows())
        self.assertIn("focus", summary)
        self.assertIn("relax", summary)
        self.assertGreater(
            summary["focus"]["focus_score"], summary["relax"]["focus_score"]
        )
        self.assertGreater(
            summary["relax"]["relax_score"], summary["focus"]["relax_score"]
        )

    def test_suggested_thresholds_separate(self):
        suggested = suggest_thresholds(analyse(_rows()))
        self.assertTrue(suggested["separates"])
        self.assertGreater(suggested["focus_threshold"], 0.0)
        self.assertGreater(suggested["relax_threshold"], 0.0)
        self.assertIn("gamma_threshold", suggested)
        self.assertIn("active_threshold", suggested)

    def test_load_labelled_csv_roundtrip(self):
        with tempfile.NamedTemporaryFile(
            "w", suffix=".csv", delete=False, newline=""
        ) as fh:
            fh.write("timestamp,alpha,beta,gamma,label\n")
            for row in _rows()[:5]:
                fh.write(",".join(str(v) for v in row) + "\n")
            path = fh.name
        try:
            rows = load_labelled_csv(path)
            self.assertEqual(len(rows), 5)
            self.assertEqual(rows[0][4], "baseline")
        finally:
            os.unlink(path)


if __name__ == "__main__":
    unittest.main()
