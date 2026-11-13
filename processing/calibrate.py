"""Calibrate SignalProcessor thresholds from a labelled session.

The default thresholds in ``processing/processor.py`` (1.0 z for
focus / relax / gamma / active_channel, 0.25 z margin) are untuned
starting points. This tool turns a short labelled recording into
suggested replacements, so the labelled focus-then-relax session
that tuning needs is actually usable.

Input: a CSV with columns ``timestamp,alpha,beta,gamma,label`` where
``label`` is one of ``baseline``, ``focus``, ``relax``, ``active`` or
``neutral``. Record it in segments, e.g. 30 s neutral baseline, then
30 s deliberately focusing, then 30 s deliberately relaxing. The
``baseline`` rows set the processor baseline (they should be neutral
resting data); every other row is fed through the processor and its
scores are grouped by label.

    python -m processing.calibrate --csv labelled.csv
    python -m processing.calibrate --csv labelled.csv --output thresholds.json

The report prints, per label, the mean focus / relax / activity
scores and mean z-scores, whether the labels separate, and the
suggested threshold for each knob. Suggestions are midpoints between
label means, floored at 0.5 z so a weakly separating session does
not suggest a hair-trigger. Nothing is changed automatically: pass
the suggested values to ``SignalProcessor(...)`` yourself, or write
them to JSON with ``--output``.
"""

import argparse
import csv
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from processing.processor import SignalProcessor  # noqa: E402

LABELS = ("baseline", "focus", "relax", "active", "neutral")
SCORES = ("focus_score", "relax_score", "activity_score")
CHANNELS = ("alpha", "beta", "gamma")


def load_labelled_csv(path):
    rows = []
    with open(path, newline="") as fh:
        reader = csv.DictReader(fh)
        required = {"timestamp", "alpha", "beta", "gamma", "label"}
        if reader.fieldnames is None or not required.issubset(
            {f.strip() for f in reader.fieldnames}
        ):
            raise ValueError(
                f"{path}: expected columns timestamp,alpha,beta,gamma,label"
            )
        for line, row in enumerate(reader, start=2):
            label = (row.get("label") or "").strip().lower()
            if label not in LABELS:
                raise ValueError(f"{path}:{line}: unknown label {label!r}")
            try:
                rows.append((
                    float(row["timestamp"]),
                    float(row["alpha"]),
                    float(row["beta"]),
                    float(row["gamma"]),
                    label,
                ))
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{path}:{line}: bad numeric value") from exc
    return rows


def _mean(values):
    return sum(values) / len(values) if values else 0.0


def analyse(rows, window_size=10):
    baseline_rows = [r[:4] for r in rows if r[4] == "baseline"]
    if len(baseline_rows) < 5:
        raise ValueError(
            "need at least 5 baseline rows to set the calibration baseline"
        )
    processor = SignalProcessor(window_size=window_size)
    processor.calibrate(baseline_rows)
    grouped = {}
    for row in rows:
        if row[4] == "baseline":
            continue
        result = processor.add_sample(row[:4])
        if result is not None:
            grouped.setdefault(row[4], []).append(result)
    if not grouped:
        raise ValueError("no labelled rows were processed")
    summary = {}
    for label, results in grouped.items():
        summary[label] = {
            "n": len(results),
            **{s: _mean([r[s] for r in results]) for s in SCORES},
            "z": {ch: _mean([r["z"][ch] for r in results]) for ch in CHANNELS},
            "min_focus": min(r["focus_score"] for r in results),
            "max_focus": max(r["focus_score"] for r in results),
        }
    return summary


def _midpoint(a, b, floor=0.5):
    return round(max(floor, (a + b) / 2.0), 2)


def suggest_thresholds(summary):
    out = {}
    if "focus" in summary and "relax" in summary:
        out["focus_threshold"] = _midpoint(
            summary["focus"]["focus_score"], summary["relax"]["focus_score"]
        )
        out["relax_threshold"] = _midpoint(
            summary["relax"]["relax_score"], summary["focus"]["relax_score"]
        )
        out["separates"] = (
            summary["focus"]["min_focus"] > summary["relax"]["max_focus"]
        )
    if "active" in summary:
        others = [s["activity_score"] for k, s in summary.items() if k != "active"]
        if others:
            out["gamma_threshold"] = _midpoint(
                summary["active"]["activity_score"], _mean(others)
            )
    channel_for = {"focus": "beta", "relax": "alpha", "active": "gamma"}
    peaks = []
    for label, channel in channel_for.items():
        if label in summary:
            peaks.append(summary[label]["z"][channel])
    if peaks:
        out["active_threshold"] = _midpoint(_mean(peaks), 0.0)
        out["active_margin"] = 0.25
    return out


def format_report(summary, suggested):
    lines = ["Labelled session summary (mean scores):", ""]
    header = f"{'label':<9} {'n':>4} {'focus':>7} {'relax':>7} {'active':>7}  z(alpha,beta,gamma)"
    lines.append(header)
    for label in LABELS:
        if label not in summary:
            continue
        s = summary[label]
        z = s["z"]
        lines.append(
            f"{label:<9} {s['n']:>4} {s['focus_score']:>+7.2f} "
            f"{s['relax_score']:>+7.2f} {s['activity_score']:>+7.2f}  "
            f"({z['alpha']:+.2f},{z['beta']:+.2f},{z['gamma']:+.2f})"
        )
    lines.append("")
    if "separates" in suggested:
        lines.append(
            "Focus vs relax separation: "
            + ("clean (every focus window beat every relax window)"
               if suggested["separates"] else
               "overlapping -- thresholds below are a weak fit, record more data")
        )
    if suggested:
        lines.append("Suggested thresholds (starting points, not fitted constants):")
        for key in ("focus_threshold", "relax_threshold", "gamma_threshold",
                    "active_threshold", "active_margin"):
            if key in suggested:
                lines.append(f"  {key} = {suggested[key]}")
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", required=True, help="Labelled session CSV")
    parser.add_argument("--window", type=int, default=10)
    parser.add_argument("--output", help="Also write suggested thresholds as JSON")
    args = parser.parse_args(argv)
    rows = load_labelled_csv(args.csv)
    summary = analyse(rows, window_size=args.window)
    suggested = suggest_thresholds(summary)
    print(format_report(summary, suggested))
    if args.output:
        with open(args.output, "w") as fh:
            json.dump(suggested, fh, indent=2)
        print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
