"""Processing entry point for the brain-signal controller.

Replaces the previous version of this file, which could not run at all:
it imported ``FlyBrian`` from ``signals.signal`` (that class does not
exist -- the class in ``signals/signal.py`` on ``Brain_sim_Test`` is
``FlyBrain``) and called ``brain.main(number_of_samples=...)`` (that
method does not exist either).

This version uses the real API:

* ``FlyBrain`` + ``FlyBrain.listen(source, on_sample=...)`` from
  ``signals/signal.py``
* ``SignalProcessor`` from ``processing/processor.py``, fed one sample
  at a time from the ``on_sample`` callback

Usage, from the repo root:

    python -m processing.main --csv signals/samples/samples.csv
    python -m processing.main --samples 20            # live ADS1115, else simulator
    python -m processing.main --samples 20 --output processed.csv

Live channel mode -- output just which signal is activated, live:

    python -m processing.main --mode channel --samples 30
    python -m processing.main --mode channel --csv signals/samples/samples.csv

That prints one line per window, e.g. ``t=...  alpha`` when alpha is
the activated channel, ``beta`` / ``gamma`` likewise, or ``none``
when no channel clearly stands out. Activation means: highest
z-score against the calibration baseline, at least 1.0 z, and at
least 0.25 z ahead of the next channel, held for 3 windows in a row.

Map mode -- Option C drone mapping (see drone-mapping-plan.md):

    python -m processing.main --mode map --samples 30
    python -m processing.main --mode map --csv signals/samples/samples.csv

One channel = one signed axis: alpha -> pitch (forward/back),
beta -> roll (right/left), gamma -> throttle (up/down), yaw = 0.
Each debounced window adds one +1/-1 step of 0.25 to that axis
(max 0.5 stick); dead zone decays it back toward hover. Prints one
line per window, e.g. ``t=...  pitch=+0.25 roll=+0.00
throttle=+0.00 yaw=+0.00``. With ``--drone-dry-run`` the axes are
sent to a dry-run Drone via ``drone.set(...)`` every window.

Live mode note: ``signals.signal.get_source()`` falls back to
``from brain_sim import simulate_brain``, which only works if the
``signals/`` folder is on ``sys.path``. If that import fails here, we
fall back to ``signals.brain_sim.simulate_brain`` directly, so the
simulator still runs. The hardware path is unchanged.

Drone wiring (optional, ``--drone-dry-run``): if ``hardwaretodrone``
is importable (it lives on ``main``, not on ``Brain_sim_Test``), a
``Drone(dry_run=True)`` is created and every processed result is sent
to it with ``drone.command(result["command"])`` -- every cycle, even
when the command has not changed, because the Drone's 0.5 s watchdog
returns the sticks to hover otherwise. Without that flag, or without
the package, this just prints states/commands and does not touch a
drone. Real (non-dry-run) driving is deliberately not offered here:
takeoff/landing are done by hand with the remote's physical button.
"""

import argparse
import os
import sys

# Allow running as `python processing/main.py` as well as `-m processing.main`.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from processing.drone_map import DroneMapper  # noqa: E402
from processing.processor import SignalProcessor, drive_drone  # noqa: E402
from signals.signal import FlyBrain, get_source  # noqa: E402


def _live_source(rate_hz, max_samples):
    """ADS1115 if present, else the brain simulator (with import fallback)."""
    try:
        return get_source(rate_hz=rate_hz, max_samples=max_samples)
    except (ImportError, ModuleNotFoundError):
        from signals.brain_sim import simulate_brain

        print("No ADS1115 found, using the brain simulator.")
        return simulate_brain(rate_hz=rate_hz, max_samples=max_samples)


def _show(result, mode="command"):
    if result is None:
        return
    if mode == "channel-only":
        print(result["active_channel"])
        return
    if mode == "map":
        axes = result.get("axes") or {}
        print(f"t={result['timestamp']:.3f}  "
              f"pitch={axes.get('pitch', 0.0):+.2f} "
              f"roll={axes.get('roll', 0.0):+.2f} "
              f"throttle={axes.get('throttle', 0.0):+.2f} "
              f"yaw={axes.get('yaw', 0.0):+.2f}")
        return
    if mode == "channel":
        # Live output: just the activated channel, e.g. "alpha".
        z = result["z"]
        print(f"t={result['timestamp']:.3f}  {result['active_channel']}  "
              f"(z alpha={z['alpha']:+.2f} beta={z['beta']:+.2f} gamma={z['gamma']:+.2f})")
        return
    z = result["z"]
    print(
        f"t={result['timestamp']:.3f}  "
        f"alpha={result['smoothed'][0]:.3f} beta={result['smoothed'][1]:.3f} "
        f"gamma={result['smoothed'][2]:.3f}  "
        f"z=({z['alpha']:+.2f},{z['beta']:+.2f},{z['gamma']:+.2f})  "
        f"focus={result['focus_score']:+.2f} relax={result['relax_score']:+.2f}  "
        f"state={result['state']}  command={result['command']}"
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", help="Process a samples.csv file instead of live/sim data")
    parser.add_argument("--samples", type=int, default=20, help="Live/sim samples to collect")
    parser.add_argument("--rate", type=int, default=10, help="Samples per second (live/sim)")
    parser.add_argument("--window", type=int, default=10, help="Processor window size")
    parser.add_argument("--output", help="Write processed rows to this CSV file")
    parser.add_argument("--mode", choices=("command", "channel", "channel-only", "map"), default="command",
                        help="command: print state/command (default); "
                             "channel: print the activated channel with z-scores, live; "
                             "channel-only: print only the activated channel name, one per line; "
                             "map: print the Option C drone axes (pitch/roll/throttle/yaw) per window")
    parser.add_argument("--drone-dry-run", action="store_true",
                        help="Also send each command to a dry-run Drone (FakeDAC, no hardware)")
    args = parser.parse_args(argv)

    processor = SignalProcessor(window_size=args.window)
    mapper = DroneMapper() if args.mode == "map" else None
    drone = None
    if args.drone_dry_run:
        try:
            from hardwaretodrone import Drone

            drone = Drone(dry_run=True)
            print("Drone dry-run enabled (FakeDAC, no hardware).")
        except ImportError:
            print("hardwaretodrone is not importable here "
                  "(it lives on main, not Brain_sim_Test) -- "
                  "continuing without a drone.")
    try:
        return _run(args, processor, drone, mapper)
    finally:
        if drone is not None:
            drone.close()


def _send(args, mapper, drone, result):
    """Route one processed result to the mapper / drone for this mode."""
    if result is None:
        return
    if mapper is not None:
        if drone is not None:
            result["axes"] = mapper.drive(drone, result)
        else:
            result["axes"] = mapper.update(result)
    elif drone is not None:
        drive_drone(drone, result)


def _run(args, processor, drone, mapper=None):

    if args.csv:
        results = processor.process_csv(args.csv, output_path=args.output)
        for result in results:
            _send(args, mapper, drone, result)
            _show(result, args.mode)
        print(f"Processed {len(results)} windows from {args.csv}"
              + (f", wrote {args.output}" if args.output else ""))
        return results

    brain = FlyBrain()
    results = []

    def on_sample(sample):
        result = processor.add_sample(sample)
        if result is not None:
            results.append(result)
            _send(args, mapper, drone, result)
        _show(result, args.mode)

    received = brain.listen(
        _live_source(rate_hz=args.rate, max_samples=args.samples),
        on_sample=on_sample,
    )
    print(f"Received {len(received)} samples, "
          f"final state={processor.state}, command={processor.command}, "
          f"active_channel={processor.active_channel}"
          + (f", axes={mapper.velocity}" if mapper is not None else ""))
    if args.output and results:
        processor.write_csv(results, args.output)
        print(f"Wrote {args.output}")
    return received


if __name__ == "__main__":
    main()
