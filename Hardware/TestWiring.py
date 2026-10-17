"""Check every soldered DAC channel (A-D) by moving one axis at a time.

  1. Center all sticks
  2. Bind: throttle UP 0.5s, DOWN 0.5s, center
  3. 10s countdown -- press TAKEOFF on the remote during this
  4. For each channel A-D:  MIN -> center -> MAX -> center
  5. Center all sticks, close

    python3 test_wiring.py                    # 30% moves, 2s each
    python3 test_wiring.py --strength 50 --step 1.5
    python3 test_wiring.py --only C
    python3 test_wiring.py --dry-run          # no hardware
"""

import argparse
import time
import sys

from hardwaretodrone import Drone
from hardwaretodrone.axes import axis_to_volts

CENTER_PAUSE = 0.5     # seconds at center between every motion
TAKEOFF_WAIT = 10      # seconds after binding to press takeoff


def hold(drone, seconds, **axes):
    """Hold exact stick positions; any axis not given is centered."""
    pos = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0, "throttle": 0.0, **axes}
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        drone.set(**pos)               # keep re-sending so the watchdog stays happy
        time.sleep(0.05)


def center(drone, seconds):
    hold(drone, seconds)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--strength", type=int, default=30, help="1-100 (default 30)")
    p.add_argument("--step", type=float, default=2.0, help="seconds per move (default 2)")
    p.add_argument("--only", choices=list("ABCD"), help="test a single channel")
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()
    if not 1 <= args.strength <= 100:
        p.error("--strength must be 1-100")
    amount = args.strength / 100.0

    print("Connecting to DAC...")
    # slew=1000: sticks jump straight to each position (needed for the 0.5s bind)
    drone = Drone(dry_run=args.dry_run, slew=1000)
    axes = sorted(drone.cfg.axes.values(), key=lambda c: c.channel)
    if args.only:
        axes = [c for c in axes if "ABCD"[c.channel] == args.only]

    try:
        print("Centering all sticks...")
        center(drone, 1.0)

        print("Binding: throttle UP 0.5s")
        hold(drone, 5, throttle=1.0)
        print("Binding: throttle DOWN 0.5s")
        hold(drone, 5, throttle=-1.0)
        print("Centering...")
        center(drone, CENTER_PAUSE)
        drone.close()
        sys.exit(1)
        print(f"Press TAKEOFF on the remote now. Starting in {TAKEOFF_WAIT}s:")
        for s in range(TAKEOFF_WAIT, 0, -1):
            print(f"  {s}...", flush=True)
            center(drone, 1.0)

        for cal in axes:
            ch = "ABCD"[cal.channel]
            print(f"\n=== Channel {ch} -> {cal.name.upper()} ({args.strength}%) ===")
            for label, value in (("MIN", -amount), ("MAX", +amount)):
                print(f"  {cal.name} {label}  ({axis_to_volts(cal, value):.2f}V) for {args.step:g}s")
                hold(drone, args.step, **{cal.name: value})
                print(f"  centering {CENTER_PAUSE:g}s")
                center(drone, CENTER_PAUSE)

        print("\nAll channels tested. Hovering 2s...")
        center(drone, 2.0)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        drone.close()
        print("All sticks centered, closed. Land with the remote button.")


if __name__ == "__main__":
    main()