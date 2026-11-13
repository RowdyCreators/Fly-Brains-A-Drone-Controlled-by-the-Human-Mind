"""Command line for the three phases.

    python -m processing train  --profile NAME [--sim] [--seconds 60]
    python -m processing assign --profile NAME [--sim] [--primary clench --prev focus --lock eyes_closed]
    python -m processing fly    --profile NAME [--sim | --dry-run]

--sim      simulated EEG (and a fake DAC). In train/assign the simulator
           follows the cues automatically; in fly you drive it with keys.
--dry-run  real EEG, fake DAC (bench test on the Pi without moving the drone).
"""

import argparse
import itertools
import sys

from . import assignment, sim, training
from . import stream as st
from .userprofile import Profile

SUGGESTED = "jaw clench, focus (mental math), eyes closed, tongue press, eyebrow raise"


def make_source(args):
    if args.sim:
        return sim.SimBrain()
    from signals.eeg_pipeline import ADSSource
    return ADSSource()


def make_cue(brain):
    """Prints the prompt; with the simulator, also acts it out."""
    def cue(state, msg):
        if brain is not None:
            brain.state = state if state in sim.STATES else "rest"
        print(f"\a>> {msg}", flush=True)
    return cue


def load_existing(name):
    if not Profile.path_for(name).exists():
        sys.exit(f"no profile {name!r} at {Profile.path_for(name)}; run `train` first")
    return Profile.load(name)


def load_or_new(name):
    path = Profile.path_for(name)
    return Profile.load(name) if path.exists() else Profile(name)


def cmd_train(args):
    source = make_source(args)
    profile = load_or_new(args.profile)
    with st.LiveStream(source) as stream:
        print(f"warming up filters ({st.SETTLE_SEC:.0f} s)...", flush=True)
        training.record_baseline(stream.frames(), profile, args.seconds,
                                 make_cue(source if args.sim else None))
    print(training.describe_baseline(profile))
    print(f"saved {profile.save()}")


def ask_roles(args):
    roles = {r: getattr(args, r) for r in ("primary", "prev", "lock") if getattr(args, r)}
    if "primary" not in roles:
        print(f"Pick a distinct action for each role (ideas: {SUGGESTED}).")
        if args.sim:
            print(f"With --sim the actions must be one of: {', '.join(sim.ACTIONS)}")
        roles["primary"] = input("primary (tap = next mode, hold = go): ").strip()
        for role, desc in (("prev", "tap = previous mode"), ("lock", "hold = lock/unlock")):
            v = input(f"{role} ({desc}, Enter to skip): ").strip()
            if v:
                roles[role] = v
    roles = {r: v.lower().replace(" ", "_") for r, v in roles.items()}
    if args.sim:
        bad = [v for v in roles.values() if v not in sim.ACTIONS]
        if bad:
            sys.exit(f"--sim can only act out {sim.ACTIONS}; got {bad}")
    return roles


def cmd_assign(args):
    profile = load_existing(args.profile)
    roles = ask_roles(args)
    source = make_source(args)
    profile.check_stream(st.FS, st.CHANNELS, st.BANDS, st.WINDOW_SEC, st.UPDATE_SEC)
    print(f"{args.trials} trials x {len(roles)} actions: hold each action for the whole "
          f"{args.action_sec:.0f} s cue, relax during rest.")
    with st.LiveStream(source) as stream:
        assignment.record_patterns(stream.frames(), profile, roles,
                                   make_cue(source if args.sim else None),
                                   trials=args.trials, rest_sec=args.rest_sec,
                                   action_sec=args.action_sec)
    print(assignment.report(profile))
    print(f"saved {profile.save()}")


def cmd_fly(args):
    from hardwaretodrone import Drone
    from .control import Pilot, fly

    profile = load_existing(args.profile)
    profile.check_stream(st.FS, st.CHANNELS, st.BANDS, st.WINDOW_SEC, st.UPDATE_SEC)
    source = make_source(args)
    dry = args.sim or args.dry_run

    print("roles:", ", ".join(f"{r}={a}" for r, a in profile.roles.items()))
    print("modes:", " -> ".join(profile.modes))
    if not dry:
        print("REAL DRONE. Take off with the remote's button. Ctrl+C = hover + stop.")

    def show(msg):
        print(f"\r\033[K\a** {msg}", flush=True)

    def status(line):
        print(f"\r\033[K{line}", end="", flush=True)

    with Drone(dry_run=dry) as drone, st.LiveStream(source) as stream:
        pilot = Pilot(profile, drone)
        frames = stream.frames()
        if args.sim and sys.stdin.isatty():
            from .keys import HELP, SimKeyboard
            print(HELP)
            with SimKeyboard(source) as kb:
                fly(itertools.takewhile(lambda _: not kb.quit.is_set(), frames),
                    pilot, show, status)
        else:
            fly(frames, pilot, show, status)
    print("\nstopped, sticks centered")


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m processing", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="phase", required=True)

    def common(p):
        p.add_argument("--profile", required=True, help="profile name (profiles/NAME.json)")
        p.add_argument("--sim", action="store_true", help="simulated EEG + fake DAC")
        return p

    p = common(sub.add_parser("train", help="phase 1: record resting baseline"))
    p.add_argument("--seconds", type=float, default=60.0)
    p.set_defaults(func=cmd_train)

    p = common(sub.add_parser("assign", help="phase 2: record input patterns, assign roles"))
    p.add_argument("--primary", help="action for primary (tap = next mode, hold = go)")
    p.add_argument("--prev", help="action for prev (tap = previous mode)")
    p.add_argument("--lock", help="action for lock (hold = lock/unlock)")
    p.add_argument("--trials", type=int, default=8)
    p.add_argument("--rest-sec", type=float, default=4.0)
    p.add_argument("--action-sec", type=float, default=3.0)
    p.set_defaults(func=cmd_assign)

    p = common(sub.add_parser("fly", help="phase 3: fly with the profile"))
    p.add_argument("--dry-run", action="store_true", help="real EEG but fake DAC")
    p.set_defaults(func=cmd_fly)

    args = ap.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
