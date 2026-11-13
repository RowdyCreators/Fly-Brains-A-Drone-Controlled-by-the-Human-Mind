"""Phase 2: record the user's input patterns and assign them to roles.

The user picks an action for each role (e.g. primary = jaw clench, lock = eyes
closed). We cue rest / action repeatedly, in shuffled order, and learn what
each action looks like in alpha/beta/gamma relative to the baseline. Then we
cross-validate (leave one trial out) and report how reliably each action and
rest are recognised, so unreliable actions can be swapped before flying.
"""

import random

import numpy as np

from . import stream as st
from .classifier import REST, Classifier, cross_validate, zscore
from .userprofile import ROLES

MIN_ACCURACY = 0.8
REACTION_SEC = 0.5   # time to react to a cue, on top of the window lag


def record_patterns(frames, profile, roles, cue=print, *, trials=8,
                    rest_sec=4.0, action_sec=3.0, seed=None):
    """roles: {role: action label}, must include 'primary'. Fits the classifier
    into the profile and returns per-class cross-validated accuracy."""
    if not profile.has_baseline:
        raise RuntimeError("profile has no baseline; run `train` first")
    bad = set(roles) - set(ROLES)
    if bad or "primary" not in roles:
        raise ValueError(f"roles must include 'primary' and only use {ROLES}; got {list(roles)}")
    labels = list(roles.values())
    if len(set(labels)) != len(labels) or REST in labels:
        raise ValueError("each role needs its own distinct action (and not 'rest')")

    skip = profile.window_sec + REACTION_SEC
    rng = random.Random(seed)
    X, y, groups = [], [], []

    def take(label, seconds, trial):
        feats = st.collect(frames, seconds, skip_sec=skip)
        X.extend(zscore(feats, profile.baseline_mean, profile.baseline_std))
        y.extend([label] * len(feats))
        groups.extend([trial] * len(feats))

    for trial in range(trials):
        order = labels[:]
        rng.shuffle(order)
        for label in order:
            cue(REST, f"[{trial + 1}/{trials}] relax")
            take(REST, rest_sec, trial)
            cue(label, f"[{trial + 1}/{trials}] {label.upper()} now - keep it going")
            take(label, action_sec, trial)
    cue(REST, "done, relax")

    X, y = np.array(X), np.array(y)
    clf = Classifier.fit(X, y)
    acc = cross_validate(X, y, groups)
    profile.classifier = clf.to_dict()
    profile.accuracy = {k: round(float(v), 3) for k, v in acc.items()}
    profile.roles = dict(roles)
    return profile.accuracy


def report(profile):
    lines = ["recognition accuracy (leave-one-trial-out):"]
    role_of = {v: k for k, v in profile.roles.items()}
    weak = []
    for label, a in sorted(profile.accuracy.items(), key=lambda kv: kv[0] != REST):
        tag = f"({role_of[label]})" if label in role_of else ""
        flag = "" if a >= MIN_ACCURACY else "  <-- unreliable"
        lines.append(f"  {label:<14}{tag:<10} {a:6.1%}{flag}")
        if a < MIN_ACCURACY:
            weak.append(label)
    if REST in weak:
        lines.append("WARNING: rest is often mistaken for an action -> the drone may move "
                     "on its own. Re-record, or pick more distinct actions.")
    elif weak:
        lines.append(f"WARNING: {', '.join(weak)} not recognised reliably; consider a "
                     "different action for that role.")
    return "\n".join(lines)
