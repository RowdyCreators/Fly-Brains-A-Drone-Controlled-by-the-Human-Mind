"""Recognise the user's input patterns in band-power frames.

1. z-score each frame against the user's resting baseline.
2. Classifier: nearest class mean with a shared per-feature spread
   (a diagonal LDA / Gaussian naive Bayes). Classes are "rest" plus every
   pattern recorded in the assignment phase. Frames that look like nothing
   seen in training (too far from every class) are rejected as "rest",
   so a random artifact never turns into a command.
3. EventDetector: debounces the per-frame labels into start/end events with
   durations, so the mode controller can tell taps from holds.

numpy only (no sklearn), so it runs on the Pi with what's already installed.
"""

from dataclasses import dataclass
from typing import List, Optional

import numpy as np

REST = "rest"


def zscore(features, mean, std):
    return (np.asarray(features) - np.asarray(mean)) / np.asarray(std)


class Classifier:
    def __init__(self, labels, means, spread, reject_d2, min_conf=0.7):
        self.labels = list(labels)
        self.means = np.asarray(means, dtype=float)       # (classes, features)
        self.spread = np.asarray(spread, dtype=float)     # (features,)
        self.reject_d2 = np.asarray(reject_d2, dtype=float)  # (classes,)
        self.min_conf = float(min_conf)

    @classmethod
    def fit(cls, X, y, min_conf=0.7, spread_floor=0.1, reject_pct=99, reject_margin=1.5):
        X, y = np.asarray(X, dtype=float), np.asarray(y)
        labels = [REST] + sorted(set(y) - {REST})
        means = np.array([X[y == c].mean(axis=0) for c in labels])
        resid = np.concatenate([X[y == c] - means[i] for i, c in enumerate(labels)])
        spread = np.maximum(resid.std(axis=0), spread_floor)
        d2 = (((X[:, None, :] - means[None]) / spread) ** 2).sum(axis=-1)
        reject = [np.percentile(d2[y == c, i], reject_pct) * reject_margin
                  for i, c in enumerate(labels)]
        return cls(labels, means, spread, reject, min_conf)

    def scores(self, z):
        """Squared distance from z to every class mean (in spread units)."""
        z = np.atleast_2d(z)
        return (((z[:, None, :] - self.means[None]) / self.spread) ** 2).sum(axis=-1)

    def predict(self, z):
        """Returns (label, confidence) for one z-scored frame."""
        d2 = self.scores(z)[0]
        logp = -0.5 * d2
        p = np.exp(logp - logp.max())
        p /= p.sum()
        best = int(np.argmax(p))
        label, conf = self.labels[best], float(p[best])
        if label != REST and (conf < self.min_conf or d2[best] > self.reject_d2[best]):
            return REST, conf
        return label, conf

    def to_dict(self):
        return {"labels": self.labels, "means": self.means.tolist(),
                "spread": self.spread.tolist(), "reject_d2": self.reject_d2.tolist(),
                "min_conf": self.min_conf}

    @classmethod
    def from_dict(cls, d):
        return cls(d["labels"], d["means"], d["spread"], d["reject_d2"], d.get("min_conf", 0.7))


def cross_validate(X, y, groups, **fit_kw):
    """Leave-one-trial-out accuracy per class (fraction of frames labelled right)."""
    X, y, groups = np.asarray(X), np.asarray(y), np.asarray(groups)
    correct = {c: 0 for c in set(y)}
    total = {c: 0 for c in set(y)}
    for g in np.unique(groups):
        test = groups == g
        if test.all():
            continue
        clf = Classifier.fit(X[~test], y[~test], **fit_kw)
        for z, truth in zip(X[test], y[test]):
            total[truth] += 1
            correct[truth] += clf.predict(z)[0] == truth
    return {c: correct[c] / total[c] for c in total if total[c]}


@dataclass
class Event:
    kind: str        # "start" or "end"
    label: str
    t: float
    duration: float = 0.0


class EventDetector:
    """A pattern starts after `on_frames` frames in a row and ends after
    `off_frames` frames without it. Durations are measured start-to-end."""

    def __init__(self, on_frames=2, off_frames=2):
        self.on_frames = on_frames
        self.off_frames = off_frames
        self.active: Optional[str] = None
        self.since = 0.0
        self._cand, self._cand_n, self._cand_t = None, 0, 0.0
        self._miss = 0

    def duration(self, t):
        return t - self.since if self.active else 0.0

    def update(self, label, t) -> List[Event]:
        events = []
        if self.active is not None:
            if label == self.active:
                self._miss = 0
                return events
            self._miss += 1
            if self._miss < self.off_frames:
                return events
            events.append(Event("end", self.active, t, t - self.since))
            self.active, self._miss = None, 0
            self._cand, self._cand_n = None, 0

        if label == REST:
            self._cand, self._cand_n = None, 0
            return events
        if label != self._cand:
            self._cand, self._cand_n, self._cand_t = label, 0, t
        self._cand_n += 1
        if self._cand_n >= self.on_frames:
            self.active, self.since = label, self._cand_t
            self._cand, self._cand_n, self._miss = None, 0, 0
            events.append(Event("start", label, t))
        return events
