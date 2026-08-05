#!/usr/bin/env python3
"""Whether a trajectory blew up, and when.

One rule, used in two places: `Detector` is incremental, so record_tum.py can feed it a
pose at a time while a run is still playing, and aggregate.py can feed it a finished
trajectory.tum through `scan`. Two implementations would be two definitions of "diverged",
and the status a run carries would depend on which one happened to see it.

**This detects a blow-up, not drift.** A run that stays physically plausible throughout
and simply ends 300 m off is not diverged here — that is accuracy, and gnss_ape_* owns it.
Where a dataset has no usable GNSS reference there is no drift measure at all, for the
same reason configs/bags.yaml carries no `drift_zero`: a magnitude cannot answer a
question about a vector.

Stdlib only, like aggregate.py and for the same reason — its tests run on the host under
plain pytest, with no container and no ROS.
"""
import math

# Measured over the 112 trajectories in results/: healthy runs top out at 24.6 m/s and
# diverged ones start at 69.0 m/s, with nothing observed in between. 40 m/s (144 km/h)
# sits in the middle of that gap and means something physically — it bounds the *vehicle*,
# not the route, which is what makes one number right for a 1.8 km underground drift and a
# 7.3 km road alike. That is the difference from aggregate's --split-at, whose boundary is
# a dataset-dependent distance and so ships without a default.
DEFAULT_MAX_SPEED_MPS = 40.0

# Sustained, not single-step: online this rule kills a run, and one spurious pose must not
# be able to do that. The robustness turned out to be free — every one of the 33 diverged
# runs has a window that is 100% over the limit, and no healthy run has a single step over
# it, so the fraction could be 0.1 or 0.9 without changing a verdict.
WINDOW_S = 2.0
MIN_FRACTION = 0.5
# Below this many steps the window is the opening of the run rather than a window.
MIN_STEPS = 5


class Detector:
    """Latching verdict over a stream of poses.

    push() returns the reason the first time the run is judged diverged and None every
    other time, so a caller can act on the transition without tracking it itself.
    """

    def __init__(
        self,
        max_speed_mps=DEFAULT_MAX_SPEED_MPS,
        window_s=WINDOW_S,
        min_fraction=MIN_FRACTION,
        min_steps=MIN_STEPS,
    ):
        self.max_speed_mps = max_speed_mps
        self.window_s = window_s
        self.min_fraction = min_fraction
        self.min_steps = min_steps

        self.verdict = None
        self.diverged_at_s = None
        self.v_max_mps = None
        self.nonfinite_poses = 0

        self._t0 = None
        self._prev = None      # (t, x, y, z) of the last usable pose
        self._window = []      # [(t, over)] within window_s of the newest step

    def push(self, t, x, y, z):
        """Feed one pose. Returns the reason on the transition to diverged, else None."""
        if self._t0 is None:
            self._t0 = t

        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            # A non-finite pose has no position to measure a step against, so it cannot
            # join _prev — but it is itself conclusive, so it counts as an over-limit step.
            self.nonfinite_poses += 1
            return self._observe(t, True)

        prev, self._prev = self._prev, (t, x, y, z)
        if prev is None:
            return None
        dt = t - prev[0]
        if dt <= 0:
            # Odometry restamped or replayed out of order. No speed is defined; skipping
            # the step is the only reading that does not invent one.
            return None

        speed = math.dist(prev[1:], (x, y, z)) / dt
        if self.v_max_mps is None or speed > self.v_max_mps:
            self.v_max_mps = speed
        return self._observe(t, speed > self.max_speed_mps)

    def _observe(self, t, over):
        self._window.append((t, over))
        cutoff = t - self.window_s
        # Linear scan rather than a deque: the window holds ~20 entries at 10 Hz.
        drop = 0
        while drop < len(self._window) and self._window[drop][0] < cutoff:
            drop += 1
        if drop:
            del self._window[:drop]

        if self.verdict is not None:
            return None
        n = len(self._window)
        if n < self.min_steps:
            return None
        frac = sum(1 for _, o in self._window if o) / n
        if frac < self.min_fraction:
            return None

        self.diverged_at_s = t - self._t0
        self.verdict = (
            "{:.0%} of a {:g}s window over {:g} m/s at {:.1f}s into the trajectory"
            .format(frac, self.window_s, self.max_speed_mps, self.diverged_at_s)
        )
        if self.nonfinite_poses:
            self.verdict += " ({} non-finite pose(s))".format(self.nonfinite_poses)
        return self.verdict


def read_tum(path):
    """[(t, x, y, z)] from a TUM file, skipping lines that are not poses.

    Deliberately tolerant: a trajectory truncated mid-write by a killed run ends in a
    partial line, and that run's verdict is exactly what we are here for.
    """
    poses = []
    with open(path) as fh:
        for line in fh:
            fields = line.split()
            if len(fields) < 4:
                continue
            try:
                poses.append(tuple(float(f) for f in fields[:4]))
            except ValueError:
                continue
    return poses


def scan(path, max_speed_mps=DEFAULT_MAX_SPEED_MPS, **kw):
    """The same verdict, over a finished trajectory file.

    Keys read null rather than 0 when nothing could be measured — a trajectory too short
    to contain a step has no maximum speed, and 0.0 would claim a stationary vehicle.
    """
    det = Detector(max_speed_mps=max_speed_mps, **kw)
    for t, x, y, z in read_tum(path):
        det.push(t, x, y, z)
    return {
        "v_max_mps": det.v_max_mps,
        "diverged_at_s": det.diverged_at_s,
        "diverged_reason": det.verdict,
        "nonfinite_poses": det.nonfinite_poses,
    }
