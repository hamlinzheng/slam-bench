#!/usr/bin/env python3
"""Absolute position error of a trajectory against the GNSS reference track.

Split out from aggregate.py, which is stdlib-only so its tests run on the host with
plain pytest. This needs numpy for the alignment, so it stays a separate module that
aggregate.py imports if it can and reports nulls if it cannot — the same rule the rest
of metrics.json follows: anything not measured is null, never 0.

Position only. A single-antenna GNSS track has no attitude, so the reference's
quaternion is a dummy and any rotation metric computed against it would be meaningless
(see gnss_ref.py). Scale is never corrected either: LiDAR range and IMU acceleration are
both metric, so a LIO trajectory's scale is observable and fitting it would hide error.

Reported split into horizontal and vertical for the same reason aggregate.py splits
end_pos: on a flat outdoor route the 3D scalar is dominated by the vertical, where every
system is weakest, and ranks runs by that axis alone.
"""
import math

import numpy as np


def load_tum(path):
    a = np.loadtxt(str(path))
    if a.ndim == 1:
        a = a.reshape(1, -1)
    return a[:, 0], a[:, 1:4]


def umeyama(src, dst):
    """R, t minimising |R*src + t - dst|, without scale."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    cov = (dst - mu_d).T @ (src - mu_s) / len(src)
    u, _, vt = np.linalg.svd(cov)
    d = np.eye(3)
    # Guard against a reflection: a planar point set leaves the third singular vector
    # free, and without this the fit can silently mirror the trajectory.
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        d[2, 2] = -1
    r = u @ d @ vt
    return r, mu_d - r @ mu_s


def associate(ts, ps, ref_t, ref_p, max_dt):
    """Nearest-stamp pairing, dropping anything further apart than max_dt."""
    if len(ref_t) < 2 or len(ts) == 0:
        return ps[:0], ref_p[:0]
    idx = np.clip(np.searchsorted(ref_t, ts), 1, len(ref_t) - 1)
    pick = np.where(np.abs(ref_t[idx] - ts) < np.abs(ref_t[idx - 1] - ts), idx, idx - 1)
    ok = np.abs(ref_t[pick] - ts) <= max_dt
    return ps[ok], ref_p[pick[ok]]


# Wider than evo's 0.01 s default: one baseline stamps its odometry at the publish
# instant rather than the sweep time, which at 5x playback lands ~0.15 s off.
DEFAULT_T_MAX_DIFF = 0.05

# Below this many pairs the alignment is not constrained and the numbers would be noise.
MIN_PAIRS = 50


def ape(ref_tum, traj_tum, t_max_diff=DEFAULT_T_MAX_DIFF):
    """Aligned position error against the reference, or {} when it cannot be measured."""
    ref_t, ref_p = load_tum(ref_tum)
    ts, ps = load_tum(traj_tum)
    src, dst = associate(ts, ps, ref_t, ref_p, t_max_diff)
    if len(src) < MIN_PAIRS:
        return {}
    r, t = umeyama(src, dst)
    err = (r @ src.T).T + t - dst
    horiz = np.linalg.norm(err[:, :2], axis=1)
    vert = np.abs(err[:, 2])
    full = np.linalg.norm(err, axis=1)
    rms = lambda v: float(math.sqrt(float((v ** 2).mean())))  # noqa: E731
    return {
        "gnss_ape_rmse_m": rms(full),
        "gnss_ape_horiz_rmse_m": rms(horiz),
        "gnss_ape_vert_rmse_m": rms(vert),
        "gnss_ape_max_m": float(full.max()),
        "gnss_ape_pairs": int(len(src)),
        # What M-DRIFT reaches for, measured rather than inferred: end_pos_m is a
        # magnitude and a route's physical closure is a vector, so subtracting one from
        # the other gives only a lower bound. Against a reference both are vectors and no
        # closure constant is needed. Two poses rather than thousands, so noisier than the
        # RMS — read beside it, not instead of it.
        "gnss_end_err_horiz_m": float(horiz[-1]),
        "gnss_end_err_vert_m": float(vert[-1]),
    }


# Every key ape() can produce, all absent — so a run that could not be measured against a
# reference reads as null rather than as zero error.
NO_APE = {
    k: None
    for k in (
        "gnss_ape_rmse_m",
        "gnss_ape_horiz_rmse_m",
        "gnss_ape_vert_rmse_m",
        "gnss_ape_max_m",
        "gnss_ape_pairs",
        "gnss_end_err_horiz_m",
        "gnss_end_err_vert_m",
    )
}
